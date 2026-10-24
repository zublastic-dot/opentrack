/* Copyright (c) 2012 Patrick Ruoff
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#include "point_tracker.h"
#include "compat/math-imports.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>

#include <QDebug>

namespace pt_impl {

using namespace numeric_types;

static void get_row(const mat33& m, int i, vec3& v)
{
    v[0] = m(i,0);
    v[1] = m(i,1);
    v[2] = m(i,2);
}

static void set_row(mat33& m, int i, const vec3& v)
{
    m(i,0) = v[0];
    m(i,1) = v[1];
    m(i,2) = v[2];
}

PointModel::PointModel(const pt_settings& s)
{
    set_model(s);
    initialize_geometry();
}

PointModel::PointModel(const vec3& m01, const vec3& m02) : M01(m01), M02(m02)
{
    initialize_geometry();
}

void PointModel::initialize_geometry()
{
    // Reject zero/collinear/nonfinite geometry before the reciprocal Gram determinant.
    const cv::Vec3d a(M01), b(M02);
    const double scale = a.dot(a) * b.dot(b);
    const double determinant = a.cross(b).dot(a.cross(b));
    valid_ = std::isfinite(scale) && std::isfinite(determinant) && scale > 0 &&
             determinant > scale * 1e-10;
    if (!valid_)
    {
        u = vec3(0, 0, 0);
        P = mat22::zeros();
        return;
    }
    // calculate u
    u = M01.cross(M02);
    u = cv::normalize(u);

    // calculate projection matrix on M01,M02 plane
    f s11 = M01.dot(M01);
    f s12 = M01.dot(M02);
    f s22 = M02.dot(M02);
    P = 1/(s11*s22-s12*s12) * mat22(s22, -s12, -s12,  s11);
    for (f value : P.val)
        valid_ = valid_ && std::isfinite(value);
}

void PointModel::set_model(const pt_settings& s)
{
    switch (s.active_model_panel)
    {
    default:
        eval_once(qDebug() << "pt: wrong model type selected");
        [[fallthrough]];
    case Clip:
        M01 = vec3(0, s.clip_ty, -s.clip_tz);
        M02 = vec3(0, -s.clip_by, -s.clip_bz);
        break;
    case Cap:
        M01 = vec3(-s.cap_x, -s.cap_y, -s.cap_z);
        M02 = vec3(s.cap_x, -s.cap_y, -s.cap_z);
        break;
    case Custom:
        M01 = vec3(s.m01_x, s.m01_y, s.m01_z);
        M02 = vec3(s.m02_x, s.m02_y, s.m02_z);
        break;
    }
}

void PointModel::get_d_order(const vec2* points, unsigned* d_order, const vec2& d)
{
    constexpr unsigned cnt = PointModel::N_POINTS;
    // fit line to orthographically projected points
    using t = std::pair<f,unsigned>;
    t d_vals[cnt];
    // get sort indices with respect to d scalar product
    for (unsigned i = 0; i < cnt; ++i)
        d_vals[i] = {d.dot(points[i]), i};

    std::sort(std::begin(d_vals), std::end(d_vals),
              [](t a, t b) { return a.first < b.first; });

    for (unsigned i = 0; i < cnt; ++i)
        d_order[i] = d_vals[i].second;
}


PointTracker::PointTracker() = default;

PointTracker::PointOrder PointTracker::find_correspondences_previous(const vec2* points,
                                                                     const PointModel& model,
                                                                     const pt_camera_info& info)
{
    const f fx = pt_camera_info::get_focal_length(info.fov, info.res_x, info.res_y);
    PointTracker::PointOrder p {
        project(vec3(0,0,0), fx),
        project(model.M01, fx),
        project(model.M02, fx)
    };

    constexpr unsigned sz = PointModel::N_POINTS;

    // set correspondences by minimum distance to projected model point
    bool point_taken[sz] {};

    for (unsigned i=0; i < sz; ++i)
    {
        f min_sdist = 0;
        unsigned min_idx = 0;
        // find closest point to projected model point i
        for (unsigned j=0; j < sz; ++j)
        {
            vec2 d = p[i]-points[j];
            f sdist = d.dot(d);
            if (sdist < min_sdist || j == 0)
            {
                min_idx = j;
                min_sdist = sdist;
            }
        }

        // if one point is closest to more than one model point, fallback
        if (point_taken[min_idx])
        {
            reset_state();
            diagnostics_.reset = PoseDiagnostics::Reset::duplicate_match;
            return find_correspondences(points, model);
        }
        point_taken[min_idx] = true;
        p[i] = points[min_idx];
        diagnostics_.correspondence[i] = (int)min_idx;
    }

    return p;
}

void PointTracker::track(const std::vector<vec2>& points,
                         const PointModel& model,
                         const pt_camera_info& info,
                         int init_phase_timeout,
                         point_filter& filter,
                          f deadzone_amount,
                          bool continuity)
{
    track_impl(points, model, info, init_phase_timeout, &filter, deadzone_amount,
               continuity, t.elapsed_ms());
}

void PointTracker::track_impl(const std::vector<vec2>& points,
                             const PointModel& model, const pt_camera_info& info,
                             int init_phase_timeout, point_filter* filter,
                             f deadzone_amount, bool continuity, double elapsed_ms)
{
    diagnostics_ = {};
    diagnostics_.point_count = (unsigned)points.size();
    diagnostics_.elapsed_ms = elapsed_ms;
    if (points.size() < PointModel::N_POINTS)
    {
        diagnostics_.reset = PoseDiagnostics::Reset::insufficient_points;
        return;
    }
    if (!model.valid() || info.res_x <= 0 || info.res_y <= 0 ||
        !std::isfinite(info.fov) || info.fov <= 0 || info.fov >= 180)
    {
        diagnostics_.reset = PoseDiagnostics::Reset::invalid_input;
        return;
    }
    for (unsigned i = 0; i < PointModel::N_POINTS; ++i)
        if (!std::isfinite(points[i][0]) || !std::isfinite(points[i][1]))
        {
            diagnostics_.reset = PoseDiagnostics::Reset::invalid_input;
            return;
        }
    const f fx = pt_camera_info::get_focal_length(info.fov, info.res_x, info.res_y);
    PointOrder order;

    if (init_phase || init_phase_timeout <= 0 || elapsed_ms > init_phase_timeout)
    {
        diagnostics_.reset = init_phase ? PoseDiagnostics::Reset::initial :
                             init_phase_timeout <= 0 ? PoseDiagnostics::Reset::dynamic_disabled :
                                                       PoseDiagnostics::Reset::timeout;
        reset_state();
        order = find_correspondences(points.data(), model);
    }
    else
        order = find_correspondences_previous(points.data(), model, info);

    if (filter)
        order = (*filter)(order, deadzone_amount);
    diagnostics_.ordered_points = order;

    const Affine previous = X_CM;
    const Affine previous_expected = X_CM_expected;
    // A checked but disabled UI option must not change the legacy path when
    // Dynamic pose is off, including its initialization acceptance criteria.
    const bool continuity_enabled = continuity && init_phase_timeout > 0;
    const bool use_continuity = continuity_enabled && !init_phase;
    diagnostics_.solver = use_continuity ? PoseDiagnostics::Solver::continuity : PoseDiagnostics::Solver::posit;
    bool accepted;
    if (use_continuity)
        accepted = refine_pose(model, order, fx, info.res_x);
    else
    {
        const int iterations = POSIT(model, order, fx);
        diagnostics_.iterations = iterations;
        accepted = iterations != -1;
        const bool valid = measure_pose(model, order, fx, info.res_x, X_CM);
        // Preserve original POSIT acceptance with the experiment disabled. For
        // the candidate, a finite iteration-limit result is not a usable-fit guarantee.
        if (continuity_enabled)
            accepted = accepted && valid && diagnostics_.residual_max_px <= 1.0;
    }
    if (accepted)
    {
        diagnostics_.accepted = true;
        init_phase = false;
        t.start();
    }
    else
    {
        X_CM = previous;
        X_CM_expected = previous_expected;
        diagnostics_.reset = PoseDiagnostics::Reset::rejected_pose;
        // Retain the last accepted pose without refreshing its age. A later timeout
        // requests initialization; a failed refinement is not replaced by a jump.
        if (!use_continuity)
            reset_state();
    }
}

bool PointTracker::measure_pose(const PointModel& model, const PointOrder& order,
                                f focal_length, int width, const Affine& pose)
{
    diagnostics_.positive_depth = false;
    for (f value : pose.R.val)
        if (!std::isfinite(value))
            return false;
    for (f value : pose.t.val)
        if (!std::isfinite(value))
            return false;
    const cv::Vec3d object_points[] = {{0, 0, 0}, cv::Vec3d(model.M01), cv::Vec3d(model.M02)};
    const cv::Matx33d rotation(pose.R);
    const cv::Vec3d translation(pose.t);
    double squared_error = 0, maximum = 0;
    for (unsigned i = 0; i < PointModel::N_POINTS; ++i)
    {
        const cv::Vec3d p = rotation * object_points[i] + translation;
        if (!std::isfinite(p[2]) || p[2] <= 1e-6)
            return false;
        const double dx = (focal_length * p[0] / p[2] - order[i][0]) * width;
        const double dy = (focal_length * p[1] / p[2] - order[i][1]) * width;
        const double error = dx*dx + dy*dy;
        if (!std::isfinite(error))
            return false;
        squared_error += error;
        maximum = std::max(maximum, error);
    }
    diagnostics_.positive_depth = true;
    diagnostics_.residual_rms_px = std::sqrt(squared_error / PointModel::N_POINTS);
    diagnostics_.residual_max_px = std::sqrt(maximum);
    return true;
}

bool PointTracker::refine_pose(const PointModel& model, const PointOrder& order,
                               f focal_length, int width)
{
    diagnostics_.iterations = -1;
    try
    {
        const std::vector<cv::Point3d> object_points {
            {0, 0, 0}, {model.M01[0], model.M01[1], model.M01[2]},
            {model.M02[0], model.M02[1], model.M02[2]}};
        const std::vector<cv::Point2d> image_points {
            {order[0][0], order[0][1]}, {order[1][0], order[1][1]}, {order[2][0], order[2][1]}};
        const cv::Matx33d intrinsics(focal_length, 0, 0, 0, focal_length, 0, 0, 0, 1);
        cv::Mat rotation_vector;
        cv::Rodrigues(cv::Mat(cv::Matx33d(X_CM.R)), rotation_vector);
        cv::Mat translation = (cv::Mat_<double>(3, 1) << X_CM.t[0], X_CM.t[1], X_CM.t[2]);
        cv::solvePnPRefineLM(object_points, image_points, intrinsics, cv::noArray(),
                            rotation_vector, translation,
                            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 50, 1e-10));
        cv::Mat rotation;
        cv::Rodrigues(rotation_vector, rotation);
        Affine candidate;
        for (int row = 0; row < 3; ++row)
        {
            candidate.t[row] = (f)translation.at<double>(row, 0);
            for (int col = 0; col < 3; ++col)
                candidate.R(row, col) = (f)rotation.at<double>(row, col);
        }
        if (!measure_pose(model, order, focal_length, width, candidate) ||
            diagnostics_.residual_max_px > 1.0)
            return false;
        X_CM = candidate;
        X_CM_expected = candidate;
        // OpenCV does not expose its iteration count or convergence status. Keep
        // those diagnostics unknown rather than inventing successful convergence.
        return true;
    }
    catch (const cv::Exception&)
    {
        return false;
    }
}

PointTracker::PointOrder PointTracker::find_correspondences(const vec2* points, const PointModel& model)
{
    constexpr unsigned cnt = PointModel::N_POINTS;
    // We do a simple freetrack-like sorting in the init phase...
    unsigned point_d_order[cnt];
    unsigned model_d_order[cnt];
    // calculate d and d_order for simple freetrack-like point correspondence
    vec2 d(model.M01[0]-model.M02[0], model.M01[1]-model.M02[1]);
    // sort points
    model.get_d_order(points, point_d_order, d);
    vec2 pts[cnt] {
        { 0, 0 },
        { model.M01[0], model.M01[1] },
        { model.M02[0], model.M02[1] },
    };
    model.get_d_order(pts, model_d_order, d);

    // set correspondences
    PointOrder p;
    for (unsigned i = 0; i < cnt; ++i)
    {
        p[model_d_order[i]] = points[point_d_order[i]];
        diagnostics_.correspondence[model_d_order[i]] = (int)point_d_order[i];
    }

    return p;
}

#ifdef __clang__
#   pragma clang diagnostic push
#   pragma clang diagnostic ignored "-Wfloat-equal"
#endif

int PointTracker::POSIT(const PointModel& model, const PointOrder& order, f focal_length)
{
    diagnostics_.converged = 0;
    // POSIT algorithm for coplanar points as presented in
    // [Denis Oberkampf, Daniel F. DeMenthon, Larry S. Davis: "Iterative Pose Estimation Using Coplanar Feature Points"]
    // we use the same notation as in the paper here

    // The expected rotation used for resolving the ambiguity in POSIT:
    // In every iteration step the rotation closer to R_expected is taken
    const mat33& R_expected{X_CM_expected.R};

    // initial pose = last (predicted) pose
    vec3 k;
    get_row(R_expected, 2, k);
    f Z0 = X_CM.t[2] < f(1e-4) ? f(1000) : X_CM.t[2];

    f old_epsilon_1 = 0;
    f old_epsilon_2 = 0;
    f epsilon_1, epsilon_2;

    vec3 I0, J0;
    vec2 I0_coeff, J0_coeff;

    vec3 I_1, J_1, I_2, J_2;
    mat33 R_1, R_2;
    mat33* R_current = &R_1;

    constexpr int max_iter = 100;

    int i;
    for (i = 1; i < max_iter; ++i)
    {
        epsilon_1 = k.dot(model.M01)/Z0;
        epsilon_2 = k.dot(model.M02)/Z0;

        // vector of scalar products <I0, M0i> and <J0, M0i>
        vec2 I0_M0i(order[1][0]*(1 + epsilon_1) - order[0][0],
                order[2][0]*(1 + epsilon_2) - order[0][0]);
        vec2 J0_M0i(order[1][1]*(1 + epsilon_1) - order[0][1],
                order[2][1]*(1 + epsilon_2) - order[0][1]);

        // construct projection of I, J onto M0i plane: I0 and J0
        I0_coeff = model.P * I0_M0i;
        J0_coeff = model.P * J0_M0i;
        I0 = I0_coeff[0]*model.M01 + I0_coeff[1]*model.M02;
        J0 = J0_coeff[0]*model.M01 + J0_coeff[1]*model.M02;

        // calculate u component of I, J
        f II0 = I0.dot(I0);
        f IJ0 = I0.dot(J0);
        f JJ0 = J0.dot(J0);
        f rho, theta;
        // CAVEAT don't change to comparison with an epsilon -sh 20160423
        if (JJ0 == II0) {
            rho = sqrt(fabs(2*IJ0));
            theta = -pi/4;
            if (IJ0<0) theta *= -1;
        }
        else {
            rho = sqrt(sqrt( (JJ0-II0)*(JJ0-II0) + 4*IJ0*IJ0 ));
            theta = atan( -2*IJ0 / (JJ0-II0) );
            // avoid branch misprediction
            theta += (JJ0 - II0 < 0) * pi;
            theta *= f(.5);
        }

        // construct the two solutions
        I_1 = I0 + rho*cos(theta)*model.u;
        I_2 = I0 - rho*cos(theta)*model.u;

        J_1 = J0 + rho*sin(theta)*model.u;
        J_2 = J0 - rho*sin(theta)*model.u;

        f norm_const = (f)(1/cv::norm(I_1)); // all have the same norm

        // create rotation matrices
        I_1 *= norm_const; J_1 *= norm_const;
        I_2 *= norm_const; J_2 *= norm_const;

        set_row(R_1, 0, I_1);
        set_row(R_1, 1, J_1);
        set_row(R_1, 2, I_1.cross(J_1));

        set_row(R_2, 0, I_2);
        set_row(R_2, 1, J_2);
        set_row(R_2, 2, I_2.cross(J_2));

        // the single translation solution
        Z0 = norm_const * focal_length;

        // pick the rotation solution closer to the expected one
        // in simple metric d(A,B) = || I - A * B^T ||
        f R_1_deviation = (f)(cv::norm(mat33::eye() - R_expected * R_1.t()));
        f R_2_deviation = (f)(cv::norm(mat33::eye() - R_expected * R_2.t()));

        if (R_1_deviation < R_2_deviation)
        {
            R_current = &R_1;
            diagnostics_.branch = 1;
        }
        else
        {
            R_current = &R_2;
            diagnostics_.branch = 2;
        }
        diagnostics_.branch_distance_1 = R_1_deviation;
        diagnostics_.branch_distance_2 = R_2_deviation;

        get_row(*R_current, 2, k);

        // check for convergence condition
        const f delta = fabs(epsilon_1 - old_epsilon_1) + fabs(epsilon_2 - old_epsilon_2);
        diagnostics_.convergence_delta = delta;

        if (delta < eps)
        {
            diagnostics_.converged = 1;
            break;
        }

        old_epsilon_1 = epsilon_1;
        old_epsilon_2 = epsilon_2;
    }

    const f t[3] = {
        order[0][0] * Z0/focal_length,
        order[0][1] * Z0/focal_length,
        Z0
    };
    const mat33& r = *R_current;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            int ret = std::fpclassify(r(i, j));
            if (ret == FP_NAN || ret == FP_INFINITE)
            {
                qDebug() << "posit nan R";
                return -1;
            }
        }

    for (unsigned i = 0; i < 3; i++) // NOLINT(modernize-loop-convert)
    {
        int ret = std::fpclassify(t[i]);
        if (ret == FP_NAN || ret == FP_INFINITE)
        {
            qDebug() << "posit nan T";
            return -1;
        }
    }

    // apply results
    X_CM.R = r;
    X_CM.t[0] = t[0];
    X_CM.t[1] = t[1];
    X_CM.t[2] = t[2];

    X_CM_expected = X_CM;

    //qDebug() << "iter:" << i;

    return i;
}

#ifdef __clang__
#   pragma clang diagnostic pop
#endif

vec2 PointTracker::project(const vec3& v_M, f focal_length)
{
    return project(v_M, focal_length, X_CM);
}

vec2 PointTracker::project(const vec3& v_M, f focal_length, const Affine& X_CM)
{
    vec3 v_C = X_CM * v_M;
    return vec2(focal_length*v_C[0]/v_C[2], focal_length*v_C[1]/v_C[2]);
}

void PointTracker::reset_state()
{
    init_phase = true;
    X_CM_expected = {};
}

} // ns pt_impl
