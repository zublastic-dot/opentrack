/* Copyright (c) 2012 Patrick Ruoff
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#pragma once

#include "compat/timer.hpp"
#include "cv/affine.hpp"
#include "cv/numeric.hpp"
#include "pt-api.hpp"
#include "point-filter.hpp"

#include <vector>
#include <array>
#include <limits>

#include <QObject>

namespace pt_impl {

// ----------------------------------------------------------------------------
// Describes a 3-point model
// nomenclature as in
// [Denis Oberkampf, Daniel F. DeMenthon, Larry S. Davis: "Iterative Pose Estimation Using Coplanar Feature Points"]

using namespace numeric_types;

struct PointModel final
{
    static constexpr unsigned N_POINTS = 3;

    vec3 M01;      // M01 in model frame
    vec3 M02;      // M02 in model frame

    vec3 u;        // unit vector perpendicular to M01,M02-plane

    mat22 P;

    enum Model { Clip, Cap, Custom };

    explicit PointModel(const pt_settings& s);
    PointModel(const vec3& m01, const vec3& m02);
    void set_model(const pt_settings& s);
    bool valid() const { return valid_; }

    static void get_d_order(const vec2* points, unsigned* d_order, const vec2& d);
private:
    void initialize_geometry();
    bool valid_ = false;
};

// One camera-frame result. Missing or rejected observations never imply a fresh pose.
struct PoseDiagnostics
{
    enum class Reset { none, initial, timeout, dynamic_disabled, duplicate_match,
                       insufficient_points, invalid_input, rejected_pose };
    enum class Solver { held, posit, continuity };
    Reset reset = Reset::none;
    Solver solver = Solver::held;
    unsigned point_count = 0;
    std::array<int, 3> correspondence {{-1, -1, -1}};
    std::array<vec2, 3> ordered_points {};
    int iterations = 0;
    int branch = -1;
    double branch_distance_1 = std::numeric_limits<double>::quiet_NaN();
    double branch_distance_2 = std::numeric_limits<double>::quiet_NaN();
    double convergence_delta = std::numeric_limits<double>::quiet_NaN();
    double residual_rms_px = std::numeric_limits<double>::quiet_NaN();
    double residual_max_px = std::numeric_limits<double>::quiet_NaN();
    double elapsed_ms = 0;
    // -1: unavailable (OpenCV refinement/held frame), 0: no, 1: yes.
    int converged = -1;
    bool accepted = false;
    bool positive_depth = false;
};

struct PointTrackerTestAccess;

// ----------------------------------------------------------------------------
// Tracks a 3-point model
// implementing the POSIT algorithm for coplanar points as presented in
// [Denis Oberkampf, Daniel F. DeMenthon, Larry S. Davis: "Iterative Pose Estimation Using Coplanar Feature Points"]
class PointTracker final
{
public:
    PointTracker();
    // track the pose using the set of normalized point coordinates (x pos in range -0.5:0.5)
    // f : (focal length)/(sensor width)
    // dt : time since last call
    void track(const std::vector<vec2>& projected_points,
               const PointModel& model,
               const pt_camera_info& info,
               int init_phase_timeout,
               point_filter& filter,
               f deadzone_amount,
               bool continuity = false);
    Affine pose() const { return X_CM; }
    const PoseDiagnostics& diagnostics() const { return diagnostics_; }
    vec2 project(const vec3& v_M, f focal_length);
    vec2 project(const vec3& v_M, f focal_length, const Affine& X_CM);
    void reset_state();

private:
    friend struct PointTrackerTestAccess;
    // the points in model order
    using PointOrder = std::array<vec2, 3>;

    PointOrder find_correspondences(const vec2* projected_points, const PointModel &model);
    PointOrder find_correspondences_previous(const vec2* points, const PointModel &model, const pt_camera_info& info);
    void track_impl(const std::vector<vec2>& points, const PointModel& model,
                    const pt_camera_info& info, int timeout_ms, point_filter* filter,
                    f deadzone_amount, bool continuity, double elapsed_ms);
    bool refine_pose(const PointModel& model, const PointOrder& order, f focal_length, int width);
    bool measure_pose(const PointModel& model, const PointOrder& order,
                      f focal_length, int width, const Affine& pose);
    // The POSIT algorithm, returns the number of iterations
    int POSIT(const PointModel& point_model, const PointOrder& order, f focal_length);

    Affine X_CM;  // transform from model to camera
    Affine X_CM_expected;
    Timer t;
    bool init_phase = true;
    PoseDiagnostics diagnostics_;
};

} // ns pt_impl

using PointTracker = pt_impl::PointTracker;
using PointModel = pt_impl::PointModel;
