/* Copyright (c) 2012 Patrick Ruoff
 * Copyright (c) 2014-2016 Stanislaw Halik <sthalik@misaki.pl>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#undef NDEBUG
#include "ftnoir_tracker_pt.h"
#include "pt-api.hpp"
#include "cv/init.hpp"
#include "video/video-widget.hpp"
#include "compat/math-imports.hpp"
#include "compat/check-visible.hpp"
#include "compat/thread-name.hpp"
#include "compat/qt-dpi.hpp"

#include <cassert>
#include <QHBoxLayout>
#include <QDebug>
#include <QFile>
#include <QCoreApplication>
#include <QTextStream>
#include <QDateTime>

using namespace options;

namespace pt_impl {

Tracker_PT::Tracker_PT(pointer<pt_runtime_traits> const& traits) :
    traits { traits },
    s { traits->get_module_name() },
    point_extractor { traits->make_point_extractor() },
    frame { traits->make_frame() }
{
    opencv_init();

    connect(&*s.b, &bundle_::saving, this, [this]{ reopen_camera_flag = true; }, Qt::DirectConnection);
}

Tracker_PT::~Tracker_PT()
{
    requestInterruption();
    wait();

    if (camera)
        camera->stop();
}

bool Tracker_PT::check_camera()
{
    if (reopen_camera_flag)
    {
        reopen_camera_flag = false;

        camera = nullptr;
        camera = traits->make_camera();
        if (!camera || !camera->start(s))
            return false;
    }
    assert(camera);
    if (progn(bool x = true; return open_camera_dialog_flag.compare_exchange_strong(x, false);))
        run_in_thread_sync(qApp->thread(), [this] { camera->show_camera_settings(); });
    return true;
}

void Tracker_PT::run()
{
    portable::set_curthread_name("tracker/pt");

    while(!isInterruptionRequested())
    {
        if (!check_camera())
            break;

        pt_camera_info info;
        bool new_frame = false;

        {
            camera->set_fov(s.fov);
            std::tie(new_frame, info) = camera->get_frame(*frame);
        }

        if (new_frame)
        {
            const bool preview_visible = check_is_visible();

            if (preview_visible && !widget->fresh())
                preview_frame->set_last_frame(*frame);

            point_extractor->extract_points(*frame, *preview_frame, preview_visible && !widget->fresh(), points);
            point_count.store(points.size(), std::memory_order_relaxed);

            Affine X_CM;
            PoseDiagnostics diagnostics;
            const PointModel model(s);

            {
                QMutexLocker l(&center_lock);
                QMutexLocker l2(&data_lock);

                const int dynamic_pose_ms = s.dynamic_pose ? s.init_phase_timeout : 0;
                point_tracker.track(points, model, info, dynamic_pose_ms, filter,
                                    camera->deadzone_amount(), s.continuity_pose_solver);
                diagnostics = point_tracker.diagnostics();
                if (diagnostics.accepted)
                    ever_success.store(true, std::memory_order_relaxed);
                X_CM = point_tracker.pose();
            }
            record_diagnostics(diagnostics, model, info, X_CM);

            if (preview_visible && !widget->fresh())
            {
                const f fx = pt_camera_info::get_focal_length(info.fov, info.res_x, info.res_y);
                Affine X_MH(mat33::eye(), vec3(s.t_MH_x, s.t_MH_y, s.t_MH_z));
                Affine X_GH = X_CM * X_MH;
                vec3 p = X_GH.t; // head (center?) position in global space

                if (p[2] > f(.1))
                    preview_frame->draw_head_center((p[0] * fx) / p[2], (p[1] * fx) / p[2]);

                widget->update_image(preview_frame->get_bitmap());
            }
        }
    }
}

void Tracker_PT::record_diagnostics(const PoseDiagnostics& d, const PointModel& model,
                                    const pt_camera_info& info, const Affine& pose)
{
    const QString requested_path = s.pose_diagnostics_path;
    if (requested_path != diagnostics_path)
    {
        diagnostics_file.close();
        diagnostics_path = requested_path;
        diagnostics_rows = 0;
        diagnostics_timer.start();
        if (!requested_path.isEmpty())
        {
            diagnostics_file.setFileName(requested_path);
            if (!diagnostics_file.open(QIODevice::WriteOnly | QIODevice::NewOnly | QIODevice::Text))
                qWarning() << "pt: diagnostic CSV must be a new writable file:" << diagnostics_file.errorString();
            else
            {
                QTextStream out(&diagnostics_file);
                out << "utc,elapsed_ms,frame,point_count,accepted,solver,reset,age_ms,iterations,converged,branch,branch_distance_1,branch_distance_2,delta,residual_rms_px,residual_max_px,positive_depth,width,height,fov,m01_x,m01_y,m01_z,m02_x,m02_y,m02_z";
                for (int i = 0; i < 3; ++i)
                    out << ",input_" << i << "_x,input_" << i << "_y,match_" << i << ",ordered_" << i << "_x,ordered_" << i << "_y";
                for (int row = 0; row < 3; ++row)
                    for (int col = 0; col < 3; ++col)
                        out << ",r" << row << col;
                out << ",tx_mm,ty_mm,tz_mm\n";
            }
        }
    }
    if (!diagnostics_file.isOpen())
        return;
    // At most five minutes at 60 FPS. Do not silently reopen or overwrite a log.
    if (diagnostics_rows >= 18000)
    {
        diagnostics_file.close();
        return;
    }
    const char* reset_names[] = {"none", "initial", "timeout", "dynamic_disabled", "duplicate_match",
                                 "insufficient_points", "invalid_input", "rejected_pose"};
    const char* solver_names[] = {"held", "posit", "continuity"};
    QTextStream out(&diagnostics_file);
    out.setRealNumberPrecision(12);
    out << QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs) << ','
        << diagnostics_timer.elapsed_ms() << ',' << diagnostics_rows++ << ',' << d.point_count << ','
        << d.accepted << ',' << solver_names[(int)d.solver] << ',' << reset_names[(int)d.reset] << ','
        << d.elapsed_ms << ',' << d.iterations << ',' << d.converged << ',' << d.branch << ','
        << d.branch_distance_1 << ',' << d.branch_distance_2 << ',' << d.convergence_delta << ','
        << d.residual_rms_px << ',' << d.residual_max_px << ',' << d.positive_depth << ','
        << info.res_x << ',' << info.res_y << ',' << info.fov;
    for (const vec3& v : {model.M01, model.M02})
        for (int axis = 0; axis < 3; ++axis)
            out << ',' << v[axis];
    for (unsigned i = 0; i < 3; ++i)
    {
        const double x = i < points.size() ? points[i][0] : std::numeric_limits<double>::quiet_NaN();
        const double y = i < points.size() ? points[i][1] : std::numeric_limits<double>::quiet_NaN();
        out << ',' << x << ',' << y << ',' << d.correspondence[i] << ','
            << d.ordered_points[i][0] << ',' << d.ordered_points[i][1];
    }
    for (f value : pose.R.val)
        out << ',' << value;
    for (f value : pose.t.val)
        out << ',' << value;
    out << '\n';
    out.flush();
}

module_status Tracker_PT::start_tracker(QFrame* video_frame)
{
    {
        auto camera = traits->make_camera();
        if (!camera || !camera->start(s))
            return error(tr("Failed to open camera '%1'").arg(s.camera_name));
    }

    widget = std::make_unique<video_widget>(video_frame);
    layout = std::make_unique<QHBoxLayout>(video_frame);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(&*widget);
    video_frame->setLayout(&*layout);
    //video_widget->resize(video_frame->width(), video_frame->height());
    video_frame->show();

    double dpi = screen_dpi(video_frame);
    preview_frame = traits->make_preview(iround(preview_width * dpi),
                                         iround(preview_height * dpi));

    start(QThread::HighPriority);

    return {};
}

void Tracker_PT::data(double *data)
{
    if (ever_success.load(std::memory_order_relaxed))
    {
        Affine X_CM;
        {
            QMutexLocker l(&data_lock);
            X_CM = point_tracker.pose();
        }

        Affine X_MH(mat33::eye(), vec3(s.t_MH_x, s.t_MH_y, s.t_MH_z));
        Affine X_GH(X_CM * X_MH);

        // translate rotation matrix from opengl (G) to roll-pitch-yaw (E) frame
        // -z -> x, y -> z, x -> -y
        mat33 R_EG(0, 0,-1,
                   -1, 0, 0,
                   0, 1, 0);
        mat33 R(R_EG *  X_GH.R * R_EG.t());

        // get translation(s)
        const vec3& t = X_GH.t;

        // extract rotation angles
        auto r00 = (double)R(0, 0);
        auto r10 = (double)R(1,0), r20 = (double)R(2,0);
        auto r21 = (double)R(2,1), r22 = (double)R(2,2);

        double beta  = atan2(-r20, sqrt(r21*r21 + r22*r22));
        double alpha = atan2(r10, r00);
        double gamma = atan2(r21, r22);

        constexpr double rad2deg = 180/M_PI;

        data[Yaw]   = rad2deg * alpha;
        data[Pitch] = -rad2deg * beta;
        data[Roll]  = rad2deg * gamma;

        // convert to cm
        data[TX] = (double)t[0] / 10;
        data[TY] = (double)t[1] / 10;
        data[TZ] = (double)t[2] / 10;
    }
}

bool Tracker_PT::center()
{
    QMutexLocker l(&center_lock);

    point_tracker.reset_state();
    return false;
}

int Tracker_PT::get_n_points()
{
    return (int)point_count.load(std::memory_order_relaxed);
}

bool Tracker_PT::get_cam_info(pt_camera_info& info)
{
    bool ret = false;

    if (camera)
        std::tie(ret, info) = camera->get_info();
    return ret;
}

Affine Tracker_PT::pose() const
{
    QMutexLocker l(&data_lock);
    return point_tracker.pose();
}

} // ns pt_impl
