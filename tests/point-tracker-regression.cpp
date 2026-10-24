// Camera-free regression of the production PointTracker implementation.
// No copied POSIT implementation, camera access, profile writes, or game output.
#include "tracker-pt/point_tracker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>

namespace pt_impl {
struct PointTrackerTestAccess
{
    static void frame(PointTracker& tracker, const std::vector<vec2>& points,
                      const PointModel& model, const pt_camera_info& camera,
                      bool continuity, double elapsed_ms = 1000./60, int timeout_ms = 250)
    {
        tracker.track_impl(points, model, camera, timeout_ms, nullptr, 0,
                           continuity, elapsed_ms);
    }
    static bool measure(PointTracker& tracker, const PointModel& model,
                        const std::array<vec2, 3>& points, f focal, const Affine& pose)
    {
        return tracker.measure_pose(model, points, focal, 640, pose);
    }
    static void seed_known_pose(PointTracker& tracker, const Affine& pose)
    {
        tracker.X_CM = pose;
        tracker.X_CM_expected = pose;
        tracker.init_phase = false;
        tracker.t.start();
    }
};
}

using pt_impl::PointTrackerTestAccess;
using pt_impl::PoseDiagnostics;
using namespace numeric_types;

namespace {
constexpr double pi_d = 3.14159265358979323846;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

const PointModel model({-59, -166, -55}, {81, -166, -55});

pt_camera_info camera_info()
{
    pt_camera_info camera;
    camera.res_x = 640;
    camera.res_y = 480;
    camera.fov = 75;
    camera.fps = 60;
    return camera;
}

cv::Matx33d rotation(double yaw, double pitch, double roll = 0)
{
    // Matches Tracker_PT::data(): yaw about camera Y, pitch about camera X.
    const double y = yaw*pi_d/180, p = pitch*pi_d/180, r = roll*pi_d/180;
    const cv::Matx33d ry(std::cos(y), 0, std::sin(y), 0, 1, 0, -std::sin(y), 0, std::cos(y));
    const cv::Matx33d rx(1, 0, 0, 0, std::cos(p), -std::sin(p), 0, std::sin(p), std::cos(p));
    const cv::Matx33d rz(std::cos(r), -std::sin(r), 0, std::sin(r), std::cos(r), 0, 0, 0, 1);
    return ry * rx * rz;
}

double rotation_error(const mat33& actual, const cv::Matx33d& expected)
{
    const cv::Matx33d delta = expected.t() * cv::Matx33d(actual);
    const double cosine = std::clamp((delta(0,0)+delta(1,1)+delta(2,2)-1)/2, -1., 1.);
    return std::acos(cosine) * 180 / pi_d;
}

std::vector<vec2> project(const cv::Matx33d& rotation, const cv::Vec3d& translation,
                          const pt_camera_info& camera)
{
    const double focal = pt_camera_info::get_focal_length(camera.fov, camera.res_x, camera.res_y);
    const cv::Vec3d vertices[] = {{0, 0, 0}, cv::Vec3d(model.M01), cv::Vec3d(model.M02)};
    std::vector<vec2> points;
    for (const cv::Vec3d& vertex : vertices)
    {
        const cv::Vec3d p = rotation * vertex + translation;
        require(p[2] > 0, "synthetic point must be in front of the camera");
        points.emplace_back((f)(focal*p[0]/p[2]), (f)(focal*p[1]/p[2]));
    }
    return points;
}

std::vector<std::pair<double, double>> regression_path()
{
    std::vector<std::pair<double, double>> path;
    for (int pitch = 0; pitch >= -10; --pitch)
        path.emplace_back(0, pitch);
    for (int yaw = -1; yaw >= -15; --yaw)
        path.emplace_back(yaw, -10);
    for (int yaw = -14; yaw <= 15; ++yaw)
        path.emplace_back(yaw, -10);
    for (int yaw = 14; yaw >= 0; --yaw)
        path.emplace_back(yaw, -10);
    return path;
}

double run_path(bool continuity, const std::vector<std::pair<double, double>>& path,
                const std::array<int, 3>& permutation, double noise_px = 0,
                bool seed_known_pose = false)
{
    PointTracker tracker;
    const auto camera = camera_info();
    if (seed_known_pose)
        PointTrackerTestAccess::seed_known_pose(tracker,
            Affine(mat33(rotation(path.front().first, path.front().second)), {0, 0, 700}));
    std::mt19937 random(20231024);
    std::normal_distribution<double> noise(0, 1);
    double maximum_error = 0;
    unsigned frame_index = 0;
    unsigned printed_frames = 0;
    std::array<unsigned, 3> solver_frames {};
    for (const auto& [yaw, pitch] : path)
    {
        const cv::Matx33d truth = rotation(yaw, pitch);
        auto points = project(truth, {0, 0, 700}, camera);
        // Initialize cleanly; later frames exercise centroid noise.
        if (frame_index++ > 0 && noise_px > 0)
            for (auto& point : points)
                point += vec2((f)(noise(random) * noise_px / camera.res_x),
                              (f)(noise(random) * noise_px / camera.res_x));
        const std::vector<vec2> shuffled {points[permutation[0]], points[permutation[1]], points[permutation[2]]};
        PointTrackerTestAccess::frame(tracker, shuffled, model, camera, continuity);
        const auto& diagnostics = tracker.diagnostics();
        ++solver_frames[(unsigned)diagnostics.solver];
        const double error = rotation_error(tracker.pose().R, truth);
        // Bound CI output while retaining the initial pose and first major divergence.
        if (printed_frames < 20 &&
            (frame_index <= 5 || !diagnostics.accepted || error > maximum_error + 1))
        {
            ++printed_frames;
            std::cout << "TRACE continuity=" << continuity << " known_seed=" << seed_known_pose
                      << " permutation=" << permutation[0] << permutation[1] << permutation[2]
                      << " frame=" << frame_index-1 << " truth=" << yaw << ',' << pitch
                      << " error_deg=" << error << " accepted=" << diagnostics.accepted
                      << " solver=" << (int)diagnostics.solver << " reset=" << (int)diagnostics.reset
                      << " iterations=" << diagnostics.iterations << " converged=" << diagnostics.converged
                      << " residual_px=" << diagnostics.residual_max_px << " match=";
            for (int match : diagnostics.correspondence) std::cout << match << ',';
            std::cout << " R=";
            for (f value : tracker.pose().R.val) std::cout << value << ',';
            std::cout << " t=";
            for (f value : tracker.pose().t.val) std::cout << value << ',';
            std::cout << " observed=";
            for (const auto& point : shuffled) std::cout << point[0] << ',' << point[1] << ',';
            std::cout << '\n';
        }
        require(diagnostics.accepted, "valid synthetic trajectory rejected at yaw=" + std::to_string(yaw) +
                " pitch=" + std::to_string(pitch));
        if (continuity)
        {
            require(diagnostics.positive_depth, "published pose has invalid depth");
            require(std::isfinite(diagnostics.residual_max_px) && diagnostics.residual_max_px <= 1,
                    "published pose exceeds residual gate");
        }
        maximum_error = std::max(maximum_error, error);
    }
    std::cout << "TRACE summary continuity=" << continuity << " known_seed=" << seed_known_pose
              << " held_frames=" << solver_frames[0] << " posit_frames=" << solver_frames[1]
              << " continuity_frames=" << solver_frames[2] << '\n';
    return maximum_error;
}

void regression_and_permutations()
{
    const auto path = regression_path();
    const double baseline_error = run_path(false, path, {0, 1, 2});
    std::cout << "Original POSIT maximum rotation error: " << baseline_error << " degrees\n";
    require(baseline_error > 10, "the real legacy solver must reproduce the reported branch regression");

    // Diagnostic separation only: this bypasses initialization with ground truth.
    // It cannot replace the end-to-end assertion below or establish startup correctness.
    const double seeded_error = run_path(true, path, {0, 1, 2}, 0, true);
    std::cout << "Known-correct seed diagnostic maximum error: " << seeded_error << " degrees\n";

    std::array<int, 3> permutation {{0, 1, 2}};
    do
    {
        const double candidate_error = run_path(true, path, permutation);
        std::cout << "Continuity permutation " << permutation[0] << permutation[1] << permutation[2]
                  << " maximum error: " << candidate_error << " degrees\n";
        require(candidate_error < 0.25, "continuity candidate failed the exact smooth regression");
    }
    while (std::next_permutation(permutation.begin(), permutation.end()));
    require(seeded_error < 0.25, "known-correct seed failed to retain the exact trajectory");
}

void wider_range_and_noise()
{
    std::vector<std::pair<double, double>> path {{0, 0}};
    for (int yaw = -1; yaw >= -45; --yaw) path.emplace_back(yaw, 0);
    for (int yaw = -44; yaw <= 45; ++yaw) path.emplace_back(yaw, 0);
    for (int yaw = 44; yaw >= 0; --yaw) path.emplace_back(yaw, 0);
    const double exact_error = run_path(true, path, {0, 1, 2});
    const double noisy_error = run_path(true, path, {2, 0, 1}, 0.1);
    std::cout << "Wide yaw range error: " << exact_error << "; 0.1-pixel noise error: " << noisy_error << '\n';
    require(exact_error < 0.25, "wide-range exact yaw regression");
    require(noisy_error < 5, "small centroid noise produces excessive pose error");
}

void disabled_dynamic_ignores_continuity_setting()
{
    const auto camera = camera_info();
    PointTracker unchecked, retained_checked;
    const auto compare_frame = [&](const std::vector<vec2>& points)
    {
        PointTrackerTestAccess::frame(unchecked, points, model, camera, false, 1000./60, 0);
        PointTrackerTestAccess::frame(retained_checked, points, model, camera, true, 1000./60, 0);
        const auto& baseline = unchecked.diagnostics();
        const auto& retained = retained_checked.diagnostics();
        require(baseline.accepted == retained.accepted &&
                baseline.solver == retained.solver && baseline.reset == retained.reset &&
                baseline.iterations == retained.iterations &&
                baseline.correspondence == retained.correspondence,
                "disabled Dynamic pose must ignore the retained experimental setting");
        require(baseline.solver == PoseDiagnostics::Solver::posit,
                "disabled Dynamic pose unexpectedly selected refinement");
        require(cv::norm(unchecked.pose().R-retained_checked.pose().R) == 0 &&
                cv::norm(unchecked.pose().t-retained_checked.pose().t) == 0,
                "retained experimental setting changed the disabled-Dynamic pose");
    };
    for (const auto& [yaw, pitch] : regression_path())
        compare_frame(project(rotation(yaw, pitch), {0, 0, 700}, camera));

    // A deliberate outlier distinguishes legacy acceptance from the experiment's
    // residual gate; good-fit observations alone would miss the stale-checkbox bug.
    compare_frame({{f(-.22), f(-.34)}, {f(-.02), f(.16)}, {f(.29), f(.09)}});
    require(unchecked.diagnostics().accepted &&
            (!unchecked.diagnostics().positive_depth || unchecked.diagnostics().residual_max_px > 1),
            "compatibility fixture must exercise a pose rejected by the experimental gate");
}

void invalid_inputs_and_dropouts()
{
    const auto camera = camera_info();
    const auto points = project(rotation(0, 0), {0, 0, 700}, camera);
    PointTracker tracker;
    PointTrackerTestAccess::frame(tracker, points, model, camera, true);
    require(tracker.diagnostics().accepted, "initialization failed");
    for (double gap_ms : {100., 8500.})
    {
        const Affine saved = tracker.pose();
        PointTrackerTestAccess::frame(tracker, {points[0], points[1]}, model, camera, true, gap_ms);
        require(!tracker.diagnostics().accepted && tracker.diagnostics().solver == PoseDiagnostics::Solver::held,
                "two points must not produce a fresh pose");
        require(cv::norm(tracker.pose().R - saved.R) == 0 && cv::norm(tracker.pose().t - saved.t) == 0,
                "dropout must retain the previous pose");
        PointTrackerTestAccess::frame(tracker, points, model, camera, true, gap_ms);
        require(tracker.diagnostics().accepted, "same-pose reacquisition failed");
        require(tracker.diagnostics().solver == (gap_ms < 250 ? PoseDiagnostics::Solver::continuity : PoseDiagnostics::Solver::posit),
                "timeout did not choose the intended recovery path");
    }
    const auto moved_points = project(rotation(5, 0), {0, 0, 700}, camera);
    PointTrackerTestAccess::frame(tracker, {moved_points[0], moved_points[1]}, model, camera, true, 100);
    require(!tracker.diagnostics().accepted, "moving two-point frame must remain held");
    PointTrackerTestAccess::frame(tracker, moved_points, model, camera, true, 150);
    require(tracker.diagnostics().accepted && tracker.diagnostics().solver == PoseDiagnostics::Solver::continuity &&
            rotation_error(tracker.pose().R, rotation(5, 0)) < 0.25,
            "brief dropout with a small head turn did not recover continuously");
    const Affine after_recovery = tracker.pose();
    auto nonfinite = points;
    nonfinite[1][0] = std::numeric_limits<f>::quiet_NaN();
    PointTrackerTestAccess::frame(tracker, nonfinite, model, camera, true);
    require(!tracker.diagnostics().accepted, "NaN input accepted");
    require(cv::norm(tracker.pose().R-after_recovery.R) == 0 && cv::norm(tracker.pose().t-after_recovery.t) == 0,
            "invalid input changed the pose");
    const PointModel collinear({1, 2, 3}, {2, 4, 6});
    require(!collinear.valid(), "collinear geometry accepted");
    PointTrackerTestAccess::frame(tracker, points, collinear, camera, true);
    require(!tracker.diagnostics().accepted, "invalid model published a pose");
    const std::array<vec2, 3> ordered {{points[0], points[1], points[2]}};
    const f focal = pt_camera_info::get_focal_length(camera.fov, camera.res_x, camera.res_y);
    Affine behind(mat33::eye(), {0, 0, -700});
    require(!PointTrackerTestAccess::measure(tracker, model, ordered, focal, behind), "behind-camera pose accepted");
    Affine badfit = after_recovery;
    badfit.t[0] += 100;
    require(PointTrackerTestAccess::measure(tracker, model, ordered, focal, badfit) &&
            tracker.diagnostics().residual_max_px > 1, "bad-fit residual was not detected");
}
}

int main()
{
    std::cout << std::setprecision(9);
    std::cout << "TRACE solver: 0=held, 1=POSIT, 2=continuity; reset: 0=none, 1=initial, "
                 "2=timeout, 3=dynamic-disabled, 4=duplicate-match, 5=missing-points, "
                 "6=invalid-input, 7=rejected-pose\n";
    require(model.valid(), "measured custom model is invalid");
    regression_and_permutations();
    wider_range_and_noise();
    disabled_dynamic_ignores_continuity_setting();
    invalid_inputs_and_dropouts();
    std::cout << "PointTracker source regressions passed. This does not establish real-camera acceptance.\n";
    return 0;
}
