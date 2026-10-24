# Cap pose comparison build

This experimental fork tests pose continuity for a tall, asymmetric three-LED cap. It is based on opentrack 2023.2.0 (`1ea5bb0aa7f2f271724ded372957b6f3759fa5a8`) to retain the Windows 32-bit DirectShow camera path. It does not establish that a real camera tracking problem has been fixed.

## Scope

The optional solver refines the previous accepted pose against the current three image points using OpenCV's `solvePnPRefineLM`. After initialization or a configured timeout, PointTracker's existing POSIT initializer is still used. Three points can have multiple valid poses: a small reprojection error alone cannot prove the chosen orientation is correct. This change also cannot reconstruct an LED that is hidden or outside the camera's view.

The synthetic regression uses the production C++ PointTracker, the measured triangle `(0,0,0)`, `(-59,-166,-55)`, `(81,-166,-55)` mm, a 75-degree diagonal camera field of view, and 640x480 observations. It compares the legacy path and experimental path, then checks point-order permutations, wider yaw, centroid noise, missing observations, and invalid inputs. Camera tests remain a separate acceptance step.

## Isolated settings

Extract the artifact into its own writable directory. Keep `portable.txt` next to `opentrack.exe`; on Windows this makes opentrack use `globals.ini` and `ini/*.ini` inside that directory instead of the installed application's settings. Do not copy the experimental DLLs over an existing installation.

Copy a known profile into the portable `ini` directory and give it a distinct name. In its `[tracker-pt]` section:

```ini
dynamic-pose-resolution=true
continuity-pose-solver=true
```

The experimental option defaults to false and can also be toggled in PointTracker's camera settings. Leave point geometry and camera settings unchanged for the first A/B comparison. Start while facing forward with all three LEDs visible. Use opentrack's preview for initial tests with the game closed.

For numeric diagnostics, set `pose-diagnostics-path` in the same section to an absolute, new CSV filename. An empty path disables logging. The logger refuses to overwrite an existing file and stops after 18,000 camera-frame rows. Logs include extracted coordinates, point correspondence, accepted/held status, pose, and available solver diagnostics; they contain no camera images. Logging is intended for short diagnosis, not normal play.

Returning to the installed executable restores the stock solver. Profiles in the installed application's Documents directory are not edited by a correctly isolated portable build.

## Build

The `Cap pose Windows comparison` workflow builds Windows x86 with Qt 5.15.2, OpenCV 4.6.0, and Visual Studio 2022. Its artifact includes `SOURCE-COMMIT.txt` identifying the exact commit. Only the `opentrack-cap-x86-*` artifact has passed the regression; `cap-comparison-unaccepted-*` preserves a failed run for diagnosis and must not be treated as a working fix. The bundle selects PointTracker, DirectShow/OpenCV video, Accela, and the existing FreeTrack output; it installs no camera driver and changes no game settings.

Configure with `OPENTRACK_CAP_ONLY=ON` and `OPENTRACK_CAP_TESTS=ON`, build/install, and run `ctest --test-dir build-cap -C Release --output-on-failure`. Successful synthetic tests are necessary but not sufficient for real-camera acceptance.
