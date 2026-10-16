# Camera-to-LiDAR extrinsic calibration

## What is being solved

The unknown is the rigid transform `T_cam_lidar` mapping points from
`lidar_link` into `camera_optical_frame`.

A checkerboard is presented in several poses. For each pose the camera gives the
board **plane** in the camera frame (unit normal `n`, offset `d`, so `n . p = d`
for any point `p` on the board) and the LiDAR gives a set of points that lie on
that same physical board. The correct transform puts every LiDAR board point on
the camera's board plane, so the residual for LiDAR point `q` is

```
r = n . (R q + t) - d
```

a point-to-plane error in metres. Six parameters, minimised by
Levenberg-Marquardt over an axis-angle rotation.

**Why point-to-plane.** Nothing associates a particular LiDAR return with a
particular pixel: a 2D LiDAR sees a line across the board, not corners. The
plane constraint is the strongest correspondence-free constraint available and
it is what makes the problem solvable from a target that is just a flat board.

## Observability

A single board pose constrains only the direction normal to it. **At least three
non-parallel board orientations are required** to fix all three rotational
degrees of freedom and translation along a direction lying in every observed
plane is never observable at all.

So vary the board's **orientation**, not just its distance. Holding it
fronto-parallel and walking backwards produces a set of nearly identical
normals and the solve is then ill-posed no matter how many captures are taken.

The solver reports the condition number of `J^T J` and warns above `1e4`. This is
the check that stops a degenerate capture session from producing a confident
wrong answer and `tests/test_calibration.py` verifies that a near-parallel set is
correctly flagged.

## Reproducing the reported residual

No hardware needed:

```bash
docker compose run --rm calibration
```

or directly:

```bash
python3 -m shuttle_perception.synth_calib_data --out calibration/observations.json
python3 -m shuttle_perception.calibrate_extrinsics \
    --observations calibration/observations.json \
    --out-yaml calibration/extrinsics.yaml
```

The synthetic generator starts from a known transform, so the solve is scored
against a true answer and not only against its own residual. That distinction
matters: a solver that converges neatly to the wrong transform still reports a
small residual.

The reported figures are in [../docs/RESULTS.md](../docs/RESULTS.md) and the
README.

## Capturing on a real robot

```bash
ros2 run shuttle_perception collect_calib_data \
    --ros-args -p board_cols:=9 -p board_rows:=6 -p square_size:=0.045
```

For each board pose:

```bash
ros2 service call /collect_calib_data/capture std_srvs/srv/Trigger
```

then when done:

```bash
ros2 service call /collect_calib_data/save std_srvs/srv/Trigger
```

and run `calibrate_extrinsics` on the resulting JSON. The schema is identical to
the synthetic one, so the same solver runs on both.

Practical notes:

- Keep the board **fully inside both sensors' fields of view**. With a 2D LiDAR
  that means the scan plane must actually cross the board, so mount height
  matters: a board held too high produces a perfect image and no LiDAR points.
- 10 to 15 poses spread over roughly 1.5 to 3 m works well. Beyond that the residual
  stops improving.
- The scan gate (`scan_max_range`, `scan_half_fov`) should be tight enough that
  the board is the dominant planar feature in the window, or RANSAC will
  happily fit a wall instead.

## Interpreting the residual

The residual should sit at the **LiDAR noise floor**. Much larger means a model
error: a wrong board size, a bad PnP solve, or scan points that are not
actually on the board. Much smaller means the pose set is too small or too
degenerate and the solve is fitting noise.

`tests/test_calibration.py` asserts this relationship directly, checking the
residual lands within a factor of two of the injected sensor noise and that it
scales when the noise does.
