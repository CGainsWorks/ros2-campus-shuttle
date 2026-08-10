# Aggregate metrics for one completed tracking run.
#
# Mirrors the columns of results/bench/metrics.csv exactly, so a Gazebo run and
# an offline benchmark run can be fed to the same analysis script.

std_msgs/Header header

string controller
string scenario
float64 target_speed
bool noise

# Cross-track error statistics, m.
float64 ct_mean_abs
float64 ct_rms
float64 ct_max_abs
float64 ct_p95_abs
float64 ct_final_abs

# Heading error statistics, rad.
float64 hd_mean_abs
float64 hd_rms
float64 hd_max_abs
float64 hd_p95_abs
float64 hd_final_abs

float64 duration             # s
float64 path_length          # m
float64 distance_travelled   # m
float64 progress             # 0..1 fraction of the path completed
bool goal_reached
bool diverged
bool stalled

float64 mean_speed           # m/s
float64 control_effort       # rad/s^2, RMS of d(omega)/dt
float64 rms_omega            # rad/s
float64 max_lateral_accel    # m/s^2
uint32 samples
