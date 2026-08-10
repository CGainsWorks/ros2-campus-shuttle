# Instantaneous path-tracking error, published every control step.
#
# cross_track is measured at base_link and is positive when the robot is to the
# LEFT of the path tangent. Both controllers report at the same reference point
# so the signals are directly comparable, even though Stanley internally
# regulates the error at a virtual front axle.

std_msgs/Header header

# "pure_pursuit" or "stanley"
string controller

float64 cross_track      # m, signed, positive to the left of the path
float64 heading          # rad, wrapped, path heading minus robot heading
float64 arc_length       # m, projection of the robot onto the path
float64 curvature        # 1/m, path curvature at the projection

float64 v_cmd            # m/s, commanded linear velocity
float64 omega_cmd        # rad/s, commanded yaw rate

# Controller-specific diagnostics. Unused fields are published as zero.
float64 lookahead        # m, Pure Pursuit lookahead distance
float64 steer            # rad, Stanley virtual steering angle
float64 front_axle_error # m, Stanley cross-track error at the virtual front axle

bool goal_reached
