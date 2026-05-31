
import numpy as np

class PIDController:
    """PID controller for throttle (velocity tracking)."""

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        
        output = 0
        cp=self.Kp*error
        self.integral+=error*dt
        ci=self.Ki*self.integral
        cd=self.Kd*(error-self.prev_error)/dt
        self.prev_error=error
        output=cp+ci+cd
        return output


def pure_pursuit_steering(x, y, yaw, v, waypoints, L, k_dd, ld_min, max_steer):

    # Step 1: Compute the lookahead distance based on speed
    #   ld = k_dd * v + ld_min
    ld = k_dd * v + ld_min


    # Step 2: Find the nearest waypoint to the rear axle
    #   Compute distances from (x, y) to every waypoint, pick the closest index.
    p=0
    j=0
    for i in range(len(waypoints)):
        d=(waypoints[i][0]-x)**2 + (waypoints[i][1]-y)**2
        if(p==0): p=d
        if(d<p):
            p=d
            j=i # j is the nearest index       


    # Step 3: Search forward from the nearest waypoint to find the goal point
    #   Starting from the nearest index, walk forward along the waypoints until
    #   you find the first waypoint whose distance from (x, y) >= ld.
    #   If none is found, use the last waypoint in the array.
    m=0
    for k in range(j,len(waypoints)):
        d=(waypoints[k][0]-x)**2 + (waypoints[k][1]-y)**2
        if(d>=ld**2): 
            m=k
            break
        if(m==0): m=len(waypoints)-1

    # Step 4: Compute alpha — the angle between the vehicle heading and the
    #   direction from the rear axle to the goal point.
    #   alpha = arctan2(goal_y - y, goal_x - x) - yaw
    alpha = np.arctan2( waypoints[m][1] - y, waypoints[m][0] - x) - yaw
    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))


    # Step 5: Compute the steering angle using the Pure Pursuit formula:
    #   steer = arctan(2 * L * sin(alpha) / ld)
    steer = np.arctan(2 * L * np.sin(alpha) / ld)
    # Step 6: Clip steering to max_steer
    if steer > max_steer: steer = max_steer
    elif steer < -max_steer: steer = -max_steer

    return steer, m