import math
import rclpy
from geometry_msgs.msg import Twist

K_VERTICAL_THRUST = 68.5
K_VERTICAL_P      = 5.0
K_VERTICAL_D      = 2.5
K_ROLL_P          = 50.0
K_PITCH_P         = 30.0
K_YAW_P           = 2.0
K_X_VELOCITY_P    = 1.0
K_Y_VELOCITY_P    = 1.0
K_X_VELOCITY_I    = 0.01
K_Y_VELOCITY_I    = 0.01
LIFT_HEIGHT       = 3.0

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class MavicDriver:
    def init(self, webots_node, properties):
        self.__robot    = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.__gps   = self.__robot.getDevice('gps')
        self.__gyro  = self.__robot.getDevice('gyro')
        self.__imu   = self.__robot.getDevice('inertial unit')

        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller'),
        ]
        for p in self.__propellers:
            p.setPosition(float('inf'))
            p.setVelocity(0)

        self.__target_twist      = Twist()
        self.__vertical_ref      = LIFT_HEIGHT
        self.__linear_x_integral = 0
        self.__linear_y_integral = 0
        self.__prev_alt_err      = 0.0

        rclpy.init(args=None)
        name = self.__robot.name
        self.__node = rclpy.create_node('driver_' + name)
        self.__node.create_subscription(
            Twist, f'cmd_vel_{name}', self.__cmd_vel_callback, 1)
        self.__node.get_logger().info(
            f'Driver demarre - hauteur cible: {LIFT_HEIGHT}m')

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
        _, _, vertical = self.__gps.getValues()
        if not math.isnan(vertical) and vertical > 0.2:
            new_ref = clamp(
                self.__vertical_ref + twist.linear.z * (self.__timestep / 1000) * 3.0,
                0.3,
                vertical + 1.0
            )
            self.__vertical_ref = new_ref

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        roll, pitch, _       = self.__imu.getRollPitchYaw()
        _, _, vertical       = self.__gps.getValues()
        roll_v, pitch_v, yaw_v = self.__gyro.getValues()
        speed = self.__gps.getSpeed()

        if math.isnan(speed) or math.isnan(vertical):
            return

        roll_ref  = 0.0
        pitch_ref = 0.0

        if vertical > 0.2:
            denom = abs(roll) + abs(pitch) + 1e-6
            vx =  (pitch / denom) * speed
            vy = -(roll  / denom) * speed

            ex = self.__target_twist.linear.x - vx
            ey = self.__target_twist.linear.y - vy
            self.__linear_x_integral = clamp(
                self.__linear_x_integral + ex, -2, 2)
            self.__linear_y_integral = clamp(
                self.__linear_y_integral + ey, -2, 2)

            pitch_ref = -(K_X_VELOCITY_P * ex + K_X_VELOCITY_I * self.__linear_x_integral)
            roll_ref  =   K_Y_VELOCITY_P * ey + K_Y_VELOCITY_I * self.__linear_y_integral

        # PID altitude avec derivee pour amortir les oscillations
        alt_err = self.__vertical_ref - vertical
        alt_d   = alt_err - self.__prev_alt_err
        self.__prev_alt_err = alt_err
        vertical_input = K_VERTICAL_P * alt_err + K_VERTICAL_D * alt_d

        yaw_input   = K_YAW_P   * (self.__target_twist.angular.z - yaw_v)
        roll_input  = K_ROLL_P  * clamp(roll,  -1, 1) + roll_v  + roll_ref
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_v + pitch_ref

        base = K_VERTICAL_THRUST + vertical_input
        m1 = base + yaw_input + pitch_input + roll_input
        m2 = base - yaw_input + pitch_input - roll_input
        m3 = base - yaw_input - pitch_input + roll_input
        m4 = base + yaw_input - pitch_input - roll_input

        self.__propellers[0].setVelocity(-clamp(m1, -200, 200))
        self.__propellers[1].setVelocity( clamp(m2, -200, 200))
        self.__propellers[2].setVelocity( clamp(m3, -200, 200))
        self.__propellers[3].setVelocity(-clamp(m4, -200, 200))
