import math
import rclpy
from geometry_msgs.msg import Twist

K_VERTICAL_THRUST = 68.5
K_VERTICAL_P      = 3.0
K_ROLL_P          = 50.0
K_PITCH_P         = 30.0
K_YAW_P           = 2.0
K_X_VELOCITY_P    = 1
K_Y_VELOCITY_P    = 1
K_X_VELOCITY_I    = 0.01
K_Y_VELOCITY_I    = 0.01
LIFT_HEIGHT       = 1.5

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class MavicDriver:
    def init(self, webots_node, properties):
        self.__robot     = webots_node.robot
        self.__timestep  = int(self.__robot.getBasicTimeStep())

        self.__gps   = self.__robot.getDevice('gps')
        self.__gyro  = self.__robot.getDevice('gyro')
        self.__imu   = self.__robot.getDevice('inertial unit')

        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller'),
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        self.__target_twist       = Twist()
        self.__vertical_ref       = LIFT_HEIGHT
        self.__linear_x_integral  = 0
        self.__linear_y_integral  = 0

        rclpy.init(args=None)
        name = self.__robot.name
        self.__node = rclpy.create_node('driver_' + name)
        self.__node.create_subscription(Twist, f'cmd_vel_{name}', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        roll, pitch, _       = self.__imu.getRollPitchYaw()
        _, _, vertical       = self.__gps.getValues()
        roll_v, pitch_v, yaw_v = self.__gyro.getValues()
        speed = self.__gps.getSpeed()

        if math.isnan(speed):
            return

        roll_ref  = 0
        pitch_ref = 0

        if vertical > 0.2:
            denom = abs(roll) + abs(pitch) + 1e-6
            vx =  (pitch / denom) * speed
            vy = -(roll  / denom) * speed

            ex = self.__target_twist.linear.x - vx
            ey = self.__target_twist.linear.y - vy
            self.__linear_x_integral += ex
            self.__linear_y_integral += ey

            roll_ref  =  K_Y_VELOCITY_P * ey + K_Y_VELOCITY_I * self.__linear_y_integral
            pitch_ref = -K_X_VELOCITY_P * ex - K_X_VELOCITY_I * self.__linear_x_integral

            # Mise a jour altitude reference
            self.__vertical_ref = clamp(
                self.__vertical_ref + self.__target_twist.linear.z * (self.__timestep / 1000),
                0.3,          # peut descendre jusqu a 0.3m
                vertical + 0.5
            )

        vertical_input = K_VERTICAL_P * (self.__vertical_ref - vertical)
        yaw_input      = K_YAW_P * (self.__target_twist.angular.z - yaw_v)
        roll_input     = K_ROLL_P  * clamp(roll,  -1, 1) + roll_v  + roll_ref
        pitch_input    = K_PITCH_P * clamp(pitch, -1, 1) + pitch_v + pitch_ref

        m1 = K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        m2 = K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        m3 = K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input
        m4 = K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input

        self.__propellers[0].setVelocity(-m1)
        self.__propellers[1].setVelocity( m2)
        self.__propellers[2].setVelocity( m3)
        self.__propellers[3].setVelocity(-m4)
