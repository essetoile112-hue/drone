import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys,tty,termios,threading

def get_key():
    fd=sys.stdin.fileno()
    old=termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch=sys.stdin.read(1)
        if ch=="":ch+=sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old)
    return ch

class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_drone")
        self.pub=self.create_publisher(Twist,"/cmd_vel_Mavic_2_PRO_1",10)
        self.vx=self.vy=self.vz=self.yaw=0.0
        self.create_timer(0.05,self.publish)
    def publish(self):
        cmd=Twist()
        cmd.linear.x=self.vx
        cmd.linear.y=self.vy
        cmd.linear.z=self.vz
        cmd.angular.z=self.yaw
        self.pub.publish(cmd)
    def run(self):
        print("z=MONTER s=DESCENDRE q=GAUCHE d=DROITE fleches=AVANCER ESPACE=STOP")
        while True:
            key=get_key()
            if key=="z":self.vz=1.0;self.vx=self.vy=self.yaw=0.0;print("MONTER")
            elif key=="s":self.vz=-1.0;self.vx=self.vy=self.yaw=0.0;print("DESCENDRE")
            elif key=="q":self.yaw=1.0;self.vx=self.vy=self.vz=0.0;print("GAUCHE")
            elif key=="d":self.yaw=-1.0;self.vx=self.vy=self.vz=0.0;print("DROITE")
            elif key=="[A":self.vx=0.5;self.vy=self.vz=self.yaw=0.0;print("AVANCER")
            elif key=="[B":self.vx=-0.5;self.vy=self.vz=self.yaw=0.0;print("RECULER")
            elif key=="[C":self.vy=-0.5;self.vx=self.vz=self.yaw=0.0;print("DROITE")
            elif key=="[D":self.vy=0.5;self.vx=self.vz=self.yaw=0.0;print("GAUCHE")
            elif key==" ":self.vx=self.vy=self.vz=self.yaw=0.0;print("STOP")
            elif key=="":self.vx=self.vy=self.vz=self.yaw=0.0;break

def main():
    rclpy.init()
    node=TeleopNode()
    t=threading.Thread(target=rclpy.spin,args=(node,),daemon=True)
    t.start()
    node.run()
    rclpy.shutdown()

if __name__=="__main__":main()
