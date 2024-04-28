import rclpy
from geometry_msgs.msg import Twist

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   w
a  s  d

w : Foward
s : Backward
a : Left
d : Right

anything else : stop

CTRL-C to quit
"""

up_down = {
    'w':1.0,
    's':-1.0,
}

left_right = {
    'a':1.0,
    'd':-1.0,
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(vx, vy):
    return "currently:\tvx %s\tvy %s" % (vx, vy)

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    try:
        print(msg)
        while(1):
            vx = 0.0
            vy = 0.0
            key = getKey()
            if key in up_down.keys():
                vx = up_down[key]
                print(vels(vx, vy))
            elif key in left_right.keys():
                vy = left_right[key]
                print(vels(vx, vy))
            else:
                vx = 0
                vy = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
