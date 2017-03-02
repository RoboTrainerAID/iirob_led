
#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image

def talker():
    pub = rospy.Publisher('/leds_stripe/led_setLeds', Image, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        image = Image()
        image.height = 1
        image.width = 3
        image.encoding = 'rgba8'
        image.data = chr(255)+chr(0)+chr(0)+chr(255)+chr(0)+chr(255)+chr(0)+chr(255)+chr(0)+chr(0)+chr(255)+chr(255)
        pub.publish(image)
        rospy.sleep(5)
        image = Image()
        image.height = 1
        image.width = 3
        image.encoding = 'rgba8'
        image.data = chr(0)+chr(0)+chr(0)+chr(255)+chr(0)+chr(0)+chr(0)+chr(255)+chr(0)+chr(0)+chr(0)+chr(255)
        pub.publish(image)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
