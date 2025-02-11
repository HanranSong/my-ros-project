#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import Header, ColorRGBA
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern
from led_controller.srv import SetLEDColor, SetLEDColorResponse

host_name = os.environ["HOSTNAME"]

def get_color_rgba(color):
    """
    Map a color string to a ColorRGBA value.
    Adjust these values as needed for your hardware.
    """
    color = color.lower()
    if color == "blue":
        return ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
    elif color == "red":
        return ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    elif color == "green":
        return ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    elif color == "off":
        return ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
    else:
        # Default to white if color not recognized.
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

class LEDController(DTROS):
    def __init__(self, node_name):
        # DTROS automatically initializes the node.
        super(LEDController, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        publisher_topic = f"/{host_name}/led_emitter_node/led_pattern"
        self.led_pub = rospy.Publisher(publisher_topic, LEDPattern, queue_size=10)
        
        self.current_color = "blue"  # default color
        
        self.led_service = rospy.Service("/led/set_color", SetLEDColor, self.handle_set_color)
        
        rospy.loginfo("LED Controller Node is up and running, current color: %s", self.current_color)
        rospy.on_shutdown(self.shutdown_hook)

    def handle_set_color(self, req):
        """
        Service callback that changes the LED color.
        """
        rospy.loginfo("LED Controller: Changing LED color to: %s", req.color)
        self.current_color = req.color
        response_message = "LED updated to " + req.color
        rospy.loginfo("LED Controller: " + response_message)
        return SetLEDColorResponse(success=True, message=response_message)

    def run(self):
        """
        Publish the current LED pattern at a fixed rate.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            led_msg = LEDPattern()
            # Set the header with the current timestamp.
            led_msg.header = Header(stamp=rospy.Time.now())
            # Populate both color_list and rgb_vals fields.
            led_msg.color_list = [self.current_color]
            led_msg.rgb_vals = [get_color_rgba(self.current_color)]
            # For a steady light, set frequency to 0.0.
            led_msg.frequency = 0.0
            self.led_pub.publish(led_msg)
            rate.sleep()

    def shutdown_hook(self):
        """
        Publish an 'off' command when the node shuts down.
        """
        rospy.loginfo("Shutting down LED Controller, turning off LED.")
        led_msg = LEDPattern()
        led_msg.header = Header(stamp=rospy.Time.now())
        led_msg.color_list = ["off"]
        led_msg.rgb_vals = [get_color_rgba("off")]
        led_msg.frequency = 0.0
        self.led_pub.publish(led_msg)
        rospy.sleep(1)  # Allow time for the message to be sent.

if __name__ == '__main__':
    # DTROS handles node initialization.
    node = LEDController("led_controller_node")
    rospy.spin()
