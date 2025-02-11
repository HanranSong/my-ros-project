#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import Header, ColorRGBA
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern
from led_controller.srv import SetLEDColor, SetLEDColorResponse

# Use the HOSTNAME environment variable to build our topic name.
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
        # Default to white if the color is not recognized.
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

class LEDController(DTROS):
    def __init__(self, node_name):
        """
        Initialize the LED Controller node.
        """
        # DTROS automatically initializes the node.
        super(LEDController, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Publisher for LED patterns. Adjust the topic name as needed.
        publisher_topic = f"/{host_name}/led_emitter_node/led_pattern"
        self.led_pub = rospy.Publisher(publisher_topic, LEDPattern, queue_size=10)

        # Default color is blue.
        self.current_color = "blue"

        # Service to change the LED color.
        self.led_service = rospy.Service("/led/set_color", SetLEDColor, self.handle_set_color)

        rospy.loginfo("LED Controller Node is up and running, current color: %s", self.current_color)
        rospy.on_shutdown(self.shutdown_hook)

    def handle_set_color(self, req):
        """
        Service callback to change the LED color.
        """
        rospy.loginfo("LED Controller: Changing LED color to: %s", req.color)
        self.current_color = req.color
        response_message = "LED updated to " + req.color
        rospy.loginfo("LED Controller: " + response_message)
        return SetLEDColorResponse(success=True, message=response_message)

    def run(self):
        """
        Publish the current LED pattern at a fixed rate.
        In this example, we assume that the LED emitter expects a pattern for 5 LEDs.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            led_msg = LEDPattern()
            # Set the header with the current timestamp.
            led_msg.header = Header(stamp=rospy.Time.now())
            # Populate the LED pattern lists with 5 identical entries.
            led_msg.color_list = [self.current_color] * 5
            led_msg.rgb_vals = [get_color_rgba(self.current_color)] * 5
            # Optionally, set masks if your emitter node uses them.
            led_msg.color_mask = [1] * 5
            led_msg.frequency = 0.0  # steady (non-blinking) light
            led_msg.frequency_mask = [0] * 5
            self.led_pub.publish(led_msg)
            rate.sleep()

    def shutdown_hook(self):
        """
        Called when the node shuts down.
        Turns off the LED by publishing an 'off' pattern.
        """
        rospy.loginfo("Shutting down LED Controller, turning off LED.")
        led_msg = LEDPattern()
        led_msg.header = Header(stamp=rospy.Time.now())
        # Send an "off" command to all LED positions.
        led_msg.color_list = ["off"] * 5
        led_msg.rgb_vals = [get_color_rgba("off")] * 5
        led_msg.frequency = 0.0
        led_msg.frequency_mask = [0] * 5
        led_msg.color_mask = [1] * 5
        self.led_pub.publish(led_msg)
        rospy.sleep(1)  # Allow time for the message to be sent.

if __name__ == '__main__':
    # Initialize the LED controller node.
    node = LEDController("led_controller_node")
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
