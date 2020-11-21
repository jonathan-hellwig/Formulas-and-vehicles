#!/usr/bin/env python
import rospy
from bluerov_sim.msg import ActuatorCommands
from mavros_msgs.srv import CommandBool

class TestNode():
    def __init__(self, name):
        rospy.init_node(name)
        
        self.arm_vehicle()

        self.thrust = 0.0
        self.thrust_stepsize = 0.1
        self.thrust_scaler = 0.8
        self.lateral_thrust = 0.0
        self.lateral_thrust_stepsize = 0.1
        self.lateral_thrust_scaler = 0.4
        self.vertical_thrust = 0.0
        self.vertical_thrust_stepsize = 0.1
        self.vertical_thrust_scaler = 0.4
        self.yaw_rate = 0.0
        self.yaw_rate_stepsize = 0.1
        self.yaw_rate_scaler = 0.2

        self.actuator_pub = rospy.Publisher("mixer/actuator_commands",
                                            ActuatorCommands,
                                            queue_size=1)
    def arm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        while not arm(True).success:
            rospy.logwarn_throttle(1, "Could not arm vehicle. Keep trying.")
        rospy.loginfo("Armed successfully.")

    def disarm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        arm(False)

    def set_thrust(self, value):
        value *= self.thrust_scaler
        self.thrust = max(-1, min(1, value))

    def set_yaw_rate(self, value):
        value *= self.yaw_rate_scaler
        self.yaw_rate = max(-1, min(1, value))

    def set_vertical_thrust(self, value):
        value *= self.vertical_thrust_scaler
        self.vertical_thrust = max(-1, min(1, value))

    def set_lateral_thrust(self, value):
        value *= self.lateral_thrust_scaler
        self.lateral_thrust = max(-1, min(1, value))

    def increase_thrust_scaler(self, value):
        self.thrust_scaler += value
        self.thrust_scaler = max(0, min(1, self.thrust_scaler))

    def increase_yaw_rate_scaler(self, value):
        self.yaw_rate_scaler += value
        self.yaw_rate_scaler = max(0, min(1, self.yaw_rate_scaler))

    def increase_vertical_thrust_scaler(self, value):
        self.vertical_thrust_scaler += value
        self.vertical_thrust_scaler = max(0, min(1,
                                                 self.vertical_thrust_scaler))

    def increase_lateral_thrust_scaler(self, value):
        self.lateral_thrust_scaler += value
        self.lateral_thrust_scaler = max(0, min(1, self.lateral_thrust_scaler))
    
    def publish_message(self):
        self.set_vertical_thrust(-1)
        msg = ActuatorCommands()
        msg.header.stamp = rospy.Time.now()
        msg.thrust = self.thrust
        msg.yaw = self.yaw_rate
        msg.lateral_thrust = self.lateral_thrust
        msg.vertical_thrust = self.vertical_thrust
        self.actuator_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():

            self.publish_message()
            rate.sleep()

def main():
    node = TestNode("test")
    rospy.logwarn("This code is run")
    node.run()


if __name__ == "__main__":
    main()