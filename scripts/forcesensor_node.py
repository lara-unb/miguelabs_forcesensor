#!/usr/bin/env python

"""

Particularly, this code initializes the force sensor device, receives its
measurements and publish them as ROS messages.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

"""

import rospy
import modules.forcesensor as forcesensor

# Import ROS msgs
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped


def main():
    # Init forcesensor node
    rospy.init_node('forcesensor', anonymous=False)

    # Get forcesensor config
    fs_manager = forcesensor.ForceSensor(rospy.get_param('/ema/forcesensor'))

    # List published topics
    pub = {}
    for name in fs_manager.sensors:
        pub[name] = rospy.Publisher('forcesensor/' + name, WrenchStamped, queue_size=10)
        pub[name + '_signal'] = rospy.Publisher('forcesensor/' + name + '_fsignal', 
                                                 Float64, queue_size=10)

    # Define loop rate (in hz)
    rate = rospy.Rate(20)

    # Node loop
    while not rospy.is_shutdown():

        try:
            timestamp = rospy.Time.now()
            frame_id = 'base_link'

            # Messages are shared by all force sensors
            forceMsg = WrenchStamped()
            forceMsg.header.stamp = timestamp
            forceMsg.header.frame_id = frame_id
            signalMsg = Float64()
            
            for name in fs_manager.sensors:
                force_vector = fs_manager.getForce(name)

                forceMsg.wrench.force.x = force_vector[0]
                forceMsg.wrench.force.y = force_vector[1]
                forceMsg.wrench.force.z = force_vector[2]
                forceMsg.wrench.torque.x = 0
                forceMsg.wrench.torque.y = 0
                forceMsg.wrench.torque.z = 0
                
                pub[name].publish(forceMsg)

                signalMsg.data = force_vector[1]
                pub[name + '_signal'].publish(signalMsg)

        except TypeError:
            print 'TypeError occured!'

        # Sleep until it's time to work again
        rate.sleep()
        
    # Cleanup the communication
    fs_manager.terminate()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
