#!/usr/bin/env python

import rospy

# import ros msgs
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64

import ema.modules.forcesensor as forcesensor

def main():
    # init forcesensor node
    rospy.init_node('forcesensor', anonymous=False)

    # get forcesensor config
    fs_manager = forcesensor.ForceSensor(rospy.get_param('/ema/forcesensor'))

    # list published topics
    pub = {}
    for name in fs_manager.sensors:
        pub[name] = rospy.Publisher('forcesensor/' + name, WrenchStamped, queue_size=10)
        pub[name + '_signal'] = rospy.Publisher('forcesensor/' + name + '_fsignal', Float64, queue_size=10)

    # define loop rate (in hz)
    rate = rospy.Rate(100)

    # node loop
    while not rospy.is_shutdown():

        try:
            timestamp = rospy.Time.now()
            frame_id = 'base_link'

            ## messages are shared by all force sensors
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

        # sleep until it's time to work again
        rate.sleep()
        
    # cleanup
    fs_manager.shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
