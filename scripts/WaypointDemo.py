#!/usr/bin/env python

import rospy
import mavros
import threading
from geometry_msgs.msg import PoseStamped
from mavros import command
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix


VALID_MODES = ["ACRO", "AIRMODE", "ALTHOLD", "AUTO", "AUTOTUNE", "BRAKE", "CIRCLE", "DRIFT", "FLIP", "FLOWHOLD", "FOLLOW", "GUIDED", "HELI_AUTOROTATE", "LAND", "LOITER", "POSHOLD", "RTL", "SIMPLE", "SMARTRTL", "SPORT", "STABILIZE", "SYSID", "THROW", "ZIGZAG"]
TRAVEL_TIME = 10


class WaypointDemo(object):
    def __init__(self):

        # Reqired call - I think if you are running more than one drone it allows you to give them different topic namespaces, but defaults to mavros
        mavros.set_namespace()
        self._curr_state = State()
        self._is_done = False
        self.flight_path = []
        self.rate = rospy.Rate(20)
        self.nextPos = PoseStamped()
        self.lat = 0
        self.lng = 0

        # Various ros publishers and subscribers - somewhat self documenting
        self.gps_subscriber  = rospy.Subscriber(mavros.get_topic('global_position', 'global'), NavSatFix, self._get_fix)
        self.state_subscriber = rospy.Subscriber("mavros/state", State, self._update_state)
        self._local_position_publisher = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self._arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self._set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self._takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)


    # Callback for the gps subscriber
    def _get_fix(self, fix):
        self.lat = fix.latitude
        self.lng = fix.longitude

    # Changes the mode of the craft - the craft will only fly in certain modes, and guided mode allows it to take gps positions as waypoints
    def setMode(self, mode="GUIDED"):
        rospy.wait_for_service("mavros/set_mode")
        if not mode in VALID_MODES:
            rospy.logerr("INVALID MODE ATTEMPTED: {}".format(mode))
            return False
        try:
            isModeChanged = self._set_mode_client(custom_mode=mode)
            return isModeChanged
        except rospy.ServiceException:
            rospy.logerr("Service set_mode call failed: {} mode not set".format(mode))
            return False

    # arm or disarm the craft. If the craft is armed, not flying and doesn't recieve input for a timeout period, it disarms automatically
    def arm(self, arming=True):
        rospy.wait_for_service("mavros/cmd/arming")
        try:
            return self._arming_client(arming)
        except rospy.ServiceException:
            rospy.logerr("Service call to arm failed")

    #Callback for the state subscriber
    def _update_state(self, data):
        self._curr_state = data

    # Sets the next target position of the craft
    def setPosition(self, x, y, z):
        self.nextPos.pose.position.x = x
        self.nextPos.pose.position.y = y
        self.nextPos.pose.position.z = z

    # Publishes the setpoint for the next position the craft should target
    # Infinite loop while ros is running, so should be threaded
    def publishPos(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self._is_done:
            self._local_position_publisher.publish(self.nextPos)
            rate.sleep()

    # Reads a list of waypoint from a file 
    def readFlightFromFile(self, file_name):
        with open(file_name, 'r') as f:
            for line in f:
                print(line)
                x, y, z = [float(val.strip()) for val in line.split(',')]
                self.flight_path.append([x, y, z])


    # A ""dumb" flight path follower. Launches the craft, then fly's towards each waypoint in the flight path. Fly's for a set period of time instead of checking for arrival because it was simpler for testing purposes. 
    def startFlight(self):
        self.setMode(mode='GUIDED')
        self.arm()
        command.takeoff(min_pitch=0, yaw=1, latitude=self.lat, longitude=self.lng, altitude=10)
        rospy.sleep(TRAVEL_TIME)

        pub_thread = threading.Thread(target=self.publishPos)
        pub_thread.start()
        for point in self.flight_path:
            self.setPosition(point[0], point[1], point[2])
            rospy.sleep(TRAVEL_TIME)



        self._is_done = True
        pub_thread.join()

        command.land(min_pitch=0.0, yaw=0, latitude=self.lat, longitude=self.lng, altitude=0.0)


if __name__ == "__main__":
    rospy.init_node("waypoint_demo")
    wp = WaypointDemo()
    #This could be improved with rospack to allow calling this node from any directory instead of just the scripts folder.
    wp.readFlightFromFile("waypoints_test.data")
    wp.startFlight()

