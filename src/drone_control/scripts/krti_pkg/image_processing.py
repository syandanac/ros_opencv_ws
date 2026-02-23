# Heading or FLU image processing code

import rospy
from geometry_msgs.msg import Point

class ChaseObject:
    def __init__(self):
        self.K_LAT_DIST_TO_STEER = 1.0
        self.blob_x = 0.0
        self.blob_y = 0.0
        self.conditional = 0.0
        self._time_detected = rospy.Time(0)  

        self.sub_center = rospy.Subscriber("/blob/point_blob", Point, self.update_ball) ##
    
    def saturate(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    @property
    def is_detected(self):
        return (rospy.get_rostime() - self._time_detected).to_sec() < 1.0

    def update_ball(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self.conditional = message.z
        self._time_detected = rospy.get_rostime()  

    def get_control_action(self):
        steer_action_x = 0.0
        steer_action_y = 0.0
        condition = True if self.conditional == 1.0 else False 

        if self.is_detected:
            steer_action_x = -self.K_LAT_DIST_TO_STEER * self.blob_x
            steer_action_x = self.saturate(steer_action_x, -0.4, 0.4)

            steer_action_y = -self.K_LAT_DIST_TO_STEER * self.blob_y
            steer_action_y = self.saturate(steer_action_y, -0.4, 0.4)

            if abs(self.blob_x) < 0.01 and abs(self.blob_y) < 0.01:
                return None, None, condition  

            rospy.loginfo("Object Found: x: {}, y:{}, condition:{}".format(steer_action_x, steer_action_y, condition))
        else:
            rospy.loginfo("Object not detected: x: 0.3, y: 0.0")
            steer_action_y = 0.3
            steer_action_x = 0.0

        return steer_action_x, steer_action_y, condition

class ChaseBasket:
    def __init__(self):
        self.K_LAT_DIST_TO_STEER = 1.0
        self.blob_x = 0.0
        self.blob_y = 0.0
        self.conditional = 0.0
        self._time_detected = rospy.Time(0)  

        self.sub_center = rospy.Subscriber("/blob/point_blob", Point, self.update_ball)
    
    def saturate(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    @property
    def is_detected(self):
        return (rospy.get_rostime() - self._time_detected).to_sec() < 1.0

    def update_ball(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self.conditional = message.z
        self._time_detected = rospy.get_rostime()  

    def get_control_action(self):
        steer_action_x = 0.0
        steer_action_y = 0.0
        condition = True if self.conditional == 1.0 else False 

        if self.is_detected:
            steer_action_x = -self.K_LAT_DIST_TO_STEER * self.blob_x
            steer_action_x = self.saturate(steer_action_x, -0.4, 0.4)

            steer_action_y = -self.K_LAT_DIST_TO_STEER * self.blob_y
            steer_action_y = self.saturate(steer_action_y, -0.4, 0.4)

            if abs(self.blob_x) < 0.01 and abs(self.blob_y) < 0.01:
                return None, None, condition  

            rospy.loginfo("Basket Found: x: {}, y:{}, condition:{}".format(steer_action_x, steer_action_y, condition))
        else:
            rospy.loginfo("Basket not detected: x: 0.3, y: 0.0")
            steer_action_y = 0.3
            steer_action_x = 0.0

        return steer_action_x, steer_action_y, condition
    
class ChaseOutdoor:
    def __init__(self):
        self.K_LAT_DIST_TO_STEER = 1.0
        self.blob_x = 0.0
        self.blob_y = 0.0
        self.conditional = 0.0
        self._time_detected = rospy.Time(0)  

        self.sub_center = rospy.Subscriber("/outdoor/point_contour", Point, self.update_ball)
    
    def saturate(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    @property
    def is_detected(self):
        return (rospy.get_rostime() - self._time_detected).to_sec() < 1.0

    def update_ball(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self.conditional = message.z
        self._time_detected = rospy.get_rostime()  

    def get_control_action(self):
        steer_action_x = 0.0
        steer_action_y = 0.0
        condition = True if self.conditional == 1.0 else False

        if self.is_detected:
            steer_action_x = self.K_LAT_DIST_TO_STEER * self.blob_x
            steer_action_x = self.saturate(steer_action_x, -0.4, 0.4)

            steer_action_y = -self.K_LAT_DIST_TO_STEER * self.blob_y
            steer_action_y = self.saturate(steer_action_y, -0.4, 0.4)

            if abs(self.blob_x) < 0.05 and abs(self.blob_y) < 0.05:
                return None, None, condition  

            rospy.loginfo("Outdoor Found: x: {}, y:{}, condition:{}".format(steer_action_x, steer_action_y, condition))
        else:
            rospy.loginfo("Outdoor not detected: x: 0.0, y: 0.0")
            steer_action_y = 0.0
            steer_action_x = 0.0

        return -steer_action_x, steer_action_y, condition
