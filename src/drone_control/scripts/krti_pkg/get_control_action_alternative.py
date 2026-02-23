def get_control_action(self):
        steer_action_x = 0.0
        steer_action_y = 0.0
        condition = True if self.conditional == 1.0 else False 

        if self.is_detected:
            # Calculate steer actions
            steer_action_x = -self.K_LAT_DIST_TO_STEER * self.blob_x
            steer_action_x = self.saturate(steer_action_x, -0.4, 0.4)

            steer_action_y = -self.K_LAT_DIST_TO_STEER * self.blob_y
            steer_action_y = self.saturate(steer_action_y, -0.4, 0.4)

            # DEADZONE: If within 5% of center, return None to signal "Target Reached"
            if abs(self.blob_x) < 0.05 and abs(self.blob_y) < 0.05:
                return None, None, True  

            rospy.loginfo("Object Found: x: {}, y:{}".format(steer_action_x, steer_action_y))
            return steer_action_x, steer_action_y, True
            
        else:
            # NOT DETECTED: Return 0.0, but 'condition' or 'detected' is False
            rospy.loginfo("Object not detected")
            return 0.0, 0.0, False
