#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
# Assuming gnc_api is in your scripts or krti_pkg folder
from krti_pkg.mavros_pkg import gnc_api 
from krti_pkg.image_processing import ChaseObject

def main():
    rospy.init_node("drone_controller", anonymous=True)
    
    # Initialize GNC API (Make sure this connects to MAVROS for ArduPilot)
    drone = gnc_api()
    image_processing_object = ChaseObject()
    
    #Set mode to GUIDED
    drone.set_mode("GUIDED")
    
    # ArduPilot Step 1: Wait for FCU connection and Arm
    drone.wait4start()
    
    
    # ArduPilot Step 2: Takeoff
    target_alt = 1.1
    drone.takeoff(target_alt)
    rospy.sleep(3)

    rospy.loginfo("Starting Object Chase mission...")
    rate = rospy.Rate(10) # 10Hz control loop

    while not rospy.is_shutdown():
        altitude = drone.get_altitude()
        # steer_x/y are normalized errors from the detector
        steer_x, steer_y, detected = image_processing_object.get_control_action()

        if altitude is not None:
            # CASE 1: Object found and centered, drone is low enough -> STOP & BREAK
            if detected and steer_x is None and altitude <= 1.4:
                drone.heading_set_vel(0, 0, 0)
                rospy.loginfo("Object Centered & Altitude reached. Mission Complete.")
                rospy.sleep(2)
                break

            # CASE 2: Object detected -> Move toward it
            elif detected and steer_x is not None:
                # Coordinate Mapping: 
                # Image X error -> Drone Lateral Vel (Y)
                # Image Y error -> Drone Forward Vel (X)
                # Note the signs: you may need to flip steer_x/y based on camera orientation
                forward_vel = steer_y  
                lateral_vel = steer_x
                
                # If high, descend slowly while chasing
                down_vel = -0.2 if altitude >= 1.4 else 0.0
                
                drone.heading_set_vel(forward_vel, lateral_vel, down_vel)

            # CASE 3: Object NOT detected -> Search pattern (slow forward drift)
            else:
                rospy.loginfo("Searching for object...")
                drone.heading_set_vel(0.2, 0.0, 0.0)

        rate.sleep()

    # Final Sequence
    rospy.loginfo("Landing...")
    drone.land()

if __name__ == "__main__":
    main()
    
