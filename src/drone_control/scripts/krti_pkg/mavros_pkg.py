from krti_pkg.print_colours import *
import rospy
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, OverrideRCIn, GlobalPositionTarget, PositionTarget
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from mavros_msgs.srv import CommandLong, CommandLongRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from threading import Thread, Lock
import math
import os



class gnc_api:
    def __init__(self):
        """This function is called at the beginning of a program and will start of the communication links to the FCU.
        """
        self.current_state_g = State()
        self.current_pose_g = Odometry()
        self.pose_lock = Lock()
        self.correction_vector_g = Pose()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()
        self.global_waypoint = GlobalPositionTarget()
        self.target_msg = PositionTarget()
        self.altitude_msg = Float64()
        self.local_position_odom = Odometry()

        self.current_heading_g = 0.0
        self.local_offset_g = 0.0
        self.correction_heading_g = 0.0
        self.local_desired_heading_g = 0.0
        self.current_position = None
        self.relative_altitude = 0.0
        self.current_yaw = 0.0

        self.current_pose = None
        self.target_waypoint = None
        self.navigation_active = False
        self.waypoint_reached_threshold = 0.2  
        self.max_velocity = 0.5  
        self.kp_linear = 0.5     
        self.kp_yaw = 0.8        

        self.ns = rospy.get_namespace()
        if self.ns == "/":
            rospy.loginfo(CBLUE2 + "Using default namespace" + CEND)
        else:
            rospy.loginfo(CBLUE2 + "Using {} namespace".format(self.ns) + CEND)

        self.local_pos_pub = rospy.Publisher(
            name="{}mavros/setpoint_position/local".format(self.ns),
            data_class=PoseStamped,
            queue_size=10,
        )

        self.global_pos_pub = rospy.Publisher(
            name="{}mavros/setpoint_raw/global".format(self.ns),
            data_class=GlobalPositionTarget,
            queue_size=10, 
        )

        self.local_target_pub = rospy.Publisher(
            name="{}mavros/setpoint_raw/local".format(self.ns),
            data_class=PositionTarget,
            queue_size=1,
        )

        self.cmd_vel_pub = rospy.Publisher(
            name="{}mavros/setpoint_velocity/cmd_vel_unstamped".format(self.ns), 
            data_class=Twist, 
            queue_size=1
        )

        # Tanda
        self.rc_override_pub = rospy.Publisher(
            name="{}mavros/rc/override".format(self.ns), 
            data_class=OverrideRCIn, 
            queue_size=1
        )


        self.local_position_odom = rospy.Subscriber(
            name='{}mavros/local_position/odom'.format(self.ns), 
            data_class=Odometry, 
            callback=self.odom_callback
        )

        self.altitude_msg = rospy.Subscriber(
            name="{}mavros/global_position/rel_alt".format(self.ns),
            data_class=Float64,
            callback=self.rel_alt_cb
        )

        self.waypoint_reached_sub = rospy.Subscriber(
            name= "{}mavros/global_position/global".format(self.ns),
            data_class=NavSatFix,
            callback=self.waypoint_reached_cb
        )

        self.currentPos = rospy.Subscriber(
            name="{}mavros/global_position/local".format(self.ns),
            data_class=Odometry,
            queue_size=10,
            callback=self.pose_cb,
        )

        self.state_sub = rospy.Subscriber(
            name="{}mavros/state".format(self.ns),
            data_class=State,
            queue_size=10,
            callback=self.state_cb,
        )

        rospy.Subscriber(
            name="{}mavros/vision_pose/pose".format(self.ns), 
            data_class=PoseStamped, 
            callback=self.pose_t265_callback
            )

        rospy.wait_for_service("{}mavros/cmd/arming".format(self.ns))

        self.arming_client = rospy.ServiceProxy(
            name="{}mavros/cmd/arming".format(self.ns), service_class=CommandBool
        )

        rospy.wait_for_service("{}mavros/cmd/land".format(self.ns))

        self.land_client = rospy.ServiceProxy(
            name="{}mavros/cmd/land".format(self.ns), service_class=CommandTOL
        )

        rospy.wait_for_service("{}mavros/cmd/takeoff".format(self.ns))

        self.takeoff_client = rospy.ServiceProxy(
            name="{}mavros/cmd/takeoff".format(self.ns), service_class=CommandTOL
        )

        rospy.wait_for_service("{}mavros/set_mode".format(self.ns))

        self.set_mode_client = rospy.ServiceProxy(
            name="{}mavros/set_mode".format(self.ns), service_class=SetMode
        )

        rospy.wait_for_service("{}mavros/cmd/command".format(self.ns))

        self.command_client = rospy.ServiceProxy(
            name="{}mavros/cmd/command".format(self.ns), service_class=CommandLong
        )
        rospy.loginfo(CBOLD + CGREEN2 + "Initialization Complete." + CEND)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)

    def state_cb(self, message):
        self.current_state_g = message

    def waypoint_reached_cb(self, msg):
        self.current_position = msg

    def rel_alt_cb(self, data):
        self.relative_altitude = data.data

    def pose_cb(self, msg):
        """Gets the raw pose of the drone and processes it for use in control.

        Args:
                msg (geometry_msgs/Pose): Raw pose of the drone.
        """
        self.current_pose_g = msg
        self.enu_2_local()

        q0, q1, q2, q3 = (
            self.current_pose_g.pose.pose.orientation.w,
            self.current_pose_g.pose.pose.orientation.x,
            self.current_pose_g.pose.pose.orientation.y,
            self.current_pose_g.pose.pose.orientation.z,
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

        self.current_heading_g = degrees(psi) - self.local_offset_g

    def pose_t265_callback(self, msg):
        """Callback for T265 vision pose updates"""
        with self.pose_lock:
            self.current_pose = msg.pose

    def enu_2_local(self):
        x, y, z = (
            self.current_pose_g.pose.pose.position.x,
            self.current_pose_g.pose.pose.position.y,
            self.current_pose_g.pose.pose.position.z,
        )

        current_pos_local = Point()

        current_pos_local.x = x * cos(radians((self.local_offset_g - 90))) - y * sin(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.y = x * sin(radians((self.local_offset_g - 90))) + y * cos(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.z = z

        return current_pos_local

    def get_current_heading(self):
        """Returns the current heading of the drone.

        Returns:
            Heading (Float): θ in is degrees.
        """
        return self.current_heading_g

    def get_current_location(self):
        """Returns the current position of the drone.

        Returns:
            Position (geometry_msgs.Point()): Returns position of type geometry_msgs.Point().
        """
        return self.enu_2_local()
    
    def get_altitude(self):
        """
        Get altitude data for advance precision dropping outdoor.
        """
        return self.relative_altitude

    def set_speed(self, speed_mps):
        """This function is used to change the speed of the vehicle in guided mode. It takes the speed in meters per second as a float as the input.

        Args:
                speed_mps (Float): Speed in m/s.

        Returns:
                0 (int): Speed set successful.
                -1 (int): Speed set unsuccessful.
        """
        speed_cmd = CommandLongRequest()
        speed_cmd.command = 178
        speed_cmd.param1 = 1
        speed_cmd.param2 = speed_mps
        speed_cmd.param3 = -1
        speed_cmd.param4 = 0

        rospy.loginfo(
            CBLUE2 + "Setting speed to {}m/s".format(str(speed_mps)) + CEND)
        response = self.command_client(speed_cmd)

        if response.success:
            rospy.loginfo(
                CGREEN2 + "Speed set successfully with code {}".format(str(response.success)) + CEND)
            rospy.loginfo(
                CGREEN2 + "Change Speed result was {}".format(str(response.result)) + CEND)
            return 0
        else:
            rospy.logerr(
                CRED2 + "Speed set failed with code {}".format(str(response.success)) + CEND)
            rospy.logerr(
                CRED2 + "Speed set result was {}".format(str(response.result)) + CEND)
            return -1

    def land(self):
        """The function changes the mode of the drone to LAND.

        Returns:
                0 (int): LAND successful
                -1 (int): LAND unsuccessful.
        """
        srv_land = CommandTOLRequest(0, 0, 0, 0, 0)
        response = self.land_client(srv_land)
        if response.success:
            rospy.loginfo(
                CGREEN2 + "Land Sent {}".format(str(response.success)) + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "Landing failed" + CEND)
            return -1

    def wait4connect(self):
        """Wait for connect is a function that will hold the program until communication with the FCU is established.

        Returns:
                0 (int): Connected to FCU.
                -1 (int): Failed to connect to FCU.
        """
        rospy.loginfo(CYELLOW2 + "Waiting for FCU connection" + CEND)
        while not rospy.is_shutdown() and not self.current_state_g.connected:
            rospy.sleep(0.01)
        else:
            if self.current_state_g.connected:
                rospy.loginfo(CGREEN2 + "FCU connected" + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Error connecting to drone's FCU" + CEND)
                return -1

    def wait4start(self):
        """This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.

        Returns:
                0 (int): Mission started successfully.
                -1 (int): Failed to start mission.
        """
        rospy.loginfo(CYELLOW2 + CBLINK +
                      "Waiting for user to set mode to GUIDED" + CEND)
        while not rospy.is_shutdown() and self.current_state_g.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            if self.current_state_g.mode == "GUIDED":
                rospy.loginfo(
                    CGREEN2 + "Mode set to GUIDED. Starting Mission..." + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Error startting mission" + CEND)
                return -1

    def set_mode(self, mode):
        """This function changes the mode of the drone to a user specified mode. This takes the mode as a string. Ex. set_mode("GUIDED").

        Args:
                mode (String): Can be set to modes given in https://ardupilot.org/copter/docs/flight-modes.html

        Returns:
                0 (int): Mode Set successful.
                -1 (int): Mode Set unsuccessful.
        """
        SetMode_srv = SetModeRequest(0, mode)
        response = self.set_mode_client(SetMode_srv)
        if response.mode_sent:
            rospy.loginfo(CGREEN2 + "SetMode Was successful" + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "SetMode has failed" + CEND)
            return -1

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send command velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def set_vel_frame(self, vx=0.0, vy=0.0, vz=0.0):
        """
        Send command velocities based on frame. Must be in GUIDED mode. 
        """
        self.target_msg.header.stamp = rospy.Time.now()

        self.target_msg.coordinate_frame = 8

        self.target_msg.velocity.x = vx
        self.target_msg.velocity.y = vy
        self.target_msg.velocity.z = vz

        self.target_msg.type_mask = 1479

        self.local_target_pub.publish(self.target_msg)
    
    def set_vel_duration(self, vx, vy, vz, duration):
        """
        Send command velocities with some of duration. Must in GUIDED mode.
        """
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(duration):
            self.set_vel(vx, vy, vz)
            rospy.sleep(0.1)
        self.set_vel(0,0,0)

    def set_vel_frame_duration(self, vx, vy, vz, duration):
        """
        Send command velocities based on frame with some of duration. Must in GUIDED mode.
        """
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(duration):
            self.set_vel_frame(vx, vy, vz)
            rospy.sleep(0.1)
        self.set_vel_frame(0,0,0)

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculate the distance between two GPS coordinates in meters Haversine formula
        """
        R = 6371000  
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

    def move_waypoint(self, latitude, longitude, altitude):
        """
        Move using waypoint coordinate. Must in GUIDED mode.
        """
        self.set_speed(0.5)
        self.global_waypoint.coordinate_frame = 6  
        self.global_waypoint.type_mask = 4088 #448 #455 #1472   
        
        self.global_waypoint.altitude = altitude
        self.global_waypoint.latitude = latitude
        self.global_waypoint.longitude = longitude

        while True:
            if self.current_position is not None:
                current_lat = self.current_position.latitude
                current_lon = self.current_position.longitude
                distance = self.calculate_distance(current_lat, current_lon, latitude, longitude)
                
                rospy.loginfo("Current distance to waypoint: {:.2f} meters".format(distance))

                if distance < 1.5: 
                    rospy.loginfo("Reached waypoint within threshold distance. Breaking loop.")
                    break

            self.global_pos_pub.publish(self.global_waypoint)
            rospy.loginfo("Waypoint move: lat: {}, long: {}, alt: {}".format(latitude, longitude, altitude))     
            rospy.sleep(0.2)  

    def rotate_right(self):  
        """
        Rotate drone by 90 deegre. Must in GUIDED mode
        """  
        angular_speed = math.radians(45) 
        
        angle = math.radians(90) 
        
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        
        while current_angle < angle:
            #pub.publish(vel_msg)
            self.set_vel(0.3,0,0,avz=-angular_speed)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
            rospy.loginfo("Rotate : avz: 0.5")
        
        self.set_vel(0,0,0)
        rospy.sleep(1)

    def rotate_left(self):  
        """
        Rotate drone by 90 deegre. Must in GUIDED mode
        """  
        angular_speed = math.radians(45) 
        
        angle = math.radians(90) 
        
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        
        while current_angle < angle:
            #pub.publish(vel_msg)
            self.set_vel(0.3,0,0,avz=angular_speed)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
            rospy.loginfo("Rotate : avz: 0.5")
        
        self.set_vel(0,0,0)
        rospy.sleep(1)

    def set_heading(self, heading):
        """This function is used to specify the drone's heading in the local reference frame. Psi is a counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

        Args:
                heading (Float): θ(degree) Heading angle of the drone.
        """
        self.local_desired_heading_g = heading
        heading = heading + self.correction_heading_g + self.local_offset_g

        rospy.loginfo("The desired heading is {}".format(
            self.local_desired_heading_g))

        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        self.waypoint_g.pose.orientation = Quaternion(qx, qy, qz, qw)

    def set_destination(self, x, y, z, psi):
        """This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

        Args:
                x (Float): x(m) Distance with respect to your local frame.
                y (Float): y(m) Distance with respect to your local frame.
                z (Float): z(m) Distance with respect to your local frame.
                psi (Float): θ(degree) Heading angle of the drone.
        """
        self.set_heading(psi)

        theta = radians((self.correction_heading_g + self.local_offset_g - 90))

        Xlocal = x * cos(theta) - y * sin(theta)
        Ylocal = x * sin(theta) + y * cos(theta)
        Zlocal = z

        x = Xlocal + self.correction_vector_g.position.x + self.local_offset_pose_g.x

        y = Ylocal + self.correction_vector_g.position.y + self.local_offset_pose_g.y

        z = Zlocal + self.correction_vector_g.position.z + self.local_offset_pose_g.z

        rospy.loginfo(
            "Destination set to x:{} y:{} z:{} origin frame".format(x, y, z))

        self.waypoint_g.pose.position = Point(x, y, z)

        self.local_pos_pub.publish(self.waypoint_g)

    def arm(self):
        """Arms the drone for takeoff.

        Returns:
                0 (int): Arming successful.
                -1 (int): Arming unsuccessful.
        """
        self.set_destination(0, 0, 0, 0)

        for _ in range(100):
            self.local_pos_pub.publish(self.waypoint_g)
            rospy.sleep(0.01)

        rospy.loginfo(CBLUE2 + "Arming Drone" + CEND)

        arm_request = CommandBoolRequest(True)

        while not rospy.is_shutdown() and not self.current_state_g.armed:
            rospy.sleep(0.1)
            response = self.arming_client(arm_request)
            self.local_pos_pub.publish(self.waypoint_g)
        else:
            if response.success:
                rospy.loginfo(CGREEN2 + "Arming successful" + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Arming failed" + CEND)
                return -1

    def takeoff(self, takeoff_alt):
        """The takeoff function will arm the drone and put the drone in a hover above the initial position.

        Args:
                takeoff_alt (Float): The altitude at which the drone should hover.

        Returns:
                0 (int): Takeoff successful.
                -1 (int): Takeoff unsuccessful.
        """
        self.arm()
        takeoff_srv = CommandTOLRequest(0, 0, 0, 0, takeoff_alt)
        response = self.takeoff_client(takeoff_srv)
        rospy.sleep(3)
        if response.success:
            rospy.loginfo(CGREEN2 + "Takeoff successful" + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "Takeoff failed" + CEND)
            return -1

    def initialize_local_frame(self):
        """This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to."""
        self.local_offset_g = 0.0

        for i in range(30):
            rospy.sleep(0.1)

            q0, q1, q2, q3 = (
                self.current_pose_g.pose.pose.orientation.w,
                self.current_pose_g.pose.pose.orientation.x,
                self.current_pose_g.pose.pose.orientation.y,
                self.current_pose_g.pose.pose.orientation.z,
            )

            psi = atan2((2 * (q0 * q3 + q1 * q2)),
                        (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

            self.local_offset_g += degrees(psi)
            self.local_offset_pose_g.x += self.current_pose_g.pose.pose.position.x
            self.local_offset_pose_g.y += self.current_pose_g.pose.pose.position.y
            self.local_offset_pose_g.z += self.current_pose_g.pose.pose.position.z

        self.local_offset_pose_g.x /= 30.0
        self.local_offset_pose_g.y /= 30.0
        self.local_offset_pose_g.z /= 30.0
        self.local_offset_g /= 30.0

        rospy.loginfo(CBLUE2 + "Coordinate offset set" + CEND)
        rospy.loginfo(
            CGREEN2 + "The X-Axis is facing: {}".format(self.local_offset_g) + CEND)

    def check_waypoint_reached(self, pos_tol=0.3, head_tol=0.01):
        """This function checks if the waypoint is reached within given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.

        Args:
                pos_tol (float, optional): Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.3.
                head_tol (float, optional): Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.

        Returns:
                1 (int): Waypoint reached successfully.
                0 (int): Failed to reach Waypoint.
        """
        self.local_pos_pub.publish(self.waypoint_g)

        dx = abs(
            self.waypoint_g.pose.position.x - self.current_pose_g.pose.pose.position.x
        )
        dy = abs(
            self.waypoint_g.pose.position.y - self.current_pose_g.pose.pose.position.y
        )
        dz = abs(
            self.waypoint_g.pose.position.z - self.current_pose_g.pose.pose.position.z
        )

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        cosErr = cos(radians(self.current_heading_g)) - cos(
            radians(self.local_desired_heading_g)
        )

        sinErr = sin(radians(self.current_heading_g)) - sin(
            radians(self.local_desired_heading_g)
        )

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dMag < pos_tol and dHead < head_tol:
            return 1
        else:
            return 0
    
    def kill_node(self, node_name):
        """
        Kill the find_ball node using rosnode kill
        """
        os.system('rosnode kill /{}'.format(node_name))

    def gps_usage(self):
        """
        This function is to change the position control gps.

        Args:
                pwm_value (int): Sending PWM value using mavros. 
        """
        pwm_value = 1000

        rc_msg = OverrideRCIn()
        rc_msg.channels[4] = pwm_value
        self.rc_override_pub.publish(rc_msg)
        rospy.loginfo_once("GPS EKF used with PWM value: %d", pwm_value)

    def t265_vision_usage(self):
        """
        This function is to change the position control to vision.

        Args:
                pwm_value (int): Sending PWM value using mavros. 
        """
        pwm_value = 1000

        rc_msg = OverrideRCIn()
        rc_msg.channels[4] = pwm_value
        self.rc_override_pub.publish(rc_msg)
        rospy.loginfo_once("T265 vision EKF used with PWM value: %d", pwm_value)
    
    def servo_usage(self, servo_number, pwm_value):
        """
        This function is to control servo output based on PWM.
        Need : 

        Args:
                pwm_value (int): Sending PWM value using mavros. 
        """
        rc_msg = OverrideRCIn()
        rc_msg.channels[servo_number] = pwm_value
        self.rc_override_pub.publish(rc_msg)
        rospy.loginfo_once("Servo {} Override sent with PWM value: {}".format(servo_number, pwm_value))

    def heading_set_vel_yaw(self, v_x_body, v_y_body, v_z_body, yaw_deg):
        """
        Send velocity command based on heading but still in ENU coordinate system with yaw. Must in GUIDED mode.
        """
        self.target_msg.header.stamp = rospy.Time.now()
        self.target_msg.header.frame_id = "local_ned"  

        self.target_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        self.target_msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |  
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |  
            PositionTarget.IGNORE_YAW_RATE  
        )

        yaw_rad = math.radians(yaw_deg)
        new_yaw = self.current_yaw + yaw_rad

        if new_yaw > math.pi:
            new_yaw -= 2 * math.pi
        elif new_yaw < -math.pi:
            new_yaw += 2 * math.pi

        self.target_msg.velocity.x = v_x_body * math.cos(self.current_yaw) - v_y_body * math.sin(self.current_yaw)
        self.target_msg.velocity.y = v_x_body * math.sin(self.current_yaw) + v_y_body * math.cos(self.current_yaw)
        self.target_msg.velocity.z = v_z_body  

        self.target_msg.yaw = new_yaw

        self.local_target_pub.publish(self.target_msg)
        rospy.loginfo("Published raw velocity: x={}, y={}, z={}, yaw={}".format(
            self.target_msg.velocity.x, self.target_msg.velocity.y, self.target_msg.velocity.z, yaw_deg
        ))

    def heading_set_vel(self, v_x_body, v_y_body, v_z_body):
        """
        Send velocity command based on heading but still in ENU coordinate system without yaw. Must in GUIDED mode.
        """
        self.target_msg.header.stamp = rospy.Time.now()
        self.target_msg.header.frame_id = "local_ned"  

        self.target_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        self.target_msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |  
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |  
            PositionTarget.IGNORE_YAW_RATE | PositionTarget.IGNORE_YAW  
        )

        self.target_msg.velocity.x = v_x_body * math.cos(self.current_yaw) - v_y_body * math.sin(self.current_yaw)
        self.target_msg.velocity.y = v_x_body * math.sin(self.current_yaw) + v_y_body * math.cos(self.current_yaw)
        self.target_msg.velocity.z = v_z_body  

        self.local_target_pub.publish(self.target_msg)
        rospy.loginfo("Published raw velocity: x={}, y={}, z={}".format(
            self.target_msg.velocity.x, self.target_msg.velocity.y, self.target_msg.velocity.z
        ))

    def heading_set_vel_duration(self, v_x_body, v_y_body, v_z_body, duration): 
        """
        Send command velocities based on NED drone heading with some of duration. Must in GUIDED mode.
        """
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(duration):
            self.heading_set_vel(v_x_body, v_y_body, v_z_body)
            rospy.sleep(0.1)
        self.heading_set_vel(0,0,0)

    # T265 Waypoint Control
    def set_waypoint_t265(self, x, y, z):
        """Set a new target waypoint"""
        self.target_waypoint = {
        'position': {'x': x, 'y': y, 'z': z},
        }
        rospy.loginfo(f"New waypoint set: {self.target_waypoint}")

    def calculate_distance(self, point_a, point_b):
        """
        Calculate Euclidean distance between two 3D points in meters
        """
        dx = point_b['x'] - point_a['x']
        dy = point_b['y'] - point_a['y']
        dz = point_b['z'] - point_a['z']
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def calculate_direction_vector(self, current_pos, target_pos):
        """
        Calculate normalized direction vector from current to target position
        """
        dx = target_pos['x'] - current_pos['x']
        dy = target_pos['y'] - current_pos['y']
        dz = target_pos['z'] - current_pos['z']
        
        magnitude = math.sqrt(dx**2 + dy**2 + dz**2)
        if magnitude > 0:
            return dx/magnitude, dy/magnitude, dz/magnitude
        return 0, 0, 0

    def navigate_to_waypoint_t265(self):
        """
        Main navigation function - runs in a loop
        """
        rate = rospy.Rate(3)
        while not rospy.is_shutdown() and self.navigation_active:
            if self.current_pose is None or self.target_waypoint is None:
                rate.sleep()
                continue
            
            current_pos = {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'z': self.current_pose.position.z
            }
            
            distance = self.calculate_distance(current_pos, self.target_waypoint['position'])
            
            if distance < self.waypoint_reached_threshold:
                rospy.loginfo("Waypoint reached!")
                self.heading_set_vel(0, 0, 0)  
                self.navigation_active = False
                break
            
            dir_x, dir_y, dir_z = self.calculate_direction_vector(
                current_pos, self.target_waypoint['position']
            )
            
            velocity_magnitude = min(self.kp_linear * distance, self.max_velocity)
            
            v_x = velocity_magnitude * dir_x
            v_y = velocity_magnitude * dir_y  
            v_z = velocity_magnitude * dir_z
            
            
            self.heading_set_vel(v_x, v_y, v_z)
        
            rate.sleep()

    def start_navigation(self):
        """Start autonomous navigation to current waypoint"""
        if self.target_waypoint is None:
            rospy.logwarn("No waypoint set!")
            return False
        
        if not self.navigation_active:
            self.navigation_active = True
            nav_thread = Thread(target=self.navigate_to_waypoint_t265)
            nav_thread.daemon = True
            nav_thread.start()
            return True
        return False
