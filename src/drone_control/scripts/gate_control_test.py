#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, TwistStamped
from krti_pkg.mavros_pkg import gnc_api 

class GateTopicReader:
    def __init__(self):
        # Deklarasi variabel error posisinya drone terhadap gate, dibikin 0 semua di awal
        self.steer_x = 0.0
        self.steer_y = 0.0
        self.yaw_error = 0.0
        self.detected = False
        # Subcribe ke topik pengolahan citra/blob deteksi gate
        self.sub = rospy.Subscriber("/blob/gate_blob", Point, self.callback)

    def callback(self, msg):
        # Kalau dapet data koordinat z = 999, artinya gate gak kelihatan
        if msg.z == 999.0:
            self.detected = False
        else:
            # Update data koordinat target (x, y) dan error yaw (z)
            self.steer_x = msg.x
            self.steer_y = msg.y
            self.yaw_error = msg.z
            self.detected = True

    def get_control_action(self):
        return self.steer_x, self.steer_y, self.yaw_error, self.detected

def main():
    # Bikin node ROS buat tracking gate mandiri
    rospy.init_node("standalone_gate_controller", anonymous=True)
    
    drone = gnc_api()
    gate_reader = GateTopicReader()
    
    # Set mode drone ke GUIDED terus nunggu arming/siap terbang
    drone.set_mode("GUIDED")
    drone.wait4start()
    
    # Set ketinggian terbang awal buat nyari gate
    test_alt = 2.0
    rospy.loginfo(f"Taking off to testing horizon altitude: {test_alt}m")
    drone.takeoff(test_alt)
    rospy.sleep(5)

    rospy.loginfo("Beginning Standalone Gate Tracking Loop...")
    rate = rospy.Rate(10) # loop program dijalanin pake frekuensi 10Hz

    GATE_ALIGNED = False
    ALIGNMENT_THRESHOLD = 0.10  # Threshold toleransi error kanan kiri minimum
    YAW_THRESHOLD = 0.06        # Threshold toleransi error yaw minimum
    alignment_counter = 0

    # Publisher kecepatan pake TwistStamped (messages posisi yang ada "penanda waktunya" atau Stamp)
    vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    while not rospy.is_shutdown():
        # Ambil data pembacaan posisi gate terbaru
        steer_x, steer_y, yaw_error, detected = gate_reader.get_control_action()

        # Tahap 1: Proses centering / ngepasin posisi drone di tengah-tengah gate
        if not GATE_ALIGNED:
            if detected:
                # kecepatan kanan kiri (dikali negatif karena kalo positif arahnya kebalik)
                lateral_vel = -1.0 * (steer_x * 0.8)  
                
                # kecepatan vertikal (naik turun)
                vertical_vel = -1.0 * (steer_y * 0.8) 
                
                # kecepatan maju di-nol-kan dulu biar drone fokus centering di tempat
                forward_vel = 0.0 
                
                # kendali yaw (belok kanan kiri) pake p-gain 0.5
                kp_yaw = 0.5
                yaw_rate = yaw_error * kp_yaw
                
                # Bungkus semua variabel kecepatan tadi ke message TwistStamped
                move_cmd = TwistStamped()
                move_cmd.header.stamp = rospy.Time.now()
                move_cmd.header.frame_id = "base_link" # Pakai koordinat lokal/body drone sendiri
                
                move_cmd.twist.linear.x = forward_vel
                move_cmd.twist.linear.y = lateral_vel
                move_cmd.twist.linear.z = vertical_vel
                move_cmd.twist.angular.z = yaw_rate
                
                # Kirim perintah gerak ke FCU via MAVROS
                vel_pub.publish(move_cmd)
                
                # Cek apakah posisi drone udah masuk range toleransi aman (tengah banget)
                x_ok = abs(steer_x) <= ALIGNMENT_THRESHOLD
                y_ok = abs(steer_y) <= ALIGNMENT_THRESHOLD
                yaw_ok = abs(yaw_error) <= YAW_THRESHOLD
                
                if x_ok and y_ok and yaw_ok:
                    # Kalau udah pas di tengah, counter naik terus
                    alignment_counter += 1
                    rospy.loginfo(f"Centering Active. Alignment counter: {alignment_counter}/15")
                else:
                    # Kalau posisinya goyang/melenceng, counternya dikurangi pelan-pelan (biar stabil)
                    alignment_counter = max(0, alignment_counter - 1)
                
                # Harus nahan posisi stabil di tengah selama 15 frame berturut-turut (~1.5 detik)
                if alignment_counter >= 15:
                    GATE_ALIGNED = True
                    break
            else:
                # Proteksi darurat: Kalo gate tiba-tiba ilang, reset counter dan suruh drone diem/hover
                rospy.logwarn("Gate target lost! Resetting alignment lock and holding position...")
                drone.heading_set_vel(0.0, 0.0, 0.0)
                alignment_counter = 0

        rate.sleep()

    # Tahap 2: GASS POL TEROBOS GATE (Cuma jalan kalau drone udah bener-bener lurus di tengah)
    if GATE_ALIGNED and not rospy.is_shutdown():
        rospy.loginfo("AXIAL CONVERGENCE PERFECT. PUNCHING STRAIGHT THROUGH THE GATE.")
        
        punch_speed = 3.5  # Kecepatan maju (m/s) pas nerobos
        duration = 3.5     # Durasi nerobos dalam detik sampe bener-bener lewatin gate
        
        # Suruh drone maju lurus sesuai durasi yang diset
        drone.set_vel_frame_duration(punch_speed, 0.0, 0.0, duration)
        rospy.sleep(duration)
        
        # Setelah sukses lewat, langsung ngerem biar gak kebablasan nabrak yang lain
        rospy.loginfo("Gate cleared successfully. Braking craft.")
        drone.heading_set_vel(0.0, 0.0, 0.0)
        rospy.sleep(2)
    else:
        # Gagal nerobos karena dari awal gak dapet posisi tengah yang stabil
        rospy.logerr("Aborted breakthrough sequence: Target was never properly aligned.")

    # Selesai misi, drone otomatis landing
    rospy.loginfo("Mission complete. Landing...")
    drone.land()

if __name__ == "__main__":
    main()