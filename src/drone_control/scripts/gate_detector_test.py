#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class StandaloneGateDetector:
    def __init__(self):
        # Inisialisasi node ROS buat deteksi posisi gate via kamera
        rospy.init_node('gate_detector_test_node')
        self.bridge = CvBridge()
        
        # Subscribe ke topik kamera bawaan Gazebo (RGB/Color)
        self.image_sub = rospy.Subscriber("camera/color/image_raw", Image, self.callback)
        
        # Output topik: koordinat blob gate buat GNC, sama streaming gambar buat debug
        self.blob_pub = rospy.Publisher("/blob/gate_blob", Point, queue_size=1)
        self.debug_pub = rospy.Publisher("/vision/gate_tracker/compressed", CompressedImage, queue_size=1)

        # Range warna HSV Oranye (sudah dituning pas buat lighting di Gazebo 11)
        self.orange_min = np.array([5, 100, 100])
        self.orange_max = np.array([25, 255, 255])

    def callback(self, data):
        try:
            # Ubah format gambar ROS Image ke format OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Ambil dimensi gambar dan cari titik tengah kamera (crosshair)
        h, w, _ = cv_image.shape
        cam_center_x = w // 2
        cam_center_y = h // 2
        
        # 1. Thresholding Warna & Filtering Noise
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.orange_min, self.orange_max)
        
        # Pake operasi morfologi OPEN buat ngilangin noise bintik-bintik kecil
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 2. Cari Kontur dan Filter Geometri Gate
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        point_msg = Point()
        gate_detected = False

        if contours:
            # Ambil kontur warna oranye yang paling besar
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # Filter luas minimum (biar gak nge-detect objek oranye yang jauh banget)
            if area > 400:  
                x, y, gate_w, gate_h = cv2.boundingRect(largest_contour)
                aspect_ratio = float(gate_w) / gate_h
                
                # Filter Aspect Ratio: Biar gak ketuker sama tiang ramping atau garis di tanah
                if 0.4 < aspect_ratio < 2.5:
                    gate_detected = True
                    # Cari titik tengah (center) dari kotak gate
                    cx = x + (gate_w // 2)
                    cy = y + (gate_h // 2)
                    
                    # --- Rumus Estimasi Error Yaw (Kemiringan Drone) ---
                    # Cuplik kerapatan pixel mask di area 15% paling kiri dan 15% paling kanan kotak
                    left_column = mask[y:y+gate_h, x:x+int(gate_w*0.15)]
                    right_column = mask[y:y+gate_h, x+int(gate_w*0.85):x+gate_w]
                    
                    # Hitung jumlah pixel putih (oranye yang terdeteksi) di kiri vs kanan
                    left_height = np.sum(left_column > 0)
                    right_height = np.sum(right_column > 0)
                    
                    # Hitung rasio ketidakseimbangan (kalau kiri lebih padat/deket, yaw_error bernilai positif)
                    yaw_error = float(left_height - right_height) / float(left_height + right_height + 1e-5)
                    
                    # Normalisasi koordinat target ke range -1.0 sampai 1.0 (biar enak diolah PID GNC)
                    point_msg.x = (cx - (w / 2.0)) / (w / 2.0)
                    point_msg.y = (cy - (h / 2.0)) / (h / 2.0)
                    point_msg.z = yaw_error  # Numpang sumbu Z buat ngirim data error yaw beloknya
                    
                    # Gambar HUD overlay: Kotak hijau, titik tengah merah, dan garis tracking biru
                    cv2.rectangle(cv_image, (x, y), (x + gate_w, y + gate_h), (0, 255, 0), 2)
                    cv2.circle(cv_image, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.line(cv_image, (cam_center_x, cam_center_y), (cx, cy), (255, 255, 0), 2)
                    
                    # Tampilin teks info error yaw di atas kotak buat debugging di darat
                    cv2.putText(cv_image, f"Yaw Err: {yaw_error:.2f}", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if not gate_detected:
            # Proteksi: Kalo gate gak kelihatan, kirim kode 999 biar GNC tahu target ilang
            point_msg.x = 0.0
            point_msg.y = 0.0
            point_msg.z = 999.0  
            
        # Gambar titik patokan tengah kamera (warna biru)
        cv2.circle(cv_image, (cam_center_x, cam_center_y), 6, (255, 0, 0), -1)

        # 3. Kirim data koordinat ke GNC dan streaming gambar terkompresi
        self.publish_compressed_telemetry(cv_image)
        self.blob_pub.publish(point_msg)

    def publish_compressed_telemetry(self, frame):
        # Kompres gambar BGR jadi JPEG biar enteng pas di-stream lewat jaringan wifi/ROS
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        success, encoded_img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        if success:
            msg.data = np.array(encoded_img).tobytes()
            self.debug_pub.publish(msg)

if __name__ == '__main__':
    try:
        detector = StandaloneGateDetector()
        rospy.spin() # Jaga node tetep hidup nunggu input kamera
    except rospy.ROSInterruptException:
        pass