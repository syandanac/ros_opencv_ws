# 🛸 OpenCV Drone Control - ArduPilot SITL

*Repository* ini berisi *workspace* ROS Noetic untuk kendali *quadcopter* secara *autonomous* menggunakan deteksi objek berbasis OpenCV. Sistem ini mendeteksi target berwarna oranye melalui kamera yang menghadap ke bawah (*downward-facing camera*) dan menggunakan *MAVLink/MAVROS* untuk memandu *drone* melakukan *precision landing* dalam simulasi Gazebo.

## 🏗 Arsitektur Sistem

Proyek ini mengikuti arsitektur ROS yang modular:

1. **Simulasi:** ArduPilot SITL + Gazebo 11.
2. **Vision:** *Node* OpenCV memproses *raw image frames* untuk menghitung *centroid* dari objek berwarna oranye.
3. **Kendali:** Sebuah *mission script* menginterpretasikan *vision errors* dan mengirimkan *velocity setpoints* ke *Flight Controller* (FCU) melalui *MAVROS*.

## 📁 Struktur Workspace

```text
opencv_ws/
├── src/
│   └── drone_control/
│       ├── scripts/
│       │   ├── detector_node.py      # Pemrosesan OpenCV (HSV Masking)
│       │   ├── mission_control.py    # Logika misi & pemanggilan GNC API
│       │   ├── image_processing.py   # Helper class untuk logika
│       │   └── krti_pkg/             # Modul kustom penerbangan & citra
│       └── CMakeLists.txt
└── README.md

```

## 🚀 Memulai (Getting Started)

### 1. Prasyarat (*Prerequisites*)

* Ubuntu 20.04 dengan ROS Noetic.
* ArduPilot SITL dan Gazebo 11 sudah terpasang.
* *Plugin* `ardupilot_gazebo`.
* *MAVROS* dan *dataset* GeographicLib.

### 2. Pengaturan Workspace

```bash
# 1. Buat folder workspace
mkdir -p ~/opencv_ws/src
cd ~/opencv_ws/src

# 2. Clone repository langsung ke folder src
git clone https://github.com/syandanac/ros_opencv_ws.git

# 3. Buat package drone_control (jika belum ada di dalam repo tersebut)
catkin_create_pkg drone_control std_msgs rospy sensor_msgs geometry_msgs cv_bridge mavros_msgs

# 4. Install dependencies sistem yang diperlukan
cd ~/opencv_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 5. Build dan Konfigurasi Environment
catkin_make
echo "source ~/opencv_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 6. Berikan izin eksekusi pada semua file python
chmod +x ~/opencv_ws/src/ros_opencv_ws/*.py
chmod +x ~/opencv_ws/src/drone_control/scripts/*.py

```

### 3. Menjalankan Simulasi

Buka terminal terpisah untuk setiap langkah berikut:

**Terminal 1: ArduPilot SITL**

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console

```

**Terminal 2: Gazebo**

```bash
# 1. Source file bash agar workspace terbaca oleh sistem
cd ~/ardupilot_gazebo_krti
source devel/setup.bash
# 2. Jalankan node simulasi KRTI pada g
roslaunch ardupilot_gazebo krti.launch

```

**Terminal 3: MAVROS**

```bash
roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"

```

**Terminal 4: Vision Node**

```bash
rosrun drone_control detector_node.py

```

**Terminal 5: Mission Node**

```bash
rosrun drone_control mission_control.py

```

## 🛠 Konfigurasi

### Pemetaan Koordinat (*Coordinate Mapping*)

Untuk kamera yang menghadap ke bawah, pemetaan berikut digunakan dalam `mission_control.py`:

| Image Error | Drone Body Frame | MAVROS Axis |
| --- | --- | --- |
| **Y-axis Error** | Maju / Mundur (*Forward / Backward*) | `vx` |
| **X-axis Error** | Kiri / Kanan (*Left / Right*) | `vy` |
| **Altitude** | Turun (*Descend*) | `vz` (negatif) |

### Ambang Batas HSV (*HSV Thresholds*)

Jika objek oranye tidak terdeteksi di Gazebo, sesuaikan nilai berikut pada `detector_node.py`:

* **Min:** `[5, 100, 100]`
* **Max:** `[25, 255, 255]`

## ⚠️ Catatan Keselamatan

* ***Failsafe*:*** *Mission script* mencakup status "Searching". Jika objek hilang dari pandangan, *drone* akan bergerak maju perlahan untuk menemukan kembali target.
* ***Mode*:*** Pastikan `gnc_api`diatur ke mode `GUIDED` untuk kompatibilitas dengan ArduPilot.
