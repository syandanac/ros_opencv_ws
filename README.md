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
# Buat workspace
mkdir -p ~/opencv_ws/src
cd ~/opencv_ws/src

# Clone/Salin package di sini
catkin_create_pkg drone_control std_msgs rospy sensor_msgs geometry_msgs cv_bridge mavros_msgs

# Build dan Source
cd ~/opencv_ws
catkin_make
echo "source ~/opencv_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Berikan izin eksekusi pada script
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
gazebo --verbose worlds/iris_arducopter_runway.world

```

**Terminal 3: MAVROS**

```bash
roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"

```

**Terminal 4: Vision & Mission**

```bash
rosrun drone_control detector_node.py
# Pada tab/terminal baru:
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
