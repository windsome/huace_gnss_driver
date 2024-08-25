# Message from GPCHC NMEA String
# $GPCHCX,2277,271768.00,271.21,-1.67,-2.16,0.48,-0.14,0.11,0.0387,- 0.0281,0.9986,31.15959761,121.17848221,52.17,0.01,0.00,0.03,0.01,28,23,61,0,2,2.968,3.01,3.30,0.01,0.01,0.02,0.39,0.39,1329.22,X,0.00,180.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,40,27,3512308*7F

std_msgs/Header header

string message_id

uint32    wnc  # 自 1980-1-6 至当前的星期数（GPS 时间）
float32   tow  # 自本周日 0:00:00 至 当前的秒数（GPS 时间）

float32 heading2  # 航向角（0 至 359.99）北偏东顺时针为正
float32 pitch    # 俯仰角（-90 至 90）车头上扬为正
float32 roll     # 横滚角（-180 至180）车身右倾为正

float32 gyrox
float32 gyroy
float32 gyroz
float32 accx
float32 accy
float32 accz

float64 latitude
float64 longitude
float64 altitude

float32 ve
float32 vn
float32 vu
float32 v2d

uint32  nsv1
uint32  nsv2

uint32  status
uint32  age
uint32  warning

float32 latitude_std
float32 longitude_std
float32 altitude_std

float32 ve_std
float32 vn_std
float32 vu_std

float32 roll_std
float32 pitch_std
float32 heading2_std

string separator_x

float32 heading
float32 heading_std

float32 ins2gnss_vector_x  # 设备到天线位置，车辆坐标系下，x 轴杆臂，单位 m
float32 ins2gnss_vector_y  # 设备到天线位置，车辆坐标系下，y 轴杆臂，单位 m
float32 ins2gnss_vector_z  # 设备到天线位置，车辆坐标系下，z 轴杆臂，单位 m

float32 ins2body_angle_x   # 设备坐标系到车辆坐标系旋转欧拉角，ZX-Y 顺序旋转，deg 注：补偿小角值，不包含大安装角
float32 ins2body_angle_y
float32 ins2body_angle_z

float32 gnss2body_angle_z   # 车辆朝向到 GNSS 定向方向旋转角，沿车辆坐标系 Z 轴旋转，deg 注：补偿小角值，不包含大安装角

uint32  vis_nsv1
uint32  vis_nsv2

string sn
