# Message from GPCHC NMEA String
# $GPCHC,GPSWeek,GPSTime,Heading,Pitch,Roll,gyro x,gyro y,gyro z,acc x,acc y,accz,Lattitude,Longitude,Altitude,Ve,Vn,Vu,V,NSV1,NSV2,Status,Age,WarningCs<CR><LF>

std_msgs/Header header

string message_id

uint32    wnc # 自 1980-1-6 至当前的星期数（GPS 时间）
float32   tow # 自本周日 0:00:00 至 当前的秒数（GPS 时间）

float32 heading  # 航向角（0 至 359.99）北偏东顺时针为正
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
float32 v

uint32  nsv1
uint32  nsv2

uint32  status
uint32  age
uint32  warning