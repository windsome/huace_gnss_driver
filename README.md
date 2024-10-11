华测CGI-430带IMU惯导的GNSS设备驱动(ROS2下)

## 过程
1. HuaceNode集成于HuaceNodeBase，实现了节点
2. HuaceNodeBase主要功能
  + param处理
  + 实现了registerSubscriber()，监听odometry或twist
  + 实现了publishMessage()，用于发布消息到autoware，会缓存topic和publisher到topicMap_
  + 实现了publishTf，用于发送TransformStamped
  + 实现了callbackOdometry，callbackTwist用于调用sendVelocity更新速度（对于CGI430不需要,去掉了）
3. io.hpp用于实现各种IO
4. CommunicationCore通过IO连接华测RTK，处理收到的NMEA消息，调用MessageHandler发布出去
5. NMEA消息解析

## 注意
1. 华测CGI430是通过CAN总线接收轮速和速度信息的，在网页配置页面中进行配置，无需通过程序监听轮速进行设置。见文档《CHC®+CGI-430厘米级组合导航系统用户手册-20230830.pdf》(2.8.5 里程计设置)

2. 华测CGI430通信方式包含如下（见文档中《2.4 I/O 设置》），不同端口数据内容不同
 + 串口
 + CGI430作为tcp/udp客户端
 + CGI430作为TCP服务端


## 测试io.hpp
1. 测试UdpServer
 + 执行命令`ros2 run huace_gnss_driver udp_server_test`，启动Udp服务端
 + 执行命令`echo "Hello" | nc -u 127.0.0.1 8080`可以模拟客户端，向服务端发送消息，服务端发送消息也能看到

1. 测试TcpClient
 + 先在终端中执行`nc -l 8081`,会启动一个Tcp服务器
 + 执行命令`ros2 run huace_gnss_driver udp_server_test`，启动Tcp客户端
 + 在服务器的终端中输入内容，连接到它的客户端都会收到消息，客户端发消息，它也会收到

1. 测试串口
 + 安装虚拟串口`sudo apt-get install socat minicom`
 + 创建虚拟串口对:`sudo socat PTY,link=/dev/ttyV0,mode=0666 PTY,link=/dev/ttyV1,mode=0666`,这将创建两个虚拟串口 /dev/ttyV0 和 /dev/ttyV1
 + 使用 /dev/ttyV0 作为发送端，使用 /dev/ttyV1 作为接收端
 + 使用 screen、minicom 或 picocom 等终端工具连接到 /dev/ttyV1，检查接收到的数据。
 + 执行命令`ros2 run huace_gnss_driver udp_server_test`，启动串口接收端
 + 在新开一个终端中执行`sudo minicom -D /dev/ttyV0 -b 9600`启动串口发送端，输入内容后，可以在程序中看到接收的内容

## gdb调试
colcon build --packages-select huace_gnss_driver --cmake-args -DCMAKE_BUILD_TYPE=Debug

gdb --args ~/dev/autoware/install/huace_gnss_driver/lib/huace_gnss_driver/huace_gnss_node_exe --ros-args -r __node:=huace_gnss_node --params-file /home/guo/dev/autoware/install/huace_gnss_driver/share/huace_gnss_driver/config/huace.yaml

## 发送数据测试
$GPCHC,GPSWeek,GPSTime,Heading,Pitch,Roll,gyro x,gyro y,gyro z,acc x,accy,accz,Lattitude,Longitude,Altitude,Ve,Vn,Vu,V,NSV1,NSV2,Status,Age,WarningCs<CR><LF>

$GPCHCX,2277,271768.00,271.21,-1.67,-2.16,0.48,-0.14,0.11,0.0387,-0.0281,0.9986,31.15959761,121.17848221,52.17,0.01,0.00,0.03,0.01,28,23,61,0,2,2.868,3.01,3.30,0.01,0.01,0.02,0.39,0.39,1329.22,X,0.00,180.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,40,27,3512308*7F##