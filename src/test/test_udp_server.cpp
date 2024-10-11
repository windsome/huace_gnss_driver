#include <huace_gnss_driver/io.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

/**
 * @file main.cpp
 * @date 01/12/21
 * @brief Main function of the Huace driver:
 */

int receive_callback(uint8_t* buff, int size) {
    std::cerr << "buff: " << size << std::endl;
    return 0;
}

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), // 每秒执行一次
            [this]() {
                RCLCPP_INFO(this->get_logger(), "MyNode is running...");
            });
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<boost::asio::io_context> ctx;

void signal_handler(int signum) {
    if (ctx) {
        ctx->stop();  // 停止 io_context
    }
    rclcpp::shutdown(); // 关闭 ROS 2
    // exit(signum);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto rx_node = std::make_shared<MyNode>();

    std::signal(SIGINT, signal_handler);
    
    try {
        ctx = std::make_shared<boost::asio::io_context>();
        io::UdpServer server(ctx, 8080);  // 服务器监听8080端口
        server.receive(receive_callback);

        io::TcpClient tcpClient(ctx, "127.0.0.1", 8081);
        tcpClient.receive(receive_callback);

        io::SerialIo serial(ctx, "/dev/ttyV1", 9600);
        serial.receive(receive_callback);

        std::thread t([ctx]() {
            try {
                ctx->run(); // 运行 io_context
            } catch (const std::exception& e) {
                std::cerr << "Thread Error: " << e.what() << std::endl;
            }
        });
        t.detach(); // 将线程分离，继续后台运行
        rclcpp::spin(rx_node);
        // rclcpp::shutdown();
        // ctx->run();
        RCLCPP_INFO(rx_node->get_logger(), "finish normal");
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    RCLCPP_INFO(rx_node->get_logger(), "exit");
    return 0;
}