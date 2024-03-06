#include "xv_ros2_node.h"
using namespace xv;
using namespace std::chrono_literals;

void signalHandler(int signum) {}
int main(int argc, char **argv) 
{
    signal(SIGSEGV, signalHandler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<xvision_ros2_node>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
