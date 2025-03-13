#include <rclcpp/rclcpp.hpp>

class TestNode : public rclcpp::Node
{
public:
    TestNode() : Node("test_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS2!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}
