#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <iostream>
#include <limits>
#include <thread>
#include <mutex>
#include <atomic>

#define ADDR_PRO_TORQUE_ENABLE        64
#define ADDR_PRO_GOAL_CURRENT         102
#define ADDR_PRO_PRESENT_CURRENT      126
#define ADDR_PRO_OPERATING_MODE       11

#define PROTOCOL_VERSION              2.0

#define DXL_ID                        1
#define BAUDRATE                      1000000
#define DEVICENAME                    "/dev/ttyUSB0"

#define CURRENT_CONVERSION_FACTOR    3.36 //from dynamixel wizard, why?

class DynamixelController : public rclcpp::Node
{
public:
    DynamixelController() : Node("dynamixel_controller"), desired_current_mA_(0.0), shutdown_requested_(false)
    {
        port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (port_handler_->openPort()) {
        } else {
            rclcpp::shutdown();
        }

        if (port_handler_->setBaudRate(BAUDRATE)) {
        } else {
            rclcpp::shutdown();
        }

        uint8_t operating_mode = 0;
        uint8_t ret = 0;
        int dxl_ret = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_OPERATING_MODE, operating_mode, &ret);
        if (dxl_ret != COMM_SUCCESS) {
            rclcpp::shutdown();
        }

        uint8_t torque_enable = 1;
        dxl_ret = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_TORQUE_ENABLE, torque_enable, &ret);
        if (dxl_ret != COMM_SUCCESS) {
            rclcpp::shutdown();
        }

        update_goal_current();

        input_thread_ = std::thread(&DynamixelController::input_loop, this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&DynamixelController::check_current, this)
        );
    }

    ~DynamixelController()
    {
        shutdown_requested_ = true; //gracefull shutdown of input thread doesnt really work, still...

        if (input_thread_.joinable()) {
            input_thread_.join();
        }

        uint8_t torque_enable = 0;
        uint8_t ret = 0;
        int dxl_ret = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_TORQUE_ENABLE, torque_enable, &ret);
        if (dxl_ret != COMM_SUCCESS) {
        }
    }

private:
    void check_current()
    {
        int16_t present_current = 0;
        uint8_t ret = 0;
        int dxl_ret = packet_handler_->read2ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&present_current, &ret);
        if (dxl_ret != COMM_SUCCESS) {
            rclcpp::shutdown();
        }

        float current_mA = present_current * CURRENT_CONVERSION_FACTOR;
        //RCLCPP_INFO(this->get_logger(), "Current: %.2f ", current_mA);
    }

    void input_loop()
    {
        while (!shutdown_requested_) {
            std::cout << "Enter the desired current (in mA)! Positvie values go in CW and negative in CCW: ";
            float new_current;
            std::cin >> new_current;

            while (std::cin.fail()) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Invalid input. Try again: ";
                std::cin >> new_current;
            }

            {
                std::lock_guard<std::mutex> lock(current_mutex_); //do we really need a mutex here?
                desired_current_mA_ = new_current;
            }
            update_goal_current();
        }
    }

    void update_goal_current()
    {
        int goal_current_raw = static_cast<int>(desired_current_mA_ / CURRENT_CONVERSION_FACTOR); //do we really need to scale values?

        uint8_t ret = 0;
        int dxl_ret = packet_handler_->write2ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_GOAL_CURRENT, goal_current_raw, &ret);
        if (dxl_ret != COMM_SUCCESS) {
        }
    }

    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    std::mutex current_mutex_;
    std::atomic<bool> shutdown_requested_;
    float desired_current_mA_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto dynamixel_controller = std::make_shared<DynamixelController>();
    exec.add_node(dynamixel_controller);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
