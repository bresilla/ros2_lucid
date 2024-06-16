#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <ros2_lucid/srv/trigger_camera.hpp>
#include <ros2_lucid/srv/ask_camera.hpp>
#include <ros2_lucid/msg/camera_device.hpp>
#include <ros2_lucid/msg/camera_device_array.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

#include <ArenaApi.h>



class CameraManager : public rclcpp::Node {
    private:
        typedef std::pair<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>> cstate;
        typedef std::pair<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> cchange;
        
        rclcpp::Subscription<ros2_lucid::msg::CameraDeviceArray>::SharedPtr camera_info_sub;
        rclcpp::TimerBase::SharedPtr camera_check_timer;
        rclcpp::Client<ros2_lucid::srv::AskCamera>::SharedPtr camera_get;

        ros2_lucid::msg::CameraDeviceArray::SharedPtr cam_discovered;
        std::vector<ros2_lucid::msg::CameraDevice> camera_devices;

        std::map<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>> camera_get_state;
        std::map<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> camera_change_state;

        bool has_cam_discovered;

    public:
        CameraManager();
        ~CameraManager();

    private:
        void cam_info_update(const ros2_lucid::msg::CameraDeviceArray::SharedPtr msg);
        void cam_check_update();
        void state_controller(const ros2_lucid::msg::CameraDevice, lifecycle_msgs::msg::State curr_state, bool runnning);
        template <typename FutureT, typename WaitTimeT>
        
        std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait);
        unsigned int get_state(ros2_lucid::msg::CameraDevice camera_node, std::chrono::seconds timeout);
        bool change_state(ros2_lucid::msg::CameraDevice camera_node, std::uint8_t transition, std::chrono::seconds timeout);
};