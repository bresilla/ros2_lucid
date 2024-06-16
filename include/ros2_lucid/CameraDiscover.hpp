#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ros2_lucid/msg/camera_device.hpp>
#include <ros2_lucid/msg/camera_device_array.hpp>
#include <ros2_lucid/srv/ask_camera.hpp>
#include <ArenaApi.h>


class CameraArray : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr publish_camera_discovered_timer;
    ros2_lucid::msg::CameraDeviceArray::SharedPtr camera_discovered;
    rclcpp::Publisher<ros2_lucid::msg::CameraDeviceArray>::SharedPtr array_pub_;
    rclcpp::Service<ros2_lucid::srv::AskCamera>::SharedPtr service;
    public:
        CameraArray(ros2_lucid::msg::CameraDeviceArray::SharedPtr camera_discovered);
    private:
        void publish_camera_discovered();
        void service_trigger(const std::shared_ptr<ros2_lucid::srv::AskCamera::Request> request, std::shared_ptr<ros2_lucid::srv::AskCamera::Response> response);
};

class CameraInfo : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr update_camera_discovered_timer;
    ros2_lucid::msg::CameraDeviceArray::SharedPtr camera_discovered;
    public:
        CameraInfo(ros2_lucid::msg::CameraDeviceArray::SharedPtr camera_discovered);
    private:
        void update_camera_discovered();
};