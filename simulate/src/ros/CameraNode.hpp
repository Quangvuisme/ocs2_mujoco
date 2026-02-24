#ifdef USE_ROS2

#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <queue>
#include <atomic>
#include "FastMujoco2RosBridge.hpp"
#include <mujoco/mujoco.h>

class CameraNode{
private:

    static constexpr int MAX_WIDTH = 4096;   // 4K width
    static constexpr int MAX_HEIGHT = 2160;  // 4K height
    static constexpr size_t MAX_RGB_SIZE = MAX_WIDTH * MAX_HEIGHT * 3;
    static constexpr size_t MAX_DEPTH_SIZE = MAX_WIDTH * MAX_HEIGHT;

    std::unique_ptr<FastMujoco2RosBridge> camera_bridge;
    std::shared_ptr<rclcpp::Node> ros_node;

    std::unique_ptr<uint8_t[]> rgb_buffer;
    std::unique_ptr<float[]> depth_buffer;

    int width, height;
    float fovy;

public:

    CameraNode(int width=1280, int height=480, float fovy=45.0f) : width(width), height(height), fovy(fovy) {

        std::cout << "Initializing camera Node with width=" << width
              << ", height=" << height << ", fovy=" << fovy << std::endl;

        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.allow_undeclared_parameters(true);
        ros_node = rclcpp::Node::make_shared("mujoco_camera_node",options);
        ros_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

        rgb_buffer = std::make_unique<uint8_t[]>(MAX_RGB_SIZE);
        depth_buffer = std::make_unique<float[]>(MAX_DEPTH_SIZE);

        std::memset(rgb_buffer.get(), 0, MAX_RGB_SIZE);
        std::memset(depth_buffer.get(), 0, MAX_DEPTH_SIZE * sizeof(float));

        camera_bridge = std::make_unique<FastMujoco2RosBridge>(
            ros_node->shared_from_this(), width, height, fovy);

        // stats_timer = ros_node->create_wall_timer(
        //     std::chrono::seconds(5),
        //     [this]() { printStats(); });
    }

    void publishCameraFrame() {
        if (!camera_bridge->TryPublishCamera(rgb_buffer.get(), depth_buffer.get())) {
            RCLCPP_WARN_THROTTLE(ros_node->get_logger(), *ros_node->get_clock(), 1000,
                               "Frame dropped - processing thread overloaded");
        }
    }

    void SetMujocoData(mjData *data, mjModel *model){
        mj_data_ = data;
        mj_model_= model;
        camera_bridge->SetMujocoData(data,model);
    }

    void checkResourcesAllocation(int frame_width, int frame_height, float frame_fovy){
        if (frame_width!=width || frame_height!=height || frame_fovy != fovy){
            width = frame_width;
            height = frame_height;
            fovy = frame_fovy;
            camera_bridge->allocateResources(frame_width, frame_height, frame_fovy);
        }
    }

    void Run(){
        std::cout << "Spinning up Mujoco ROS Node " << std::endl;
        rclcpp::spin(ros_node);
        std::cout << "Shutting down Mujoco ROS Node " << std::endl;
        camera_bridge->stopProcessing();
        rclcpp::shutdown();
    }

    uint8_t* getRgbBuffer() {
        return rgb_buffer.get();
    }

    float* getDepthBuffer() {
        return depth_buffer.get();
    }

private:
    rclcpp::TimerBase::SharedPtr stats_timer;
    mjData *mj_data_ = nullptr;
    mjModel *mj_model_ = nullptr;

    void printStats() {
        uint64_t processed, dropped;
        camera_bridge->getStats(processed, dropped);

        if (processed > 0) {
            double drop_rate = (double)dropped / (processed + dropped) * 100.0;
            RCLCPP_INFO(ros_node->get_logger(),
                       "Camera stats: %lu processed, %lu dropped (%.1f%%)",
                       processed, dropped, drop_rate);
        }
    }
};


#endif


#endif