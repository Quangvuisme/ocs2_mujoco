#ifndef FAST_MUJOCO_ROS_BRIDGE_H
#define FAST_MUJOCO_ROS_BRIDGE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <array>
#include <chrono>
#include <std_msgs/msg/u_int8.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
// Include the separated GPU point cloud processor
#include "gpu_pointcloud.hpp"

using std::placeholders::_1;

#define TOPIC_CAMERA_EXTRINSIC_INFO "client/ip_192_168_1_51/camera_666666666666/camera_ex_info"
#define TOPIC_CAMERA_COLOR_INFO "client/ip_192_168_1_51/camera_666666666666/color/camera_info"
#define TOPIC_CAMERA_COLOR_IMAGE "client/ip_192_168_1_51/camera_666666666666/color/image_raw"
#define TOPIC_CAMERA_DEPTH_INFO "client/ip_192_168_1_51/camera_666666666666/aligned_depth_to_color/camera_info"
#define TOPIC_CAMERA_DEPTH_IMAGE "client/ip_192_168_1_51/camera_666666666666/aligned_depth_to_color/image_raw"
#define TOPIC_CAMERA_POINTCLOUD "client/ip_192_168_1_51/camera_666666666666/depth/color/points"
#define TOPIC_GRIPPER_CMD "client/ip_192_168_1_51/gripper_control"

class FastMujoco2RosBridge {
private:
    int camera_body_id = -1;
    Eigen::Matrix4d camera_transform;  // 4x4 transform matrix
    bool use_camera_transform = false;
    
    // Downsampling configuration - FASTEST METHOD
    int downsample_factor = 4;  // Skip every N pixels (configurable for max speed)
    
    // =================================================================
    // LOCK-FREE RING BUFFER CONFIGURATION
    // =================================================================
    static constexpr size_t RING_SIZE = 4;

    struct alignas(64) RingSlot {  // Cache line aligned
        std::atomic<bool> ready{false};
        std::vector<uint8_t> rgb_data;
        std::vector<float> depth_data;
        int width, height;
        rclcpp::Time timestamp;

        RingSlot() = default;

        void initialize(int w, int h) {
            width = w;
            height = h;
            rgb_data.resize(w * h * 3);
            depth_data.resize(w * h);
            ready.store(false);
        }
    };

    std::array<RingSlot, RING_SIZE> ring_buffer;
    std::atomic<size_t> write_index{0};
    std::atomic<size_t> read_index{0};
    std::atomic<bool> should_stop{false};
    std::thread processing_thread;

    mjData *mj_data_ = nullptr;
    mjModel *mj_model_ = nullptr;

    // =================================================================
    // ROS PUBLISHERS
    // =================================================================
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ex_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr gripper_cmd;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // =================================================================
    // PRE-ALLOCATED PROCESSING BUFFERS
    // =================================================================
    cv::Mat color_image_mat;
    std::vector<uint16_t> depth_buffer;
    sensor_msgs::msg::Image depth_image;
    sensor_msgs::msg::CameraInfo color_info, depth_info;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    
    // Fast point cloud buffer (no GPU processor needed for downsampled)
    std::vector<Point3D> pointcloud_buffer;

    // Camera parameters
    std::array<double, 9> K, R;
    std::array<double, 12> P;
    std::vector<double> D;

    // Frame dimensions
    int frame_width, frame_height;
    float frame_fovy;

    // Performance tracking
    std::atomic<uint64_t> frames_processed{0};
    std::atomic<uint64_t> frames_dropped{0};

public:
    FastMujoco2RosBridge(rclcpp::Node::SharedPtr node, int width, int height, float fovy, int downsample = 4)
        : ros_node(node), downsample_factor(downsample) {

        // Initialize publishers
        color_pub = node->create_publisher<sensor_msgs::msg::Image>(
            TOPIC_CAMERA_COLOR_IMAGE, 1);
        depth_pub = node->create_publisher<sensor_msgs::msg::Image>(
            TOPIC_CAMERA_DEPTH_IMAGE, 1);
        ex_info_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
            TOPIC_CAMERA_EXTRINSIC_INFO, 1);
        color_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
            TOPIC_CAMERA_COLOR_INFO, 1);
        depth_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
            TOPIC_CAMERA_DEPTH_INFO, 1);
        pointcloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            TOPIC_CAMERA_POINTCLOUD, 1);
        gripper_cmd = ros_node->create_subscription<std_msgs::msg::UInt8MultiArray>(
            TOPIC_GRIPPER_CMD, 10,
            std::bind(&FastMujoco2RosBridge::GripperCallback, this, std::placeholders::_1)
        );
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
        RCLCPP_INFO(node->get_logger(), "TF broadcaster initialized");

        allocateResources(width, height, fovy);

        RCLCPP_INFO(node->get_logger(), 
                   "FastMujoco2RosBridge initialized: %dx%d with %dx downsampled point cloud", 
                   width, height, downsample_factor);
    }

    ~FastMujoco2RosBridge() {
        should_stop.store(true);
        if (processing_thread.joinable()) {
            processing_thread.join();
        }

        RCLCPP_INFO(ros_node->get_logger(),
                   "FastMujoco2RosBridge stats - Processed: %lu, Dropped: %lu",
                   frames_processed.load(), frames_dropped.load());
    }

    void allocateResources(int width, int height, float fovy){
        frame_width = width;
        frame_height = height;
        frame_fovy = fovy;

        // Stop processing
        should_stop.store(true);
        if (processing_thread.joinable()){
            processing_thread.join();
            std::cout << "Previous Processing Thread terminated . " << std::endl;
        }

        RCLCPP_INFO(ros_node->get_logger(), 
                   "FastMujoco2RosBridge Buffers allocation with : %dx%d, downsample factor: %d. The received fovy is : %f", 
                   frame_width, frame_height, downsample_factor, frame_fovy);

        // Initialize ring buffer
        for (auto& slot : ring_buffer) {
            slot.initialize(frame_width, frame_height);
        }

        // Initialize processing buffers
        initializeProcessingBuffers(frame_width, frame_height);

        // Setup camera info
        setupCameraInfo(frame_width, frame_height, frame_fovy);

        // Calculate max downsampled points and allocate buffer
        int max_downsampled_points = ((frame_width / downsample_factor) + 1) * 
                                   ((frame_height / downsample_factor) + 1);
        pointcloud_buffer.resize(max_downsampled_points);

        // Restart processing
        should_stop.store(false);
        processing_thread = std::thread(&FastMujoco2RosBridge::processingLoop, this);
    }

    void setDownsampleFactor(int factor) {
        if (factor < 1) factor = 1;
        downsample_factor = factor;
        RCLCPP_INFO(ros_node->get_logger(), "Downsample factor set to: %d", factor);
        
        // Reallocate point cloud buffer
        int max_downsampled_points = ((frame_width / downsample_factor) + 1) * 
                                   ((frame_height / downsample_factor) + 1);
        pointcloud_buffer.resize(max_downsampled_points);
    }

    void GripperCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        int hand = msg->data[0];   // Index 0: hand selection
        int value = msg->data[1];
        std::cout << "Received Gripper Command with value of: "
            << value
            << " for hand " << hand << std::endl;
        if (mj_data_)
        {
            // std::cout << "mj_data_ intialized" << std::endl;
            if (hand == 0)
            {
                std::cout << "gripper left hand" << std::endl;
                mj_data_->ctrl[27] = value;
            }
            else
            {
                std::cout << "gripper right hand" << std::endl;
                mj_data_->ctrl[28] = value;
            }
        }
        else
        {
            std::cout << "mj_data_ is null" << std::endl;
        }
    }


    void SetMujocoData(mjData *data, mjModel *model){
        mj_data_ = data;
        mj_model_= model;
    }

    // =================================================================
    // ULTRA-FAST FRAME SUBMISSION (CALLING THREAD)
    // =================================================================
    bool TryPublishCamera(uint8_t* rgb, float* depth) {
        // Get current write position
        size_t write_idx = write_index.load(std::memory_order_acquire);
        size_t next_write = (write_idx + 1) % RING_SIZE;

        // Check if slot is available
        auto& slot = ring_buffer[write_idx];
        if (slot.ready.load(std::memory_order_acquire)) {
            frames_dropped.fetch_add(1, std::memory_order_relaxed);
            return false;  // Ring buffer full, drop frame
        }

        // Fast memory copy
        std::memcpy(slot.rgb_data.data(), rgb, frame_width * frame_height * 3);
        std::memcpy(slot.depth_data.data(), depth, frame_width * frame_height * sizeof(float));

        // Set metadata
        slot.width = frame_width;
        slot.height = frame_height;
        slot.timestamp = ros_node->now();

        // Atomically publish the frame
        slot.ready.store(true, std::memory_order_release);
        write_index.store(next_write, std::memory_order_release);

        return true;
    }

    // Optional: Get performance stats
    void getStats(uint64_t& processed, uint64_t& dropped) const {
        processed = frames_processed.load();
        dropped = frames_dropped.load();
    }

    // Optional: Reset stats
    void resetStats() {
        frames_processed.store(0);
        frames_dropped.store(0);
    }

    void stopProcessing(){
        should_stop.store(true);
        if (processing_thread.joinable()) {
            processing_thread.join();
        }
    }

private:

    void initializeProcessingBuffers(int width, int height) {
        // Pre-allocate OpenCV mat
        color_image_mat = cv::Mat(height, width, CV_8UC3);

        // Pre-allocate depth processing buffer
        depth_buffer.reserve(width * height);

        // Pre-setup depth image message
        depth_image.height = height;
        depth_image.width = width;
        depth_image.step = width * 2;
        depth_image.is_bigendian = false;
        depth_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        depth_image.data.reserve(width * height * 2);
        
        // Pre-setup point cloud message
        setupPointCloudMessage(width, height);
    }

    void setupPointCloudMessage(int width, int height) {
        pointcloud_msg.header.frame_id = "camera_666666666666_color_optical_frame";
        pointcloud_msg.height = 1; // Unorganized point cloud
        pointcloud_msg.is_dense = false;
        pointcloud_msg.is_bigendian = false;
        
        // Define fields: x, y, z, rgb
        pointcloud_msg.fields.clear();
        pointcloud_msg.fields.resize(4);
        
        // Position fields
        pointcloud_msg.fields[0].name = "x";
        pointcloud_msg.fields[0].offset = 0;
        pointcloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[0].count = 1;
        
        pointcloud_msg.fields[1].name = "y";
        pointcloud_msg.fields[1].offset = 4;
        pointcloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[1].count = 1;
        
        pointcloud_msg.fields[2].name = "z";
        pointcloud_msg.fields[2].offset = 8;
        pointcloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[2].count = 1;
        
        // RGB field (packed as uint32)
        pointcloud_msg.fields[3].name = "rgb";
        pointcloud_msg.fields[3].offset = 12;
        pointcloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
        pointcloud_msg.fields[3].count = 1;
        
        pointcloud_msg.point_step = 16; // 3 floats + 1 uint32
        
        // Reserve for max possible downsampled points
        int max_points = ((width / downsample_factor) + 1) * ((height / downsample_factor) + 1);
        pointcloud_msg.data.reserve(max_points * pointcloud_msg.point_step);
    }

    void setupCameraInfo(int width, int height, float fovy) {
        double fovy_rad = fovy * (M_PI / 180.0);

        double f_y = (height / 2.0) / std::tan(fovy_rad / 2.0);
        double f_x = f_y;  // Square pixels

        double c_x = (width - 1) / 2.0;
        double c_y = (height - 1) / 2.0;

        K = {f_x,  0.0, c_x,
            0.0,  f_y, c_y,
            0.0,  0.0, 1.0};

        P = {f_x,  0.0, c_x, 0.0,
            0.0,  f_y, c_y, 0.0,
            0.0,  0.0, 1.0, 0.0};

        R = {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};

        D = {0.0, 0.0, 0.0, 0.0, 0.0};

        auto setupInfo = [&](sensor_msgs::msg::CameraInfo& info, const std::string& frame_id) {
            info.height = height;
            info.width = width;
            info.distortion_model = "plumb_bob";
            info.header.frame_id = frame_id;
            info.k = K;
            info.p = P;
            info.r = R;
            info.d = D;
        };

        setupInfo(color_info, "camera_666666666666_color_optical_frame");
        setupInfo(depth_info, "camera_666666666666_depth_optical_frame");
    }

    // =================================================================
    // BACKGROUND PROCESSING LOOP
    // =================================================================
    void processingLoop() {
        RCLCPP_INFO(ros_node->get_logger(), 
                   "Processing thread started with %dx skip-based point cloud downsampling", 
                   downsample_factor);

        while (!should_stop.load(std::memory_order_acquire)) {
            // Get current read position
            size_t read_idx = read_index.load(std::memory_order_acquire);
            auto& slot = ring_buffer[read_idx];

            // Check if frame is ready
            if (!slot.ready.load(std::memory_order_acquire)) {
                // No frame ready, sleep briefly
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }

            // Process the frame
            processFrame(slot);

            // Mark slot as available and advance
            slot.ready.store(false, std::memory_order_release);
            read_index.store((read_idx + 1) % RING_SIZE, std::memory_order_release);

            frames_processed.fetch_add(1, std::memory_order_relaxed);
        }

        RCLCPP_INFO(ros_node->get_logger(), "Processing thread stopped");
    }

    void processFrame(RingSlot& slot) {
        // === COLOR IMAGE PROCESSING ===
        processColorImage(slot);

        // === DEPTH IMAGE PROCESSING ===
        processDepthImage(slot);

        // === FAST DOWNSAMPLED POINT CLOUD PROCESSING ===
        processFastPointCloud(slot);

        // === CAMERA INFO PUBLISHING ===
        publishCameraInfo(slot.timestamp);

        publishTf(slot.timestamp);
    }

    void processColorImage(RingSlot& slot) {
        // Copy RGB data to OpenCV mat
        std::memcpy(color_image_mat.data, slot.rgb_data.data(),
                   slot.width * slot.height * 3);

        // Flip image vertically (MuJoCo to ROS coordinate conversion)
        cv::flip(color_image_mat, color_image_mat, 0);

        // Create ROS message
        std_msgs::msg::Header header;
        header.frame_id = "camera_666666666666_color_optical_frame";
        header.set__stamp(slot.timestamp);

        auto color_image = cv_bridge::CvImage(header, "rgb8", color_image_mat).toImageMsg();
        color_pub->publish(*color_image);
    }

    void processDepthImage(RingSlot& slot) {
        // Clear depth buffer
        depth_buffer.clear();

        // Process depth data with y-flip and conversion
        const float near_plane = 0.01f;
        const float far_plane = 50.0f;

        for (int i = slot.height - 1; i >= 0; i--) {
            const float* row_ptr = slot.depth_data.data() + i * slot.width;
            for (int j = 0; j < slot.width; j++) {
                float normalized_depth = row_ptr[j];
                uint16_t depth_mm = 0;

                // Handle edge cases and convert to actual depth
                if (normalized_depth >= 1.0f) {
                    depth_mm = 0;  // Invalid/infinite depth
                } else {
                    // Convert normalized depth to actual distance
                    float depth_meters = near_plane / (1.0f - normalized_depth * (1.0f - near_plane / far_plane));

                    // Convert to millimeters and clamp to uint16 range
                    depth_mm = static_cast<uint16_t>(std::min(depth_meters * 1000.0f, 65535.0f));
                }

                depth_buffer.push_back(depth_mm);
            }
        }

        // Single memcpy to final buffer
        depth_image.data.resize(depth_buffer.size() * sizeof(uint16_t));
        std::memcpy(depth_image.data.data(), depth_buffer.data(), depth_image.data.size());

        // Set header and publish
        depth_image.header.frame_id = "camera_666666666666_depth_optical_frame";
        depth_image.header.set__stamp(slot.timestamp);

        depth_pub->publish(depth_image);
    }

    void updateCameraTransform() {
        if (!mj_model_ || !mj_data_) return;
        
        if (camera_body_id < 0) {
            camera_body_id = mj_name2id(mj_model_, mjOBJ_BODY, "camera_body");
            if (camera_body_id < 0) {
                RCLCPP_WARN(ros_node->get_logger(), "Camera body 'camera_body' not found");
                use_camera_transform = false;
                return;
            }
        }
        
        // Get camera body transform from MuJoCo
        mjtNum* body_pos = mj_data_->xpos + 3 * camera_body_id;
        mjtNum* body_quat = mj_data_->xquat + 4 * camera_body_id;

        // Convert MuJoCo quaternion (w,x,y,z) to Eigen quaternion
        Eigen::Quaterniond quat(body_quat[0], body_quat[1], body_quat[2], body_quat[3]);
        // std::cout << "Quaternion: " << quat.w() << ", " << quat.x() << ", " 
        //         << quat.y() << ", " << quat.z() << std::endl;
        
        // Create optical to ROS rotation matrix (matches Python optical_2_ros_rot)
        Eigen::Matrix3d optical_2_ros_rot, calib_rot;
        optical_2_ros_rot << 0.0, 0.0,  -1.0,
                            0.0,  1.0, 0.0,
                            1.0,  0.0,  0.0;
        
        calib_rot << 1.0, 0.0,  0.0,
                    0.0,  0.0, -1.0,
                    0.0,  1.0,  0.0;
        
        // Combine rotations: optical_2_ros_rot @ body rotation
        Eigen::Matrix3d total_rotation_1 = optical_2_ros_rot * quat.toRotationMatrix();
        Eigen::Matrix3d total_rotation = total_rotation_1 * calib_rot;
        //std::cout << "Extrinsics Matrix:\n" << rotation_matrix << std::endl;

        // Create 4x4 rotation matrix
        Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
        R.block<3,3>(0,0) = total_rotation;
        // Create 4x4 translation matrix
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(0,3) = 0.13;
        T(1,3) = 0;
        T(2,3) = 0.47;

        // Combine transforms: R @ T
        camera_transform = T * R;
        // std::cout << matrix << std::endl << std::endl;
        use_camera_transform = false;
    }

    // // Batch transform multiple points (more efficient)
    // void transformPointsBatch(Point3D* points, int num_points) {
    //     if (!use_camera_transform || num_points == 0) return;
        
    //     // Create matrices for batch processing
    //     Eigen::MatrixXd camera_points(4, num_points);
        
    //     // Fill camera points matrix
    //     for (int i = 0; i < num_points; i++) {
    //         camera_points(0, i) = points[i].x;
    //         camera_points(1, i) = points[i].y;
    //         camera_points(2, i) = points[i].z;
    //         camera_points(3, i) = 1.0;
    //     }
        
    //     // Batch transform
    //     Eigen::MatrixXd world_points = camera_transform * camera_points;
        
    //     // Copy back to points array
    //     for (int i = 0; i < num_points; i++) {
    //         points[i].x = world_points(0, i);
    //         points[i].y = world_points(1, i);
    //         points[i].z = world_points(2, i);
    //         // RGB colors remain unchanged
    //     }
    // }

    // =================================================================
    // ULTRA-FAST SKIP-BASED POINT CLOUD PROCESSING
    // =================================================================
    void processFastPointCloud(RingSlot& slot) {
        updateCameraTransform();
        // Set up the layout for a 4x4 matrix
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.layout.dim.resize(2);
        
        // First dimension (rows)
        msg.layout.dim[0].label = "rows";
        msg.layout.dim[0].size = 4;
        msg.layout.dim[0].stride = 16;  // total number of elements
        
        // Second dimension (columns)
        msg.layout.dim[1].label = "cols";
        msg.layout.dim[1].size = 4;
        msg.layout.dim[1].stride = 4;   // elements per row
        
        msg.layout.data_offset = 0;
        
        // Resize data array and copy matrix elements (row-major order)
        msg.data.resize(16);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                msg.data[i * 4 + j] = camera_transform(i, j);
            }
        }
        ex_info_pub->publish(msg);
        
        // Process only selected pixels (skip-based downsampling)
        int valid_points = 0;
        const float near_plane = 0.01f;
        const float far_plane = 50.0f;
        
        // Camera intrinsics
        float fx = K[0], fy = K[4], cx = K[2], cy = K[5];
        
        // Skip-based processing for maximum speed
        for (int y = 0; y < slot.height; y += downsample_factor) {
            for (int x = 0; x < slot.width; x += downsample_factor) {
                int idx = (slot.height - 1 - y) * slot.width + x; // Y-flip for MuJoCo
                float depth_norm = slot.depth_data[idx];
                
                // Skip invalid depth
                if (depth_norm >= 1.0f) continue;
                
                // Convert to actual depth
                float depth_m = near_plane / (1.0f - depth_norm * (1.0f - near_plane / far_plane));
                if (depth_m <= 0.01f || depth_m >= far_plane) continue;
                
                // Convert to 3D point (camera coordinates)
                float px = (x - cx) * depth_m / fx;
                float py = (y - cy) * depth_m / fy;
                float pz = depth_m;
                
                // Get RGB color
                int rgb_idx = idx * 3;
                uint8_t r = slot.rgb_data[rgb_idx];
                uint8_t g = slot.rgb_data[rgb_idx + 1];
                uint8_t b = slot.rgb_data[rgb_idx + 2];
                
                // Store point
                if (valid_points < pointcloud_buffer.size()) {
                    pointcloud_buffer[valid_points] = {px, py, pz, r, g, b};
                    valid_points++;
                }
            }
        }
        
        if (valid_points == 0) return;

        // Apply camera transform if needed
        //transformPointsBatch(pointcloud_buffer.data(), valid_points);

        // Convert to ROS PointCloud2 format
        pointcloud_msg.header.set__stamp(slot.timestamp);
        pointcloud_msg.width = valid_points;
        pointcloud_msg.row_step = valid_points * pointcloud_msg.point_step;
        
        // Resize data buffer
        pointcloud_msg.data.resize(valid_points * pointcloud_msg.point_step);
        
        // Fill point cloud data
        uint8_t* data_ptr = pointcloud_msg.data.data();
        for (int i = 0; i < valid_points; i++) {
            const Point3D& point = pointcloud_buffer[i];
            
            // Copy position (x, y, z)
            std::memcpy(data_ptr, &point.x, sizeof(float));
            std::memcpy(data_ptr + 4, &point.y, sizeof(float));
            std::memcpy(data_ptr + 8, &point.z, sizeof(float));
            
            // Pack RGB into uint32 (0x00RRGGBB format)
            uint32_t rgb_packed = (static_cast<uint32_t>(point.r) << 16) |
                                 (static_cast<uint32_t>(point.g) << 8) |
                                 static_cast<uint32_t>(point.b);
            std::memcpy(data_ptr + 12, &rgb_packed, sizeof(uint32_t));
            
            data_ptr += pointcloud_msg.point_step;
        }

        // Publish point cloud
        pointcloud_pub->publish(pointcloud_msg);
    }

    void publishCameraInfo(rclcpp::Time timestamp) {
        // Update timestamps and publish
        color_info.header.set__stamp(timestamp);
        depth_info.header.set__stamp(timestamp);

        color_info_pub->publish(color_info);
        depth_info_pub->publish(depth_info);
    }

    void publishTf(rclcpp::Time timestamp) {
        // Add this debug line first to see if function is being called
        //RCLCPP_INFO(ros_node->get_logger(), "publishTf called!");
        
        // Uncomment and check the validation
        // if (!use_camera_transform || !mj_model_ || !mj_data_) {
        //     RCLCPP_WARN(ros_node->get_logger(), 
        //             "Cannot publish TF: use_camera_transform=%d, mj_model_=%p, mj_data_=%p", 
        //             use_camera_transform, (void*)mj_model_, (void*)mj_data_);
        //     return;
        // }

        //RCLCPP_INFO(ros_node->get_logger(), "About to create transform message");

        geometry_msgs::msg::TransformStamped transform_stamped;
        
        // Use the actual camera transform instead of zeros
        transform_stamped.transform.translation.x = camera_transform(0, 3);
        transform_stamped.transform.translation.y = camera_transform(1, 3);
        transform_stamped.transform.translation.z = camera_transform(2, 3);
        
        // Extract rotation matrix (3x3) from the 4x4 matrix
        Eigen::Matrix3d rotation_matrix = camera_transform.block<3,3>(0,0);
        
        // Convert rotation matrix to quaternion
        Eigen::Quaterniond eigen_quat(rotation_matrix);
        transform_stamped.transform.rotation.x = eigen_quat.x();
        transform_stamped.transform.rotation.y = eigen_quat.y();
        transform_stamped.transform.rotation.z = eigen_quat.z();
        transform_stamped.transform.rotation.w = eigen_quat.w();
        
        // Set frame information
        transform_stamped.header.stamp = timestamp;
        transform_stamped.header.frame_id = "torso_link";
        transform_stamped.child_frame_id = "camera_666666666666_color_optical_frame";
        
        // RCLCPP_INFO(ros_node->get_logger(), 
        //         "Publishing TF: [%.3f, %.3f, %.3f] quat[%.3f, %.3f, %.3f, %.3f]",
        //         transform_stamped.transform.translation.x,
        //         transform_stamped.transform.translation.y,
        //         transform_stamped.transform.translation.z,
        //         transform_stamped.transform.rotation.x,
        //         transform_stamped.transform.rotation.y,
        //         transform_stamped.transform.rotation.z,
        //         transform_stamped.transform.rotation.w);
        
        // Publish the transform
        tf_broadcaster_->sendTransform(transform_stamped);
        
        // RCLCPP_INFO(ros_node->get_logger(), "TF published successfully!");
    }
};

#endif