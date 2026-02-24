// gpu_pointcloud.hpp
#ifndef GPU_POINTCLOUD_HPP
#define GPU_POINTCLOUD_HPP

#include <cstdint>
#include <vector>
#include <memory>

// GPU Point Cloud structures
struct Point3D {
    float x, y, z;
    std::uint8_t r, g, b;
};

struct CameraIntrinsics {
    float fx, fy, cx, cy;
};

// CPU-only point cloud processing (fallback)
class CPUPointCloudProcessor {
public:
    CPUPointCloudProcessor(int w, int h, const CameraIntrinsics& cam) 
        : width(w), height(h), intrinsics(cam) {}
    
    int processPointCloud(const float* depth_data, const std::uint8_t* rgb_data, Point3D* output_points) {
        int valid_count = 0;
        const float near_plane = 0.01f;
        const float far_plane = 50.0f;
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                float normalized_depth = depth_data[idx];
                
                if (normalized_depth >= 1.0f || normalized_depth <= 0.0f) continue;
                
                float z = near_plane / (1.0f - normalized_depth * (1.0f - near_plane / far_plane));
                if (z > 10.0f) continue;
                
                float x_3d = (x - intrinsics.cx) * z / intrinsics.fx;
                float y_3d = -((y - intrinsics.cy) * z / intrinsics.fy);
                
                output_points[valid_count].x = x_3d;
                output_points[valid_count].y = y_3d;
                output_points[valid_count].z = z;
                output_points[valid_count].r = rgb_data[idx * 3];
                output_points[valid_count].g = rgb_data[idx * 3 + 1];
                output_points[valid_count].b = rgb_data[idx * 3 + 2];
                valid_count++;
            }
        }
        return valid_count;
    }
    
private:
    int width, height;
    CameraIntrinsics intrinsics;
};

#ifdef CUDA_ENABLED
#include <cuda_runtime.h>

// GPU Point Cloud Processor (CUDA implementation)
class GPUPointCloudProcessor {
private:
    float* d_depth;
    std::uint8_t* d_rgb;
    Point3D* d_points;
    std::uint8_t* d_valid;  // Use uint8_t instead of bool to avoid std::vector<bool> issues
    
    int width, height, total_pixels;
    CameraIntrinsics intrinsics;
    cudaStream_t stream;

public:
    GPUPointCloudProcessor(int w, int h, const CameraIntrinsics& cam);
    ~GPUPointCloudProcessor();
    int processPointCloud(const float* depth_data, const std::uint8_t* rgb_data, Point3D* output_points);
};

// Use GPU processor when CUDA is available
using PointCloudProcessor = GPUPointCloudProcessor;
#else
// Use CPU processor when CUDA is not available
using PointCloudProcessor = CPUPointCloudProcessor;
#endif

#endif // GPU_POINTCLOUD_HPP