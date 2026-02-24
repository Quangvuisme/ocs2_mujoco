// gpu_pointcloud.cu
#include "gpu_pointcloud.hpp"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <vector>
#include <cstdint>

// CUDA kernel for RGB-D to point cloud conversion
__global__ void rgbdToPointCloudKernel(
    const float* depth,
    const std::uint8_t* rgb,
    Point3D* points,
    std::uint8_t* valid,  // Changed to uint8_t
    const CameraIntrinsics intrinsics,
    int width,
    int height,
    float near_plane = 0.01f,
    float far_plane = 50.0f
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    int idx = y * width + x;
    float normalized_depth = depth[idx];
    
    // Handle invalid depth values
    if (normalized_depth >= 1.0f || normalized_depth <= 0.0f) {
        valid[idx] = 0;
        return;
    }
    
    // Convert normalized depth to actual distance (same as your depth processing)
    float z = near_plane / (1.0f - normalized_depth * (1.0f - near_plane / far_plane));
    
    // Skip points too far away
    if (z > 10.0f) {
        valid[idx] = 0;
        return;
    }
    
    // Back-project to 3D (flipped y coordinate for MuJoCo)
    float x_3d = (x - intrinsics.cx) * z / intrinsics.fx;
    float y_3d = -((y - intrinsics.cy) * z / intrinsics.fy); // Flip Y for MuJoCo
    
    // Store point
    points[idx].x = x_3d;
    points[idx].y = y_3d;
    points[idx].z = z;
    
    // Store RGB (assuming RGB interleaved)
    int rgb_idx = idx * 3;
    points[idx].r = rgb[rgb_idx];
    points[idx].g = rgb[rgb_idx + 1];
    points[idx].b = rgb[rgb_idx + 2];
    
    valid[idx] = 1;
}

// GPU Point Cloud Processor Implementation
GPUPointCloudProcessor::GPUPointCloudProcessor(int w, int h, const CameraIntrinsics& cam) 
    : width(w), height(h), total_pixels(w * h), intrinsics(cam) {
    
    // Allocate GPU memory
    cudaMalloc(&d_depth, total_pixels * sizeof(float));
    cudaMalloc(&d_rgb, total_pixels * 3 * sizeof(std::uint8_t));
    cudaMalloc(&d_points, total_pixels * sizeof(Point3D));
    cudaMalloc(&d_valid, total_pixels * sizeof(std::uint8_t));  // Changed to uint8_t
    
    // Create CUDA stream
    cudaStreamCreate(&stream);
}

GPUPointCloudProcessor::~GPUPointCloudProcessor() {
    cudaFree(d_depth);
    cudaFree(d_rgb);
    cudaFree(d_points);
    cudaFree(d_valid);
    cudaStreamDestroy(stream);
}

int GPUPointCloudProcessor::processPointCloud(const float* depth_data, const std::uint8_t* rgb_data, Point3D* output_points) {
    // Copy data to GPU
    cudaMemcpyAsync(d_depth, depth_data, total_pixels * sizeof(float), 
                   cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(d_rgb, rgb_data, total_pixels * 3 * sizeof(std::uint8_t), 
                   cudaMemcpyHostToDevice, stream);
    
    // Launch kernel
    dim3 blockSize(16, 16);
    dim3 gridSize((width + blockSize.x - 1) / blockSize.x,
                 (height + blockSize.y - 1) / blockSize.y);
    
    rgbdToPointCloudKernel<<<gridSize, blockSize, 0, stream>>>(
        d_depth, d_rgb, d_points, d_valid, intrinsics, width, height
    );
    
    // Wait for kernel completion
    cudaStreamSynchronize(stream);
    
    // Copy results back and compact on CPU
    std::vector<std::uint8_t> valid_flags(total_pixels);
    std::vector<Point3D> all_points(total_pixels);
    
    cudaMemcpy(valid_flags.data(), d_valid, total_pixels * sizeof(std::uint8_t), cudaMemcpyDeviceToHost);
    cudaMemcpy(all_points.data(), d_points, total_pixels * sizeof(Point3D), cudaMemcpyDeviceToHost);
    
    // Compact on CPU
    int valid_count = 0;
    for (int i = 0; i < total_pixels; i++) {
        if (valid_flags[i]) {
            output_points[valid_count++] = all_points[i];
        }
    }
    
    return valid_count;
}