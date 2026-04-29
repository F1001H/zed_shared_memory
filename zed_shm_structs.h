#ifndef ZED_SHM_STRUCTS_H
#define ZED_SHM_STRUCTS_H

#include <cstdint>

// --- CONSTANTS ---
const int WIDTH = 1920;
const int HEIGHT = 1080;
const int CHANNELS = 4; // RGBA
const size_t IMG_BYTES = (size_t)WIDTH * HEIGHT * CHANNELS;
const size_t DEPTH_BYTES = (size_t)WIDTH * HEIGHT * sizeof(float);
const size_t SINGLE_CAM_SIZE = IMG_BYTES + DEPTH_BYTES;
const size_t CAM_SET_SIZE = SINGLE_CAM_SIZE * 3; // 3 Cameras

// --- SHARED STRUCTURE ---
#pragma pack(push, 1)
struct SHMHeader {
    uint64_t frame_count;
    int32_t active_buffer; 
    int32_t padding;
    double joints[18];      // Franka 7-DOF * 2
    uint64_t joint_timestamp;
};
#pragma pack(pop)

#endif