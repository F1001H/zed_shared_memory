#include <sl/Camera.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <thread>
#include <vector>
#include <iostream>
#include <cstring>
#include <csignal>
#include <atomic>
#include <chrono>

// --- CONFIGURATION ---
const int WIDTH = 1920;
const int HEIGHT = 1080;
const int CHANNELS = 4; // RGBA
const size_t IMG_BYTES = WIDTH * HEIGHT * CHANNELS;
const size_t DEPTH_BYTES = WIDTH * HEIGHT * sizeof(float);
const size_t SINGLE_CAM_SIZE = IMG_BYTES + DEPTH_BYTES;
const size_t CAM_SET_SIZE = SINGLE_CAM_SIZE * 3; 

// --- SHARED STRUCTURE ---
#pragma pack(push, 1)
struct SHMHeader {
    uint64_t frame_count;
    int32_t active_buffer; 
    int32_t padding;
    double joints[18];      // Placeholder for your ROS relay
    uint64_t joint_timestamp;
};
#pragma pack(pop)

std::atomic<bool> exit_program(false);
std::atomic<int> cams_finished(0); // The Barrier Sync

void signal_handler(int signal) { exit_program = true; }

using namespace sl;

class CameraWorker {
public:
    void run(std::string ip, int port, int cam_id, uint8_t* data_ptr, SHMHeader* header) {
        Camera zed;
        InitParameters init_params;
        init_params.input.setFromStream(String(ip.c_str()), port);
        init_params.camera_resolution = RESOLUTION::HD1080;
        init_params.depth_mode = DEPTH_MODE::NEURAL;
        init_params.sdk_verbose = false;

        if (zed.open(init_params) != ERROR_CODE::SUCCESS) {
            std::cerr << "[Cam " << cam_id << "] Failed to connect to " << ip << ":" << port << std::endl;
            return;
        }

        Mat rgb, depth;
        while (!exit_program) {
            // 1. Grab the frame from the Jetson stream
            if (zed.grab() == ERROR_CODE::SUCCESS) {
                // Determine which slot the system is currently filling
                int current_slot = header->active_buffer;

                zed.retrieveImage(rgb, VIEW::LEFT);
                zed.retrieveMeasure(depth, MEASURE::DEPTH);

                // Calculate memory offsets
                uint8_t* slot_base = data_ptr + (current_slot * CAM_SET_SIZE);
                uint8_t* rgb_target = slot_base + (cam_id * IMG_BYTES);
                uint8_t* depth_target = slot_base + (3 * IMG_BYTES) + (cam_id * DEPTH_BYTES);

                // Copy data to SHM
                std::memcpy(rgb_target, rgb.getPtr<sl::uchar1>(), IMG_BYTES);
                std::memcpy(depth_target, depth.getPtr<sl::float1>(), DEPTH_BYTES);

                // 2. BARRIER SYNC
                // Increment counter to signal this camera is done with this slot
                int count = ++cams_finished;

                if (count == 3) {
                    // This was the last camera to finish. 
                    // Update global pointers and reset barrier.
                    header->active_buffer = (header->active_buffer + 1) % 3;
                    header->frame_count++;
                    cams_finished = 0;
                } else {
                    // Wait for the other threads to catch up so we don't 
                    // jump into the next grab cycle prematurely.
                    while (cams_finished != 0 && !exit_program) {
                        std::this_thread::yield();
                    }
                }
            }
        }
        zed.close();
    }
};

int main() {
    std::signal(SIGINT, signal_handler);
    std::string jetson_ip = "192.168.1.2"; 
    
    using namespace boost::interprocess;
    size_t total_shm = sizeof(SHMHeader) + (3 * CAM_SET_SIZE);

    try {
        shared_memory_object::remove("zed_shm");
        shared_memory_object shm(create_only, "zed_shm", read_write);
        shm.truncate(total_shm);
        mapped_region region(shm, read_write);

        SHMHeader* header = static_cast<SHMHeader*>(region.get_address());
        uint8_t* data = static_cast<uint8_t*>(region.get_address()) + sizeof(SHMHeader);
        
        std::memset(header, 0, sizeof(SHMHeader));
        header->active_buffer = 0;

        std::cout << ">>> Receiver Online | 3 Cameras | Triple Buffered" << std::endl;
        std::cout << ">>> Buffer Size: " << total_shm / (1024*1024) << " MB" << std::endl;

        std::vector<std::thread> workers;
        for(int i=0; i<3; ++i) {
            workers.emplace_back([&, i, jetson_ip, data, header]() {
                CameraWorker().run(jetson_ip, 30000 + (2*i), i, data, header);
            });
        }

        while(!exit_program) {
            static uint64_t last_f = 0;
            if (header->frame_count != last_f) {
                last_f = header->frame_count;
                if (last_f % 30 == 0) 
                    std::cout << "Syncing at " << last_f << " total frames..." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        for(auto& t : workers) if(t.joinable()) t.join();
        shared_memory_object::remove("zed_shm");

    } catch (const std::exception& e) {
        std::cerr << "SHM Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}