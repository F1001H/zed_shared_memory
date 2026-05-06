#include <sl/Camera.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <thread>
#include <vector>
#include <iostream>
#include <cstring>
#include <csignal>
#include <atomic>

// --- MATCHING CONSTANTS ---
const int WIDTH = 1280;   
const int HEIGHT = 720;   
const int CHANNELS = 3;   // BGR
const size_t IMG_BYTES = WIDTH * HEIGHT * CHANNELS; 
const size_t DEPTH_BYTES = WIDTH * HEIGHT * sizeof(float); // Back to Float32
const size_t SINGLE_CAM_SIZE = IMG_BYTES + DEPTH_BYTES;
const size_t CAM_SET_SIZE = (IMG_BYTES * 3) + (DEPTH_BYTES * 3); 

#pragma pack(push, 1)
struct SHMHeader {
    uint64_t frame_count;
    int32_t active_buffer; 
    int32_t padding;
    double joints[18];
    uint64_t joint_timestamp;
};
#pragma pack(pop)

std::atomic<bool> exit_program(false);
std::atomic<int> cams_finished(0);

void signal_handler(int signal) { exit_program = true; }

class CameraWorker {
public:
    void run(std::string ip, int port, int cam_id, uint8_t* data_ptr, SHMHeader* header) {
        sl::Camera zed;
        sl::InitParameters init_params;
        init_params.input.setFromStream(sl::String(ip.c_str()), port);
        init_params.camera_resolution = sl::RESOLUTION::HD720;
        init_params.depth_mode = sl::DEPTH_MODE::NEURAL;

        if (zed.open(init_params) != sl::ERROR_CODE::SUCCESS) return;

        sl::Mat rgb_map, depth_map;
        
        while (!exit_program) {
            if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
                int current_slot = header->active_buffer;

                // Force resolution and CPU memory to prevent padding/stride issues
                zed.retrieveImage(rgb_map, sl::VIEW::LEFT, sl::MEM::CPU, sl::Resolution(WIDTH, HEIGHT));
                zed.retrieveMeasure(depth_map, sl::MEASURE::DEPTH, sl::MEM::CPU, sl::Resolution(WIDTH, HEIGHT));

                // Calculate precise offsets
                uint8_t* slot_base = data_ptr + (current_slot * CAM_SET_SIZE);
                uint8_t* rgb_dst = slot_base + (cam_id * IMG_BYTES);
                uint8_t* depth_dst = slot_base + (3 * IMG_BYTES) + (cam_id * DEPTH_BYTES);

                // 1. Manual BGRA -> BGR (Stripping Alpha and flipping R/B for OpenCV)
                sl::uchar4* src_ptr = (sl::uchar4*)rgb_map.getPtr<sl::uchar1>();
                for (int i = 0; i < WIDTH * HEIGHT; ++i) {
                    rgb_dst[i*3 + 0] = src_ptr[i].z; // B
                    rgb_dst[i*3 + 1] = src_ptr[i].y; // G
                    rgb_dst[i*3 + 2] = src_ptr[i].x; // R
                }

                // 2. Direct Float32 Copy (Max Granularity)
                std::memcpy(depth_dst, depth_map.getPtr<sl::uchar1>(), DEPTH_BYTES);

                // 3. Sync Barrier
                if (++cams_finished == 3) {
                    header->active_buffer = (current_slot + 1) % 3;
                    header->frame_count++;
                    cams_finished = 0;
                } else {
                    while (cams_finished != 0 && !exit_program) std::this_thread::yield();
                }
            }
        }
        zed.close();
    }
};

int main() {
    std::signal(SIGINT, signal_handler);
    size_t total_shm = sizeof(SHMHeader) + (3 * CAM_SET_SIZE);
    
    boost::interprocess::shared_memory_object::remove("zed_shm");
    boost::interprocess::shared_memory_object shm(boost::interprocess::create_only, "zed_shm", boost::interprocess::read_write);
    shm.truncate(total_shm);
    boost::interprocess::mapped_region region(shm, boost::interprocess::read_write);

    SHMHeader* header = static_cast<SHMHeader*>(region.get_address());
    uint8_t* data = static_cast<uint8_t*>(region.get_address()) + sizeof(SHMHeader);
    std::memset(header, 0, sizeof(SHMHeader));

    std::vector<std::thread> workers;
    for(int i=0; i<3; ++i) 
        workers.emplace_back([&, i, data, header]() { CameraWorker().run("192.168.1.2", 30000 + (i*2), i, data, header); });

    while(!exit_program) std::this_thread::sleep_for(std::chrono::seconds(1));
    for(auto& t : workers) t.join();
    return 0;
}