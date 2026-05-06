#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <csignal>
#include <cstring>

// --- CONSTANTS MATCHING RECEIVER ---
const int WIDTH = 1280;
const int HEIGHT = 720;
const int CHANNELS = 3;
const size_t IMG_BYTES = WIDTH * HEIGHT * CHANNELS;
const size_t DEPTH_BYTES = WIDTH * HEIGHT * sizeof(float);
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

struct DiskTask {
    uint64_t relative_id;
    uint64_t ts;
    double joints[18];
    char* data; // Changed to raw pointer for efficiency

    DiskTask() : data(new char[CAM_SET_SIZE]) {}
    ~DiskTask() { delete[] data; }
};

std::atomic<bool> keep_running(true);
void signal_handler(int signal) { keep_running = false; }

class AsyncWriter {
private:
    std::ofstream file;
    std::queue<DiskTask*> queue;
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> running{true};
    std::thread worker;

    void writeLoop() {
        while (running || !queue.empty()) {
            DiskTask* task = nullptr;
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, [this] { return !queue.empty() || !running; });
                if (queue.empty() && !running) break;
                if (queue.empty()) continue;
                task = queue.front();
                queue.pop();
            }

            if (task) {
                // Write Header Metadata
                file.write(reinterpret_cast<char*>(&task->relative_id), sizeof(uint64_t));
                file.write(reinterpret_cast<char*>(&task->ts), sizeof(uint64_t));
                file.write(reinterpret_cast<char*>(task->joints), 18 * sizeof(double));
                
                // Write 3x RGB + 3x Float Depth
                file.write(task->data, CAM_SET_SIZE);
                
                delete task;
            }
        }
    }

public:
    AsyncWriter(std::string path) {
        file.open(path, std::ios::binary | std::ios::out);
        // Optimized for NVMe: Large internal fstream buffer
        std::vector<char> internal_buf(128 * 1024 * 1024); 
        file.rdbuf()->pubsetbuf(internal_buf.data(), internal_buf.size());
        worker = std::thread(&AsyncWriter::writeLoop, this);
    }

    void push(uint64_t rel_id, uint64_t ts, const double* joints_ptr, const char* src) {
        DiskTask* t = new DiskTask(); // Note: Constructor allocates CAM_SET_SIZE
        t->relative_id = rel_id;
        t->ts = ts;
        std::memcpy(t->joints, joints_ptr, 18 * sizeof(double));
        std::memcpy(t->data, src, CAM_SET_SIZE);
        
        {
            std::lock_guard<std::mutex> lock(mtx);
            queue.push(t);
        }
        cv.notify_one();
    }

    ~AsyncWriter() {
        running = false;
        cv.notify_all();
        if (worker.joinable()) worker.join();
        file.close();
    }
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    
    using namespace boost::interprocess;
    
    // Open SHM
    shared_memory_object shm(open_only, "zed_shm", read_only);
    mapped_region region(shm, read_only);
    
    SHMHeader* header = static_cast<SHMHeader*>(region.get_address());
    const char* data_ptr = static_cast<const char*>(region.get_address()) + sizeof(SHMHeader);

    std::string filename = "dataset_capture.bin";
    if (argc > 1) filename = argv[1];

    std::cout << ">>> Receiver/Recorder Online | 9950X Performance Mode" << std::endl;
    std::cout << ">>> Target: " << filename << std::endl;

    AsyncWriter writer(filename);
    
    uint64_t start_frame_id = header->frame_count;
    uint64_t last_recorded_id = start_frame_id;

    while (keep_running) {
        uint64_t current_global_id = header->frame_count;

        if (current_global_id > last_recorded_id) {
            auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
            uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(now).count();

            uint64_t relative_id = current_global_id - start_frame_id;

            // Access the stable buffer (the one NOT being written by the receiver)
            int stable_idx = (header->active_buffer - 1 + 3) % 3;
            const char* src = data_ptr + (stable_idx * CAM_SET_SIZE);

            writer.push(relative_id, ts, header->joints, src);
            
            last_recorded_id = current_global_id;

            if (relative_id % 30 == 0) {
                std::cout << "\rRecorded Frame: " << relative_id << " (Queue: " << "...) " << std::flush;
            }
        }
        // Tight poll for 60FPS (~16ms). Sleep slightly less to avoid missing pulses.
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::cout << "\n>>> Signal received. Flushing queue to disk..." << std::endl;
    return 0;
}