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

#include "zed_shm_structs.h" 

// --- UPDATED SCHEMA ---
// Each block: [uint64 RelativeID][uint64 TS][14*double Joints][Raw Pixels]
struct DiskTask {
    uint64_t relative_id;
    uint64_t ts;
    double joints[18];
    std::vector<char> data; 
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
                if (queue.empty()) continue;
                task = queue.front();
                queue.pop();
            }

            if (task) {
                // Write Header: ID(8) + TS(8) + Joints(112) = 128 bytes
                file.write(reinterpret_cast<char*>(&task->relative_id), 8);
                file.write(reinterpret_cast<char*>(&task->ts), 8);
                file.write(reinterpret_cast<char*>(task->joints), 18 * sizeof(double));
                
                // Write Payload: Pixels
                file.write(task->data.data(), task->data.size());
                
                delete task;
            }
        }
    }

public:
    AsyncWriter(std::string path) {
        file.open(path, std::ios::binary | std::ios::out);
        std::vector<char> buf(64 * 1024 * 1024); // 64MB buffer for NVMe
        file.rdbuf()->pubsetbuf(buf.data(), buf.size());
        worker = std::thread(&AsyncWriter::writeLoop, this);
    }

    void push(uint64_t rel_id, uint64_t ts, const double* joints_ptr, const char* src) {
        DiskTask* t = new DiskTask();
        t->relative_id = rel_id;
        t->ts = ts;
        std::memcpy(t->joints, joints_ptr, 18 * sizeof(double));
        t->data.assign(src, src + CAM_SET_SIZE);
        
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
    
    shared_memory_object shm(open_only, "zed_shm", read_write);
    mapped_region region(shm, read_write);
    SHMHeader* header = static_cast<SHMHeader*>(region.get_address());
    const char* data_ptr = static_cast<const char*>(region.get_address()) + sizeof(SHMHeader);

    std::string filename = "dataset_capture.bin";

    // Check if an argument was provided
    if (argc > 1) {
        filename = argv[1];
    }

    std::cout << "Recording to: " << filename << std::endl;

    AsyncWriter writer(filename);
    
    // --- RELATIVE INDEXING SETUP ---
    // Capture the first frame count we see as our "Zero Point"
    uint64_t start_frame_id = header->frame_count;
    uint64_t last_recorded_id = start_frame_id;

    std::cout << ">>> Recording Started at Global Frame: " << start_frame_id << std::endl;

    while (keep_running) {
        if (header->frame_count > last_recorded_id) {
            auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
            uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(now).count();

            uint64_t current_global_id = header->frame_count;
            uint64_t relative_id = current_global_id - start_frame_id;

            int stable_idx = (header->active_buffer - 1 + 3) % 3;
            const char* src = data_ptr + (stable_idx * CAM_SET_SIZE);

            // Now pushing ID, TS, Joints, and Pixels
            writer.push(relative_id, ts, header->joints, src);
            
            last_recorded_id = current_global_id;

            if (relative_id % 30 == 0) {
                std::cout << "\rStored Frame (Relative): " << relative_id << std::flush;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    return 0;
}