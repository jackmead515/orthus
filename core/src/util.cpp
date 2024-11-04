#include <iostream>
#include <iomanip>

namespace util {

    struct FPS {
        int fps_buffer_len { 32 };

        int fps_index { 0 };

        double value { 0.0 };

        double fps_buffer[32];

        double sum_sec { 0.0 };

        bool ready { false };

        // initialize the FPS buffer. Fills the buffer with 0.0
        FPS() {
            for (int i { 0 }; i < fps_buffer_len; i++) {
                fps_buffer[i] = 0.0;
            }
        }

        // update the FPS value if the fps_index has reached the buffer length
        void update(double duration) {
            fps_buffer[fps_index] = duration;
            sum_sec += duration;
            fps_index++;

            if (fps_index >= fps_buffer_len) {
                fps_index = 0;
                value = static_cast<double>(fps_buffer_len) / sum_sec;
                sum_sec = 0.0;
            }
        }
    };

    void print_progress(int index, int total, double fps) {
        int progress_percent = static_cast<int>((static_cast<double>(index) / static_cast<double>(total)) * 100.0);

        std::cout << "\r[" << std::setfill('#') << std::setw(progress_percent) << '#';
        std::cout << std::setfill(' ') << std::setw(100 - progress_percent) << "]";
        std::cout << std::setw(3) << progress_percent << "%";
        std::cout << " (" << index << "/" << total << ")" << " fps: " << fps;
        std::cout.flush();
    }

}
