#pragma once

namespace util {

    struct FPS {
        int fps_buffer_len;

        int fps_index;

        double value;

        double fps_buffer[32];

        double sum_sec;

        bool ready;

        // // initialize the FPS buffer. Fills the buffer with 0.0
        // FPS();

        // // update the FPS value if the fps_index has reached the buffer length
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

    void print_progress(int index, int total, double fps);
}