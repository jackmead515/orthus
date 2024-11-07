#pragma once

namespace util {

    struct FPS {
        int buffer_len;

        int index;

        double value;

        double buffer[32];

        double sum;

        FPS() {
            this->buffer_len = 32;
            this->index = 0;
            this->value = 0.0;
            this->sum = 0.0;
            for (int i {0}; i < this->buffer_len; i++) this->buffer[i] = 0.0;
        }

        void update(double duration) {
            this->buffer[index] = duration;
            this->sum += duration;
            this->index++;

            if (this->index >= this->buffer_len) {
                this->index = 0;
                this->value = static_cast<double>(this->buffer_len) / this->sum;
                this->sum = 0.0;
            }
        }
    };

    void print_progress(int index, int total, double fps);
}