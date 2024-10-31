#include <iostream>
#include <iomanip>

namespace util {

    void print_progress(int index, int total, double fps) {
        int progress_percent = static_cast<int>((static_cast<double>(index) / static_cast<double>(total)) * 100.0);

        std::cout << "\r[" << std::setfill('#') << std::setw(progress_percent) << '#';
        std::cout << std::setfill(' ') << std::setw(100 - progress_percent) << "]";
        std::cout << std::setw(3) << progress_percent << "%";
        std::cout << " (" << index << "/" << total << ")" << " fps: " << fps;
        std::cout.flush();
    }

}
