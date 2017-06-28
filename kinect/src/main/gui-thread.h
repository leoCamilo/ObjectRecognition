#ifndef GUI_THREAD_H
#define GUI_THREAD_H

// #include <mutex>
// #include <kinect/cloud-tools.h>

namespace leo {
    void gui_main(int argc, char *argv[], vars& var, std::mutex& mtx);
}

#endif
