#include <thread>
#include <mutex>
#include <kinect/cloud-tools.h>
#include "ros-thread.h"
#include "gui-thread.h"

int main (int argc, char *argv[]) {
    std::mutex mtx;
    leo::vars var;

    std::thread _gui_thread(leo::gui_main, argc, argv, std::ref(var), std::ref(mtx));
    std::thread _ros_thread(leo::ros_main, argc, argv, std::ref(var), std::ref(mtx));

    _gui_thread.join();
    _ros_thread.join();

    return 0;
}
