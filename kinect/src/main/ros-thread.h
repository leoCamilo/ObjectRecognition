#ifndef ROS_THREAD_H
#define ROS_THREAD_H

// #include <mutex>
// #include <kinect/cloud-tools.h>

namespace leo {
    void ros_main(int argc, char *argv[], vars& var, std::mutex& mtx);
}

#endif
