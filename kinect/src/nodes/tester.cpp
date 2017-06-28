#include <ros/ros.h>
#include <kinect/test-lib.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tester_node");
  ros::NodeHandle nh;

  test_hello();
  say_hello();

  ros::spin();
  return 0;
}
