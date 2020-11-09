/*
 * ROSのお勉強用プログラムです．
 * どなたでもご自由にお使いください．
 * 作者: 赤井直紀
 */

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "hello_world");
    ros::NodeHandle nh("~");
    printf("Hello world\n");
    return 0;
}
