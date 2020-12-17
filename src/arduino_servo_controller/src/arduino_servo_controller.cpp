/*
 * ROSのお勉強用プログラムです．
 * どなたでもご自由にお使いください．
 * 作者: 赤井直紀
 */

// Arduinoにサーボモータが繋がっている前提のプロフラムです．
// Arduinoに書き込まれているプログラムは以下になります．
// ros_intro/arduino/ServoVelocityControl/ServoVelocityControl.ino

#include <ros/ros.h>

// サーボモータへ速度の指令はgeometry_msgs::TwistStampedで受け取ります．
#include <geometry_msgs/TwistStamped.h>

// シリアル通信のためのヘッダファイルをインクルードします．
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class ArduinoServoController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber twistSub_;
    std::string inputTopicName_;

    // // シリアル通信設定のためのパラメータです．
    int fd_;
    struct termios tio_;
    std::string device_;
    int baudRate_;

public:
    ArduinoServoController(void):
        nh_("~"),
        inputTopicName_("/servo_angular_velocities"),
        device_("/dev/ttyACM0"),
        baudRate_(B9600)
    {
        nh_.param("arduino_device", device_, device_);
        nh_.param("arduino_baud_rate", baudRate_, baudRate_);

        twistSub_ = nh_.subscribe(inputTopicName_, 1, &ArduinoServoController::twistCB, this);

        // デバイスを開き，シリアル通信の設定をします．
        fd_ = open(device_.c_str(), O_RDWR);
        if (fd_ < 0) {
            fprintf(stderr, "cannot open the serial port -> %s\n", device_.c_str());
            exit(1);
        }
        tio_.c_cflag += CREAD;
        tio_.c_cflag += CLOCAL;
        tio_.c_cflag += CS8;
        tio_.c_cflag += 0;
        tio_.c_cflag += 0;
        cfsetispeed(&tio_, baudRate_);
        cfsetospeed(&tio_, baudRate_);
        cfmakeraw(&tio_);
        tcsetattr(fd_, TCSANOW, &tio_);
        ioctl(fd_, TCSETS, &tio_);
        sleep(1);
    }

    ~ArduinoServoController(void) {
        // プログラムの終了と同時にデバイスを閉じます．
        close(fd_);
    }

    void spin(void);

    void twistCB(const geometry_msgs::TwistStamped::ConstPtr &msg);
};

void ArduinoServoController::spin(void) {
    ros::spin();
}

void ArduinoServoController::twistCB(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    // 前回書き込みをした時間を記録しておきます．
    static bool isFirst = true;
    static double prevTime;
    if (isFirst) {
        prevTime = msg->header.stamp.toSec();
        isFirst = false;
    }

    // 書き込みの間隔が短い場合には書き込みをしません．
    // Arduinoが追いつけない？
    double currTime = msg->header.stamp.toSec();
    double deltaTime = currTime - prevTime;
    if (deltaTime < 0.01)
        return;

    // サーボモータへの速度指令を書き込みます．
    // yaw軸方向の速度のみを利用します．
    int yawAngVel = (int)(msg->twist.angular.z);
    // '\0'がコマンドの最後を意味する文字となっています．
    std::string cmd = std::to_string(yawAngVel) + '\0';
    int retVal = write(fd_, cmd.c_str(), cmd.length());
    prevTime = currTime;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "arduino_servo_controller");
    ArduinoServoController asc;
    asc.spin();
    return 0;
}