/*
 * ROSのお勉強用プログラムです．
 * どなたでもご自由にお使いください．
 * 作者: 赤井直紀
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// 角速度の制御司令はgeometry_msgs::TwistStampedで配信します．
#include <geometry_msgs/TwistStamped.h>

class AngularVelocityController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointsSub_;
    ros::Publisher twistPub_;
    std::string inputTopicName_, outputTopicName_;

    // 角速度制御様のパラメータです．
    float pGain_, iGain_, dGain_;
    float maxAngVel_;

public:
    AngularVelocityController(void):
        nh_("~"),
        inputTopicName_("/camera/depth_registered/cluster_centroids"),
        outputTopicName_("/servo_motor_twist_cmd"),
        pGain_(1.0f),
        dGain_(0.01f),
        iGain_(0.001f),
        maxAngVel_(40.0f)
    {
        nh_.param("clustered_centroids_name", inputTopicName_, inputTopicName_);
        nh_.param("servo_motor_twist_cmd", outputTopicName_, outputTopicName_);
        nh_.param("servo_controller_p_gain", pGain_, pGain_);
        nh_.param("servo_controller_i_gain", iGain_, iGain_);
        nh_.param("servo_controller_d_gain", dGain_, dGain_);
        nh_.param("servo_controller_max_ang_vel", maxAngVel_, maxAngVel_);

        pointsSub_ = nh_.subscribe(inputTopicName_, 1, &AngularVelocityController::pointsCB, this);

        twistPub_ = nh_.advertise<geometry_msgs::TwistStamped>(outputTopicName_, 1);
    }

    ~AngularVelocityController(void) {}

    void spin(void);

    void pointsCB(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);
};

void AngularVelocityController::spin(void) {
    ros::spin();
}

// カメラと重心との距離を返します．
float getDist(pcl::PointXYZ centroid) {
    return sqrt(centroid.x * centroid.x + centroid.y * centroid.y + centroid.z * centroid.z);
}

// カメラから最も近い重心を返します．
pcl::PointXYZ getClosestCentroid(pcl::PointCloud<pcl::PointXYZ> centroids) {
    // 0番目の重心を最短距離と仮定します．
    int idx = 0;
    float minDist = getDist(centroids.points[0]);

    // 1番目以降の重心と比較していきます．
    for (size_t i = 1; i < centroids.points.size(); i++) {
        float dist = getDist(centroids.points[i]);
        // i番目の重心との距離が今の最短距離より近い場合は更新します．
        if (dist < minDist) {
            idx = i;
            minDist = dist;
        }
    }
    return centroids.points[idx];
} 

void AngularVelocityController::pointsCB(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg) {
    // staic変数はこの関数終了時の最後の値を保存します（最初の値だけ0となります）．
    static float prevDeltaTheta = 0.0f, deltaThetaSum = 0.0f;

    // pclのメッセージに付属するヘッダの情報をROS様に変換します．
    std_msgs::Header header;
    pcl_conversions::fromPCL(msg->header, header);

    // 配信様のメッセージを定義しヘッダの情報を格納します．
    // frame_idは重要でなく，stampのみが重要になります．
    geometry_msgs::TwistStamped twistCmd;
    twistCmd.header = header;

    // 重心がない場合は角速度0を司令して戻ります．
    if ((int)msg->points.size() == 0) {
        twistCmd.twist.angular.z = 0.0f;
        twistPub_.publish(twistCmd);
        return;
    }

    // 最も近い重心を取得します．
    pcl::PointXYZ closestCentroid = getClosestCentroid(*msg);

    // カメラ画像座標をカメラ座標に変換しています．
    float targetX = closestCentroid.z;
    float targetY = -closestCentroid.x;

    // 重心とカメラが向いている方向の差分をdegreeで求めます．
    float deltaTheta = atan2(targetY, targetX) * 180.0f / M_PI;

    // D制御のための値を計算します（角度の差分は必ず−180から180度の値でないといけません）．
    float dDeltaTheta = deltaTheta - prevDeltaTheta;
    while (dDeltaTheta < -180.0f)
        dDeltaTheta += 180.0f;
    while (dDeltaTheta > 180.0f)
        dDeltaTheta -= 180.0f;
//    printf("targetX = %f, targetY = %f, deltaTheta = %f\n", targetX, targetY, deltaTheta);

    // PID制御で角速度を求めます．
    float angVel = pGain_ * deltaTheta + iGain_ * deltaThetaSum + dGain_ * dDeltaTheta;
//    printf("time = %lf, angVel = %.2f [deg/sec]\n", header.stamp.toSec(), angVel);

    // 配信する前に角速度のチェックを行います（超重要！）．
    if (std::isnan(angVel))
        angVel = 0.0f;
    else if (angVel < -maxAngVel_)
        angVel = -maxAngVel_;
    else if (angVel > maxAngVel_)
        angVel = maxAngVel_;
    twistCmd.twist.angular.z = angVel;
    twistPub_.publish(twistCmd);

    // D制御とI制御のためのパラメータを更新します．
    // これらはstatic変数なので，次にこの関数が始まるときはこの時の値となっています．
    prevDeltaTheta = deltaTheta;
    deltaThetaSum += deltaTheta;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "angular_velocity_controller");
    AngularVelocityController avc;
    avc.spin();
    return 0;
}