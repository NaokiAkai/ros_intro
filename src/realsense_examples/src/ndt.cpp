/*
 * ROSのお勉強用プログラムです．
 * どなたでもご自由にお使いください．
 * 作者: 赤井直紀
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/registration/ndt.h>

#include <tf/transform_broadcaster.h>

class NDT {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointsSub_;
    ros::Publisher pointsPub_;
    std::string inputTopicName_, outputTopicName_;
    double filtereResoX_, filtereResoY_, filtereResoZ_;

    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt_;

    tf::TransformBroadcaster tfBroadcaster_;

public:
    NDT(void):
        nh_("~"),
        inputTopicName_("/camera/depth_registered/points"),
        outputTopicName_("/camera/depth_registered/registered_points"),
        filtereResoX_(0.1),
        filtereResoY_(0.1),
        filtereResoZ_(0.1)
    {
        nh_.param("raw_points_name", inputTopicName_, inputTopicName_);
        nh_.param("filtered_points_name", outputTopicName_, outputTopicName_);
        nh_.param("vgf_reso_x", filtereResoX_, filtereResoX_);
        nh_.param("vgf_reso_y", filtereResoY_, filtereResoY_);
        nh_.param("vgf_reso_z", filtereResoZ_, filtereResoZ_);

        pointsSub_ = nh_.subscribe(inputTopicName_, 1, &NDT::pointsCB, this);

        pointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(outputTopicName_, 1);
    };

    ~NDT(void) {};

    void spin(void);

    void pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);
};

void NDT::spin(void) {
    ros::spin();
}

void NDT::pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg) {
    static bool isFisrt = true;
    static pcl::PointCloud<pcl::PointXYZRGB> targetPoints;
    if (isFisrt) {
        targetPoints = *msg;
        ndt_.setInputTarget(msg);
        isFisrt = false;
        return;
    }

    std_msgs::Header header;
    pcl_conversions::fromPCL(msg->header, header);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setLeafSize(filtereResoX_, filtereResoY_, filtereResoZ_);
    filter.setInputCloud(rawCloud);
    filter.filter(*filteredCloud);
    ndt_.setInputSource(filteredCloud);

    Eigen::AngleAxisf initRotationRoll(0.0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf initRotationPitch(0.0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf initRotationYaw(0.0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f initTranslation(0.0, 0.0, 0.0);
    Eigen::Matrix4f initGuess = (initTranslation * initRotationYaw * initRotationPitch * initRotationRoll).matrix();

    pcl::PointCloud<pcl::PointXYZRGB> outputCloud;
    ndt_.align(outputCloud, initGuess);

    Eigen::Matrix4f finalTrans = ndt_.getFinalTransformation();
    double x = static_cast<double>(finalTrans(0, 3));
    double y = static_cast<double>(finalTrans(1, 3));
    double z = static_cast<double>(finalTrans(2, 3));

    tf::Matrix3x3 finalRot;
    finalRot.setValue(static_cast<double>(finalTrans(0, 0)), static_cast<double>(finalTrans(0, 1)), static_cast<double>(finalTrans(0, 2)),
                      static_cast<double>(finalTrans(1, 0)), static_cast<double>(finalTrans(1, 1)), static_cast<double>(finalTrans(1, 2)),
                      static_cast<double>(finalTrans(2, 0)), static_cast<double>(finalTrans(2, 1)), static_cast<double>(finalTrans(2, 2)));
    double roll, pitch, yaw;
    finalRot.getRPY(roll, pitch, yaw);

    tf::Transform tf;
    tf::Quaternion q;
    tf.setOrigin(tf::Vector3(x, y, z));
    q.setRPY(roll, pitch, yaw);
    tf.setRotation(q);
    tfBroadcaster_.sendTransform(tf::StampedTransform(tf, header.stamp, "/world", "/camera_link"));

    pcl::PointCloud<pcl::PointXYZRGB> registeredPoints;
    registeredPoints.header = msg->header;
    for (size_t i = 0; i < targetPoints.points.size(); ++i)
        registeredPoints.points.push_back(targetPoints.points[i]);
//    outputCloud = finalTrans * rawCloud;
    for (size_t i = 0; i < outputCloud.points.size(); ++i)
        registeredPoints.points.push_back(outputCloud.points[i]);
    pointsPub_.publish(registeredPoints);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "realsense_ndt");
    NDT vgf;
    vgf.spin();
    return 0;
}
