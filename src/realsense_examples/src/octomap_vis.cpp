/*
 * ROSのお勉強用プログラムです．
 * どなたでもご自由にお使いください．
 * 作者: 赤井直紀
 */

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

class OctomapVis {
private:
    ros::NodeHandle nh_;
    std::string inputTopicName_, outputTopicName_;
    ros::Subscriber pointsSub_;
    ros::Publisher octomapPub_;

    double octomapReso_;

public:
    OctomapVis(void):
        nh_("~"),
        inputTopicName_("/camera/depth_registered/points"),
        outputTopicName_("/camera/depth_registered/octomap"),
        octomapReso_(0.1)
    {
        nh_.param("raw_points_name", inputTopicName_, inputTopicName_);
        nh_.param("local_octomap_name", outputTopicName_, outputTopicName_);
        nh_.param("local_octomap_reso", octomapReso_, octomapReso_);

        pointsSub_ = nh_.subscribe(inputTopicName_, 1, &OctomapVis::pointsCB, this);

        octomapPub_ = nh_.advertise<octomap_msgs::Octomap>(outputTopicName_, 1);
    }

    ~OctomapVis(void) {};

    void spin(void);

    void pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);
};

void OctomapVis::spin(void) {
    ros::spin();
}

void OctomapVis::pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg) {
    std_msgs::Header header;
    pcl_conversions::fromPCL(msg->header, header);

    octomap::OcTree octree(octomapReso_);
    octomap::KeySet occupiedCells;
    for (size_t i = 0; i < msg->points.size(); i++) {
        double x = msg->points[i].x;
        double y = msg->points[i].y;
        double z = msg->points[i].z;
        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
            continue;
        octomap::OcTreeKey key;
        octomap::point3d endPoint(x, y, z);
        if (octree.coordToKeyChecked(endPoint, key))
            occupiedCells.insert(key);
    }

    for (octomap::KeySet::iterator it = occupiedCells.begin(), end = occupiedCells.end(); it != end; ++it)
        octree.updateNode(*it, true);
    octree.prune();

    octomap_msgs::Octomap octomapMsg;
//    octomap_msgs::fullMapToMsg(octree, octomapMsg); // .ot
    octomap_msgs::binaryMapToMsg(octree, octomapMsg); // .bt
    octomapMsg.header = header;
    octomapMsg.binary = true; // if binary is true, binaryMapToMsg must be used
    octomapMsg.id = 1;
    octomapMsg.resolution = octomapReso_;
    octomapPub_.publish(octomapMsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "realsense_octomap_vis");
    OctomapVis vis;
    vis.spin();
    return 0;
}