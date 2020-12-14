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

class PointsPrinter {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointsSub_;
    ros::Publisher pointsPub_;
    std::string inputTopicName_, outputTopicName_;

public:
    PointsPrinter(void):
        nh_("~"),
        inputTopicName_("/camera/depth_registered/points"),
        outputTopicName_("/camera/depth_registered/points_from_points_printer")
    {
        nh_.param("raw_points_name", inputTopicName_, inputTopicName_);

        pointsSub_ = nh_.subscribe(inputTopicName_, 1, &PointsPrinter::pointsCB, this);

        pointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(outputTopicName_, 1);
    };

    ~PointsPrinter(void) {};

    void spin(void);

    void pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);
};

void PointsPrinter::spin(void) {
    ros::spin();
}

// 受け取った点を表示します．
// ついでに，受け取った点群をコピーしたものをパブリッシュします．
// このパブリッシュを例にして，何か処理をしたものをパプリッシュしてみましょう．
// 注意！端末に大量の文字をプリントすると処理が遅くなります．
// 実際何か処理をする場合は，printfをコメントアウトしてください．
void PointsPrinter::pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZRGB> copiedPoints;
    copiedPoints.header = msg->header;

    for (size_t i = 0; i < msg->points.size(); i++) {
        printf("x = %f, y = %f, z = %f, r = %d, g = %d, b = %d\n",
            msg->points[i].x, msg->points[i].y, msg->points[i].z, msg->points[i].r, msg->points[i].g, msg->points[i].b);
        if (msg->points[i].b > 200)
            copiedPoints.points.push_back(msg->points[i]);
    }
    printf("\n");

    pointsPub_.publish(copiedPoints);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "realsense_points_printer");
    PointsPrinter pp;
    pp.spin();
    return 0;
}