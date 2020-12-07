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

class ColoredPointsExtractor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointsSub_;
    ros::Publisher pointsPub_;
    std::string inputTopicName_, outputTopicName_;

public:
    ColoredPointsExtractor(void):
        nh_("~"),
        inputTopicName_("/camera/depth_registered/points"),
        outputTopicName_("/camera/depth_registered/colored_points")
    {
        nh_.param("raw_points_name", inputTopicName_, inputTopicName_);

        pointsSub_ = nh_.subscribe(inputTopicName_, 1, &ColoredPointsExtractor::pointsCB, this);

        pointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(outputTopicName_, 1);
    };

    ~ColoredPointsExtractor(void) {};

    void spin(void);

    void pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);
};

void ColoredPointsExtractor::spin(void) {
    ros::spin();
}

// RGBをHSVに変換します．
void rgb2hsv(int r, int g, int b, float *h, float *s, float *v) {
    // minとmaxを取り出します．
    int max, min;
    if (r > g && r > b) {
        max = r;
        if (g < b)
            min = g;
        else
            min = b;
    } else if (g > r && g > b) {
        max = g;
        if (r < b)
            min = r;
        else
            min = b;
    } else {
        max = b;
        if (r < g)
            min = r;
        else
            min = g;
    }

    // maxがゼロの場合は黒なのですべてゼロとなります．
    if (max == 0) {
        *h = 0.0f, *s = 0.0f, *v = 0.0f;
        return;
    }

    // HSVを計算します．
    *v = (float)max / 255.0f;
    *s = ((float)(max - min) / (float)max);
    if (max == r)
        *h = 60.0 * ((float)(b - g) / (float)(max - min));
    else if (max == g)
        *h = 60.0 * (2.0f + (float)(r - b) / (float)(max - min));
    else
        *h = 60.0 * (4.0f + (float)(g - r) / (float)(max - min));

    // hは必ず正の値になります．
    while (*h < 0)
        *h += 360.0f;
}

void ColoredPointsExtractor::pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZRGB> coloredPoints;
    coloredPoints.header = msg->header;

    for (size_t i = 0; i < msg->points.size(); i++) {
        float h, s, v;
        rgb2hsv(msg->points[i].r, msg->points[i].g, msg->points[i].b, &h, &s, &v);

        // H = 0の周辺が赤です．
        // その差分を取り，30度以内のデータを赤と認識します．
        float deltaAngle = h - 0.0f;
        while (deltaAngle < -180.0f)
            deltaAngle += 360.0f;
        while (deltaAngle > 180.0f)
            deltaAngle -= 360.0f;
        if (fabs(deltaAngle) <= 30.0f && s >= 0.5f && v > 0.7f)
            coloredPoints.points.push_back(msg->points[i]);
    }
    pointsPub_.publish(coloredPoints);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "colored_points_extractor");
    ColoredPointsExtractor cpe;
    cpe.spin();
    return 0;
}