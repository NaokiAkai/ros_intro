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

// rvizに表示するマーカー用のヘッダーをインクルードします．
#include <visualization_msgs/Marker.h>

// マーカーに色をつけるための変数をインクルードします．
#include <std_msgs/ColorRGBA.h>

class CubesVis {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointsSub_;
    ros::Publisher markersPub_;
    std::string inputTopicName_, outputTopicName_;
    double cubeResoX_, cubeResoY_, cubeResoZ_;
    double yMin_, yMax_;

public:
    CubesVis(void):
        nh_("~"),
        inputTopicName_("/camera/depth_registered/filtered_points"),
        outputTopicName_("/camera/depth_registered/cube_markers"),
        cubeResoX_(0.1),
        cubeResoY_(0.1),
        cubeResoZ_(0.1),
        yMin_(-2.0),
        yMax_(2.0)
    {
        nh_.param("filtered_points_name", inputTopicName_, inputTopicName_);
        nh_.param("cube_markers", inputTopicName_, inputTopicName_);

        pointsSub_ = nh_.subscribe(inputTopicName_, 1, &CubesVis::pointsCB, this);

        markersPub_ = nh_.advertise<visualization_msgs::Marker>(outputTopicName_, 1);
    }

    ~CubesVis(void) {};

    void spin(void);

    void pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);
};

void CubesVis::spin(void) {
    ros::spin();
}

// value = (x - min) / (max - min)となります．
// min，maxに対応したxに相当するrgbに0から255の値を代入します．
void val2rgb(double value, int* r, int* g, int* b) {
    double tmp_val = cos(4.0 * M_PI * value);
    int col_val = (int)(( -tmp_val / 2.0 + 0.5) * 255.0);
    if (value >= (4.0 / 4.0)) {
        *r = 255, *g = 0, *b = 0;
    } else if (value >= (3.0 / 4.0)) {
        *r = 255, *g = col_val, *b = 0;
    } else if (value >= (2.0 / 4.0)) {
        *r = col_val, *g = 255, *b = 0;
    } else if (value >= (1.0 / 4.0)) {
        *r = 0, *g = 255, *b = col_val;
    } else if (value >= (0.0 / 4.0)) {
        *r = 0, *g = col_val, *b = 255;
    } else {
        *r = 0, *g = 0, *b = 255;
    }
}

void CubesVis::pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg) {
    std_msgs::Header header;
    pcl_conversions::fromPCL(msg->header, header);

    // マーカーを定義してパラメータを設定します．
    visualization_msgs::Marker cubes;
    cubes.header = header;
    cubes.ns = "realsense_cubes";
    cubes.id = 0;
    cubes.type = visualization_msgs::Marker::CUBE_LIST;
    cubes.action = visualization_msgs::Marker::ADD;
    cubes.pose.orientation.x = 0.0;
    cubes.pose.orientation.y = 0.0;
    cubes.pose.orientation.z = 0.0;
    cubes.pose.orientation.w = 1.0;
    cubes.scale.x = cubeResoX_;
    cubes.scale.y = cubeResoY_;
    cubes.scale.z = cubeResoZ_;
    cubes.color.r = 1.0;
    cubes.color.g = 0.0;
    cubes.color.b = 0.0;
    cubes.color.a = 1.0;

    // 受け取った点群でキューブリストを作成します．
    for (size_t i = 0; i < msg->points.size(); ++i) {
        // 点をキューブとして追加します．
        geometry_msgs::Point p;
        p.x = msg->points[i].x;
        p.y = msg->points[i].y;
        p.z = msg->points[i].z;
        cubes.points.push_back(p);

        // y軸の値に合わせて色をつけます．
        std_msgs::ColorRGBA c;
        int r, g, b;
        val2rgb((p.y - yMin_) / (yMax_ - yMin_), &r, &g, &b);
        c.r = (float)r / 255.0f;
        c.g = (float)g / 255.0f;
        c.b = (float)b / 255.0f;
        c.a = 1.0f;
        cubes.colors.push_back(c);
    }

    // キューブを配信します．
    markersPub_.publish(cubes);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "realsense_cubes_vis");
    CubesVis cvis;
    cvis.spin();
    return 0;
}