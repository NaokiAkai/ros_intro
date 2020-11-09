/*
 * ROSのお勉強用プログラムです．
 * どなたでもご自由にお使いください．
 * 作者: 赤井直紀
 */

#include <ros/ros.h>

// REALSENSEの点群データはsensor_msgs/PointCloud2で配信されています．
#include <sensor_msgs/PointCloud2.h>

// Point Cloud Library（PCL）のヘッダをインクルードします．
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// pcl::PointXYZRGBを使うためにインクルードします．
#include <pcl_ros/point_cloud.h>

// Voxel Grid Filterを実行するクラスです．
class VoxelGridFilter {
private:
    ros::NodeHandle nh_;

    // REALSENSEの配信する点群を受信します．
    ros::Subscriber pointsSub_;

    // 受信した点群にVoxel Grid Filterを適用した結果を送信します．
    ros::Publisher pointsPub_;

    // 受信，送信する点群の名前です．
    std::string inputTopicName_, outputTopicName_;

    // フィルターの解像度です．
    double filtereResoX_, filtereResoY_, filtereResoZ_;

public:
    // コンストラクタの飛び出しと同時に変数をデフォルト値に設定します．
    VoxelGridFilter(void):
        nh_("~"),
        inputTopicName_("/camera/depth_registered/points"),
        outputTopicName_("/camera/depth_registered/filtered_points"),
        filtereResoX_(0.1),
        filtereResoY_(0.1),
        filtereResoZ_(0.1)
    {
        // nh_("~")で初期化しておくと，以下の様にパラメータを受け取れます．
        // 例えば，/raw_points_nameというパラメータに何かstring変数が設定されていれば，inputTopicName_にはそれが入ります．
        // もしその様なパラメータがなければ，inputTopicName_は初期化された状態から変わりません．
        // この様に設定しておくと，後にlaunchスクリプトでプログラムを一括起動する際に，変数の整合性が取りやすくなります．
        nh_.param("raw_points_name", inputTopicName_, inputTopicName_);
        nh_.param("filtered_points_name", outputTopicName_, outputTopicName_);
        nh_.param("vgf_reso_x", filtereResoX_, filtereResoX_);
        nh_.param("vgf_reso_y", filtereResoY_, filtereResoY_);
        nh_.param("vgf_reso_z", filtereResoZ_, filtereResoZ_);

        // Subscriberの設定です．
        // class内で設定する場合，&VoxelGridFilter::pointsCBとthisが着くことに注意してください．
        pointsSub_ = nh_.subscribe(inputTopicName_, 1, &VoxelGridFilter::pointsCB, this);

        // Publisherの設定です．
        // PublisherはSubscriberと異なり，class内で設定する場合も特に変わりません．
        pointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(outputTopicName_, 1);
    };

    // デコンストラクタですが，特に何もしません．
    ~VoxelGridFilter(void) {};

    // ros::spin()をするだけの関数です．
    void spin(void);

    // 実際に点群を受信して処理を行うコールバック関数です．
    void pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);
};

void VoxelGridFilter::spin(void) {
    ros::spin();
}

// REALSENSEの点群データはsensor_msgs/PointCloud2で配信されていますが，PCLの点群として受信することもできます．
// PCLを用いて処理を行うので，PCLの点群として受信します．
// PCLの点群（pcl::PointCloud）とROSの点群（sensor_msgs::PointCloud2）は違うことに注意してください．
void VoxelGridFilter::pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg) {
    // Voxel Grid Filterを実行します．
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setLeafSize(filtereResoX_, filtereResoY_, filtereResoZ_);
    filter.setInputCloud(inCloud);
    filter.filter(*outCloud);

    // Voxel Grid Filterを適用した点群を送信します．
    // sensor_msgs::PointCloud2の型を送信すると設定していますが，実際にはPCLの型の点群で送信しています．
    // 自動でsensor_msgs::PointCloud2の型に変更してくれるので，これも問題ありません．
    pointsPub_.publish(outCloud);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "realsense_voxel_grid_filter");

    // Voxel Grid Filterを実行するクラスを宣言し，待機するだけです．
    VoxelGridFilter vgf;
    vgf.spin();

    return 0;
}
