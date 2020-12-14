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
#include <pcl/filters/voxel_grid.h>

// クラスタリング用のヘッダをインクルードします．
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

class Color {
private:
   unsigned char r_, g_, b_;

public:
    Color(void): r_(0), g_(0), b_(0) {}
    Color(unsigned char r, unsigned char g, unsigned char b): r_(r), g_(g), b_(b) {}
    ~Color(void) {}

    inline unsigned char getR(void) { return r_; }
    inline unsigned char getG(void) { return g_; }
    inline unsigned char getB(void) { return b_; }
    inline Color getColor(void) { return Color(r_, g_, b_); }

    inline void setR(unsigned char r) { r_ = r; }
    inline void setG(unsigned char g) { g_ = g; }
    inline void setB(unsigned char b) { b_ = b; }
    inline void setColor(unsigned char r, unsigned char g, unsigned char b) { r_ = r, b_ = b, g_ = g; }
};

class PointsCluster {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointsSub_;
    ros::Publisher clusteredPointsPub_, filteredPointsPub_, centroidsPub_;
    std::string inputTopicName_, clusteredPointsTopicName_, filteredCloudTopicName_, centroidsTopicName_;

    // ボクセルグリッドフィルタのパラメータです．
    double filtereResoX_, filtereResoY_, filtereResoZ_;

    // クラスタリングのパラメータです．
    int minClusterSize_;
    double clusterTolerance_;

    // クラスタリングの結果表示用のための色情報です．
    std::vector<Color> colors_;

public:
    PointsCluster(void):
        nh_("~"),
        inputTopicName_("/camera/depth_registered/colored_points"),
        clusteredPointsTopicName_("/camera/depth_registered/clustered_points"),
        filteredCloudTopicName_("/camera/depth_registered/filtered_points"),
        centroidsTopicName_("/camera/depth_registered/cluster_centroids"),
        filtereResoX_(0.01),
        filtereResoY_(0.01),
        filtereResoZ_(0.01),
        minClusterSize_(10),
        clusterTolerance_(0.03)
    {
        nh_.param("source_points_name", inputTopicName_, inputTopicName_);
        nh_.param("clustered_points_name", clusteredPointsTopicName_, clusteredPointsTopicName_);
        nh_.param("filtered_points_name", filteredCloudTopicName_, filteredCloudTopicName_);
        nh_.param("clustered_centroids_name", centroidsTopicName_, centroidsTopicName_);
        nh_.param("clustering_vgf_reso_x", filtereResoX_, filtereResoX_);
        nh_.param("clustering_vgf_reso_y", filtereResoY_, filtereResoY_);
        nh_.param("clustering_vgf_reso_z", filtereResoZ_, filtereResoZ_);
        nh_.param("min_cluster_size", minClusterSize_, minClusterSize_);
        nh_.param("cluster_tolerance", clusterTolerance_, clusterTolerance_);

        pointsSub_ = nh_.subscribe(inputTopicName_, 1, &PointsCluster::pointsCB, this);

        clusteredPointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(clusteredPointsTopicName_, 1);
        filteredPointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(filteredCloudTopicName_, 1);
        centroidsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(centroidsTopicName_, 1);

        // クラスタリングの結果表示様に適当に10色用意します．
        colors_.push_back(Color(255, 0, 0));
        colors_.push_back(Color(255, 165, 0));
        colors_.push_back(Color(255, 255, 0));
        colors_.push_back(Color(165, 255, 0));
        colors_.push_back(Color(0, 255, 0));
        colors_.push_back(Color(0, 165, 165));
        colors_.push_back(Color(0, 0, 255));
        colors_.push_back(Color(128, 68, 170));
        colors_.push_back(Color(165, 90, 165));
        colors_.push_back(Color(220, 80, 140));
    };

    ~PointsCluster(void) {};

    void spin(void);

    void pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);

    inline Color getColor(int idx) {
        int i = idx % (int)colors_.size();
        return colors_[i];
    }
};

void PointsCluster::spin(void) {
    ros::spin();
}

void PointsCluster::pointsCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg) {
    // 点の数が多すぎると処理に時間がかかりすぎるためボクセルグリッドフィルタで点の数を少なくします．
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> vgf;
    vgf.setLeafSize(filtereResoX_, filtereResoY_, filtereResoZ_);
    vgf.setInputCloud(cloud);
    vgf.filter(*filteredCloud);
    filteredCloud->header = msg->header;

    // kd-treeを構築します．
    // 効率的にクラスタリングを行うための点群の表現方法です．
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(filteredCloud);

    // クラスタリング用のパラメータをセットして実行します．
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clusterExtractor;
    clusterExtractor.setClusterTolerance(clusterTolerance_);
    clusterExtractor.setMinClusterSize(minClusterSize_);
    clusterExtractor.setMaxClusterSize(filteredCloud->points.size());
    clusterExtractor.setSearchMethod(tree);
    clusterExtractor.setInputCloud(filteredCloud);
    clusterExtractor.extract(clusterIndices);

    // クラスタリングの結果を配信できる形にします．
    pcl::ExtractIndices<pcl::PointXYZRGB> extractIndices;
    extractIndices.setInputCloud(filteredCloud);
    extractIndices.setNegative(false);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusteredPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroids(new pcl::PointCloud<pcl::PointXYZ>);
    clusteredPoints->header = centroids->header = msg->header;
    for (size_t i = 0; i < clusterIndices.size(); i++) {
        // tmpClusteredPointsがそれぞれクラスタリングされた点になります．
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpClusteredPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointIndices::Ptr tmpClusteredIndices(new pcl::PointIndices);
        *tmpClusteredIndices = clusterIndices[i];
        extractIndices.setIndices(tmpClusteredIndices);
        extractIndices.filter(*tmpClusteredPoints);

        // それぞれの点に表示用の色を付けます．
        // 同時に各クラスタの重心を計算します．
        Color color = getColor((int)i);
        pcl::PointXYZ centroid(0.0, 0.0, 0.0);
        int clusteredPointsSize = tmpClusteredPoints->points.size();
        for (size_t j = 0; j < clusteredPointsSize; j++) {
            tmpClusteredPoints->points[j].r = color.getR();
            tmpClusteredPoints->points[j].g = color.getG();
            tmpClusteredPoints->points[j].b = color.getB();
            clusteredPoints->push_back(tmpClusteredPoints->points[j]);

            centroid.x += tmpClusteredPoints->points[j].x;
            centroid.y += tmpClusteredPoints->points[j].y;
            centroid.z += tmpClusteredPoints->points[j].z;
        }
        centroid.x /= clusteredPointsSize;
        centroid.y /= clusteredPointsSize;
        centroid.z /= clusteredPointsSize;
        centroids->points.push_back(centroid);
    }

    clusteredPointsPub_.publish(clusteredPoints);
    filteredPointsPub_.publish(filteredCloud);
    centroidsPub_.publish(centroids);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "points_cluster");
    PointsCluster pc;
    pc.spin();
    return 0;
}
