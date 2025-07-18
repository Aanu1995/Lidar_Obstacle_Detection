// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include "render/box.h"
#include "kdtree.h"

template<typename PointT>
class ProcessPointClouds {

    public:
        //constructor
        ProcessPointClouds();
        //deconstructor
        ~ProcessPointClouds();

        void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

        typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlaneUsingRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

        std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

        std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

        void clusterHelper(int pointIdx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, class KdTree* tree, float clusterTolerance);

        Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

        BoxQ BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

        void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

        typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

        std::vector<std::filesystem::path> streamPcd(std::string dataPath);

};
#endif /* PROCESSPOINTCLOUDS_H_ */