// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <string>
#include <vector>
#include <unordered_set>
#include <ctime>
#include <chrono>
#include <iostream>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include "render/box.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud {new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>};

    // for(const auto& index: inliers->indices){
    //     planeCloud->points.push_back(cloud->points[index]);
    // }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract {};

    // Extract the inliers from the cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // Extract the plane
     extract.setNegative(false);
     extract.filter(*planeCloud);

    // Extract the obstacles
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult{obstCloud, planeCloud};

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::high_resolution_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    pcl::SACSegmentation<PointT> seg {};

    // Find inliers for the cloud.
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneUsingRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::high_resolution_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    std::unordered_set<int> inliersResult;
	// For max iterations
	while(maxIterations--){
		std::unordered_set<int> localInliers;

		// Randomly pick three different points
		while (localInliers.size() < 3) {
			localInliers.insert(rand() % cloud->points.size());
		}

		auto it = localInliers.begin();
		PointT point1 = cloud->points[*it];
		it++;
		PointT point2 = cloud->points[*it];
		it++;
		PointT point3 = cloud->points[*it];

		// Calculate distance from point to plane
		// Plane equation: Ax + By + Cz + D = 0
		// A = (y2 - y1)(z3 - z1) - (z2 - z1)(y3 - y1)
		// B = (x3 - x1)(z2 - z1) - (z3 - z1)(x2 - x1)
		// C = (x2 - x1)(y3 - y1) - (y2 - y1)(x3 - x1)
		// D = -(Ax1 + By1 + Cz1)
		// Distance = |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)
		float A = ((point2.y - point1.y) * (point3.z - point1.z)) - ((point2.z - point1.z) * (point3.y - point1.y));
		float B = ((point3.x - point1.x) * (point2.z - point1.z)) - ((point3.z - point1.z) * (point2.x - point1.x));
		float C = ((point2.x - point1.x) * (point3.y - point1.y)) - ((point2.y - point1.y) * (point3.x - point1.x));
		float D = -(A * point1.x + B * point1.y + C * point1.z);

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		for (int index = 0; index < cloud->points.size(); index++) {
			if (localInliers.count(index) > 0) {
				continue; // Skip already inliers
			}

			PointT point = cloud->points[index];
			// Distance = |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)
			float distance = fabs((A * point.x) + (B * point.y) + (C * point.z) + D) / sqrt((A * A) + (B * B) + (C * C));

			// If distance is smaller than threshold, count it as inlier
			if (distance <= distanceThreshold) {
				localInliers.insert(index);
			}
		}

		// If number of inliers is greater than previous best, save it
		if (localInliers.size() > inliersResult.size()) {
			inliersResult = localInliers;
		}
	}

    // Convert inliersResult to pcl::PointIndices
    inliers->indices.clear();
    for (const auto& index : inliersResult) {
        inliers->indices.push_back(index);
    }

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


 // Perform euclidean clustering to group detected obstacles
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters {};

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree {new pcl::search::KdTree<PointT>};
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices {};
    pcl::EuclideanClusterExtraction<PointT> ec {};

    // Set the clustering parameters
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    // Loop through each cluster and create a new point cloud for it
    for(const auto& cluster: clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster {new pcl::PointCloud<PointT>};

        for(const auto& index: cluster.indices) {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<std::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

    std::vector<std::filesystem::path> paths;
    for (const auto& entry : std::filesystem::directory_iterator(dataPath)) {
        if (entry.path().extension() == ".pcd") {
            paths.push_back(entry.path());
        }
    }

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}