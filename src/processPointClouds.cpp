// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <string>
#include <vector>
#include <unordered_set>
#include <ctime>
#include <chrono>
#include <iostream>
#include <filesystem>
#include <random>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>


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

// This function filter PCD using voxel grid point reduction and region based filtering
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered {new pcl::PointCloud<PointT>};

    // Filter the point cloud using voxel grid
    pcl::VoxelGrid<PointT> vg {};
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Filter the point cloud using region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudRegion {new pcl::PointCloud<PointT>};

    // Filter the point cloud using crop box
    pcl::CropBox<PointT> region {};
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.setNegative(false);
    region.filter(*cloudRegion);

    typename pcl::PointCloud<PointT>::Ptr cloudRoof {new pcl::PointCloud<PointT>};

    // Filter out the point cloud on the roof of the car where lidar is installed
    pcl::CropBox<PointT> roof {};
    roof.setInputCloud(cloudRegion);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setNegative(true);
    roof.filter(*cloudRoof);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRoof;
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
    auto startTime = std::chrono::steady_clock::now();
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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneUsingRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    std::size_t pointsSize = cloud->points.size();

    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, pointsSize - 1);

    std::unordered_set<int> inliersResult {};
    std::unordered_set<int> localInliers {};


    // Early termination parameters
    constexpr int maxIterationsWithoutImprovement = 10;
    int iterationsWithoutImprovement = 0;

	while(maxIterations--){
		localInliers.clear(); // Clear local inliers for each iteration

		// Randomly pick three different points using faster RNG
		while (localInliers.size() < 3) {
			localInliers.insert(dis(gen));
		}

		auto it = localInliers.begin();
		const PointT& point1 = cloud->points[*it];
		it++;
		const PointT& point2 = cloud->points[*it];
		it++;
		const PointT& point3 = cloud->points[*it];

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

        float denominator = sqrt((A * A) + (B * B) + (C * C));

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		for (int index = 0; index < pointsSize; index++) {
			if (localInliers.find(index) != localInliers.end()) {
				continue; // Skip already inliers
			}

			const PointT& point = cloud->points[index];
			// Distance = |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)
			float distance = fabs((A * point.x) + (B * point.y) + (C * point.z) + D) / denominator;

			// If distance is smaller than threshold, count it as inlier
			if (distance <= distanceThreshold) {
				localInliers.insert(index);
			}
		}

		// If number of inliers is greater than previous best, save it
		if (localInliers.size() > inliersResult.size()) {
			inliersResult = localInliers;
			iterationsWithoutImprovement = 0; // Reset counter when we find improvement
		} else {
			iterationsWithoutImprovement++;
			// Early termination: no improvement for several iterations
			if (iterationsWithoutImprovement >= maxIterationsWithoutImprovement) {
				break;
			}
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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


 // Perform euclidean clustering to group detected obstacles
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)  {
	// Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters {};

    // Build KdTree with all points
    std::unique_ptr<KdTree> tree = std::make_unique<KdTree>();
    std::size_t pointsSize = cloud->points.size();

    for (int i = 0; i < pointsSize; i++) {
        tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i);
    }

    std::vector<bool> processed(pointsSize, false);
    // Process each unprocessed point
    for (int i = 0; i < pointsSize; i++) {
        if (processed[i]){
            continue;
        }

        // Create a new cluster
        std::vector<int> cluster {};
        clusterHelper(i, cloud, cluster, processed, tree.get(), clusterTolerance);

        // Only keep clusters within size bounds
        if (cluster.size() >= minSize && cluster.size() <= maxSize) {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster {new pcl::PointCloud<PointT>};
            for (const auto& index : cluster) {
                cloudCluster->points.push_back(cloud->points[index]);
            }
            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}




template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(const typename pcl::PointCloud<PointT>::Ptr cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box {};
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(const typename pcl::PointCloud<PointT>::Ptr cluster) {
    // Compute principal directions using PCA
    Eigen::Vector4f pcaCentroid {};
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    // Compute the covariance matrix of the points in the cluster
    Eigen::Matrix3f covariance {};
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);

    // Perform eigen decomposition to find the principal components
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver{covariance, Eigen::ComputeEigenvectors};
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    // Ensure proper orientation - this line is necessary for proper orientation in some cases
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.0f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected {new pcl::PointCloud<PointT>};
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Create the oriented bounding box
    BoxQ boxQ {};

    // Calculate the quaternion for rotation using the eigenvectors
    boxQ.bboxQuaternion = Eigen::Quaternionf {eigenVectorsPCA};

    // Calculate the transform (translation) to put the box in correct location
    boxQ.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // Set the dimensions of the box
    boxQ.cube_length = maxPoint.x - minPoint.x;
    boxQ.cube_width = maxPoint.y - minPoint.y;
    boxQ.cube_height = maxPoint.z - minPoint.z;

    return boxQ;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {

    typename pcl::PointCloud<PointT>::Ptr cloud {new pcl::PointCloud<PointT>};

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) { //* load the file
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

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int pointId, typename pcl::PointCloud<PointT>::Ptr cloud,
                                               std::vector<int>& cluster, std::vector<bool>& processed,
                                               KdTree* tree, float clusterTolerance) {
    // Mark current point as processed and add to cluster
    processed[pointId] = true;
    cluster.push_back(pointId);

    // Find all nearby points within cluster tolerance
    std::vector<int> nearby = tree->search({cloud->points[pointId].x, cloud->points[pointId].y, cloud->points[pointId].z}, clusterTolerance);
    // Recursively process all unprocessed nearby points
    for (const int nearbyIdx : nearby) {
        if (!processed[nearbyIdx]) {
            clusterHelper(nearbyIdx, cloud, cluster, processed, tree, clusterTolerance);
        }
    }
}
