// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++) {
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--) {
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene() {
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	// viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations
	while(maxIterations--){
		std::unordered_set<int> inliers;

		// Randomly pick two different points
		while (inliers.size() < 2) {
			inliers.insert(rand() % cloud->points.size());
		}

		auto it = inliers.begin();
		pcl::PointXYZ point1 = cloud->points[*it];
		it++;
		pcl::PointXYZ point2 = cloud->points[*it];

		// Calculate distance from point to line
		// Ax + By + C = 0
		// A = y1 - y2
		// B = x2 - x1
		// C = x1 * y2 - x2 * y1
		// Distance = |Ax + By + C| / sqrt(A^2 + B^2)
		float A = point1.y - point2.y;
		float B = point2.x - point1.x;
		float C = point1.x * point2.y - point2.x * point1.y;


		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int index = 0; index < cloud->points.size(); index++) {
			if (inliers.count(index) > 0) {
				continue; // Skip already inliers
			}

			pcl::PointXYZ point = cloud->points[index];
			// Distance = |Ax + By + C| / sqrt(A^2 + B^2)
			float distance = fabs((A * point.x) + (B * point.y) + C) / sqrt((A * A) + (B * B));

			// If distance is smaller than threshold, count it as inlier
			if (distance <= distanceTol) {
				inliers.insert(index);
			}
		}

		// If number of inliers is greater than previous best, save it
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
	std::cout << "RANSAC took " << elapsedTime << " milliseconds to run." << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations
	while(maxIterations--){
		std::unordered_set<int> inliers;

		// Randomly pick three different points
		while (inliers.size() < 3) {
			inliers.insert(rand() % cloud->points.size());
		}

		auto it = inliers.begin();
		pcl::PointXYZ point1 = cloud->points[*it];
		it++;
		pcl::PointXYZ point2 = cloud->points[*it];
		it++;
		pcl::PointXYZ point3 = cloud->points[*it];

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
			if (inliers.count(index) > 0) {
				continue; // Skip already inliers
			}

			pcl::PointXYZ point = cloud->points[index];
			// Distance = |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)
			float distance = fabs((A * point.x) + (B * point.y) + (C * point.z) + D) / sqrt((A * A) + (B * B) + (C * C));

			// If distance is smaller than threshold, count it as inlier
			if (distance <= distanceTol) {
				inliers.insert(index);
			}
		}

		// If number of inliers is greater than previous best, save it
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
	std::cout << "RANSAC took " << elapsedTime << " milliseconds to run." << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main () {

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data for 2D point cloud
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// Create data for 3D point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// Ransac function for 2D line fitting
	// std::unordered_set<int> inliers = Ransac2D(cloud, 50, 0.5);

	// Ransac function for 3D line fitting
	std::unordered_set<int> inliers = Ransac3D(cloud, 1000, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++) {
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Cloud filtering
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
	for (const auto& point : cloud->points) {
		if (inliers.count(&point - &cloud->points[0])) {
			cloudFiltered->points.push_back(point);
		}
	}
	cloudFiltered->width = cloudFiltered->points.size();
	cloudFiltered->height = 1;

	// Render 2D or 3D point cloud with inliers and outliers
	if (inliers.size()) {
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	} else {
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped()) {
		viewer->spinOnce();
	}
}
