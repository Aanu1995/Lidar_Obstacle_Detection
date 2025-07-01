// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <filesystem>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer){

    Car egoCar {Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar"};
    Car car1 {Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1"};
    Car car2 {Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2"};
    Car car3 {Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3"};

    std::vector<Car> cars {egoCar, car1, car2, car3};

    if(renderScene){
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer){
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Create lidar sensor
    auto lidar = std::make_unique<Lidar>(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {lidar->scan()};

    // renderRays(viewer, lidar->position, cloud);
    // renderPointCloud(viewer, cloud, "pointsCloud");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor {};

    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlaneUsingRansac(cloud, 1000, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(cloud, 1000, 0.2);

    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 4, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(const auto& cluster: clusters){
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
  	    renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);

        // Box box = pointProcessor->BoundingBox(cluster);
        // renderBox(viewer, box, clusterId);

        BoxQ boxq = pointProcessor.BoundingBoxQ(cluster);
        renderBox(viewer, boxq, clusterId);

        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer){

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    switch(setAngle) {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block   -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI> pointProcessor {};
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // Filter the input cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor.FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(35, 6, 2, 1));

    // Segment the filtered cloud into obstacles and road
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlaneUsingRansac(filteredCloud, 50, 0.2);
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filteredCloud, 50, 0.2);
    renderPointCloud(viewer, segmentCloud.second, "roadCloud", Color(0, 1, 0));

    // Clustering the obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessor.EuclideanClustering(segmentCloud.first, 0.5, 10, 500);
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessor.Clustering(segmentCloud.first, 0.5, 10, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(const auto& cluster: clusters){
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}


void streamCityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block   -----
    // ----------------------------------------------------

    // Filter the input cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI.FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(35, 6, 2, 1));

    // Segment the filtered cloud into obstacles and road
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(filteredCloud, 50, 0.2);
    renderPointCloud(viewer, segmentCloud.second, "roadCloud", Color(0, 1, 0));

    // Clustering the obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI.Clustering(segmentCloud.first, 0.5, 10, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(const auto& cluster: clusters){
        // std::cout << "cluster size ";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

int main (int argc, char** argv){
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer {new pcl::visualization::PCLVisualizer ("3D Viewer")};
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // simpleHighway(viewer);
    // cityBlock(viewer);

    // Create point processor
    ProcessPointClouds<pcl::PointXYZI> pointProcessor {};

    std::vector<std::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    while (!viewer->wasStopped ()){
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointProcessor.loadPcd((*streamIterator).string());
        streamCityBlock(viewer, pointProcessor, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end()){
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    }
}
