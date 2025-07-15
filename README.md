# Lidar Obstacle Detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

A comprehensive 3D object detection system using LiDAR point cloud data, implemented as part of the Udacity Sensor Fusion Nanodegree program. This project demonstrates real-time obstacle detection and tracking in autonomous driving scenarios through custom implementations of fundamental computer vision algorithms.

## Project Overview

This project implements a complete LiDAR-based obstacle detection pipeline that processes point cloud data to identify and track vehicles and other obstacles in a 3D environment. The system is designed to work with real-world LiDAR data and provides robust detection capabilities suitable for autonomous driving applications.

### Key Features

- **Real-time Point Cloud Processing**: Efficient filtering and preprocessing of LiDAR point cloud data
- **Custom 3D RANSAC Implementation**: Ground plane segmentation using a from-scratch RANSAC algorithm
- **KD-Tree Data Structure**: Custom 3D KD-Tree for efficient nearest neighbor searches
- **Euclidean Clustering**: Custom clustering algorithm for object detection and grouping
- **Bounding Box Generation**: Automatic generation of 3D bounding boxes around detected objects
- **Multi-frame Object Tracking**: Consistent detection and tracking of objects across video frames

## Technical Implementation

### Core Algorithms

#### 1. 3D RANSAC Plane Segmentation

- **Purpose**: Separates ground plane from obstacle points
- **Implementation**: Custom RANSAC algorithm that iteratively fits planes to point cloud data
- **Key Features**:
  - Handles 3D point cloud data
  - Configurable iteration count and distance threshold
  - Robust to outliers and noise

#### 2. KD-Tree for Spatial Indexing

- **Purpose**: Efficient spatial data structure for nearest neighbor searches
- **Implementation**: Custom 3D KD-Tree with insert and search operations
- **Key Features**:
  - Balanced binary tree structure
  - O(log n) search complexity
  - Supports 3D euclidean distance queries

#### 3. Euclidean Clustering

- **Purpose**: Groups nearby points into distinct object clusters
- **Implementation**: Custom clustering algorithm using KD-Tree for neighbor searches
- **Key Features**:
  - Configurable cluster size limits
  - Distance-based grouping
  - Efficient cluster formation and validation

### Processing Pipeline

1. **Point Cloud Loading**: Read LiDAR data from PCD files
2. **Filtering**: Remove noise and limit processing region
3. **Ground Segmentation**: Separate ground plane using RANSAC
4. **Clustering**: Group obstacle points into distinct objects
5. **Bounding Box Generation**: Create 3D boxes around detected objects
6. **Visualization**: Render results with color-coded objects

## Project Structure

```text
SFND_Lidar_Obstacle_Detection/
├── src/
│   ├── environment.cpp          # Main application and visualization
│   ├── processPointClouds.cpp   # Core point cloud processing algorithms
│   ├── processPointClouds.h     # Point cloud processing header
│   ├── kdtree.h                 # Custom KD-Tree implementation
│   ├── render/                  # Visualization and rendering utilities
│   ├── sensors/                 # LiDAR sensor simulation and data
│   └── quiz/                    # Algorithm development and testing
├── build/                       # Build directory
├── media/                       # Demo images and videos
└── CMakeLists.txt              # Build configuration
```
