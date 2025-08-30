# Lidar Obstacle Detection

This project implements a real-time lidar obstacle detection pipeline using C++ and the Point Cloud Library (PCL). It processes raw point cloud data to segment the road plane, cluster nearby obstacles, and visualize the results in 3D.

![alt text](obstacledetectionfps.png)

## Features

*   **Point Cloud Processing:** Efficiently load and filter raw PCD data.
*   **Point Cloud Segmentation:** Separate the road plane from obstacles using RANSAC.
*   **Clustering:** Group individual points into obstacles using Euclidean clustering.
*   **3D Bounding Boxes:** Render bounding boxes around detected obstacles.
*   **Real-time Visualization:** See the results in a dynamic 3D viewer.

## Dependencies

*   **C++14**
*   **PCL** (Point Cloud Library) 1.2
*   **CMake** 3.5 

## Building and Running

1.  **Clone the repository**
    
2.  **Create a build directory, compile and Run the obstacle detector**
    ```bash
    execute build.sh
    ```


