#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h> 
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/hdl_grabber.h>

#include "parser.h"
#include "viewer.h"
#include "example.h"
#include "surface.h"
#include "utils.h"

using namespace std::chrono_literals;

void showExample(PARSE_STATUS parserStatus, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "Generating example point clouds.\n\n";
    generateExampleCloudPoints(basic_cloud_ptr, point_cloud_ptr);

    // ----------------------------------------------------------------
    // -----Calculate surface normals with a search radius of 0.05-----
    // ----------------------------------------------------------------
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
    generateSurface(point_cloud_ptr, cloud_normals1, 0.05);

    // ---------------------------------------------------------------
    // -----Calculate surface normals with a search radius of 0.1-----
    // ---------------------------------------------------------------
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    generateSurface(point_cloud_ptr, cloud_normals2, 0.1);
    viewerGeneration(viewer, parserStatus, basic_cloud_ptr, point_cloud_ptr, cloud_normals1, cloud_normals2);
}

void showRabbit(PARSE_STATUS parserStatus, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr rabbit_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_rabbit_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yangtx/Projects/PointCloud/data/rabbit/rabbit.pcd", *rabbit_cloud);

    createColorPointByDistance(rabbit_cloud, color_rabbit_cloud, 10.0);
    viewerGeneration(viewer, parserStatus, NULL, color_rabbit_cloud, NULL, NULL);
}

void showOffice(PARSE_STATUS parserStatus, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr office_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr office_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr office_cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr office_cloud4 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_office_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yangtx/Projects/PointCloud/data/office/office1.pcd", *office_cloud1);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yangtx/Projects/PointCloud/data/office/office2.pcd", *office_cloud2);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yangtx/Projects/PointCloud/data/office/office3.pcd", *office_cloud3);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yangtx/Projects/PointCloud/data/office/office4.pcd", *office_cloud4);

    *office_cloud1 += (*office_cloud2);
    *office_cloud1 += (*office_cloud3);
    *office_cloud1 += (*office_cloud4);

    createColorPointByDistance(office_cloud3, color_office_cloud, 2.5);
    viewerGeneration(viewer, parserStatus, NULL, color_office_cloud, NULL, NULL);
}

// void showHDL(PARSE_STATUS parserStatus, pcl::visualization::PCLVisualizer::Ptr viewer)
// {
//     pcl::HDLGrabber grabber ("", "/home/yangtx/Projects/PointCloud/data/HDL/Nottoway-Annex-track5-fast-1loop.pcap");
//     pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");
//     std::cout << "Frames Per Second:" << grabber.getFramesPerSecond() << std::endl;
// }

int main(int argc, char** argv) {
    // --------------------------------------
    //      Parse Command Line Arguments
    // --------------------------------------
    
    PARSE_STATUS parserStatus;
    parserStatus = checkParameters(argc, argv, parserStatus);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // --------------------------------------
    //          Create example
    // --------------------------------------
    // showExample(parserStatus, viewer);

    // --------------------------------------
    //         Read office.pcd
    // --------------------------------------
    // showOffice(parserStatus, viewer);
    showRabbit(parserStatus, viewer);

    // showHDL(parserStatus, viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
}

