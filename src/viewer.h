#ifndef VIEWER_
#define VIEWER_

#include <pcl/visualization/pcl_visualizer.h>
#include "parser.h"

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void);

void viewerGeneration(pcl::visualization::PCLVisualizer::Ptr viewer, PARSE_STATUS parserStatus,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr,
                      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1,
                      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2
);

#endif