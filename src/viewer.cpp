#include "viewer.h"

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
      std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

      char str[512];
      sprintf (str, "text#%03d", text_id ++);
      viewer->addText ("clicked here", event.getX (), event.getY (), str);
    }
}

void viewerGeneration(pcl::visualization::PCLVisualizer::Ptr viewer, PARSE_STATUS parserStatus,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr,
                      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1,
                      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2
)
{
    if (parserStatus.rgb && point_cloud_ptr)
    {
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
        viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
    }
    else if (parserStatus.simple && basic_cloud_ptr)
    {
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
    }
    else if (parserStatus.custom_c)
    {
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(basic_cloud_ptr, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, single_color, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
    }
    else if (parserStatus.normals)
    {
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
        viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point_cloud_ptr, cloud_normals2, 10, 0.05, "normals");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
    }
    else if (parserStatus.shapes)
    {
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
        viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();

        //------------------------------------
        //-----Add shapes at cloud points-----
        //------------------------------------
        viewer->addLine<pcl::PointXYZRGB> ((*point_cloud_ptr)[0],
                                            (*point_cloud_ptr)[point_cloud_ptr->size() - 1], "line");
        viewer->addSphere ((*point_cloud_ptr)[0], 0.2, 0.5, 0.5, 0.0, "sphere");

        //---------------------------------------
        //-----Add shapes at other locations-----
        //---------------------------------------
        pcl::ModelCoefficients coeffs;
        coeffs.values.push_back (0.0);
        coeffs.values.push_back (0.0);
        coeffs.values.push_back (1.0);
        coeffs.values.push_back (0.0);
        viewer->addPlane (coeffs, "plane");
        coeffs.values.clear ();
        coeffs.values.push_back (0.3);
        coeffs.values.push_back (0.3);
        coeffs.values.push_back (0.0);
        coeffs.values.push_back (0.0);
        coeffs.values.push_back (1.0);
        coeffs.values.push_back (0.0);
        coeffs.values.push_back (5.0);
        viewer->addCone (coeffs, "cone");
    }
    else if (parserStatus.viewports)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters ();

        int v1(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor (0, 0, 0, v1);
        viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
        viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud1", v1);

        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(point_cloud_ptr, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, single_color, "sample cloud2", v2);

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
        viewer->addCoordinateSystem (1.0);

        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point_cloud_ptr, cloud_normals1, 10, 0.05, "normals1", v1);
        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point_cloud_ptr, cloud_normals2, 10, 0.05, "normals2", v2);
    }
    else if (parserStatus.interaction_customization)
    {
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);

        viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
        viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());
    }
}

