/* \author Geoffrey Biggs */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "footstep_visualizer.h"

// --------------
// -----Help-----
// --------------
  void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
    << "Options:\n"
    << "-------------------------------------------\n"
    << "-h           this help\n"
    << "-s           Simple visualisation example\n"
    << "-r           RGB colour visualisation example\n"
    << "-c           Custom colour visualisation example\n"
    << "-n           Normals visualisation example\n"
    << "-a           Shapes visualisation example\n"
    << "-v           Viewports example\n"
    << "-i           Interaction Customization example\n"
    << "-f           Custom file input\n"
    << "\n\n";
}


boost::shared_ptr<footsteps::FootstepVisualizer> footstepVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<footsteps::FootstepVisualizer> viewer (new footsteps::FootstepVisualizer ("Footstep Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters();

  if (normals)
  {
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  }

  return (viewer);
}

// --------------
// -----Main-----
// --------------
  int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }

  bool calculateNormals(false), downsample(false), filterOutliers(false);
  if (pcl::console::find_argument(argc, argv, "-d") >= 0)
  {
    downsample = true;
  }
  if (pcl::console::find_argument (argc, argv, "-f") >= 0)
  {
    filterOutliers = true;
  }
  if (pcl::console::find_argument (argc, argv, "-n") >= 0)
  {
    calculateNormals = true;
  }

  /*
   * Data loading
   */

  // Create cloud pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  // Load passed-in files
  std::vector<int> file_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
  if (file_indices.empty())
  {
    std::cout << "No file input" << std::endl;
    return 0;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_buffer_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  for (std::vector<int>::iterator it = file_indices.begin(); it < file_indices.end(); it++)
  {
    std::string filename = (std::string)argv[file_indices[*it] + 1];
    std::cout << "Loading point cloud named " << filename << "\n\n";
    reader.read(filename, *cloud_buffer_ptr); // todo: don't overwrite current cloud
    *cloud_ptr += *cloud_buffer_ptr;
  }

  // Create color copy
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_color_ptr->resize(cloud_ptr->size());
  for (size_t i = 0; i < cloud_ptr->size(); i++)
  {
    pcl::PointXYZRGB color_point(90, 90, 90);
    pcl::PointXYZ point = cloud_ptr->points[i];

    color_point.x = point.x;
    color_point.y = point.y;
    color_point.z = point.z;

    // white color
    cloud_color_ptr->points[i].x = cloud_ptr->points[i].x;
    cloud_color_ptr->points[i] = color_point;
  }

  /*
   * Post processing
   */

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (downsample)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_color_ptr);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered_ptr);

    cloud_color_ptr = cloud_filtered_ptr;
  }

  if (filterOutliers)
  {

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_color_ptr);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered_ptr);

    cloud_color_ptr = cloud_filtered_ptr;
  }

  /*
   * Computing normals
   */

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  if (calculateNormals)
  {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered_ptr);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    
    // Compute the features
    ne.compute (*cloud_normals_ptr);
  }

  /* 
   * Visualization
   */

  // Create viewer
  boost::shared_ptr<footsteps::FootstepVisualizer> viewer;
  viewer = footstepVis(cloud_color_ptr, cloud_normals_ptr);

  // Generate random footsteps
  footsteps::FootstepVector steps;
  
  for (int i = 0; i < 10; i++)
  {
    pcl::PointXYZRGB target_pt = (*cloud_color_ptr)[rand() % cloud_color_ptr->size()];
    pcl::PointNormal pt;
    pt.x = target_pt.x; pt.y = target_pt.y; pt.z = target_pt.z;
    footsteps::Footstep footstep (pt, (footsteps::Chirality::Kind)(rand() % 2));
    steps.push_back (footstep);
  }
  
  viewer->addFootsteps(steps);

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (1000);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

