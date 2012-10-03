#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

extern "C" {
    #include "demo.h";
}

int
main (int argc, char** argv)
{
  int input1, input2;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

    ASTAR path_node;

  std::cout << "Select data number: ";
  std::cin >> input1;

  for (int i=0;i<18;++i)
  {
//	  for (int j=0;j<4;j++)
//	  {
	      int j = 2;
		  std::stringstream s; 
		  //s << "D:/Point cloud data/valve/data" << 1+i <<"_" << j << ".pcd"; // 
		  s << "step" << input1 <<"_8.pcd";
		  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (s.str(), *cloud2) == -1) //* load the file
		  {
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			return (-1);
		  }
/*
		   std::stringstream s2; 
		   s2 << "D:/Point cloud data/valve/M_data" << 1+i <<"_" << j << ".pcd"; // 
		   pcl::io::savePCDFileASCII(s2.str(), *cloud2);
*/
//	  }
  }
   
   pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
   viewer.showCloud(cloud2);
   while (!viewer.wasStopped())
   {
	   sleep(1000);
   }
   
  return (0);
}
