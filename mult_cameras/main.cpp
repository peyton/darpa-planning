/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  Multiple Cameras - minimal example
 *
 *        Version:  1.0
 *        Created:  10/14/2012 01:38:01 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Peyton Randolph (),
 *   Organization:  Carnegie Mellon University
 *
 * =====================================================================================
 */

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

void
printUsage (const char* prog_name)
{
  std::cout << "\n\nUsage: "<<prog_name<<" [options]\n\n"
    << "Options:\n"
    << "--------------------------------------\n"
    << "-h          this help\n"
    << "-n          number of cameras\n"
    << "\n\n";
}

class SimpleOpenNIViewer
{
  protected:
    int num_cameras_;

  public:
    SimpleOpenNIViewer (int num_cameras = 1) : viewer ("PCL OpenNI Viewer") {
      num_cameras_ = num_cameras;
    }

    void cloud_cb_2_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      //do nothing...
    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      //show cloud
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    }

    void run ()
    {
      //* *********************************
      //start first kinect
      pcl::Grabber* interface = new pcl::OpenNIGrabber("#1");

      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

      interface->registerCallback (f);
      interface->start();
      //* *********************************

      //* *********************************
      //start second kinect
      pcl::Grabber* interface2 = new pcl::OpenNIGrabber("#2");

      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f2 =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_2_, this, _1);

      interface2->registerCallback (f2);

      interface2->start();
      //* *********************************


      while (!viewer.wasStopped())
      {
        sleep (1);
      }

      interface->stop ();
      interface2->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};

int
main (int argc, char** argv)
{
  // Default values
  int num_cameras(1);

  /*
   * Parse command line arguments
   */
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }

  // Number of cameras
  if (pcl::console::find_argument(argc, argv, "-n") >= 0)
  {
    pcl::console::parse_argument(argc, argv, "-n", num_cameras);
  }

  SimpleOpenNIViewer v (num_cameras);
  v.run ();
  return 0;
}
