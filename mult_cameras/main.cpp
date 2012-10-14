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

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

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

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}
