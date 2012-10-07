/* \author Geoffrey Biggs */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace footsteps {
    /*
     * Constants
     */
    struct Chirality {
        enum Enum{ left, right };
    };
    typedef struct Chirality Chirality;
}

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


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //------------------------------------
    //-----Add shapes at cloud points-----
    //------------------------------------
    viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
            cloud->points[cloud->size() - 1], "line");
    viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

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

    bool downsample(false), filterOutliers(false);
    if (pcl::console::find_argument(argc, argv, "-d") >= 0)
    {
        downsample = true;
    }
    if (pcl::console::find_argument (argc, argv, "-f") >= 0)
    {
        filterOutliers = true;
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
        reader.read(filename, *cloud_ptr); // todo: don't overwrite current cloud
        *cloud_ptr += *cloud_buffer_ptr;
    }

    // Create color copy
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_color_ptr->resize(cloud_ptr->size());
    for (size_t i = 0; i < cloud_ptr->size(); i++)
    {
        pcl::PointXYZRGB color_point(255, 255, 255);
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
     * Visualization
     * */

    // Create viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(cloud_color_ptr);


    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (1000);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

