#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/lexical_cast.hpp>

#include "footstep_visualizer.h"

extern "C" {
    #include "demo.h"
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tr2_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
Eigen::Matrix4f Ti2 = Eigen::Matrix4f::Identity (), targetToSource2;
boost::mutex mtx_;


/*
 * Visualization globals
 */

// Flag set by worker to tell visualizer to refresh data
volatile bool viewerNeedsRefresh_;
// Point cloud to show after refresh
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_refresh_ptr_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
// Footsteps to show after refresh
footsteps::FootstepVector footsteps_;
// Mutex to protect access to refresh globals
boost::mutex refreshMtx_;
// Output counter
int numOutputs_ = 0;

// Planar normals
float planar_normals[4] = {0.0f, -1.0f, 0.0f, 0.0f};

/*
 * Astar globals
 */
float terrain_value[TERRAIN_N_X][TERRAIN_N_Y];
float cost_map[TERRAIN_N_X][TERRAIN_N_Y];
int terrain_index[TERRAIN_N_X][TERRAIN_N_Y];
int terrain_count[TERRAIN_N_X][TERRAIN_N_Y];


/*
 * Downsample point cloud
 */
void
downsample (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &points, float leaf_size,
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &downsampled_out)
{
	pcl::VoxelGrid<pcl::PointXYZRGBA> vox_grid;
	vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	vox_grid.setInputCloud (points);
	vox_grid.filter (*downsampled_out);
}

float terrain_cost( int ix, int iy, float terrain_value[TERRAIN_N_X][TERRAIN_N_Y])
{

  int i, j;
  float sum, max, rough;
  float value[15];

  sum = 0;
  for(i=ix-1;i<ix+2;i++){
	  for(j=iy-2;j<iy+3;j++){
		if (i>=0 && j>=0 && i<TERRAIN_N_X && j<TERRAIN_N_Y)
		  value[5*(i-ix+1)+j-iy+2] = terrain_value[i][j];
		else
		  value[5*(i-ix+1)+j-iy+2] = 0;
		  sum += value[5*(i-ix+1)+j-iy+2];
	  }
  }
  max = 0;
  rough = 0;
  for(i=0;i<15;i++)
  {
	rough += fabsf(value[i]-sum/15.0);
	if (fabsf(value[i]-sum/15)>max)
		max = fabsf(value[i]-sum/15.0);
  }
  rough = rough/15;


  if (max>0.1)
	  return 1;
  else if (rough > 0.05)
	  return 1;
  else if (sum/15>0.2)
	  return 1;
  else
	  return (float) ((max+2*rough+sum/15));

}

void Get_Transformation_Matrix()
{
	int i, j, f1, f2;
	int data2[12][2];
	// =========== load the point cloud data =====================//
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_sc2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("cal_data2_2.pcd", *cloud_sc2) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		exit(-1);
	}
    // Read index file
	FILE *my_stream;
	my_stream=fopen("input2.txt", "r");
    if (my_stream==NULL){
		printf("Cannot open the file");
		fclose(my_stream);
    }
	else{
		for(int i=0; i<12; i++){
			fscanf(my_stream,"%d %d\n", &f1, &f2);
			data2[i][0]=f1;
			data2[i][1]=f2;
			//std::cout << data1[i][0] << " "<< data1[i][1] << std::endl;
		}
		fclose(my_stream);
	}
	// =============== Pre-process the point cloud data ========================//
	// Get the correct coordinate
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::Correspondences correspondences;

	// Fill the point cloud data
	  cloud_temp1->width    = 12;
	  cloud_temp1->height   = 1;
	  cloud_temp1->is_dense = false;
	  cloud_temp1->points.resize (cloud_temp1->width * cloud_temp1->height);
	  for (size_t k = 0; k < cloud_temp1->points.size (); ++k)
	  {
		cloud_temp1->points[k].x = cloud_sc2->points[640*(data2[k][1]-1)+data2[k][0]-1].x;
		cloud_temp1->points[k].y = cloud_sc2->points[640*(data2[k][1]-1)+data2[k][0]-1].y;
		cloud_temp1->points[k].z = cloud_sc2->points[640*(data2[k][1]-1)+data2[k][0]-1].z;
	  }
	// Fill the point cloud data 2
	  cloud_temp2->width    = 12;
	  cloud_temp2->height   = 1;
	  cloud_temp2->is_dense = false;
	  cloud_temp2->points.resize (cloud_temp2->width * cloud_temp2->height);
	  for (int k1= 0; k1 < 4; ++k1){
		  for (int k2=0; k2<3; ++k2){
			cloud_temp2->points[3*k1+k2].x = (9.3125*2.54*(k1-2))/100;
			cloud_temp2->points[3*k1+k2].y = 0.01;
			cloud_temp2->points[3*k1+k2].z = (60.375*2.54 + 9.25*2.54*(2-k2))/100;
		  }
	  }

	  correspondences.resize(cloud_temp1->size ());
	  for (int j = 0; j < cloud_temp2->points.size (); ++j)
	  {
		  correspondences[j].index_query = j;
		  correspondences[j].index_match = j;
		  double xx, yy, zz;
		  xx = (cloud_temp1->points[j].x-cloud_temp2->points[j].x)*(cloud_temp1->points[j].x-cloud_temp2->points[j].x);
		  yy = (cloud_temp1->points[j].y-cloud_temp2->points[j].y)*(cloud_temp1->points[j].y-cloud_temp2->points[j].y);
		  zz = (cloud_temp1->points[j].z-cloud_temp2->points[j].z)*(cloud_temp1->points[j].z-cloud_temp2->points[j].z);
		  correspondences[j].distance = xx+yy+zz;
	  }
	  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
	  svd.estimateRigidTransformation(*cloud_temp2, *cloud_temp1, correspondences, Ti2);
	  targetToSource2 = Ti2.inverse();
	  pcl::transformPointCloud (*cloud_sc2, *cloud_tr2_, targetToSource2);
}

void
cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
	boost::mutex::scoped_lock lock (mtx_);
	pcl::transformPointCloud (*cloud, *cloud_src_, targetToSource2);
  lock.unlock();
}

// Fit a plane to our data
// Used to determine the normals for each footstep point
pcl::ModelCoefficients::Ptr
plane_fit(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  return coefficients;
}

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
    void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  // fit a plane to the current data, then use the normal vector of that plane as the normal vector
  // for future footsteps
  if (event.getKeySym () == "m" && event.keyDown ())
  {
    std::cout << "m was pressed => fitting plane" << std::endl;


    // Fit a plane to the most recent data
    boost::mutex::scoped_lock lock (mtx_);
    pcl::ModelCoefficients::Ptr coefficients = plane_fit(cloud_src_);
    // Do something with the coefficents.
    std::cout << "Coefficients: " << coefficients << std::endl;

    planar_normals[0] = coefficients->values[1];
    planar_normals[1] = coefficients->values[2];
    planar_normals[2] = coefficients->values[3];

    lock.unlock();

    text_id = 0;
  }
  // write data to file
  if (event.getKeySym () == "y" && event.keyDown ())
  {
    boost::mutex::scoped_lock lock (refreshMtx_);
    pcl::io::savePCDFile(std::string("out") + boost::lexical_cast<std::string>(numOutputs_) + std::string(".pcd"), *cloud_refresh_ptr_);
    std::cout << "Wrote cloud to `out" << numOutputs_ << ".pcd`!" << std::endl;

    std::string footsteps_filename = std::string("./footsteps") + boost::lexical_cast<std::string>(numOutputs_) + std::string(".txt");
    std::ofstream output_file((char *)footsteps_filename.c_str());
    for (footsteps::FootstepVector::const_iterator i = footsteps_.begin(); i != footsteps_.end(); ++i)
      output_file << i->getPoint().data[0] << " " << i->getPoint().data[1] << " " << i->getPoint().data[2] << std::endl;


    std::cout << "Wrote footsteps to `footsteps" << numOutputs_ << ".txt`!" << std::endl;
    numOutputs_++;
    lock.unlock();
  }
  // Output camera parameters
  if (event.getKeySym () == "C" && event.keyDown ())
  {
    std::vector<pcl::visualization::Camera> cameras;

    viewer->getCameras(cameras);
    pcl::visualization::Camera camera = cameras[0];
    std::cout << "Pos: " << camera.pos[0] << "," << camera.pos[1] << "," << camera.pos[2] << std::endl;
    std::cout << "View: " << camera.view[0] << "," << camera.view[1] << "," << camera.view[2] << std::endl;
  }
}


void visualizerFunc()
{
  boost::shared_ptr<footsteps::FootstepVisualizer> viewer;
	// Create visualizer

  viewer = boost::shared_ptr<footsteps::FootstepVisualizer>(new footsteps::FootstepVisualizer ("Viewer"));

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters();

  viewer->setCameraPosition(0.456335f, -4.92199f, -3.07703f, -0.193089f, -0.680615f, 0.706739f);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

	while(!viewer->wasStopped())
	{
    // Refresh data if necessary
    if (viewerNeedsRefresh_)
    {
      viewerNeedsRefresh_ = false;
      boost::mutex::scoped_lock lock (refreshMtx_);
      // --------------------------------------------
      // -----Remove old point cloud and add point cloud-----
      // --------------------------------------------
      const std::string pc_id = "cloud";
      if (!viewer->updatePointCloud(cloud_refresh_ptr_, pc_id))
      {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_refresh_ptr_);
        viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_refresh_ptr_, rgb, pc_id);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, pc_id);
      }

      // -------------------------------------------------------
      // ------Remove old footsteps and add new footsteps-------
      // -------------------------------------------------------
      const std::string fs_id = "footsteps";
      viewer->removeFootsteps(fs_id);
      viewer->addFootsteps(footsteps_, fs_id);

      lock.unlock();
    }

    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds( 100000));
	}
}

void workerFunc()
{
	int i, j, loc_i, loc_j, count, input1;
	int ix, iy, LL;
	float kx, ky, sum;
	float temp[TERRAIN_N_X][TERRAIN_N_Y];
	FILE * pFile;
	boost::posix_time::seconds workTime(3);
	// Downsample the cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGBA>);
    const float voxel_grid_leaf_size = 0.01;
    boost::this_thread::sleep(workTime);

	while(true){
    boost::mutex::scoped_lock lock (mtx_);
    downsample (cloud_src_, voxel_grid_leaf_size, downsampled);
    lock.unlock();
    LL = downsampled->height*downsampled->width;
    for (i=0;i<LL;i++)
    {
      if (downsampled->points[i].x != 0){
        terrain_indices( t1, downsampled->points[i].x+10, downsampled->points[i].z+10, &ix, &iy, NULL );
        terrain_value[TERRAIN_N_X-1-ix][iy] = terrain_value[TERRAIN_N_X-1-ix][iy] + downsampled->points[i].y;
        terrain_count[TERRAIN_N_X-1-ix][iy] = terrain_count[TERRAIN_N_X-1-ix][iy] + 1;
        terrain_index[TERRAIN_N_X-1-ix][iy] = i;

      }
    }
    for (i=0 ; i<TERRAIN_N_X ; i++){
      for (j=0 ; j<TERRAIN_N_Y ; j++){
        if (terrain_count[i][j]!=0)
          terrain_value[i][j] = terrain_value[i][j]/terrain_count[i][j];
        temp[i][j] = terrain_value[i][j];
      }
    }

    for (i=0 ; i<TERRAIN_N_X ; i++){
      for (j=0 ; j<TERRAIN_N_Y ; j++){
        if (temp[i][j]==0){
          count = 0;
          sum = 0;
          for (loc_i=i-4; loc_i<i+2; loc_i++){
            for (loc_j=j-4; loc_j<j+2; loc_j++){
              if (loc_i>=0&&loc_j>=0&&loc_i<TERRAIN_N_X&&loc_j<TERRAIN_N_X){
                if (temp[loc_i][loc_j]!=0){
                  sum = sum + temp[loc_i][loc_j];
                  count = count + 1;
                }
              }
            }
          }
          if (count>0)
            terrain_value[i][j] = sum/count;
        }
      }
    }


    for (i=0 ; i<TERRAIN_N_X ; i++)
      for (j=0 ; j<TERRAIN_N_Y ; j++)
        cost_map[i][j] = terrain_cost( i,j, terrain_value);
    /*
    // Save to teh file
    pFile = fopen ("cost_map.txt","w");
    for (i=0 ; i<TERRAIN_N_X ; i++){
	   for (j=0 ; j<TERRAIN_N_Y ; j++){
		   if (j==TERRAIN_N_Y-1)
			fprintf (pFile, "%.4f\n",cost_map[i][j]);
		   else
			fprintf (pFile, "%.4f ",cost_map[i][j]);
	   }
   }
   fclose (pFile);
    std::cout << "range ";
    std::cin >> input1;
*/

	/*
	* Astar
	*/

	// Find the path based on Astar
	//generate_true_cost_map( t1, cost_map);
  //a1 = create_astar( t1 );

  //plan();
  //printf_final_path( p1 );

	// Generate footsteps
	footsteps::FootstepVector steps;

  // get indices of target pcl points
  while (p1 != NULL)
  {
    terrain_indices( t1, p1->state[XX], p1->state[YY], &ix, &iy, NULL);
    // footstep parameters
    pcl::PointXYZ target_pt = pcl::PointXYZ(p1->state[XX] - 10.f, 0.0f, p1->state[YY] - 10.f);

    float rotation = p1->state[ANGLE] - 3.14f/2.0f;
    footsteps::Chirality::Kind chirality = (footsteps::Chirality::Kind)p1->state[SIDE];

    // create new point to be footstep target
    pcl::PointNormal pt_nrm;
    memcpy(pt_nrm.data, target_pt.data, 3 * sizeof(float)); // copy xyz

    // Dummy normal vector
    memcpy(pt_nrm.data_n, planar_normals, 3 * sizeof(float));
    footsteps::Footstep footstep (pt_nrm, rotation, chirality);
      steps.push_back(footstep);
    p1 = p1->previous;
  }

  // Update refresh global variables
  {
    boost::mutex::scoped_lock lock(refreshMtx_);
    pcl::copyPointCloud(*downsampled, *cloud_refresh_ptr_);
    footsteps_ = steps;
    viewerNeedsRefresh_ = true;
    lock.unlock();
  }
  }
}

int
main (int argc, char** argv)
{
	int i, j, input1;

	// =============== Creat an empty Terrain size 25mx25m=================//
	t1 = create_terrain();
	for (i=0;i<TERRAIN_N_X;i++){
		for (j=0;j<TERRAIN_N_Y;j++){
			terrain_value[i][j] = 0;
			terrain_count[i][j] = 0;
			terrain_index[i][j] = 0;
		}
	}
    t1->min[XX] = 0.0;
    t1->min[YY] = 0.0;
    t1->max[XX] = 25.0;
    t1->max[YY] = 25.0;

	Get_Transformation_Matrix();

	//std::cout << "Select data number: ";
	//std::cin >> input1;

	//=================== Get data from Camera ================================//
	pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>
          f = boost::bind(&cloud_cb_, _1);
	boost::signals2::connection c = interface->registerCallback (f);
	interface->start();

	//std::cout << "Select data number: ";
	//std::cin >> input1;

	// Threading
	boost::thread workerThread(workerFunc);
	boost::thread visualizerThread(visualizerFunc);

	boost::thread_group group;
	group.add_thread(&workerThread);
	group.add_thread(&visualizerThread);

	group.join_all();

  std::cout << "Select data number: ";
  std::cin >> input1;
  interface->stop();
  return (0);
}
