#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "footstep_visualizer.h"

extern "C" {
    #include "demo.h";
}


boost::shared_ptr<footsteps::FootstepVisualizer> viewer;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tr2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
Eigen::Matrix4f Ti2 = Eigen::Matrix4f::Identity (), targetToSource2;
boost::mutex mtx_; 

float terrain_value[TERRAIN_N_X][TERRAIN_N_Y];
float cost_map[TERRAIN_N_X][TERRAIN_N_Y];
int terrain_index[TERRAIN_N_X][TERRAIN_N_Y];
int terrain_count[TERRAIN_N_X][TERRAIN_N_Y];


/*
 * Show point cloud
 */

void
 showCloud (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Remove old point cloud and add point cloud-----
  // --------------------------------------------
const std::string pc_id = "cloud";
if (!viewer->updatePointCloud(cloud, pc_id))
	{
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, pc_id);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, pc_id);
	}
}

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
	  pcl::transformPointCloud (*cloud_sc2, *cloud_tr2, targetToSource2);
}

void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud){ 

	//const Eigen::Vector3f translate (0, 0.0, 0.0);
	//const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
	//pcl::transformPointCloud (*cloud, *cloud_src, translate, no_rotation);
	boost::mutex::scoped_lock lock (mtx_);
	pcl::transformPointCloud (*cloud, *cloud_src, targetToSource2);
	//boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > pointer(cloud_src);
    //viewer.showCloud(pointer);
} 

void visualizerFunc()
{
	while(!viewer->wasStopped())
	{
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

	while(1){
	boost::mutex::scoped_lock lock (mtx_);
	downsample (cloud_src, voxel_grid_leaf_size, downsampled);
	lock.unlock();
	LL = downsampled->height*downsampled->width;
	for (i=0;i<LL;i++)
	{
		if (downsampled->points[i].x!=NULL){
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
 * Compute normals
 */
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud (downsampled);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    
    // Compute the features
    ne.compute (*cloud_normals_ptr);

	// Generate footsteps
	footsteps::FootstepVector steps;

	/*
	* Astar
	*/

	// Find the path based on Astar	
	generate_true_cost_map( t1, cost_map);
	a1 = create_astar( t1 );

	plan();
	printf_final_path( p1 );

	for ( ; ; )
    {
      if ( p1 == NULL )
		break;
			terrain_indices( t1, p1->state[XX], p1->state[YY], &ix, &iy, NULL);
			if (terrain_index[ix][iy]!=NULL){
				// footstep parameters
				int index = terrain_index[ix][iy];
				pcl::PointXYZRGBA target_pt = downsampled->points[index];
				
				pcl::Normal target_normal = (*cloud_normals_ptr)[index];
				float rotation = p1->state[ANGLE];
				footsteps::Chirality::Kind chirality = (footsteps::Chirality::Kind)p1->state[SIDE];
				
				// create new point to be footstep target
				pcl::PointNormal pt_nrm;
				memcpy(pt_nrm.data, target_pt.data, 3 * sizeof(float)); // copy xyz
				memcpy(pt_nrm.data_n, target_normal.data_n, 3 * sizeof(float));
			
				footsteps::Footstep footstep (pt_nrm, rotation, chirality);
				steps.push_back(footstep);
	  }
      p1 = p1->previous;
    }
	p1 = NULL;
	boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > pointer(downsampled);
	
	std::cout << "SIZE:::::::::::::::::" << steps.size() << std::endl;
	for (footsteps::FootstepVector::iterator it = steps.begin(); it != steps.end(); ++it)
	{
		std::cout << "STEP POINT" << it->getPoint() << std::endl;
	}

	// remove old footsteps and add new footsteps
	const std::string fid = "footsteps";
	//viewer->removeFootsteps(fid);
	viewer->addFootsteps(steps, fid);
	std::cout << "Showing cloud" << std::endl;
    showCloud(pointer);
	std::cout << "Showed cloud" << std::endl;

     //std::stringstream s; 
     //s << "test.pcd"; 
	 //pcl::io::savePCDFileASCII(s.str(), *downsampled); 
	}
}

int
main (int argc, char** argv)
{
	// Create visualizer

  viewer = boost::shared_ptr<footsteps::FootstepVisualizer>(new footsteps::FootstepVisualizer ("Viewer"));

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters();
	int i, j, input1;

	// =============== Creat an empty Terrain size 25mx25m=================//
	t1 = create_terrain();
	for (i=0;i<TERRAIN_N_X;i++){
		for (j=0;j<TERRAIN_N_Y;j++){
			terrain_value[i][j] = 0;
			terrain_count[i][j] = 0;
			terrain_index[i][j] = NULL;
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
	std::cout << "Start Main Thread " << std::endl;
//	workerThread.join();
	boost::thread_group group;
	group.add_thread(&workerThread);
	group.add_thread(&visualizerThread);
 
	group.join_all();
	
  std::cout << "Select data number: ";
  std::cin >> input1;
  interface->stop(); 
  return (0);
}
