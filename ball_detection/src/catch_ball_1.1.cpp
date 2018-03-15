/***********************************************************************************
Name:           catch_ball.cpp
Revision:
Date:           06-03-2013
Author:         Paulo Dias
Comments:       adaptation of catch_baal to work with tf
images
Revision:
Libraries:
Notes:          Code generated under Ubuntu using openCV, VTK and PCL (for reading clouds)
		Compiled using CMake
***********************************************************************************/

// General includes
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

// other includes
#include "utils.h"
#include "CGrid_3D.h"
#include "CView_3D.h"
#include "CView_2D.h"
#include "CTrajectory.h"

// ROS
#include <pcl_ros/point_cloud.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>

// Message publication
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

// tf
 #include <tf/transform_listener.h>

// size of the grid for point counting
double gridsize = BALLRADIUS * 1.5;

// Minimum points inside template
int min_points = 50;

// height fro roots detection
double roots_height = 0.40;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// For publishing info
ros::Publisher pub;

// Global variable for rendering acess
CView_3D    *my_view_3D;
CView_2D    *my_view_2D;
CTrajectory *my_trajectory;

Eigen::Matrix4f RBT_pcl;

int ***mask;
int ***mask_ground;

int im_num = 0;
int ground_detect = 0;
int air_detect = 0;

//int debug = DEBUG + DEBUG_2D + DEBUG_3D + DEBUG_MASK + DEBUG_TRAJECTORY;
//              1	4	     8		16		32 
int debug = DEBUG + DEBUG_2D + DEBUG_3D + DEBUG_TRAJECTORY;
int ros_com = 1;

// tf stuff
tf::TransformListener *listener;
tf::StampedTransform transform;
  

/*************************************************************************************************
* Compute Ground Plane and return transform to align axis with ground
* if debug show plane in vtkRenderWindow
*************************************************************************************************/
void compute_ground_plane_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f *RBT_pcl, int debug)
{
  std::vector<int> inliers;

  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
  model_l(new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
	
  Eigen::VectorXf model_coefficients(4);
      
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
  ransac.setDistanceThreshold (0.05);
  ransac.computeModel();
  ransac.getInliers(inliers);
  
  model_l->optimizeModelCoefficients(inliers,model_coefficients,model_coefficients);
  
  double nx,ny,nz,p;
  double a = model_coefficients[0];
  double b = model_coefficients[1];
  double c = model_coefficients[2];
  double d = model_coefficients[3];
  nx = a / sqrt(a*a + b*b + c*c);
  ny = b / sqrt(a*a + b*b + c*c);
  nz = c / sqrt(a*a + b*b + c*c);
  p = -d / sqrt(a*a + b*b + c*c);
  
  // Compute Rigid Body transform between plane axis and coordinate system axes
  pcl::PointCloud<pcl::PointXYZ> source_points;
  pcl::PointCloud<pcl::PointXYZ> target_points;

  target_points.push_back (pcl::PointXYZ (0,0,0));
  target_points.push_back (pcl::PointXYZ (1,0,0)); 
  target_points.push_back (pcl::PointXYZ (0,1,0));
      
  source_points.push_back (pcl::PointXYZ (nx*p,ny*p,nz*p));
  source_points.push_back (pcl::PointXYZ (nx*p+1,ny*p+0,nz*p+0)); 
  source_points.push_back (pcl::PointXYZ (nx*p+nx,ny*p+ny,nz*p+nz)); 

  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_pcl;
  trans_pcl.estimateRigidTransformation(source_points,target_points,*RBT_pcl);   
}

/*************************************************************************************************
* ROS Callback
*************************************************************************************************/
void Callback(const PointCloud::ConstPtr& cloud_in)
{  
  srand ( time(NULL) );

  clock_t clock_start=clock();
  
  // 3D coordinates of ball
  double ball_centre_coord3D[3];
  double ball_centre_pixel2D[3];
  int ball_grid_index[3];
  ball_grid_index[0] = 0;ball_grid_index[1] = 0;ball_grid_index[2] = 0;
  
  double previous_ball[3];
  
  double roots[3];
  double root_1_coords[3];
  double root_2_coords[3];
  root_1_coords[0] = root_1_coords[1] = root_1_coords[2];
  root_2_coords[0] = root_2_coords[1] = root_2_coords[2];
      
  im_num++;
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud_in, *cloud);
  
  /////////////////////////////////
  // aligning to plane
  if (read_RBT(&RBT_pcl,"calib.xml")!=1)
  {
    printf("\nCan't read calib.xml");
    getchar();
    exit (0);
  }
  
  ///////////////////////////////////
  // Transform point cloud to align xOy Plane withcompute_grid(cloud,ptr,gridnode,gridsize,xdim,ydim,zdim,min_pt,max_pt); ground plane
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud,*cloud,RBT_pcl);
  
  
  /////////////////////////////////////////
  // Remove points above given distance and below ground
  /////////////////////////////////////////
  for (unsigned int i=0;i<cloud->height;i++)
    for (unsigned int j=0;j<cloud->width;j++)
    {
      unsigned int index_image = i * cloud->width + j;
      
      if (cloud->points[index_image].x > DEPTH_LIMIT)
      {
	cloud->points[index_image].x = sqrt(-1.0); // NaN
	cloud->points[index_image].y = sqrt(-1.0);
	cloud->points[index_image].z = sqrt(-1.0);
      }
    }

  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D (*cloud, min_pt, max_pt);
  //printf("\n min %f %f %f max %f %f %f",min_pt[0],min_pt[1],min_pt[2],max_pt[0],max_pt[1],max_pt[2]);
  // Case of all points are Nan (object in front kinect
  // Not ok still some problems
  //if (isnan(min_pt[0]))
  if (min_pt[0] >= max_pt[0] || min_pt[1] >= max_pt[1] || min_pt[2] >= max_pt[2] )
    return;
  

    
  ///////////////////////////////////
  // Create 3D scene with first imageView cloud with VTK
  if ((im_num==1) && (debug & DEBUG_3D))
  {
    // show point cloud
    my_view_3D->show_point_cloud(cloud,125,125,125);
    my_view_3D->start_interaction();
  }
  
  //////////////
  // start Video
  if ((im_num==1) && (debug & DEBUG_2D))
    my_view_2D = new CView_2D("out.avi",cloud);
       
  /////////////////////////////////////
  // Create Depth image from pointcloud
  if (debug & DEBUG_2D)
    my_view_2D->update_image(cloud,min_pt,max_pt);
    
  ///////////////////////////////////////////////////////////////////////
  // Grid definition (counting number of points in each voxel of the grid)
  CGrid_3D *my_grid = new CGrid_3D(gridsize,min_pt,max_pt); 
  my_grid->compute_grid(cloud);
  
  //////////////////////////////////////////////////////
  // Ball detection UFO
  ball_centre_coord3D[0]=0; ball_centre_coord3D[1]=0; ball_centre_coord3D[2]=0;
  ball_centre_pixel2D[0]=0; ball_centre_pixel2D[1]=0;
  
  // Check if previous_ball support trajectory, if so use it
  if (my_trajectory->trajecto_pose[HISTORY-1] == 1)
  {
    previous_ball[0] = my_trajectory->previous_pose[HISTORY-1][0];
    previous_ball[1] = my_trajectory->previous_pose[HISTORY-1][1];
    previous_ball[2] = my_trajectory->previous_pose[HISTORY-1][2];
  }
  else
  {
    previous_ball[0] = 0;
    previous_ball[1] = 0;
    previous_ball[2] = 0;
  }
    
  my_grid->detect_ball(min_points,mask,MASKSIZE,
		      ball_centre_coord3D,ball_centre_pixel2D,
		      ball_grid_index,
		      0, my_grid->xdim-MASKSIZE,
		      0, my_grid->ydim-MASKSIZE,
		      2, my_grid->zdim-MASKSIZE,
		      previous_ball);
  // Ball detection UFO
  //////////////////////////////////////////////////////

  if (ball_centre_coord3D[0] !=0)
  {
    air_detect++;
    if (debug & DEBUG)
      printf("\n%3d - Flying ball - %6f %6f %6f",im_num,ball_centre_coord3D[0],ball_centre_coord3D[1],ball_centre_coord3D[2]);
  }
 
  //////////////////////////////////////////////////////
  // Ball detection Ground
//   if (ball_centre_coord3D[0] ==0)
//   {
//     ball_centre_coord3D[0]=0; ball_centre_coord3D[1]=0; ball_centre_coord3D[2]=0;
//     ball_centre_pixel2D[0]=0; ball_centre_pixel2D[1]=0;
//       
//     ////////
//     //// Careful
//     //// Need to check because compute_ball_centre is called several times for ground ball. 
//     //// Should be checked and optimized
//     ////
//     ////////
//     // Check if previous_ball support trajectory, if so use it
//     my_grid->detect_ball(min_points,mask_ground,MASKSIZE,
// 		      ball_centre_coord3D,ball_centre_pixel2D,
// 		      ball_grid_index,
// 		      0, my_grid->xdim-MASKSIZE,
// 		      0, my_grid->ydim-MASKSIZE,
// 		      0, 1,
// 		      previous_ball);
//       
//     if (ball_centre_coord3D[0] !=0)
//     {
//       ground_detect++;
//       if (debug)
// 	printf("\n%3d - ground ball - %6f %6f %6f",im_num,ball_centre_coord3D[0],ball_centre_coord3D[1],ball_centre_coord3D[2]);
//     }      
//   }
  // Ball detection Ground
  //////////////////////////////////////////////////////

  ///////////////////
  // Trajectory stuff
  // trajecto_pose -1 - no ball
  //		      0 - ball no supporting actual trajectory
  // 		      1 - ball supporting actual trajectory
    
  // update previous list
  // if ball detected check if it is supporting previous trajectory
  my_trajectory->update_list(ball_centre_coord3D);
  
  // if last two position do not support trajectory
  if (my_trajectory->trajecto_pose[HISTORY-1]!=1 && my_trajectory->trajecto_pose[HISTORY-2]!=1)
  //if (my_trajectory->trajecto_pose[HISTORY-1]!=1 )
  {
    // reset trajectory
    for (int i=0;i<HISTORY;i++)
      if (my_trajectory->trajecto_pose[i]==1)
	my_trajectory->trajecto_pose[i]=0;
    my_trajectory->has_trajectory = 0;
  
	
    // compute trajectory from last three position if existing
    my_trajectory->compute_trajectory_from_last_four();
    //my_trajectory->compute_trajectory_from_last_three();  
  }
  // update actual trajectory
  else if (my_trajectory->has_trajectory)
    my_trajectory->refine_trajectory();
  
  // Compute roots
  //if (my_trajectory->trajecto_pose[HISTORY -1] == 1 || my_trajectory->trajecto_pose[HISTORY -2] == 1)
  if (my_trajectory->has_trajectory)
  {
    my_trajectory->get_roots(roots,roots_height);
    if (roots[0] != 0)
    {
      // Check which root has the same direction and put this one in roots1
      if (roots[1] > 0)
      {
	my_trajectory->compute_3D_from_1D(roots[1],root_1_coords);
	my_trajectory->compute_3D_from_1D(roots[2],root_2_coords);
      }
      else
      {
	my_trajectory->compute_3D_from_1D(roots[2],root_1_coords);
	my_trajectory->compute_3D_from_1D(roots[1],root_2_coords);
	double temp = roots[1];
	roots[1] = roots[2];
	roots[2]=temp;
      }
    }
    else
    {
      // reset trajectory
      for (int i=0;i<HISTORY;i++)
	if (my_trajectory->trajecto_pose[i]==1)
	  my_trajectory->trajecto_pose[i]=0;
      my_trajectory->has_trajectory = 0;
    }
  }
      
  ///////////////////////////////////////////////////////
  // Debug 
  if (debug & DEBUG)
  {
     if (ball_centre_coord3D[0]==0)
	printf("\n%3d - no ball -                                 ",im_num);
     if (!my_trajectory->has_trajectory)
       printf(" No trajectory find");
     else
       printf(" roots(%.2f %.2f %.2f [%.2f]) (%.2f %.2f %.2f [.%2f])",root_1_coords[0],root_1_coords[1],root_1_coords[2],roots[1],
								      root_2_coords[0],root_2_coords[1],root_2_coords[2],roots[2]);
  }
  
  if (debug & DEBUG_2D)
    my_view_2D->display_ball(ball_centre_pixel2D);
      
  if (debug & DEBUG_3D)
  {
    // show last balls
    my_view_3D->show_last_balls(my_trajectory->previous_pose, my_trajectory->trajecto_pose);
     
    // show 3D Grid Template
    if (debug & DEBUG_MASK) 
      my_view_3D->show_mask(ball_grid_index,MASKSIZE,gridsize,min_pt);  
    
    // Show trajectory
    if (debug & DEBUG_TRAJECTORY)
    {
      if  (my_trajectory->has_trajectory)
      {
	for (int i=0;i<TRAJECTORY_SAMPLE;i++)
	{
	  double coord[3];
	  //my_trajectory->compute_3D_from_1D( ((roots[1] - roots[2])*1.0*i/TRAJECTORY_SAMPLE) ,coord);
	  my_trajectory->compute_3D_from_1D( roots[2] + ((roots[1] - roots[2])*1.0*i/TRAJECTORY_SAMPLE) ,coord);
	  //my_trajectory->compute_3D_from_1D(roots[1]*1.0*i/TRAJECTORY_SAMPLE,coord);
	  my_view_3D->trajectoryActor[i]->SetPosition(coord[0],coord[1],coord[2]);
	}
	my_view_3D->expected_point_1->SetPosition(root_1_coords[0],root_1_coords[1],root_1_coords[2]);
	my_view_3D->expected_point_2->SetPosition(root_2_coords[0],root_2_coords[1],root_2_coords[2]);
      }
      else
      {
	for (int i=0;i<TRAJECTORY_SAMPLE;i++)
	{
	  my_view_3D->trajectoryActor[i]->SetPosition(0.0,0.0,0.0);
	}
	my_view_3D->expected_point_1->SetPosition(0,0,0);
	my_view_3D->expected_point_2->SetPosition(0,0,0);
      }
    }
    my_view_3D->render();
  }
  // Debug 
  ///////////////////////////////////////////////////////
  
   
  //////////////////////
  // Publish Info in ROS
  if (ros_com && my_trajectory->has_trajectory)
  {
    geometry_msgs::PointStamped p;
    p.header.stamp = cloud_in->header.stamp;
    
    p.point.x = root_1_coords[0];
    p.point.y = root_1_coords[1];
    p.point.z = root_1_coords[2];

    //Publish array
    pub.publish(p);
    //ROS_INFO("published");
  }
  // Publish Info in ROS
  //////////////////////
  
  delete (my_grid);
  
  if (debug & DEBUG)
    printf("\t - Time %.2f", 1000*(double)(clock()- clock_start)/CLOCKS_PER_SEC);
}   

/***********
 * MAIN
 **********/
int main(int argc, char **argv)
{
  /////////////////////////////////////////////////////////
  // mask definition for analysis of neighborhood of voxels UFO 
  mask = create_mask(MASKSIZE);
  mask_ground = create_ground_mask_Z(MASKSIZE);
  
  /////////////////////////////////////////////////////////
  // 3D Viewing
  if (debug & DEBUG_3D)
  {
    my_view_3D = new CView_3D(); 
    my_view_3D->show_axis();
    my_view_3D->initialize_ball_actors();
    if (debug & DEBUG_MASK)
      my_view_3D->initialize_mask_actors(mask,MASKSIZE,gridsize);   
    if (debug & DEBUG_TRAJECTORY)
      my_view_3D->initialize_traj_actors(TRAJECTORY_SAMPLE);   
  }
  
  my_trajectory = new CTrajectory();
  
  ///////////////////////////////////
  // for pointcloud reading from disk
  int image_number = 0;
  //int first_image = 0;
  int first_image = 0;
  //int last_image = 126;
  int last_image = 17;
  char filename[200];
  
  // debug
  if (argc == 2)
  {
    debug = atoi(argv[1]);
  }
  
  if (ros_com==0)
  {
    // update fisrt and last image from args
    if (argc == 3)
    {
      first_image = atoi(argv[1]);
      last_image = atoi(argv[2]);
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
      
    for (image_number = first_image; image_number < last_image; image_number++)
    {
      //////////////////////
      // Read point cloud
	
      //sprintf(filename,"/home/paulo/rosbag/at_home_15032013/cloud/kinect_cloud_%d.pcd",image_number);
      sprintf(filename,"/home/paulo/rosbag/ball_arm/cloud/kinect_cloud_%d.pcd",image_number);
      //printf("\n reading %s",filename);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_in) == -1) //* load the file
      {
	printf ("Couldn't read file %s\n",filename);
	getchar();
	return (-1);
      }
      
      Callback(cloud_in);
    }
  }
  // for pointcloud reading from disk
  ///////////////////////////////////
    
   
   //////////////////////////////////
  // for ROS Operation
  else
  {
    ros::init(argc, argv, "camera_depth_optical_frame");
    listener = new (tf::TransformListener);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/depth/points", 1, Callback);
    
    pub = n.advertise<geometry_msgs::PointStamped>("/ball3D", 10);
    
    ros::spin();
  }
  //for ROS Operation
  //////////////////////////////////
  
  
  printf("\n---------------------------------");
  printf("\n-		General Report");
  printf("\n Air ball detected:    %d",air_detect);
  printf("\n Ground ball detected: %d",ground_detect);
  printf("\n Processed images:     %d",im_num);
  printf("\n---------------------------------\n");
  
  if (debug & DEBUG_3D)
    my_view_3D->start_interaction();
      
  // HouseKeeping
  delete(mask);
  delete(mask_ground);
    
  return 0;
}
