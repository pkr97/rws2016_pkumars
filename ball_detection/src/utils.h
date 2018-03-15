#ifndef _UTILS
#define _UTILS

// General includes
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define  HISTORY 10
#define  TRAJECTORY_SAMPLE 100
#define  MASKSIZE 5
//#define  BALLRADIUS 0.115 // football;
#define  BALLRADIUS 0.05   // arm trial;
#define  DEPTH_LIMIT 5.5


// Function to create masks
int ***create_mask (int mask_size);

// Delete Mask
void delete_mask(int ***mask);

#endif //_UTILS
