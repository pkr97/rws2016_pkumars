/***********************************************************************************
Name:           grid_3D.cpp
Revision:
Date:           19-02-2013
Author:         Paulo Dias
Comments:       Class with all info needed for 3D grid generation
images
Revision:
Libraries:
Notes:          Code generated under Ubuntu using openCV, VTK and PCL (for reading clouds)
		Compiled using CMake
***********************************************************************************/
#include "CGrid_3D.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/////////////
//constructor
CGrid_3D::CGrid_3D(double grid,Eigen::Vector4f min,Eigen::Vector4f max)
{
	long int x,y,z;

	gridsize = grid;
	min_pt = min;
	max_pt = max;
  
	xdim = (int) ((max_pt[0]-min_pt[0]) / gridsize)+1;
	ydim = (int) ((max_pt[1]-min_pt[1]) / gridsize)+1;
	zdim = (int) ((max_pt[2]-min_pt[2]) / gridsize)+1;

	if (xdim < 0 || ydim < 0 || zdim < 0)
	{
		xdim = 0;
		ydim = 0;
		zdim = 0;
		std::cout << "problems" << std::endl;
		return;
	}
 
  // Memory allocation
	ptr = new int**[xdim];
	for(x=0; x<xdim; x++)      
	{
		ptr[x]=new int*[ydim];
		for(y=0; y<ydim; y++)
		{
			ptr[x][y]=new int[zdim];
			for(z=0; z<zdim; z++)
			{
				ptr[x][y][z]=0;
			}
		}
	}
  
  
	gridnodeXYZ = new double*[xdim*ydim*zdim];
	for(x=0; x<xdim*ydim*zdim; x++)      
	{
		gridnodeXYZ[x]=new double[3];
		for(y=0; y<3; y++)
		{
			gridnodeXYZ[x][y]=0;
		}
	}

}

/////////////
// Destructor
CGrid_3D::~CGrid_3D () 
{
	long int x; 
	
	for(x=0; x<xdim; x++)
	{      
		for(int y=0; y<ydim; y++)
		{
			delete (ptr[x][y]);
		}
	}
  
	for(x=0; x<xdim; x++) 
	{     
		delete (ptr[x]);
	}
	delete(ptr);
  
	for(x=0; x<xdim*ydim*zdim; x++)      
	{
		delete(gridnodeXYZ[x]);
	}
	delete (gridnodeXYZ);
}

/*************************************************************************************************
* Compute Grid - counting number of points in each voxel of the gridnode
* gridsize: xdim * ydim * zdim
* min_pt and max_pt must correspond to the pointCloud
*************************************************************************************************/   
void CGrid_3D::compute_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	long int i,j,x,y,z;
	long int index_grid,index_image;
  
	for (i=0;i<(int) cloud->height;i=i+3)
	{
		for (j=0;j<(int) cloud->width;j=j+3)
		{
			index_image = i * cloud->width + j;
      
			if (!isnan(cloud->points[index_image].x))
			{
				x = (int) ((cloud->points[index_image].x-min_pt[0])/gridsize);
				y = (int) ((cloud->points[index_image].y-min_pt[1])/gridsize);
				z = (int) ((cloud->points[index_image].z-min_pt[2])/gridsize);
			
				if ( x < xdim && y< ydim && z < zdim && x>0 && y>0 && z >0 )
				{
  
					ptr[x][y][z] = ptr[x][y][z]+1;
		
					index_grid = x * ydim * zdim + y * zdim + z;
		
					gridnodeXYZ[index_grid][0] += cloud->points[index_image].x;
					gridnodeXYZ[index_grid][1] += cloud->points[index_image].y;
					gridnodeXYZ[index_grid][2] += cloud->points[index_image].z;
				}
			}
		}
	}
}

/*************************************************************************************************
* Compute Grid - Resetting grid before new analysis
*************************************************************************************************/   
void CGrid_3D::reset()
{
	long int x,y,z;  

	 // Memory allocation
	for(x=0; x<xdim; x++)      
		for(y=0; y<ydim; y++)
			for(z=0; z<zdim; z++)
				ptr[x][y][z]=0;
			
	for(x=0; x<xdim*ydim*zdim; x++)      
	{
		gridnodeXYZ[x][0]=0;
		gridnodeXYZ[x][1]=0;
		gridnodeXYZ[x][2]=0;
	}
}

/*************************************************************************************************
* Compute ball coordinates using UFO algorithm
* Return number of points in mask
* compute 3D coordinate of ball
*************************************************************************************************/
void CGrid_3D::detect_ball(int min_points,				// Minimum points inside mask to consider valid
		int ***mask,int mask_size,    	// Mask info
		double *ball_centre_coord3D,int *ball_grid_index, //Coordinate of ball 3D and 2D and grid index
		int x_init, int x_fin,
		int y_init, int y_fin,
		int z_init, int z_fin,
		double *previous_ball) 
{
	int temp, candidate;
	int i,j,k,ii,jj,kk;
	int count = 0;
	double distance = 10000;
    
	double temp_ball_centre_coord3D[3];
	int temp_count;
	double temp_distance;

	for (i=x_init;i<x_fin;i++)
	for (j=y_init;j<y_fin;j++)  // start above ground
	for (k=z_init;k<z_fin;k++)
	{
		// Count points inside template
		temp = 0;
		for (ii=1;ii<mask_size-1;ii++)
		for (jj=1;jj<mask_size-1;jj++)
		for (kk=1;kk<mask_size-1;kk++)
		temp += ptr[i+ii][j+jj][k+kk];
	
		if (temp > min_points)
		{
			candidate = 1;
			for (ii=0;ii<mask_size;ii++)
			for (jj=0;jj<mask_size;jj++)
			for (kk=0;kk<mask_size;kk++)
	         
			//Test if template border does not contain points
			if (ptr[i+ii][j+jj][k+kk] * mask[ii][jj][kk] > 0)
			{
				candidate = 0;
			}
		 
			if (candidate)
			{
				//std::cout << "temp ="  << temp << std::endl;
				temp_count = compute_ball_centre(i+1,j+1,k+1,mask_size-2,temp_ball_centre_coord3D);
				//std::cout << "temp ="  << temp_count << std::endl;
													
				if (previous_ball[0]!=0) //previous ball available (otherwise 0)
				{
					temp_distance =  sqrt ( (temp_ball_centre_coord3D[0] - previous_ball[0]) * (temp_ball_centre_coord3D[0] - previous_ball[0])
											+(temp_ball_centre_coord3D[1] - previous_ball[1]) * (temp_ball_centre_coord3D[1] - previous_ball[1])
													+(temp_ball_centre_coord3D[2] - previous_ball[2]) * (temp_ball_centre_coord3D[2] - previous_ball[2]));
					// new point closer to previous ball
					if (temp_distance < distance)
					{
						distance = temp_distance;
						ball_centre_coord3D[0] = temp_ball_centre_coord3D[0];
						ball_centre_coord3D[1] = temp_ball_centre_coord3D[1];
						ball_centre_coord3D[2] = temp_ball_centre_coord3D[2];
						
						ball_grid_index[0] = i;
						ball_grid_index[1] = j;
						ball_grid_index[2] = k;
					}
				}
				 // else ball with more points
				{
					if (temp_count > count)
					{
						count = temp_count;
						ball_centre_coord3D[0] = temp_ball_centre_coord3D[0];
						ball_centre_coord3D[1] = temp_ball_centre_coord3D[1];
						ball_centre_coord3D[2] = temp_ball_centre_coord3D[2];
						
						ball_grid_index[0] = i; 
						ball_grid_index[1] = j; 
						ball_grid_index[2] = k;
					}
				}
			}
		}
	}
}


/*************************************************************************************************
* Compute 3D coordinate of ball in given grid, and area of points of interest in grid
* Return number of points in mask
* compute_ball_centre
*************************************************************************************************/
int CGrid_3D::compute_ball_centre(int x,int y,int z,		// upper left coordinate
									int mask_size,						// size of mask to apply
										double *ball_centre_coord3D)
{
  
	int ii,jj,kk, index;
  
	ball_centre_coord3D[0]=0; ball_centre_coord3D[1]=0; ball_centre_coord3D[2]=0;
  
	int count = 0;
  
	for (ii=x;ii<x+mask_size;ii++)
		for (jj=y;jj<y+mask_size;jj++)
		for (kk=z;kk<z+mask_size;kk++)
		{
			index = ii * ydim * zdim + jj * zdim + kk;
	
			ball_centre_coord3D[0] += gridnodeXYZ[index][0];
			ball_centre_coord3D[1] += gridnodeXYZ[index][1];
			ball_centre_coord3D[2] += gridnodeXYZ[index][2];

			count += ptr[ii][jj][kk];
		}
	
	if (count>0)
	{
		ball_centre_coord3D[0] = ball_centre_coord3D[0]/count;
		ball_centre_coord3D[1] = ball_centre_coord3D[1]/count;
		ball_centre_coord3D[2] = ball_centre_coord3D[2]/count;

	}
	return count;
}
