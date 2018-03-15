
#ifndef _GRID_3D
#define _GRID_3D

#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class CGrid_3D {
  
  int ***ptr;		       		// pointer to number of points in each grid node
  double gridsize;	       		// size of the node
  double **gridnodeXYZ;	       		// Sum of the 3D coordinate in each node to compute centre of geometry
  double **gridnodeIJ;	       		// Sum of the 2D coordinate in each node to compute position in 2D image
  
  Eigen::Vector4f min_pt, max_pt;	// Coordinate of limits of cloud of points
  
  public:
    int xdim,ydim,zdim; 	// grid dimension
 
  
  public:
    // constructor
    CGrid_3D(double grid,Eigen::Vector4f min_pt,Eigen::Vector4f max_pt); 	
    // destructor
    ~CGrid_3D();								
    //compute grid from pointcloud
    void compute_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);  	
    //reset grid 
    void reset();  		
    // detect ball in grid
    void detect_ball(int min_points,						// Minimum points inside mask to consider valid
		      int ***mask,int mask_size,    				// Mask info
		      double *ball_centre_coord3D,  //Coordinate of ball 2D and 3D
		      int *ball_grid_index,					// index of ball in grid (position 0,0,0)
		      int x_init, int x_fin,
		      int y_init, int y_fin,
		      int z_init, int z_fin,
		      double *previous_ball);					// previous_ball coordinate
    // compute_ball_centre
    int compute_ball_centre(int x,int y,int z,	// upper left coordinate
			    int mask_size,				// size of mask to apply
			    double *ball_centre_coord3D);

};

#endif //_CVIEW_3D
