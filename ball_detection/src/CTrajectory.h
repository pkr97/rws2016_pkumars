
#ifndef _CTRAJECTORY
#define _CTRAJECTORY

#include "utils.h"
#include "vtkMath.h"

#define MIN_DISTANCE_TRAJECTORY 0.01

class CTrajectory {
  
  Eigen::Vector3f c;			// Solves a quadratic equation c1*t^2 + c2*t + c3 = 0 
  Eigen::Vector3f origin;		// Origin (3D) of actual trajectory
    
  public:
    Eigen::Vector3f previous_pose[HISTORY];
    int  trajecto_pose[HISTORY];		// - -1 no 3D point in previous_pose
										// -  0 3D point not supporting current trajectory 
										// -  1 3D point supporting current trajectory
    int  has_trajectory;				// indicate trajectory active with 1 
    
  
  public:
    // Constructor
    CTrajectory();
    
    // Destructor
    ~CTrajectory(); 
    
    // if ball detected check if it is supporting previous trajectory and update trajecto_pose accordingly
    void update_list(double *ball_centre_coord3D);
  
    // Compute trajectory from last three positions if valid ball pos, otherwise c(0)=0 (no trajectory)
    void compute_trajectory_from_last_three();
    void compute_trajectory_from_last_four();
    
    // Compute trajectory from all the valid positions
    void refine_trajectory();
    
    // compute the roots of the current quadratic using vtkMath
    void get_roots(double *roots, double height);
    
    // compute the 3D coordinate of a point, from the x coordinate in the quadratic
    void compute_3D_from_1D(double x,double *coords);

    // get_last_supporting_ball_index
    int get_last_supporting_ball_index();
};
#endif //_CTRAJECTORY
