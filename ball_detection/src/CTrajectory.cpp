/***********************************************************************************
Name:           CTrajectory.cpp
Revision:
Date:           25-02-2013
Author:         Paulo Dias
Comments:       Class with all info needed for ballistic trajectory estimation
		(using simple quadratic: parabolic movement)
images
Revision:
Libraries:
Notes:          Code generated under Ubuntu using openCV, VTK and PCL (for reading clouds)
		Compiled using CMake
***********************************************************************************/
#include "CTrajectory.h"

/*************************************************************************************************
* Constructor
*************************************************************************************************/ 
CTrajectory::CTrajectory()
{
  // Initialization
  for (int i=0;i<HISTORY;i++)
  {
    previous_pose[i][0] = 0;
    previous_pose[i][1] = 0;
    previous_pose[i][2] = 0;
    trajecto_pose[i] = -1;
  }
  
  c(0)=c(1)=c(2)=0;
  origin[0]=origin[1]=origin[2]=0;
  
  has_trajectory = 0;
}

/*************************************************************************************************
* Destructor
*************************************************************************************************/ 
CTrajectory::~CTrajectory() 
{
   
}

/*************************************************************************************************
* update_list
* if ball detected check if it is supporting previous trajectory and update trajecto_pose accordingly
*************************************************************************************************/ 
void CTrajectory::update_list(double *ball_centre_coord3D)
{
  Eigen::Vector3f temp;
  Eigen::Vector3f temporigin;
  
  double coords[3];
  
  for (int i=1;i<HISTORY;i++)
    {
    previous_pose[i-1] = previous_pose[i];
    trajecto_pose[i-1] = trajecto_pose[i];
    }
    
  if (ball_centre_coord3D[0] != 0)
  {
    previous_pose[HISTORY-1][0] = ball_centre_coord3D[0];
    previous_pose[HISTORY-1][1] = ball_centre_coord3D[1];
    previous_pose[HISTORY-1][2] = ball_centre_coord3D[2];
      
    // check if new position support actual trajectory
    if (has_trajectory)
    {
      temp = previous_pose[HISTORY-1];
      temp[2]=0;
      temporigin = origin;
      temporigin[2]=0;
      double d = (temp-temporigin).norm();
      
      compute_3D_from_1D(d,coords);
      
      if ( sqrt ( (coords[0]-previous_pose[HISTORY-1][0]) * (coords[0]-previous_pose[HISTORY-1][0])
		 +(coords[1]-previous_pose[HISTORY-1][1]) * (coords[1]-previous_pose[HISTORY-1][1])
		 +(coords[2]-previous_pose[HISTORY-1][2]) * (coords[2]-previous_pose[HISTORY-1][2]) ) < MIN_DISTANCE_TRAJECTORY )
      //if (fabs(ball_centre_coord3D[2] - (c(0)*d*d +c(1)*d +c(2) ) ) < MIN_DISTANCE_TRAJECTORY)
	trajecto_pose[HISTORY-1] = 1;  
      else
	trajecto_pose[HISTORY-1] = 0;
    }
    else
      trajecto_pose[HISTORY-1] = 0;
  }
  else
  {
    previous_pose[HISTORY-1][0] = 0;
    previous_pose[HISTORY-1][1] = 0;
    previous_pose[HISTORY-1][2] = 0;
    trajecto_pose[HISTORY-1] = -1;  
  }
}

/*************************************************************************************************
* compute_trajectory_from_last_three
* Compute trajectory from last three positions if valid ball pos, otherwise c(0)=0 (no trajectory)
*************************************************************************************************/ 
void CTrajectory::compute_trajectory_from_last_four()
{
  Eigen::Vector3f temp;
  Eigen::Vector3f temporigin;
  
  double coords1[3],coords2[3],coords3[3],coords4[3];
  
  has_trajectory = 0;
  c(0)=0;
  origin(0) = origin(1)=origin(2)=0; 
  
  if (trajecto_pose[HISTORY-4]>=0 && trajecto_pose[HISTORY-3]>=0 && trajecto_pose[HISTORY-2]>=0 && trajecto_pose[HISTORY-1]>=0)
  {
    double d1,d2,d3,d4;
    origin = previous_pose[HISTORY-4];
    temporigin = origin;
    
    // set orgin in ground plane
    temporigin[2]=0;
    d4 = 0;
    
    temp = previous_pose[HISTORY-3];
    temp[2]=0;
    d3 = (temp-temporigin).norm();
    
    temp = previous_pose[HISTORY-2];
    temp[2]=0;
    d2 = (temp-temporigin).norm();
    
    temp = previous_pose[HISTORY-1];
    temp[2]=0;
    d1 = (temp-temporigin).norm();
	
    //Eigen::Matrix3f A;
    Eigen::MatrixXf A(4,3);
    //A <<  d1*d1,d1,1,d2*d2,d2,1,d3*d3,d3,1;
    A <<  d4*d4,d4,1,d3*d3,d3,1,d2*d2,d2,1,d1*d1,d1,1;
    Eigen::VectorXf b(4,1);
    //Eigen::Vector3f b(previous_pose[HISTORY-3][2],previous_pose[HISTORY-2][2],previous_pose[HISTORY-1][2]);
    b << previous_pose[HISTORY-4][2],previous_pose[HISTORY-3][2],previous_pose[HISTORY-2][2],previous_pose[HISTORY-1][2];
   
    c = A.colPivHouseholderQr().solve(b);
	
    // check if all three points fits well to curve
    compute_3D_from_1D(d4,coords4);
    compute_3D_from_1D(d3,coords3);
    compute_3D_from_1D(d2,coords2);
    compute_3D_from_1D(d1,coords1);
    if ( sqrt ( (coords4[0]-previous_pose[HISTORY-4][0]) * (coords4[0]-previous_pose[HISTORY-4][0])
	       +(coords4[1]-previous_pose[HISTORY-4][1]) * (coords4[1]-previous_pose[HISTORY-4][1])
	       +(coords4[2]-previous_pose[HISTORY-4][2]) * (coords4[2]-previous_pose[HISTORY-4][2]) ) > MIN_DISTANCE_TRAJECTORY
      || sqrt ( (coords3[0]-previous_pose[HISTORY-3][0]) * (coords3[0]-previous_pose[HISTORY-3][0])
	       +(coords3[1]-previous_pose[HISTORY-3][1]) * (coords3[1]-previous_pose[HISTORY-3][1])
	       +(coords3[2]-previous_pose[HISTORY-3][2]) * (coords3[2]-previous_pose[HISTORY-3][2]) ) > MIN_DISTANCE_TRAJECTORY
      || sqrt ( (coords2[0]-previous_pose[HISTORY-2][0]) * (coords2[0]-previous_pose[HISTORY-2][0])
	       +(coords2[1]-previous_pose[HISTORY-2][1]) * (coords2[1]-previous_pose[HISTORY-2][1])
	       +(coords2[2]-previous_pose[HISTORY-2][2]) * (coords2[2]-previous_pose[HISTORY-2][2]) ) > MIN_DISTANCE_TRAJECTORY
      || sqrt ( (coords1[0]-previous_pose[HISTORY-1][0]) * (coords1[0]-previous_pose[HISTORY-1][0])
	       +(coords1[1]-previous_pose[HISTORY-1][1]) * (coords1[1]-previous_pose[HISTORY-1][1])
	       +(coords1[2]-previous_pose[HISTORY-1][2]) * (coords1[2]-previous_pose[HISTORY-1][2]) ) > MIN_DISTANCE_TRAJECTORY)
/*    if ( fabs(previous_pose[HISTORY-4][2] - (c(0)*d4*d4 +c(1)*d4 +c(2) ) ) > MIN_DISTANCE_TRAJECTORY
      || fabs(previous_pose[HISTORY-3][2] - (c(0)*d3*d3 +c(1)*d3 +c(2) ) ) > MIN_DISTANCE_TRAJECTORY
      || fabs(previous_pose[HISTORY-2][2] - (c(0)*d2*d2 +c(1)*d2 +c(2) ) ) > MIN_DISTANCE_TRAJECTORY
      || fabs(previous_pose[HISTORY-1][2] - (c(0)*d1*d1 +c(1)*d1 +c(2) ) ) > MIN_DISTANCE_TRAJECTORY )*/
    {
      has_trajectory = 0;
      c(0)=0;
      origin(0) = origin(1)=origin(2)=0; 
    }
    else
    {
      has_trajectory = 1;
      trajecto_pose[HISTORY-4]=1;
      trajecto_pose[HISTORY-3]=1;
      trajecto_pose[HISTORY-2]=1;
      trajecto_pose[HISTORY-1]=1;
      origin = previous_pose[HISTORY-4];
    }
  }
}


/*************************************************************************************************
* compute_trajectory_from_last_three
* Compute trajectory from last three positions if valid ball pos, otherwise c(0)=0 (no trajectory)
*************************************************************************************************/ 
void CTrajectory::compute_trajectory_from_last_three()
{
  Eigen::Vector3f temp;
  Eigen::Vector3f temporigin;
  
  double coords1[3],coords2[3],coords3[3];
  
  has_trajectory = 0;
  c(0)=0;
  origin(0) = origin(1)=origin(2)=0; 
  
  if (trajecto_pose[HISTORY-3]>=0 && trajecto_pose[HISTORY-2]>=0 && trajecto_pose[HISTORY-1]>=0)
  {
    double d1,d2,d3;
    origin = previous_pose[HISTORY-3];
    temporigin = origin;
    
    // set orgin in ground plane
    temporigin[2]=0;
    d3 = 0;
    
    temp = previous_pose[HISTORY-2];
    temp[2]=0;
    d2 = (temp-temporigin).norm();
    
    temp = previous_pose[HISTORY-1];
    temp[2]=0;
    d1 = (temp-temporigin).norm();
	
    //Eigen::Matrix3f A;
    Eigen::MatrixXf A(3,3);
    //A <<  d1*d1,d1,1,d2*d2,d2,1,d3*d3,d3,1;
    A <<  d3*d3,d3,1,d2*d2,d2,1,d1*d1,d1,1;
    Eigen::VectorXf b(3,1);
    //Eigen::Vector3f b(previous_pose[HISTORY-3][2],previous_pose[HISTORY-2][2],previous_pose[HISTORY-1][2]);
    b << previous_pose[HISTORY-3][2],previous_pose[HISTORY-2][2],previous_pose[HISTORY-1][2];
   
    c = A.colPivHouseholderQr().solve(b);
	
    // check if all three points fits well to curve
    compute_3D_from_1D(d3,coords3);
    compute_3D_from_1D(d2,coords2);
    compute_3D_from_1D(d1,coords1);
    if ( sqrt ( (coords3[0]-previous_pose[HISTORY-3][0]) * (coords3[0]-previous_pose[HISTORY-3][0])
	       +(coords3[1]-previous_pose[HISTORY-3][1]) * (coords3[1]-previous_pose[HISTORY-3][1])
	       +(coords3[2]-previous_pose[HISTORY-3][2]) * (coords3[2]-previous_pose[HISTORY-3][2]) ) > MIN_DISTANCE_TRAJECTORY
      || sqrt ( (coords2[0]-previous_pose[HISTORY-2][0]) * (coords2[0]-previous_pose[HISTORY-2][0])
	       +(coords2[1]-previous_pose[HISTORY-2][1]) * (coords2[1]-previous_pose[HISTORY-2][1])
	       +(coords2[2]-previous_pose[HISTORY-2][2]) * (coords2[2]-previous_pose[HISTORY-2][2]) ) > MIN_DISTANCE_TRAJECTORY
      || sqrt ( (coords1[0]-previous_pose[HISTORY-1][0]) * (coords1[0]-previous_pose[HISTORY-1][0])
	       +(coords1[1]-previous_pose[HISTORY-1][1]) * (coords1[1]-previous_pose[HISTORY-1][1])
	       +(coords1[2]-previous_pose[HISTORY-1][2]) * (coords1[2]-previous_pose[HISTORY-1][2]) ) > MIN_DISTANCE_TRAJECTORY)
/*    if ( fabs(previous_pose[HISTORY-4][2] - (c(0)*d4*d4 +c(1)*d4 +c(2) ) ) > MIN_DISTANCE_TRAJECTORY
      || fabs(previous_pose[HISTORY-3][2] - (c(0)*d3*d3 +c(1)*d3 +c(2) ) ) > MIN_DISTANCE_TRAJECTORY
      || fabs(previous_pose[HISTORY-2][2] - (c(0)*d2*d2 +c(1)*d2 +c(2) ) ) > MIN_DISTANCE_TRAJECTORY
      || fabs(previous_pose[HISTORY-1][2] - (c(0)*d1*d1 +c(1)*d1 +c(2) ) ) > MIN_DISTANCE_TRAJECTORY )*/
    {
      has_trajectory = 0;
      c(0)=0;
      origin(0) = origin(1)=origin(2)=0; 
    }
    else
    {
      has_trajectory = 1;
      trajecto_pose[HISTORY-3]=1;
      trajecto_pose[HISTORY-2]=1;
      trajecto_pose[HISTORY-1]=1;
      origin = previous_pose[HISTORY-3];
    }
  }
}
/*************************************************************************************************
* refine_trajectory
* Compute trajectory from all the valid positions
*************************************************************************************************/ 
void CTrajectory::refine_trajectory()
{
  Eigen::Vector3f temp;
  Eigen::Vector3f temporigin;
  
  Eigen::MatrixXf A(HISTORY,3);
  Eigen::VectorXf b(HISTORY,1);
  //origin(0)=0;
  for(int k=0;k<HISTORY;k++)
    if (trajecto_pose[k]==1)
    {
//       if (origin(0)==0)
//       {
// 	origin = previous_pose[k];
// 	temporigin = origin;
// 	temporigin[2]=0;
//       }
      double d;
      temp = previous_pose[k]; 
      temp[2]=0;
      temporigin = origin;
      temporigin[2]=0;
      d = (temp-temporigin).norm();
      A(k,0) = d*d;
      A(k,1) = d;
      A(k,2) = 1;
      b(k,0) = previous_pose[k][2];
    }
    else	
    {
      A(k,0) = 0;
      A(k,1) = 0;
      A(k,2) = 1;
      b(k,0) = origin[2];
    }
    
  c = A.colPivHouseholderQr().solve(b);
}

/*************************************************************************************************
* get_roots
* compute the roots of the current quadratic using vtkMath
* Solves a quadratic equation c1*t^2 + c2*t + c3 = height
* Return array contains number of (real) roots (counting multiple roots as one) followed by roots themselves
* Return null if no trajectory is estimated
*************************************************************************************************/ 
void CTrajectory::get_roots(double *roots, double height)
{
  if (!has_trajectory)
    roots[0]=0;
  
  double *rootst;
  rootst = vtkPolynomialSolversUnivariate::SolveQuadratic(c(0),c(1),c(2)-height);
  
  roots[0] = rootst[0];
  roots[1] = rootst[1];
  roots[2] = rootst[2];
}

/*************************************************************************************************
* compute_3D_from_1D
* compute the 3D coordinate of a point, from the x coordinate in the quadratic
* based on the quadratic equation and the 3D origin used for computing the quadratic
*************************************************************************************************/ 
void CTrajectory::compute_3D_from_1D(double x,double *coords)
{
  double vx,vy,d;
  Eigen::Vector3f temp;
  Eigen::Vector3f temporigin;
  
  int index;
  
  index = get_last_supporting_ball_index();
  
  vx = previous_pose[index][0]-origin[0];
  vy = previous_pose[index][1]-origin[1];
  temp = previous_pose[index];
  temp[2]=0;
  temporigin = origin;
  temporigin[2]=0;
  d = (temp-temporigin).norm();
    
  coords[0] = x*(vx/d) + origin[0];
  coords[1] = x*(vy/d) + origin[1];
  coords[2] = c(0)*x*x + c(1)*x+ c(2);
}

/*************************************************************************************************
* get_last_supporting_ball_index
* return the index of the last ball that supports trajectory 
* if no trajectory, return -1
*************************************************************************************************/ 
int CTrajectory::get_last_supporting_ball_index()
{
  int i;
  
  if (!has_trajectory)
    return 0;
  
  for (i = HISTORY-1; i >= 0; i--)
    if (trajecto_pose[i] == 1)
      return i;
    
  return 0;
}
