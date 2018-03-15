/***********************************************************************************
Name:           utils.cpp
Revision:
Date:           19-02-2013
Author:         Paulo Dias
Comments:       several general functions
images
Revision:
Libraries:
Notes:          Code generated under Ubuntu using openCV, VTK and PCL (for reading clouds)
		Compiled using CMake
***********************************************************************************/

#include "utils.h"
// openCV Includes
#include <cv.h>
#include <highgui.h>

/*************************************************************************************************
* Function to create masks
*************************************************************************************************/
int ***create_mask (int mask_size)
{
  int ii,jj,kk;
  int ***mask = new int**[mask_size];
  for(ii=0; ii<mask_size; ii++)      
  {
    mask[ii]=new int*[mask_size];
    for(jj=0; jj<mask_size; jj++)
    {
      mask[ii][jj]=new int[mask_size];
      for(kk=0; kk<mask_size; kk++)
	mask[ii][jj][kk]=1;
    }
  }
  
  for (ii=1;ii<mask_size-1;ii++)
    for (jj=1;jj<mask_size-1;jj++)	
      for (kk=1;kk<mask_size-1;kk++)
      {
	mask[ii][jj][kk]=0;	
      }
  return mask;
}

