//=============================================================================
//CBBoxInt32
// 3D bounding box with int32 coordinates
//
//=============================================================================
// (C) 2005 ATI Research, Inc., All rights reserved.
//=============================================================================
#ifndef CBBOXINT32_H
#define CBBOXINT32_H

#include <math.h>
#include <stdio.h>

//#include "Types.h"
#include "VectorMacros.h"


//bounding box class with coords specified as int32
class CBBoxInt32
{
public:
  int m_minCoord[3];  //upper left back corner
  int m_maxCoord[3];  //lower right front corner

  CBBoxInt32();
  bool Empty(void);
  void  Clear(void);
  void  Augment(int a_X, int a_Y, int a_Z);
  void  AugmentX(int a_X);
  void  AugmentY(int a_Y);
  void  AugmentZ(int a_Z);
  void  ClampMin(int a_X, int a_Y, int a_Z);
  void  ClampMax(int a_X, int a_Y, int a_Z);
};

#endif //CBBOXINT32_H
