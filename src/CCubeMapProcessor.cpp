//--------------------------------------------------------------------------------------
//CCubeMapFilter
// Classes for filtering and processing cubemaps
//
//
//--------------------------------------------------------------------------------------
// (C) 2005 ATI Research, Inc., All rights reserved.
//--------------------------------------------------------------------------------------
#include "StdAfx.h"
#include "CCubeMapProcessor.h"
#include <algorithm>

#define CP_PI   3.14159265358979323846

//------------------------------------------------------------------------------
// D3D cube map face specification
//   mapping from 3D x,y,z cube map lookup coordinates 
//   to 2D within face u,v coordinates
//
//   --------------------> U direction 
//   |                   (within-face texture space)
//   |         _____
//   |        |     |
//   |        | +Y  |
//   |   _____|_____|_____ _____
//   |  |     |     |     |     |
//   |  | -X  | +Z  | +X  | -Z  |
//   |  |_____|_____|_____|_____|
//   |        |     |
//   |        | -Y  |
//   |        |_____|
//   |
//   v   V direction
//      (within-face texture space)
//------------------------------------------------------------------------------

//Information about neighbors and how texture coorrdinates change across faces 
//  in ORDER of left, right, top, bottom (e.g. edges corresponding to u=0, 
//  u=1, v=0, v=1 in the 2D coordinate system of the particular face.
//Note this currently assumes the D3D cube face ordering and orientation
CPCubeMapNeighbor sg_CubeNgh[6][4] =
{
  //XPOS face
  {{CP_FACE_Z_POS, CP_EDGE_RIGHT },
  {CP_FACE_Z_NEG, CP_EDGE_LEFT  },
  {CP_FACE_Y_POS, CP_EDGE_RIGHT },
  {CP_FACE_Y_NEG, CP_EDGE_RIGHT }},
  //XNEG face
  {{CP_FACE_Z_NEG, CP_EDGE_RIGHT },
  {CP_FACE_Z_POS, CP_EDGE_LEFT  },
  {CP_FACE_Y_POS, CP_EDGE_LEFT  },
  {CP_FACE_Y_NEG, CP_EDGE_LEFT  }},
  //YPOS face
  {{CP_FACE_X_NEG, CP_EDGE_TOP },
  {CP_FACE_X_POS, CP_EDGE_TOP },
  {CP_FACE_Z_NEG, CP_EDGE_TOP },
  {CP_FACE_Z_POS, CP_EDGE_TOP }},
  //YNEG face
  {{CP_FACE_X_NEG, CP_EDGE_BOTTOM},
  {CP_FACE_X_POS, CP_EDGE_BOTTOM},
  {CP_FACE_Z_POS, CP_EDGE_BOTTOM},
  {CP_FACE_Z_NEG, CP_EDGE_BOTTOM}},
  //ZPOS face
  {{CP_FACE_X_NEG, CP_EDGE_RIGHT  },
  {CP_FACE_X_POS, CP_EDGE_LEFT   },
  {CP_FACE_Y_POS, CP_EDGE_BOTTOM },
  {CP_FACE_Y_NEG, CP_EDGE_TOP    }},
  //ZNEG face
  {{CP_FACE_X_POS, CP_EDGE_RIGHT  },
  {CP_FACE_X_NEG, CP_EDGE_LEFT   },
  {CP_FACE_Y_POS, CP_EDGE_TOP    },
  {CP_FACE_Y_NEG, CP_EDGE_BOTTOM }}
};


//3x2 matrices that map cube map indexing vectors in 3d 
// (after face selection and divide through by the 
//  _ABSOLUTE VALUE_ of the max coord)
// into NVC space
//Note this currently assumes the D3D cube face ordering and orientation
#define CP_UDIR     0
#define CP_VDIR     1
#define CP_FACEAXIS 2

float sgFace2DMapping[6][3][3] = {
  //XPOS face
  {{ 0,  0, -1},   //u towards negative Z
  { 0, -1,  0},   //v towards negative Y
  {1,  0,  0}},  //pos X axis  
  //XNEG face
  {{0,  0,  1},   //u towards positive Z
  {0, -1,  0},   //v towards negative Y
  {-1,  0,  0}},  //neg X axis       
  //YPOS face
  {{1, 0, 0},     //u towards positive X
  {0, 0, 1},     //v towards positive Z
  {0, 1 , 0}},   //pos Y axis  
  //YNEG face
  {{1, 0, 0},     //u towards positive X
  {0, 0 , -1},   //v towards negative Z
  {0, -1 , 0}},  //neg Y axis  
  //ZPOS face
  {{1, 0, 0},     //u towards positive X
  {0, -1, 0},    //v towards negative Y
  {0, 0,  1}},   //pos Z axis  
  //ZNEG face
  {{-1, 0, 0},    //u towards negative X
  {0, -1, 0},    //v towards negative Y
  {0, 0, -1}},   //neg Z axis  
};


//The 12 edges of the cubemap, (entries are used to index into the neighbor table)
// this table is used to average over the edges.
int sg_CubeEdgeList[12][2] = {
  {CP_FACE_X_POS, CP_EDGE_LEFT},
  {CP_FACE_X_POS, CP_EDGE_RIGHT},
  {CP_FACE_X_POS, CP_EDGE_TOP},
  {CP_FACE_X_POS, CP_EDGE_BOTTOM},

  {CP_FACE_X_NEG, CP_EDGE_LEFT},
  {CP_FACE_X_NEG, CP_EDGE_RIGHT},
  {CP_FACE_X_NEG, CP_EDGE_TOP},
  {CP_FACE_X_NEG, CP_EDGE_BOTTOM},

  {CP_FACE_Z_POS, CP_EDGE_TOP},
  {CP_FACE_Z_POS, CP_EDGE_BOTTOM},
  {CP_FACE_Z_NEG, CP_EDGE_TOP},
  {CP_FACE_Z_NEG, CP_EDGE_BOTTOM}
};


//Information about which of the 8 cube corners are correspond to the 
//  the 4 corners in each cube face
//  the order is upper left, upper right, lower left, lower right
int sg_CubeCornerList[6][4] = {
  { CP_CORNER_PPP, CP_CORNER_PPN, CP_CORNER_PNP, CP_CORNER_PNN }, // XPOS face
  { CP_CORNER_NPN, CP_CORNER_NPP, CP_CORNER_NNN, CP_CORNER_NNP }, // XNEG face
  { CP_CORNER_NPN, CP_CORNER_PPN, CP_CORNER_NPP, CP_CORNER_PPP }, // YPOS face
  { CP_CORNER_NNP, CP_CORNER_PNP, CP_CORNER_NNN, CP_CORNER_PNN }, // YNEG face
  { CP_CORNER_NPP, CP_CORNER_PPP, CP_CORNER_NNP, CP_CORNER_PNP }, // ZPOS face
  { CP_CORNER_PPN, CP_CORNER_NPN, CP_CORNER_PNN, CP_CORNER_NNN }  // ZNEG face
};


//--------------------------------------------------------------------------------------
//Error handling for cube map processor
//  Pop up dialog box, and terminate application
//--------------------------------------------------------------------------------------
void CPFatalError(WCHAR *a_Msg)
{
  //MessageBoxW(NULL, a_Msg, L"Error: Application Terminating", MB_OK);
  //exit(EM_FATAL_ERROR);
}

// SL BEGIN
void slerp(float* res, float* a, float* b, float t)
{
  float angle = acosf(VM_DOTPROD3(a, b));

  if (0.0f == angle)
  {
    res[0] = a[0];
    res[1] = a[1];
    res[2] = a[2];
  }
  else if (CP_PI == angle)
  {
    // Can't recovert!
    res[0] = 0;
    res[1] = 0;
    res[2] = 0;
  }
  else
  {
    res[0] = (sinf((1.0-t)*angle)/sinf(angle))*a[0] + (sinf(t*angle)/sinf(angle))*b[0];
    res[1] = (sinf((1.0-t)*angle)/sinf(angle))*a[1] + (sinf(t*angle)/sinf(angle))*b[1];
    res[2] = (sinf((1.0-t)*angle)/sinf(angle))*a[2] + (sinf(t*angle)/sinf(angle))*b[2];
  }
}

#define LERP(A, B, FACTOR) ( (A) + (FACTOR)*((B) - (A)) )	
// SL END

//--------------------------------------------------------------------------------------
// Convert cubemap face texel coordinates and face idx to 3D vector
// note the U and V coords are integer coords and range from 0 to size-1
//  this routine can be used to generate a normalizer cube map
//--------------------------------------------------------------------------------------
// SL BEGIN
void TexelCoordToVect(int a_FaceIdx, float a_U, float a_V, int a_Size, float *a_XYZ, int a_FixupType)
{
  float nvcU, nvcV;
  float tempVec[3];

  if (a_FixupType == CP_FIXUP_STRETCH && a_Size > 1)
  {
    // Code from Nvtt : http://code.google.com/p/nvidia-texture-tools/source/browse/trunk/src/nvtt/CubeSurface.cpp		
    // transform from [0..res - 1] to [-1 .. 1], match up edges exactly.
    nvcU = (2.0f * (float)a_U / ((float)a_Size - 1.0f) ) - 1.0f;
    nvcV = (2.0f * (float)a_V / ((float)a_Size - 1.0f) ) - 1.0f;
  }
  else
  {
    // Change from original AMD code
    // transform from [0..res - 1] to [- (1 - 1 / res) .. (1 - 1 / res)]
    // + 0.5f is for texel center addressing
    nvcU = (2.0f * ((float)a_U + 0.5f) / (float)a_Size ) - 1.0f;
    nvcV = (2.0f * ((float)a_V + 0.5f) / (float)a_Size ) - 1.0f;
  }

  if (a_FixupType == CP_FIXUP_WARP && a_Size > 1)
  {
#if 0
    // Code from Nvtt : http://code.google.com/p/nvidia-texture-tools/source/browse/trunk/src/nvtt/CubeSurface.cpp
    float a = powf(float(a_Size), 2.0f) / powf(float(a_Size - 1), 3.0f);
    nvcU = a * powf(nvcU, 3) + nvcU;
    nvcV = a * powf(nvcV, 3) + nvcV;
#else
    const float aSizeF = static_cast<float>(a_Size);
    const float aSizeF_1 = static_cast<float>(a_Size-1);
    const float a = (aSizeF*aSizeF) / (aSizeF_1*aSizeF_1*aSizeF_1);
    nvcU = a * nvcU*nvcU*nvcU + nvcU;
    nvcV = a * nvcV*nvcV*nvcV + nvcV;
#endif

    // Get current vector
    //generate x,y,z vector (xform 2d NVC coord to 3D vector)
    //U contribution
    VM_SCALE3(a_XYZ, sgFace2DMapping[a_FaceIdx][CP_UDIR], nvcU);    
    //V contribution
    VM_SCALE3(tempVec, sgFace2DMapping[a_FaceIdx][CP_VDIR], nvcV);
    VM_ADD3(a_XYZ, tempVec, a_XYZ);
    //add face axis
    VM_ADD3(a_XYZ, sgFace2DMapping[a_FaceIdx][CP_FACEAXIS], a_XYZ);
    //normalize vector
    VM_NORM3(a_XYZ, a_XYZ);
  }
  else if (a_FixupType == CP_FIXUP_BENT && a_Size > 1)
  {
    // Method following description of Physically based rendering slides from CEDEC2011 of TriAce

    // Get vector at edge
    float EdgeNormalU[3];
    float EdgeNormalV[3];
    float EdgeNormal[3];
    float EdgeNormalMinusOne[3];

    // Recover vector at edge
    //U contribution
    VM_SCALE3(EdgeNormalU, sgFace2DMapping[a_FaceIdx][CP_UDIR], nvcU < 0.0 ? -1.0f : 1.0f);    
    //V contribution
    VM_SCALE3(EdgeNormalV, sgFace2DMapping[a_FaceIdx][CP_VDIR], nvcV < 0.0 ? -1.0f : 1.0f);
    VM_ADD3(EdgeNormal, EdgeNormalV, EdgeNormalU);
    //add face axis
    VM_ADD3(EdgeNormal, sgFace2DMapping[a_FaceIdx][CP_FACEAXIS], EdgeNormal);
    //normalize vector
    VM_NORM3(EdgeNormal, EdgeNormal);

    // Get vector at (edge - 1)
    float nvcUEdgeMinus1 = (2.0f * ((float)(nvcU < 0.0f ? 0 : a_Size-1) + 0.5f) / (float)a_Size ) - 1.0f;
    float nvcVEdgeMinus1 = (2.0f * ((float)(nvcV < 0.0f ? 0 : a_Size-1) + 0.5f) / (float)a_Size ) - 1.0f;

    // Recover vector at (edge - 1)
    //U contribution
    VM_SCALE3(EdgeNormalU, sgFace2DMapping[a_FaceIdx][CP_UDIR], nvcUEdgeMinus1);    
    //V contribution
    VM_SCALE3(EdgeNormalV, sgFace2DMapping[a_FaceIdx][CP_VDIR], nvcVEdgeMinus1);
    VM_ADD3(EdgeNormalMinusOne, EdgeNormalV, EdgeNormalU);
    //add face axis
    VM_ADD3(EdgeNormalMinusOne, sgFace2DMapping[a_FaceIdx][CP_FACEAXIS], EdgeNormalMinusOne);
    //normalize vector
    VM_NORM3(EdgeNormalMinusOne, EdgeNormalMinusOne);

    // Get angle between the two vector (which is 50% of the two vector presented in the TriAce slide)
    float AngleNormalEdge = acosf(VM_DOTPROD3(EdgeNormal, EdgeNormalMinusOne));

    // Here we assume that high resolution required less offset than small resolution (TriAce based this on blur radius and custom value) 
    // Start to increase from 50% to 100% target angle from 128x128x6 to 1x1x6
    float NumLevel = (logf(std::min(a_Size, 128))  / logf(2)) - 1;
    AngleNormalEdge = LERP(0.5 * AngleNormalEdge, AngleNormalEdge, 1.0f - (NumLevel/6) );

    float factorU = abs((2.0f * ((float)a_U) / (float)(a_Size - 1) ) - 1.0f);
    float factorV = abs((2.0f * ((float)a_V) / (float)(a_Size - 1) ) - 1.0f);
    AngleNormalEdge = LERP(0.0f, AngleNormalEdge, std::max(factorU, factorV) );

    // Get current vector
    //generate x,y,z vector (xform 2d NVC coord to 3D vector)
    //U contribution
    VM_SCALE3(a_XYZ, sgFace2DMapping[a_FaceIdx][CP_UDIR], nvcU);    
    //V contribution
    VM_SCALE3(tempVec, sgFace2DMapping[a_FaceIdx][CP_VDIR], nvcV);
    VM_ADD3(a_XYZ, tempVec, a_XYZ);
    //add face axis
    VM_ADD3(a_XYZ, sgFace2DMapping[a_FaceIdx][CP_FACEAXIS], a_XYZ);
    //normalize vector
    VM_NORM3(a_XYZ, a_XYZ);

    float RadiantAngle = AngleNormalEdge;
    // Get angle between face normal and current normal. Used to push the normal away from face normal.
    float AngleFaceVector = acosf(VM_DOTPROD3(sgFace2DMapping[a_FaceIdx][CP_FACEAXIS], a_XYZ));

    // Push the normal away from face normal by an angle of RadiantAngle
    slerp(a_XYZ, sgFace2DMapping[a_FaceIdx][CP_FACEAXIS], a_XYZ, 1.0f + RadiantAngle / AngleFaceVector);
  }
  else
  {
    //generate x,y,z vector (xform 2d NVC coord to 3D vector)
    //U contribution
    VM_SCALE3(a_XYZ, sgFace2DMapping[a_FaceIdx][CP_UDIR], nvcU);    
    //V contribution
    VM_SCALE3(tempVec, sgFace2DMapping[a_FaceIdx][CP_VDIR], nvcV);
    VM_ADD3(a_XYZ, tempVec, a_XYZ);
    //add face axis
    VM_ADD3(a_XYZ, sgFace2DMapping[a_FaceIdx][CP_FACEAXIS], a_XYZ);

    //normalize vector
    VM_NORM3(a_XYZ, a_XYZ);
  }
}
// SL END

//--------------------------------------------------------------------------------------
// Convert 3D vector to cubemap face texel coordinates and face idx 
// note the U and V coords are integer coords and range from 0 to size-1
//  this routine can be used to generate a normalizer cube map
//
// returns face IDX and texel coords
//--------------------------------------------------------------------------------------
// SL BEGIN
/*
Mapping Texture Coordinates to Cube Map Faces
Because there are multiple faces, the mapping of texture coordinates to positions on cube map faces
is more complicated than the other texturing targets.  The EXT_texture_cube_map extension is purposefully
designed to be consistent with DirectX 7's cube map arrangement.  This is also consistent with the cube
map arrangement in Pixar's RenderMan package. 
For cube map texturing, the (s,t,r) texture coordinates are treated as a direction vector (rx,ry,rz)
emanating from the center of a cube.  (The q coordinate can be ignored since it merely scales the vector
without affecting the direction.) At texture application time, the interpolated per-fragment (s,t,r)
selects one of the cube map face's 2D mipmap sets based on the largest magnitude coordinate direction 
the major axis direction). The target column in the table below explains how the major axis direction
maps to the 2D image of a particular cube map target. 

major axis 
direction     target                              sc     tc    ma 
----------    ---------------------------------   ---    ---   --- 
+rx          GL_TEXTURE_CUBE_MAP_POSITIVE_X_EXT   -rz    -ry   rx 
-rx          GL_TEXTURE_CUBE_MAP_NEGATIVE_X_EXT   +rz    -ry   rx 
+ry          GL_TEXTURE_CUBE_MAP_POSITIVE_Y_EXT   +rx    +rz   ry 
-ry          GL_TEXTURE_CUBE_MAP_NEGATIVE_Y_EXT   +rx    -rz   ry 
+rz          GL_TEXTURE_CUBE_MAP_POSITIVE_Z_EXT   +rx    -ry   rz 
-rz          GL_TEXTURE_CUBE_MAP_NEGATIVE_Z_EXT   -rx    -ry   rz

Using the sc, tc, and ma determined by the major axis direction as specified in the table above,
an updated (s,t) is calculated as follows 
s   =   ( sc/|ma| + 1 ) / 2 
t   =   ( tc/|ma| + 1 ) / 2
If |ma| is zero or very nearly zero, the results of the above two equations need not be defined
(though the result may not lead to GL interruption or termination).  Once the cube map face's 2D mipmap
set and (s,t) is determined, texture fetching and filtering proceeds like standard OpenGL 2D texturing. 
*/
// Note this method return U and V in range from 0 to size-1
// SL END
void VectToTexelCoord(float *a_XYZ, int a_Size, int *a_FaceIdx, int *a_U, int *a_V )
{
  float nvcU, nvcV;
  float absXYZ[3];
  float maxCoord;
  float onFaceXYZ[3];
  int   faceIdx;
  int   u, v;

  //absolute value 3
  VM_ABS3(absXYZ, a_XYZ);

  if( (absXYZ[0] >= absXYZ[1]) && (absXYZ[0] >= absXYZ[2]) )
  {
    maxCoord = absXYZ[0];

    if(a_XYZ[0] >= 0) //face = XPOS
    {
      faceIdx = CP_FACE_X_POS;            
    }    
    else
    {
      faceIdx = CP_FACE_X_NEG;                    
    }
  }
  else if ( (absXYZ[1] >= absXYZ[0]) && (absXYZ[1] >= absXYZ[2]) )
  {
    maxCoord = absXYZ[1];

    if(a_XYZ[1] >= 0) //face = XPOS
    {
      faceIdx = CP_FACE_Y_POS;            
    }    
    else
    {
      faceIdx = CP_FACE_Y_NEG;                    
    }    
  }
  else  // if( (absXYZ[2] > absXYZ[0]) && (absXYZ[2] > absXYZ[1]) )
  {
    maxCoord = absXYZ[2];

    if(a_XYZ[2] >= 0) //face = XPOS
    {
      faceIdx = CP_FACE_Z_POS;            
    }    
    else
    {
      faceIdx = CP_FACE_Z_NEG;                    
    }    
  }

  //divide through by max coord so face vector lies on cube face
  VM_SCALE3(onFaceXYZ, a_XYZ, 1.0f/maxCoord);
  nvcU = VM_DOTPROD3(sgFace2DMapping[ faceIdx ][CP_UDIR], onFaceXYZ );
  nvcV = VM_DOTPROD3(sgFace2DMapping[ faceIdx ][CP_VDIR], onFaceXYZ );

  // SL BEGIN
  // Modify original AMD code to return value from 0 to Size - 1
  u = (int)floor( (a_Size - 1) * 0.5f * (nvcU + 1.0f) );
  v = (int)floor( (a_Size - 1) * 0.5f * (nvcV + 1.0f) );
  // SL END

  *a_FaceIdx = faceIdx;
  *a_U = u;
  *a_V = v;
}


//--------------------------------------------------------------------------------------
// gets texel ptr in a cube map given a direction vector, and an array of 
//  CImageSurfaces that represent the cube faces.
//   
//--------------------------------------------------------------------------------------
CP_ITYPE *GetCubeMapTexelPtr(float *a_XYZ, CImageSurface *a_Surface)
{
  int u, v, faceIdx;    

  //get face idx and u, v texel coordinate in face
  VectToTexelCoord(a_XYZ, a_Surface[0].m_Width, &faceIdx, &u, &v );

  return( a_Surface[faceIdx].GetSurfaceTexelPtr(u, v) );
}

inline float fast_atan2(const float& y, const float& x) {
  static const float PI = 3.14159265f;
  static const float PI_2 = PI/2.0f;
  if (x == 0.0f) {
    if (y > 0.0f) {
      return PI_2;
    }
    if (y == 0.0f) {
      return 0.0f;
    }
    return -PI_2;
  }
  float xx = x*x;
  float yy = y*y;
  if (yy < xx) {
    float atan = x*y/(xx + 0.28f*yy);
    if (x < 0.0f) {
      if (y < 0.0f) {
        return atan - PI;
      }
      return atan + PI;
    }
    return atan;
  } else {
    float atan = PI_2 - y*x/(yy + 0.28f*xx);
    if (y < 0.0f) {
      return atan - PI;
    }
    return atan;
  }
}

static float AreaElement( float x, float y )
{
  return fast_atan2(x * y, sqrt(x * x + y * y + 1));
}

float TexelCoordSolidAngle(int a_FaceIdx, float a_U, float a_V, int a_Size)
{
  // transform from [0..res - 1] to [- (1 - 1 / res) .. (1 - 1 / res)]
  // (+ 0.5f is for texel center addressing)
  float U = (2.0f * ((float)a_U + 0.5f) / (float)a_Size ) - 1.0f;
  float V = (2.0f * ((float)a_V + 0.5f) / (float)a_Size ) - 1.0f;

  // Shift from a demi texel, mean 1.0f / a_Size with U and V in [-1..1]
  float InvResolution = 1.0f / a_Size;

  // U and V are the -1..1 texture coordinate on the current face.
  // Get projected area for this texel
  float x0 = U - InvResolution;
  float y0 = V - InvResolution;
  float x1 = U + InvResolution;
  float y1 = V + InvResolution;
  float SolidAngle = AreaElement(x0, y0) - AreaElement(x0, y1) - AreaElement(x1, y0) + AreaElement(x1, y1);

  return SolidAngle;
}

//--------------------------------------------------------------------------------------
//Builds a normalizer cubemap
//
// Takes in a cube face size, and an array of 6 surfaces to write the cube faces into
//
// Note that this normalizer cube map stores the vectors in unbiased -1 to 1 range.
//  if _bx2 style scaled and biased vectors are needed, uncomment the SCALE and BIAS
//  below
//--------------------------------------------------------------------------------------
// SL BEGIN
void CCubeMapProcessor::BuildNormalizerCubemap(int a_Size, CImageSurface *a_Surface, int a_FixupType)
  // SL END
{
  int iCubeFace, u, v;

  //iterate over cube faces
  for(iCubeFace=0; iCubeFace<6; iCubeFace++)
  {
    a_Surface[iCubeFace].Clear();
    a_Surface[iCubeFace].Init(a_Size, a_Size, 3);

    //fast texture walk, build normalizer cube map
    CP_ITYPE *texelPtr = a_Surface[iCubeFace].m_ImgData;

    for(v=0; v < a_Surface[iCubeFace].m_Height; v++)
    {
      for(u=0; u < a_Surface[iCubeFace].m_Width; u++)
      {
        // SL_BEGIN
        TexelCoordToVect(iCubeFace, (float)u, (float)v, a_Size, texelPtr, a_FixupType);
        // SL END

        //VM_SCALE3(texelPtr, texelPtr, 0.5f);
        //VM_BIAS3(texelPtr, texelPtr, 0.5f);

        texelPtr += a_Surface[iCubeFace].m_NumChannels;
      }         
    }
  }
}


//--------------------------------------------------------------------------------------
//Builds a normalizer cubemap, with the texels solid angle stored in the fourth component
//
//Takes in a cube face size, and an array of 6 surfaces to write the cube faces into
//
//Note that this normalizer cube map stores the vectors in unbiased -1 to 1 range.
// if _bx2 style scaled and biased vectors are needed, uncomment the SCALE and BIAS
// below
//--------------------------------------------------------------------------------------
// SL BEGIN
void CCubeMapProcessor::BuildNormalizerSolidAngleCubemap(int a_Size, CImageSurface *a_Surface, int a_FaceIdx, int a_FixupType)
{
  a_Surface[a_FaceIdx].Clear();
  a_Surface[a_FaceIdx].Init(a_Size, a_Size, 4, true);  //First three channels for norm cube, and last channel for solid angle

  CacheMap& cache = m_NormalizerCache[a_FaceIdx];
  CacheMap::iterator it = cache.find(a_Size);
  if (it == cache.end()) {
    // not found, need to compute
    CP_ITYPE* data = new CP_ITYPE[a_Size * a_Size * 4];

    CP_ITYPE* texelPtr = data;

    for (int v=0; v<a_Size; v++)
    {
      for (int u=0; u<a_Size; u++)
      {
        // SL_BEGIN
        TexelCoordToVect(a_FaceIdx, (float)u, (float)v, a_Size, texelPtr, a_FixupType);
        // SL END
        //VM_SCALE3(texelPtr, texelPtr, 0.5f);
        //VM_BIAS3(texelPtr, texelPtr, 0.5f);

        *(texelPtr + 3) = TexelCoordSolidAngle(a_FaceIdx, (float)u, (float)v, a_Size);

        texelPtr += a_Surface[a_FaceIdx].m_NumChannels;
      }         
    }
    cache[a_Size] = data;
  }
  a_Surface[a_FaceIdx].m_ImgData = cache[a_Size];
  a_Surface[a_FaceIdx].m_ImgDataShared = true;
}


//--------------------------------------------------------------------------------------
//Clear filter extents for the 6 cube map faces
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::ClearFilterExtents(CBBoxInt32 *aFilterExtents)
{
  int iCubeFaces;

  for(iCubeFaces=0; iCubeFaces<6; iCubeFaces++)
  {
    aFilterExtents[iCubeFaces].Clear();    
  }
}


//--------------------------------------------------------------------------------------
//Define per-face bounding box filter extents
//
// These define conservative texel regions in each of the faces the filter can possibly 
// process.  When the pixels in the regions are actually processed, the dot product  
// between the tap vector and the center tap vector is used to determine the weight of 
// the tap and whether or not the tap is within the cone.
//
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::DetermineFilterExtents(float *a_CenterTapDir, int a_SrcSize, int a_BBoxSize, 
                                               CBBoxInt32 *a_FilterExtents )
{
  int u, v;
  int faceIdx;
  int minU, minV, maxU, maxV;
  int i;

  //neighboring face and bleed over amount, and width of BBOX for
  // left, right, top, and bottom edges of this face
  int bleedOverAmount[4];
  int bleedOverBBoxMin[4];
  int bleedOverBBoxMax[4];

  int neighborFace;
  int neighborEdge;

  //get face idx, and u, v info from center tap dir
  VectToTexelCoord(a_CenterTapDir, a_SrcSize, &faceIdx, &u, &v );

  //define bbox size within face
  a_FilterExtents[faceIdx].Augment(u - a_BBoxSize, v - a_BBoxSize, 0);
  a_FilterExtents[faceIdx].Augment(u + a_BBoxSize, v + a_BBoxSize, 0);

  a_FilterExtents[faceIdx].ClampMin(0, 0, 0);
  a_FilterExtents[faceIdx].ClampMax(a_SrcSize-1, a_SrcSize-1, 0);

  //u and v extent in face corresponding to center tap
  minU = a_FilterExtents[faceIdx].m_minCoord[0];
  minV = a_FilterExtents[faceIdx].m_minCoord[1];
  maxU = a_FilterExtents[faceIdx].m_maxCoord[0];
  maxV = a_FilterExtents[faceIdx].m_maxCoord[1];

  //bleed over amounts for face across u=0 edge (left)    
  bleedOverAmount[0] = (a_BBoxSize - u);
  bleedOverBBoxMin[0] = minV;
  bleedOverBBoxMax[0] = maxV;

  //bleed over amounts for face across u=1 edge (right)    
  bleedOverAmount[1] = (u + a_BBoxSize) - (a_SrcSize-1);
  bleedOverBBoxMin[1] = minV;
  bleedOverBBoxMax[1] = maxV;

  //bleed over to face across v=0 edge (up)
  bleedOverAmount[2] = (a_BBoxSize - v);
  bleedOverBBoxMin[2] = minU;
  bleedOverBBoxMax[2] = maxU;

  //bleed over to face across v=1 edge (down)
  bleedOverAmount[3] = (v + a_BBoxSize) - (a_SrcSize-1);
  bleedOverBBoxMin[3] = minU;
  bleedOverBBoxMax[3] = maxU;

  //compute bleed over regions in neighboring faces
  for(i=0; i<4; i++)
  {
    if(bleedOverAmount[i] > 0)
    {
      neighborFace = sg_CubeNgh[faceIdx][i].m_Face;
      neighborEdge = sg_CubeNgh[faceIdx][i].m_Edge;

      //For certain types of edge abutments, the bleedOverBBoxMin, and bleedOverBBoxMax need to 
      //  be flipped: the cases are 
      // if a left   edge mates with a left or bottom  edge on the neighbor
      // if a top    edge mates with a top or right edge on the neighbor
      // if a right  edge mates with a right or top edge on the neighbor
      // if a bottom edge mates with a bottom or left  edge on the neighbor
      //Seeing as the edges are enumerated as follows 
      // left   =0 
      // right  =1 
      // top    =2 
      // bottom =3            
      // 
      // so if the edge enums are the same, or the sum of the enums == 3, 
      //  the bbox needs to be flipped
      if( (i == neighborEdge) || ((i+neighborEdge) == 3) )
      {
        bleedOverBBoxMin[i] = (a_SrcSize-1) - bleedOverBBoxMin[i];
        bleedOverBBoxMax[i] = (a_SrcSize-1) - bleedOverBBoxMax[i];
      }


      //The way the bounding box is extended onto the neighboring face
      // depends on which edge of neighboring face abuts with this one
      switch(sg_CubeNgh[faceIdx][i].m_Edge)
      {
      case CP_EDGE_LEFT:
        a_FilterExtents[neighborFace].Augment(0, bleedOverBBoxMin[i], 0);
        a_FilterExtents[neighborFace].Augment(bleedOverAmount[i], bleedOverBBoxMax[i], 0);
        break;
      case CP_EDGE_RIGHT:                
        a_FilterExtents[neighborFace].Augment( (a_SrcSize-1), bleedOverBBoxMin[i], 0);
        a_FilterExtents[neighborFace].Augment( (a_SrcSize-1) - bleedOverAmount[i], bleedOverBBoxMax[i], 0);
        break;
      case CP_EDGE_TOP:   
        a_FilterExtents[neighborFace].Augment(bleedOverBBoxMin[i], 0, 0);
        a_FilterExtents[neighborFace].Augment(bleedOverBBoxMax[i], bleedOverAmount[i], 0);
        break;
      case CP_EDGE_BOTTOM:   
        a_FilterExtents[neighborFace].Augment(bleedOverBBoxMin[i], (a_SrcSize-1), 0);
        a_FilterExtents[neighborFace].Augment(bleedOverBBoxMax[i], (a_SrcSize-1) - bleedOverAmount[i], 0);            
        break;
      }

      //clamp filter extents in non-center tap faces to remain within surface
      a_FilterExtents[neighborFace].ClampMin(0, 0, 0);
      a_FilterExtents[neighborFace].ClampMax(a_SrcSize-1, a_SrcSize-1, 0);
    }

    //If the bleed over amount bleeds past the adjacent face onto the opposite face 
    // from the center tap face, then process the opposite face entirely for now. 
    //Note that the cases in which this happens, what usually happens is that 
    // more than one edge bleeds onto the opposite face, and the bounding box 
    // encompasses the entire cube map face.
    if(bleedOverAmount[i] > a_SrcSize)
    {
      unsigned int oppositeFaceIdx; 

      //determine opposite face 
      switch(faceIdx)
      {
      case CP_FACE_X_POS:
        oppositeFaceIdx = CP_FACE_X_NEG;
        break;
      case CP_FACE_X_NEG:
        oppositeFaceIdx = CP_FACE_X_POS;
        break;
      case CP_FACE_Y_POS:
        oppositeFaceIdx = CP_FACE_Y_NEG;
        break;
      case CP_FACE_Y_NEG:
        oppositeFaceIdx = CP_FACE_Y_POS;
        break;
      case CP_FACE_Z_POS:
        oppositeFaceIdx = CP_FACE_Z_NEG;
        break;
      case CP_FACE_Z_NEG:
        oppositeFaceIdx = CP_FACE_Z_POS;
        break;
      default:
        break;
      }

      //just encompass entire face for now
      a_FilterExtents[oppositeFaceIdx].Augment(0, 0, 0);
      a_FilterExtents[oppositeFaceIdx].Augment((a_SrcSize-1), (a_SrcSize-1), 0);            
    }
  }

  minV=minV;
}


//--------------------------------------------------------------------------------------
//ProcessFilterExtents 
//  Process bounding box in each cube face 
//
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::ProcessFilterExtents(float *a_CenterTapDir, float a_DotProdThresh, 
                                             CBBoxInt32 *a_FilterExtents, CImageSurface *a_NormCubeMap, CImageSurface *a_SrcCubeMap, 
                                             CP_ITYPE *a_DstVal, unsigned int a_FilterType, bool a_bUseSolidAngleWeighting,
                                             float a_SpecularPower, int a_LightingModel)
{
  int iFaceIdx, u, v;
  int faceWidth;
  int k;

  //pointers used to walk across the image surface to accumulate taps
  CP_ITYPE *normCubeRowStartPtr;
  CP_ITYPE *srcCubeRowStartPtr;
  CP_ITYPE *texelVect;


  //accumulators are 64-bit floats in order to have the precision needed 
  // over a summation of a large number of pixels 
  double dstAccum[4];
  double weightAccum;

  CP_ITYPE tapDotProd;   //dot product between center tap and current tap

  int normCubePitch;
  int srcCubePitch;
  int normCubeRowWalk;
  int srcCubeRowWalk;

  int uStart, uEnd;
  int vStart, vEnd;

  int nSrcChannels; 

  nSrcChannels = a_SrcCubeMap[0].m_NumChannels;

  //norm cube map and srcCubeMap have same face width
  faceWidth = a_NormCubeMap[0].m_Width;

  //amount to add to pointer to move to next scanline in images
  normCubePitch = faceWidth * a_NormCubeMap[0].m_NumChannels;
  srcCubePitch = faceWidth * a_SrcCubeMap[0].m_NumChannels;

  //dest accum
  for(k=0; k<m_NumChannels; k++)
  {
    dstAccum[k] = 0.0f;
  }

  weightAccum = 0.0f;

  // SL BEGIN
  // Add a more efficient path (without test and switch) for cosine power,
  // Basically just a copy past.
  if (a_FilterType != CP_FILTER_TYPE_COSINE_POWER)
  {
    // SL END
    //iterate over cubefaces
    for(iFaceIdx=0; iFaceIdx<6; iFaceIdx++ )
    {
      //if bbox is non empty
      if(a_FilterExtents[iFaceIdx].Empty() == false) 
      {
        uStart = a_FilterExtents[iFaceIdx].m_minCoord[0];
        vStart = a_FilterExtents[iFaceIdx].m_minCoord[1];
        uEnd = a_FilterExtents[iFaceIdx].m_maxCoord[0];
        vEnd = a_FilterExtents[iFaceIdx].m_maxCoord[1];

        normCubeRowStartPtr = a_NormCubeMap[iFaceIdx].m_ImgData + (a_NormCubeMap[iFaceIdx].m_NumChannels * 
          ((vStart * faceWidth) + uStart) );

        srcCubeRowStartPtr = a_SrcCubeMap[iFaceIdx].m_ImgData + (a_SrcCubeMap[iFaceIdx].m_NumChannels * 
          ((vStart * faceWidth) + uStart) );

        //note that <= is used to ensure filter extents always encompass at least one pixel if bbox is non empty
        for(v = vStart; v <= vEnd; v++)
        {
          normCubeRowWalk = 0;
          srcCubeRowWalk = 0;

          for(u = uStart; u <= uEnd; u++)
          {
            //pointer to direction in cube map associated with texel
            texelVect = (normCubeRowStartPtr + normCubeRowWalk);

            //check dot product to see if texel is within cone
            tapDotProd = VM_DOTPROD3(texelVect, a_CenterTapDir);

            if( tapDotProd >= a_DotProdThresh )
            {
              CP_ITYPE weight;

              //for now just weight all taps equally, but ideally
              // weight should be proportional to the solid angle of the tap
              if(a_bUseSolidAngleWeighting == true)
              {   //solid angle stored in 4th channel of normalizer/solid angle cube map
                weight = *(texelVect+3); 
              }
              else
              {   //all taps equally weighted
                weight = 1.0f;          
              }

              switch(a_FilterType)
              {
              case CP_FILTER_TYPE_CONE:                                
              case CP_FILTER_TYPE_ANGULAR_GAUSSIAN:
                {
                  //weights are in same lookup table for both of these filter types
                  weight *= m_FilterLUT[(int)(tapDotProd * (m_NumFilterLUTEntries - 1))];
                }
                break;
              case CP_FILTER_TYPE_COSINE:
                {
                  if(tapDotProd > 0.0f)
                  {
                    weight *= tapDotProd;
                  }
                  else
                  {
                    weight = 0.0f;
                  }
                }
                break;
              case CP_FILTER_TYPE_DISC:
              default:
                break;
              }

              //iterate over channels
              for(k=0; k<nSrcChannels; k++)   //(aSrcCubeMap[iFaceIdx].m_NumChannels) //up to 4 channels 
              {
                dstAccum[k] += weight * *(srcCubeRowStartPtr + srcCubeRowWalk);
                srcCubeRowWalk++;                            
              } 

              weightAccum += weight; //accumulate weight
            }
            else
            {   
              //step across source pixel
              srcCubeRowWalk += nSrcChannels;                    
            }

            normCubeRowWalk += a_NormCubeMap[iFaceIdx].m_NumChannels;
          }

          normCubeRowStartPtr += normCubePitch;
          srcCubeRowStartPtr += srcCubePitch;
        }       
      }
    }
    // SL BEGIN
  }
  else // if (a_FilterType != CP_FILTER_TYPE_COSINE_POWER)
  {

    int IsPhongBRDF = (a_LightingModel == CP_LIGHTINGMODEL_PHONG_BRDF || a_LightingModel == CP_LIGHTINGMODEL_BLINN_BRDF) ? 1 : 0; // This value will be added to the specular power

    //iterate over cubefaces
    for(iFaceIdx=0; iFaceIdx<6; iFaceIdx++ )
    {
      //if bbox is non empty
      if(a_FilterExtents[iFaceIdx].Empty() == false) 
      {
        uStart = a_FilterExtents[iFaceIdx].m_minCoord[0];
        vStart = a_FilterExtents[iFaceIdx].m_minCoord[1];
        uEnd = a_FilterExtents[iFaceIdx].m_maxCoord[0];
        vEnd = a_FilterExtents[iFaceIdx].m_maxCoord[1];

        normCubeRowStartPtr = a_NormCubeMap[iFaceIdx].m_ImgData + (a_NormCubeMap[iFaceIdx].m_NumChannels * 
          ((vStart * faceWidth) + uStart) );

        srcCubeRowStartPtr = a_SrcCubeMap[iFaceIdx].m_ImgData + (a_SrcCubeMap[iFaceIdx].m_NumChannels * 
          ((vStart * faceWidth) + uStart) );

        //note that <= is used to ensure filter extents always encompass at least one pixel if bbox is non empty
        for(v = vStart; v <= vEnd; v++)
        {
          normCubeRowWalk = 0;
          srcCubeRowWalk = 0;

          for(u = uStart; u <= uEnd; u++)
          {
            //pointer to direction in cube map associated with texel
            texelVect = (normCubeRowStartPtr + normCubeRowWalk);

            //check dot product to see if texel is within cone
            tapDotProd = VM_DOTPROD3(texelVect, a_CenterTapDir);

            if( tapDotProd >= a_DotProdThresh && tapDotProd > 0.0f)
            {
              CP_ITYPE weight;

              //solid angle stored in 4th channel of normalizer/solid angle cube map
              weight = *(texelVect+3); 

              // Here we decide if we use a Phong/Blinn or a Phong/Blinn BRDF.
              // Phong/Blinn BRDF is just the Phong/Blinn model multiply by the cosine of the lambert law
              // so just adding one to specularpower do the trick.					   
              weight *= pow(tapDotProd, (a_SpecularPower + (float)IsPhongBRDF));

              //iterate over channels
              for(k=0; k<nSrcChannels; k++)   //(aSrcCubeMap[iFaceIdx].m_NumChannels) //up to 4 channels 
              {
                dstAccum[k] += weight * *(srcCubeRowStartPtr + srcCubeRowWalk);
                srcCubeRowWalk++;
              } 

              weightAccum += weight; //accumulate weight
            }
            else
            {   
              //step across source pixel
              srcCubeRowWalk += nSrcChannels;                    
            }

            normCubeRowWalk += a_NormCubeMap[iFaceIdx].m_NumChannels;
          }

          normCubeRowStartPtr += normCubePitch;
          srcCubeRowStartPtr += srcCubePitch;
        }       
      }
    }
  } // else // (a_FilterType != CP_FILTER_TYPE_COSINE_POWER)
  // SL END


  //divide through by weights if weight is non zero
  if(weightAccum != 0.0f)
  {
    for(k=0; k<m_NumChannels; k++)
    {
      a_DstVal[k] = (float)(dstAccum[k] / weightAccum);
    }
  }
  else
  {   //otherwise sample nearest
    CP_ITYPE *texelPtr;

    texelPtr = GetCubeMapTexelPtr(a_CenterTapDir, a_SrcCubeMap);

    for(k=0; k<m_NumChannels; k++)
    {
      a_DstVal[k] = texelPtr[k];
    }
  }
}


//--------------------------------------------------------------------------------------
// Fixup cube edges
//
// average texels on cube map faces across the edges
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::FixupCubeEdges(CImageSurface *a_CubeMap, int a_FixupType, int a_FixupWidth)
{
  int i, j, k;
  int face;
  int edge;
  int neighborFace;
  int neighborEdge;

  int nChannels = a_CubeMap[0].m_NumChannels;
  int size = a_CubeMap[0].m_Width;

  CPCubeMapNeighbor neighborInfo;

  CP_ITYPE* edgeStartPtr;
  CP_ITYPE* neighborEdgeStartPtr;

  int edgeWalk;
  int neighborEdgeWalk;

  //pointer walk to walk one texel away from edge in perpendicular direction
  int edgePerpWalk;
  int neighborEdgePerpWalk;

  //number of texels inward towards cubeface center to apply fixup to
  int fixupDist;
  int iFixup;   

  // note that if functionality to filter across the three texels for each corner, then 
  CP_ITYPE *cornerPtr[8][3];      //indexed by corner and face idx
  CP_ITYPE *faceCornerPtrs[4];    //corner pointers for face
  int cornerNumPtrs[8];         //indexed by corner and face idx
  int iCorner;                  //corner iterator
  int iFace;                    //iterator for faces
  int corner;

  //if there is no fixup, or fixup width = 0, do nothing
  if((a_FixupType == CP_FIXUP_NONE) ||
    (a_FixupWidth == 0) 
    // SL BEGIN
    || (a_FixupType == CP_FIXUP_BENT && a_CubeMap[0].m_Width != 1) // In case of Bent Fixup and width of 1, we take the average of the texel color.
    || (a_FixupType == CP_FIXUP_WARP && a_CubeMap[0].m_Width != 1)
    || (a_FixupType == CP_FIXUP_STRETCH && a_CubeMap[0].m_Width != 1)	  
    // SL END
    )
  {
    return;
  }

  //special case 1x1 cubemap, average face colors
  if( a_CubeMap[0].m_Width == 1 )
  {
    //iterate over channels
    for(k=0; k<nChannels; k++)
    {   
      CP_ITYPE accum = 0.0f;

      //iterate over faces to accumulate face colors
      for(iFace=0; iFace<6; iFace++)
      {
        accum += *(a_CubeMap[iFace].m_ImgData + k);
      }

      //compute average over 6 face colors
      accum /= 6.0f;

      //iterate over faces to distribute face colors
      for(iFace=0; iFace<6; iFace++)
      {
        *(a_CubeMap[iFace].m_ImgData + k) = accum;
      }
    }

    return;
  }


  //iterate over corners
  for(iCorner = 0; iCorner < 8; iCorner++ )
  {
    cornerNumPtrs[iCorner] = 0;
  }

  //iterate over faces to collect list of corner texel pointers
  for(iFace=0; iFace<6; iFace++ )
  {
    //the 4 corner pointers for this face
    faceCornerPtrs[0] = a_CubeMap[iFace].m_ImgData;
    faceCornerPtrs[1] = a_CubeMap[iFace].m_ImgData + ( (size - 1) * nChannels );
    faceCornerPtrs[2] = a_CubeMap[iFace].m_ImgData + ( (size) * (size - 1) * nChannels );
    faceCornerPtrs[3] = a_CubeMap[iFace].m_ImgData + ( (((size) * (size - 1)) + (size - 1)) * nChannels );

    //iterate over face corners to collect cube corner pointers
    for(i=0; i<4; i++ )
    {
      corner = sg_CubeCornerList[iFace][i];   
      cornerPtr[corner][ cornerNumPtrs[corner] ] = faceCornerPtrs[i];
      cornerNumPtrs[corner]++;
    }
  }


  //iterate over corners to average across corner tap values
  for(iCorner = 0; iCorner < 8; iCorner++ )
  {
    for(k=0; k<nChannels; k++)
    {             
      CP_ITYPE cornerTapAccum;

      cornerTapAccum = 0.0f;

      //iterate over corner texels and average results
      for(i=0; i<3; i++ )
      {
        cornerTapAccum += *(cornerPtr[iCorner][i] + k);
      }

      //divide by 3 to compute average of corner tap values
      cornerTapAccum *= (1.0f / 3.0f);

      //iterate over corner texels and average results
      for(i=0; i<3; i++ )
      {
        *(cornerPtr[iCorner][i] + k) = cornerTapAccum;
      }
    }
  }   


  //maximum width of fixup region is one half of the cube face size
  fixupDist = VM_MIN( a_FixupWidth, size / 2);

  //iterate over the twelve edges of the cube to average across edges
  for(i=0; i<12; i++)
  {
    face = sg_CubeEdgeList[i][0];
    edge = sg_CubeEdgeList[i][1];

    neighborInfo = sg_CubeNgh[face][edge];
    neighborFace = neighborInfo.m_Face;
    neighborEdge = neighborInfo.m_Edge;

    edgeStartPtr = a_CubeMap[face].m_ImgData;
    neighborEdgeStartPtr = a_CubeMap[neighborFace].m_ImgData;
    edgeWalk = 0;
    neighborEdgeWalk = 0;

    //amount to pointer to sample taps away from cube face
    edgePerpWalk = 0;
    neighborEdgePerpWalk = 0;

    //Determine walking pointers based on edge type
    // e.g. CP_EDGE_LEFT, CP_EDGE_RIGHT, CP_EDGE_TOP, CP_EDGE_BOTTOM
    switch(edge)
    {
    case CP_EDGE_LEFT:
      // no change to faceEdgeStartPtr  
      edgeWalk = nChannels * size;
      edgePerpWalk = nChannels;
      break;
    case CP_EDGE_RIGHT:
      edgeStartPtr += (size - 1) * nChannels;
      edgeWalk = nChannels * size;
      edgePerpWalk = -nChannels;
      break;
    case CP_EDGE_TOP:
      // no change to faceEdgeStartPtr  
      edgeWalk = nChannels;
      edgePerpWalk = nChannels * size;
      break;
    case CP_EDGE_BOTTOM:
      edgeStartPtr += (size) * (size - 1) * nChannels;
      edgeWalk = nChannels;
      edgePerpWalk = -(nChannels * size);
      break;
    }

    //For certain types of edge abutments, the neighbor edge walk needs to 
    //  be flipped: the cases are 
    // if a left   edge mates with a left or bottom  edge on the neighbor
    // if a top    edge mates with a top or right edge on the neighbor
    // if a right  edge mates with a right or top edge on the neighbor
    // if a bottom edge mates with a bottom or left  edge on the neighbor
    //Seeing as the edges are enumerated as follows 
    // left   =0 
    // right  =1 
    // top    =2 
    // bottom =3            
    // 
    //If the edge enums are the same, or the sum of the enums == 3, 
    //  the neighbor edge walk needs to be flipped
    if( (edge == neighborEdge) || ((edge + neighborEdge) == 3) )
    {   //swapped direction neighbor edge walk
      switch(neighborEdge)
      {
      case CP_EDGE_LEFT:  //start at lower left and walk up
        neighborEdgeStartPtr += (size - 1) * (size) *  nChannels;
        neighborEdgeWalk = -(nChannels * size);
        neighborEdgePerpWalk = nChannels;
        break;
      case CP_EDGE_RIGHT: //start at lower right and walk up
        neighborEdgeStartPtr += ((size - 1)*(size) + (size - 1)) * nChannels;
        neighborEdgeWalk = -(nChannels * size);
        neighborEdgePerpWalk = -nChannels;
        break;
      case CP_EDGE_TOP:   //start at upper right and walk left
        neighborEdgeStartPtr += (size - 1) * nChannels;
        neighborEdgeWalk = -nChannels;
        neighborEdgePerpWalk = (nChannels * size);
        break;
      case CP_EDGE_BOTTOM: //start at lower right and walk left
        neighborEdgeStartPtr += ((size - 1)*(size) + (size - 1)) * nChannels;
        neighborEdgeWalk = -nChannels;
        neighborEdgePerpWalk = -(nChannels * size);
        break;
      }            
    }
    else
    { //swapped direction neighbor edge walk
      switch(neighborEdge)
      {
      case CP_EDGE_LEFT: //start at upper left and walk down
        //no change to neighborEdgeStartPtr for this case since it points 
        // to the upper left corner already
        neighborEdgeWalk = nChannels * size;
        neighborEdgePerpWalk = nChannels;
        break;
      case CP_EDGE_RIGHT: //start at upper right and walk down
        neighborEdgeStartPtr += (size - 1) * nChannels;
        neighborEdgeWalk = nChannels * size;
        neighborEdgePerpWalk = -nChannels;
        break;
      case CP_EDGE_TOP:   //start at upper left and walk left
        //no change to neighborEdgeStartPtr for this case since it points 
        // to the upper left corner already
        neighborEdgeWalk = nChannels;
        neighborEdgePerpWalk = (nChannels * size);
        break;
      case CP_EDGE_BOTTOM: //start at lower left and walk left
        neighborEdgeStartPtr += (size) * (size - 1) * nChannels;
        neighborEdgeWalk = nChannels;
        neighborEdgePerpWalk = -(nChannels * size);
        break;
      }
    }


    //Perform edge walk, to average across the 12 edges and smoothly propagate change to 
    //nearby neighborhood

    //step ahead one texel on edge
    edgeStartPtr += edgeWalk;
    neighborEdgeStartPtr += neighborEdgeWalk;

    // note that this loop does not process the corner texels, since they have already been
    //  averaged across faces across earlier
    for(j=1; j<(size - 1); j++)       
    {             
      //for each set of taps along edge, average them
      // and rewrite the results into the edges
      for(k = 0; k<nChannels; k++)
      {             
        CP_ITYPE edgeTap, neighborEdgeTap, avgTap;  //edge tap, neighborEdgeTap and the average of the two
        CP_ITYPE edgeTapDev, neighborEdgeTapDev;

        edgeTap = *(edgeStartPtr + k);
        neighborEdgeTap = *(neighborEdgeStartPtr + k);

        //compute average of tap intensity values
        avgTap = 0.5f * (edgeTap + neighborEdgeTap);

        //propagate average of taps to edge taps
        (*(edgeStartPtr + k)) = avgTap;
        (*(neighborEdgeStartPtr + k)) = avgTap;

        edgeTapDev = edgeTap - avgTap;
        neighborEdgeTapDev = neighborEdgeTap - avgTap;

        //iterate over taps in direction perpendicular to edge, and 
        //  adjust intensity values gradualy to obscure change in intensity values of 
        //  edge averaging.
        for(iFixup = 1; iFixup < fixupDist; iFixup++)
        {
          //fractional amount to apply change in tap intensity along edge to taps 
          //  in a perpendicular direction to edge 
          CP_ITYPE fixupFrac = (CP_ITYPE)(fixupDist - iFixup) / (CP_ITYPE)(fixupDist); 
          CP_ITYPE fixupWeight;

          switch(a_FixupType )
          {
          case CP_FIXUP_PULL_LINEAR:
            {
              fixupWeight = fixupFrac;
            }
            break;
          case CP_FIXUP_PULL_HERMITE:
            {
              //hermite spline interpolation between 1 and 0 with both pts derivatives = 0 
              // e.g. smooth step
              // the full formula for hermite interpolation is:
              //              
              //                  [  2  -2   1   1 ][ p0 ] 
              // [t^3  t^2  t  1 ][ -3   3  -2  -1 ][ p1 ]
              //                  [  0   0   1   0 ][ d0 ]
              //                  [  1   0   0   0 ][ d1 ]
              // 
              // Where p0 and p1 are the point locations and d0, and d1 are their respective derivatives
              // t is the parameteric coordinate used to specify an interpoltion point on the spline
              // and ranges from 0 to 1.
              //  if p0 = 0 and p1 = 1, and d0 and d1 = 0, the interpolation reduces to
              //
              //  p(t) =  - 2t^3 + 3t^2
              fixupWeight = ((-2.0 * fixupFrac + 3.0) * fixupFrac * fixupFrac);
            }
            break;
          case CP_FIXUP_AVERAGE_LINEAR:
            {
              fixupWeight = fixupFrac;

              //perform weighted average of edge tap value and current tap
              // fade off weight linearly as a function of distance from edge
              edgeTapDev = 
                (*(edgeStartPtr + (iFixup * edgePerpWalk) + k)) - avgTap;
              neighborEdgeTapDev = 
                (*(neighborEdgeStartPtr + (iFixup * neighborEdgePerpWalk) + k)) - avgTap;
            }
            break;
          case CP_FIXUP_AVERAGE_HERMITE:
            {
              fixupWeight = ((-2.0 * fixupFrac + 3.0) * fixupFrac * fixupFrac);

              //perform weighted average of edge tap value and current tap
              // fade off weight using hermite spline with distance from edge
              //  as parametric coordinate
              edgeTapDev = 
                (*(edgeStartPtr + (iFixup * edgePerpWalk) + k)) - avgTap;
              neighborEdgeTapDev = 
                (*(neighborEdgeStartPtr + (iFixup * neighborEdgePerpWalk) + k)) - avgTap;
            }
            break;
          }

          // vary intensity of taps within fixup region toward edge values to hide changes made to edge taps
          *(edgeStartPtr + (iFixup * edgePerpWalk) + k) -= (fixupWeight * edgeTapDev);
          *(neighborEdgeStartPtr + (iFixup * neighborEdgePerpWalk) + k) -= (fixupWeight * neighborEdgeTapDev);
        }

      }

      edgeStartPtr += edgeWalk;
      neighborEdgeStartPtr += neighborEdgeWalk;
    }        
  }
}


//--------------------------------------------------------------------------------------
//Constructor
//--------------------------------------------------------------------------------------
CCubeMapProcessor::CCubeMapProcessor(void)
{
  //int numberOfProcessors = static_cast<int>(std::thread::hardware_concurrency());
  // TODO : In case of command line, we want to use all hardware thread available
  //m_NumFilterThreads  = numberOfProcessors - 1;	// - 1 cause the main core is used for the application.
  // If no extra hardware thread is available the filering will be done the main process.
  //m_NumFilterThreads = std::max(std::min(numberOfProcessors, 6), 1); // We don't handle more than 6 hardware thread (6 face of cubemap) for now
  //sg_ThreadFilterFace = new SThreadFilterFace[m_NumFilterThreads];

  //clear all threads
  //memset(sg_ThreadFilterFace, 0, m_NumFilterThreads * sizeof(SThreadFilterFace));
  // SL END

  m_InputSize = 0;             
  m_OutputSize = 0;             
  m_NumMipLevels = 0;     
  m_NumChannels = 0; 

  m_NumFilterLUTEntries = 0;
  m_FilterLUT = NULL;

  //Constructors are automatically called for m_InputSurface and m_OutputSurface arrays
}


//--------------------------------------------------------------------------------------
//destructor
//--------------------------------------------------------------------------------------
//CCubeMapProcessor::~CCubeMapProcessor()
//{
//    Clear();
//
//	// SL BEGIN
//	CP_SAFE_DELETE_ARRAY(sg_ThreadFilterFace);
//	// SL END
//}


//--------------------------------------------------------------------------------------
// Stop any currently running threads, and clear all allocated data from cube map 
//   processor.
//
// To use the cube map processor after calling Clear(....), you need to call Init(....) 
//   again
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::Clear(void)
{
  int i, j;

  TerminateActiveThreads();

  // SL BEGIN
  /*for(i=0; i<m_NumFilterThreads; i++ )
  {
  sg_ThreadFilterFace[i].m_bThreadInitialized = false;
  }*/
  // SL END

  m_InputSize = 0;             
  m_OutputSize = 0;             
  m_NumMipLevels = 0;     
  m_NumChannels = 0; 

  //Iterate over faces for input images
  for(i=0; i<6; i++)
  {
    m_InputSurface[i].Clear();
  }

  //Iterate over mip chain, and allocate memory for mip-chain
  for(j=0; j<CP_MAX_MIPLEVELS; j++)
  {
    //Iterate over faces for output images
    for(i=0; i<6; i++)
    {
      m_OutputSurface[j][i].Clear();            
    }
  }

  m_NumFilterLUTEntries = 0;
  CP_SAFE_DELETE_ARRAY( m_FilterLUT );
}


//--------------------------------------------------------------------------------------
// Terminates execution of active threads
//
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::TerminateActiveThreads(void)
{
#if 0
  int i;
  // SL BEGIN
  for(i=0; i<m_NumFilterThreads; i++)
  {
    if(sg_ThreadFilterFace[i].m_bThreadInitialized == true)
    {
      if(sg_ThreadFilterFace[i].m_ThreadHandle != NULL)
      {
        TerminateThread(sg_ThreadFilterFace[i].m_ThreadHandle, CP_THREAD_TERMINATED);
        CloseHandle(sg_ThreadFilterFace[i].m_ThreadHandle);
        sg_ThreadFilterFace[i].m_ThreadHandle = NULL;
      }
    }
  }

  // Terminate main thread if needed
  TerminateThread(DumbThreadHandle, CP_THREAD_TERMINATED);

  m_Status = CP_STATUS_FILTER_TERMINATED;
  // SL END
#else
  /*for (int i=0; i<m_NumFilterThreads; i++)
  {
  SThreadFilterFace& face = sg_ThreadFilterFace[i];
  if (face.m_bThreadInitialized)
  {
  face.m_Thread.interrupt();
  }
  }*/
  m_Status = CP_STATUS_FILTER_TERMINATED;
#endif

}


//--------------------------------------------------------------------------------------
//Init cube map processor
//
// note that if a_MaxNumMipLevels is set to 0, the entire mip chain will be created
//  similar to the way the D3D tools allow you to specify 0 to denote an entire mip chain
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::Init(int a_InputSize, int a_OutputSize, int a_MaxNumMipLevels, int a_NumChannels)
{
  int i, j;
  int mipLevelSize;
  int maxNumMipLevels;

  m_Status = CP_STATUS_READY;

  //since input is being modified, terminate any active filtering threads
  TerminateActiveThreads();

  m_InputSize = a_InputSize;
  m_OutputSize = a_OutputSize;

  m_NumChannels = a_NumChannels;

  maxNumMipLevels = a_MaxNumMipLevels;

  //if nax num mip levels is set to 0, set it to generate the entire mip chain
  if(maxNumMipLevels == 0 )
  {
    maxNumMipLevels = CP_MAX_MIPLEVELS;
  }

  //Iterate over faces for input images
  for(i=0; i<6; i++)
  {
    m_InputSurface[i].Init(m_InputSize, m_InputSize, m_NumChannels );
  }

  //zero mip levels constructed so far
  m_NumMipLevels = 0;

  //first miplevel size 
  mipLevelSize = m_OutputSize;

  //Iterate over mip chain, and init CImageSurfaces for mip-chain
  for(j=0; j<a_MaxNumMipLevels; j++)
  {
    //Iterate over faces for output images
    for(i=0; i<6; i++)
    {
      m_OutputSurface[j][i].Init(mipLevelSize, mipLevelSize, a_NumChannels );            
    }

    //next mip level is half size
    mipLevelSize >>= 1;

    m_NumMipLevels++;

    //terminate if mip chain becomes too small
    if(mipLevelSize == 0)
    {            
      return;
    }
  }
}


//--------------------------------------------------------------------------------------
//Copy and convert cube map face data from an external image/surface into this object
//
// a_FaceIdx        = a value 0 to 5 speciying which face to copy into (one of the CP_FACE_? )
// a_Level          = mip level to copy into
// a_SrcType        = data type of image being copyed from (one of the CP_TYPE_? types)
// a_SrcNumChannels = number of channels of the image being copied from (usually 1 to 4)
// a_SrcPitch       = number of bytes per row of the source image being copied from
// a_SrcDataPtr     = pointer to the image data to copy from
// a_Degamma        = original gamma level of input image to undo by degamma
// a_Scale          = scale to apply to pixel values after degamma (in linear space)
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::SetInputFaceData(int a_FaceIdx, int a_SrcType, int a_SrcNumChannels, 
                                         int a_SrcPitch, void *a_SrcDataPtr, float a_MaxClamp, float a_Degamma, float a_Scale)
{
  //since input is being modified, terminate any active filtering threads
  TerminateActiveThreads();

  m_InputSurface[a_FaceIdx].SetImageDataClampDegammaScale( a_SrcType, a_SrcNumChannels, a_SrcPitch, 
    a_SrcDataPtr, a_MaxClamp, a_Degamma, a_Scale );
}


//--------------------------------------------------------------------------------------
//Copy and convert cube map face data from this object into an external image/surface
//
// a_FaceIdx        = a value 0 to 5 speciying which face to copy into (one of the CP_FACE_? )
// a_Level          = mip level to copy into
// a_DstType        = data type of image to copy to (one of the CP_TYPE_? types)
// a_DstNumChannels = number of channels of the image to copy to (usually 1 to 4)
// a_DstPitch       = number of bytes per row of the dest image to copy to
// a_DstDataPtr     = pointer to the image data to copy to
// a_Scale          = scale to apply to pixel values (in linear space) before gamma for output
// a_Gamma          = gamma level to apply to pixels after scaling
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::GetInputFaceData(int a_FaceIdx, int a_DstType, int a_DstNumChannels, 
                                         int a_DstPitch, void *a_DstDataPtr, float a_Scale, float a_Gamma)
{
  m_InputSurface[a_FaceIdx].GetImageDataScaleGamma( a_DstType, a_DstNumChannels, a_DstPitch, 
    a_DstDataPtr, a_Scale, a_Gamma );
}


//--------------------------------------------------------------------------------------
//ChannelSwapInputFaceData
//  swizzle data in first 4 channels for input faces
//
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::ChannelSwapInputFaceData(int a_Channel0Src, int a_Channel1Src, 
                                                 int a_Channel2Src, int a_Channel3Src )
{
  int iFace, u, v, k;
  int size;
  CP_ITYPE texelData[4];
  int channelSrcArray[4];

  //since input is being modified, terminate any active filtering threads
  TerminateActiveThreads();

  size = m_InputSize;

  channelSrcArray[0] = a_Channel0Src;
  channelSrcArray[1] = a_Channel1Src;
  channelSrcArray[2] = a_Channel2Src;
  channelSrcArray[3] = a_Channel3Src;

  //Iterate over faces for input images
  for(iFace=0; iFace<6; iFace++)
  {
    for(v=0; v<m_InputSize; v++ )
    {
      for(u=0; u<m_InputSize; u++ )
      {
        //get channel data
        for(k=0; k<m_NumChannels; k++)
        {
          texelData[k] = *(m_InputSurface[iFace].GetSurfaceTexelPtr(u, v) + k);
        }

        //repack channel data accoring to swizzle information
        for(k=0; k<m_NumChannels; k++)
        {
          *( m_InputSurface[iFace].GetSurfaceTexelPtr(u, v) + k) = 
            texelData[ channelSrcArray[k] ];
        }
      }
    }
  }
}


//--------------------------------------------------------------------------------------
//ChannelSwapOutputFaceData
//  swizzle data in first 4 channels for input faces
//
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::ChannelSwapOutputFaceData(int a_Channel0Src, int a_Channel1Src, 
                                                  int a_Channel2Src, int a_Channel3Src )
{
  int iFace, iMipLevel, u, v, k;
  int size;
  CP_ITYPE texelData[4];
  int channelSrcArray[4];

  //since output is being modified, terminate any active filtering threads
  TerminateActiveThreads();

  size = m_OutputSize;

  channelSrcArray[0] = a_Channel0Src;
  channelSrcArray[1] = a_Channel1Src;
  channelSrcArray[2] = a_Channel2Src;
  channelSrcArray[3] = a_Channel3Src;

  //Iterate over faces for input images
  for(iMipLevel=0; iMipLevel<m_NumMipLevels; iMipLevel++ )
  {
    for(iFace=0; iFace<6; iFace++)
    {
      for(v=0; v<m_OutputSurface[iMipLevel][iFace].m_Height; v++ )
      {
        for(u=0; u<m_OutputSurface[iMipLevel][iFace].m_Width; u++ )
        {
          //get channel data
          for(k=0; k<m_NumChannels; k++)
          {
            texelData[k] = *(m_OutputSurface[iMipLevel][iFace].GetSurfaceTexelPtr(u, v) + k);
          }

          //repack channel data accoring to swizzle information
          for(k=0; k<m_NumChannels; k++)
          {
            *(m_OutputSurface[iMipLevel][iFace].GetSurfaceTexelPtr(u, v) + k) = texelData[ channelSrcArray[k] ];
          }
        }
      }
    }
  }
}


//--------------------------------------------------------------------------------------
//Copy and convert cube map face data out of this class into an external image/surface
//
// a_FaceIdx        = a value 0 to 5 specifying which face to copy from (one of the CP_FACE_? )
// a_Level          = mip level to copy from
// a_DstType        = data type of image to copyed into (one of the CP_TYPE_? types)
// a_DstNumChannels = number of channels of the image to copyed into  (usually 1 to 4)
// a_DstPitch       = number of bytes per row of the source image to copyed into 
// a_DstDataPtr     = pointer to the image data to copyed into 
// a_Scale          = scale to apply to pixel values (in linear space) before gamma for output
// a_Gamma          = gamma level to apply to pixels after scaling
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::GetOutputFaceData(int a_FaceIdx, int a_Level, int a_DstType, 
                                          int a_DstNumChannels, int a_DstPitch, void *a_DstDataPtr, float a_Scale, float a_Gamma )   
{
  switch(a_DstType)
  {
  case CP_VAL_UNORM8:
  case CP_VAL_UNORM8_BGRA:
  case CP_VAL_UNORM16:
  case CP_VAL_FLOAT16:
  case CP_VAL_FLOAT32:
    {
      m_OutputSurface[a_Level][a_FaceIdx].GetImageDataScaleGamma( a_DstType, a_DstNumChannels, 
        a_DstPitch, a_DstDataPtr, a_Scale, a_Gamma ); 
    }
    break;
  default:
    break;
  }
}


//--------------------------------------------------------------------------------------
//Cube map filtering and mip chain generation.
// the cube map filtereing is specified using a number of parameters:
// Filtering per miplevel is specified using 2D cone angle (in degrees) that 
//  indicates the region of the hemisphere to filter over for each tap. 
//                
// Note that the top mip level is also a filtered version of the original input images 
//  as well in order to create mip chains for diffuse environment illumination.
// The cone angle for the top level is specified by a_BaseAngle.  This can be used to
//  generate mipchains used to store the resutls of preintegration across the hemisphere.
//
// Then the mip angle used to genreate the next level of the mip chain from the first level 
//  is a_InitialMipAngle
//
// The angle for the subsequent levels of the mip chain are specified by their parents 
//  filtering angle and a per-level scale and bias
//   newAngle = oldAngle * a_MipAnglePerLevelScale;
//
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::FilterCubeMapMipChain()
{
  //Build filter lookup tables based on the source miplevel size
  // SL BEGIN
  PrecomputeFilterLookupTables(m_FilterType, m_InputSurface[0].m_Width, m_BaseFilterAngle, m_FixupType);
  // SL END

  //initialize thread progress
  // SL BEGIN
  m_ThreadProgress[0].m_CurrentMipLevel = 0;
  m_ThreadProgress[0].m_CurrentRow = 0;
  m_ThreadProgress[0].m_CurrentFace = 0;
  // SL END

  // SL BEGIN
  // If diffuse convolution is required, go through SH filtering for the base level
  if (m_bIrradianceCubemap)
  {
    // Note that we redo the normalization as in PrecomputeFilterLookupTables
    // Don't care for now because we should use Multithread approach
    SHFilterCubeMap(m_bUseSolidAngle, m_FixupType);
  }
  else
  {
    // generate top level mipmap
    for (int i=0; i<6; i++) {
      //SThreadFilterFace& face = sg_ThreadFilterFace[i];
      m_ThreadProgress[i].m_CurrentMipLevel = 0;
      m_ThreadProgress[i].m_CurrentRow = 0;
      m_ThreadProgress[i].m_CurrentFace = i;
      m_Threads[i] = std::thread(&CCubeMapProcessor::FilterCubeSurfaces,
        this,
        m_InputSurface, 
        m_OutputSurface[0], 
        m_BaseFilterAngle,
        i);
    }

    for (int i=0; i<6; i++) {
      m_Threads[i].join();
    }

    FixupCubeEdges(m_OutputSurface[0], m_FixupType, m_FixupWidth);

    float coneAngle = m_InitialMipAngle;
    if (m_FilterType == CP_FILTER_TYPE_COSINE_POWER) {
      m_FilterType = CP_FILTER_TYPE_COSINE;
    }
    // generate subsequent levels
    for (int i=1; i<m_NumMipLevels; i++) {
      PrecomputeFilterLookupTables(m_FilterType, m_OutputSurface[i-1][0].m_Width, coneAngle, m_FixupType);
      for (int j=0; j<6; j++) {
        m_ThreadProgress[j].m_CurrentMipLevel = i;
        m_ThreadProgress[j].m_CurrentRow = 0;
        m_ThreadProgress[j].m_CurrentFace = j;
        m_Threads[j] = std::thread(&CCubeMapProcessor::FilterCubeSurfaces,
          this,
          m_OutputSurface[i-1], 
          m_OutputSurface[i], 
          coneAngle,
          j);
      }
      for (int j=0; j<6; j++) {
        m_Threads[j].join();
      }
      FixupCubeEdges(m_OutputSurface[i], m_FixupType, m_FixupWidth);
      coneAngle *= m_MipAnglePerLevelScale;
    }
  }

  m_Status = CP_STATUS_FILTER_COMPLETED;
}


//--------------------------------------------------------------------------------------
//Builds the following lookup tables prior to filtering:
//  -normalizer cube map
//  -tap weight lookup table
// 
//--------------------------------------------------------------------------------------
// SL BEGIN
void CCubeMapProcessor::PrecomputeFilterLookupTables(unsigned int a_FilterType, int a_SrcCubeMapWidth, float a_FilterConeAngle, int a_FixupType)
{
  float srcTexelAngle;
  int   iCubeFace;

  //angle about center tap that defines filter cone
  float filterAngle;

  //min angle a src texel can cover (in degrees)
  srcTexelAngle = (180.0f / (float)CP_PI) * atan2f(1.0f, (float)a_SrcCubeMapWidth);  

  //filter angle is 1/2 the cone angle
  filterAngle = a_FilterConeAngle / 2.0f;

  //ensure filter angle is larger than a texel
  if(filterAngle < srcTexelAngle)
  {
    filterAngle = srcTexelAngle;    
  }

  //ensure filter cone is always smaller than the hemisphere
  if(filterAngle > 90.0f)
  {
    filterAngle = 90.0f;
  }

  //build lookup table for tap weights based on angle between current tap and center tap
  BuildAngleWeightLUT(a_SrcCubeMapWidth * 2, a_FilterType, filterAngle);

  //clear pre-existing normalizer cube map
  for(iCubeFace=0; iCubeFace<6; iCubeFace++)
  {
    m_NormCubeMap[iCubeFace].Clear();            
  }

  //Normalized vectors per cubeface and per-texel solid angle 
  // SL BEGIN
  for (int i=0; i<6; i++) {
    m_Threads[i] = std::thread(&CCubeMapProcessor::BuildNormalizerSolidAngleCubemap,
      this,
      a_SrcCubeMapWidth,
      m_NormCubeMap,
      i,
      a_FixupType);
  }
  for (int i=0; i<6; i++) {
    m_Threads[i].join();
  }
  // SL END

}

//--------------------------------------------------------------------------------------
//The key to the speed of these filtering routines is to quickly define a per-face 
//  bounding box of pixels which enclose all the taps in the filter kernel efficiently.  
//  Later these pixels are selectively processed based on their dot products to see if 
//  they reside within the filtering cone.
//
//This is done by computing the smallest per-texel angle to get a conservative estimate 
// of the number of texels needed to be covered in width and height order to filter the
// region.  the bounding box for the center taps face is defined first, and if the 
// filtereing region bleeds onto the other faces, bounding boxes for the other faces are 
// defined next
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::FilterCubeSurfaces(CImageSurface *a_SrcCubeMap, CImageSurface *a_DstCubeMap, float a_FilterConeAngle, int a_FaceIdx)
{
  CBBoxInt32    filterExtents[6];   //bounding box per face to specify region to process
  int u, v;    
  int srcSize = a_SrcCubeMap[0].m_Width;
  int dstSize = a_DstCubeMap[0].m_Width;

  float srcTexelAngle;
  float dotProdThresh;
  int   filterSize;

  //angle about center tap to define filter cone
  float filterAngle;

  //min angle a src texel can cover (in degrees)
  srcTexelAngle = (180.0f / (float)CP_PI) * atan2f(1.0f, (float)srcSize);  

  //filter angle is 1/2 the cone angle
  filterAngle = a_FilterConeAngle / 2.0f;

  //ensure filter angle is larger than a texel
  if(filterAngle < srcTexelAngle)
  {
    filterAngle = srcTexelAngle;    
  }

  //ensure filter cone is always smaller than the hemisphere
  if(filterAngle > 90.0f)
  {
    filterAngle = 90.0f;
  }

  //the maximum number of texels in 1D the filter cone angle will cover
  //  used to determine bounding box size for filter extents
  filterSize = (int)ceil(filterAngle / srcTexelAngle);   

  //ensure conservative region always covers at least one texel
  if(filterSize < 1)
  {
    filterSize = 1;
  }

  //dotProdThresh threshold based on cone angle to determine whether or not taps 
  // reside within the cone angle
  dotProdThresh = cosf( ((float)CP_PI / 180.0f) * filterAngle );

  //thread progress
  // SL BEGIN
  m_ThreadProgress[a_FaceIdx].m_StartFace = a_FaceIdx;
  m_ThreadProgress[a_FaceIdx].m_EndFace = a_FaceIdx;
  // SL END

  //process required faces
  CP_ITYPE *texelPtr = a_DstCubeMap[a_FaceIdx].m_ImgData;

  m_ThreadProgress[a_FaceIdx].m_CurrentFace = a_FaceIdx;

  //iterate over dst cube map face texel
  for(v = 0; v < dstSize; v++)
  {
    m_ThreadProgress[a_FaceIdx].m_CurrentRow = v;

    for(u=0; u<dstSize; u++)
    {
      float centerTapDir[3];  //direction of center tap

      //get center tap direction
      TexelCoordToVect(a_FaceIdx, (float)u, (float)v, dstSize, centerTapDir, m_FixupType);

      //clear old per-face filter extents
      ClearFilterExtents(filterExtents);

      //define per-face filter extents
      DetermineFilterExtents(centerTapDir, srcSize, filterSize, filterExtents );

      //perform filtering of src faces using filter extents 
      ProcessFilterExtents(centerTapDir, dotProdThresh, filterExtents, m_NormCubeMap, a_SrcCubeMap, texelPtr, m_FilterType, m_bUseSolidAngle, m_SpecularPower, m_LightingModel);

      texelPtr += a_DstCubeMap[a_FaceIdx].m_NumChannels;
    }            
  }
}


// SL BEGIN
// This function return the BaseFilterAngle require by cubemapgen to its FilterExtends
// It allow to optimize the texel to access base on the specular power.
static float GetBaseFilterAngle(float cosinePower)
{
  // We want to find the alpha such that:
  // cos(alpha)^cosinePower = epsilon
  // That's: acos(epsilon^(1/cosinePower))
  const float threshold = 0.000001f;  // Empirical threshold (Work perfectly, didn't check for a more big number, may get some performance and still god approximation)
  float Angle = 180.0f;
  if (Angle != 0.0f)
  {
    Angle = acosf(powf(threshold, 1.0f / cosinePower)); 
    Angle *= 180.0f / (float)CP_PI; // Convert to degree
    Angle *= 2.0f; // * 2.0f because cubemapgen divide by 2 later
  }

  return Angle;
}
// SL END

//--------------------------------------------------------------------------------------
//starts a new thread to execute the filtering options
//
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::InitiateFiltering(float a_BaseFilterAngle, float a_InitialMipAngle, float a_MipAnglePerLevelScale,
                                          int a_FilterType, int a_FixupType, int a_FixupWidth, bool a_bUseSolidAngle
                                          , bool a_bUseMultithread, float a_SpecularPower, float a_CosinePowerDropPerMip, int a_NumMipmap, int a_CosinePowerMipmapChainMode
                                          ,	bool a_bExcludeBase, bool a_bIrradianceCubemap,	int a_LightingModel, float a_GlossScale, float a_GlossBias
                                          )
{
  m_SpecularPower				= a_SpecularPower; 
  m_CosinePowerDropPerMip		= a_CosinePowerDropPerMip;
  m_NumMipmap					= a_NumMipmap;
  m_CosinePowerMipmapChainMode	= a_CosinePowerMipmapChainMode;
  m_bExcludeBase				= a_bExcludeBase;
  m_bIrradianceCubemap			= a_bIrradianceCubemap;
  m_LightingModel				= a_LightingModel;
  m_FilterType				= a_FilterType;
  m_bUseSolidAngle		= a_bUseSolidAngle;
  m_FixupType					= a_FixupType;
  m_FixupWidth			 = a_FixupWidth;
  m_GlossScale					= a_GlossScale;
  m_GlossBias					= a_GlossBias;
  m_BaseFilterAngle = a_BaseFilterAngle;
  m_InitialMipAngle = a_InitialMipAngle;
  m_MipAnglePerLevelScale = a_MipAnglePerLevelScale;

  FilterCubeMapMipChain();
}


//--------------------------------------------------------------------------------------
//build filter lookup table
//
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::BuildAngleWeightLUT(int a_NumFilterLUTEntries, int a_FilterType, float a_FilterAngle)
{
  int iLUTEntry;

  CP_SAFE_DELETE_ARRAY( m_FilterLUT );

  m_NumFilterLUTEntries = 4096; //a_NumFilterLUTEntries;
  m_FilterLUT = new CP_ITYPE [m_NumFilterLUTEntries];

  // note that CP_FILTER_TYPE_DISC weights all taps equally and does not need a lookup table    
  if( a_FilterType == CP_FILTER_TYPE_CONE )
  {
    //CP_FILTER_TYPE_CONE is a cone centered around the center tap and falls off to zero 
    //  over the filtering radius
    CP_ITYPE filtAngleRad = a_FilterAngle * CP_PI / 180.0f;

    for(iLUTEntry=0; iLUTEntry<m_NumFilterLUTEntries; iLUTEntry++ )
    {
      CP_ITYPE angle = acos( (float)iLUTEntry / (float)(m_NumFilterLUTEntries - 1) );
      CP_ITYPE filterVal;

      filterVal = (filtAngleRad - angle) / filtAngleRad;

      if(filterVal < 0)
      {
        filterVal = 0;
      }

      //note that gaussian is not weighted by 1.0 / (sigma* sqrt(2 * PI)) seen as weights
      // weighted tap accumulation in filters is divided by sum of weights
      m_FilterLUT[iLUTEntry] = filterVal;
    }
  }
  else if( a_FilterType == CP_FILTER_TYPE_ANGULAR_GAUSSIAN )
  {
    //fit 3 standard deviations within angular extent of filter
    CP_ITYPE stdDev = (a_FilterAngle * CP_PI / 180.0) / 3.0;
    CP_ITYPE inv2Variance = 1.0 / (2.0 * stdDev * stdDev);

    for(iLUTEntry=0; iLUTEntry<m_NumFilterLUTEntries; iLUTEntry++ )
    {
      CP_ITYPE angle = acos( (float)iLUTEntry / (float)(m_NumFilterLUTEntries - 1) );
      CP_ITYPE filterVal;

      filterVal = exp( -(angle * angle) * inv2Variance );

      //note that gaussian is not weighted by 1.0 / (sigma* sqrt(2 * PI)) seen as weights
      // weighted tap accumulation in filters is divided by sum of weights
      m_FilterLUT[iLUTEntry] = filterVal;
    }
  }
}


//--------------------------------------------------------------------------------------
// WriteMipLevelIntoAlpha
//
//  Writes the current mip level into alpha in order for 2.0 shaders that need to 
//  know the current mip-level
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::WriteMipLevelIntoAlpha(void)
{
  int iFace, iMipLevel;

  //since output is being modified, terminate any active filtering threads
  TerminateActiveThreads();

  //generate subsequent mip levels
  for(iMipLevel = 0; iMipLevel < m_NumMipLevels; iMipLevel++)
  {
    //Iterate over faces for input images
    for(iFace = 0; iFace < 6; iFace++)
    {
      m_OutputSurface[iMipLevel][iFace].ClearChannelConst(3, (float) (16.0f * (iMipLevel / 255.0f)) );
    }
  }
}


//--------------------------------------------------------------------------------------
// Horizonally flip input cube map faces
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::FlipInputCubemapFaces(void)
{
  int iFace;

  //since input is being modified, terminate any active filtering threads
  TerminateActiveThreads();

  //Iterate over faces for input images
  for(iFace = 0; iFace < 6; iFace++)
  {
    m_InputSurface[iFace].InPlaceHorizonalFlip();
  }
}


//--------------------------------------------------------------------------------------
//Horizonally flip output cube map faces
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::FlipOutputCubemapFaces(void)
{
  int iFace, iMipLevel;

  //since output is being modified, terminate any active filtering threads
  TerminateActiveThreads();

  //Iterate over faces for input images
  for(iMipLevel = 0; iMipLevel < m_NumMipLevels; iMipLevel++)
  {
    for(iFace = 0; iFace < 6; iFace++)
    {
      m_OutputSurface[iMipLevel][iFace].InPlaceHorizonalFlip();
    }
  }
}

//--------------------------------------------------------------------------------------
//estimate fraction completed of filter thread based on current conditions
//
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::EstimateFilterThreadProgress(SFilterProgress *a_FilterProgress)
{
  float totalMipComputation = 0.0f;     //time to compute all mip levels as a function of the time it takes 
  //to compute the top mip level

  float progressMipComputation = 0.0f;	//progress based on entirely computed mip levels
  float currentMipComputation = 0.0f;	//amount of computation it takes to process this entire mip level
  float progressFaceComputation = 0.0f;	//progress based on entirely computed faces for this mip level
  float currentFaceComputation = 0.0f;	//amount of computation it takes to process this entire face
  float progressRowComputation = 0.0f;	//progress based on entirely computed rows for this face
  //estimated fraction of total computation time the current face will take

  int i;

  float filterAngle = 1.0f;					//filter angle for given miplevel
  int dstSize = 1;						//destination cube map size of given mip level
  int currentMipSize = 1;				//size of mip level currently being processed

  //compuate total compuation time as a function of the time  
  // cubemap processing for each miplevel is roughly O(n^2 * m^2) 
  //  where n is the cube map size, and m is the filter size   
  // Each miplevel is half the size of the previous level,  
  //  and the filter size in texels is roughly proportional to the 
  // (filter angle size * size of source cubemap texels are fetched from) ^2 

  // computation to generate base mip level (generated from input cube map)
  if(m_BaseFilterAngle > 0.0f)
  {
    totalMipComputation = pow(m_InputSize * m_BaseFilterAngle , 2.0f) * (m_OutputSize * m_OutputSize);
  }
  else
  {
    totalMipComputation = pow(m_InputSize * 0.01f , 2.0f) * (m_OutputSize * m_OutputSize);
  }

  progressMipComputation = 0.0f;
  if(a_FilterProgress->m_CurrentMipLevel > 0)
  {
    progressMipComputation = totalMipComputation;
  }

  //filtering angle for this miplevel
  filterAngle = m_InitialMipAngle;
  dstSize = m_OutputSize;

  //computation for entire base mip level (if current level is base level)
  if(a_FilterProgress->m_CurrentMipLevel == 0)
  {
    currentMipComputation = totalMipComputation;
    currentMipSize = dstSize;
  }

  //compuatation to generate subsequent mip levels
  for(i=1; i<m_NumMipLevels; i++)
  {
    float computation;

    dstSize /= 2;
    filterAngle *= m_MipAnglePerLevelScale;

    if(filterAngle > 180)
    {
      filterAngle = 180;
    }

    //note src size is dstSize*2 since miplevels are generated from the subsequent level
    computation = pow(dstSize * 2 * filterAngle, 2.0f) * (dstSize * dstSize);

    totalMipComputation += computation;

    //accumulate computation for completed mip levels
    if(a_FilterProgress->m_CurrentMipLevel > i)
    {
      progressMipComputation = totalMipComputation;
    }

    //computation for entire current mip level
    if(a_FilterProgress->m_CurrentMipLevel == i)
    {
      currentMipComputation = computation;
      currentMipSize = dstSize;
    }
  }

  //fraction of compuation time processing the entire current mip level will take
  currentMipComputation  /= totalMipComputation; 
  progressMipComputation /= totalMipComputation;

  progressFaceComputation = currentMipComputation * 
    (float)(a_FilterProgress->m_CurrentFace - a_FilterProgress->m_StartFace) /
    (float)(1 + a_FilterProgress->m_EndFace - a_FilterProgress->m_StartFace);

  currentFaceComputation = currentMipComputation * 
    1.0f /
    (1 + a_FilterProgress->m_EndFace - a_FilterProgress->m_StartFace);

  progressRowComputation = currentFaceComputation * 
    ((float)a_FilterProgress->m_CurrentRow / (float)currentMipSize);  

  //progress completed
  a_FilterProgress->m_FractionCompleted = 
    progressMipComputation +
    progressFaceComputation +
    progressRowComputation;


  if( a_FilterProgress->m_CurrentFace < 0)
  {
    a_FilterProgress->m_CurrentFace = 0;
  }

  if( a_FilterProgress->m_CurrentMipLevel < 0)
  {
    a_FilterProgress->m_CurrentMipLevel = 0;
  }

  if( a_FilterProgress->m_CurrentRow < 0)
  {
    a_FilterProgress->m_CurrentRow = 0;
  }

}


//--------------------------------------------------------------------------------------
// Return string describing the current status of the cubemap processing threads
//
//--------------------------------------------------------------------------------------
//WCHAR *CCubeMapProcessor::GetFilterProgressString(void)
//{
//   // SL BEGIN
//   static WCHAR threadProgressString[CP_MAX_PROGRESS_STRING];
//
//   m_ProgressString[0] = '\0';
//
//   for(int i=0; i<m_NumFilterThreads; i++)
//   {
//      if(IsFilterThreadActive(i))
//      {
//         //set wait
//         SetCursor(LoadCursor(NULL, IDC_WAIT ));
//
//         EstimateFilterThreadProgress(&sg_ThreadFilterFace[i].m_ThreadProgress);
//
//         _snwprintf_s(threadProgressString,
//            CP_MAX_PROGRESS_STRING,
//            CP_MAX_PROGRESS_STRING,
//            L"Thread %d: %5.2f%% Complete (Level %3d, Face %3d, Row %3d)\n", 
//			i,
//            100.0f * sg_ThreadFilterFace[i].m_ThreadProgress.m_FractionCompleted,
//            sg_ThreadFilterFace[i].m_ThreadProgress.m_CurrentMipLevel, 
//            sg_ThreadFilterFace[i].m_ThreadProgress.m_CurrentFace,
//            sg_ThreadFilterFace[i].m_ThreadProgress.m_CurrentRow        
//            );
//      }
//      else
//      {
//         //set arrow
//         SetCursor(LoadCursor(NULL, IDC_ARROW ));
//
//		 _snwprintf_s(threadProgressString, 
//            CP_MAX_PROGRESS_STRING,
//            CP_MAX_PROGRESS_STRING,
//            L"Thread %d: Ready\n", i);   
//      }
//
//	  wcscat_s(m_ProgressString, CP_MAX_PROGRESS_STRING, threadProgressString);
//   }
//   // SL END
//
//   return m_ProgressString;
//}


//--------------------------------------------------------------------------------------
//get status of cubemap processor
//
//--------------------------------------------------------------------------------------
int CCubeMapProcessor::GetStatus(void)
{
  return m_Status;
}


//--------------------------------------------------------------------------------------
//refresh status
// sets cubemap processor to ready state if not processing
//--------------------------------------------------------------------------------------
void CCubeMapProcessor::RefreshStatus(void)
{
  if(m_Status != CP_STATUS_PROCESSING )
  {
    m_Status = CP_STATUS_READY;

  }
}

// SH order use for approximation of irradiance cubemap is 5, mean 5*5 equals 25 coefficients
#define MAX_SH_ORDER 5
#define NUM_SH_COEFFICIENT (MAX_SH_ORDER * MAX_SH_ORDER)

// See Peter-Pike Sloan paper for these coefficients
static double SHBandFactor[NUM_SH_COEFFICIENT] = { 1.0,
  2.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0,
  1.0 / 4.0, 1.0 / 4.0, 1.0 / 4.0, 1.0 / 4.0, 1.0 / 4.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // The 4 band will be zeroed
  - 1.0 / 24.0, - 1.0 / 24.0, - 1.0 / 24.0, - 1.0 / 24.0, - 1.0 / 24.0, - 1.0 / 24.0, - 1.0 / 24.0, - 1.0 / 24.0, - 1.0 / 24.0};

void EvalSHBasis(const float* dir, double* res )
{
  // Can be optimize by precomputing constant.
  static const double SqrtPi = sqrt(CP_PI);

  double xx = dir[0];
  double yy = dir[1];
  double zz = dir[2];

  // x[i] == pow(x, i), etc.
  double x[MAX_SH_ORDER+1], y[MAX_SH_ORDER+1], z[MAX_SH_ORDER+1];
  x[0] = y[0] = z[0] = 1.;
  for (int i = 1; i < MAX_SH_ORDER+1; ++i)
  {
    x[i] = xx * x[i-1];
    y[i] = yy * y[i-1];
    z[i] = zz * z[i-1];
  }

  res[0]  = (1/(2.*SqrtPi));

  res[1]  = -(sqrt(3/CP_PI)*yy)/2.;
  res[2]  = (sqrt(3/CP_PI)*zz)/2.;
  res[3]  = -(sqrt(3/CP_PI)*xx)/2.;

  res[4]  = (sqrt(15/CP_PI)*xx*yy)/2.;
  res[5]  = -(sqrt(15/CP_PI)*yy*zz)/2.;
  res[6]  = (sqrt(5/CP_PI)*(-1 + 3*z[2]))/4.;
  res[7]  = -(sqrt(15/CP_PI)*xx*zz)/2.;
  res[8]  = sqrt(15/CP_PI)*(x[2] - y[2])/4.;

  res[9]  = (sqrt(35/(2.*CP_PI))*(-3*x[2]*yy + y[3]))/4.;
  res[10] = (sqrt(105/CP_PI)*xx*yy*zz)/2.;
  res[11] = -(sqrt(21/(2.*CP_PI))*yy*(-1 + 5*z[2]))/4.;
  res[12] = (sqrt(7/CP_PI)*zz*(-3 + 5*z[2]))/4.;
  res[13] = -(sqrt(21/(2.*CP_PI))*xx*(-1 + 5*z[2]))/4.;
  res[14] = (sqrt(105/CP_PI)*(x[2] - y[2])*zz)/4.;
  res[15] = -(sqrt(35/(2.*CP_PI))*(x[3] - 3*xx*y[2]))/4.;

  res[16] = (3*sqrt(35/CP_PI)*xx*yy*(x[2] - y[2]))/4.;
  res[17] = (-3*sqrt(35/(2.*CP_PI))*(3*x[2]*yy - y[3])*zz)/4.;
  res[18] = (3*sqrt(5/CP_PI)*xx*yy*(-1 + 7*z[2]))/4.;
  res[19] = (-3*sqrt(5/(2.*CP_PI))*yy*zz*(-3 + 7*z[2]))/4.;
  res[20] = (3*(3 - 30*z[2] + 35*z[4]))/(16.*SqrtPi);
  res[21] = (-3*sqrt(5/(2.*CP_PI))*xx*zz*(-3 + 7*z[2]))/4.;
  res[22] = (3*sqrt(5/CP_PI)*(x[2] - y[2])*(-1 + 7*z[2]))/8.;
  res[23] = (-3*sqrt(35/(2.*CP_PI))*(x[3] - 3*xx*y[2])*zz)/4.;
  res[24] = (3*sqrt(35/CP_PI)*(x[4] - 6*x[2]*y[2] + y[4]))/16.;
}

void CCubeMapProcessor::SHFilterCubeMap(bool a_bUseSolidAngleWeighting, int a_FixupType)
{
  CImageSurface* SrcCubeImage = m_InputSurface;
  CImageSurface* DstCubeImage = m_OutputSurface[0];

  int SrcSize = SrcCubeImage->m_Width;
  int DstSize = DstCubeImage->m_Width;

  //pointers used to walk across the image surface
  CP_ITYPE *normCubeRowStartPtr;
  CP_ITYPE *srcCubeRowStartPtr;
  CP_ITYPE *dstCubeRowStartPtr;
  CP_ITYPE *texelVect;

  const int SrcCubeMapNumChannels	= SrcCubeImage[0].m_NumChannels;
  const int DstCubeMapNumChannels	= DstCubeImage[0].m_NumChannels;

  //First step - Generate SH coefficient for the diffuse convolution

  const int NormCubeMapNumChannels = m_NormCubeMap[0].m_NumChannels; // This need to be init here after the generation of m_NormCubeMap

  //This is a custom implementation of D3DXSHProjectCubeMap to avoid to deal with LPDIRECT3DSURFACE9 pointer
  //Use Sh order 2 for a total of 9 coefficient as describe in http://www.cs.berkeley.edu/~ravir/papers/envmap/
  //accumulators are 64-bit floats in order to have the precision needed 
  //over a summation of a large number of pixels 
  double SHr[NUM_SH_COEFFICIENT];
  double SHg[NUM_SH_COEFFICIENT];
  double SHb[NUM_SH_COEFFICIENT];
  double SHdir[NUM_SH_COEFFICIENT];

  memset(SHr, 0, NUM_SH_COEFFICIENT * sizeof(double));
  memset(SHg, 0, NUM_SH_COEFFICIENT * sizeof(double));
  memset(SHb, 0, NUM_SH_COEFFICIENT * sizeof(double));
  memset(SHdir, 0, NUM_SH_COEFFICIENT * sizeof(double));

  double weightAccum = 0.0;
  double weight = 0.0;


  for (int iFaceIdx = 0; iFaceIdx < 6; iFaceIdx++)
  {
    for (int y = 0; y < SrcSize; y++)
    {
      normCubeRowStartPtr = &m_NormCubeMap[iFaceIdx].m_ImgData[NormCubeMapNumChannels * (y * SrcSize)];
      srcCubeRowStartPtr	= &SrcCubeImage[iFaceIdx].m_ImgData[SrcCubeMapNumChannels * (y * SrcSize)];

      for (int x = 0; x < SrcSize; x++)
      {
        //pointer to direction and solid angle in cube map associated with texel
        texelVect = &normCubeRowStartPtr[NormCubeMapNumChannels * x];

        if(a_bUseSolidAngleWeighting == true)
        {   //solid angle stored in 4th channel of normalizer/solid angle cube map
          weight = *(texelVect+3);
        }
        else
        {   //all taps equally weighted
          weight = 1.0;   
        }

        EvalSHBasis(texelVect, SHdir);

        // Convert to double
        double R = srcCubeRowStartPtr[(SrcCubeMapNumChannels * x) + 0];
        double G = srcCubeRowStartPtr[(SrcCubeMapNumChannels * x) + 1];
        double B = srcCubeRowStartPtr[(SrcCubeMapNumChannels * x) + 2];

        for (int i = 0; i < NUM_SH_COEFFICIENT; i++)
        {
          SHr[i] += R * SHdir[i] * weight;
          SHg[i] += G * SHdir[i] * weight;
          SHb[i] += B * SHdir[i] * weight;
        }

        weightAccum += weight;
      }
    }
  }

  //Normalization - The sum of solid angle should be equal to the solid angle of the sphere (4 PI), so
  // normalize in order our weightAccum exactly match 4 PI.
  for (int i = 0; i < NUM_SH_COEFFICIENT; ++i)
  {
    SHr[i] *= 4.0 * CP_PI / weightAccum;
    SHg[i] *= 4.0 * CP_PI / weightAccum;
    SHb[i] *= 4.0 * CP_PI / weightAccum;
  }

  //Second step - Generate cubemap from SH coefficient

  // regenerate normalization cubemap for the destination cubemap
  //clear pre-existing normalizer cube map
  for(int iCubeFace=0; iCubeFace<6; iCubeFace++)
  {
    m_NormCubeMap[iCubeFace].Clear();            
  }

  //Normalized vectors per cubeface and per-texel solid angle
  for (int i=0; i<6; i++) {
    m_Threads[i] = std::thread(&CCubeMapProcessor::BuildNormalizerSolidAngleCubemap,
      this,
      DstCubeImage->m_Width,
      m_NormCubeMap,
      i,
      a_FixupType);
  }
  for (int i=0; i<6; i++) {
    m_Threads[i].join();
  }

  for (int iFaceIdx = 0; iFaceIdx < 6; iFaceIdx++)
  {
    for (int y = 0; y < DstSize; y++)
    {
      normCubeRowStartPtr = &m_NormCubeMap[iFaceIdx].m_ImgData[NormCubeMapNumChannels * (y * DstSize)];
      dstCubeRowStartPtr	= &DstCubeImage[iFaceIdx].m_ImgData[DstCubeMapNumChannels * (y * DstSize)];

      for (int x = 0; x < DstSize; x++)
      {
        //pointer to direction and solid angle in cube map associated with texel
        texelVect = &normCubeRowStartPtr[NormCubeMapNumChannels * x];

        EvalSHBasis(texelVect, SHdir);

        // get color value
        CP_ITYPE R = 0.0f, G = 0.0f, B = 0.0f;

        for (int i = 0; i < NUM_SH_COEFFICIENT; ++i)
        {
          R += (CP_ITYPE)(SHr[i] * SHdir[i] * SHBandFactor[i]);
          G += (CP_ITYPE)(SHg[i] * SHdir[i] * SHBandFactor[i]);
          B += (CP_ITYPE)(SHb[i] * SHdir[i] * SHBandFactor[i]);
        }

        dstCubeRowStartPtr[(DstCubeMapNumChannels * x) + 0] = R;
        dstCubeRowStartPtr[(DstCubeMapNumChannels * x) + 1] = G;
        dstCubeRowStartPtr[(DstCubeMapNumChannels * x) + 2] = B;
        if (DstCubeMapNumChannels > 3)
        {
          dstCubeRowStartPtr[(DstCubeMapNumChannels * x) + 3] = 1.0f;
        }
      }
    }
  }
}

inline float GetSpecularPowerFactorToMatchPhong(float SpecularPower)
{
  // Scale highlight shape to better match lighting model as we can only filter cubemap with Phong filtering.
  // http://seblagarde.wordpress.com/2012/03/29/relationship-between-phong-and-blinn-lighting-model/

  /*
  // Approximate curve
  float Factor = 1.0f;
  if (SpecularPower != 0.0f)
  {
  float SP_2 = SpecularPower * SpecularPower;
  float SP_3 = SP_2 * SpecularPower;
  Factor		 = 4.00012f - (0.624042f / SP_3) + (0.728329f / SP_2) + (1.22792f / SpecularPower);
  }
  */

  // Simply use 4
  return 4.0f;
}
