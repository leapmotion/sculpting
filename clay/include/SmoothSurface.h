#ifndef __SMOOTHSURFACE_H__
#define __SMOOTHSURFACE_H__

#define USE_SMOOTH_SURFACE




#ifdef USE_SMOOTH_SURFACE

//#include "GL/glew.h"
//#include "cinder/GL/glee.h"

#include "DynamicMesh.h"
#include <vector>


class SmoothSurface {

public:

  SmoothSurface();
  void createFromDynamicMesh(DynamicMesh& mesh);
  void updateGeom();
  void displaySmoothSurface();

private:

  std::vector<float> g_orgPositions;
  std::vector<float> g_normals;
  float g_size;
  int g_width;
  int g_height;
  int g_frame;
  int g_level;
  ci::Vec3f g_center;
	unsigned int g_vao;

};

#endif // #ifdef USE_SMOOTH_SURFACE

#endif // #ifndef __SMOOTHSURFACE_H__
