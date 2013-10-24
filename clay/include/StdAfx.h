#pragma once

#include "cinder/app/AppNative.h"
#include "cinder/params/Params.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Fbo.h"
#include "cinder/Vector.h"

#include "Leap.h"
#include "cinder/Thread.h"


#define FREEIMAGE_LIB
#include "FreeImage.h"

#include "Common.h"

// Cinder -- Eigen conversions
inline cinder::Vec3f ToVec3f(const Vector3& v) { return cinder::Vec3f(v.x(), v.y(), v.z()); }
