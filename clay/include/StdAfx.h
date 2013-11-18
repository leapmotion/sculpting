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

#if _WIN32
#include <Windows.h>
#include <Shellapi.h>
#include <direct.h>
#include <WinTrust.h>
#include <SoftPub.h>
#include <CommDlg.h>
#include <UserEnv.h>
#endif

#include <boost/filesystem.hpp>

#define FREEIMAGE_LIB
#include "FreeImage.h"

#include "Common.h"

// Cinder -- Eigen conversions
inline cinder::Vec3f ToVec3f(const Vector3& v) { return cinder::Vec3f(v.x(), v.y(), v.z()); }
