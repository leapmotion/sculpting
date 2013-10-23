#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include "DataTypes.h"
#include <cinder/gl/gl.h>
#include <cinder/Vector.h>
#include <cinder/Rand.h>
#ifdef __APPLE__
#include <OpenGL/GLU.h>
#else
#include <gl/GLU.h>
#endif

namespace Utilities {

static inline float SmootherStep(float x)
{
	// x is blending parameter between 0 and 1
	return x*x*x*(x*(x*6 - 15) + 10);
}

static void getRotation(const ci::Vec3f& _From, const ci::Vec3f& _To, ci::Vec3f& _Axis, float& _Angle)
{
	_Axis = _From.cross(_To);
	_Angle = std::acos(_From.dot(_To));
}

static GLUquadric* quadric()
{
	static GLUquadric* gluQuadric = gluNewQuadric();
	return gluQuadric;
}

static void drawDisk(const float _Inner, const float _Outer, const ci::Vec3f& _Center, const ci::Vec3f& _Normal)
{
	float angle;
	ci::Vec3f axis;
	getRotation(ci::Vec3f::zAxis(), _Normal, axis, angle);
	glPushMatrix();
	ci::gl::translate(_Center);
	glRotated(ci::toDegrees(angle), axis.x, axis.y, axis.z);
	gluDisk(quadric(), _Inner, _Outer, 50, 1);
	glPopMatrix();
}

static void drawCapsule(const float _Radius, const float _Length, const ci::Vec3f& _Point, const ci::Vec3f& _Direction)
{
	float angle;
	ci::Vec3f axis;
	getRotation(ci::Vec3f::zAxis(), _Direction, axis, angle);
	glPushMatrix();
	ci::gl::translate(_Point);
	glRotated(ci::toDegrees(angle), axis.x, axis.y, axis.z);
	gluCylinder(quadric(), _Radius, _Radius, _Length, 30, 1);
	ci::gl::drawSphere(ci::Vec3f::zero(), _Radius, 30);
	ci::gl::drawSphere(ci::Vec3f(0, 0, _Length), _Radius, 30);
	glPopMatrix();
}

static const Vector3& colorForIndex(int idx) {
  static const int NUM_COLORS = 256;
  static Vector3 colors[NUM_COLORS];
  static bool loaded = false;
  idx = idx % NUM_COLORS;
  if (!loaded) {
    const float inc = 0.2f;
    float curAdd = 0;
    float curVal;
    for (int i=0; i<NUM_COLORS; i++) {
      curVal = inc*ci::randFloat() + curAdd;
      if (curVal > 1.0f) {
        curVal -= 1.0f;
      }
      ci::Vec3f hsv(curVal, 0.75f, 0.75f);
      ci::Colorf rgb = ci::hsvToRGB(hsv);
      colors[i] << rgb.r, rgb.g, rgb.b;
      curAdd += inc;
      if (curAdd > 1.0f) {
        curAdd -= 1.0f;
      }
    }
    loaded = true;
  }
  return colors[idx];
}

static inline float falloff(float dist) {
  return 3.f*dist*dist*dist*dist-4.f*dist*dist*dist+1.f;
}

struct FPSCounter {
  FPSCounter() : lastTime(0.0), lastReportTime(0.0) {
    reportedDeltaTime = deltaTime = 1.0f / 60.0f;
  }
  inline float FPS() { return 1.0f / reportedDeltaTime; }
  inline void Update(double time) {
    static const float SMOOTH_STRENGTH = 0.8f;
    static const double TIME_BETWEEN_UPDATES = 1.0;
    const float sinceLast = static_cast<float>(time - lastTime);
    deltaTime = SMOOTH_STRENGTH*deltaTime + (1.0f-SMOOTH_STRENGTH)*sinceLast;
    lastTime = time;
    if (time - lastReportTime > TIME_BETWEEN_UPDATES) {
      reportedDeltaTime = deltaTime;
      lastReportTime = time;
    }
  }
  float deltaTime;
  float reportedDeltaTime;
  double lastTime;
  double lastReportTime;
};

static const int TIME_STAMP_TICKS_PER_SEC = 1000000;
static const double TIME_STAMP_SECS_TO_TICKS  = static_cast<double>(TIME_STAMP_TICKS_PER_SEC);
static const double TIME_STAMP_TICKS_TO_SECS  = 1.0/TIME_STAMP_SECS_TO_TICKS;

}

#endif
