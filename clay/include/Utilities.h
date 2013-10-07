#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <cinder/gl/gl.h>
#include <cinder/Vector.h>
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

static void getRotation(const Vec3f& _From, const Vec3f& _To, Vec3f& _Axis, float& _Angle)
{
	_Axis = _From.cross(_To);
	_Angle = std::acos(_From.dot(_To));
}

static GLUquadric* quadric()
{
	static GLUquadric* gluQuadric = gluNewQuadric();
	return gluQuadric;
}

static void drawDisk(const float _Inner, const float _Outer, const Vec3f& _Center, const Vec3f& _Normal)
{
	float angle;
	Vec3f axis;
	getRotation(Vec3f::zAxis(), _Normal, axis, angle);
	glPushMatrix();
	gl::translate(_Center);
	glRotated(toDegrees(angle), axis.x, axis.y, axis.z);
	gluDisk(quadric(), _Inner, _Outer, 50, 1);
	glPopMatrix();
}

static void drawCapsule(const float _Radius, const float _Length, const Vec3f& _Point, const Vec3f& _Direction)
{
	float angle;
	Vec3f axis;
	getRotation(Vec3f::zAxis(), _Direction, axis, angle);
	glPushMatrix();
	gl::translate(_Point);
	glRotated(toDegrees(angle), axis.x, axis.y, axis.z);
	gluCylinder(quadric(), _Radius, _Radius, _Length, 30, 1);
	gl::drawSphere(Vec3f::zero(), _Radius, 30);
	gl::drawSphere(Vec3f(0, 0, _Length), _Radius, 30);
	glPopMatrix();
}

}

#endif
