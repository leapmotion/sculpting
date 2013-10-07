#ifndef __SCULPTOR_H__
#define __SCULPTOR_H__

#include "cinder/Cinder.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
using namespace cinder;
using namespace cinder::gl;

class DynamicMesh; // forward declation


struct SculptorBrush
{
	float _radius;
	float _radius_squared;
	float _transformed_radius;
	float _transformed_radius_squared;
	float _strength;
	float _weight;
	Vec3f _position;
	Vec3f _transformed_position;
	Vec3f _direction;
};

//**************************************************
class Sculptor
{
public:

	enum SCULPTMODE { GROW, SHRINK, SMOOTH, PINCH };

	Sculptor();
	virtual ~Sculptor();
	int getSculptMode() const { return _sculpt_mode; }
	void setSculptMode(const int _Mode) { _sculpt_mode = _Mode; }
	int getNumBrushes() const { return (int)_brushes.size(); }
	void addBrush(const Vec3f& _Pos,const Vec3f& _Dir, const float _Radius,const float _Strength,const float _Weight);
	void clearBrushes();
	float getBrushStrengthAtPosition(SculptorBrush* _Brush,const Vec3f& _Pos);
	void growBrush(SculptorBrush* _Brush,DynamicMesh* _Mesh);
	void shrinkBrush(SculptorBrush* _Brush,DynamicMesh* _Mesh);
	void smoothBrush(SculptorBrush* _Brush,DynamicMesh* _Mesh);
	void pinchBrush(SculptorBrush* _Brush,DynamicMesh* _Mesh);
	void applyBrushes(DynamicMesh* _Mesh,const Matrix44f& _Transform,const float _Scaling=1.f);
	void drawBrushes(GlslProg* _Shader);
	std::vector<Vec3f> brushPositions() const;
	std::vector<float> brushWeights() const;
	double getLastInteractionTime() const;

private:

	int _sculpt_mode;
	std::vector<SculptorBrush> _brushes;
	double _last_interaction_time;

};


#endif // #ifndef __SCULPTOR_H__


