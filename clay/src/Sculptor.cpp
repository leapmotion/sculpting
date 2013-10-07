#include "Sculptor.h"
#include "cinder/app/App.h"
#include "DynamicMesh.h"
#include "Utilities.h"


const float GROW_STRENGTH = 0.1f;
const float SHRINK_STRENGTH = 0.1f;
const float SMOOTH_STRENGTH = 2.5f;
const float PINCH_STRENGTH = 0.1f;



#define SMOOTHSTEP(x)       ((x)*(x)*(3.f-2.f*(x)))
#define SMOOTHSTEP2(x)      ((x)*(x)*(x)*((x)*((x)*6.f-15.f)+10.f))

Sculptor::Sculptor()
	: _sculpt_mode(GROW)
	, _last_interaction_time(0.0)
{
}

Sculptor::~Sculptor()
{
}

void Sculptor::addBrush(const Vec3f& _Pos,const Vec3f& _Dir, const float _Radius,const float _Strength,const float _Weight)
{
	_brushes.push_back(SculptorBrush());
	_brushes.back()._radius = _Radius;
	_brushes.back()._radius_squared = _brushes.back()._radius*_brushes.back()._radius;
	_brushes.back()._strength = _Weight*_Strength;
	_brushes.back()._weight = _Weight;
	switch( _sculpt_mode )
	{
		case GROW: _brushes.back()._strength *= GROW_STRENGTH; break;
		case SHRINK: _brushes.back()._strength *= SHRINK_STRENGTH; break;
		case SMOOTH: _brushes.back()._strength *= SMOOTH_STRENGTH; break;
		case PINCH: _brushes.back()._strength *= PINCH_STRENGTH; break;
	}
	_brushes.back()._position = _Pos;
	_brushes.back()._direction = _Dir;
}

void Sculptor::clearBrushes()
{
    _brushes.clear();
}

float Sculptor::getBrushStrengthAtPosition(SculptorBrush* _Brush,const Vec3f& _Pos)
{
	const float l = _Brush->_transformed_position.distanceSquared(_Pos);
	if( l>_Brush->_transformed_radius_squared ) return 0.f; // point outside sphere of influence

	float f = _Brush->_transformed_radius-sqrtf(l);
	if( f<0.f ) return 0.f;
	f = f/_Brush->_transformed_radius; // normalise field
	f = SMOOTHSTEP2(f);
	return f*_Brush->_strength;
}

void Sculptor::growBrush(SculptorBrush* _Brush,DynamicMesh* _Mesh)
{
	const double cur_time = app::getElapsedSeconds();
	DynamicMesh::VertexList& vertices = _Mesh->getVertices();
	for(DynamicMesh::VertexIterator i=vertices.begin(); i!=vertices.end(); ++i)
	{
		const float field = getBrushStrengthAtPosition(_Brush,(*i)->getPosition());
		if( field>0.f )
		{
			(*i)->modifyPosition((*i)->getNormal()*field);
			_Mesh->setDirty(true);
			_last_interaction_time = cur_time;
		}
	}
}

void Sculptor::shrinkBrush(SculptorBrush* _Brush,DynamicMesh* _Mesh)
{
	const double cur_time = app::getElapsedSeconds();
	DynamicMesh::VertexList& vertices = _Mesh->getVertices();
	for(DynamicMesh::VertexIterator i=vertices.begin(); i!=vertices.end(); ++i)
	{
		const float field = getBrushStrengthAtPosition(_Brush,(*i)->getPosition());
		if( field>0.f )
		{
			(*i)->modifyPosition((*i)->getNormal()*(-field));
			_Mesh->setDirty(true);
			_last_interaction_time = cur_time;
		}
	}
}

void Sculptor::smoothBrush(SculptorBrush* _Brush,DynamicMesh* _Mesh)
{
	const double cur_time = app::getElapsedSeconds();
	DynamicMesh::VertexList& vertices = _Mesh->getVertices();
	for(DynamicMesh::VertexIterator i=vertices.begin(); i!=vertices.end(); ++i)
	{
		const float field = getBrushStrengthAtPosition(_Brush,(*i)->getPosition());
		if( field>0.f )
		{
			(*i)->smooth(field);
			_Mesh->setDirty(true);
			_last_interaction_time = cur_time;
		}
	}
}

void Sculptor::pinchBrush(SculptorBrush* _Brush,DynamicMesh* _Mesh)
{
/*
	DynamicMesh::VertexList& vertices = _Mesh->getVertices();
	for(DynamicMesh::VertexIterator i=vertices.begin(); i!=vertices.end(); ++i)
	{
		const float field = getBrushStrengthAtPosition(_Brush,(*i)->getPosition());
		if( field>0.f )
		{
			(*i)->modifyPosition((*i)->getNormal()*field);
			_Mesh->setDirty(true);
		}
	}
*/
}

void Sculptor::applyBrushes(DynamicMesh* _Mesh,const Matrix44f& _Transform,const float _Scaling)
{
	// *** first calculate brush positions transformed into object space ***
	for(size_t b=0; b<_brushes.size(); ++b)
	{
		_brushes[b]._transformed_position = _Transform.transformPoint(_brushes[b]._position /* /_Scaling */ );
		_brushes[b]._transformed_radius = _brushes[b]._radius * _Scaling;
		_brushes[b]._transformed_radius_squared = _brushes[b]._transformed_radius*_brushes[b]._transformed_radius;
	}

	// *** now apply brushes dependent on sculpt mode ***
	switch( _sculpt_mode )
	{
		case GROW: 
		{
			for(size_t b=0; b<_brushes.size(); ++b) 
			{
				growBrush(&_brushes[b],_Mesh); 
			}
			break;
		}
		case SHRINK: 
		{
			for(size_t b=0; b<_brushes.size(); ++b) 
			{
				shrinkBrush(&_brushes[b],_Mesh); 
			}
			break;
		}
		case SMOOTH: 
		{
			for(size_t b=0; b<_brushes.size(); ++b) 
			{
				smoothBrush(&_brushes[b],_Mesh); 
			}
			break;
		}
		case PINCH: 
		{
			for(size_t b=0; b<_brushes.size(); ++b)
			{
				pinchBrush(&_brushes[b],_Mesh); 
			}
			break;
		}
	}
//	_Mesh->calculateAllNormals();
}

void Sculptor::drawBrushes(GlslProg* _Shader)
{
	for(size_t b=0; b<_brushes.size(); ++b)
	{
		_Shader->uniform("alphaMult", _brushes[b]._weight);
		gl::drawSphere(_brushes[b]._position, _brushes[b]._radius/2.0f, 30);
	}
}

std::vector<Vec3f> Sculptor::brushPositions() const
{
	std::vector<Vec3f> positions;
	for (size_t i=0; i<_brushes.size(); i++)
	{
		positions.push_back(_brushes[i]._position);
	}
	return positions;
}

std::vector<float> Sculptor::brushWeights() const
{
	std::vector<float> weights;
	for (size_t i=0; i<_brushes.size(); i++)
	{
		weights.push_back(_brushes[i]._weight);
	}
	return weights;
}

double Sculptor::getLastInteractionTime() const
{
	return _last_interaction_time;
}
