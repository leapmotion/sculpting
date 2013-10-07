#ifndef __USERINTERFACE_H__
#define __USERINTERFACE_H__

#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/params/Params.h"
#include "cinder/Thread.h"
#include "Environment.h"
#include <boost/function.hpp>
#include <vector>
using namespace ci;
using namespace ci::gl;

class UIElement
{

public:

	UIElement(
		const std::string& _Name = "",
		const boost::function<void(const std::string&)>& _Callback = boost::function<void(const std::string&)>())
		: _name(_Name)
		, _callback(_Callback)
		, _activation(0.0f)
		, _clicking(0.0f)
		, _visibility(0.0f)
		, _metaball_idx(-1)
		, _visited(false)
		, _cooldown(false)
		, _force_offset(Vec2f::zero())
	{ }

	inline void modifyActivation(float delta) {
		if (_activation == 1.0f && _callback)
		{
			if (delta > 0 && !_cooldown)
			{
				_clicking = math<float>::clamp(_clicking + 0.5f*delta);
				if (_clicking == 1.0f)
				{
					_callback(_name);
					_cooldown = true;
				}
			}
		}
		if (_cooldown || delta < 0)
		{
			_clicking = math<float>::clamp(_clicking - 2.0f*fabs(delta));
		}
		if (_cooldown && _activation == 0.0f)
		{
			_cooldown = false;
		}
		else
		{
			_activation = math<float>::clamp(_activation + delta);
		}
	}

	inline void setMetaballIdx(int idx) { _metaball_idx = idx; }
	inline float getActivation() const { return _activation; }
	inline float getClicking() const { return _clicking; }
	inline void setVisibility(float _Visibility) { _visibility = _Visibility; }
	inline float getVisibility() const { return _visibility; }
	inline const std::string& getName() const { return _name; }
	inline int getMetaballIdx() const { return _metaball_idx; }
	inline void addConnection(int idx) { _connections.push_back(idx); }
	inline const std::vector<int>& getConnections() const { return _connections; }
	inline bool isLeafNode() const { return _connections.size() == 1; }
	inline void setVisited(bool visited) { _visited = visited; }
	inline bool isVisited() { return _visited; }
	inline void smoothForceOffset(const Vec2f& _Force)
	{
		static const float FORCE_SMOOTH = 0.9f;
		_force_offset = FORCE_SMOOTH*_force_offset + (1.0f - FORCE_SMOOTH)*_Force;
	}
	inline const Vec2f& getForceOffset() const
	{
		return _force_offset;
	}

private:

	float _activation;
	float _visibility;
	float _clicking;
	std::string _name;
	boost::function<void(const std::string&)> _callback;
	int _metaball_idx;
	std::vector<int> _connections;
	bool _visited;
	bool _cooldown;
	Vec2f _force_offset;

};

class UserInterface
{

public:

	UserInterface();
	void addElement(const UIElement& _Element, const std::string& _ParentName = "");
	void addConnection(const std::string& _First, const std::string& _Second);
	void update(const std::vector<Vec4f>& _Tips);
	void draw(Environment* _Env, const Matrix33f& normalMatrix) const;
	void setWindowSize(const Vec2i& _Size);
	void setShader(GlslProg* _Shader);
	void setRootNode(const std::string& _Name);
	float maxActivation() const;

private:

	Vec2f normalizePosition(const Vec2i& _Position) const;
	Vec2f getCombinedForcesAt(const Vec2f& _Position, int _Id) const;
	void clearMetaballs();
	int addMetaball(const Vec2f& _Position, float _Radius, const Vec4f& _Color, float _Weight, float _Ambient);
	size_t numMetaballs() const;
	float radiusForDepth(float depth) const;

	struct TraversalInfo
	{
		TraversalInfo(int _Idx, int _Depth, float _Angle, const Vec2f& _Transform)
			: _idx(_Idx)
			, _depth(_Depth)
			, _angle(_Angle)
			, _transform(_Transform)
		{ }

		int _idx;
		int _depth;
		float _angle;
		Vec2f _transform;
	};

	struct Force
	{
		Force(const Vec2f& _Position, float _Strength, int _Id)
			: _position(_Position)
			, _strength(_Strength)
			, _id(_Id)
		{ }

		Vec2f _position;
		float _strength;
		int _id;
	};

	std::vector<UIElement> _elements;
	std::vector<Force> _forces;
	std::vector<Vec2f> _metaball_positions;
	std::vector<float> _metaball_radii;
	std::vector<Vec4f> _metaball_colors;
	std::vector<float> _metaball_weights;
	std::vector<float> _metaball_ambients;
	double _lastUpdateTime;
	Vec2i _windowSize;
	float _windowDiagonal;
	GlslProg* _shader;
	std::map<std::string, int> _name_map;
	int _root_node;
	std::vector< std::vector<int> > _indices_by_depth;
	Vec2f _focus_offset;
	float _depth_offset;
	float _max_level1_activation;
	float _zoom_amount;
	float _radius_mult;

	static const ColorA LEAF_COLOR;
	static const ColorA NORMAL_COLOR;
	static const ColorA LEAF_ACTIVATED_COLOR;
	static const ColorA NORMAL_ACTIVATED_COLOR;
	static const float RADIUS_FALLOFF;

};

#endif
