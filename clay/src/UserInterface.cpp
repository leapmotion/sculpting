#include "StdAfx.h"
#include "UserInterface.h"
#include "cinder/app/App.h"
#include "Utilities.h"

using namespace ci;
using namespace ci::gl;

//const ColorA UserInterface::LEAF_COLOR(1.0f, 0.35f, 0.1f, 0.5f);
//const ColorA UserInterface::NORMAL_COLOR(0.2f, 0.3f, 0.5f, 0.4f);
//const ColorA UserInterface::LEAF_ACTIVATED_COLOR(0.1f, 1.0f, 0.35f, 1.0f);
//const ColorA UserInterface::NORMAL_ACTIVATED_COLOR(0.65f, 0.7f, 1.0, 1.0f);
const ColorA UserInterface::LEAF_COLOR(1.0f, 0.35f, 0.1f, 1.0f);
const ColorA UserInterface::NORMAL_COLOR(0.2f, 0.3f, 0.5f, 1.0f);
const ColorA UserInterface::LEAF_ACTIVATED_COLOR(0.1f, 1.0f, 0.35f, 1.0f);
const ColorA UserInterface::NORMAL_ACTIVATED_COLOR(0.65f, 0.7f, 1.0, 1.0f);
const float UserInterface::RADIUS_FALLOFF = 0.6f;

UserInterface::UserInterface()
  : _lastUpdateTime(0)
  , _windowSize(Vec2i::one())
  , _windowDiagonal(1.0f)
  , _shader(NULL)
  , _root_node(-1)
  , _focus_offset(Vec2f::zero())
  , _depth_offset(0.0f)
  , _max_level1_activation(0.0f)
  , _zoom_amount(1.0f)
  , _radius_mult(0.0f)
{ }

void UserInterface::addElement(const UIElement& _Element, const std::string& _ParentName)
{
  int cur = static_cast<int>(_elements.size());
  _name_map[_Element.getName()] = cur;
  _elements.push_back(_Element);
  if (!_ParentName.empty())
  {
    addConnection(_Element.getName(), _ParentName);
  }
}

void UserInterface::addConnection(const std::string& _First, const std::string& _Second)
{
  // should be called after the two nodes have already been added
  std::map<std::string, int>::iterator it1 = _name_map.find(_First);
  std::map<std::string, int>::iterator it2 = _name_map.find(_Second);
  if (it1 != _name_map.end() && it2 != _name_map.end())
  {
    int id1 = it1->second;
    int id2 = it2->second;
    _elements[id1].addConnection(id2);
    _elements[id2].addConnection(id1);
  }
}

void UserInterface::update(const std::vector<Vec4f>& _Tips)
{
  static const float ACTIVATION_BONUS_RADIUS = 0.4f;
  static const float ACTIVATION_RATE = 1.75f;

  double curTimeSeconds = app::getElapsedSeconds();
  float elapsed = static_cast<float>(curTimeSeconds - _lastUpdateTime);

  // update metaballs
  clearMetaballs();

  _zoom_amount = 1.0f/(pow(RADIUS_FALLOFF, _depth_offset));

  // perform depth first search
  for (size_t i=0; i<_elements.size(); i++)
  {
    _elements[i].setVisited(false);
    _elements[i].setMetaballIdx(-1);
  }
  std::vector<TraversalInfo> stack;
  std::vector<Force> forces;

  const float activationAmt = elapsed * ACTIVATION_RATE;

  // add root node at middle
  const Vec2f window_center = Vec2f(_windowSize.x/2.0f, _windowSize.y/2.0f);
  stack.push_back(TraversalInfo(_root_node, 0, static_cast<float>(3*M_PI/2), _focus_offset + window_center));
  float weighted_radius = _zoom_amount * radiusForDepth(0);
  Vec2f weighted_offset = Vec2f::zero();
  float weighted_depth = 0.0f;
  float offset_weight = 1.0f;
  float maxLevel1 = 0.0f;
  while (!stack.empty())
  {
    // retrieve info from stack
    const TraversalInfo& info = stack.back();
    const int cur_idx = info._idx;
    const int cur_depth = info._depth;
    const Vec2f cur_transform = info._transform;
    float cur_angle = info._angle;
    float cur_radius = _zoom_amount * _windowDiagonal * radiusForDepth(static_cast<float>(cur_depth));
    stack.pop_back();
    const std::vector<int>& connections = _elements[cur_idx].getConnections();
    const int num_connections = static_cast<int>(connections.size());

    // process this node
    float maxChildActivation = 0;
    for (size_t i=0; i<connections.size(); i++)
    {
      int connect = connections[i];
      if (!_elements[connect].isVisited())
      {
        maxChildActivation = std::max(maxChildActivation, _elements[connect].getActivation());
      }
    }

    float oldSmoothActivation = Utilities::SmootherStep(_elements[cur_idx].getActivation());
    if (cur_depth > 0)
    {
      cur_radius *= (1.0f + 0.25f*oldSmoothActivation);
    }
    float maxDelta = maxChildActivation > 0 ? activationAmt : -activationAmt;
    for (size_t j=0; j<_Tips.size(); j++)
    {
      Vec2f pos(_windowSize.x*_Tips[j].x, _windowSize.y*_Tips[j].y);
      float dist = cur_radius / pos.distance(cur_transform);
      dist = math<float>::clamp(dist);
      dist = 2.0f * (dist*dist - 0.5f);
      maxDelta = std::max(maxDelta, _Tips[j].w * (activationAmt * dist));
    }
    _elements[cur_idx].modifyActivation(_elements[cur_idx].getVisibility() * maxDelta);

    float smoothActivation = Utilities::SmootherStep(_elements[cur_idx].getActivation());
    if (cur_depth > 0)
    {
      ColorA idleColor = _elements[cur_idx].isLeafNode() ? LEAF_COLOR : NORMAL_COLOR;
      ColorA activeColor = _elements[cur_idx].isLeafNode() ? LEAF_ACTIVATED_COLOR : NORMAL_ACTIVATED_COLOR;
      ColorA curColor = (1.0f - smoothActivation)*idleColor + smoothActivation*activeColor;
      if (_elements[cur_idx].getVisibility() > 0.01f)
      {
        int idx = addMetaball(cur_transform, cur_radius, Vec4f(curColor.r, curColor.g, curColor.b, curColor.a), _elements[cur_idx].getVisibility(), _elements[cur_idx].getClicking());
        _elements[cur_idx].setMetaballIdx(idx);
      }
      float weight = cur_depth*smoothActivation;
      weighted_offset += weight*(window_center - cur_transform);
      weighted_radius += weight*cur_radius;
      offset_weight += weight;
      weighted_depth += weight*cur_depth;
      if (cur_depth == 1)
      {
        forces.push_back(Force(cur_transform, (_windowDiagonal/4.0f)*smoothActivation, cur_idx));
      }
    }
    else
    {
      smoothActivation = 1.0f;
    }
    _elements[cur_idx].setVisited(true);

    // process neighboring nodes that haven't already been visited
    if (num_connections > 0)
    {
      float angle_inc = static_cast<float>(1.4*M_PI/num_connections);
      float angle_offset = 0.0f;
      float horiz_mult = static_cast<float>(_windowSize.y);
      float vert_mult = static_cast<float>(_windowSize.y);
      if (cur_depth == 0)
      {
        // root node has children wrapping all the way around
        angle_inc = static_cast<float>(0.9f*M_PI/num_connections);
        angle_offset = angle_inc/2.0f;
        const float aspect = static_cast<float>(_windowSize.x)/static_cast<float>(_windowSize.y);
        horiz_mult *= aspect;
      }
      else if (cur_depth == 1)
      {
        cur_angle += static_cast<float>(M_PI);
      }
      const float start_angle = cur_angle - (num_connections/2.0f)*angle_inc + angle_offset;
      float cur_angle = start_angle;

      // add all neighbors to stack
      for (int i=0; i<num_connections; i++)
      {
        int connect = connections[i];
        if (!_elements[connect].isVisited())
        {
          _elements[connect].setVisibility(smoothActivation);
          Vec2f offset = 2.0f*radiusForDepth(static_cast<float>(cur_depth)) * smoothActivation * Vec2f(horiz_mult*std::cos(cur_angle), vert_mult*std::sin(cur_angle));
          if (cur_depth == 0)
          {
            _elements[connect].smoothForceOffset(getCombinedForcesAt(cur_transform + offset, connect));
            offset += _elements[connect].getForceOffset();
          }
          offset *= _zoom_amount;
          stack.push_back(TraversalInfo(connect, cur_depth+1, cur_angle, cur_transform + offset));
        }
        cur_angle += angle_inc;
      }
    }
  }

  _max_level1_activation = maxLevel1;

  static const float CHANGE_SPEED = 0.05f;
  weighted_depth /= offset_weight;
  _depth_offset = (1.0f-CHANGE_SPEED)*_depth_offset + CHANGE_SPEED*weighted_depth;
  weighted_offset /= offset_weight;
  _focus_offset = (1.0f-CHANGE_SPEED)*_focus_offset + CHANGE_SPEED*_zoom_amount*weighted_offset;

  // add tips as metaballs
  const float max_activation = maxActivation();
  for (size_t i=0; i<_Tips.size(); i++)
  {
    Vec2f pixelLocation(_windowSize.x*_Tips[i].x, _windowSize.y*_Tips[i].y);
    addMetaball(pixelLocation, _windowDiagonal*_Tips[i].z, Vec4f(LEAF_ACTIVATED_COLOR.r, LEAF_ACTIVATED_COLOR.g, LEAF_ACTIVATED_COLOR.b, max_activation), _Tips[i].w, 0.75f);
  }

  const float new_radius_mult = 1.0f + 0.7f*(1.0f-max_activation);
  static const float RADIUS_MULT_SMOOTH = 0.9f;
  _radius_mult = RADIUS_MULT_SMOOTH*_radius_mult + (1.0f-RADIUS_MULT_SMOOTH)*new_radius_mult;

  _forces = forces;
  _lastUpdateTime = curTimeSeconds;
}

void UserInterface::draw(Environment* _Env, const Matrix33f& normalMatrix) const
{
  if (!_shader)
  {
    return;
  }

  // set uniforms and draw metaballs
  int numElements = static_cast<int>(numMetaballs());
  _Env->bindCubeMap(Environment::CUBEMAP_IRRADIANCE, 1);
  _Env->bindCubeMap(Environment::CUBEMAP_RADIANCE, 0);
  _shader->bind();
  _shader->uniform("positions", _metaball_positions.data(), numElements);
  _shader->uniform("radii", _metaball_radii.data(), numElements);
  _shader->uniform("colors", _metaball_colors.data(), numElements);
  _shader->uniform("weights", _metaball_weights.data(), numElements);
  _shader->uniform("ambients", _metaball_ambients.data(), numElements);
  _shader->uniform("irradiance", 1);
  _shader->uniform("radiance", 0);
  _shader->uniform("normalTransform", normalMatrix);
  _shader->uniform("num", numElements);
  _shader->uniform("epsilon", 0.05f*_windowDiagonal*_zoom_amount);
  _shader->uniform("diffuseFactor", 1.0f);
  _shader->uniform("reflectionFactor", 0.1f);
  drawSolidRect(Rectf(0, 0, static_cast<float>(_windowSize.x), static_cast<float>(_windowSize.y)));
  _shader->unbind();
  _Env->unbindCubeMap(1);
  _Env->unbindCubeMap(0);

  // draw text
  static const float FONT_SIZE = 24.0f;
  Font font("Arial", FONT_SIZE);
  for (size_t i=0; i<_elements.size(); i++)
  {
    int idx = _elements[i].getMetaballIdx();
    if (idx >= 0)
    {
      float activation = Utilities::SmootherStep(_elements[i].getActivation());
      Vec2f pos = _metaball_positions[idx];
      pos.y = _windowSize.y - pos.y;
      ColorA textColor(ColorA::white());
      textColor.a = _elements[i].getVisibility();
      drawStringCentered(_elements[i].getName(), pos - Vec2f(0, FONT_SIZE/2.0f), textColor, font);
    }
  }
}

void UserInterface::setWindowSize(const Vec2i& _Size)
{
  _windowSize = _Size;
  const float width = static_cast<float>(_windowSize.x);
  const float height = static_cast<float>(_windowSize.y);
  _windowDiagonal = std::sqrt(width*width + height*height)/2.0f;
}

void UserInterface::setShader(GlslProg* _Shader)
{
  _shader = _Shader;
}

void UserInterface::setRootNode(const std::string& _Name)
{
  std::map<std::string, int>::iterator it = _name_map.find(_Name);
  if (it != _name_map.end())
  {
    _root_node = it->second;
  }
}

float UserInterface::maxActivation() const
{
  float result = -1;
  for (size_t i=0; i<_elements.size(); i++)
  {
    result = std::max(_elements[i].getActivation(), result);
  }
  return result;
}

Vec2f UserInterface::getCombinedForcesAt(const Vec2f& _Position, int _Id) const
{
  Vec2f result(Vec2f::zero());
  for (size_t i=0; i<_forces.size(); i++)
  {
    Vec2f diff = _Position - _forces[i]._position;
    float normSq = diff.lengthSquared();
    if (normSq > 0.0001f && _forces[i]._id != _Id)
    {
      float norm = std::sqrt(normSq);
      result += _forces[i]._strength * (diff/norm);
    }
  }
  return result;
}

void UserInterface::clearMetaballs()
{
  _metaball_positions.clear();
  _metaball_radii.clear();
  _metaball_colors.clear();
  _metaball_weights.clear();
  _metaball_ambients.clear();
}

int UserInterface::addMetaball(const Vec2f& _Position, float _Radius, const Vec4f& _Color, float _Weight, float _Ambient)
{
  int idx = static_cast<int>(_metaball_positions.size());
  _metaball_positions.push_back(_Position);
  _metaball_radii.push_back(_Radius);
  _metaball_colors.push_back(_Color);
  _metaball_weights.push_back(_Weight);
  _metaball_ambients.push_back(_Ambient);
  return idx;
}

size_t UserInterface::numMetaballs() const
{
  return _metaball_positions.size();
}

float UserInterface::radiusForDepth(float depth) const
{
  const float MAX_RADIUS = 0.16f;
  float radius = MAX_RADIUS * std::powf(RADIUS_FALLOFF, depth);
  if (depth == 0)
  {
    return radius * 1.1f * _radius_mult;
  }
  return radius;
}
