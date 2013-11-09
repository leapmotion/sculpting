#include "StdAfx.h"
#include "UserInterface.h"
#include "cinder/app/App.h"
#include "Utilities.h"
#include "Sculpt.h"
#include "LeapInteraction.h"

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

const float Menu::FONT_SIZE = 36.0f;
const float Menu::RING_THICKNESS_RATIO = 0.3f;
ci::Font Menu::m_font;
ci::Font Menu::m_boldFont;
std::vector<ci::svg::DocRef> Menu::m_icons;

Menu::Menu() : m_outerRadius(0.4f), m_innerRadius(0.1f), m_sweepAngle(2.75f), m_windowSize(Vector2::Ones()),
  m_curSelectedEntry(-1), m_deselectTime(0.0), m_selectionTime(0.0), m_prevSelectedEntry(-1) {
  m_activation.value = 0.0f;
}

void Menu::update(const std::vector<Vec4f>& tips, Sculpt* sculpt) {
  static const float ACTIVATION_SMOOTH_STRENGTH = 0.925f;
  static const float SELECTION_COOLDOWN = 1.0f;
  static const float MIN_TIME_SINCE_SCULPTING = 0.5f; // prevent interacting with menu if we're sculpting
  const double lastSculptTime = sculpt->getLastSculptTime();
  const double curTime = ci::app::getElapsedSeconds();
  const int numTips = tips.size();
  const int numEntries = m_entries.size();
  const float timeSinceSculpting = static_cast<float>(curTime - lastSculptTime);

  if (timeSinceSculpting > MIN_TIME_SINCE_SCULPTING) {
    float angle, radius;
    for (int i=0; i<numTips; i++) {
      const Vector2 pos((tips[i].x - 0.5f)*m_windowAspect + 0.5f, 1.0f - tips[i].y);
      const float z = tips[i].z;
      const float strength = tips[i].w;
      toRadialCoordinates(pos, radius, angle);

      // check for hovering in wedges
      const int hit = checkCollision(pos);
      if (hit >= 0) {
        m_entries[hit].m_hoverStrength.Update(1.0f, curTime, ACTIVATION_SMOOTH_STRENGTH);
        m_entries[hit].m_activationStrength.Update(radius/m_outerRadius, curTime, ACTIVATION_SMOOTH_STRENGTH);
      }

      // check for center pad
      const float curRadius = m_activation.value*(m_outerRadius - m_innerRadius) + m_innerRadius;
      if (radius < curRadius) {
        // colliding with pad
        m_activation.Update(1.0f, curTime, ACTIVATION_SMOOTH_STRENGTH);
      }
    }
  }
  float maxActivation = 0.0f;
  int maxIdx;
  for (int i=0; i<numEntries; i++) {
    const float curActivation = m_entries[i].m_activationStrength.value;
    if (curActivation > maxActivation) {
      maxActivation = curActivation;
      maxIdx = i;
    }
  }
  if (maxActivation > 0.2f) {
    m_activeName = m_entries[maxIdx].toString();
  } else {
    m_activeName = m_actualName;
  }

  if (m_activation.lastTimeSeconds < curTime) {
    // we weren't updated this frame
    m_activation.Update(0.0f, curTime, ACTIVATION_SMOOTH_STRENGTH);

    // finger left the area, so check whether to select the most activated entry
    if (maxActivation > 0.2f) {
      if ((maxIdx != m_prevSelectedEntry && m_curSelectedEntry != maxIdx) || (m_curSelectedEntry == -1 && static_cast<float>(curTime - m_deselectTime) > SELECTION_COOLDOWN)) {
        m_curSelectedEntry = maxIdx;
        m_selectionTime = curTime;
        m_actualName = m_activeName = m_entries[m_curSelectedEntry].toString();
      }
    }
  }

  if (m_curSelectedEntry != -1) {
    m_entries[m_curSelectedEntry].m_hoverStrength.Update(1.0f, curTime, ACTIVATION_SMOOTH_STRENGTH);
    m_entries[m_curSelectedEntry].m_activationStrength.Update(1.0f, curTime, ACTIVATION_SMOOTH_STRENGTH);
    if (static_cast<float>(curTime - m_selectionTime) > SELECTION_COOLDOWN) {
      m_prevSelectedEntry = m_curSelectedEntry;
      m_curSelectedEntry = -1;
      m_deselectTime = curTime;
    }
  }

  for (int i=0; i<numEntries; i++) {
    if (m_entries[i].m_hoverStrength.lastTimeSeconds < curTime) {
      m_entries[i].m_hoverStrength.Update(0.0f, curTime, ACTIVATION_SMOOTH_STRENGTH);
      m_entries[i].m_activationStrength.Update(0.0f, curTime, ACTIVATION_SMOOTH_STRENGTH);
    }
  }

  // update the positions of menu entries
  const float activation = m_activation.value;
  const float angleStart = getSweepStart();
  const float angleInc = m_sweepAngle / numEntries;
  const float center = activation*getRingRadius();
  const float thickness = RING_THICKNESS_RATIO*activation*m_outerRadius;
  const float wedgeStart = center - thickness/2.0f;

  float curAngle = angleStart + angleInc/2.0f;
  for (int i=0; i<numEntries; i++) {
    const Vector2 dir(std::sin(curAngle), std::cos(curAngle));
    const float entryActivation = m_entries[i].m_activationStrength.value;
    const float entryRadius = std::max(m_activation.value, entryActivation) * getRingRadius();
    const float highlightRadius = activation * m_outerRadius * entryActivation;
    //const float fillRadius = m_outerRadius * entryActivation;
    const float fillRadius = getRingRadius() * entryActivation;
    const float bonus = std::max(0.0f, highlightRadius - wedgeStart);
    //const float bonus = std::max(0.0f, fillRadius - RING_THICKNESS_RATIO*m_outerRadius);
    m_entries[i].m_position = m_position + (bonus+entryRadius) * dir;
    m_entries[i].m_angleStart = curAngle - angleInc/2.0f;
    m_entries[i].m_angleWidth = angleInc;
    curAngle += angleInc;
  }
}

void Menu::setNumEntries(int num) {
  m_entries.resize(num);
}

void Menu::draw() const {
  glPushMatrix();
  const float activation = m_activation.value;
  const float menuOpacity = getOpacity(activation);
  ci::Vec2f pos = relativeToAbsolute(m_position);
#if 0
  if (activation > 0.01f) {
    ci::ColorA circleColor(0.05f, 0.05f, 0.05f, activation);
    gl::color(circleColor);
    gl::drawSolidCircle(pos, relativeToAbsolute(m_innerRadius), 50);
  }
#endif
  //const float inner = activation * m_innerRadius;
  //const float outer = activation * m_outerRadius;
  //gl::color(wedgeColor);
  //Utilities::drawPartialDisk(pos, wedgeStart, wedgeEnd, Utilities::RADIANS_TO_DEGREES*getSweepStart(), Utilities::RADIANS_TO_DEGREES*m_sweepAngle);
  //if (activation > 0.01f) {
    for (size_t i=0; i<m_entries.size(); i++) {
      const float entryActivation = m_entries[i].m_activationStrength.value;
      const float maxActivation = std::max(activation, entryActivation);
      const float center = maxActivation*getRingRadius();
      const float thickness = RING_THICKNESS_RATIO*maxActivation*m_outerRadius;
      const float wedgeStart = relativeToAbsolute(center - thickness/2.0f);
      const float wedgeEnd = relativeToAbsolute(center + thickness/2.0f);
      const float highlightRadius = relativeToAbsolute(activation * m_outerRadius * entryActivation);
      const float entryOpacity = getOpacity(entryActivation);
      //const float highlightRadius = relativeToAbsolute(wedgeStart + thickness * entryActivation);

      // draw wedge behind
      const float brightness = 0.2f + 0.15f * m_entries[i].m_hoverStrength.value;
      const float green = 0.65f * entryActivation;
      glColor4f(brightness, brightness + green, brightness, entryOpacity);
      const float bonus = std::max(0.0f, highlightRadius - wedgeStart);
      Utilities::drawPartialDisk(pos, wedgeStart + bonus, wedgeEnd + bonus, Utilities::RADIANS_TO_DEGREES*m_entries[i].m_angleStart, Utilities::RADIANS_TO_DEGREES*m_entries[i].m_angleWidth);

      if (highlightRadius > wedgeStart) {
        //glColor4f(0.3f, 1.0f, 0.3f, opacity);
        //Utilities::drawPartialDisk(pos, wedgeStart, highlightRadius, Utilities::RADIANS_TO_DEGREES*m_entries[i].m_angleStart, Utilities::RADIANS_TO_DEGREES*m_entries[i].m_angleWidth);
      }

      m_entries[i].draw(this);
    }
  //}
#if 0
  glColor3f(0.35f, 0.35f, 0.35f);
  gl::drawSolidCircle(pos, relativeToAbsolute(m_innerRadius), 50);
#endif
  
  ci::ColorA textColor(0.95f, 0.95f, 0.95f, menuOpacity);
  gl::drawStringCentered(m_name, pos - Vec2f(0, FONT_SIZE/2.0f), textColor, m_boldFont);
  gl::drawStringCentered(m_activeName, pos + Vec2f(0, FONT_SIZE/2.0f), textColor, m_font);
  glPopMatrix();
}

int Menu::checkCollision(const Vector2& pos) const {
  // returns the entry this position hits, or -1 if outside

  float angle, radius;
  toRadialCoordinates(pos, radius, angle);
  const float sweepStart = getSweepStart();
  const float sweepEnd = getSweepEnd();

  if (angle <= sweepStart || angle >= sweepEnd) {
    // outside angle range
    return -1;
  } else if (radius > m_activation.value*m_outerRadius) {
    // outside radius range
    return -1;
  } else {
    // calculate which entry the point hits
    const float ratio = (angle - sweepStart)/(sweepEnd - sweepStart);
    const int entry = static_cast<int>(0.999999f * ratio * m_entries.size());
    return entry;
  }
}

bool Menu::checkPadCollision(const Vector2& pos) const {
  const Vector2 diff = (pos - m_position);
  return diff.squaredNorm() < (m_innerRadius * m_innerRadius);
}

void Menu::setWindowSize(const ci::Vec2i& size) {
  const float width = static_cast<float>(size.x);
  const float height = static_cast<float>(size.y);
  m_windowDiagonal = std::sqrt(width*width + height*height)/2.0f;
  m_windowAspect = width / height;

  m_windowSize << width, height;
  m_windowCenter = 0.5f * m_windowSize;
}

void Menu::toRadialCoordinates(const Vector2& pos, float& radius, float& angle) const {
  const Vector2 diff = (pos - m_position);
  angle = std::atan2(diff.x(), diff.y());
  if (angle < 0) {
    angle += static_cast<float>(2.0 * M_PI);
  }
  radius = diff.norm();
}

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
  , _draw_color_menu(false)
{
  _type_menu.m_name = "Tool";
  _type_menu.m_position << 0.5f, 0.9f;
  _type_menu.setNumEntries(7);

  _type_menu.m_entries[0].m_iconType = Menu::TOOL_PAINT;
  _type_menu.m_entries[0].useSvg = true;
  _type_menu.m_entries[1].m_iconType = Menu::TOOL_PUSH;
  _type_menu.m_entries[1].useSvg = true;
  _type_menu.m_entries[2].m_iconType = Menu::TOOL_SWEEP;
  _type_menu.m_entries[2].useSvg = true;
  _type_menu.m_entries[3].m_iconType = Menu::TOOL_FLATTEN;
  _type_menu.m_entries[3].useSvg = true;
  _type_menu.m_entries[4].m_iconType = Menu::TOOL_SMOOTH;
  _type_menu.m_entries[4].useSvg = true;
  _type_menu.m_entries[5].m_iconType = Menu::TOOL_SHRINK;
  _type_menu.m_entries[5].useSvg = true;
  _type_menu.m_entries[6].m_iconType = Menu::TOOL_GROW;
  _type_menu.m_entries[6].useSvg = true;
  
  static const int NUM_STRENGTH_ENTRIES = 8;
  _strength_menu.m_name = "Strength";
  _strength_menu.m_position << 0.025f, 0.9f;
  _strength_menu.setNumEntries(NUM_STRENGTH_ENTRIES);
  for (int i=0; i<NUM_STRENGTH_ENTRIES; i++) {
    const float ratio = static_cast<float>(i+1)/static_cast<float>(NUM_STRENGTH_ENTRIES);
    Menu::MenuEntry& entry = _strength_menu.m_entries[i];
    entry.useSvg = false;
    entry.m_radius = 0.005f + ratio*0.03f;
    entry.m_value = ratio;
  }
    
  static const int NUM_SIZE_ENTRIES = 8;
  _size_menu.m_name = "Size";
  _size_menu.m_position << 0.975f, 0.9f;
  _size_menu.setNumEntries(NUM_SIZE_ENTRIES);
  for (int i=0; i<NUM_SIZE_ENTRIES; i++) {
    const float ratio = static_cast<float>(i+1)/static_cast<float>(NUM_SIZE_ENTRIES);
    Menu::MenuEntry& entry = _size_menu.m_entries[i];
    entry.useSvg = false;
    entry.m_radius = 0.005f + ratio*0.03f;
    entry.m_value = 40.0f*ratio;
  }

  _color_menu.m_name = "Color";
}

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

void UserInterface::update(const std::vector<Vec4f>& _Tips, Sculpt* sculpt)
{
  //static const float ACTIVATION_BONUS_RADIUS = 0.4f;

  _type_menu.update(_Tips, sculpt);
  _strength_menu.update(_Tips, sculpt);
  _size_menu.update(_Tips, sculpt);
  if (_draw_color_menu) {
    _color_menu.update(_Tips, sculpt);
  }

  static const float ACTIVATION_RATE = 1.75f;

  double curTimeSeconds = app::getElapsedSeconds();
  float elapsed = static_cast<float>(curTimeSeconds - _lastUpdateTime);

  // update metaballs
  clearMetaballs();

  _zoom_amount = 1.0f/(pow(RADIUS_FALLOFF, _depth_offset));

  // perform depth first search
  for (size_t i=0; i<_elements.size(); i++) {
    _elements[i].setVisited(false);
    _elements[i].setMetaballIdx(-1);
  }
  _elements[_root_node].setVisibility(1.0f);
  _elements[_root_node].setVisited(true);
  std::vector<TraversalInfo> stack;
  std::vector<Force> forces;

  const float activationAmt = elapsed * ACTIVATION_RATE;

  // add root node at middle
  const Vec2f window_center = Vec2f(_windowSize.x/2.0f, _windowSize.y/2.0f);
  const Vec2f window_corner = Vec2f(0.0f, static_cast<float>(_windowSize.y));
  stack.push_back(TraversalInfo(_root_node, 0, static_cast<float>(-M_PI/4.0), _focus_offset + window_corner));
  float weighted_radius = _zoom_amount * radiusForDepth(0);
  Vec2f weighted_offset = Vec2f::zero();
  float weighted_depth = 0.0f;
  float offset_weight = 1.0f;
  float maxLevel1 = 0.0f;
  while (!stack.empty()) {
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
    for (size_t i=0; i<connections.size(); i++) {
      int connect = connections[i];
      if (!_elements[connect].isVisited()) {
        maxChildActivation = std::max(maxChildActivation, _elements[connect].getActivation());
      }
    }

    float oldSmoothActivation = Utilities::SmootherStep(_elements[cur_idx].getActivation());
    //if (cur_depth > 0)
    //{
      cur_radius *= (1.0f + 0.25f*oldSmoothActivation);
    //}
    float maxDelta = maxChildActivation > 0 ? activationAmt : -activationAmt;
    for (size_t j=0; j<_Tips.size(); j++) {
      Vec2f pos(_windowSize.x*_Tips[j].x, _windowSize.y*_Tips[j].y);
      float dist = cur_radius / pos.distance(cur_transform);
      dist = math<float>::clamp(dist);
      dist = 2.0f * (dist*dist - 0.5f);
      maxDelta = std::max(maxDelta, _Tips[j].w * (activationAmt * dist));
    }
    _elements[cur_idx].modifyActivation(_elements[cur_idx].getVisibility() * maxDelta);

    float smoothActivation = Utilities::SmootherStep(_elements[cur_idx].getActivation());
    //if (cur_depth > 0)
    //{
      ColorA idleColor = _elements[cur_idx].isLeafNode() ? LEAF_COLOR : NORMAL_COLOR;
      ColorA activeColor = _elements[cur_idx].isLeafNode() ? LEAF_ACTIVATED_COLOR : NORMAL_ACTIVATED_COLOR;
      ColorA curColor = (1.0f - smoothActivation)*idleColor + smoothActivation*activeColor;
      if (_elements[cur_idx].getVisibility() > 0.01f) {
        int idx = addMetaball(cur_transform, cur_radius, Vec4f(curColor.r, curColor.g, curColor.b, curColor.a), _elements[cur_idx].getVisibility(), _elements[cur_idx].getClicking());
        _elements[cur_idx].setMetaballIdx(idx);
      }
      float weight = cur_depth*smoothActivation;
      weighted_offset += weight*(window_center - cur_transform);
      weighted_radius += weight*cur_radius;
      offset_weight += weight;
      weighted_depth += weight*cur_depth;
      //if (cur_depth == 1)
      //{
      //  forces.push_back(Force(cur_transform, (_windowDiagonal/4.0f)*smoothActivation, cur_idx));
      //}
    //}
    //else
    //{
      //smoothActivation = 1.0f;
    //}
    _elements[cur_idx].setVisited(true);

    // process neighboring nodes that haven't already been visited
    if (num_connections > 0) {
      float angle_inc = static_cast<float>(1.4*M_PI/num_connections);
      float angle_offset = 0.0f;
      float horiz_mult = static_cast<float>(_windowSize.y);
      float vert_mult = static_cast<float>(_windowSize.y);
      if (cur_depth == 0) {
        // root node has children wrapping all the way around
        angle_inc = static_cast<float>(0.5f*M_PI/num_connections);
        //angle_offset = angle_inc/2.0f;
        //const float aspect = static_cast<float>(_windowSize.x)/static_cast<float>(_windowSize.y);
        //horiz_mult *= aspect;
      }
      //else if (cur_depth == 1)
      //{
      //  cur_angle += static_cast<float>(M_PI);
      //}
      const float start_angle = cur_angle - (num_connections/2.0f)*angle_inc + angle_offset;
      float cur_angle = start_angle;

      // add all neighbors to stack
      for (int i=0; i<num_connections; i++) {
        int connect = connections[i];
        if (!_elements[connect].isVisited()) {
          //const float childActivation = Utilities::SmootherStep(_elements[connect].getActivation());
          //cur_angle += childActivation * angle_inc;
          _elements[connect].setVisibility(smoothActivation);
          Vec2f offset = 2.0f*radiusForDepth(static_cast<float>(cur_depth)) * smoothActivation * Vec2f(horiz_mult*std::cos(cur_angle), vert_mult*std::sin(cur_angle));
          /*if (cur_depth == 0) {
            _elements[connect].smoothForceOffset(getCombinedForcesAt(cur_transform + offset, connect));
            offset += _elements[connect].getForceOffset();
          }*/
          offset *= _zoom_amount;
          stack.push_back(TraversalInfo(connect, cur_depth+1, cur_angle, cur_transform + offset));
          //cur_angle += childActivation * angle_inc;
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
      //float activation = Utilities::SmootherStep(_elements[i].getActivation());
      Vec2f pos = _metaball_positions[idx];
      pos.y = _windowSize.y - pos.y;
      ColorA textColor(ColorA::white());
      textColor.a = _elements[i].getVisibility();
      drawStringCentered(_elements[i].getName(), pos - Vec2f(0, FONT_SIZE/2.0f), textColor, font);
    }
  }

  enableAlphaBlending();
  _type_menu.draw();
  _strength_menu.draw();
  _size_menu.draw();
  if (_draw_color_menu) {
    _color_menu.draw();
  }
}

void UserInterface::setWindowSize(const Vec2i& _Size)
{
  _windowSize = _Size;
  const float width = static_cast<float>(_windowSize.x);
  const float height = static_cast<float>(_windowSize.y);
  _windowDiagonal = std::sqrt(width*width + height*height)/2.0f;

  _type_menu.setWindowSize(_Size);
  _strength_menu.setWindowSize(_Size);
  _size_menu.setWindowSize(_Size);
  if (_draw_color_menu) {
    _color_menu.setWindowSize(_Size);
  }
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

  result = std::max(result, _type_menu.m_activation.value);
  result = std::max(result, _strength_menu.m_activation.value);
  result = std::max(result, _size_menu.m_activation.value);
  result = std::max(result, _color_menu.m_activation.value);

  return result;
}

void UserInterface::handleSelections(Sculpt* sculpt, LeapInteraction* leap) {
  if (_type_menu.m_curSelectedEntry != -1) {
    Menu::MenuEntry& entry = _type_menu.m_entries[_type_menu.m_curSelectedEntry];
    switch (entry.m_iconType) {
    case Menu::TOOL_GROW: sculpt->setSculptMode(Sculpt::INFLATE); break;
    case Menu::TOOL_SHRINK: sculpt->setSculptMode(Sculpt::DEFLATE); break;
    case Menu::TOOL_SMOOTH: sculpt->setSculptMode(Sculpt::SMOOTH); break;
    case Menu::TOOL_FLATTEN: sculpt->setSculptMode(Sculpt::FLATTEN); break;
    case Menu::TOOL_SWEEP: sculpt->setSculptMode(Sculpt::SWEEP); break;
    case Menu::TOOL_PUSH: sculpt->setSculptMode(Sculpt::PUSH); break;
    case Menu::TOOL_PAINT: sculpt->setSculptMode(Sculpt::PAINT); break;
    }
  }
  if (_strength_menu.m_curSelectedEntry != -1) {
    Menu::MenuEntry& entry = _strength_menu.m_entries[_strength_menu.m_curSelectedEntry];
    leap->setBrushStrength(entry.m_value);
  }
  if (_size_menu.m_curSelectedEntry != -1) {
    Menu::MenuEntry& entry = _size_menu.m_entries[_size_menu.m_curSelectedEntry];
    leap->setBrushRadius(entry.m_value);
  }

  if (_draw_color_menu) {
    //_color_menu
  }
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
  float radius = MAX_RADIUS * std::pow(RADIUS_FALLOFF, depth);
  /*if (depth == 0)
  {
    return radius * 1.1f * _radius_mult;
  }*/
  return radius;
}
