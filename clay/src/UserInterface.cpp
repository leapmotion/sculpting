#include "StdAfx.h"
#include "UserInterface.h"
#include "cinder/app/App.h"
#include "Utilities.h"
#include "Sculpt.h"
#include "LeapInteraction.h"
#include "Environment.h"
#include "ThreeForm.h"

using namespace ci;
using namespace ci::gl;

const ColorA UserInterface::LEAF_COLOR(1.0f, 0.35f, 0.1f, 1.0f);
const ColorA UserInterface::NORMAL_COLOR(0.2f, 0.3f, 0.5f, 1.0f);
const ColorA UserInterface::LEAF_ACTIVATED_COLOR(0.1f, 1.0f, 0.35f, 1.0f);
const ColorA UserInterface::NORMAL_ACTIVATED_COLOR(0.65f, 0.7f, 1.0, 1.0f);
const float UserInterface::RADIUS_FALLOFF = 0.6f;

const float Menu::FONT_SIZE = 36.0f;
const float Menu::RING_THICKNESS_RATIO = 0.3f;
const float Menu::STRENGTH_UI_MULT = 10.0f;
const float Menu::BASE_OUTER_RADIUS = 0.15f;
const float Menu::OUTER_RADIUS_PER_ENTRY = 0.025f;
const float Menu::BASE_INNER_RADIUS = 0.05f;
const float Menu::SWEEP_ANGLE = 3.5f;
ci::Font Menu::m_font;
ci::Font Menu::m_boldFont;
Vector2 Menu::m_windowSize = Vector2::Constant(2.0f);
float Menu::m_windowDiagonal = 1.0f;
float Menu::m_windowAspect = 1.0f;
Vector2 Menu::m_windowCenter = Vector2::Constant(1.0f);
std::vector<ci::gl::Texture> Menu::m_icons;

Menu::Menu() : m_outerRadius(BASE_OUTER_RADIUS), m_innerRadius(BASE_INNER_RADIUS), m_sweepAngle(SWEEP_ANGLE),
  m_curSelectedEntry(-1), m_deselectTime(0.0), m_selectionTime(0.0), m_prevSelectedEntry(-1),
  m_activeColor(Color::white()), m_defaultEntry(0), m_actionsOnly(false) {
  m_activation.value = 0.0f;
}

void Menu::update(const std::vector<Vec4f>& tips, Sculpt* sculpt) {
  static const float PARENT_SMOOTH_STRENGTH = 0.925f;
  static const float CHILD_SMOOTH_STRENGTH = 0.85f;
  static const float SELECTION_COOLDOWN = 1.0f;
  static const float MIN_TIME_SINCE_SCULPTING = 0.5f; // prevent interacting with menu if we're sculpting
  static const float MIN_PARENT_ACTIVATION = 0.75f; // amount of menu activation needed before children can be activated
  const double lastSculptTime = sculpt->getLastSculptTime();
  const double curTime = ci::app::getElapsedSeconds();
  const int numTips = tips.size();
  const int numEntries = m_entries.size();
  const float timeSinceSculpting = static_cast<float>(curTime - lastSculptTime);

  if (timeSinceSculpting > MIN_TIME_SINCE_SCULPTING) {
    float angle, radius;
    for (int i=0; i<numTips; i++) {
      //const Vector2 pos((tips[i].x - 0.5f)*m_windowAspect + 0.5f, 1.0f - tips[i].y);
      const Vector2 pos(tips[i].x, 1.0f - tips[i].y);
      const float z = tips[i].z;
      const float strength = tips[i].w;
      toRadialCoordinates(pos, radius, angle);

      // check for hovering in wedges
      const int hit = checkCollision(pos);
      if (hit >= 0 && m_activation.value > MIN_PARENT_ACTIVATION) {
        m_entries[hit].m_hoverStrength.Update(1.0f, curTime, CHILD_SMOOTH_STRENGTH);
        m_entries[hit].m_activationStrength.Update(radius/m_outerRadius, curTime, CHILD_SMOOTH_STRENGTH);
      }

      // check for center pad
      const float curRadius = m_activation.value*(m_outerRadius - m_innerRadius) + m_innerRadius;
      if (radius < curRadius) {
        // colliding with pad
        m_activation.Update(1.0f, curTime, PARENT_SMOOTH_STRENGTH);
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
    m_activation.Update(0.0f, curTime, PARENT_SMOOTH_STRENGTH);

    // finger left the area, so check whether to select the most activated entry
    if (maxActivation > 0.2f) {
      if ((maxIdx != m_prevSelectedEntry && m_curSelectedEntry != maxIdx) || (m_curSelectedEntry == -1 && static_cast<float>(curTime - m_deselectTime) > SELECTION_COOLDOWN)) {
        m_curSelectedEntry = maxIdx;
        m_selectionTime = curTime;
        m_actualName = m_activeName = m_entries[m_curSelectedEntry].toString();
      }
    }
  }

  if (hasSelectedEntry()) {
    m_entries[m_curSelectedEntry].m_hoverStrength.Update(1.0f, curTime, CHILD_SMOOTH_STRENGTH);
    m_entries[m_curSelectedEntry].m_activationStrength.Update(1.0f, curTime, CHILD_SMOOTH_STRENGTH);
    if (static_cast<float>(curTime - m_selectionTime) > SELECTION_COOLDOWN) {
      m_prevSelectedEntry = m_curSelectedEntry;
      m_curSelectedEntry = -1;
      m_deselectTime = curTime;
    }
  }

  for (int i=0; i<numEntries; i++) {
    if (m_entries[i].m_hoverStrength.lastTimeSeconds < curTime) {
      m_entries[i].m_hoverStrength.Update(0.0f, curTime, CHILD_SMOOTH_STRENGTH);
      m_entries[i].m_activationStrength.Update(0.0f, curTime, CHILD_SMOOTH_STRENGTH);
    }
  }

  float parentCenter = relativeToAbsolute(m_activation.value * getRingRadius());
  float parentThickness = relativeToAbsolute(RING_THICKNESS_RATIO * m_activation.value * m_outerRadius);
  m_wedgeStart = parentCenter - parentThickness/2.0f;
  m_wedgeEnd = parentCenter + parentThickness/2.0f;

  const float parentActivation = m_activation.value;
  const float angleStart = getSweepStart();
  const float angleInc = m_sweepAngle / numEntries;
  float curAngle = angleStart + angleInc/2.0f;
  const ci::Vec2f absolute = relativeToAbsolute(m_position);
  const Vector2 parentPosition(absolute.x, absolute.y);
  
  // update the positions of menu entries
  for (int i=0; i<numEntries; i++) {
    const Vector2 dir(std::sin(curAngle), std::cos(curAngle));
    const float entryActivation = m_entries[i].m_activationStrength.value;
    const float maxActivation = std::max(parentActivation, entryActivation);
    const float entryRadius = relativeToAbsolute(maxActivation * getRingRadius());
    const float thickness = relativeToAbsolute(RING_THICKNESS_RATIO * maxActivation * m_outerRadius);
    const float highlightRadius = relativeToAbsolute(parentActivation * m_outerRadius * entryActivation);
    const float wedgeStart = entryRadius - thickness/2.0f;
    const float bonus = std::max(0.0f, highlightRadius - wedgeStart);

    m_entries[i].m_position = parentPosition + (bonus+entryRadius) * dir;
    m_entries[i].m_angleStart = curAngle - angleInc/2.0f;
    m_entries[i].m_angleWidth = angleInc;
    m_entries[i].m_wedgeStart = entryRadius - thickness/2.0f + bonus;
    m_entries[i].m_wedgeEnd = entryRadius + thickness/2.0f + bonus;

    curAngle += angleInc;
  }
}

void Menu::setNumEntries(int num) {
  m_entries.resize(num);
  m_outerRadius = BASE_OUTER_RADIUS + num*OUTER_RADIUS_PER_ENTRY;
}

void Menu::draw() const {
  glPushMatrix();

  static const float BASE_BRIGHTNESS = 0.2f;
  const float activation = m_activation.value;
  const float menuOpacity = getOpacity(activation);
  const ci::Vec2f pos = relativeToAbsolute(m_position);

  // complete the remainder of the ring
  const float parentStart =  Utilities::RADIANS_TO_DEGREES*(getSweepStart() + m_sweepAngle);
  const float parentSweep = 360.0f - Utilities::RADIANS_TO_DEGREES * m_sweepAngle;
  glColor4f(BASE_BRIGHTNESS, BASE_BRIGHTNESS, BASE_BRIGHTNESS, getOpacity(0.0f));
  Utilities::drawPartialDisk(pos, m_wedgeStart, m_wedgeEnd, parentStart, parentSweep);

  // draw each entry
  for (size_t i=0; i<m_entries.size(); i++) {
    const float entryActivation = m_entries[i].m_activationStrength.value;
    const float entryOpacity = getOpacity(entryActivation);
    const float wedgeStart = m_entries[i].m_wedgeStart;
    const float wedgeEnd = m_entries[i].m_wedgeEnd;
    const float angleStart = Utilities::RADIANS_TO_DEGREES*m_entries[i].m_angleStart;
    const float angleWidth = Utilities::RADIANS_TO_DEGREES*m_entries[i].m_angleWidth;
    const bool isSelected = (!m_actionsOnly) && (i == m_prevSelectedEntry);

    // draw wedge behind
    if (m_entries[i].drawMethod == MenuEntry::COLOR) {
      const ci::Color& origColor = m_entries[i].m_color;
      ci::Vec3f color(origColor.r, origColor.g, origColor.b);
      color *= (1.0f + 0.2f*m_entries[i].m_hoverStrength.value);
      color *= (1.0f + 0.4f*(isSelected ? 1.0f : entryActivation));
      color.x = ci::math<float>::clamp(color.x);
      color.y = ci::math<float>::clamp(color.y);
      color.z = ci::math<float>::clamp(color.z);
      gl::color(ci::ColorA(color.x, color.y, color.z, entryOpacity));
    } else {
      const float brightness = BASE_BRIGHTNESS + 0.15f * (isSelected ? 1.0f : m_entries[i].m_hoverStrength.value);
      const float green = 0.65f * entryActivation;
      glColor4f(brightness, brightness + green, brightness, entryOpacity);
    }
    Utilities::drawPartialDisk(pos, wedgeStart, wedgeEnd, angleStart, angleWidth);

    m_entries[i].draw(this, isSelected);
  }
  
  // variables for string drawing
  const ci::ColorA titleColor(1.0f, 1.0f, 1.0f, menuOpacity);
  const ci::ColorA valueColor(0.75f, 0.75f, 0.75f, menuOpacity);
  const ci::Vec2f textPos = pos - Vec2f(0.0f, FONT_SIZE/2.0f);
  const ci::Vec2f offset = Vec2f(0.0f, FONT_SIZE/2.0f);
  const float textScale = (0.25f * activation) + relativeToAbsolute(0.04f) / FONT_SIZE;

  // draw menu title and value
  glPushMatrix();
  gl::translate(textPos);
  gl::scale(textScale, textScale);
  gl::drawStringCentered(m_name, -offset, titleColor, m_boldFont);
  if (!m_actionsOnly) {
    if (m_name == "Color") {
      gl::color(ci::ColorA(m_activeColor.r, m_activeColor.g, m_activeColor.b, menuOpacity));
      gl::drawSolidCircle(3.0f*offset, relativeToAbsolute(0.02f), 40);
    } else {
      gl::drawStringCentered(m_activeName, offset, valueColor, m_font);
    }
  }
  glPopMatrix();

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
  static const float TWO_PI = static_cast<float>(2.0*M_PI);
  const ci::Vec2f abs = relativeToAbsolute(pos);
  const ci::Vec2f absMenu = relativeToAbsolute(m_position);
  const ci::Vec2f diff = (abs - absMenu);
  angle = std::atan2(diff.x, diff.y);
  if (angle < getSweepStart()) {
    angle += TWO_PI;
  }
  radius = absoluteToRelative(diff.length());
}

UserInterface::UserInterface() : _lastUpdateTime(0), _windowSize(Vec2i::one()), _windowDiagonal(1.0f),
  _shader(NULL), _root_node(-1), _focus_offset(Vec2f::zero()), _depth_offset(0.0f), _max_level1_activation(0.0f),
  _zoom_amount(1.0f), _radius_mult(0.0f), _draw_color_menu(false), _first_selection_check(true)
{
  int entryType;

  const int NUM_STRENGTH_ENTRIES = 3;
  _strength_menu.m_name = "Strength";
  _strength_menu.m_position << 0.2f, 0.925f;
  _strength_menu.setNumEntries(NUM_STRENGTH_ENTRIES);
  _strength_menu.m_angleOffset = angleOffsetForPosition(_strength_menu.m_position);
  _strength_menu.m_defaultEntry = NUM_STRENGTH_ENTRIES/2;
  entryType = Menu::STRENGTH_LOW;
  for (int i=0; i<NUM_STRENGTH_ENTRIES; i++) {
    const float ratio = static_cast<float>(i+1)/static_cast<float>(NUM_STRENGTH_ENTRIES);
    Menu::MenuEntry& entry = _strength_menu.m_entries[i];
    entry.drawMethod = Menu::MenuEntry::ICON;
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    //entry.m_radius = 0.005f + ratio*0.03f;
    entry.m_value = Menu::STRENGTH_UI_MULT*ratio;
  }

  const int NUM_TOOL_ENTRIES = 7;
  _type_menu.m_name = "Tool";
  _type_menu.m_position << 0.5f, 0.925f;
  _type_menu.m_angleOffset = angleOffsetForPosition(_type_menu.m_position);
  _type_menu.setNumEntries(NUM_TOOL_ENTRIES);
  entryType = Menu::TOOL_PAINT;
  _type_menu.m_defaultEntry = 1;
  for (int i=0; i<NUM_TOOL_ENTRIES; i++) {
    Menu::MenuEntry& entry = _type_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::ICON;
  }
    
  const int NUM_SIZE_ENTRIES = 8;
  _size_menu.m_name = "Size";
  _size_menu.m_position << 0.8f, 0.925f;
  _size_menu.setNumEntries(NUM_SIZE_ENTRIES);
  _size_menu.m_angleOffset = angleOffsetForPosition(_size_menu.m_position); //static_cast<float>(M_PI);
  _size_menu.m_defaultEntry = NUM_SIZE_ENTRIES/2;
  for (int i=0; i<NUM_SIZE_ENTRIES; i++) {
    const float ratio = static_cast<float>(i+1)/static_cast<float>(NUM_SIZE_ENTRIES);
    Menu::MenuEntry& entry = _size_menu.m_entries[i];
    //entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::CIRCLE;
    entry.m_radius = 0.005f + ratio*0.03f;
    entry.m_value = 40.0f*ratio;
  }

  const int NUM_COLOR_ENTRIES = 10;
  _color_menu.m_name = "Color";
  _color_menu.m_position << 0.925f, 0.2f;
  _color_menu.setNumEntries(NUM_COLOR_ENTRIES);
  _color_menu.m_angleOffset = angleOffsetForPosition(_color_menu.m_position); //static_cast<float>(3.0*M_PI/2.0);
  _color_menu.m_defaultEntry = NUM_COLOR_ENTRIES/2;
  for (int i=0; i<NUM_COLOR_ENTRIES; i++) {
    const float ratio = static_cast<float>(i-3)/static_cast<float>(NUM_COLOR_ENTRIES-4);
    Menu::MenuEntry& entry = _color_menu.m_entries[i];
    //entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::COLOR;
    if (i == 0) {
      entry.m_color = ci::Color::white();
    } else if (i == 1) {
      entry.m_color = ci::Color::gray(0.6f);
    } else if (i == 2) {
      entry.m_color = ci::Color::gray(0.1f);
    } else {
      entry.m_color = ci::hsvToRGB(ci::Vec3f(ratio, 0.75f, 0.75f));
    }
  }

  const int NUM_MATERIAL_ENTRIES = 5;
  _material_menu.m_name = "Material";
  _material_menu.m_position << 0.075f, 0.3f;
  _material_menu.setNumEntries(NUM_MATERIAL_ENTRIES);
  entryType = Menu::MATERIAL_PORCELAIN;
  _material_menu.m_angleOffset = angleOffsetForPosition(_material_menu.m_position); //static_cast<float>(M_PI/2.0);
  _material_menu.m_defaultEntry = 0;
  for (int i=0; i<NUM_MATERIAL_ENTRIES; i++) {
    Menu::MenuEntry& entry = _material_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::ICON;
  }

  const int NUM_SPIN_ENTRIES = 4;
  _spin_menu.m_name = "Spin";
  _spin_menu.m_position << 0.075f, 0.6f;
  _spin_menu.setNumEntries(NUM_SPIN_ENTRIES);
  entryType = Menu::SPIN_OFF;
  _spin_menu.m_angleOffset = angleOffsetForPosition(_spin_menu.m_position); //static_cast<float>(M_PI/2.0);
  _spin_menu.m_defaultEntry = 0;
  for (int i=0; i<NUM_SPIN_ENTRIES; i++) {
    Menu::MenuEntry& entry = _spin_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }
  
  const int NUM_WIREFRAME_ENTRIES = 2;
  _wireframe_menu.m_name = "Wireframe";
  _wireframe_menu.m_position << 0.2f, 0.075f;
  _wireframe_menu.setNumEntries(NUM_WIREFRAME_ENTRIES);
  entryType = Menu::WIREFRAME_OFF;
  _wireframe_menu.m_angleOffset = angleOffsetForPosition(_wireframe_menu.m_position); //static_cast<float>(2.0f*M_PI);
  _wireframe_menu.m_defaultEntry = 0;
  for (int i=0; i<NUM_WIREFRAME_ENTRIES; i++) {
    Menu::MenuEntry& entry = _wireframe_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_SYMMETRY_ENTRIES = 2;
  _symmetry_menu.m_name = "Symmetry";
  _symmetry_menu.m_position << 0.35f, 0.075f;
  _symmetry_menu.setNumEntries(NUM_SYMMETRY_ENTRIES);
  entryType = Menu::SYMMETRY_OFF;
  _symmetry_menu.m_angleOffset = angleOffsetForPosition(_symmetry_menu.m_position); //static_cast<float>(2.0f*M_PI);
  _symmetry_menu.m_defaultEntry = 0;
  for (int i=0; i<NUM_SYMMETRY_ENTRIES; i++) {
    Menu::MenuEntry& entry = _symmetry_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_HISTORY_ENTRIES = 2;
  _history_menu.m_name = "History";
  _history_menu.m_position << 0.5f, 0.075f;
  _history_menu.setNumEntries(NUM_HISTORY_ENTRIES);
  entryType = Menu::HISTORY_UNDO;
  _history_menu.m_angleOffset = angleOffsetForPosition(_history_menu.m_position); //static_cast<float>(2.0f*M_PI);
  _history_menu.m_defaultEntry = 0;
  _history_menu.m_actionsOnly = true;
  for (int i=0; i<NUM_HISTORY_ENTRIES; i++) {
    Menu::MenuEntry& entry = _history_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_TIME_OF_DAY_ENTRIES = 2;
  _time_of_day_menu.m_name = "Time of Day";
  _time_of_day_menu.m_position << 0.65f, 0.075f;
  _time_of_day_menu.setNumEntries(NUM_TIME_OF_DAY_ENTRIES);
  entryType = Menu::TIME_DAWN;
  _time_of_day_menu.m_angleOffset = angleOffsetForPosition(_time_of_day_menu.m_position); //static_cast<float>(2.0f*M_PI);
  _time_of_day_menu.m_defaultEntry = rand() % 2;
  for (int i=0; i<NUM_TIME_OF_DAY_ENTRIES; i++) {
    Menu::MenuEntry& entry = _time_of_day_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const std::vector<Environment::EnvironmentInfo>& infos = Environment::getEnvironmentInfos();
  const int NUM_ENVIRONMENT_ENTRIES = infos.size();
  _environment_menu.m_name = "Scene";
  _environment_menu.m_position << 0.8f, 0.075f;
  _environment_menu.setNumEntries(NUM_ENVIRONMENT_ENTRIES);
  entryType = Menu::ENVIRONMENT_ISLANDS;
  _environment_menu.m_angleOffset = angleOffsetForPosition(_environment_menu.m_position); //static_cast<float>(2.0f*M_PI);
  _environment_menu.m_defaultEntry = rand() % NUM_ENVIRONMENT_ENTRIES;
  for (int i=0; i<NUM_ENVIRONMENT_ENTRIES; i++) {
    Menu::MenuEntry& entry = _environment_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_MAIN_ENTRIES = 3;
  _main_menu.m_name = "3Form";
  _main_menu.m_position << 0.075f, 0.075f;
  _main_menu.setNumEntries(NUM_MAIN_ENTRIES);
  entryType = Menu::MAIN_ABOUT;
  _main_menu.m_angleOffset = angleOffsetForPosition(_main_menu.m_position); //static_cast<float>(2.0f*M_PI);
  _main_menu.m_defaultEntry = 0;
  _main_menu.m_actionsOnly = true;
  for (int i=0; i<NUM_MAIN_ENTRIES; i++) {
    Menu::MenuEntry& entry = _main_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_FILE_ENTRIES = 5;
  _file_menu.m_name = "File";
  _file_menu.m_position << 0.925f, 0.5f;
  _file_menu.setNumEntries(NUM_FILE_ENTRIES);
  entryType = Menu::FILE_LOAD;
  _file_menu.m_angleOffset = angleOffsetForPosition(_file_menu.m_position); //static_cast<float>(2.0f*M_PI);
  _file_menu.m_defaultEntry = 0;
  _file_menu.m_actionsOnly = true;
  for (int i=0; i<NUM_FILE_ENTRIES; i++) {
    Menu::MenuEntry& entry = _file_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_SOUND_ENTRIES = 2;
  _sound_menu.m_name = "Audio";
  _sound_menu.m_position << 0.925f, 0.8f;
  _sound_menu.setNumEntries(NUM_SOUND_ENTRIES);
  entryType = Menu::SOUND_ON;
  _sound_menu.m_angleOffset = angleOffsetForPosition(_sound_menu.m_position); //static_cast<float>(2.0f*M_PI);
  _sound_menu.m_defaultEntry = 0;
  for (int i=0; i<NUM_SOUND_ENTRIES; i++) {
    Menu::MenuEntry& entry = _sound_menu.m_entries[i];
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }
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
  _type_menu.update(_Tips, sculpt);
  _strength_menu.update(_Tips, sculpt);
  _size_menu.update(_Tips, sculpt);
  if (_draw_color_menu) {
    _color_menu.update(_Tips, sculpt);
  }
  _material_menu.update(_Tips, sculpt);
  _spin_menu.update(_Tips, sculpt);
  _wireframe_menu.update(_Tips, sculpt);
  _symmetry_menu.update(_Tips, sculpt);
  _history_menu.update(_Tips, sculpt);
  _environment_menu.update(_Tips, sculpt);
  _time_of_day_menu.update(_Tips, sculpt);
  _main_menu.update(_Tips, sculpt);
  _file_menu.update(_Tips, sculpt);
  _sound_menu.update(_Tips, sculpt);

  _cursor_positions.clear();
  for (size_t i=0; i<_Tips.size(); i++) {
    const ci::Vec2f pos(_Tips[i].x, 1.0f - _Tips[i].y);
    _cursor_positions.push_back(pos);
  }

  static const float ACTIVATION_RATE = 1.75f;

  double curTimeSeconds = app::getElapsedSeconds();
#if 0
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
#if 0
  for (size_t i=0; i<_Tips.size(); i++)
  {
    Vec2f pixelLocation(_windowSize.x*_Tips[i].x, _windowSize.y*_Tips[i].y);
    addMetaball(pixelLocation, _windowDiagonal*_Tips[i].z, Vec4f(LEAF_ACTIVATED_COLOR.r, LEAF_ACTIVATED_COLOR.g, LEAF_ACTIVATED_COLOR.b, max_activation), _Tips[i].w, 0.75f);
  }
#endif

  const float new_radius_mult = 1.0f + 0.7f*(1.0f-max_activation);
  static const float RADIUS_MULT_SMOOTH = 0.9f;
  _radius_mult = RADIUS_MULT_SMOOTH*_radius_mult + (1.0f-RADIUS_MULT_SMOOTH)*new_radius_mult;

  _forces = forces;
#endif

  _lastUpdateTime = curTimeSeconds;
}

void UserInterface::draw() const {
  const float opacity = maxActivation();
  for (size_t i=0; i<_cursor_positions.size(); i++) {
    drawCursor(_cursor_positions[i], opacity*opacity);
  }

  enableAlphaBlending();
  _type_menu.draw();
  _strength_menu.draw();
  _size_menu.draw();
  if (_draw_color_menu) {
    _color_menu.draw();
  }
  _material_menu.draw();
  _spin_menu.draw();
  _wireframe_menu.draw();
  _symmetry_menu.draw();
  _history_menu.draw();
  _environment_menu.draw();
  _time_of_day_menu.draw();
  _main_menu.draw();
  _file_menu.draw();
  _sound_menu.draw();
}

void UserInterface::setWindowSize(const Vec2i& _Size)
{
  _windowSize = _Size;
  const float width = static_cast<float>(_windowSize.x);
  const float height = static_cast<float>(_windowSize.y);
  _windowDiagonal = std::sqrt(width*width + height*height)/2.0f;

  Menu::setWindowSize(_Size);
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
  result = std::max(result, _material_menu.m_activation.value);
  result = std::max(result, _spin_menu.m_activation.value);
  result = std::max(result, _wireframe_menu.m_activation.value);
  result = std::max(result, _symmetry_menu.m_activation.value);
  result = std::max(result, _history_menu.m_activation.value);
  result = std::max(result, _environment_menu.m_activation.value);
  result = std::max(result, _time_of_day_menu.m_activation.value);
  result = std::max(result, _main_menu.m_activation.value);
  result = std::max(result, _file_menu.m_activation.value);
  result = std::max(result, _sound_menu.m_activation.value);

  return result;
}

void UserInterface::handleSelections(Sculpt* sculpt, LeapInteraction* leap, ThreeFormApp* app, Mesh* mesh) {
  if (_first_selection_check) {
    initializeMenu(_type_menu);
    initializeMenu(_strength_menu);
    initializeMenu(_size_menu);
    initializeMenu(_color_menu);
    initializeMenu(_material_menu);
    initializeMenu(_spin_menu);
    initializeMenu(_wireframe_menu);
    initializeMenu(_symmetry_menu);
    initializeMenu(_history_menu);
    initializeMenu(_environment_menu);
    initializeMenu(_time_of_day_menu);
    initializeMenu(_main_menu);
    initializeMenu(_file_menu);
    initializeMenu(_sound_menu);
    _first_selection_check = false;
  }

  if (_type_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _type_menu.getSelectedEntry();
    Sculpt::SculptMode mode = entry.toSculptMode();
    sculpt->setSculptMode(mode);
    _draw_color_menu = (mode == Sculpt::PAINT);
  }

  if (_strength_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _strength_menu.getSelectedEntry();
    leap->setBrushStrength(entry.m_value / Menu::STRENGTH_UI_MULT);
  }

  if (_size_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _size_menu.getSelectedEntry();
    leap->setBrushRadius(entry.m_value);
  }

  if (_draw_color_menu && _color_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _color_menu.getSelectedEntry();
    const ci::Color& desiredColor = entry.m_color;
    _color_menu.m_activeColor = desiredColor;
    Vector3 color(desiredColor.r, desiredColor.g, desiredColor.b);
    sculpt->setMaterialColor(color);
  }

  if (_material_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _material_menu.getSelectedEntry();
    app->setMaterial(entry.toMaterial());
  }

  if (_spin_menu.hasSelectedEntry() && mesh) {
    const Menu::MenuEntry& entry = _spin_menu.getSelectedEntry();
    mesh->setRotationVelocity(entry.toSpinVelocity());
  }

  if (_wireframe_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _wireframe_menu.getSelectedEntry();
    app->setWireframe(entry.toWireframe());
  }

  if (_symmetry_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _symmetry_menu.getSelectedEntry();
    app->setSymmetry(entry.toSymmetry());
  }

  if (_history_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _history_menu.getSelectedEntry();
  }
  
  if (_environment_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _environment_menu.getSelectedEntry();
    app->setEnvironment(entry.toEnvironmentName());
  }

  if (_time_of_day_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _time_of_day_menu.getSelectedEntry();
    app->setTimeOfDay(entry.toTimeOfDay());
  }

  if (_main_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _main_menu.getSelectedEntry();
    switch (entry.m_entryType) {
    case Menu::MAIN_ABOUT: break;
    case Menu::MAIN_TUTORIAL: break;
    case Menu::MAIN_EXIT: app->quit(); break;
    }
  }

  if (_file_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _file_menu.getSelectedEntry();
    switch (entry.m_entryType) {
    case Menu::FILE_LOAD: app->loadFile(); break;
    case Menu::FILE_RESET: break;
    case Menu::FILE_SAVE_OBJ: app->saveFile(".obj"); break;
    case Menu::FILE_SAVE_PLY: app->saveFile(".ply"); break;
    case Menu::FILE_SAVE_STL: app->saveFile(".stl"); break;
    }
  }

  if (_sound_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _sound_menu.getSelectedEntry();
  }
}

void UserInterface::drawCursor(const ci::Vec2f& position, float opacity) const {
  static const float CURSOR_RADIUS = 40.0f;
  const float bonusRadius = 40.0f * (1.0f - opacity);
  ci::Vec2f screenPos(position.x * _windowSize.x, position.y * _windowSize.y);
  glColor4f(0.7f, 0.9f, 1.0f, opacity);
  ci::gl::drawSolidCircle(screenPos, CURSOR_RADIUS + bonusRadius, 40);
}

void UserInterface::initializeMenu(Menu& menu) {
  const int defaultOption = menu.m_defaultEntry;
  menu.m_curSelectedEntry = menu.m_actionsOnly ? -1 : defaultOption;
  menu.m_actualName = menu.m_activeName = menu.m_entries[defaultOption].toString();
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

float UserInterface::angleOffsetForPosition(const Vector2& pos) {
  const Vector2 diff = pos - Vector2::Constant(0.5f);
  float rotAngle = std::atan2(diff.x(), diff.y()) + static_cast<float>(M_PI);
  if (rotAngle < 0) {
    rotAngle += static_cast<float>(2.0*M_PI);
  }

  float edgeAngle = 0;
  if (fabs(diff.x()) > fabs(diff.y())) {
    if (diff.x() < 0) {
      edgeAngle = static_cast<float>(M_PI/2.0);
    } else {
      edgeAngle = static_cast<float>(3.0*M_PI/2.0);
    }
  } else {
    if (diff.y() < 0) {
      edgeAngle = static_cast<float>(2.0 * M_PI);
    } else {
      edgeAngle = static_cast<float>(M_PI);
    }
  }

  //return 0.5f*(rotAngle + edgeAngle);
  return edgeAngle;
}
