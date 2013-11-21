#include "StdAfx.h"
#include "UserInterface.h"
#include "cinder/app/App.h"
#include "Utilities.h"
#include "Sculpt.h"
#include "LeapInteraction.h"
#include "Environment.h"
#include "Freeform.h"

#include "ReplayUtil.h"

using namespace ci;
using namespace ci::gl;

const float Menu::FONT_SIZE = 36.0f;
const float Menu::RING_THICKNESS_RATIO = 0.3f;
const float Menu::STRENGTH_UI_MULT = 10.0f;
const float Menu::BASE_OUTER_RADIUS = 0.15f;
const float Menu::OUTER_RADIUS_PER_ENTRY = 0.0175f;
const float Menu::BASE_INNER_RADIUS = 0.075f;
const float Menu::SWEEP_ANGLE = 3.5f;
ci::Font Menu::g_font;
ci::Font Menu::g_boldFont;
ci::gl::TextureFontRef Menu::g_textureFont;
ci::gl::TextureFontRef Menu::g_boldTextureFont;
Vector2 Menu::g_windowSize = Vector2::Constant(2.0f);
float Menu::g_windowDiagonal = 1.0f;
float Menu::g_windowAspect = 1.0f;
Vector2 Menu::g_windowCenter = Vector2::Constant(1.0f);
std::vector<ci::gl::Texture> Menu::g_icons;
ci::Vec2f Menu::g_shadowOffset = ci::Vec2f(1.5f, 2.0f);
float Menu::g_zoomFactor = 1.0f;
float Menu::g_maxMenuActivation = 0.0f;
Utilities::ExponentialFilter<float> Menu::g_maxMenuActivationSmoother;
Vector2 Menu::g_forceCenter(Vector2(0.5f, 0.5f));
float Menu::g_timeSinceSculpting = 0.0f;
Utilities::ExponentialFilter<float> Menu::g_sculptMult;

Menu::Menu() : m_outerRadius(BASE_OUTER_RADIUS), m_innerRadius(BASE_INNER_RADIUS), m_sweepAngle(SWEEP_ANGLE),
  m_curSelectedEntry(-1), m_deselectTime(0.0), m_selectionTime(-10.0), m_prevSelectedEntry(-1), m_alwaysActivated(false),
  m_activeColor(Color::white()), m_defaultEntry(0), m_actionsOnly(false), m_haveSelection(false), m_prevClosest(99999.0f)
{
  m_activation.value = 0.0f;
}

void Menu::update(const std::vector<Vec4f>& tips, Sculpt* sculpt) {
  static const float PARENT_SMOOTH_STRENGTH = 0.925f;
  static const float CHILD_SMOOTH_STRENGTH = 0.85f;
  static const float SELECTION_COOLDOWN = 1.0f;
//  static const float MIN_TIME_SINCE_SCULPTING = 0.5f; // prevent interacting with menu if we're sculpting
  static const float MIN_PARENT_ACTIVATION = 0.75f; // amount of menu activation needed before children can be activated
  static const float MAX_ACTIVATION_LIMIT = 0.9f; // how high the max activation can be before other menus can't be activated
  static const float ACTIVATION_AMOUNT = 0.333f; // the amount to add when activating or deactivating
  const double lastSculptTime = sculpt->getLastSculptTime();
  const double curTime = ci::app::getElapsedSeconds();
  const int numTips = tips.size();
  const int numEntries = m_entries.size();
  g_timeSinceSculpting = static_cast<float>(curTime - lastSculptTime);

  float angle = 0.0f;
  float radius = 0.0f;
  const float curRadius = m_activation.value*(m_outerRadius - m_innerRadius) + m_innerRadius;
  float closest = 9999.0f;
  //if (g_sculptMult.value > 0.5f) {
    for (int i=0; i<numTips; i++) {
      //const Vector2 pos((tips[i].x - 0.5f)*m_windowAspect + 0.5f, 1.0f - tips[i].y);
      const Vector2 pos(tips[i].x, 1.0f - tips[i].y);
//      const float z = tips[i].z;
//      const float strength = tips[i].w;
      toRadialCoordinates(pos, radius, angle);

      // check for hovering in wedges
      const int hit = checkCollision(pos);
      if (hit >= 0 && m_activation.value > MIN_PARENT_ACTIVATION) {
        m_entries[hit].m_hoverStrength.Update(1.0f, curTime, CHILD_SMOOTH_STRENGTH);
        m_entries[hit].m_activationStrength.Update(radius/m_outerRadius, curTime, CHILD_SMOOTH_STRENGTH);
      }

      // check for center pad
      if (radius < curRadius && (g_maxMenuActivation == m_activation.value || g_maxMenuActivation < MAX_ACTIVATION_LIMIT)) {
        // colliding with pad
        const float newValue = ci::math<float>::clamp(m_activation.value + ACTIVATION_AMOUNT);
        m_activation.Update(newValue, curTime, PARENT_SMOOTH_STRENGTH);
      }
      closest = std::min(closest, radius);
    }
  //}
  float maxActivation = 0.0f;
  int maxIdx;

  // find the max activation of all entries
  for (int i=0; i<numEntries; i++) {
    const float curActivation = m_entries[i].m_activationStrength.value;
    if (curActivation > maxActivation) {
      maxActivation = curActivation;
      maxIdx = i;
    }
  }
  if (maxActivation > 0.2f) {
    // set this entry to be the current text of the menu
    m_activeName = m_entries[maxIdx].toString();
  } else {
    m_activeName = m_actualName;
  }

  if (closest > curRadius) {
    // we weren't updated this frame
    const float activationMult = m_alwaysActivated ? 1.0f : -1.0f;
    const float newValue = ci::math<float>::clamp(m_activation.value + activationMult*ACTIVATION_AMOUNT);
    m_activation.Update(newValue, curTime, PARENT_SMOOTH_STRENGTH);

    // finger left the area, so check whether to select the most activated entry
    if (radius > curRadius && m_prevClosest < curRadius && maxActivation > 0.25f) {
      if ((maxIdx != m_prevSelectedEntry && m_curSelectedEntry != maxIdx) || (m_curSelectedEntry == -1 && static_cast<float>(curTime - m_deselectTime) > SELECTION_COOLDOWN)) {
        m_curSelectedEntry = maxIdx;
        m_selectionTime = curTime;
        m_actualName = m_activeName = m_entries[m_curSelectedEntry].toString();
        m_haveSelection = true;
      }
    }
  }
  m_prevClosest = closest;

  // keep the currently selected entry sticking out for a bit longer
  if (m_curSelectedEntry >= 0) {
    m_entries[m_curSelectedEntry].m_hoverStrength.Update(1.0f, curTime, CHILD_SMOOTH_STRENGTH);
    m_entries[m_curSelectedEntry].m_activationStrength.Update(1.0f, curTime, CHILD_SMOOTH_STRENGTH);
    if (static_cast<float>(curTime - m_selectionTime) > SELECTION_COOLDOWN) {
      m_prevSelectedEntry = m_curSelectedEntry;
      m_curSelectedEntry = -1;
      m_deselectTime = curTime;
    }
  }

  // deactivate any entries which weren't updated this frame
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
  const ci::Vec2f absolute = relativeToAbsolute(m_position + forceAt(m_position));
  const Vector2 parentPosition(absolute.x, absolute.y);
  
  // update the positions of menu entries
  for (int i=numEntries-1; i>=0; i--) {
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

  static const float BASE_BRIGHTNESS = 0.25f;
  const float activation = m_activation.value;
  const float opacityDiff = std::max(0.0f, g_maxMenuActivationSmoother.value - activation);
  const float menuOpacity = (1.0f - opacityDiff) * getOpacity(activation);
  const ci::Vec2f pos = relativeToAbsolute(m_position + forceAt(m_position));

  if (activation > 0.01f) {
    // complete the remainder of the ring
    const float parentStart =  Utilities::RADIANS_TO_DEGREES*(getSweepStart() + m_sweepAngle);
    const float parentSweep = 360.0f - Utilities::RADIANS_TO_DEGREES * m_sweepAngle;
    glColor4f(BASE_BRIGHTNESS, BASE_BRIGHTNESS, BASE_BRIGHTNESS, getOpacity(0.0f));
    Utilities::drawPartialDisk(pos, m_wedgeStart, m_wedgeEnd, parentStart, parentSweep);
  }

  for (size_t i=0; i<m_entries.size(); i++) {
    const float entryActivation = m_entries[i].m_activationStrength.value;
    if (entryActivation > 0.01f || activation > 0.01f) {
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
        const float green = 0.6f * entryActivation;
        glColor4f(brightness, brightness + green, brightness, entryOpacity);
      }
      Utilities::drawPartialDisk(pos, wedgeStart, wedgeEnd, angleStart, angleWidth);

      m_entries[i].draw(this, isSelected);
    }
  }

  // variables for string drawing
  const ci::ColorA titleColor(1.0f, 1.0f, 1.0f, menuOpacity);
  const ci::ColorA valueColor(0.75f, 0.75f, 0.75f, menuOpacity);
  const ci::ColorA shadowColor(0.1f, 0.1f, 0.1f, menuOpacity);
  const ci::Vec2f textPos = pos - Vec2f(0.0f, FONT_SIZE/2.0f);
  const ci::Vec2f offset = Vec2f(0.0f, FONT_SIZE/2.0f);
  const float textScale = (0.35f * activation) + relativeToAbsolute(0.04f) / FONT_SIZE;

  // draw menu title and value
  glPushMatrix();
  gl::translate(textPos);
  gl::scale(textScale, textScale);
  const ci::Vec2f nameSize = g_boldTextureFont->measureString(m_name);
  const ci::Rectf nameRect(-nameSize.x/2.0f, -offset.y, nameSize.x/2.0f, 100.0f);
  gl::color(shadowColor);
  g_boldTextureFont->drawString(m_name, nameRect, g_shadowOffset);
  gl::color(titleColor);
  g_boldTextureFont->drawString(m_name, nameRect);
  if (!m_actionsOnly) {
    if (m_name == "Color") {
      gl::color(shadowColor);
      gl::drawSolidCircle(3.0f*offset + g_shadowOffset, relativeToAbsolute(0.02f), 40);
      gl::color(ci::ColorA(m_activeColor.r, m_activeColor.g, m_activeColor.b, menuOpacity));
      gl::drawSolidCircle(3.0f*offset, relativeToAbsolute(0.02f), 40);
    } else {
      const ci::Vec2f size = g_textureFont->measureString(m_activeName);
      const ci::Rectf rect(-size.x/2.0f, 2.0f*offset.y, size.x/2.0f, 100.0f);
      gl::color(shadowColor);
      glPushMatrix();
      glScalef(0.75f, 0.75f, 0.75f);
      g_textureFont->drawString(m_activeName, rect, g_shadowOffset);
      gl::color(valueColor);
      g_textureFont->drawString(m_activeName, rect);
      glPopMatrix();
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
    const float ratio = 1.0f - (angle - sweepStart)/(sweepEnd - sweepStart);
    const int entry = static_cast<int>(0.999999f * ratio * m_entries.size());
    return entry;
  }
}

void Menu::setWindowSize(const ci::Vec2i& size) {
  const float width = static_cast<float>(size.x);
  const float height = static_cast<float>(size.y);
  g_windowDiagonal = std::sqrt(width*width + height*height)/2.0f;
  g_windowAspect = width / height;
  g_windowSize << width, height;
  g_windowCenter = 0.5f * g_windowSize;
}

void Menu::toRadialCoordinates(const Vector2& pos, float& radius, float& angle) const {
  static const float TWO_PI = static_cast<float>(2.0*M_PI);
  const ci::Vec2f absPos = relativeToAbsolute(pos, false);
  const ci::Vec2f absMenu = relativeToAbsolute(m_position + forceAt(m_position));
  const ci::Vec2f diff = (absPos - absMenu);
  angle = std::atan2(diff.x, diff.y);
  if (angle < getSweepStart()) {
    angle += TWO_PI;
  }
  radius = absoluteToRelative(diff.length());
}

UserInterface::UserInterface() : _draw_color_menu(false), _first_selection_check(true),
  _draw_confirm_menu(false), _draw_tutorial_menu(false), _last_switch_time(0.0), _tutorial_slide(0)
{
  srand(static_cast<unsigned int>(time(0)));
  int entryType;

  const int NUM_STRENGTH_ENTRIES = 3;
  _strength_menu.setName("Strength");
  _strength_menu.setPosition(Vector2(0.25f, 0.925f));
  _strength_menu.setNumEntries(NUM_STRENGTH_ENTRIES);
  _strength_menu.setAngleOffset(angleOffsetForPosition(_strength_menu.getPosition()));
  _strength_menu.setDefaultEntry(NUM_STRENGTH_ENTRIES/2);
  entryType = Menu::STRENGTH_LOW;
  for (int i=0; i<NUM_STRENGTH_ENTRIES; i++) {
    const float ratio = static_cast<float>(i+1)/static_cast<float>(NUM_STRENGTH_ENTRIES);
    Menu::MenuEntry& entry = _strength_menu.getEntry(i);
    entry.drawMethod = Menu::MenuEntry::ICON;
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.m_value = Menu::STRENGTH_UI_MULT*ratio;
  }

  const int NUM_TOOL_ENTRIES = 7;
  _type_menu.setName("Tool");
  _type_menu.setPosition(Vector2(0.5f, 0.925f));
  _type_menu.setAngleOffset(angleOffsetForPosition(_type_menu.getPosition()));
  _type_menu.setNumEntries(NUM_TOOL_ENTRIES);
  entryType = Menu::TOOL_PAINT;
  _type_menu.setDefaultEntry(1);
  for (int i=0; i<NUM_TOOL_ENTRIES; i++) {
    Menu::MenuEntry& entry = _type_menu.getEntry(i);
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::ICON;
  }
    
  const int NUM_SIZE_ENTRIES = 6;
  _size_menu.setName("Size");
  _size_menu.setPosition(Vector2(0.75f, 0.925f));
  _size_menu.setNumEntries(NUM_SIZE_ENTRIES);
  _size_menu.setAngleOffset(angleOffsetForPosition(_size_menu.getPosition()));
  _size_menu.setDefaultEntry(NUM_SIZE_ENTRIES/2);
  for (int i=0; i<NUM_SIZE_ENTRIES; i++) {
    const float ratio = static_cast<float>(i+1)/static_cast<float>(NUM_SIZE_ENTRIES);
    Menu::MenuEntry& entry = _size_menu.getEntry(i);
    entry.drawMethod = Menu::MenuEntry::CIRCLE;
    entry.m_radius = 0.005f + ratio*0.02f;
    entry.m_value = 30.0f*ratio + 10.0f;
  }

  const int NUM_COLOR_ENTRIES = 10;
  _color_menu.setName("Color");
  _color_menu.setPosition(Vector2(0.925f, 0.65f));
  _color_menu.setNumEntries(NUM_COLOR_ENTRIES);
  _color_menu.setAngleOffset(angleOffsetForPosition(_color_menu.getPosition()));
  _color_menu.setDefaultEntry(NUM_COLOR_ENTRIES/2);
  for (int i=0; i<NUM_COLOR_ENTRIES; i++) {
    const float ratio = static_cast<float>(i-2)/static_cast<float>(NUM_COLOR_ENTRIES-2);
    Menu::MenuEntry& entry = _color_menu.getEntry(i);
    entry.drawMethod = Menu::MenuEntry::COLOR;
    if (i == 0) {
      entry.m_color = ci::Color::white();
    } else if (i == 1) {
      entry.m_color = ci::Color::gray(0.1f);
    } else {
      entry.m_color = ci::hsvToRGB(ci::Vec3f(ratio, 0.75f, 0.75f));
    }
  }

  const int NUM_MATERIAL_ENTRIES = 5;
  _material_menu.setName("Material");
  _material_menu.setPosition(Vector2(0.925f, 0.3f));
  _material_menu.setNumEntries(NUM_MATERIAL_ENTRIES);
  entryType = Menu::MATERIAL_PORCELAIN;
  _material_menu.setAngleOffset(angleOffsetForPosition(_material_menu.getPosition()));
  _material_menu.setDefaultEntry(0);
  for (int i=0; i<NUM_MATERIAL_ENTRIES; i++) {
    Menu::MenuEntry& entry = _material_menu.getEntry(i);
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::ICON;
  }

  const int NUM_SPIN_ENTRIES = 4;
  _spin_menu.setName("Spin");
  _spin_menu.setPosition(Vector2(0.075f, 0.65f));
  _spin_menu.setNumEntries(NUM_SPIN_ENTRIES);
  entryType = Menu::SPIN_OFF;
  _spin_menu.setAngleOffset(angleOffsetForPosition(_spin_menu.getPosition()));
  _spin_menu.setDefaultEntry(0);
  for (int i=0; i<NUM_SPIN_ENTRIES; i++) {
    Menu::MenuEntry& entry = _spin_menu.getEntry(i);
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const std::vector<Environment::EnvironmentInfo>& infos = Environment::getEnvironmentInfos();
  const int NUM_ENVIRONMENT_ENTRIES = infos.size();
  _environment_menu.setName("Scene");
  _environment_menu.setPosition(Vector2(0.75f, 0.075f));
  _environment_menu.setNumEntries(NUM_ENVIRONMENT_ENTRIES);
  entryType = Menu::ENVIRONMENT_ISLANDS;
  _environment_menu.setAngleOffset(angleOffsetForPosition(_environment_menu.getPosition()));
  _environment_menu.setDefaultEntry(rand() % NUM_ENVIRONMENT_ENTRIES);
  for (int i=0; i<NUM_ENVIRONMENT_ENTRIES; i++) {
    Menu::MenuEntry& entry = _environment_menu.getEntry(i);
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_GENERAL_ENTRIES = 4;
  _general_menu.setName("General");
  _general_menu.setPosition(Vector2(0.075f, 0.3f));
  _general_menu.setNumEntries(NUM_GENERAL_ENTRIES);
  entryType = Menu::GENERAL_TUTORIAL;
  _general_menu.setAngleOffset(angleOffsetForPosition(_general_menu.getPosition()));
  _general_menu.setDefaultEntry(0);
  _general_menu.setActionsOnly(true);
  for (int i=0; i<NUM_GENERAL_ENTRIES; i++) {
    Menu::MenuEntry& entry = _general_menu.getEntry(i);
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_OBJECT_ENTRIES = 7;
  _object_menu.setName("Object");
  _object_menu.setPosition(Vector2(0.25f, 0.075f));
  _object_menu.setNumEntries(NUM_OBJECT_ENTRIES);
  entryType = Menu::OBJECT_LOAD;
  _object_menu.setAngleOffset(angleOffsetForPosition(_object_menu.getPosition()));
  _object_menu.setDefaultEntry(0);
  _object_menu.setActionsOnly(true);
  for (int i=0; i<NUM_OBJECT_ENTRIES; i++) {
    Menu::MenuEntry& entry = _object_menu.getEntry(i);
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_EDITING_ENTRIES = 4;
  _editing_menu.setName("Edit");
  _editing_menu.setPosition(Vector2(0.5f, 0.075f));
  _editing_menu.setNumEntries(NUM_EDITING_ENTRIES);
  entryType = Menu::EDITING_TOGGLE_WIREFRAME;
  _editing_menu.setAngleOffset(angleOffsetForPosition(_editing_menu.getPosition()));
  _editing_menu.setDefaultEntry(0);
  _editing_menu.setActionsOnly(true);
  for (int i=0; i<NUM_EDITING_ENTRIES; i++) {
    Menu::MenuEntry& entry = _editing_menu.getEntry(i);
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_CONFIRM_ENTRIES = 2;
  _confirm_menu.setName("Are you sure?");
  _confirm_menu.setPosition(Vector2(0.5f, 0.45f));
  _confirm_menu.setNumEntries(NUM_CONFIRM_ENTRIES);
  entryType = Menu::CONFIRM_YES;
  _confirm_menu.setAngleOffset(angleOffsetForPosition(_confirm_menu.getPosition()));
  _confirm_menu.setDefaultEntry(0);
  _confirm_menu.setActionsOnly(true);
  _confirm_menu.setAlwaysActivated(true);
  for (int i=0; i<NUM_CONFIRM_ENTRIES; i++) {
    Menu::MenuEntry& entry = _confirm_menu.getEntry(i);
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }

  const int NUM_TUTORIAL_ENTRIES = 3;
  _tutorial_menu.setName("Tutorial");
  _tutorial_menu.setPosition(Vector2(0.65f, 0.75f));
  _tutorial_menu.setNumEntries(NUM_TUTORIAL_ENTRIES);
  entryType = Menu::TUTORIAL_NEXT;
  _tutorial_menu.setAngleOffset(static_cast<float>(M_PI) + angleOffsetForPosition(_tutorial_menu.getPosition()));
  _tutorial_menu.setDefaultEntry(0);
  _tutorial_menu.setActionsOnly(true);
  _tutorial_menu.setAlwaysActivated(true);
  for (int i=0; i<NUM_TUTORIAL_ENTRIES; i++) {
    Menu::MenuEntry& entry = _tutorial_menu.getEntry(i);
    entry.m_entryType = static_cast<Menu::MenuEntryType>(entryType++);
    entry.drawMethod = Menu::MenuEntry::STRING;
  }
}

void UserInterface::update(LeapInteraction* leap, Sculpt* sculpt) {
  static const float FORCE_SMOOTH_STRENGTH = 0.9f;
  static const float UI_INACTIVITY_FADE_TIME = 10.0f;

  std::vector<ci::Vec4f> tips = leap->getTips();
  LM_ASSERT_IDENTICAL(tips.size());
  for (unsigned i = 0; i < tips.size(); i++)
  {
     LM_ASSERT_IDENTICAL(tips[i]);
  }
  const double curTime = ci::app::getElapsedSeconds();
  LM_TRACK_CONST_VALUE(curTime);

  const float timeSinceActivity = static_cast<float>(curTime - leap->getLastActivityTime());
  LM_TRACK_CONST_VALUE(timeSinceActivity);
  const float inactivityRatio = Utilities::SmootherStep(ci::math<float>::clamp(timeSinceActivity - UI_INACTIVITY_FADE_TIME, 0.0f, UI_INACTIVITY_FADE_TIME)/UI_INACTIVITY_FADE_TIME);

  Menu::g_maxMenuActivation = ci::math<float>::clamp(inactivityRatio + maxActivation(Menu::g_forceCenter));
  if (inactivityRatio > 0.00001f || _draw_confirm_menu) {
    Menu::g_forceCenter = _confirm_menu.getPosition();
  }
  Menu::g_maxMenuActivationSmoother.Update(Menu::g_maxMenuActivation, curTime, FORCE_SMOOTH_STRENGTH);

  std::vector<Vec4f> empty;
  std::vector<Vec4f>* tipsForConfirm;
  std::vector<Vec4f>* tipsForTutorial;
  std::vector<Vec4f>* tipsForTools;
  std::vector<Vec4f>* tipsForAll;

  if (_draw_confirm_menu) {
    tipsForConfirm = &tips;
    tipsForTools = tipsForTutorial = tipsForAll = &empty;
  } else if (_draw_tutorial_menu) {
    tipsForTutorial = &tips;
    tipsForTools = _tutorial_slide == 2 ? &tips : &empty;
    tipsForConfirm = tipsForAll = &empty;
  } else {
    tipsForTools = tipsForAll = &tips;
    tipsForTutorial = tipsForConfirm = &empty;
  }

  // update the individual menus
  _type_menu.update(*tipsForTools, sculpt);
  _strength_menu.update(*tipsForAll, sculpt);
  _size_menu.update(*tipsForAll, sculpt);
  if (_draw_color_menu) {
    _color_menu.update(*tipsForAll, sculpt);
  }
  _material_menu.update(*tipsForAll, sculpt);
  _spin_menu.update(*tipsForAll, sculpt);
  _environment_menu.update(*tipsForAll, sculpt);
  _general_menu.update(*tipsForAll, sculpt);
  _object_menu.update(*tipsForAll, sculpt);
  _editing_menu.update(*tipsForAll, sculpt);
  _confirm_menu.update(*tipsForConfirm, sculpt);
  _tutorial_menu.update(*tipsForTutorial, sculpt);

  // add cursor positions
  _cursor_positions.clear();
  for (size_t i=0; i<tips.size(); i++) {
    const ci::Vec2f pos(tips[i].x, 1.0f - tips[i].y);
    _cursor_positions.push_back(pos);
  }
}

void UserInterface::draw() const {
  const float opacity = maxActivation();
  for (size_t i=0; i<_cursor_positions.size(); i++) {
    drawCursor(_cursor_positions[i], opacity*opacity);
  }

  enableAlphaBlending();
  if (_draw_confirm_menu) {
    _confirm_menu.draw();
  } else if (_draw_tutorial_menu) {
    _tutorial_menu.draw();
    if (_tutorial_slide == 2) {
      _type_menu.draw();
    }
  } else {
    _type_menu.draw();
    _strength_menu.draw();
    _size_menu.draw();
    if (_draw_color_menu) {
      _color_menu.draw();
    }
    _material_menu.draw();
    _spin_menu.draw();
    _environment_menu.draw();
    _general_menu.draw();
    _object_menu.draw();
    _editing_menu.draw();
  }
}

void UserInterface::drawTutorialSlides(float opacityMult) const {
  if (!_draw_tutorial_menu) {
    return;
  }
  static const float TUTORIAL_SCALE = 0.6f;
  static const float IMAGE_FADE_TIME = 0.5f;

  const ci::Vec2i size = getWindowSize();
  const ci::Area bounds = getWindowBounds();
  const ci::Vec2f center = getWindowCenter();
  const float yOffset = -size.y / 8.0f;

  setMatricesWindow( size );
  setViewport( bounds );
  const float aspect = _tutorial1.getAspectRatio();
  float halfWidth = TUTORIAL_SCALE*size.x/2;
  float halfHeight = halfWidth / aspect;
  ci::Rectf area(center.x - halfWidth, center.y - halfHeight + yOffset, center.x + halfWidth, center.y + halfHeight + yOffset);

  const double curTime = ci::app::getElapsedSeconds();
  const float timeSinceToggle = static_cast<float>(ci::app::getElapsedSeconds() - _last_switch_time);
  const float sculptMult = 0.1f + 0.5f * Utilities::SmootherStep(ci::math<float>::clamp(Menu::g_timeSinceSculpting/IMAGE_FADE_TIME));
  _tutorial_opacity.Update(sculptMult, curTime, 0.9f);
  const float opacity = opacityMult * _tutorial_opacity.value * Utilities::SmootherStep(ci::math<float>::clamp(timeSinceToggle/IMAGE_FADE_TIME));

  const gl::Texture* tex = 0;
  if (_tutorial_slide == 0) {
    tex = &_tutorial1;
  } else if (_tutorial_slide == 1) {
    tex = &_tutorial2;
  } else if (_tutorial_slide == 2) {
    tex = &_tutorial3;
  } else {
    tex = &_tutorial4;
  }
  glColor4f(1.0f, 1.0f, 1.0f, opacity);
  ci::gl::draw(*tex, area);
}

void UserInterface::drawDisconnected() const {
  const ci::ColorA titleColor(1.0f, 0.2f, 0.1f, 1.0f);
  const ci::ColorA shadowColor(0.1f, 0.1f, 0.1f, 1.0f);
  const ci::Vec2f offset = Vec2f(0.0f, Menu::FONT_SIZE/2.0f);
  const ci::Vec2f pos = getWindowCenter() + Vec2f(0.0f, getWindowHeight()/3.0f);

  static const std::string DISCONNECT_MESSAGE = "Leap Motion Controller is disconnected";

  glPushMatrix();
  gl::translate(pos);
  gl::scale(1.25f, 1.25f);
  const ci::Vec2f nameSize = Menu::g_boldTextureFont->measureString(DISCONNECT_MESSAGE);
  const ci::Rectf nameRect(-nameSize.x/2.0f, -offset.y, nameSize.x/2.0f, 100.0f);
  gl::color(shadowColor);
  Menu::g_boldTextureFont->drawString(DISCONNECT_MESSAGE, nameRect, Menu::g_shadowOffset);
  gl::color(titleColor);
  Menu::g_boldTextureFont->drawString(DISCONNECT_MESSAGE, nameRect);

  glPopMatrix();
}

float UserInterface::maxActivation() const {
  float result = 0.0f;

  result = std::max(result, _type_menu.getActivation());
  result = std::max(result, _strength_menu.getActivation());
  result = std::max(result, _size_menu.getActivation());
  result = std::max(result, _color_menu.getActivation());
  result = std::max(result, _material_menu.getActivation());
  result = std::max(result, _spin_menu.getActivation());
  result = std::max(result, _environment_menu.getActivation());
  result = std::max(result, _general_menu.getActivation());
  result = std::max(result, _object_menu.getActivation());
  result = std::max(result, _editing_menu.getActivation());

  return result;
}

float UserInterface::maxActivation(Vector2& pos) const {
  float max = maxActivation();
  if (max > 0.001f) {
    if (_type_menu.getActivation() == max) { pos = _type_menu.getPosition(); }
    if (_strength_menu.getActivation() == max) { pos = _strength_menu.getPosition(); }
    if (_size_menu.getActivation() == max) { pos = _size_menu.getPosition(); }
    if (_color_menu.getActivation() == max) { pos = _color_menu.getPosition(); }
    if (_material_menu.getActivation() == max) { pos = _material_menu.getPosition(); }
    if (_spin_menu.getActivation() == max) { pos = _spin_menu.getPosition(); }
    if (_environment_menu.getActivation() == max) { pos = _environment_menu.getPosition(); }
    if (_general_menu.getActivation() == max) { pos = _general_menu.getPosition(); }
    if (_object_menu.getActivation() == max) { pos = _object_menu.getPosition(); }
    if (_editing_menu.getActivation() == max) { pos = _editing_menu.getPosition(); }
  }
  return max;
}

void UserInterface::handleSelections(Sculpt* sculpt, LeapInteraction* leap, FreeformApp* app, Mesh* mesh) {
  if (_first_selection_check) {
    initializeMenu(_type_menu);
    initializeMenu(_strength_menu);
    initializeMenu(_size_menu);
    initializeMenu(_color_menu);
    initializeMenu(_material_menu);
    initializeMenu(_spin_menu);
    initializeMenu(_environment_menu);
    initializeMenu(_general_menu);
    initializeMenu(_object_menu);
    initializeMenu(_editing_menu);
    initializeMenu(_confirm_menu);
    initializeMenu(_tutorial_menu);
    _first_selection_check = false;
  }

  if (_type_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _type_menu.getSelectedEntry();
    Sculpt::SculptMode mode = entry.toSculptMode();
    sculpt->setSculptMode(mode);
    _draw_color_menu = (mode == Sculpt::PAINT);
    _type_menu.clearSelection();
  }

  if (_strength_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _strength_menu.getSelectedEntry();
    leap->setBrushStrength(entry.m_value / Menu::STRENGTH_UI_MULT);
    _strength_menu.clearSelection();
  }

  if (_size_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _size_menu.getSelectedEntry();
    leap->setBrushRadius(entry.m_value);
    _size_menu.clearSelection();
  }

  if (_draw_color_menu && _color_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _color_menu.getSelectedEntry();
    const ci::Color& desiredColor = entry.m_color;
    _color_menu.setActiveColor(desiredColor);
    Vector3 color(desiredColor.r, desiredColor.g, desiredColor.b);
    sculpt->setMaterialColor(color);
    _color_menu.clearSelection();
  }

  if (_material_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _material_menu.getSelectedEntry();
    app->setMaterial(entry.toMaterial());
    _material_menu.clearSelection();
  }

  if (_spin_menu.hasSelectedEntry() && mesh) {
    const Menu::MenuEntry& entry = _spin_menu.getSelectedEntry();
    mesh->setRotationVelocity(entry.toSpinVelocity());
    _spin_menu.clearSelection();
  }

  if (_environment_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _environment_menu.getSelectedEntry();
#if ! LM_DISABLE_THREADING_AND_ENVIRONMENT
    app->setEnvironment(entry.toEnvironmentName());
#endif
    _environment_menu.clearSelection();
  }

  if (_general_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _general_menu.getSelectedEntry();
    switch (entry.m_entryType) {
      case Menu::GENERAL_ABOUT: break;
      case Menu::GENERAL_TUTORIAL: _draw_tutorial_menu = true; _last_switch_time = ci::app::getElapsedSeconds(); break;
      case Menu::GENERAL_TOGGLE_SOUND: app->toggleSound(); break;
      case Menu::GENERAL_EXIT: _draw_confirm_menu = true; _pending_entry = Menu::GENERAL_EXIT; break;
      default: break;
    }
    _general_menu.clearSelection();
  }

  if (_object_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _object_menu.getSelectedEntry();
    switch (entry.m_entryType) {
      case Menu::OBJECT_LOAD: _draw_confirm_menu = true; _pending_entry = Menu::OBJECT_LOAD; break;
      case Menu::OBJECT_EXPORT: app->saveFile(); break;
      case Menu::OBJECT_BALL: _draw_confirm_menu = true; _pending_entry = Menu::OBJECT_BALL; break;
      case Menu::OBJECT_CAN: _draw_confirm_menu = true; _pending_entry = Menu::OBJECT_CAN; break;
      case Menu::OBJECT_DONUT: _draw_confirm_menu = true; _pending_entry = Menu::OBJECT_DONUT; break;
      case Menu::OBJECT_SHEET: _draw_confirm_menu = true; _pending_entry = Menu::OBJECT_SHEET; break;
      case Menu::OBJECT_CUBE: _draw_confirm_menu = true; _pending_entry = Menu::OBJECT_CUBE; break;
      default: break;
    }
    _object_menu.clearSelection();
  }

  if (_editing_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _editing_menu.getSelectedEntry();
    switch (entry.m_entryType) {  
      case Menu::EDITING_TOGGLE_WIREFRAME: app->toggleWireframe(); break;
      case Menu::EDITING_TOGGLE_SYMMETRY: app->toggleSymmetry(); break;
      case Menu::EDITING_UNDO: if (mesh) { mesh->undo(); } break;
      case Menu::EDITING_REDO: if (mesh) { mesh->redo(); } break;
      default: break;
    }
    _editing_menu.clearSelection();
  }

  if (_draw_confirm_menu && _confirm_menu.hasSelectedEntry()) {
    if (_confirm_menu.getSelectedEntry().m_entryType == Menu::CONFIRM_YES) {
      switch (_pending_entry) {
        case Menu::GENERAL_EXIT: app->doQuit(); break;
        case Menu::OBJECT_LOAD: app->loadFile(); break;
        case Menu::OBJECT_BALL: app->loadShape(FreeformApp::BALL); break;
        case Menu::OBJECT_CAN: app->loadShape(FreeformApp::CAN); break;
        case Menu::OBJECT_DONUT: app->loadShape(FreeformApp::DONUT); break;
        case Menu::OBJECT_SHEET: app->loadShape(FreeformApp::SHEET); break;
        case Menu::OBJECT_CUBE: app->loadShape(FreeformApp::CUBE); break;
        default: break;
      }
    }
    _confirm_menu.clearSelection();
    _draw_confirm_menu = false;
  }

  if (_draw_tutorial_menu && _tutorial_menu.hasSelectedEntry()) {
    const Menu::MenuEntry& entry = _tutorial_menu.getSelectedEntry();
    const double curTime = ci::app::getElapsedSeconds();
    switch (entry.m_entryType) {
      case Menu::TUTORIAL_CLOSE: _draw_tutorial_menu = false; break;
      case Menu::TUTORIAL_NEXT:
        _tutorial_slide = (_tutorial_slide + 1)%4;
        _last_switch_time = curTime;
        break;
      case Menu::TUTORIAL_PREVIOUS:
        _tutorial_slide = (_tutorial_slide == 0 ? 3 : _tutorial_slide-1);
        _last_switch_time = curTime;
        break;
      default: break;
    }
    _tutorial_menu.clearSelection();
  }
}

void UserInterface::drawCursor(const ci::Vec2f& position, float opacity) const {
  static const float CURSOR_RADIUS = 40.0f;
  const float bonusRadius = 40.0f * (1.0f - opacity);
  const Vector2& size = Menu::getWindowSize();
  ci::Vec2f screenPos(position.x * size.x(), position.y * size.y());
  glColor4f(0.7f, 0.9f, 1.0f, opacity);
  ci::gl::drawSolidCircle(screenPos, CURSOR_RADIUS + bonusRadius, 40);
}

void UserInterface::initializeMenu(Menu& menu) {
  const int defaultOption = menu.getDefaultEntry();
  menu.setCurSelectedEntry(menu.getActionsOnly() ? -1 : defaultOption);
  const std::string name = menu.getEntry(defaultOption).toString();
  menu.setActualName(name);
  menu.setActiveName(name);
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
  //return rotAngle;
  return edgeAngle;
}
