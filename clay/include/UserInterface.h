#ifndef __USERINTERFACE_H__
#define __USERINTERFACE_H__

#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/TextureFont.h"
#include "cinder/params/Params.h"
#include "cinder/gl/Texture.h"
#include "Environment.h"
#include "Utilities.h"
#include "Resources.h"
#include "Sculpt.h"
#include <vector>

class LeapInteraction;
class FreeformApp;

class Menu {
public:

  // the values for a particular group should be consecutive
  enum MenuEntryType {
    // quick access menu
    STRENGTH = 0,
    SIZE,
    TYPE,
    COLOR,

    // tools
    TOOL_PAINT,
    TOOL_PUSH,
    TOOL_SWEEP,
    TOOL_FLATTEN,
    TOOL_SMOOTH,
    TOOL_SHRINK,
    TOOL_GROW,

    // size
    SIZE_AUTO,

    // strength
    STRENGTH_LOW,
    STRENGTH_MEDIUM,
    STRENGTH_HIGH,

    // material
    MATERIAL_PORCELAIN,
    MATERIAL_PLASTIC,
    MATERIAL_GLASS,
    MATERIAL_METAL,
    MATERIAL_CLAY,

    // auto spin
    SPIN_OFF,
    SPIN_SLOW,
    SPIN_MEDIUM,
    SPIN_FAST,

    // environment
    ENVIRONMENT_ISLANDS,
    ENVIRONMENT_RIVER,
    ENVIRONMENT_DESERT,
    ENVIRONMENT_REDWOOD,
    ENVIRONMENT_JUNGLE_CLIFF,
    ENVIRONMENT_JUNGLE,
    ENVIRONMENT_ARCTIC,

    // general
    GENERAL_ABOUT,
    GENERAL_TUTORIAL,
    GENERAL_TOGGLE_SOUND,
    GENERAL_EXIT,

    // object
    OBJECT_LOAD,
    OBJECT_EXPORT,
    OBJECT_BALL,
    OBJECT_CAN,
    OBJECT_DONUT,
    OBJECT_SHEET,

    // editing
    EDITING_TOGGLE_WIREFRAME,
    EDITING_TOGGLE_SYMMETRY,
    EDITING_UNDO,
    EDITING_REDO,

    // confirm
    CONFIRM_YES,
    CONFIRM_NO,

    NUM_ICONS
  };

  struct MenuEntry {
    enum DrawMethod { ICON, COLOR, STRING, CIRCLE };
    MenuEntry() : m_position(Vector2::Zero()), m_radius(0.02f), m_value(0.0f) {
      m_hoverStrength.value = 0.0f;
      m_activationStrength.value = 0.0f;
    }
    void draw(const Menu* parent, bool selected) const {
      static const float ICON_ACTIVATION_BONUS_SCALE = 0.1f;
      static const float SHAPE_ACTIVATION_BONUS_SCALE = 0.5f;
      const float opacity = std::max(m_activationStrength.value, parent->m_activation.value);
      const float brightness = selected ? 1.0f : 0.5f + 0.5f*std::max(m_activationStrength.value, m_hoverStrength.value);
      const ci::Vec2f pos(m_position.x(), m_position.y());
      const ci::ColorA color(brightness, brightness, brightness, opacity);
      const ci::ColorA shadowColor(0.1f, 0.1f, 0.1f, opacity);
      gl::color(color);
      if (drawMethod == ICON) {
        ci::gl::Texture& tex = Menu::g_icons[m_entryType];
        const ci::Vec2i size = tex.getSize();
        glPushMatrix();
        glTranslatef(pos.x, pos.y, 0.0f);
        const float scale = (ICON_ACTIVATION_BONUS_SCALE*m_activationStrength.value) + parent->relativeToAbsolute(0.09f) / size.x;
        glScalef(scale, scale, scale);
        glTranslatef(-size.x/2.0f, -size.y/2.0f, 0.0f);
        if (opacity > 0.01f) {
          glColor4f(brightness, brightness, brightness, opacity);
          gl::draw(tex);
        }
        glPopMatrix();
      } else if (drawMethod == CIRCLE) {
        const float scale = 1.0f + (SHAPE_ACTIVATION_BONUS_SCALE*m_activationStrength.value);
        gl::color(shadowColor);
        gl::drawSolidCircle(pos + g_shadowOffset, parent->relativeToAbsolute(scale * m_radius), 40);
        gl::color(color);
        gl::drawSolidCircle(pos, parent->relativeToAbsolute(scale * m_radius), 40);
      } else if (drawMethod == STRING) {
        glPushMatrix();
        gl::translate(pos);
        const float scale = (SHAPE_ACTIVATION_BONUS_SCALE*m_activationStrength.value) + parent->relativeToAbsolute(0.025f) / FONT_SIZE;
        glScalef(scale, scale, scale);
        const std::string str = toString();
        const ci::Vec2f stringSize = g_textureFont->measureString(str);
        const ci::Rectf stringRect(-stringSize.x/2.0f, -Menu::FONT_SIZE/2.0f, stringSize.x/2.0f, 100.0f);
        gl::color(shadowColor);
        g_textureFont->drawString(str, stringRect, g_shadowOffset);
        gl::color(color);
        g_textureFont->drawString(str, stringRect);
        glPopMatrix();
      }
    }
    std::string toString() const {
      if (drawMethod == ICON || drawMethod == STRING) {
        switch(m_entryType) {
        case Menu::STRENGTH: return "Strength"; break;
        case Menu::SIZE: return "Size"; break;
        case Menu::TYPE: return "Tool"; break;
        case Menu::COLOR: return "Color"; break;
        case Menu::TOOL_PUSH: return "Press"; break;
        case Menu::TOOL_SWEEP: return "Smear"; break;
        case Menu::TOOL_FLATTEN: return "Flatten"; break;
        case Menu::TOOL_SMOOTH: return "Smooth"; break;
        case Menu::TOOL_SHRINK: return "Repel"; break;
        case Menu::TOOL_GROW: return "Grow"; break;
        case Menu::TOOL_PAINT: return "Paint"; break;
        case Menu::SIZE_AUTO: return "Auto"; break;
        case Menu::STRENGTH_LOW: return "Low"; break;
        case Menu::STRENGTH_MEDIUM: return "Medium"; break;
        case Menu::STRENGTH_HIGH: return "High"; break;
        case Menu::MATERIAL_PLASTIC: return "Plastic"; break;
        case Menu::MATERIAL_PORCELAIN: return "Porcelain"; break;
        case Menu::MATERIAL_GLASS: return "Glass"; break;
        case Menu::MATERIAL_METAL: return "Metal"; break;
        case Menu::MATERIAL_CLAY: return "Clay"; break;
        case Menu::SPIN_OFF: return "Off"; break;
        case Menu::SPIN_SLOW: return "Slow"; break;
        case Menu::SPIN_MEDIUM: return "Medium"; break;
        case Menu::SPIN_FAST: return "Fast"; break;
        case Menu::ENVIRONMENT_ISLANDS: return "Islands"; break;
        case Menu::ENVIRONMENT_RIVER: return "River"; break;
        case Menu::ENVIRONMENT_DESERT: return "Desert"; break;
        case Menu::ENVIRONMENT_REDWOOD: return "Redwood"; break;
        case Menu::ENVIRONMENT_JUNGLE_CLIFF: return "Jungle-Cliff"; break;
        case Menu::ENVIRONMENT_JUNGLE: return "Jungle"; break;
        case Menu::ENVIRONMENT_ARCTIC: return "Arctic"; break;
        case Menu::GENERAL_ABOUT: return "About"; break;
        case Menu::GENERAL_TUTORIAL: return "Tutorial"; break;
        case Menu::GENERAL_TOGGLE_SOUND: return "Toggle Sound"; break;
        case Menu::GENERAL_EXIT: return "Exit"; break;
        case Menu::OBJECT_LOAD: return "Load"; break;
        case Menu::OBJECT_EXPORT: return "Export"; break;
        case Menu::OBJECT_BALL: return "Sphere"; break;
        case Menu::OBJECT_CAN: return "Can"; break;
        case Menu::OBJECT_DONUT: return "Donut"; break;
        case Menu::OBJECT_SHEET: return "Sheet"; break;
        case Menu::EDITING_TOGGLE_SYMMETRY: return "Symmetry"; break;
        case Menu::EDITING_TOGGLE_WIREFRAME: return "Wireframe"; break;
        case Menu::EDITING_REDO: return "Redo"; break;
        case Menu::EDITING_UNDO: return "Undo"; break;
        case Menu::CONFIRM_YES: return "Yes"; break;
        case Menu::CONFIRM_NO: return "No"; break;
        }
        return "";
      } else {
        std::stringstream ss;
        ss << m_value;
        return ss.str();
      }
    }
    Sculpt::SculptMode toSculptMode() const {
      switch (m_entryType) {
      case Menu::TOOL_GROW: return Sculpt::INFLATE; break;
      case Menu::TOOL_SHRINK: return Sculpt::DEFLATE; break;
      case Menu::TOOL_SMOOTH: return Sculpt::SMOOTH; break;
      case Menu::TOOL_FLATTEN: return Sculpt::FLATTEN; break;
      case Menu::TOOL_SWEEP: return Sculpt::SWEEP; break;
      case Menu::TOOL_PUSH: return Sculpt::PUSH; break;
      case Menu::TOOL_PAINT: return Sculpt::PAINT; break;
      }
      return Sculpt::INVALID;
    }

    Material toMaterial() const {
      Material mat;
      switch (m_entryType) {
      case Menu::MATERIAL_PLASTIC:
        mat.reflectionFactor = 0.1f;
        mat.surfaceColor << 0.4f, 0.7f, 1.0f;
        mat.reflectionBias = 0.5f;
        break;
      case Menu::MATERIAL_PORCELAIN:
        mat.reflectionFactor = 0.1f;
        mat.surfaceColor << 1.0f, 0.95f, 0.9f;
        mat.reflectionBias = 0.5f;
        break;
      case Menu::MATERIAL_GLASS:
        mat.diffuseFactor = 0.15f;
        mat.reflectionFactor = 0.7f;
        mat.surfaceColor << 0.4f, 0.45f, 0.5f;
        mat.refractionIndex = 0.45f;
        break;
      case Menu::MATERIAL_METAL:
        mat.reflectionFactor = 0.5f;
        mat.surfaceColor << 0.2f, 0.25f, 0.275f;
        mat.reflectionBias = 2.0f;
        break;
      case Menu::MATERIAL_CLAY:
        mat.reflectionFactor = 0.02f;
        mat.reflectionBias = 3.0f;
        mat.surfaceColor << 0.7f, 0.675f, 0.6f;
        break;
      }
      return mat;
    }
    float toSpinVelocity() const {
      switch (m_entryType) {
      case Menu::SPIN_OFF: return 0.0f; break;
      case Menu::SPIN_SLOW: return 0.37f; break;
      case Menu::SPIN_MEDIUM: return 1.83f; break;
      case Menu::SPIN_FAST: return 3.27f; break;
      }
      return 0.0f;
    }
    std::string toEnvironmentName() const {
      switch (m_entryType) {
      case Menu::ENVIRONMENT_ISLANDS: return "Islands"; break;
      case Menu::ENVIRONMENT_RIVER: return "River"; break;
      case Menu::ENVIRONMENT_DESERT: return "Desert"; break;
      case Menu::ENVIRONMENT_REDWOOD: return "Redwood"; break;
      case Menu::ENVIRONMENT_JUNGLE_CLIFF: return "Jungle-Cliff"; break;
      case Menu::ENVIRONMENT_JUNGLE: return "Jungle"; break;
      case Menu::ENVIRONMENT_ARCTIC: return "Arctic"; break;
      }
      return "";
    }

    MenuEntryType m_entryType;
    DrawMethod drawMethod;
    float m_radius;
    float m_value;
    Vector2 m_position;
    float m_angleStart;
    float m_angleWidth;
    float m_wedgeStart;
    float m_wedgeEnd;
    ci::Color m_color;
    Utilities::ExponentialFilter<float> m_hoverStrength;
    Utilities::ExponentialFilter<float> m_activationStrength;
  };

  static const float FONT_SIZE;
  static const float STRENGTH_UI_MULT;
  static ci::Font g_font;
  static ci::Font g_boldFont;
  static ci::gl::TextureFontRef g_textureFont;
  static ci::gl::TextureFontRef g_boldTextureFont;
  static std::vector<ci::gl::Texture> g_icons;
  static ci::Vec2f g_shadowOffset;
  static float g_zoomFactor;
  static float g_maxMenuActivation;
  static Utilities::ExponentialFilter<float> g_maxMenuActivationSmoother;
  static Vector2 g_forceCenter;

  Menu();
  void update(const std::vector<Vec4f>& tips, Sculpt* sculpt);
  void setNumEntries(int num);
  void draw() const;
  int checkCollision(const Vector2& pos) const;
  void toRadialCoordinates(const Vector2& pos, float& radius, float& angle) const;

  float getActivation() const { return m_activation.value; }
  bool hasSelectedEntry() const { return m_haveSelection; }
  void clearSelection() { m_haveSelection = false; }
  static void setWindowSize(const ci::Vec2i& size);
  float getSweepStart() const { return static_cast<float>(-m_sweepAngle/2.0f + m_angleOffset); }
  float getSweepEnd() const { return getSweepStart() + m_sweepAngle; }
  MenuEntry& getEntry(int idx) { return m_entries[idx]; }
  const MenuEntry& getEntry(int idx) const { return m_entries[idx]; }
  const MenuEntry& getSelectedEntry() const { return m_entries[m_curSelectedEntry]; }
  float relativeToAbsolute(float radius) const { return radius * g_windowSize.y(); }
  float absoluteToRelative(float radius) const { return radius / g_windowSize.y(); }
  float getRingRadius() const { return 0.75f * m_outerRadius; }
  void setName(const std::string& name) { m_name = name; }
  void setPosition(const Vector2& pos) { m_position = pos; }
  const Vector2& getPosition() const { return m_position; }
  void setAngleOffset(float angleOffset) { m_angleOffset = angleOffset; }
  void setDefaultEntry(int idx) { m_defaultEntry = idx; }
  int getDefaultEntry() const { return m_defaultEntry; }
  void setActionsOnly(bool only) { m_actionsOnly = only; }
  bool getActionsOnly() const { return m_actionsOnly; }
  void setCurSelectedEntry(int idx) { m_curSelectedEntry = idx; m_haveSelection = (idx >= 0); }
  void setActualName(const std::string& name) { m_actualName = name; }
  void setActiveName(const std::string& name) { m_activeName = name; }
  void setActiveColor(const ci::Color& color) { m_activeColor = color; }
  ci::Color getActiveColor() const { return m_activeColor; }

  static const Vector2& getWindowSize() { return g_windowSize; }
  static void updateSculptMult(double curTime, float mult) { g_sculptMult.Update(mult, curTime, 0.975f); }

private:

  static float getOpacity(float activation) {
    static const float NORMAL_OPACITY = 0.5f;
    static const float ACTIVATED_OPACITY = 1.0f;
    return g_sculptMult.value * (NORMAL_OPACITY + (ACTIVATED_OPACITY-NORMAL_OPACITY)*activation);
  }

  ci::Vec2f relativeToAbsolute(const Vector2& pos, bool useZoom = true) const {
    Vector2 scaled = pos;
    if (useZoom) {
      scaled = g_zoomFactor*(scaled - Vector2::Constant(0.5f)) + Vector2::Constant(0.5f);
    }
    //scaled.x() = (scaled.x() - 0.5f)/m_windowAspect + 0.5f;
    //scaled.y() = (scaled.y() - 0.5f)*m_windowAspect + 0.5f;
    scaled = scaled.cwiseProduct(g_windowSize);
    return ci::Vec2f(scaled.data());
  }

  static Vector2 forceAt(const Vector2& pos) {
    static const float FORCE_STRENGTH = 0.05f;
    const Vector2 diff = pos - g_forceCenter;
    const float normSq = diff.squaredNorm();
    if (normSq > 0.001f) {
      return g_maxMenuActivationSmoother.value*FORCE_STRENGTH*(diff/normSq);
    }
    return Vector2::Zero();
  }

  static const float RING_THICKNESS_RATIO;
  static const float BASE_OUTER_RADIUS;
  static const float BASE_INNER_RADIUS;
  static const float OUTER_RADIUS_PER_ENTRY;
  static const float SWEEP_ANGLE;

  typedef std::vector<MenuEntry, Eigen::aligned_allocator<MenuEntry> > MenuEntryVector;

  Utilities::ExponentialFilter<float> m_activation;
  float m_outerRadius;
  float m_innerRadius;
  float m_angleOffset;
  float m_sweepAngle;
  MenuEntryVector m_entries;
  int m_curSelectedEntry;
  int m_prevSelectedEntry;
  double m_selectionTime;
  double m_deselectTime;
  Vector2 m_position;
  std::string m_name;
  float m_wedgeStart;
  float m_wedgeEnd;
  bool m_haveSelection;

  Menu::MenuEntryType iconType;
  std::string m_activeName;
  std::string m_actualName;
  ci::Color m_activeColor;
  int m_defaultEntry;
  bool m_actionsOnly;

  static Vector2 g_windowSize;
  static float g_windowDiagonal;
  static float g_windowAspect;
  static Vector2 g_windowCenter;
  static Utilities::ExponentialFilter<float> g_sculptMult;

};

class UserInterface {

public:

  UserInterface();
  void update(const std::vector<Vec4f>& tips, Sculpt* sculpt);
  void draw() const;
  void setWindowSize(const Vec2i& size) { Menu::setWindowSize(size); }
  float maxActivation() const;
  float maxActivation(Vector2& pos) const;
  void handleSelections(Sculpt* sculpt, LeapInteraction* leap, FreeformApp* app, Mesh* mesh);
  void setRegularFont(const ci::Font& font) {
    Menu::g_font = font;
    Menu::g_textureFont = ci::gl::TextureFont::create(font);
  }
  void setBoldFont(const ci::Font& font) {
    Menu::g_boldFont = font;
    Menu::g_boldTextureFont = ci::gl::TextureFont::create(font);
  }
  void drawCursor(const ci::Vec2f& position, float opacity) const;
  void setZoomFactor(float zoom) { Menu::g_zoomFactor = zoom; }
  float getZoomFactor() const { return Menu::g_zoomFactor; }

private:

  void initializeMenu(Menu& menu);

  static float angleOffsetForPosition(const Vector2& pos);

  std::vector<ci::Vec2f> _cursor_positions;

  Menu _type_menu;
  Menu _strength_menu;
  Menu _size_menu;
  Menu _color_menu;
  Menu _material_menu;
  Menu _spin_menu;
  Menu _environment_menu;
  Menu _general_menu;
  Menu _object_menu;
  Menu _editing_menu;
  Menu _confirm_menu;

  bool _draw_color_menu;
  bool _draw_confirm_menu;
  bool _first_selection_check;

};

#endif
