#ifndef __USERINTERFACE_H__
#define __USERINTERFACE_H__

#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/TextureFont.h"
#include "cinder/params/Params.h"
#include "cinder/gl/Texture.h"
#include "CubeMapManager.h"
#include "Utilities.h"
#include "Resources.h"
#include "Sculpt.h"
#include <vector>

class LeapInteraction;
class FreeformApp;
class Menu;

struct MenuEntry {

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
    GENERAL_TUTORIAL,
    GENERAL_ABOUT,
    GENERAL_EXIT,
    GENERAL_SCREENSHOT,
    GENERAL_TOGGLE_SOUND,

    // object
    OBJECT_LOAD,
    OBJECT_EXPORT,
    OBJECT_UPLOAD,
    OBJECT_BALL,
    OBJECT_CAN,
    OBJECT_DONUT,
    OBJECT_SHEET,
    OBJECT_SNOWMAN,
    OBJECT_CUBE,

    // editing
    EDITING_TOGGLE_WIREFRAME,
    EDITING_TOGGLE_SYMMETRY,
    EDITING_UNDO,
    EDITING_REDO,

    // confirm
    CONFIRM_YES,
    CONFIRM_NO,

    // tutorial
    TUTORIAL_NEXT,
    TUTORIAL_CLOSE,
    TUTORIAL_PREVIOUS,

    // about
    ABOUT_CLOSE,

    NUM_ICONS
  };

  enum DrawMethod { METHOD_ICON, METHOD_COLOR, METHOD_STRING, METHOD_CIRCLE, METHOD_TEXTURE };

  MenuEntry() : m_position(Vector2::Zero()), m_radius(0.02f), m_value(0.0f) {
    m_hoverStrength.value = 0.0f;
    m_activationStrength.value = 0.0f;
  }
  void draw(const Menu* parent, bool selected) const;
  Sculpt::SculptMode toSculptMode() const;
  std::string toString() const;
  Material toMaterial() const;
  float toSpinVelocity() const;
  std::string toEnvironmentName() const;

  MenuEntryType m_entryType;
  DrawMethod m_drawMethod;
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

class Menu {
public:
  Menu();

  void update(const std::vector<Vec4f>& tips, Sculpt* sculpt);
  void draw() const;
  
private:
  //Some of these are helpers, others are only ever used by UserInterface
  void setNumEntries(int num);
  int checkCollision(const Vector2& pos) const;
  bool hasSelectedEntry() const { return m_haveSelection; }
  float getActivation() const { return m_activation.value; }
  void clearSelection() { m_haveSelection = false; }
  void toRadialCoordinates(const Vector2& pos, float& radius, float& angle) const;
  float getSweepStart() const { return static_cast<float>(-m_sweepAngle / 2.0f + m_angleOffset); }
  float getSweepEnd() const { return getSweepStart() + m_sweepAngle; }
  MenuEntry& getEntry(int idx) { return m_entries[idx]; }
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
  void setAlwaysActivated(bool activated) { m_alwaysActivated = activated; }
  
  ci::Vec2f relativeToAbsolute(const Vector2& pos, bool useZoom = true) const {
    Vector2 scaled = pos;
    if (useZoom) {
      scaled = g_zoomFactor*(scaled - Vector2::Constant(0.5f)) + Vector2::Constant(0.5f);
    }
    scaled = scaled.cwiseProduct(g_windowSize);
    return ci::Vec2f(scaled.data());
  }

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
  float m_prevClosest;
  bool m_alwaysActivated;

  std::string m_activeName;
  std::string m_actualName;
  ci::Color m_activeColor;
  int m_defaultEntry;
  bool m_actionsOnly;

  friend struct MenuEntry;
  friend class UserInterface;

public:
  static void setWindowSize(const ci::Vec2i& size);
  static const Vector2& getWindowSize() { return g_windowSize; }
  static void updateSculptMult(double curTime, float mult) { g_sculptMult.Update(mult, curTime, 0.985f); }

  static const float FONT_SIZE;
  static std::vector<ci::gl::Texture> g_icons;
  static std::vector<ci::gl::Texture> g_previews;
  static ci::gl::GlslProg g_previewShader;

private:

  static float getOpacity(float activation) {
    static const float NORMAL_OPACITY = 0.6f;
    static const float ACTIVATED_OPACITY = 1.0f;
    return g_sculptMult.value * (NORMAL_OPACITY + (ACTIVATED_OPACITY - NORMAL_OPACITY)*activation);
  }

  static Vector2 forceAt(const Vector2& pos) {
    static const float FORCE_STRENGTH = 0.05f;
    const Vector2 diff = pos - g_forceCenter;
    const float normSq = diff.squaredNorm();
    if (normSq > 0.001f) {
      return g_maxMenuActivationSmoother.value*FORCE_STRENGTH*(diff / normSq);
    }
    return Vector2::Zero();
  }

  static ci::gl::TextureFontRef g_textureFont;
  static ci::gl::TextureFontRef g_boldTextureFont;

  static ci::Vec2f g_shadowOffset;
  static float g_zoomFactor;
  static float g_maxMenuActivation;
  static Utilities::ExponentialFilter<float> g_maxMenuActivationSmoother;
  static Vector2 g_forceCenter;
  static float g_timeSinceSculpting;
  static float g_menuOpacityCap;

  static Vector2 g_windowSize;
  static float g_windowDiagonal;
  static float g_windowAspect;
  static Vector2 g_windowCenter;
  static Utilities::ExponentialFilter<float> g_sculptMult;

  static const float RING_THICKNESS_RATIO;
  static const float BASE_OUTER_RADIUS;
  static const float BASE_INNER_RADIUS;
  static const float OUTER_RADIUS_PER_ENTRY;
  static const float SWEEP_ANGLE;
};

class UserInterface {

public:

  UserInterface();
  void update(LeapInteraction* leap, Sculpt* sculpt);
  void draw(float overallOpacity) const;

  void drawTutorialSlides(float opacityMult) const;
  void drawAbout(float opacityMult) const;
  void drawError(const std::string& message, int errorNum) const;
  void drawImmersive(float opacityMult) const;
  
  float maxActivation() const;
  float maxActivation(Vector2& pos) const;
  void handleSelections(Sculpt* sculpt, LeapInteraction* leap, FreeformApp* app, Mesh* mesh);
  void showConfirm(MenuEntry::MenuEntryType entryType);
  bool haveExitConfirm() const;
  void clearConfirm();

  void setRegularFont(const ci::Font& font) {
    Menu::g_textureFont = ci::gl::TextureFont::create(font);
  }
  void setBoldFont(const ci::Font& font) {
    Menu::g_boldTextureFont = ci::gl::TextureFont::create(font);
  }

  void drawCursor(const ci::Vec2f& position, float opacity) const;
  void setWindowSize(const Vec2i& size) { Menu::setWindowSize(size); }
  void setTutorialTextures(ci::gl::Texture tutorial1, ci::gl::Texture tutorial2, ci::gl::Texture tutorial3, ci::gl::Texture tutorial4) {
    _tutorial1 = tutorial1;
    _tutorial2 = tutorial2;
    _tutorial3 = tutorial3;
    _tutorial4 = tutorial4;
  }

  bool tutorialActive() const { return _draw_tutorial_menu; }
  bool toolsSlideActive() const { return _tutorial_slide == 2; }
  void forceDrawTutorialMenu() { _draw_tutorial_menu = true; _last_switch_time = ci::app::getElapsedSeconds(); }
  void setAboutTexture(ci::gl::Texture about) { _about = about; }
  bool aboutActive() const { return _draw_about_menu; }

  void setZoomFactor(float zoom) { Menu::g_zoomFactor = zoom; }

private:

  void initializeMenu(Menu& menu);
  std::string tutorialStringSuffix(int num, int total) {
    std::stringstream ss;
    ss << num << "/" << total;
    return ss.str();
  }

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
  Menu _about_menu;

  bool _draw_color_menu;
  bool _draw_confirm_menu;
  bool _first_selection_check;

  ci::gl::Texture _tutorial1;
  ci::gl::Texture _tutorial2;
  ci::gl::Texture _tutorial3;
  ci::gl::Texture _tutorial4;
  double _last_switch_time;
  int _tutorial_slide;
  bool _draw_tutorial_menu;
  Menu _tutorial_menu;
  mutable Utilities::ExponentialFilter<float> _tutorial_opacity;

  ci::gl::Texture _about;
  bool _draw_about_menu;

  MenuEntry::MenuEntryType _pending_entry;

  
};

#endif
