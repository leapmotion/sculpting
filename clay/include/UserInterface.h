#ifndef __USERINTERFACE_H__
#define __USERINTERFACE_H__

#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/params/Params.h"
#include "cinder/Thread.h"
//#include "cinder/svg/Svg.h"
//#include "cinder/svg/SvgGl.h"
#include "cinder/gl/Texture.h"
#include "Environment.h"
#include "Utilities.h"
#include "Resources.h"
#include "Sculpt.h"
#include <boost/function.hpp>
#include <vector>
using namespace ci;
using namespace ci::gl;

class LeapInteraction;

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



class Menu {
public:

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

    NUM_ICONS
  };

  Menu();
  void update(const std::vector<Vec4f>& tips, Sculpt* sculpt);
  void setNumEntries(int num);
  void draw() const;
  int checkCollision(const Vector2& pos) const;
  bool checkPadCollision(const Vector2& pos) const;
  void toRadialCoordinates(const Vector2& pos, float& radius, float& angle) const;

  bool isActivated() const { return m_activation.value >= 1.0f; }
  void setWindowSize(const ci::Vec2i& size);

//private:

  struct MenuEntry {
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
      glColor4f(brightness, brightness, brightness, opacity);
      if (useIcon) {
        ci::gl::Texture& tex = Menu::m_icons[m_iconType];
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
      } else if (!useColor) {
        const float scale = 1.0f + (SHAPE_ACTIVATION_BONUS_SCALE*m_activationStrength.value);
        gl::drawSolidCircle(pos, parent->relativeToAbsolute(scale * m_radius), 40);
      }
    }
    std::string toString() const {
      if (useIcon) {
        switch(m_iconType) {
        case Menu::STRENGTH: return "Strength"; break;
        case Menu::SIZE: return "Size"; break;
        case Menu::TYPE: return "Tool"; break;
        case Menu::COLOR: return "Color"; break;
        case Menu::TOOL_PUSH: return "Press"; break;
        case Menu::TOOL_SWEEP: return "Smear"; break;
        case Menu::TOOL_FLATTEN: return "Flatten"; break;
        case Menu::TOOL_SMOOTH: return "Smooth"; break;
        case Menu::TOOL_SHRINK: return "Repel"; break;
        case Menu::TOOL_GROW: return "Inflate"; break;
        case Menu::TOOL_PAINT: return "Paint"; break;
        case Menu::SIZE_AUTO: return "Auto"; break;
        case Menu::STRENGTH_LOW: return "Low"; break;
        case Menu::STRENGTH_MEDIUM: return "Medium"; break;
        case Menu::STRENGTH_HIGH: return "High"; break;
        }
        return "";
      } else {
        std::stringstream ss;
        ss << m_value;
        return ss.str();
      }
    }

    MenuEntryType m_iconType;
    bool useIcon;
    bool useColor;
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

  float getSweepStart() const { return static_cast<float>(-m_sweepAngle/2.0f + m_angleOffset); }
  float getSweepEnd() const { return getSweepStart() + m_sweepAngle; }
  static float getOpacity(float activation) {
    static const float NORMAL_OPACITY = 0.4f;
    static const float ACTIVATED_OPACITY = 1.0f;
    return NORMAL_OPACITY + (ACTIVATED_OPACITY-NORMAL_OPACITY)*activation;
  }
  float getRingRadius() const { return 0.75f * m_outerRadius; }
  ci::Vec2f relativeToAbsolute(const Vector2& pos) const {
    Vector2 scaled = pos;
    //scaled.x() = (scaled.x() - 0.5f)/m_windowAspect + 0.5f;
    //scaled.y() = (scaled.y() - 0.5f)*m_windowAspect + 0.5f;
    scaled = scaled.cwiseProduct(m_windowSize);
    return ci::Vec2f(scaled.data());
  }
  float relativeToAbsolute(float radius) const {
    //return m_windowDiagonal * radius;
    return radius * m_windowSize.y();
  }
  float absoluteToRelative(float radius) const {
    return radius / m_windowSize.y();
  }
  static Sculpt::SculptMode toolToSculptMode(Menu::MenuEntryType entry) {
    switch (entry) {
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

  static const float FONT_SIZE;
  static const float RING_THICKNESS_RATIO;
  static const float STRENGTH_UI_MULT;
  
  Utilities::ExponentialFilter<float> m_activation;
  float m_outerRadius;
  float m_innerRadius;
  float m_angleOffset;
  float m_sweepAngle;
  std::vector<MenuEntry> m_entries;
  int m_curSelectedEntry;
  int m_prevSelectedEntry;
  double m_selectionTime;
  double m_deselectTime;
  Vector2 m_position;
  std::string m_name;
  static ci::Font m_font;
  static ci::Font m_boldFont;
  float m_wedgeStart;
  float m_wedgeEnd;

  MenuEntryType iconType;
  Vector2 m_windowSize;
  float m_windowDiagonal;
  float m_windowAspect;
  Vector2 m_windowCenter;
  //static std::vector<ci::svg::DocRef> m_icons;
  static std::vector<ci::gl::Texture> m_icons;
  std::string m_activeName;
  std::string m_actualName;
  ci::Color m_activeColor;
  int m_defaultEntry;

};

class UserInterface
{

public:

  UserInterface();
  void addElement(const UIElement& _Element, const std::string& _ParentName = "");
  void addConnection(const std::string& _First, const std::string& _Second);
  void update(const std::vector<Vec4f>& _Tips, Sculpt* sculpt);
  void draw(Environment* _Env, const Matrix33f& normalMatrix) const;
  void setWindowSize(const Vec2i& _Size);
  void setShader(GlslProg* _Shader);
  void setRootNode(const std::string& _Name);
  float maxActivation() const;
  void handleSelections(Sculpt* sculpt, LeapInteraction* leap);
  void setRegularFont(const ci::Font& font) { Menu::m_font = font; }
  void setBoldFont(const ci::Font& font) { Menu::m_boldFont = font; }

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

  Menu _type_menu;
  Menu _strength_menu;
  Menu _size_menu;
  Menu _color_menu;
  bool _draw_color_menu;
  bool _first_selection_check;

  static const ColorA LEAF_COLOR;
  static const ColorA NORMAL_COLOR;
  static const ColorA LEAF_ACTIVATED_COLOR;
  static const ColorA NORMAL_ACTIVATED_COLOR;
  static const float RADIUS_FALLOFF;

};

#endif
