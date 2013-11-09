#ifndef __ThreeForm_h__
#define __ThreeForm_h__

#include "cinder/app/AppNative.h"
#include "cinder/params/Params.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Fbo.h"
#include "Resources.h"
#include "Environment.h"
#include "UserInterface.h"
#include "LeapListener.h"
#include "Leap.h"
#include "LeapInteraction.h"
#include "Utilities.h"
#include "cinder/Thread.h"
#include "Mesh.h"
#include "Sculpt.h"
#include "CameraUtil.h"

using namespace ci;
using namespace ci::gl;
using namespace ci::app;

class CameraUtil;
class DebugDrawUtil;

class ThreeFormApp : public AppNative
{
public:

  ThreeFormApp();
  ~ThreeFormApp();
  void prepareSettings(Settings *settings);
  void toggleFullscreen(const std::string& str);
  void setup();
  void shutdown();
  void resize();
  void mouseDown( MouseEvent event );
  void mouseUp( MouseEvent event );
  void mouseDrag( MouseEvent event );
  void mouseWheel( MouseEvent event );
  void mouseMove( MouseEvent event);
  void keyDown( KeyEvent event );
  void updateCamera(const float dTheta, const float dPhi, const float dFov);
  void update();
  void updateLeapAndMesh();
  void renderSceneToFbo(Camera& camera);
  void createBloom();
  void draw();
  void loadIcons();

  void setMaterial(const Material& mat);
  void setWireframe(bool wireframe);
  void setSymmetry(bool symmetry);
  void setEnvironment(const std::string& str);
  void setTimeOfDay(Environment::TimeOfDay time);
  int loadFile();
  int saveFile(const std::string& extension);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  // *** camera stuff ***
  CameraPersp _camera;
  float _theta;
  float _phi;
  float _fov;
  float _cam_dist;
  float _fov_modifier;

  // Free-floating camera utility.
  CameraUtil* _camera_util;

  // **** mouse stuff ***
  Vec2i _initial_mouse_pos, _current_mouse_pos, _previous_mouse_pos;
  bool _mouse_down;

  // *** scene stuff ***
  Environment* _environment;
  GlslProg _sky_shader;
  GlslProg _material_shader;
  GlslProg _brush_shader;
  GlslProg _blur_shader;
  GlslProg _wireframe_shader;
  Fbo _color_fbo;
  Fbo _depth_fbo;
  Fbo _blur_fbo;
  std::thread _loading_thread;
  std::thread _mesh_thread;
  bool _shutdown;
  Utilities::FPSCounter _mesh_update_counter;
  Vector3 _focus_point;
  float _focus_radius;
  double _last_update_time;
  Utilities::ExponentialFilter<float> _focus_opacity_smoother;

  // *** Leap stuff ***
  LeapListener _listener;
  Leap::Controller _controller;
  LeapInteraction* _leap_interaction;
  Utilities::ExponentialFilter<ci::Vec3f> _campos_smoother;
  Utilities::ExponentialFilter<ci::Vec3f> _lookat_smoother;
  Utilities::ExponentialFilter<ci::Vec3f> _up_smoother;

  // *** ui stuff ***
  params::InterfaceGlRef _params;
  Color _brush_color;
  bool _draw_edges;
  Material _material;
  bool _use_ao;
  bool _only_ao;
  std::string _last_loaded_file;
  float _ui_zoom;

  UserInterface* _ui;
  bool _draw_ui;

  Fbo _screen_fbo;
  Fbo _horizontal_blur_fbo;
  Fbo _vertical_blur_fbo;
  GlslProg _screen_shader;
  GlslProg _fxaa_shader;
  GlslProg _bloom_shader;
  bool _use_fxaa;
  float _exposure; // hdr exposure
  float _contrast;
  bool _bloom_visible;
  float _bloom_size;
  float _bloom_strength;
  float _bloom_light_threshold;
  bool _draw_background;

  // new mesh
  Mesh* mesh_;
  Sculpt sculpt_;
  bool symmetry_;
  float sumDisplacement_;
  bool sculptStart_;
  bool drawOctree_;

  // camera control settings
  CameraUtil::Params _camera_params;

  // Debug drawing utility;
  DebugDrawUtil* _debug_draw_util;

};

#endif
