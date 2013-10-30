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
using namespace std;

class CameraUtil;
class DebugDrawUtil;

class ClayDemoApp : public AppNative 
{
public:

  ClayDemoApp();
  ~ClayDemoApp();
  void prepareSettings( Settings *settings );
  int loadFile();
  int saveFile();
  void setBrushMode(const std::string& str);
  void setBrushSize(const std::string& str);
  void setBrushStrength(const std::string& str);
  void setEnvironment(const std::string& str);
  void setTimeOfDay(const std::string& str);
  void setMaterial(const std::string& str);
  void performFileAction(const std::string& str);
  void toggleFullscreen(const std::string& str);
  void toggleWireframe(const std::string& str);
  void setAutoSpin(const std::string& str);
  void setup();
  void shutdown();
  void resize();
  void mouseDown( MouseEvent event );
  void mouseUp( MouseEvent event );
  void mouseDrag( MouseEvent event );
  void mouseWheel( MouseEvent event );
  void mouseMove( MouseEvent event);
  void keyDown( KeyEvent event );
  void updateCamera(const float _DTheta,const float _DPhi,const float _DFov);
  void update();
  void updateLeapAndMesh();
  void renderSceneToFbo(Camera& _Camera);
  void createBloom();
  void draw();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  // *** camera stuff ***
  CameraPersp _camera;
  float _theta;
  float _phi;
  float _fov;
  float _cam_dist;

  // Free-floating camera utility.
  CameraUtil* _camera_util;

  // **** mouse stuff ***
  Vec2i _initial_mouse_pos, _current_mouse_pos, _previous_mouse_pos;
  bool _mouse_down;

  // *** scene stuff ***
  Environment* _environment;
  GlslProg _sky_shader;
  GlslProg _material_shader;
  GlslProg _blur_shader;
  GlslProg _wireframe_shader;
  Fbo _color_fbo;
  Fbo _depth_fbo;
  Fbo _blur_fbo;
  boost::thread _loading_thread;
  boost::thread _mesh_thread;
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
  float _ambient_factor;
  float _diffuse_factor;
  float _reflection_factor;
  Color _surface_color;
  Color _brush_color;
  bool _draw_edges;
  float _reflection_bias;
  float _refraction_bias;
  float _refraction_index;
  bool _use_ao;
  bool _only_ao;

  UserInterface* _ui;
  GlslProg _metaball_shader;
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
  bool _draw_depth_of_field;
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
