#ifndef __Freeform_h__
#define __Freeform_h__

#include "cinder/app/AppNative.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Fbo.h"
#if defined(CINDER_COCOA)
#include <sstream>
#include <boost/uuid/sha1.hpp>
#include <mach-o/getsect.h>
#include <mach-o/dyld.h>
#endif
#include "Resources.h"
#include "CubeMapManager.h"
#include "UserInterface.h"
#include "LeapListener.h"
#include "Leap.h"
#include "LeapInteraction.h"
#include "Utilities.h"
#include "cinder/Thread.h"
#include "Mesh.h"
#include "Sculpt.h"
#include "AutoSave.h"
#include "OrbiterCamera.h"

#define IRRKLANG_STATIC
#include <irrklang.h>

using namespace ci;
using namespace ci::gl;
using namespace ci::app;


#if LM_PRODUCTION_BUILD
#define LM_DISABLE_THREADING_AND_ENVIRONMENT 0
#else
#define LM_DISABLE_THREADING_AND_ENVIRONMENT 0
#endif

class FreeformApp : public AppNative
{
public:

  enum AAMode { NONE, MSAA };
  enum Shape { BALL, CAN, DONUT, SHEET, CUBE, SNOWMAN, NUM_SHAPES };

  FreeformApp();
  ~FreeformApp();
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
  void update();
  void updateLeapAndMesh();
  void renderSceneToFbo(Camera& camera);
  void createBloom();
  void draw();

  void setMaterial(const Material& mat);
  void setWireframe(bool wireframe);
  void toggleWireframe();
  void toggleSymmetry();
  void setEnvironment(const std::string& str);
  void toggleSound();
  int loadFile();
  int saveFile();
  int saveScreenshot();
  int loadShape(Shape shape);
  void print3D();

  void doQuit();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  float checkEnvironmentLoading();
  void loadIcons();
  void loadShapes();
  void loadImages();
  void loadSounds();
  irrklang::ISound* createSoundResource(ci::DataSourceRef ref, const char* name);

#if defined(CINDER_COCOA)
  ci::DataSourceRef loadResource(const std::string& macPath) {
    try {
      return ci::app::loadResource(macPath);
    } catch (...) {
      boost::uuids::detail::sha1 sha1;
      uint32_t hash[5];
      sha1.process_bytes(macPath.data(), macPath.size());
      sha1.get_digest(hash);
      std::ostringstream oss;
      oss << std::setfill('0') << std::setw(sizeof(hash[0])*2) << std::hex;
      for (size_t i = 0; i < sizeof(hash)/sizeof(hash[0]); i++) {
        oss << hash[i];
      }
      const std::string symbol = "__" + oss.str().substr(0, 14); // Max 16 characters long
      const auto* sect = getsectbyname("__DATA", symbol.c_str());
      
      if (sect != nullptr) {
        const auto size = sect->size;
        const auto addr = _dyld_get_image_vmaddr_slide(0) + sect->addr;
        if (size > 0 && addr != 0) {
          return ci::DataSourceBuffer::create(ci::Buffer(reinterpret_cast<void*>(addr), size));
        }
      }
      throw;
    }
  }
#endif
  fs::path getSaveFilePathCustom(const fs::path &initialPath = "",
                                const std::vector<std::string>& extensions = std::vector<std::string>(),
                                const std::vector<std::string>& descriptions = std::vector<std::string>());

  enum MachineSpeed { LOW, MID, HIGH };
  MachineSpeed parseRenderString(const std::string& render_string);

  // *** camera stuff ***
  OrbiterCamera m_camera;

  // **** mouse stuff ***
  Vec2i _initial_mouse_pos, _current_mouse_pos, _previous_mouse_pos;
  bool _mouse_down;

  // *** scene stuff ***
  AAMode _aa_mode;
  CubeMapManager* _environment;
  GlslProg _sky_shader;
  GlslProg _material_shader;
  GlslProg _brush_shader;
  GlslProg _blur_shader;
  GlslProg _wireframe_shader;
  std::thread _loading_thread;
  std::thread _mesh_thread;
  bool _shutdown;
  Utilities::FPSCounter _mesh_update_counter;
  double _last_update_time;
  double _last_load_time;
  Utilities::ExponentialFilter<float> _focus_opacity_smoother;
  std::mutex _mesh_mutex;
  std::mutex _mesh_update_rotation_mutex;
  MachineSpeed _machine_speed;
  bool _lock_camera;
  AutoSave _auto_save;
  bool _first_environment_load;
  bool _have_shaders;
  std::string _screenshot_path;

  // *** Leap stuff ***
  LeapListener _listener;
  Leap::Controller _controller;
  LeapInteraction* _leap_interaction;
  

  // *** ui stuff ***
  Color _brush_color;
  bool _draw_edges;
  Material _material;
  ci::gl::Texture _logo_on_black;
  ci::gl::Texture _logo_on_image;
  bool _immersive_mode;
  bool _have_entered_immersive;
  double _immersive_changed_time;
  double _immersive_entered_time;
  Utilities::ExponentialFilter<float> _ui_zoom;

  UserInterface* _ui;
  bool _draw_ui;

  Fbo _screen_fbo;
  Fbo _horizontal_blur_fbo;
  Fbo _vertical_blur_fbo;
  GlslProg _screen_shader;
  GlslProg _bloom_shader;
  float _exposure; // hdr exposure
  bool _bloom_visible;
  float _bloom_size;
  float _bloom_strength;
  float _bloom_light_threshold;
  bool _draw_background;

  // audio stuff
  bool _have_audio;
  bool _audio_paused;
  std::vector<ci::DataSourceRef> m_audioSourceRefs;
  typedef std::pair<irrklang::ISound*, irrklang::ISound*> LoopPair;
  irrklang::ISoundEngine* m_soundEngine;
  LoopPair m_activeLoop;
  std::map<std::string, LoopPair> m_audioLoops;

  // new mesh
  Mesh* _mesh;
  Sculpt _sculpt;
  bool drawOctree_;
  std::string shapes_[NUM_SHAPES];
  float remeshRadius_;

};

#endif
