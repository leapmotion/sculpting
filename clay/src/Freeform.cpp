#include "StdAfx.h"
#include "Freeform.h"
#include "Files.h"
#include "Print3D.h"
#include <time.h>

#include "CameraUtil.h"
#include "DebugDrawUtil.h"
#include "ReplayUtil.h"

#include <boost/date_time/posix_time/posix_time.hpp>

const float CAMERA_SPEED = 0.005f;

const float MIN_CAMERA_DIST = 250.0f;
const float MAX_CAMERA_DIST = 350.0f;
const float SPHERE_RADIUS = 50000.0f;
const float MIN_FOV = 50.0f;
const float MAX_FOV = 90.0f;


//*********************************************************
FreeformApp::FreeformApp() : 
_environment(0), 
_aa_mode(MSAA),
_draw_ui(true), 
_mouse_down(false),
_exposure(1.0f),
mesh_(0), 
_last_update_time(0.0),
drawOctree_(false),
_shutdown(false),
_draw_background(true), 
_focus_point(Vector3::Zero()), 
remeshRadius_(100.0f),
_lock_camera(false),
_last_load_time(0.0), 
_first_environment_load(true), 
_have_shaders(true), 
_have_entered_immersive(false),
_immersive_changed_time(0.0), 
_have_audio(true), 
m_activeLoop(nullptr, nullptr), 
_audio_paused(false),
_immersive_mode(false),
_immersive_entered_time(0.0), 
m_camera(MIN_CAMERA_DIST)
{
  Menu::updateSculptMult(0.0, 0.0f);
}

FreeformApp::~FreeformApp()
{
  delete _environment;
  std::unique_lock<std::mutex> lock(_mesh_mutex);
  if (mesh_) {
    delete mesh_;
  }

}

void FreeformApp::prepareSettings( Settings *settings )
{
  settings->setTitle("Freeform");
  settings->setWindowSize(1024, 768);
  //settings->disableFrameRate();

  ci::app::Window::Format fmt;
  fmt.setTitle("Freeform");
  fmt.setSize(1024, 768);
#if LM_PRODUCTION_BUILD
  fmt.setFullScreen(true);
#endif
  settings->prepareWindow(fmt);

  enableVerticalSync(true);
}

void FreeformApp::toggleFullscreen(const std::string& str)
{
  bool full = isFullScreen();
  setFullScreen(!full);
}

void FreeformApp::setup()
{
// osx path stuff
#ifdef __APPLE__

  char exec_path[PATH_MAX] = {0};
  uint32_t pathSize = sizeof(exec_path);
  if (!_NSGetExecutablePath(exec_path, &pathSize)) {
    char fullpath[PATH_MAX] = {0};
    if (realpath(exec_path, fullpath)) {
      std::string path(fullpath);
      size_t pos = path.find_last_of('/');

      if (pos != std::string::npos) {
        path.erase(pos+1);
      }
      if (!path.empty()) {
        chdir(path.c_str());
      }
      const char* resources = "../Resources";
      if (!access(resources, R_OK)) {
        chdir(resources);
      }
    }
  }

#endif


#if _WIN32
  HMODULE instance = ::GetModuleHandle(0);
  SetClassLongPtr(getRenderer()->getHwnd(), GCLP_HICON, (LONG)::LoadImage(instance, MAKEINTRESOURCE(ICON_IDX), IMAGE_ICON, 0, 0, LR_DEFAULTSIZE));
#endif

  FreeImage_Initialise();
  Print3D::Init();
  enableAlphaBlending();
  enableDepthRead();
  enableDepthWrite();
  glCullFace( GL_BACK );
  glEnable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  _draw_edges = false;
  _bloom_visible = true;
  _bloom_size = 1.f;
  _bloom_strength = 1.f;
  _bloom_light_threshold = 0.5f;
  _brush_color = ci::Color(0.85f, 0.95f, 1.0f);

#if !LM_PRODUCTION_BUILD
  _params = params::InterfaceGl::create( getWindow(), "App parameters", toPixels( Vec2i( 200, 400 ) ) );
  _params->minimize();

  _params->addSeparator();
  _params->addText( "text", "label=`Camera parameters:`" );

  _params->addParam( "Use triangles.", &_camera_params.queryTriangles, "" );
  _params->addParam( "Draw debug lines", &_camera_params.drawDebugLines, "" );
  _params->addParam( "Draw sphere query", &_camera_params.drawSphereQueryResults, "" );

  _params->addParam( "Iso query radius", &_camera_params.isoQueryPaddingRadius, "min=10.0 max=300.0 step=10.0" );
  _params->addParam( "#clip rot iterations", &_camera_params.numRotationClipIterations, "min=1.0 max=8.0 step=1.0" );

  _params->addParam( "Iso ref dist mlt", &_camera_params.isoRefDistMultiplier, "min=0.1 max=10.0 step=0.1" );
  _params->addParam( "Grav_k", &_camera_params.grav_k, "min=0.0001 max=1.0 step=0.001" );
  _params->addParam( "Grav_n", &_camera_params.grav_n, "min=1.0 max=8.0 step=0.5" );

  _params->addParam( "Clip to iso surface", &_camera_params.clipToIsoSurface, "" );
  _params->addParam( "Clip movement", &_camera_params.clipCameraMovement, "" );
  _params->addParam( "Movement ref dist", &_camera_params.refDistForMovemement, "min=10.0 max=500.0 step=10.0" );
  _params->addParam( "Enable cone clip", &_camera_params.enableConeClipping, "" );
  _params->addParam( "Normal cone angle", &_camera_params.normalConeAngle, "min=0.0 max=1.55 step=0.05" );
  _params->addParam( "Enable max rot", &_camera_params.enableMaxReorientationRate, "" );
  _params->addParam( "Reorientation rate", &_camera_params.maxReorientationRate, "min=0.2 max=10.0 step=0.2" );
  _params->addParam( "Scale Z movement", &_camera_params.scaleZMovement, "min=0.1 max=2.0 step=0.1" );

  _params->addParam( "Enable Cam Reset", &_camera_params.enableCameraReset, "" );
  _params->addParam( "Enable Cam Orbit", &_camera_params.enableCameraOrbit, "" );

  _params->addParam( "Min Dist", &_camera_params.minDist, "min=1.0 max=100.0 step=1.0" );
  _params->addParam( "Max Dist", &_camera_params.maxDist, "min=100.0 max=1000.0 step=20.0" );
  _params->addParam( "Speed @ Min Dist", &_camera_params.speedAtMinDist, "min=0.01 max=1.0 step=0.1" );
  _params->addParam( "Speed @ Max Dist", &_camera_params.speedAtMaxDist, "min=1.0 max=20.0 step=1.0" );
  _params->addParam( "Pin up vector", &_camera_params.pinUpVector, "" );
  _params->addParam( "Input multiplier", &_camera_params.inputMultiplier, "min=0.5 max=5.0 step=0.25" );
  _params->addParam( "Invert camera input", &_camera_params.invertCameraInput, "" );
  _params->addParam( "Smoothing", &_camera_params.enableSmoothing, "" );
  _params->addParam( "Smooth factor", &_camera_params.smoothingFactor, "min=0.0 max=1.0 step=0.05" );
  _params->addSeparator();
  _params->addText( "text", "label=`Surface parameters:`" );
  _params->addParam( "Ambient", &_material.ambientFactor, "min=0.0 max=0.5 step=0.01" );
  _params->addParam( "Diffuse", &_material.diffuseFactor, "min=0.0 max=1.0 step=0.05" );
  _params->addParam( "Reflection", &_material.reflectionFactor, "min=0.0 max=1.0 step=0.05" );
  _params->addParam( "Refraction Index", &_material.refractionIndex, "min=0.0 max=1.0 step=0.01" );
  _params->addParam( "Reflection Bias", &_material.reflectionBias, "min=0.0 max=3.0 step=0.01" );
  _params->addParam( "Refraction Bias", &_material.refractionBias, "min=0.0 max=3.0 step=0.01" );
  _params->addSeparator();
  _params->addText( "text", "label=`Draw parameters:`" );
  _params->addParam( "Draw UI", &_draw_ui, "" );
  _params->addParam( "Draw edges", &_draw_edges, "" );
  _params->addSeparator();
  _params->addText( "text", "label=`HDR parameters:`" );
  _params->addParam( "Exposure", &_exposure, "min=0.05 max=8.0 step=0.01" );
  _params->addParam( "Show bloom", &_bloom_visible, "" );
  _params->addParam( "Bloom size", &_bloom_size, "min=0.0 max=4.0 step=0.01" );
  _params->addParam( "Bloom strength", &_bloom_strength, "min=0.0 max=1.0 step=0.01" );
  _params->addParam( "Bloom threshold", &_bloom_light_threshold, "min=0.0 max=2.0 step=0.01" );
  _params->addParam( "Draw Background", &_draw_background, "" );
  _params->addParam( "Remesh Radius", &remeshRadius_, "min=20, max=200, step=2.5" );
#endif

  _environment = new CubeMapManager();

  try {
    _screen_shader = gl::GlslProg( loadResource( RES_PASSTHROUGH_VERT_GLSL ), loadResource( RES_SCREEN_FRAG_GLSL ) );
    _material_shader = gl::GlslProg( loadResource( RES_MATERIAL_VERT_GLSL ), loadResource( RES_MATERIAL_FRAG_GLSL ) );
    _brush_shader = gl::GlslProg( loadResource( RES_BRUSH_VERT_GLSL ), loadResource( RES_MATERIAL_FRAG_GLSL ) );
    _wireframe_shader = gl::GlslProg( loadResource( RES_MATERIAL_VERT_GLSL ), loadResource( RES_WIREFRAME_FRAG_GLSL ) );
    _sky_shader = gl::GlslProg( loadResource( RES_SKY_VERT_GLSL ), loadResource( RES_SKY_FRAG_GLSL ) );
    _bloom_shader = gl::GlslProg( loadResource( RES_PASSTHROUGH_VERT_GLSL ), loadResource( RES_BLOOM_FRAG_GLSL ) );
    Menu::g_previewShader = gl::GlslProg( loadResource( RES_PASSTHROUGH_VERT_GLSL ), loadResource( RES_PREVIEWS_FRAG_GLSL) );
  } catch (gl::GlslProgCompileExc e) {
    std::cout << e.what() << std::endl;
    _have_shaders = false;
  }

  sculpt_.clearBrushes();

  _machine_speed = parseRenderString(std::string((char*)glGetString(GL_RENDERER)));
  _bloom_visible = _machine_speed > FreeformApp::LOW;
  _aa_mode = _machine_speed > FreeformApp::LOW ? FreeformApp::MSAA : FreeformApp::NONE;
  _environment->setUseHDR(_machine_speed > FreeformApp::LOW);

  // set mesh detail depending on machine specs
  static const float LOW_DETAIL_LEVEL = 0.85f;
  static const float MEDIUM_DETAIL_LEVEL = 0.9f;
  static const float HIGH_DETAIL_LEVEL = 1.0f;
  float detail = LOW_DETAIL_LEVEL;
  if (_machine_speed == FreeformApp::MID) {
    detail = MEDIUM_DETAIL_LEVEL;
#if !LM_PRODUCTION_BUILD
    std::cout << "Medium Detail" << std::endl;
#endif
  } else if (_machine_speed == FreeformApp::HIGH) {
    detail = HIGH_DETAIL_LEVEL;
#if !LM_PRODUCTION_BUILD
    std::cout << "High Detail" << std::endl;
#endif
  } else {
#if !LM_PRODUCTION_BUILD
    std::cout << "Low Detail" << std::endl;
#endif
  }
  Sculpt::setDetail(detail);

  _ui = new UserInterface();
  _ui->setRegularFont(ci::Font(loadResource( RES_FONT_FREIGHTSANS_TTF ), Menu::FONT_SIZE));
  _ui->setBoldFont(ci::Font(loadResource( RES_FONT_FREIGHTSANSBOLD_TTF ), Menu::FONT_SIZE));

  _controller.addListener(_listener);

  _leap_interaction = new LeapInteraction(&sculpt_, _ui);

  sculpt_.setSculptMode(Sculpt::SWEEP);
  _leap_interaction->setBrushRadius(10.0f);
  _leap_interaction->setBrushStrength(0.5f);

//#if __APPLE__
//  if (_mesh_thread.joinable())
//  {
//    _mesh_thread.detach();
//  }
//#endif

  if (AutoSave::isFirstRun()) {
    _ui->forceDrawTutorialMenu();
  }

  loadIcons();
  loadShapes();
  loadImages();
  loadSounds();

  if (_auto_save.haveAutoSave()) {
    Files files;
    Mesh* mesh;
    try {
      std::ifstream file(_auto_save.getAutoSavePath());
      mesh = files.loadPLY(file);
      file.close();
    } catch (...) {
      mesh = 0;
    }
    mesh_ = mesh;
    if (mesh_) {
      mesh_->startPushState();
      _last_load_time = ci::app::getElapsedSeconds();
      sculpt_.setMesh(mesh_);
    } else {
      try {
        _auto_save.deleteAutoSave();
      } catch (...) { }
      loadShape(BALL);
    }
  } else {
    loadShape(BALL);
  }

#if ! LM_DISABLE_THREADING_AND_ENVIRONMENT
  _mesh_thread = std::thread(&FreeformApp::updateLeapAndMesh, this);
#endif

  _auto_save.start();
}

void FreeformApp::doQuit()
{
  shutdown();
  quit();
}

void FreeformApp::shutdown() {
  _shutdown = true;
  if (_mesh_thread.joinable())
  {
    _mesh_thread.join();
  }
  if (_loading_thread.joinable())
  {
    _loading_thread.detach();
  }
  FreeImage_DeInitialise();
  Print3D::Cleanup();
  if (_have_audio && m_soundEngine) {
    m_soundEngine->stopAllSounds();
  }
}

void FreeformApp::resize()
{
  static const int BLUR_DOWNSCALE_FACTOR = 4;

  int width = getWindowWidth();
  int height = getWindowHeight();

  if (_have_audio && m_soundEngine) {
    if (width == 0 || height == 0) {
      // app is minimized, pause all sounds
      m_soundEngine->setAllSoundsPaused(true);
    } else {
      m_soundEngine->setAllSoundsPaused(_audio_paused);
    }
  }

  width = std::max(BLUR_DOWNSCALE_FACTOR, width);
  height = std::max(BLUR_DOWNSCALE_FACTOR, height);
  const int blurWidth = width / BLUR_DOWNSCALE_FACTOR;
  const int blurHeight = height / BLUR_DOWNSCALE_FACTOR;

  // FBOs with no depth buffer and bilinear sampling

  GLBuffer::checkError("Setup");

  Fbo::Format blurFormat;
  if (_machine_speed > FreeformApp::LOW) {
    blurFormat.setColorInternalFormat(GL_RGB16F_ARB);
  }
  blurFormat.setMinFilter(GL_LINEAR);
  blurFormat.setMagFilter(GL_LINEAR);
  blurFormat.enableMipmapping(false);
  blurFormat.enableDepthBuffer(false);

  _horizontal_blur_fbo = Fbo(blurWidth, blurHeight, blurFormat);
  _vertical_blur_fbo = Fbo(blurWidth, blurHeight, blurFormat);
  GLBuffer::checkError("Blur FBOs");

  // FBOs with depth buffer

  Fbo::Format screenFormat;
  if (_machine_speed > FreeformApp::LOW) {
    screenFormat.setColorInternalFormat(GL_RGB16F_ARB);
  }
  screenFormat.setMinFilter(GL_NEAREST);
  screenFormat.setMagFilter(GL_NEAREST);
  screenFormat.enableMipmapping(false);
  screenFormat.enableDepthBuffer(true, false);
  if (_aa_mode == MSAA) {
    screenFormat.setSamples(4);
  }

  _screen_fbo = Fbo(width, height, screenFormat);
  GLBuffer::checkError("Screen FBO");

  m_camera.onResize( getWindowAspectRatio() );
  _ui->setWindowSize( Vec2i(width, height) );

  glEnable(GL_FRAMEBUFFER_SRGB);
  GLBuffer::checkError("SRGB");
}

void FreeformApp::mouseDown( MouseEvent event )
{
  _mouse_down = true;

  _current_mouse_pos = _previous_mouse_pos = _initial_mouse_pos = event.getPos();
}

void FreeformApp::mouseUp( MouseEvent event )
{
  _mouse_down = false;
}

void FreeformApp::mouseDrag( MouseEvent event )
{
  _previous_mouse_pos = _current_mouse_pos;
  _current_mouse_pos = event.getPos();

  Vec2f dMouse = _current_mouse_pos - _previous_mouse_pos;
  dMouse *= CAMERA_SPEED;
  
  m_camera.onMouseMove(dMouse.x, dMouse.y);
}

void FreeformApp::mouseWheel( MouseEvent event)
{
  m_camera.setZoom(event.getWheelIncrement());
}

void FreeformApp::mouseMove( MouseEvent event)
{
  _previous_mouse_pos = _current_mouse_pos;
  _current_mouse_pos = event.getPos();
}

void FreeformApp::keyDown( KeyEvent event )
{
  static const double MIN_TIME_SINCE_SCULPTING_FOR_UNDO = 0.25;
  const double curTime = ci::app::getElapsedSeconds();
  const double lastSculptTime = sculpt_.getLastSculptTime();
  const bool allowUndo = (curTime - lastSculptTime) > MIN_TIME_SINCE_SCULPTING_FOR_UNDO;

  switch( event.getChar() )
  {
#if !LM_PRODUCTION_BUILD
  case 'u': _draw_ui = !_draw_ui; break;
  case 'o': drawOctree_ = !drawOctree_; break;
  case 's': toggleSymmetry(); break;
  case 'r': sculpt_.setRemeshRadius(remeshRadius_); break;
#endif
#if __APPLE__
  case 'y': if (event.isMetaDown()) { if (mesh_ && allowUndo) { mesh_->redo(); } } break;
  case 'z': if (event.isMetaDown()) { if (mesh_ && allowUndo) { mesh_->undo(); } } break;
  case 'f': if (event.isMetaDown()) toggleFullscreen(""); break;
#else
  case 'y': if (event.isControlDown()) { if (mesh_ && allowUndo) { mesh_->redo(); } } break;
  case 'z': if (event.isControlDown()) { if (mesh_ && allowUndo) { mesh_->undo(); } } break;
  case 'n': if (_ui->haveExitConfirm()) { _ui->clearConfirm(); } break;
#endif
  case KeyEvent::KEY_ESCAPE:
    if (_first_environment_load) {
      doQuit();
    }
    if (_ui->haveExitConfirm()) {
      _ui->clearConfirm();
    } else {
      _ui->showConfirm(Menu::GENERAL_EXIT);
    }
    break;
  case 'c': _lock_camera = !_lock_camera; break;
  }
#if _WIN32
  if (event.isAltDown() && event.getCode() == KeyEvent::KEY_F4) {
    doQuit();
  }
  if (event.isAltDown() && event.getCode() == KeyEvent::KEY_RETURN) {
    setFullScreen(!isFullScreen());
  }
#endif
  if (event.getCode() == KeyEvent::KEY_RETURN || event.getChar() == 'y') {
    if (_ui->haveExitConfirm()) {
      doQuit();
    }
  }
}

void FreeformApp::update()
{
#ifdef ALLOW_REPLAY
  static int updateCount = 0;
  LM_ASSERT_IDENTICAL(1243);
  LM_ASSERT_IDENTICAL("\r\n\r\nUpdate frame #");
  LM_ASSERT_IDENTICAL(updateCount++);
  LM_ASSERT_IDENTICAL("\r\n");
#if LM_LOG_CAMERA_LOGIC_4
  std::cout << std::endl << "Frm#" << updateCount << " ";
#endif
#endif
  const double curTime = ci::app::getElapsedSeconds();
  LM_TRACK_CONST_VALUE(curTime);
  const float deltaTime = _last_update_time == 0.0 ? 0.0f : static_cast<float>(curTime - _last_update_time);

  static const float TIME_UNTIL_AUTOMATIC_ORBIT = 60.0f;
  static const float TIME_UNTIL_AUTOMATIC_FOV = 50.0f;
  const float timeSinceActivity = static_cast<float>(curTime - _leap_interaction->getLastActivityTime());
  LM_TRACK_CONST_VALUE(timeSinceActivity);
  
  //UI Update
  const float inactivityRatio = Utilities::SmootherStep(ci::math<float>::clamp(timeSinceActivity - TIME_UNTIL_AUTOMATIC_FOV, 0.0f, TIME_UNTIL_AUTOMATIC_FOV)/TIME_UNTIL_AUTOMATIC_FOV);
  const float scaleFactor = _leap_interaction->getScaleFactor();

  if (curTime - _immersive_changed_time > 0.5) {
    if (_immersive_mode && scaleFactor < 0.95f) {
      _immersive_mode = false;
      _immersive_changed_time = curTime;
    } else if (!_immersive_mode && scaleFactor > 1.05f && !_ui->tutorialActive()) {
      _immersive_mode = true;
      if (!_have_entered_immersive) {
        _immersive_entered_time = curTime;
        _have_entered_immersive = true;
      }
      _immersive_changed_time = curTime;
    }
  }
  static const float LOWER_BOUND = 1.0f;
  static const float UPPER_BOUND = 1.32f;
  static const float IMMERSIVE_MODE_SMOOTH_STRENGTH = 0.9f;

  _ui_zoom.Update(_immersive_mode ? UPPER_BOUND : LOWER_BOUND, curTime, IMMERSIVE_MODE_SMOOTH_STRENGTH);
  _ui->setZoomFactor(_ui_zoom.value);
  _ui->update(_leap_interaction, &sculpt_);
  _ui->handleSelections(&sculpt_, _leap_interaction, this, mesh_);

  //Camera Update
  _camera_params.forceCameraOrbit = timeSinceActivity > TIME_UNTIL_AUTOMATIC_ORBIT;
  const Vec4f deltaVector = _leap_interaction->getDeltaVector();
  
  m_camera.update(deltaVector*deltaTime, curTime, sculpt_.getLastSculptTime());
  m_camera.setFovModifier((-_ui_zoom.value * 20.0f) + (-inactivityRatio * 5.0f), curTime);

  lmTransform tCamera = m_camera.util.GetCameraInWorldSpace();

  Vec3f campos = ToVec3f(tCamera.translation);
  Vector3 up = tCamera.rotation * Vector3::UnitY();
  Vector3 to = tCamera.translation + tCamera.rotation * Vector3::UnitZ() * -200.0f;

  Matrix4x4 trans = mesh_->getTransformation();
  Vector4 temp;
  {
    std::unique_lock<std::mutex> lock(m_camera.util.m_referencePointMutex);
    temp << m_camera.util.isoState.refPosition, 1.0;
  }
  _focus_point = (trans * temp).head<3>();
  // if mesh
  if (mesh_) {
    _focus_radius = m_camera.util.IsoQueryRadius(mesh_, &m_camera.util.isoState);
  } else {
    _focus_radius = 0.0f;
  }

  _campos_smoother.Update(campos, curTime, 0.95f);
  _lookat_smoother.Update(ToVec3f(to), curTime, 0.95f);
  _up_smoother.Update(ToVec3f(up), curTime, 0.95f);

  m_camera.lookAt(_campos_smoother.value, _lookat_smoother.value, _up_smoother.value.normalized());
  m_camera.onResize(getWindowAspectRatio());
  m_camera.getProjectionMatrix();

  if (mesh_) {
    mesh_->updateGPUBuffers();
  }

  _last_update_time = curTime;

#if LM_DISABLE_THREADING_AND_ENVIRONMENT
  // work for the other thread
  FreeformApp::updateLeapAndMesh();
#endif
}

void FreeformApp::updateLeapAndMesh() {
  static const double BRUSH_DISABLE_TIME_AFTER_LOAD = 1.0;
#if ! LM_DISABLE_THREADING_AND_ENVIRONMENT
  while (!_shutdown) 
#endif
  {
    const double curTime = ci::app::getElapsedSeconds();
    LM_TRACK_CONST_VALUE(curTime);
#if ! LM_DISABLE_THREADING_AND_ENVIRONMENT
    bool suppress = _environment->getLoadingState() != CubeMapManager::LOADING_STATE_NONE;
    suppress = suppress || (curTime - _last_load_time) < BRUSH_DISABLE_TIME_AFTER_LOAD;
#else 
    bool suppress = false;
#endif 
    bool haveFrame;
    try {
      haveFrame = _leap_interaction->processInteraction(_listener, getWindowAspectRatio(), m_camera.getModelViewMatrix(), m_camera.getProjectionMatrix(), getWindowSize(), m_camera.util.GetReferenceDistance(), Utilities::DEGREES_TO_RADIANS*60.0f, suppress);
    } catch (...) {
      haveFrame = false;
    }

    if (haveFrame) {
      const double lastSculptTime = sculpt_.getLastSculptTime();
      LM_TRACK_CONST_VALUE(lastSculptTime);

      std::unique_lock<std::mutex> lock(_mesh_mutex);
      if (mesh_) {
        {
          std::unique_lock<std::mutex> lock(_mesh_update_rotation_mutex);
          mesh_->updateRotation(curTime);
        }
        if (!_lock_camera && fabs(curTime - lastSculptTime) > 0.25) {
          m_camera.util.UpdateCamera(mesh_, &_camera_params);
        }
        if (!_ui->tutorialActive() || _ui->toolsSlideActive()) {
          sculpt_.applyBrushes(curTime, &_auto_save);
          m_camera.util.m_timeOfLastScupt = static_cast<lmReal>(sculpt_.getLastSculptTime());
        }
      }
      _mesh_update_counter.Update(ci::app::getElapsedSeconds());
    } else if (mesh_) {
      // Allow camera movement when leap is disconnected
      std::unique_lock<std::mutex> lock(_mesh_mutex);
      m_camera.util.UpdateCamera(mesh_, &_camera_params);
    }
  }
}

void FreeformApp::renderSceneToFbo(Camera& _Camera)
{
  GLBuffer::checkFrameBufferStatus("1");

  const double curTime = ci::app::getElapsedSeconds();

  // set FOV and depth parameters based on current state
  float depthMinZoomed = 0.35f;
  float depthMinOut = 0.0f;
  float depth_min;

  static const float FOV_TOLERANCE = 5.0f;
  float blend = (m_camera.getZoom()-(MIN_FOV+FOV_TOLERANCE))/(MAX_FOV-MIN_FOV-(2*FOV_TOLERANCE));
  blend = Utilities::SmootherStep(std::sqrt(math<float>::clamp(blend)));
  depth_min = depthMinZoomed*(1.0f-blend) + depthMinOut*blend;

  // disable depth and culling for rendering skybox
  gl::disableDepthRead();
  gl::disableDepthWrite();
  glDisable(GL_CULL_FACE);

  _screen_fbo.bindFramebuffer();
  setViewport( _screen_fbo.getBounds() );
  clear();
  setMatrices( m_camera );
  if (_draw_background) {
    // draw color pass of skybox
    _environment->bindCubeMap(CubeMapManager::CUBEMAP_SKY, 0);
    _sky_shader.bind();
    _sky_shader.uniform("cubemap", 0);
    gl::drawSphere(Vec3f::zero(), SPHERE_RADIUS, 40);
    _sky_shader.unbind();
    _environment->unbindCubeMap(0);
  }

  // enable depth and culling for rendering mesh
  glEnable(GL_CULL_FACE);
  gl::enableDepthRead();
  gl::enableDepthWrite();
  enableAlphaBlending();

  GLBuffer::checkFrameBufferStatus("2");

  _environment->bindCubeMap(CubeMapManager::CUBEMAP_IRRADIANCE, 0);
  _environment->bindCubeMap(CubeMapManager::CUBEMAP_RADIANCE, 1);

  BrushVector brushes = sculpt_.getBrushes();
  int numBrushes = brushes.size();
  std::vector<ci::Vec3f> brushPositions;
  std::vector<float> brushWeights;
  std::vector<float> brushRadii;
  for (int i=0; i<numBrushes; i++) {
    const Vector3& pos = brushes[i]._position;
    brushPositions.push_back(ci::Vec3f(pos.x(), pos.y(), pos.z()));
    brushWeights.push_back(brushes[i]._activation);
    brushRadii.push_back(brushes[i]._radius);
  }

  static const float FOCUS_POINT_OPACITY_TIME = 2.0f;
  float focusOpacity = std::min(1.0f, static_cast<float>(curTime - _leap_interaction->getLastCameraUpdateTime())/FOCUS_POINT_OPACITY_TIME);
  focusOpacity = Utilities::SmootherStep(1.0f - focusOpacity);
  _focus_opacity_smoother.Update(focusOpacity, curTime, 0.925f);
  focusOpacity = _focus_opacity_smoother.value;
  ci::Vec3f focus(_focus_point.x(), _focus_point.y(), _focus_point.z());
  brushPositions.push_back(focus);
  brushWeights.push_back(focusOpacity);
  brushRadii.push_back(_focus_radius);
  numBrushes++;

  ci::Matrix44f transform = ci::Matrix44f::identity();
  if (mesh_) {
    transform = ci::Matrix44f(mesh_->getTransformation(curTime).data());
  }
  ci::Matrix44f transformit = transform.inverted().transposed();

  const double lastSculptTime = sculpt_.getLastSculptTime();

  GLBuffer::checkFrameBufferStatus("3");

  if (mesh_) {
    _material_shader.bind();
    GLint vertex = _material_shader.getAttribLocation("vertex");
    GLint normal = _material_shader.getAttribLocation("normal");
    GLint color = _material_shader.getAttribLocation("color");

    const ci::Color surface = ci::Color(_material.surfaceColor.x(), _material.surfaceColor.y(), _material.surfaceColor.z());

    // draw mesh
    _material_shader.uniform( "useRefraction", true);
    _material_shader.uniform( "campos", _Camera.getEyePoint() );
    _material_shader.uniform( "irradiance", 0 );
    _material_shader.uniform( "radiance", 1 );
#if !LM_DISABLE_THREADING_AND_ENVIRONMENT
    _material_shader.uniform( "ambientFactor", _material.ambientFactor);
#else
    _material_shader.uniform( "ambientFactor", 0.27f);
#endif
    _material_shader.uniform( "diffuseFactor", _material.diffuseFactor);
    _material_shader.uniform( "reflectionFactor", _material.reflectionFactor);
    _material_shader.uniform( "surfaceColor", surface);
    _material_shader.uniform( "transform", transform );
    _material_shader.uniform( "transformit", transformit );
    _material_shader.uniform( "reflectionBias", _material.reflectionBias);
    _material_shader.uniform( "refractionBias", _material.refractionBias);
    _material_shader.uniform( "refractionIndex", _material.refractionIndex);
    _material_shader.uniform( "numLights", numBrushes );
    _material_shader.uniform( "brushPositions", brushPositions.data(), numBrushes );
    _material_shader.uniform( "brushWeights", brushWeights.data(), numBrushes );
    _material_shader.uniform( "brushRadii", brushRadii.data(), numBrushes );
    _material_shader.uniform( "lightColor", 0.1f*_brush_color );
    _material_shader.uniform( "lightExponent", 30.0f);
    _material_shader.uniform( "lightRadius", 50.0f);

    glPushMatrix();
    glPolygonOffset(1.0f, 1.0f);
    glPolygonMode(GL_FRONT, GL_FILL);
    glEnable(GL_POLYGON_OFFSET_FILL);
    mesh_->draw(vertex, normal, color);
    glDisable(GL_POLYGON_OFFSET_FILL);
    _material_shader.unbind();

    if (_draw_edges) {
      _wireframe_shader.bind();
      vertex = _wireframe_shader.getAttribLocation("vertex");
      _wireframe_shader.uniform( "transform", transform );
      _wireframe_shader.uniform( "transformit", transformit );
      _wireframe_shader.uniform( "surfaceColor", Color::black() );
      glPolygonMode(GL_FRONT, GL_LINE);
      glLineWidth(1.0f);
      mesh_->drawVerticesOnly(vertex);
      glPolygonMode(GL_FRONT, GL_FILL);
      _wireframe_shader.unbind();
    }
    glPopMatrix();
    Menu::updateSculptMult(curTime, (curTime - lastSculptTime) < 0.1 ? 0.15f : 1.0f);
  }

  if (m_camera.util.m_params.drawDebugLines) {
    _wireframe_shader.bind();
    _wireframe_shader.uniform( "transform", transform );
    _wireframe_shader.uniform( "transformit", transformit );
    DebugDrawUtil::getInstance().FlushDebugPrimitives(&_wireframe_shader);
    _wireframe_shader.unbind();
  }

  // draw brushes
  _brush_shader.bind();
  _brush_shader.uniform( "campos", _Camera.getEyePoint() );
  _brush_shader.uniform( "irradiance", 0 );
  _brush_shader.uniform( "radiance", 1 );
  _brush_shader.uniform( "useRefraction", false);
  _brush_shader.uniform( "reflectionBias", 0.5f );
  _brush_shader.uniform( "numLights", 0 );
  for (size_t i=0; i<brushes.size(); i++) {
    const float uiMult = (1.0f - _ui->maxActivation());
    const float sculptMult = static_cast<float>(std::min(1.0, (curTime - lastSculptTime)/0.25));
    const float alphaMult = (0.3f*sculptMult*uiMult + 0.4f)*brushes[i]._activation;

    // draw "ghost" brush without depth testing
    _brush_shader.uniform( "surfaceColor", 0.33f*_brush_color );
    _brush_shader.uniform( "alphaMult", 0.6f*alphaMult );
    _brush_shader.uniform( "ambientFactor", 0.3f );
    _brush_shader.uniform( "diffuseFactor", 0.0f );
    _brush_shader.uniform( "reflectionFactor", 0.0f );
    glDisable(GL_DEPTH_TEST);
    brushes[i].draw(uiMult);
    if (sculpt_.symmetry()) {
      // draw "ghost" symmetry brush
      _brush_shader.uniform( "alphaMult", 0.5f*alphaMult );
      brushes[i].reflected(0).draw(uiMult);
    }

    // draw regular brush
    _brush_shader.uniform( "surfaceColor", _brush_color );
    _brush_shader.uniform( "alphaMult", alphaMult );
    _brush_shader.uniform( "ambientFactor", 0.2f );
    _brush_shader.uniform( "diffuseFactor", 0.4f );
    _brush_shader.uniform( "reflectionFactor", 0.15f );
    glEnable(GL_DEPTH_TEST);
    brushes[i].draw(uiMult);
    if (sculpt_.symmetry()) {
      // draw regular symmetry brush
      _brush_shader.uniform( "alphaMult", 0.5f*alphaMult );
      brushes[i].reflected(0).draw(uiMult);
    }
  }
  _brush_shader.unbind();
  
  _environment->unbindCubeMap(1);
  _environment->unbindCubeMap(0);

  if (drawOctree_) {
    mesh_->drawOctree();
  }

  _screen_fbo.unbindFramebuffer();

  GLBuffer::checkFrameBufferStatus("4");
}

void FreeformApp::createBloom()
{
  if (!_have_shaders) {
    return;
  }
  gl::color(ci::ColorA::white());
  const float horizSize = _bloom_size / _horizontal_blur_fbo.getWidth();
  _horizontal_blur_fbo.bindFramebuffer();
  setViewport( _horizontal_blur_fbo.getBounds() );
  clear();
  gl::setMatricesWindow(_horizontal_blur_fbo.getWidth(), _horizontal_blur_fbo.getHeight(), false);
  _screen_fbo.bindTexture(0);
  _bloom_shader.bind();
  _bloom_shader.uniform( "color_tex", 0 );
  _bloom_shader.uniform( "sample_offset", Vec2f(horizSize, 0.0f));
  _bloom_shader.uniform( "light_threshold", _bloom_light_threshold );
  gl::drawSolidRect(Rectf(0.0f, 0.0f, static_cast<float>(_horizontal_blur_fbo.getWidth()), static_cast<float>(_horizontal_blur_fbo.getHeight())));
  _bloom_shader.unbind();
  _screen_fbo.unbindTexture();
  _horizontal_blur_fbo.unbindFramebuffer();

  const float vertSize = _bloom_size / _vertical_blur_fbo.getHeight();
  _vertical_blur_fbo.bindFramebuffer();
  setViewport(_vertical_blur_fbo.getBounds());
  clear();
  gl::setMatricesWindow(_vertical_blur_fbo.getWidth(), _vertical_blur_fbo.getHeight(), false);
  _horizontal_blur_fbo.bindTexture(0);
  _bloom_shader.bind();
  _bloom_shader.uniform( "color_tex", 0 );
  _bloom_shader.uniform( "sample_offset", Vec2f(0.0f, vertSize));
  _bloom_shader.uniform( "light_threshold", 0.0f );
  gl::drawSolidRect(Rectf(0.0f, 0.0f, static_cast<float>(_vertical_blur_fbo.getWidth()), static_cast<float>(_vertical_blur_fbo.getHeight())));
  _bloom_shader.unbind();
  _horizontal_blur_fbo.unbindTexture();
  _vertical_blur_fbo.unbindFramebuffer();
}

float FreeformApp::checkEnvironmentLoading() {
  static const float LOADING_DARKEN_TIME = 0.5f;
  static const float LOADING_LIGHTEN_TIME = 2.0f;

  const CubeMapManager::LoadingState loadingState = _environment->getLoadingState();
  const double curTime = ci::app::getElapsedSeconds();
  const float timeSinceStateChange = static_cast<float>(curTime - _environment->getLastStateChangeTime());

#if !LM_DISABLE_THREADING_AND_ENVIRONMENT
  if (!_environment->haveEnvironment()) {
    Menu::updateSculptMult(curTime, 0.0f);
  }
#endif

  float exposureMult = 1.0f;
  if (loadingState == CubeMapManager::LOADING_STATE_NONE) {
    const float ratio = math<float>::clamp(timeSinceStateChange/LOADING_LIGHTEN_TIME);
    exposureMult = Utilities::SmootherStep(ratio*ratio);
  } else if (loadingState == CubeMapManager::LOADING_STATE_LOADING) {
    const float ratio = 1.0f - math<float>::clamp(timeSinceStateChange/LOADING_DARKEN_TIME);
    exposureMult = Utilities::SmootherStep(ratio*ratio);
  } else if (loadingState == CubeMapManager::LOADING_STATE_DONE_LOADING) {
    exposureMult = 0.0f;
    _environment->finishLoading();
    _loading_thread.join();
    _loading_thread = std::thread(&CubeMapManager::beginProcessing, _environment);
    const std::string& name = _environment->getPendingEnvironmentString();
    CubeMapManager::CubeMapInfo* info = CubeMapManager::getEnvironmentInfoFromString(name);
    if (info) {
      _bloom_strength = info->_bloom_strength;
      _bloom_light_threshold = info->_bloom_threshold;
      _exposure = info->_exposure;
    }
    if (_have_audio) {
      m_activeLoop = m_audioLoops[name];
    }
  } else if (loadingState == CubeMapManager::LOADING_STATE_PROCESSING) {
    exposureMult = 0.0f;
  } else if (loadingState == CubeMapManager::LOADING_STATE_DONE_PROCESSING) {
    exposureMult = 0.0f;
    _environment->finishProcessing();
    _loading_thread.join();
    _first_environment_load = false;
  }

#if !LM_DISABLE_THREADING_AND_ENVIRONMENT
  if (_first_environment_load) {
    exposureMult = 0.0f;
  }
#endif

  return exposureMult;
}

void FreeformApp::draw() {
  glDisable(GL_FRAMEBUFFER_SRGB);

  clear();
  const double curTime = ci::app::getElapsedSeconds();
  const float exposureMult = checkEnvironmentLoading();

  if (_have_audio) {
    float backgroundMult = _listener.isReceivingFrames() ? 1.0f : 0.0f;
#if __APPLE__
    NSWindow *win = [reinterpret_cast<NSView*>(getWindow()->getNative()) window];
    backgroundMult *= ([win isVisible] && [win isOnActiveSpace]) ? 1.0f : 0.0f;
#endif
    if (m_activeLoop.first && m_activeLoop.second) {
      m_activeLoop.first->setVolume(0.3f*exposureMult*backgroundMult);
      m_activeLoop.second->setVolume(0.3f*exposureMult*backgroundMult);
    }
    if (m_soundEngine) {
      m_soundEngine->update();
    }
  }

  const ci::Vec2i size = getWindowSize();
//  const ci::Area bounds = getWindowBounds();
  const ci::Vec2f center = getWindowCenter();
  const ci::Area viewport = getViewport();

  if (exposureMult > 0.0f && _have_shaders) {
    renderSceneToFbo(m_camera);

    GLBuffer::checkError("After FBO");
    GLBuffer::checkFrameBufferStatus("After FBO");
  }

  glEnable(GL_FRAMEBUFFER_SRGB);

  glDisable(GL_CULL_FACE);
  disableDepthRead();
  disableDepthWrite();
  enableAlphaBlending();

  int errorNum = 0;

  if (exposureMult > 0.0f && _have_shaders) {
    if (_bloom_visible) {
      createBloom();
    }

    const float width = static_cast<float>(size.x);
    const float height = static_cast<float>(size.y);

    setViewport( viewport );
    setMatricesWindow( getWindowWidth(), getWindowHeight(), false);
    const float overlayMult = (_ui->aboutActive() || _ui->tutorialActive()) ? 0.65f : 1.0f;

    gl::color(ci::ColorA::white());

    _screen_fbo.bindTexture(0);
    _vertical_blur_fbo.bindTexture(1);
    _screen_shader.bind();
    _screen_shader.uniform( "color_texture", 0 );
    _screen_shader.uniform( "bloom_texture", 1 );
    _screen_shader.uniform( "depth_texture", 2 );
    _screen_shader.uniform( "width", width );
    _screen_shader.uniform( "height", height );
    _screen_shader.uniform( "exposure", _exposure * exposureMult * overlayMult);
    _screen_shader.uniform( "bloom_strength", _bloom_strength * static_cast<float>(_bloom_visible) );
    _screen_shader.uniform( "vignette_radius", static_cast<float>(0.9f * sqrt((width/2)*(width/2) + (height/2)*(height/2))) );
    _screen_shader.uniform( "vignette_strength", 0.75f );
    gl::drawSolidRect(Rectf(0.0f,0.0f,width,height));
    _screen_shader.unbind();
    _vertical_blur_fbo.unbindTexture();
    _screen_fbo.unbindTexture();
    
    GLBuffer::checkError("After post process");
    GLBuffer::checkFrameBufferStatus("After post process");

#if !LM_PRODUCTION_BUILD
    if (_draw_ui) {
      int tris = 0;
      int verts = 0;
      if (mesh_) {
        tris = mesh_->getNbTriangles();
        verts = mesh_->getNbVertices();
      }
      std::stringstream ss;
      ss << getAverageFps() << " render fps, " << _mesh_update_counter.FPS() << " simulate fps, " << tris << " triangles, " << verts << " vertices";
      glPushMatrix();
      gl::scale(1, -1);
      ci::gl::drawString(ss.str(), Vec2f(5.0f, -(height-5.0f)), ColorA::white(), Font("Arial", 18));
      glPopMatrix();
    }
#endif

    gl::color(ci::ColorA::white());

    setMatricesWindow( size );
    setViewport( viewport );

    if (_screenshot_path.empty() && _draw_ui) {
      _ui->draw(exposureMult);
    }

    if (_have_entered_immersive) {
      static const float DISPLAY_TIME = 10.0;
      static const float FADE_TIME = 1.0f;
      const float timeSinceEntered = static_cast<float>(curTime - _immersive_entered_time);
      float mult = 1.0f;
      if (timeSinceEntered < FADE_TIME) {
        mult = Utilities::SmootherStep(timeSinceEntered/FADE_TIME);
      } else if (timeSinceEntered > (DISPLAY_TIME - FADE_TIME)) {
        mult = Utilities::SmootherStep((DISPLAY_TIME - timeSinceEntered)/FADE_TIME);
      } else if (timeSinceEntered >= DISPLAY_TIME) {
        mult = 0.0f;
      }
      if (mult > 0.01f) {
        _ui->drawImmersive(mult);
      }
    }


    if (_environment->haveEnvironment()) {
      _ui->drawTutorialSlides(exposureMult);
      _ui->drawAbout(exposureMult);
    }

    if (!_listener.isConnected()) {
      _ui->drawError("Leap Motion Controller is disconnected", errorNum++);
    }

    GLBuffer::checkError("After UI");
    GLBuffer::checkFrameBufferStatus("After UI");
  }

  setMatricesWindow( size );
  setViewport( viewport );

  if (exposureMult < 1.0f) {
    // draw logo
    const float aspect = _logo_on_black.getAspectRatio();
    const float yOffset = static_cast<float>(-size.y / 20.0f);
    static const float LOGO_SCALE = 0.6f;
    float halfWidth = LOGO_SCALE*size.x/2;
    float halfHeight = halfWidth / aspect;
    ci::Rectf area(center.x - halfWidth, center.y - halfHeight + yOffset, center.x + halfWidth, center.y + halfHeight + yOffset);
    glColor4f(1.0f, 1.0f, 1.0f, (1.0f - exposureMult));
    ci::gl::draw(_logo_on_black, area);

    // draw loading animation
    static const float ANIM_DRAW_LIMIT = 0.25f;
    if (exposureMult < ANIM_DRAW_LIMIT) {
      const float loadingYOffset = static_cast<float>(size.y / 4.0f);
      const ci::Vec2f loadingCenter(center.x, center.y + loadingYOffset);
      const float loadingRadius = static_cast<float>(size.x/50.0f);
      const float loadingStartAngle = -static_cast<float>(curTime * 360.0f);
      const float loadingSweepAngle = 240.0f;
      glColor4f(0.7f, 0.7f, 0.7f, (ANIM_DRAW_LIMIT - exposureMult)/ANIM_DRAW_LIMIT);
      Utilities::drawPartialDisk(loadingCenter, loadingRadius*0.85f, loadingRadius, loadingStartAngle, loadingSweepAngle);
    }
  }

  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  Utilities::drawPartialDisk(ci::Vec2f(-200, -200), 10, 20, 0, 50);
  
  if (!_have_shaders) {
    _ui->drawError("Graphics error occurred. Please update drivers and check minimum requirements.", errorNum++);
  }

  if (_environment->getLoadingState() == CubeMapManager::LOADING_STATE_FAILED) {
    _ui->drawError("Environment loading failed. Please make sure Freeform is installed correctly.", errorNum++);
  }

  GLBuffer::checkError("After logo");
  GLBuffer::checkFrameBufferStatus("After logo");

#if !LM_PRODUCTION_BUILD
  _params->draw(); // draw the interface
#endif

  glFlush();

  if (!_screenshot_path.empty()) {
    try {
      writeImage(_screenshot_path, copyWindowSurface());
    } catch (...) { }
    _screenshot_path = "";
  }
}

void FreeformApp::loadIcons() {
  std::vector<ci::gl::Texture>& icons = Menu::g_icons;
  icons.resize(Menu::NUM_ICONS);

  icons[Menu::TOOL_PAINT] = ci::gl::Texture(loadImage(loadResource(RES_PAINT_SELECTED_PNG)));
  icons[Menu::TOOL_PUSH] = ci::gl::Texture(loadImage(loadResource(RES_PUSH_SELECTED_PNG)));
  icons[Menu::TOOL_SWEEP] = ci::gl::Texture(loadImage(loadResource(RES_SWEEP_SELECTED_PNG)));
  icons[Menu::TOOL_FLATTEN] = ci::gl::Texture(loadImage(loadResource(RES_FLATTEN_SELECTED_PNG)));
  icons[Menu::TOOL_SMOOTH] = ci::gl::Texture(loadImage(loadResource(RES_SMOOTH_SELECTED_PNG)));
  icons[Menu::TOOL_SHRINK] = ci::gl::Texture(loadImage(loadResource(RES_SHRINK_SELECTED_PNG)));
  icons[Menu::TOOL_GROW] = ci::gl::Texture(loadImage(loadResource(RES_GROW_SELECTED_PNG)));

  icons[Menu::STRENGTH_LOW] = ci::gl::Texture(loadImage(loadResource(RES_STRENGTH_LOW_SELECTED_PNG)));
  icons[Menu::STRENGTH_MEDIUM] = ci::gl::Texture(loadImage(loadResource(RES_STRENGTH_MEDIUM_SELECTED_PNG)));
  icons[Menu::STRENGTH_HIGH] = ci::gl::Texture(loadImage(loadResource(RES_STRENGTH_HIGH_SELECTED_PNG)));

  icons[Menu::MATERIAL_PLASTIC] = ci::gl::Texture(loadImage(loadResource(RES_PLASTIC_PNG)));
  icons[Menu::MATERIAL_PORCELAIN] = ci::gl::Texture(loadImage(loadResource(RES_PORCELAIN_PNG)));
  icons[Menu::MATERIAL_GLASS] = ci::gl::Texture(loadImage(loadResource(RES_GLASS_PNG)));
  icons[Menu::MATERIAL_METAL] = ci::gl::Texture(loadImage(loadResource(RES_STEEL_PNG)));
  icons[Menu::MATERIAL_CLAY] = ci::gl::Texture(loadImage(loadResource(RES_CLAY_PNG)));
}

void FreeformApp::loadShapes() {
  ci::DataSourceRef ball = loadResource(RES_BALL_OBJ);
  ci::DataSourceRef can = loadResource(RES_CAN_OBJ);
  ci::DataSourceRef donut = loadResource(RES_DONUT_OBJ);
  ci::DataSourceRef sheet = loadResource(RES_SHEET_OBJ);
  ci::DataSourceRef cube = loadResource(RES_CUBE_OBJ);
  ci::DataSourceRef snowman = loadResource(RES_SNOWMAN_OBJ);
  ci::Buffer& ballBuf = ball->getBuffer();
  ci::Buffer& canBuf = can->getBuffer();
  ci::Buffer& donutBuf = donut->getBuffer();
  ci::Buffer& sheetBuf = sheet->getBuffer();
  ci::Buffer& cubeBuf = cube->getBuffer();
  ci::Buffer& snowmanBuf = snowman->getBuffer();
  shapes_[BALL] = std::string((char*)ballBuf.getData(), ballBuf.getDataSize());
  shapes_[CAN] = std::string((char*)canBuf.getData(), canBuf.getDataSize());
  shapes_[DONUT] = std::string((char*)donutBuf.getData(), donutBuf.getDataSize());
  shapes_[SHEET] = std::string((char*)sheetBuf.getData(), sheetBuf.getDataSize());
  shapes_[CUBE] = std::string((char*)cubeBuf.getData(), cubeBuf.getDataSize());
  shapes_[SNOWMAN] = std::string((char*)snowmanBuf.getData(), snowmanBuf.getDataSize());
}

void FreeformApp::loadImages() {
  _logo_on_black = ci::gl::Texture(loadImage(loadResource(RES_LOGO_ON_BLACK)));
  _logo_on_image = ci::gl::Texture(loadImage(loadResource(RES_LOGO_ON_IMAGE)));

  std::vector<ci::gl::Texture>& previews = Menu::g_previews;
  previews.resize(Menu::NUM_ICONS);

  previews[Menu::ENVIRONMENT_JUNGLE_CLIFF] = ci::gl::Texture(loadImage(loadResource(RES_PREVIEW_JUNGLE_CLIFF)));
  previews[Menu::ENVIRONMENT_JUNGLE] = ci::gl::Texture(loadImage(loadResource(RES_PREVIEW_JUNGLE)));
  previews[Menu::ENVIRONMENT_ISLANDS] = ci::gl::Texture(loadImage(loadResource(RES_PREVIEW_ISLANDS)));
  previews[Menu::ENVIRONMENT_REDWOOD] = ci::gl::Texture(loadImage(loadResource(RES_PREVIEW_REDWOOD)));
  previews[Menu::ENVIRONMENT_DESERT] = ci::gl::Texture(loadImage(loadResource(RES_PREVIEW_DESERT)));
  previews[Menu::ENVIRONMENT_ARCTIC] = ci::gl::Texture(loadImage(loadResource(RES_PREVIEW_ARCTIC)));
  previews[Menu::ENVIRONMENT_RIVER] = ci::gl::Texture(loadImage(loadResource(RES_PREVIEW_RIVER)));

  _ui->setTutorialTextures(ci::gl::Texture(loadImage(loadResource(RES_TUTORIAL_1))),
                           ci::gl::Texture(loadImage(loadResource(RES_TUTORIAL_2))),
                           ci::gl::Texture(loadImage(loadResource(RES_TUTORIAL_3))),
                           ci::gl::Texture(loadImage(loadResource(RES_TUTORIAL_4))));

  _ui->setAboutTexture(ci::gl::Texture(loadImage(loadResource(RES_CREDITS))));
}

void FreeformApp::loadSounds() {
  m_soundEngine = irrklang::createIrrKlangDevice();
  if (!m_soundEngine) {
    std::cout << "Error loading sound engine" << std::endl;
    _have_audio = false;
  }

  if (_have_audio) {
    LoopPair pair;
    pair.first = createSoundResource(loadResource(RES_AUDIO_JUNGLE_CLIFF_1), "jungle-cliff1.ogg");
    pair.second = createSoundResource(loadResource(RES_AUDIO_JUNGLE_CLIFF_2), "jungle-cliff2.ogg");
    if (pair.first) { pair.first->setVolume(0.0f); }
    if (pair.second) { pair.second->setVolume(0.0f); }
    m_audioLoops["Jungle-Cliff"] = pair;

    pair.first = createSoundResource(loadResource(RES_AUDIO_JUNGLE_1), "jungle1.ogg");
    pair.second = createSoundResource(loadResource(RES_AUDIO_JUNGLE_2), "jungle2.ogg");
    if (pair.first) { pair.first->setVolume(0.0f); }
    if (pair.second) { pair.second->setVolume(0.0f); }
    m_audioLoops["Jungle"] = pair;

    pair.first = createSoundResource(loadResource(RES_AUDIO_ISLANDS_1), "islands1.ogg");
    pair.second = createSoundResource(loadResource(RES_AUDIO_ISLANDS_2), "islands2.ogg");
    if (pair.first) { pair.first->setVolume(0.0f); }
    if (pair.second) { pair.second->setVolume(0.0f); }
    m_audioLoops["Islands"] = pair;

    pair.first = createSoundResource(loadResource(RES_AUDIO_REDWOOD_1), "redwood1.ogg");
    pair.second = createSoundResource(loadResource(RES_AUDIO_REDWOOD_2), "redwood2.ogg");
    if (pair.first) { pair.first->setVolume(0.0f); }
    if (pair.second) { pair.second->setVolume(0.0f); }
    m_audioLoops["Redwood"] = pair;

    pair.first = createSoundResource(loadResource(RES_AUDIO_DESERT_1), "desert1.ogg");
    pair.second = createSoundResource(loadResource(RES_AUDIO_DESERT_2), "desert2.ogg");
    if (pair.first) { pair.first->setVolume(0.0f); }
    if (pair.second) { pair.second->setVolume(0.0f); }
    m_audioLoops["Desert"] = pair;

    pair.first = createSoundResource(loadResource(RES_AUDIO_ARCTIC_1), "arctic1.ogg");
    pair.second = createSoundResource(loadResource(RES_AUDIO_ARCTIC_2), "arctic2.ogg");
    if (pair.first) { pair.first->setVolume(0.0f); }
    if (pair.second) { pair.second->setVolume(0.0f); }
    m_audioLoops["Arctic"] = pair;

    pair.first = createSoundResource(loadResource(RES_AUDIO_RIVER_1), "river1.ogg");
    pair.second = createSoundResource(loadResource(RES_AUDIO_RIVER_2), "river2.ogg");
    if (pair.first) { pair.first->setVolume(0.0f); }
    if (pair.second) { pair.second->setVolume(0.0f); }
    m_audioLoops["River"] = pair;

    m_soundEngine->setAllSoundsPaused(false);
  }
}

irrklang::ISound* FreeformApp::createSoundResource(ci::DataSourceRef ref, const char* name) {
  if (m_soundEngine) {
    ci::Buffer& buf = ref->getBuffer();
    irrklang::ISoundSource* ss = m_soundEngine->addSoundSourceFromMemory(buf.getData(), buf.getDataSize(), name, false);
    m_audioSourceRefs.push_back(ref);
    ss->setForcedStreamingThreshold(300000);
    return m_soundEngine->play2D(ss, true, true, true, true);
  }

  return NULL;
}

FreeformApp::MachineSpeed FreeformApp::parseRenderString(const std::string& render_string) {
  if (render_string.find("Intel HD") != std::string::npos) {
    return FreeformApp::LOW;
  }
  if (render_string.find("Intel") != std::string::npos) {
    // Intel GMA or older
    return FreeformApp::LOW;
  }

  if (render_string.find("FireGL") != std::string::npos ||
    render_string.find("FirePro") != std::string::npos ||
    render_string.find("Quadro") != std::string::npos) {
      // workstation card, give them a shot at full resolution regardless of the model #
      return FreeformApp::HIGH;
  }

  std::string::const_iterator it = render_string.begin();
  for (; it != render_string.end(); it++) {
    if (isdigit(*it)) {
      break;
    }
  }
  if (it == render_string.end()) {
    std::cout << render_string << std::endl;
    return FreeformApp::LOW;
  }
  int model_number = atoi(render_string.substr(it - render_string.begin()).c_str());

  if (render_string.find("GeForce") != std::string::npos) {
    if (model_number > 1000) {
      // old GPU
      if (model_number < 9800) {
        return FreeformApp::LOW;
      } else {
        return FreeformApp::MID;
      }
    } else {
      // 200, 300 (GT200), 400 (Fermi)... 600 or higher series
      return FreeformApp::HIGH;
    }
  }
  if (render_string.find("Radeon X") != std::string::npos) {
    // Radeon X300, X600, X700, X800, X1000, wide range of all DirectX 9
    return FreeformApp::MID;
  }
  if (render_string.find("Radeon") != std::string::npos) {
    if (model_number >= 8000 && model_number < 10000) {
      // R200 series has big numbers like 8500LE, 9200SE, 9200, 9250, DirectX 8.1
      return FreeformApp::LOW;
    }
    if (model_number < 5000) {
      // R600 (HD2000) series through HD4000 series
      return FreeformApp::MID;
    }
    // HD5000 and higher
    return FreeformApp::HIGH;
  }
  // unrecognized video card, err on the middle
  return FreeformApp::MID;
}

void FreeformApp::setMaterial(const Material& mat) {
  _material = mat;
}

void FreeformApp::setWireframe(bool wireframe) {
  _draw_edges = wireframe;
}

void FreeformApp::toggleWireframe() {
  _draw_edges = !_draw_edges;
}

void FreeformApp::toggleSymmetry() {
  sculpt_.setSymmetry(!sculpt_.symmetry());
}

void FreeformApp::setEnvironment(const std::string& str) {
  if (!_environment || _environment->getLoadingState() != CubeMapManager::LOADING_STATE_NONE) {
    return;
  }
#if __APPLE__
  if (_loading_thread.joinable())
  {
    _loading_thread.detach();
  }
#endif
  _loading_thread = std::thread(&CubeMapManager::beginLoading, _environment, str);
}

void FreeformApp::toggleSound() {
  if (_have_audio && m_soundEngine) {
    _audio_paused = !_audio_paused;
    m_soundEngine->setAllSoundsPaused(_audio_paused);
  }
}

int FreeformApp::loadFile()
{
  // *** open mesh file ***
  std::vector<std::string> file_extensions;
  file_extensions.push_back("obj");
  file_extensions.push_back("stl");
  file_extensions.push_back("3ds");
  file_extensions.push_back("ply");

  bool toggleFull = isFullScreen();
  if (toggleFull) {
    setFullScreen(false);
  }
  fs::path path = getOpenFilePath("", file_extensions);
  if (toggleFull) {
    setFullScreen(true);
  }

  int err = -1;
  if (!path.empty()) {
    const std::string ext = path.extension().string();
    if (!ext.empty()) {
      Mesh::stateMask_= 1;
      Vertex::tagMask_ = 1;
      Vertex::sculptMask_ = 1;
      Triangle::tagMask_ = 1;

      Files files;
      Mesh* mesh = 0;
      const std::string pathString = path.string();
      try {
        std::ifstream stream;
        if (ext == ".OBJ" || ext == ".obj") {
          stream.open(pathString.c_str(), std::ios::in);
          mesh = files.loadOBJ(stream);
        } else if (ext == ".STL" || ext == ".stl") {
          stream.open(pathString.c_str(), std::ios::in | std::ios::binary);
          mesh = files.loadSTL(stream);
        } else if (ext == ".3DS" || ext == ".3ds") {
          stream.open(pathString.c_str(), std::ios::in | std::ios::binary);
          mesh = files.load3DS(stream);
        } else if (ext == ".PLY" || ext == ".ply") {
          stream.open(pathString.c_str(), std::ios::in);
          mesh = files.loadPLY(stream);
        }
        stream.close();
      } catch (...) {
        mesh = 0;
      }
      std::unique_lock<std::mutex> lock(_mesh_mutex);
      float rotationVel = 0.0f;
      if (mesh_) {
        rotationVel = mesh_->getRotationVelocity();
        delete mesh_;
      }
      mesh_ = mesh;
      mesh_->setRotationVelocity(rotationVel);
      if (mesh_) {
        _last_load_time = ci::app::getElapsedSeconds();
        mesh_->startPushState();
      }
      sculpt_.setMesh(mesh_);
      err = 1;
    }
  }

  return err; // error
}

int FreeformApp::loadShape(Shape shape) {
  Mesh::stateMask_= 1;
  Vertex::tagMask_ = 1;
  Vertex::sculptMask_ = 1;
  Triangle::tagMask_ = 1;

  std::stringstream ss(shapes_[shape]);
  Files files;
  Mesh* newMesh = files.loadOBJ(ss);
  if (!newMesh) {
    return 0;
  }
  std::unique_lock<std::mutex> lock(_mesh_mutex);
  float rotationVel = 0.0f;
  if (mesh_) {
    rotationVel = mesh_->getRotationVelocity();
    delete mesh_;
  }
  mesh_ = newMesh;
  mesh_->setRotationVelocity(rotationVel);
  if (mesh_) {
    mesh_->startPushState();
    _last_load_time = ci::app::getElapsedSeconds();
  }
  sculpt_.setMesh(mesh_);

  return -1;
}

#if __APPLE__
@interface SavePanelProtocol : NSObject <NSOpenSavePanelDelegate>
{
  NSArray* m_extensions;
  NSPopUpButton* m_pulldown;
}
- (id)initWithPopUpButton:(NSPopUpButton*)pulldown extensions:(NSArray*)extensions;
- (NSString *)panel:(id)sender userEnteredFilename:(NSString *)filename confirmed:(BOOL)okFlag;
@end

@implementation SavePanelProtocol
- (id)initWithPopUpButton:(NSPopUpButton*)pulldown extensions:(NSArray*)extensions
{
  if ((self = [super init])) {
    m_extensions = extensions;
    m_pulldown = pulldown;
  }
  return self;
}

- (NSString *)panel:(id)sender userEnteredFilename:(NSString *)filename confirmed:(BOOL)okFlag
{
  if (okFlag && m_pulldown && m_extensions) {
    NSMutableString* returnFileName = [[NSMutableString alloc] initWithString: filename];
    NSString* extension = [returnFileName pathExtension];
    if (extension) {
      for (NSString* ext in m_extensions) {
        if ([extension caseInsensitiveCompare:ext] == NSOrderedSame) {
          return filename;
        }
      }
    }
    NSInteger index = [m_pulldown indexOfSelectedItem];
    if (index >= 0) {
      [returnFileName appendString:@"."];
      [returnFileName appendString:[m_extensions objectAtIndex:index]];
    }
    return returnFileName;
  }
  return filename;
}
@end
#endif

fs::path FreeformApp::getSaveFilePathCustom(const fs::path &initialPath,
                                            const std::vector<std::string>& extensions,
                                            const std::vector<std::string>& descriptions)
{
#if __APPLE__
  NSSavePanel *cinderSave = [NSSavePanel savePanel];
  NSWindow *win = [reinterpret_cast<NSView*>(getWindow()->getNative()) window];
  NSPopUpButton* pulldown = nil;
  SavePanelProtocol* savePanelProtocol = nil;

  NSMutableArray *typesArray = nil;
  if (!extensions.empty()) {
    typesArray = [NSMutableArray arrayWithCapacity:extensions.size()];
    for (auto iter = extensions.cbegin(); iter != extensions.cend(); ++iter) {
      [typesArray addObject:[NSString stringWithUTF8String:iter->c_str()]];
    }
    if (extensions.size() > 1) {
      NSArray* xib = nil;

      if ([[NSBundle mainBundle] respondsToSelector:@selector(loadNibNamed:owner:topLevelObjects:)]) {
        // Introduced in Mac OS X 10.8
        [[NSBundle mainBundle] loadNibNamed:@"SavePanelFormatView" owner:nil topLevelObjects:&xib];
      } else {
        NSNib* nib = [[NSNib alloc] initWithNibNamed:@"SavePanelFormatView" bundle:nil];
        if (nib) {
          [nib instantiateNibWithOwner:nil topLevelObjects:&xib];
        }
      }
      if (xib) {
        NSView* view = nil;
        for (id obj in xib) {
          if ([obj isKindOfClass:[NSView class]]) {
            view = obj;
            for (id child in [view subviews]) {
              if ([child isKindOfClass:[NSPopUpButton class]]) {
                pulldown = child;
                break;
              }
            }
            break;
          }
        }
        if (view && pulldown) {
          [pulldown removeAllItems];
          int index = 0;
          if (descriptions.size() == extensions.size()) {
            savePanelProtocol = [[SavePanelProtocol alloc] initWithPopUpButton:pulldown extensions:typesArray];
          }
          for (auto iter = extensions.cbegin(); iter != extensions.cend(); ++iter) {
            std::string message;
            if (savePanelProtocol) {
              message = descriptions[index] + " (." + (*iter) + ")";
            } else {
              message = (*iter);
            }
            [pulldown addItemWithTitle:[NSString stringWithUTF8String:message.c_str()]];
            index++;
          }
          if (savePanelProtocol) {
            [cinderSave setDelegate:savePanelProtocol];
          }
          [cinderSave setAccessoryView:view];
        }
      }
    }
    [cinderSave setAllowedFileTypes:typesArray];
  }

  if (!initialPath.empty()) {
    NSString *directory, *file = nil;
    directory = [[NSString stringWithUTF8String:initialPath.c_str()] stringByExpandingTildeInPath];
    BOOL isDir = NO;
    if ([[NSFileManager defaultManager] fileExistsAtPath:directory isDirectory:&isDir]) {
      if (!isDir) { // a file exists at this path, so directory is its parent
        file = [directory lastPathComponent];
        directory = [directory stringByDeletingLastPathComponent];
      }
    } else {
      file = [directory lastPathComponent];
      directory = [directory stringByDeletingLastPathComponent];
    }

    [cinderSave setDirectoryURL:[NSURL fileURLWithPath:directory]];
    if (file) {
      [cinderSave setNameFieldStringValue:file];
    }
  }

  fs::path path, *pathPtr = &path;
  [cinderSave beginSheetModalForWindow:win completionHandler:^(NSInteger result) {
    if (result == NSFileHandlingPanelOKButton) {
      fs::path userPath = fs::path([[[cinderSave URL] path] UTF8String]);
      *pathPtr = userPath; // "path" is const, but we can use a pointer to it
    }
    [NSApp stopModal];
  }];
  [NSApp runModalForWindow:win];
  [cinderSave orderOut:win];
  restoreWindowContext();
  return path;
#elif _WIN32
  OPENFILENAMEA ofn;       // common dialog box structure
  char szFile[260];       // buffer for file name

  // Initialize OPENFILENAME
  ZeroMemory( &ofn, sizeof(ofn) );
  ofn.lStructSize = sizeof(ofn);
  ofn.hwndOwner = App::get()->getRenderer()->getHwnd();
  ofn.lpstrFile = szFile;

  ofn.lpstrFile[0] = '\0';
  ofn.nMaxFile = sizeof( szFile );

  char extensionStr[10000];
  if( extensions.empty() ) {
    ofn.lpstrFilter = "All\0*.*\0";
  } else {
    size_t offset = 0;

    (void)descriptions; // If !descriptions.empty(), use the descriptions (and remove this line) -- FIXME
    for( std::vector<std::string>::const_iterator strIt = extensions.begin(); strIt != extensions.end(); ++strIt ) {
      strcpy( extensionStr + offset, strIt->c_str() );
      offset += strIt->length();
      extensionStr[offset++] = '\0';
      extensionStr[offset++] = '*';
      extensionStr[offset++] = '.';
      strcpy( extensionStr + offset, strIt->c_str() );
      offset += strIt->length();
      extensionStr[offset++] = '\0';
    }

    extensionStr[offset++] = '\0';
    ofn.lpstrFilter = extensionStr;
  }
  ofn.nFilterIndex = 1;
  ofn.lpstrFileTitle = NULL;
  ofn.nMaxFileTitle = 0;
  if( initialPath.empty() ) {
    ofn.lpstrInitialDir = NULL;
  } else {
    char initialPathStr[MAX_PATH];
    strcpy( initialPathStr, initialPath.string().c_str() );
    ofn.lpstrInitialDir = initialPathStr;
  }
  ofn.Flags = OFN_SHOWHELP | OFN_OVERWRITEPROMPT;

  // Display the Open dialog box.
  std::string result;
  if( GetSaveFileNameA( &ofn ) == TRUE ) {
    std::string ext = "." + extensions[ofn.nFilterIndex-1];
    result = std::string( ofn.lpstrFile );
    if (result.find(ext) == std::string::npos) {
      result += ext;
    }
  }

  return result;
#else
  (void)descriptions;
  return FreeformApp::getSaveFilePath(initialPath, extensions);
#endif
}

int FreeformApp::saveFile()
{
  if (!mesh_) {
    return -1;
  }

  Files files;
  std::vector<std::string> file_extensions;
  std::vector<std::string> file_extension_descriptions;
  file_extension_descriptions.push_back("Polygon File Format");
  file_extensions.push_back("ply");
  file_extension_descriptions.push_back("Stereo Lithography");
  file_extensions.push_back("stl");
  file_extension_descriptions.push_back("Wavefront");
  file_extensions.push_back("obj");

  bool toggleFull = isFullScreen();
  if (toggleFull) {
    setFullScreen(false);
  }
  fs::path path = getSaveFilePathCustom("", file_extensions, file_extension_descriptions);
  if (toggleFull) {
    setFullScreen(true);
  }

  int err = -1;
  if (!path.empty()) {
    const std::string ext = path.extension().string();
    if (!ext.empty()) {
      try {
        if (ext == ".OBJ" || ext == ".obj") {
          std::ofstream file(path.c_str());
          files.saveOBJ(mesh_, file);
          file.close();
        } else if (ext == ".STL" || ext == ".stl") {
          files.saveSTL(mesh_, path.string());
        } else if (ext == ".PLY" || ext == ".ply") {
          std::ofstream file(path.c_str());
          files.savePLY(mesh_, file);
          file.close();
        }
      } catch (...) { }
    }
    err = 1;
  }
  return err; // error
}

int FreeformApp::saveScreenshot() {
  std::vector<std::string> file_extensions;
  std::vector<std::string> file_extension_descriptions;
  file_extension_descriptions.push_back("PNG");
  file_extensions.push_back("png");
  file_extension_descriptions.push_back("JPEG");
  file_extensions.push_back("jpg");

  bool toggleFull = isFullScreen();
  if (toggleFull) {
    setFullScreen(false);
  }
  fs::path path = getSaveFilePathCustom("", file_extensions, file_extension_descriptions);
  if (toggleFull) {
    setFullScreen(true);
  }

  _screenshot_path = path.string();
  return 1;
}

void FreeformApp::print3D() {
  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
  std::string fileName = boost::posix_time::to_iso_string(now);
  if (fileName.find(',') != std::string::npos) {
    fileName = fileName.substr(0, fileName.find(','));
  }
  fileName = fileName + ".ply";

  std::string filePath = AutoSave::getUserPath(fileName);

  Files files;

  // save current mesh to a temporary file
  std::ofstream file(filePath.c_str());
  if (!file) {
    return;
  }
  files.savePLY(mesh_, file);
  file.close();

  // upload to our server using HTTP PUT
  Print3D print;
  print.Upload(filePath, fileName);

  print.LaunchForm(fileName, "MyCreation", "FreeformCreation");
  ci::deleteFile(filePath);
}

#if !LM_PRODUCTION_BUILD
#pragma comment( linker, "/subsystem:\"console\" /entry:\"mainCRTStartup\"" )

int main( int argc, char * const argv[] ) {
  CrashReport cr;
  cinder::app::AppBasic::prepareLaunch();
  cinder::app::AppBasic *app = new FreeformApp;
  //cinder::app::RendererRef ren(new RendererGl());
  cinder::app::RendererRef ren(new RendererGl(RendererGl::AA_NONE));
#if _WIN32
  cinder::app::AppBasic::executeLaunch( app, ren, "FreeForm");
#else
  cinder::app::AppBasic::executeLaunch( app, ren, "FreeForm", argc, argv);
#endif
  cinder::app::AppBasic::cleanupLaunch();
  return 0;
}
#else

#if _WIN32
int WINAPI WinMain(HINSTANCE hInstance,HINSTANCE hPrevInstance,LPSTR lpCmdLine,int nCmdShow) {
  CrashReport cr;
  cinder::app::AppBasic::prepareLaunch();
  cinder::app::AppBasic *app = new FreeformApp;
  cinder::app::RendererRef ren(new RendererGl(RendererGl::AA_NONE));
  cinder::app::AppBasic::executeLaunch( app, ren, "FreeForm");
  cinder::app::AppBasic::cleanupLaunch();
  return 0;
}
#else
int main( int argc, char * const argv[] ) {
  CrashReport cr;
  cinder::app::AppBasic::prepareLaunch();
  cinder::app::AppBasic *app = new FreeformApp;
  cinder::app::RendererRef ren(new RendererGl(RendererGl::AA_NONE));
  cinder::app::AppBasic::executeLaunch( app, ren, "FreeForm", argc, argv);
  cinder::app::AppBasic::cleanupLaunch();
}
#endif

#endif
