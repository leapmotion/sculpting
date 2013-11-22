#include "StdAfx.h"
#include "Freeform.h"
#include "Files.h"
#include <time.h>

#include "cinder/app/App.h"

#include "CameraUtil.h"
#include "DebugDrawUtil.h"
#include "ReplayUtil.h"
#include "CrashReport.h"

const float CAMERA_SPEED = 0.005f;

const float MIN_CAMERA_DIST = 250.0f;
const float MAX_CAMERA_DIST = 350.0f;
const float SPHERE_RADIUS = 50000.0f;
const float MIN_FOV = 50.0f;
const float MAX_FOV = 90.0f;


//*********************************************************
FreeformApp::FreeformApp() : _environment(0), _aa_mode(MSAA), _theta(100.f), _phi(0.f), _draw_ui(true), _mouse_down(false),
  _fov(60.0f), _cam_dist(MIN_CAMERA_DIST), _exposure(1.0f), _contrast(1.2f), mesh_(0), symmetry_(false), _last_update_time(0.0),
  drawOctree_(false), _shutdown(false), _draw_background(true), _focus_point(Vector3::Zero()), _ui_zoom(1.0f), remeshRadius_(100.0f),
  _lock_camera(false), _last_load_time(0.0), _first_environment_load(true), _have_shaders(true)
{
  _camera_util = new CameraUtil();
  _debug_draw_util = &DebugDrawUtil::getInstance();
  Menu::updateSculptMult(0.0, 0.0f);
}

FreeformApp::~FreeformApp()
{
  delete _environment;
  std::unique_lock<std::mutex> lock(_mesh_mutex);
  if (mesh_) {
    delete mesh_;
  }

  delete _camera_util;
  //delete _debug_draw_util;
}

void FreeformApp::prepareSettings( Settings *settings )
{
  settings->setTitle("Freeform");
  settings->setWindowSize(1024, 768);
  //settings->disableFrameRate();

  ci::app::Window::Format fmt;
  fmt.setTitle("Freeform");
  fmt.setSize(1024, 768);
  fmt.setFullScreen(true);
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
#if _WIN32
  HMODULE instance = ::GetModuleHandle(0);
  SetClassLongPtr(getRenderer()->getHwnd(), GCLP_HICON, (LONG)::LoadImage(instance, MAKEINTRESOURCE(ICON_IDX), IMAGE_ICON, 0, 0, LR_DEFAULTSIZE));
#endif

  FreeImage_Initialise();
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

  _params->addParam( "Iso cam", &_camera_params.cameraOverrideIso, "" );
  _params->addParam( "Use triangles.", &_camera_params.userFaultyTriangles, "" );
  _params->addParam( "Draw debug lines", &_camera_params.drawDebugLines, "" );
  _params->addParam( "Draw sphere query", &_camera_params.drawSphereQueryResults, "" );

  _params->addParam( "Iso query radius", &_camera_params.isoQueryPaddingRadius, "min=10.0 max=300.0 step=10.0" );
  _params->addParam( "#clip rot iterations", &_camera_params.numRotationClipIterations, "min=1.0 max=8.0 step=1.0" );

  _params->addParam( "Iso ref dist mlt", &_camera_params.isoRefDistMultiplier, "min=0.1 max=10.0 step=0.1" );
  _params->addParam( "Grav_k", &_camera_params.grav_k, "min=0.0001 max=1.0 step=0.001" );
  _params->addParam( "Grav_n", &_camera_params.grav_n, "min=1.0 max=8.0 step=0.5" );

  _params->addParam( "Clip to isosurface", &_camera_params.clipToIsoSurface, "" );
  _params->addParam( "Clip movement", &_camera_params.clipCameraMovement, "" );
  _params->addParam( "Movement ref dist", &_camera_params.refDistForMovemement, "min=10.0 max=500.0 step=10.0" );
  _params->addParam( "Enable cone clip", &_camera_params.enableConeClipping, "" );
  _params->addParam( "Normal cone angle", &_camera_params.normalConeAngle, "min=0.0 max=1.55 step=0.05" );
  _params->addParam( "Enable max rot", &_camera_params.enableMaxReorientationRate, "" );
  _params->addParam( "Reorientation rate", &_camera_params.maxReorientationRate, "min=0.2 max=10.0 step=0.2" );
  _params->addParam( "Scale Z movement", &_camera_params.scaleZMovement, "min=0.1 max=2.0 step=0.1" );

  _params->addParam( "Collide cam", &_camera_params.preventCameraInMesh, "" );
  _params->addParam( "Enable Cam Reset", &_camera_params.enableCameraReset, "" );
  _params->addParam( "Enable Cam Orbit", &_camera_params.enableCameraOrbit, "" );

  //_params->addParam( "Iso Mult", &_camera_params.isoMultiplier, "min=1.0 max=1000.0 step=1.0" );
  _params->addParam( "Min Dist", &_camera_params.minDist, "min=1.0 max=100.0 step=1.0" );
  _params->addParam( "Max Dist", &_camera_params.maxDist, "min=100.0 max=1000.0 step=20.0" );
  _params->addParam( "Speed @ Min Dist", &_camera_params.speedAtMinDist, "min=0.01 max=1.0 step=0.1" );
  _params->addParam( "Speed @ Max Dist", &_camera_params.speedAtMaxDist, "min=1.0 max=20.0 step=1.0" );
  _params->addParam( "Pin up vector", &_camera_params.pinUpVector, "" );
  _params->addParam( "Use Sphere Query", &_camera_params.useSphereQuery, "" );
  _params->addParam( "Sphere R Mult", &_camera_params.sphereRadiusMultiplier, "min=0.0125 max=0.75 step=0.0125" );
  _params->addParam( "Crawl mode.", &_camera_params.sphereCrawlMode, "" );
  _params->addParam( "Use avg normal", &_camera_params.useAvgNormal, "" );
  _params->addParam( "Free rotation", &_camera_params.freeRotationEnabled, "" );
  _params->addParam( "Rotation ratio", &_camera_params.freeRotationRatio, "min=1.0 max=10.0 step=0.25" );
  _params->addParam( "Input multiplier", &_camera_params.inputMultiplier, "min=0.5 max=5.0 step=0.25" );
  _params->addParam( "Invert camera input", &_camera_params.invertCameraInput, "" );
  _params->addParam( "Normal correction", &_camera_params.enableNormalCorrection, "" );
  _params->addParam( "Suppers fwd rot", &_camera_params.suppresForwardRotation, "" );
  _params->addParam( "Weight normals", &_camera_params.weightNormals, "" );
  _params->addParam( "Smoothing", &_camera_params.enableSmoothing, "" );
  _params->addParam( "Smooth factor", &_camera_params.smoothingFactor, "min=0.0 max=1.0 step=0.05" );
  _params->addParam( "Clip translation", &_camera_params.clipTranslationOnFreeRotate, "" );
  _params->addParam( "Walk smoothed normals", &_camera_params.walkSmoothedNormals, "" );
  _params->addParam( "CP for edges", &_camera_params.useClosestPointForEdges, "" );
  _params->addParam( "Move along normal", &_camera_params.moveInNormalPlane, "" );
  _params->addParam( "Back snapping", &_camera_params.enableBackSnapping, "" );
  _params->addParam( "Forward check", &_camera_params.enableForwardCheckForBackSnapping, "" );
  _params->addParam( "Override normal", &_camera_params.overrideNormal, "" );
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
  _params->addParam( "Contrast", &_contrast, "min=0.33, max=3.0, step=0.01" );
  _params->addParam( "Show bloom", &_bloom_visible, "" );
  _params->addParam( "Bloom size", &_bloom_size, "min=0.0 max=4.0 step=0.01" );
  _params->addParam( "Bloom strength", &_bloom_strength, "min=0.0 max=1.0 step=0.01" );
  _params->addParam( "Bloom threshold", &_bloom_light_threshold, "min=0.0 max=2.0 step=0.01" );
  _params->addParam( "Draw Background", &_draw_background, "" );
  _params->addParam( "Remesh Radius", &remeshRadius_, "min=20, max=200, step=2.5" );
#endif

  _environment = new Environment();

  try {
    _screen_shader = gl::GlslProg( loadResource( RES_PASSTHROUGH_VERT_GLSL ), loadResource( RES_SCREEN_FRAG_GLSL ) );
    _material_shader = gl::GlslProg( loadResource( RES_MATERIAL_VERT_GLSL ), loadResource( RES_MATERIAL_FRAG_GLSL ) );
    _brush_shader = gl::GlslProg( loadResource( RES_BRUSH_VERT_GLSL ), loadResource( RES_MATERIAL_FRAG_GLSL ) );
    _wireframe_shader = gl::GlslProg( loadResource( RES_MATERIAL_VERT_GLSL ), loadResource( RES_WIREFRAME_FRAG_GLSL ) );
    _sky_shader = gl::GlslProg( loadResource( RES_SKY_VERT_GLSL ), loadResource( RES_SKY_FRAG_GLSL ) );
    _bloom_shader = gl::GlslProg( loadResource( RES_PASSTHROUGH_VERT_GLSL ), loadResource( RES_BLOOM_FRAG_GLSL ) );
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

  glEnable(GL_FRAMEBUFFER_SRGB);

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
  loadLogos();

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
}

void FreeformApp::resize()
{
  static const int BLUR_DOWNSCALE_FACTOR = 4;

  int width = getWindowWidth();
  int height = getWindowHeight();
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

  Vec3f campos;
  campos.x = cosf(_phi)*sinf(_theta)*_cam_dist;
  campos.y = sinf(_phi)*_cam_dist;
  campos.z = cosf(_phi)*cosf(_theta)*_cam_dist;
  _camera.lookAt(campos,Vec3f(0,0,0),Vec3f(0,1,0));
  _camera.setPerspective( _fov, getWindowAspectRatio(), 1.0f, 100000.f );
  setMatrices( _camera );
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

  updateCamera(float(_current_mouse_pos.x-_previous_mouse_pos.x)*CAMERA_SPEED,float(_current_mouse_pos.y-_previous_mouse_pos.y)*CAMERA_SPEED, 0.f);

  // New camera update.
  _camera_util->RecordUserInput(float(_current_mouse_pos.x-_previous_mouse_pos.x)*CAMERA_SPEED,float(_current_mouse_pos.y-_previous_mouse_pos.y)*CAMERA_SPEED, 0.f);
}

void FreeformApp::mouseWheel( MouseEvent event)
{
  //float off = event.getWheelIncrement();
  _cam_dist -= 0.1f*event.getWheelIncrement();
  _cam_dist = std::min(MAX_CAMERA_DIST, std::max(0.5f, _cam_dist));
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
  case 's': symmetry_ = !symmetry_; break;
  case 'r': sculpt_.setRemeshRadius(remeshRadius_); break;
#endif
#if __APPLE__
  case 'y': if (event.isMetaDown()) { if (mesh_ && allowUndo) { mesh_->redo(); } } break;
  case 'z': if (event.isMetaDown()) { if (mesh_ && allowUndo) { mesh_->undo(); } } break;
  case 'f': if (event.isMetaDown()) toggleFullscreen(""); break;
#else
  case 'y': if (event.isControlDown()) { if (mesh_ && allowUndo) { mesh_->redo(); } } break;
  case 'z': if (event.isControlDown()) { if (mesh_ && allowUndo) { mesh_->undo(); } } break;
#endif
  case KeyEvent::KEY_ESCAPE:
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
  if (event.getCode() == KeyEvent::KEY_RETURN) {
    if (_ui->haveExitConfirm()) {
      doQuit();
    }
  }
}

void FreeformApp::updateCamera(const float dTheta, const float dPhi, const float dFov)
{
  _theta -= dTheta;
  _phi += dPhi;
  _fov += dFov;

  if( _theta<0.f ) _theta += float(M_PI)*2.f;
  if( _theta>=M_PI*2.f ) _theta -= float(M_PI)*2.f;
  _phi = math<float>::clamp(_phi, float(-M_PI)*0.45f, float(M_PI)*0.45f);
  _fov = math<float>::clamp(_fov, 40.f, 110.f);
}

void FreeformApp::update()
{
  static int updateCount = 0;
  LM_ASSERT_IDENTICAL("\r\n\r\nUpdate frame #");
  LM_ASSERT_IDENTICAL(updateCount++);
  LM_ASSERT_IDENTICAL("\r\n");
#if LM_LOG_CAMERA_LOGIC_4
  std::cout << std::endl << "Frm#" << updateCount << " ";
#endif

  const double curTime = ci::app::getElapsedSeconds();
  LM_TRACK_CONST_VALUE(curTime);
  const float deltaTime = _last_update_time == 0.0 ? 0.0f : static_cast<float>(curTime - _last_update_time);

  static const float TIME_UNTIL_AUTOMATIC_ORBIT = 60.0f;
  static const float TIME_UNTIL_AUTOMATIC_FOV = 50.0f;
  const float timeSinceActivity = static_cast<float>(curTime - _leap_interaction->getLastActivityTime());
  LM_TRACK_CONST_VALUE(timeSinceActivity);
  _camera_params.forceCameraOrbit = timeSinceActivity > TIME_UNTIL_AUTOMATIC_ORBIT;
  const float inactivityRatio = Utilities::SmootherStep(ci::math<float>::clamp(timeSinceActivity - TIME_UNTIL_AUTOMATIC_FOV, 0.0f, TIME_UNTIL_AUTOMATIC_FOV)/TIME_UNTIL_AUTOMATIC_FOV);

  const float dTheta = deltaTime*_leap_interaction->getDThetaVel();
  const float dPhi = deltaTime*_leap_interaction->getDPhiVel();
  const float dZoom = deltaTime*_leap_interaction->getDZoomVel();
  const float logScale = _leap_interaction->getLogScale();

  updateCamera(dTheta, dPhi, dZoom);

  const double lastSculptTime = sculpt_.getLastSculptTime();
  float sculptMult = std::min(1.0f, static_cast<float>(fabs(curTime - lastSculptTime))/0.5f);

  //_camera_util->RecordUserInput(Vector3(_leap_interaction->getPinchDeltaFromLastCall().ptr()), _leap_interaction->isPinched());
  _camera_util->RecordUserInput(sculptMult*dTheta, sculptMult*dPhi, sculptMult*dZoom);

  static const float LOWER_BOUND = 1.0f;
  static const float UPPER_BOUND = 1.3f;

  _ui_zoom = ci::math<float>::clamp(_ui_zoom + 0.75f*logScale, LOWER_BOUND, UPPER_BOUND);
  const float ratio = (_ui_zoom - LOWER_BOUND) / (UPPER_BOUND - LOWER_BOUND);
  _ui->setZoomFactor(Utilities::SmootherStep(ratio)*(UPPER_BOUND - LOWER_BOUND) + LOWER_BOUND);

  _fov_modifier.Update((-_ui->getZoomFactor() * 20.0f) + (-inactivityRatio * 5.0f), curTime, 0.95f);
  _ui->update(_leap_interaction, &sculpt_);
  _ui->handleSelections(&sculpt_, _leap_interaction, this, mesh_);

  float blend = (_fov-MIN_FOV)/(MAX_FOV-MIN_FOV);
  _cam_dist = blend*(MAX_CAMERA_DIST-MIN_CAMERA_DIST) + MIN_CAMERA_DIST;

  // Calculate initial camera position
  Vec3f campos;
  campos.x = cosf(_phi)*sinf(_theta)*_cam_dist;
  campos.y = sinf(_phi)*_cam_dist;
  campos.z = cosf(_phi)*cosf(_theta)*_cam_dist;

  // if-conditioning this will disable mouse-based movement, untill we actually handle camera update
  //if (!mesh_ || _camera_util->state == CameraUtil::STATE_INVALID) {

  if (_camera_util->state == CameraUtil::STATE_INVALID || !mesh_) {
    // Init camera
    _camera_util->SetFromStandardCamera(Vector3(campos.ptr()), Vector3(0,0,0), _cam_dist); // up vector assumed to be Vec3f(0,1,0)
    _camera_util->debugDrawUtil = _debug_draw_util;
  }
  
  lmTransform tCamera;
  {
    // Hack -- lock access to mesh rotation for a moment
    std::unique_lock<std::mutex> lock(_mesh_update_rotation_mutex);
    tCamera = _camera_util->GetCameraInWorldSpace();
  }

  campos = ToVec3f(tCamera.translation);
  Vector3 up = tCamera.rotation * Vector3::UnitY();
  Vector3 to = tCamera.translation + tCamera.rotation * Vector3::UnitZ() * -200.0f;

  //_focus_point = to;
  Matrix4x4 trans = mesh_->getTransformation();
  Vector4 temp;
  {
    std::unique_lock<std::mutex> lock(_camera_util->referencePointMutex);
    temp << _camera_util->referencePoint.position, 1.0;
  }
  _focus_point = (trans * temp).head<3>();
  _focus_radius = _camera_util->GetSphereQueryRadius();

  _campos_smoother.Update(campos, curTime, 0.95f);
  _lookat_smoother.Update(ToVec3f(to), curTime, 0.95f);
  _up_smoother.Update(ToVec3f(up), curTime, 0.95f);

  // Update camera
  _camera.lookAt(_campos_smoother.value, _lookat_smoother.value, _up_smoother.value.normalized());
  _camera.setPerspective( 80.0f + _fov_modifier.value, getWindowAspectRatio(), 1.0f, 100000.f );
  _camera.getProjectionMatrix();

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
#if ! LM_DISABLE_THREADING_AND_ENVIRONMENT
    bool suppress = _environment->getLoadingState() != Environment::LOADING_STATE_NONE;
    suppress = suppress || (curTime - _last_load_time) < BRUSH_DISABLE_TIME_AFTER_LOAD;
#else 
    bool suppress = false;
#endif 
    bool haveFrame;
    try {
      haveFrame = _leap_interaction->processInteraction(_listener, getWindowAspectRatio(), _camera.getModelViewMatrix(), _camera.getProjectionMatrix(), getWindowSize(), _camera_util->referenceDistance, Utilities::DEGREES_TO_RADIANS*60.0f, suppress);
    } catch (...) {
      haveFrame = false;
    }

    if (haveFrame) {
      const double lastSculptTime = sculpt_.getLastSculptTime();

      std::unique_lock<std::mutex> lock(_mesh_mutex);
      if (mesh_) {
        {
          std::unique_lock<std::mutex> lock(_mesh_update_rotation_mutex);
          mesh_->updateRotation(curTime);
        }
        if (!_lock_camera && fabs(curTime - lastSculptTime) > 0.25) {
          _camera_util->UpdateCamera(mesh_, &_camera_params);
        }
        if (!_ui->tutorialActive() || _ui->toolsSlideActive()) {
          sculpt_.applyBrushes(curTime, symmetry_, &_auto_save);
          _camera_util->timeOfLastScupt = sculpt_.getLastSculptTime();
        }
      }
      _mesh_update_counter.Update(ci::app::getElapsedSeconds());
    } else if (mesh_) {
      // Allow camera movement when leap is disconnected
      std::unique_lock<std::mutex> lock(_mesh_mutex);
      _camera_util->UpdateCamera(mesh_, &_camera_params);
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
  float blend = (_fov-(MIN_FOV+FOV_TOLERANCE))/(MAX_FOV-MIN_FOV-(2*FOV_TOLERANCE));
  blend = Utilities::SmootherStep(std::sqrt(math<float>::clamp(blend)));
  depth_min = depthMinZoomed*(1.0f-blend) + depthMinOut*blend;

  // disable depth and culling for rendering skybox
  gl::disableDepthRead();
  gl::disableDepthWrite();
  glDisable(GL_CULL_FACE);

  _screen_fbo.bindFramebuffer();
  setViewport( _screen_fbo.getBounds() );
  clear();
  setMatrices( _camera );
  if (_draw_background) {
    // draw color pass of skybox
    _environment->bindCubeMap(Environment::CUBEMAP_SKY, 0);
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

  _environment->bindCubeMap(Environment::CUBEMAP_IRRADIANCE, 0);
  _environment->bindCubeMap(Environment::CUBEMAP_RADIANCE, 1);

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
    _material_shader.uniform( "ambientFactor", _material.ambientFactor);
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
    Menu::updateSculptMult(curTime, (curTime - lastSculptTime) < 0.5 ? 0.25f : 1.0f);
  }

  if (_camera_util->params.drawDebugLines) {
    _wireframe_shader.bind();
    _wireframe_shader.uniform( "transform", transform );
    _wireframe_shader.uniform( "transformit", transformit );
    _debug_draw_util->FlushDebugPrimitives(&_wireframe_shader);
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
    if (symmetry_) {
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
    if (symmetry_) {
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

  const Environment::LoadingState loadingState = _environment->getLoadingState();
  const double curTime = ci::app::getElapsedSeconds();
  const float timeSinceStateChange = static_cast<float>(curTime - _environment->getLastStateChangeTime());

  if (!_environment->haveEnvironment()) {
    Menu::updateSculptMult(curTime, 0.0f);
  }

  float exposureMult = 1.0f;
  if (loadingState == Environment::LOADING_STATE_NONE) {
    const float ratio = math<float>::clamp(timeSinceStateChange/LOADING_LIGHTEN_TIME);
    exposureMult = Utilities::SmootherStep(ratio*ratio);
  } else if (loadingState == Environment::LOADING_STATE_LOADING) {
    const float ratio = 1.0f - math<float>::clamp(timeSinceStateChange/LOADING_DARKEN_TIME);
    exposureMult = Utilities::SmootherStep(ratio*ratio);
  } else if (loadingState == Environment::LOADING_STATE_DONE_LOADING) {
    exposureMult = 0.0f;
    _environment->finishLoading();
    _loading_thread.join();
    _loading_thread = std::thread(&Environment::beginProcessing, _environment);
    Environment::EnvironmentInfo* info = Environment::getEnvironmentInfoFromString(_environment->getPendingEnvironmentString());
    if (info) {
      _bloom_strength = info->_bloom_strength;
      _bloom_light_threshold = info->_bloom_threshold;
      _exposure = info->_exposure;
      _contrast = info->_contrast;
    }
    _first_environment_load = false;
  } else if (loadingState == Environment::LOADING_STATE_PROCESSING) {
    exposureMult = 0.0f;
  } else if (loadingState == Environment::LOADING_STATE_DONE_PROCESSING) {
    exposureMult = 0.0f;
    _environment->finishProcessing();
    _loading_thread.join();
  }

  if (_first_environment_load) {
    exposureMult = 0.0f;
  }

  return exposureMult;
}

void FreeformApp::draw() {
  clear();

  const double curTime = ci::app::getElapsedSeconds();
  const float exposureMult = checkEnvironmentLoading();

  const ci::Vec2i size = getWindowSize();
  const ci::Area bounds = getWindowBounds();
  const ci::Vec2f center = getWindowCenter();

  if (exposureMult > 0.0f && _have_shaders) {
    renderSceneToFbo(_camera);

    GLBuffer::checkError("After FBO");
    GLBuffer::checkFrameBufferStatus("After FBO");
  }

  glDisable(GL_CULL_FACE);
  disableDepthRead();
  disableDepthWrite();

  if (exposureMult > 0.0f && _have_shaders) {
    if (_bloom_visible) {
      createBloom();
    }

    const float width = static_cast<float>(size.x);
    const float height = static_cast<float>(size.y);

    setViewport( _screen_fbo.getBounds() );
    setMatricesWindow( getWindowWidth(), getWindowHeight(), false);
    const float overlayMult = (_ui->aboutActive() || _ui->tutorialActive()) ? 0.65f : 1.0f;

    _screen_fbo.bindTexture(0);
    _vertical_blur_fbo.bindTexture(1);
    _screen_shader.bind();
    _screen_shader.uniform( "color_texture", 0 );
    _screen_shader.uniform( "bloom_texture", 1 );
    _screen_shader.uniform( "depth_texture", 2 );
    _screen_shader.uniform( "width", width );
    _screen_shader.uniform( "height", height );
    _screen_shader.uniform( "exposure", _exposure * exposureMult * overlayMult);
    _screen_shader.uniform( "contrast", _contrast );
    _screen_shader.uniform( "bloom_strength", _bloom_strength * static_cast<float>(_bloom_visible) );
    _screen_shader.uniform( "vignette_radius", static_cast<float>(0.9f * sqrt((width/2)*(width/2) + (height/2)*(height/2))) );
    _screen_shader.uniform( "vignette_strength", 0.75f );
    gl::drawSolidRect(Rectf(0.0f,0.0f,width,height));
    _screen_shader.unbind();
    _screen_fbo.unbindTexture();
    _vertical_blur_fbo.unbindTexture();
    
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

    setMatricesWindow( size );
    setViewport( bounds );

    if (_draw_ui) {
      _ui->draw(exposureMult);
    }

    if (_environment->haveEnvironment()) {
      _ui->drawTutorialSlides(exposureMult);
      _ui->drawAbout(exposureMult);
    }

    if (!_listener.isConnected()) {
      _ui->drawDisconnected();
    }

    GLBuffer::checkError("After UI");
    GLBuffer::checkFrameBufferStatus("After UI");
  }

  setMatricesWindow( size );
  setViewport( bounds );

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
  
  if (!_have_shaders) {
    _ui->drawShaderError();
  }

  GLBuffer::checkError("After logo");
  GLBuffer::checkFrameBufferStatus("After logo");

  enableDepthRead();
  enableDepthWrite();

#if !LM_PRODUCTION_BUILD
  _params->draw(); // draw the interface
#endif
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

  _ui->setTutorialTextures(ci::gl::Texture(loadImage(loadResource(RES_TUTORIAL_1))),
                           ci::gl::Texture(loadImage(loadResource(RES_TUTORIAL_2))),
                           ci::gl::Texture(loadImage(loadResource(RES_TUTORIAL_3))),
                           ci::gl::Texture(loadImage(loadResource(RES_TUTORIAL_4))));

  _ui->setAboutTexture(ci::gl::Texture(loadImage(loadResource(RES_CREDITS))));
}

void FreeformApp::loadShapes() {
  ci::DataSourceRef ball = loadResource(RES_BALL_OBJ);
  ci::DataSourceRef can = loadResource(RES_CAN_OBJ);
  ci::DataSourceRef donut = loadResource(RES_DONUT_OBJ);
  ci::DataSourceRef sheet = loadResource(RES_SHEET_OBJ);
  ci::DataSourceRef cube = loadResource(RES_CUBE_OBJ);
  ci::Buffer& ballBuf = ball->getBuffer();
  ci::Buffer& canBuf = can->getBuffer();
  ci::Buffer& donutBuf = donut->getBuffer();
  ci::Buffer& sheetBuf = sheet->getBuffer();
  ci::Buffer& cubeBuf = cube->getBuffer();
  shapes_[BALL] = std::string((char*)ballBuf.getData(), ballBuf.getDataSize());
  shapes_[CAN] = std::string((char*)canBuf.getData(), canBuf.getDataSize());
  shapes_[DONUT] = std::string((char*)donutBuf.getData(), donutBuf.getDataSize());
  shapes_[SHEET] = std::string((char*)sheetBuf.getData(), sheetBuf.getDataSize());
  shapes_[CUBE] = std::string((char*)cubeBuf.getData(), cubeBuf.getDataSize());
}

void FreeformApp::loadLogos() {
  _logo_on_black = ci::gl::Texture(loadImage(loadResource(RES_LOGO_ON_BLACK)));
  _logo_on_image = ci::gl::Texture(loadImage(loadResource(RES_LOGO_ON_IMAGE)));
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
    return FreeformApp::MID;
  }
  int model_number = atoi(render_string.substr(it - render_string.begin()).c_str());

  if (render_string.find("GeForce") != std::string::npos) {
    if (model_number > 1000) {
      if (render_string.find("M OpenGL") != std::string::npos) {
        // mobile GPU
        if (model_number < 9000) {
          return FreeformApp::LOW;
        }
      }
      if (model_number < 6000) {
        // GeForce FX (5000) series or older, DirectX 8
        return FreeformApp::LOW;
      }
      // GeForce 6000, 7000 series
      if (model_number < 8000) {
        // GeForce 6000 series first to support DirectX 9
        return FreeformApp::MID;
      }
    }
    // GeForce 8000 == 9000, 200, 300 (GT200), 400 (Fermi)... 600 or higher series
    return FreeformApp::HIGH;
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

void FreeformApp::setSymmetry(bool symmetry) {
  symmetry_ = symmetry;
}

void FreeformApp::toggleSymmetry() {
  symmetry_ = !symmetry_;
}

void FreeformApp::setEnvironment(const std::string& str) {
  if (!_environment || _environment->getLoadingState() != Environment::LOADING_STATE_NONE) {
    return;
  }
#if __APPLE__
  if (_loading_thread.joinable())
  {
    _loading_thread.detach();
  }
#endif
  _loading_thread = std::thread(&Environment::beginLoading, _environment, str);
}

void FreeformApp::toggleSound() {
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
  std::stringstream ss(shapes_[shape]);
  Files files;
  Mesh* newMesh = files.loadOBJ(ss);
  if (!newMesh) {
    return 0;
  }
  std::unique_lock<std::mutex> lock(_mesh_mutex);
  Mesh::stateMask_= 1;
  Vertex::tagMask_ = 1;
  Vertex::sculptMask_ = 1;
  Triangle::tagMask_ = 1;
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

#if _WIN32
fs::path getSaveFilePathCustom( const fs::path &initialPath, const std::vector<std::string>& extensions )
{
	OPENFILENAMEA ofn;       // common dialog box structure
	char szFile[260];       // buffer for file name

	// Initialize OPENFILENAME
	ZeroMemory( &ofn, sizeof(ofn) );
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = App::get()->getRenderer()->getHwnd();
	ofn.lpstrFile = szFile;

	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = sizeof( szFile );

	if( extensions.empty() ) {
		ofn.lpstrFilter = "All\0*.*\0";
	} else {
		char extensionStr[10000];
		size_t offset = 0;

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
}
#endif

int FreeformApp::saveFile()
{
  if (!mesh_) {
    return -1;
  }

#if 0
  const Aabb& boundingBox = mesh_->getOctree()->getAabbSplit();
  std::cout << "min: " << boundingBox.min_.transpose() << std::endl;
  std::cout << "max: " << boundingBox.max_.transpose() << std::endl;
#endif

  Files files;
  std::vector<std::string> file_extensions;
  file_extensions.push_back("ply");
  file_extensions.push_back("stl");
  file_extensions.push_back("obj");

  bool toggleFull = isFullScreen();
  if (toggleFull) {
    setFullScreen(false);
  }
#if _WIN32
  fs::path path = getSaveFilePathCustom("", file_extensions);
#else
  fs::path path = getSaveFilePath("", file_extensions);
#endif
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
        } else if (ext == ".STL" || ext == ".stl") {
          files.saveSTL(mesh_, path.string());
        } else if (ext == ".PLY" || ext == ".ply") {
          std::ofstream file(path.c_str());
          files.savePLY(mesh_, file);
        }
      } catch (...) { }
    }
    err = 1;
  }
  return err; // error
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
//CINDER_APP_NATIVE( FreeformApp, RendererGl )
CINDER_APP_NATIVE( FreeformApp, RendererGl(RendererGl::AA_NONE) )
#endif
