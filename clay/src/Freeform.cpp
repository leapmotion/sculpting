#include "StdAfx.h"
#include "Freeform.h"
#include "Files.h"
#include <time.h>

#include "cinder/app/App.h"

#include "CameraUtil.h"
#include "DebugDrawUtil.h"

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
  _lock_camera(false)
{
  _camera_util = new CameraUtil();
  _debug_draw_util = &DebugDrawUtil::getInstance();
  Menu::updateSculptMult(0.0, 0.0f);
}

FreeformApp::~FreeformApp()
{
  _shutdown = true;
  _mesh_thread.join();

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
  ci::app::Window::Format fmt;
  fmt.setTitle("Freeform");
  fmt.setSize(1280, 960);
  settings->prepareWindow(fmt);

  settings->setWindowSize(1280, 960);
  //settings->disableFrameRate();
  //enableVerticalSync(false);
}

void FreeformApp::toggleFullscreen(const std::string& str)
{
  bool full = isFullScreen();
  setFullScreen(!full);
}

void FreeformApp::setup()
{
  FreeImage_Initialise();
  enableAlphaBlending();
  enableDepthRead();
  enableDepthWrite();
  glCullFace( GL_BACK );
  glEnable(GL_CULL_FACE);

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

  _params->addParam( "Draw debug lines", &_camera_params.drawDebugLines, "" );
  _params->addParam( "Draw sphere query", &_camera_params.drawSphereQueryResults, "" );

  _params->addParam( "Iso cam", &_camera_params.cameraOverrideIso, "" );
  _params->addParam( "Iso ref dist mlt", &_camera_params.isoRefDistMultiplier, "min=0.1 max=10.0 step=0.1" );
  _params->addParam( "Grav_k", &_camera_params.grav_k, "min=0.0001 max=1.0 step=0.001" );
  _params->addParam( "Grav_n", &_camera_params.grav_n, "min=1.0 max=8.0 step=0.5" );
  _params->addParam( "#clip rot iterations", &_camera_params.numRotationClipIterations, "min=1.0 max=8.0 step=1.0" );
  _params->addParam( "Clip to isosurface", &_camera_params.clipToIsoSurface, "" );
  _params->addParam( "Clip movement", &_camera_params.clipCameraMovement, "" );
  _params->addParam( "Movement ref dist", &_camera_params.refDistForMovemement, "min=10.0 max=500.0 step=10.0" );
  _params->addParam( "Enable cone clip", &_camera_params.enableConeClipping, "" );
  _params->addParam( "Normal cone angle", &_camera_params.normalConeAngle, "min=0.0 max=1.55 step=0.05" );
  _params->addParam( "Enable max rot", &_camera_params.enableMaxReorientationRate, "" );
  _params->addParam( "Reorientation rate", &_camera_params.maxReorientationRate, "min=0.2 max=10.0 step=0.2" );
  _params->addParam( "Scale Z movement", &_camera_params.scaleZMovement, "min=0.1 max=2.0 step=0.1" );

  _params->addParam( "Use Iso Normal", &_camera_params.useIsoNormal, "" );

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
  _params->addParam( "Use triangles.", &_camera_params.userFaultyTriangles, "" );
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
    quit();
  }

  sculpt_.clearBrushes();

  _machine_speed = parseRenderString(std::string((char*)glGetString(GL_RENDERER)));
  _bloom_visible = _machine_speed > FreeformApp::LOW;
  _aa_mode = _machine_speed > FreeformApp::LOW ? FreeformApp::MSAA : FreeformApp::NONE;

  // set mesh detail depending on machine specs
  static const float LOW_DETAIL_LEVEL = 0.9f;
  static const float MEDIUM_DETAIL_LEVEL = 0.93f;
  static const float HIGH_DETAIL_LEVEL = 0.96f;
  float detail = LOW_DETAIL_LEVEL;
  if (_machine_speed == FreeformApp::MID) {
    detail = MEDIUM_DETAIL_LEVEL;
  } else if (_machine_speed == FreeformApp::HIGH) {
    detail = HIGH_DETAIL_LEVEL;
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

#if __APPLE__
  if (_mesh_thread.joinable())
  {
    _mesh_thread.detach();
  }
#endif

  if (AutoSave::isFirstRun()) {
    _ui->forceDrawTutorialMenu();
  }

  _mesh_thread = std::thread(&FreeformApp::updateLeapAndMesh, this);

  _auto_save.start();

  loadIcons();
  loadShapes();
  loadLogos();

  if (_auto_save.haveAutoSave()) {
    std::ifstream file(_auto_save.getAutoSavePath());
    Files files;
    Mesh* mesh = files.loadPLY(file);
    std::unique_lock<std::mutex> lock(_mesh_mutex);
    mesh_ = mesh;
    if (mesh_) {
      mesh_->startPushState();
    }
    sculpt_.setMesh(mesh_);
  } else {
    loadShape(BALL);
  }
}

void FreeformApp::shutdown() {
  _shutdown = true;
  FreeImage_DeInitialise();
}

void FreeformApp::resize()
{
  static const int DOWNSCALE_FACTOR = 4;

  int width = getWindowWidth();
  int height = getWindowHeight();
  width = std::max(DOWNSCALE_FACTOR, width);
  height = std::max(DOWNSCALE_FACTOR, height);

  // FBOs with no depth buffer and bilinear sampling

  Fbo::Format formatNoDepthLinear;
  formatNoDepthLinear.setColorInternalFormat(GL_RGB32F_ARB);
  formatNoDepthLinear.setMinFilter(GL_LINEAR);
  formatNoDepthLinear.setMagFilter(GL_LINEAR);
  formatNoDepthLinear.enableDepthBuffer(false);

  _horizontal_blur_fbo = Fbo( width/DOWNSCALE_FACTOR, height/DOWNSCALE_FACTOR, formatNoDepthLinear );
  _vertical_blur_fbo = Fbo( width/DOWNSCALE_FACTOR, height/DOWNSCALE_FACTOR, formatNoDepthLinear );

  // FBOs with depth buffer

  Fbo::Format formatWithDepth;
  formatWithDepth.setColorInternalFormat(GL_RGB32F_ARB);
  if (_aa_mode == MSAA) {
    formatWithDepth.setSamples(4);
  }

  _screen_fbo = Fbo( width, height, formatWithDepth );

  Vec3f campos;
  campos.x = cosf(_phi)*sinf(_theta)*_cam_dist;
  campos.y = sinf(_phi)*_cam_dist;
  campos.z = cosf(_phi)*cosf(_theta)*_cam_dist;
  _camera.lookAt(campos,Vec3f(0,0,0),Vec3f(0,1,0));
  _camera.setPerspective( _fov, getWindowAspectRatio(), 1.0f, 100000.f );
  setMatrices( _camera );
  _ui->setWindowSize( Vec2i(width, height) );

  glEnable(GL_FRAMEBUFFER_SRGB);
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
  switch( event.getChar() )
  {
#if !LM_PRODUCTION_BUILD
  case KeyEvent::KEY_ESCAPE: quit(); break;
  case 'u': _draw_ui = !_draw_ui; break;
  case 'o': drawOctree_ = !drawOctree_; break;
  case 's': symmetry_ = !symmetry_; break;
  case 'r': sculpt_.setRemeshRadius(remeshRadius_); break;
#endif
  case 'y': if (event.isControlDown()) { if (mesh_) { mesh_->redo(); } } break;
  case 'z': if (event.isControlDown()) { if (mesh_) { mesh_->undo(); } } break;
  case 'c': _lock_camera = !_lock_camera; break;
  case 'f': { std::string dummyString; toggleFullscreen(dummyString); } break;
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
  const double curTime = ci::app::getElapsedSeconds();
  const float deltaTime = _last_update_time == 0.0 ? 0.0f : static_cast<float>(curTime - _last_update_time);

  static const float TIME_UNTIL_AUTOMATIC_ORBIT = 60.0f;
  static const float TIME_UNTIL_AUTOMATIC_FOV = 50.0f;
  const float timeSinceActivity = static_cast<float>(curTime - _leap_interaction->getLastActivityTime());
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
  
  std::unique_lock<std::mutex> lock(_camera_util->mutex);
  lmTransform tCamera = _camera_util->GetCameraInWorldSpace();
  lock.unlock();

  campos = ToVec3f(tCamera.translation);
  Vector3 up = tCamera.rotation * Vector3::UnitY();
  Vector3 to = tCamera.translation + tCamera.rotation * Vector3::UnitZ() * -200.0f;

  //_focus_point = to;
  Matrix4x4 trans = mesh_->getTransformation();
  Vector4 temp;
  temp << _camera_util->referencePoint.position, 1.0;
  _focus_point = (trans * temp).head<3>();
  _focus_radius = _camera_util->GetSphereQueryRadius();

  _campos_smoother.Update(campos, curTime, 0.95f);
  _lookat_smoother.Update(ToVec3f(to), curTime, 0.95f);
  _up_smoother.Update(ToVec3f(up), curTime, 0.95f);

  // Update camera
  _camera.lookAt(_campos_smoother.value, _lookat_smoother.value, _up_smoother.value.normalized());
  _camera.setPerspective( 80.0f + _fov_modifier.value, getWindowAspectRatio(), 1.0f, 100000.f );
  _camera.getProjectionMatrix();

#if 0
  // Force load-file prompt
  static bool init = true;
  if (init && ci::app::getElapsedSeconds() > 1) {
    init = false;
    loadFile();
  }
#endif

  _last_update_time = curTime;
}

void FreeformApp::updateLeapAndMesh() {
  while (!_shutdown) {
    bool suppress = _environment->getLoadingState() != Environment::LOADING_STATE_NONE;
    bool haveFrame = _leap_interaction->processInteraction(_listener, getWindowAspectRatio(), _camera.getModelViewMatrix(), _camera.getProjectionMatrix(), getWindowSize(), _camera_util->referenceDistance, Utilities::DEGREES_TO_RADIANS*60.0f, suppress);

    if (haveFrame) {
      const double curTime = ci::app::getElapsedSeconds();
      const double lastSculptTime = sculpt_.getLastSculptTime();

      std::unique_lock<std::mutex> lock(_mesh_mutex);
      if (mesh_) {
        mesh_->updateRotation(curTime);
        if (!_lock_camera && fabs(curTime - lastSculptTime) > 0.25) {
          _camera_util->UpdateCamera(mesh_, &_camera_params);
        }
        sculpt_.applyBrushes(curTime, symmetry_, &_auto_save);
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
  if (_draw_background) {
    // draw color pass of skybox
    _environment->bindCubeMap(Environment::CUBEMAP_SKY, 0);
    _sky_shader.bind();
    _sky_shader.uniform("cubemap", 0);
    setViewport( _screen_fbo.getBounds() );
    clear();
    setMatrices( _camera );
    gl::drawSphere(Vec3f::zero(), SPHERE_RADIUS, 40);
    _sky_shader.unbind();
    _environment->unbindCubeMap(0);
  } else {
    clear();
  }

  setMatrices( _camera );

  // enable depth and culling for rendering mesh
  glEnable(GL_CULL_FACE);
  gl::enableDepthRead();
  gl::enableDepthWrite();
  enableAlphaBlending();

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

  if (mesh_) {
    mesh_->updateGPUBuffers();
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
}

void FreeformApp::createBloom()
{
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

void FreeformApp::draw()
{
  static const float LOADING_DARKEN_TIME = 0.75f;
  static const float LOADING_LIGHTEN_TIME = 1.5f;
  float exposure_mult = 1.0f;
  const Environment::LoadingState loading_state = _environment->getLoadingState();
  const float loading_time = _environment->getTimeSinceLoadingStateChange();
  if (!_environment->haveEnvironment()) {
    exposure_mult = 0.0f;
    Menu::updateSculptMult(ci::app::getElapsedSeconds(), 0.0f);
  } else if (loading_state == Environment::LOADING_STATE_LOADING) {
    float mult = 1.0f - math<float>::clamp(loading_time/LOADING_DARKEN_TIME);
    exposure_mult *= mult;
  } else if (loading_state == Environment::LOADING_STATE_DONE_LOADING && loading_time > LOADING_DARKEN_TIME) {
    _environment->transitionComplete();
    exposure_mult = 0.0f;
    Environment::EnvironmentInfo* info = Environment::getEnvironmentInfoFromString(_environment->getCurEnvironmentString());
    if (info) {
      _bloom_strength = info->_bloom_strength;
      _bloom_light_threshold = info->_bloom_threshold;
      _exposure = info->_exposure;
      _contrast = info->_contrast;
    }
  } else if (loading_state == Environment::LOADING_STATE_PROCESSING) {
    exposure_mult = 0.0f;
  } else if (loading_state == Environment::LOADING_STATE_NONE) {
    exposure_mult *= math<float>::clamp(loading_time/LOADING_LIGHTEN_TIME);
  }

  exposure_mult = Utilities::SmootherStep(exposure_mult*exposure_mult);

  clear();

  const float width = static_cast<float>(getWindowBounds().getWidth());
  const float height = static_cast<float>(getWindowBounds().getHeight());

  if (exposure_mult > 0.0f) {
    renderSceneToFbo(_camera);

    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);

    if (_bloom_visible) {
      createBloom();
    }

    setViewport( _screen_fbo.getBounds() );
    setMatricesWindow( getWindowWidth(), getWindowHeight(), false);
    const float tutorialMult = _ui->tutorialActive() ? 0.65f : 1.0f;

    _screen_fbo.bindTexture(0);
    _vertical_blur_fbo.bindTexture(1);
    _screen_shader.bind();
    _screen_shader.uniform( "color_texture", 0 );
    _screen_shader.uniform( "bloom_texture", 1 );
    _screen_shader.uniform( "depth_texture", 2 );
    _screen_shader.uniform( "width", width );
    _screen_shader.uniform( "height", height );
    _screen_shader.uniform( "exposure", _exposure * exposure_mult * tutorialMult);
    _screen_shader.uniform( "contrast", _contrast );
    _screen_shader.uniform( "bloom_strength", _bloom_strength * static_cast<float>(_bloom_visible) );
    _screen_shader.uniform( "vignette_radius", static_cast<float>(0.9f * sqrt((width/2)*(width/2) + (height/2)*(height/2))) );
    _screen_shader.uniform( "vignette_strength", 0.75f );
    gl::drawSolidRect(Rectf(0.0f,0.0f,width,height));
    _screen_shader.unbind();
    _screen_fbo.unbindTexture();
    _vertical_blur_fbo.unbindTexture();

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

    if (_draw_ui) {
      glDisable(GL_CULL_FACE);
      disableDepthRead();
      disableDepthWrite();
      setMatricesWindow( getWindowSize() );
      setViewport( getWindowBounds() );
      _ui->draw();
      enableDepthRead();
      enableDepthWrite();
    }

    if (_environment->haveEnvironment()) {
      _ui->drawTutorialSlides(exposure_mult);
    }
  }

  glPushMatrix();
  const ci::Vec2i size = getWindowSize();
  const ci::Area bounds = getWindowBounds();
  const ci::Vec2f center = getWindowCenter();
  setMatricesWindow( size );
  setViewport( bounds );
  const float aspect = _logo_on_black.getAspectRatio();
  const float yOffset = 0.0f;
  static const float LOGO_SCALE = 0.8f;
  float halfWidth = LOGO_SCALE*size.x/2;
  float halfHeight = halfWidth / aspect;
  ci::Rectf area(center.x - halfWidth, center.y - halfHeight + yOffset, center.x + halfWidth, center.y + halfHeight + yOffset);
  ci::ColorA color(1.0f, 1.0f, 1.0f, 1.0f - exposure_mult);
  ci::gl::color(color);
  ci::gl::draw(_logo_on_black, area);
  glPopMatrix();

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
                           ci::gl::Texture(loadImage(loadResource(RES_TUTORIAL_3))));
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
  // would be cleaner if I used Boost.Spirit
  std::string::const_iterator it = render_string.begin();
  for (; it != render_string.end(); it++) {
    if (isdigit(*it)) {
      break;
    }
  }
  assert(it != render_string.end());
  int model_number = atoi(render_string.substr(it - render_string.begin()).c_str());
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
  _loading_thread = std::thread(&Environment::setEnvironment, _environment, str);
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
  fs::path path = getOpenFilePath("", file_extensions);

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
      std::ifstream stream;
      if (stream) {
        if (ext == ".OBJ" || ext == ".obj") {
          stream.open(pathString);
          mesh = files.loadOBJ(stream);
        } else if (ext == ".STL" || ext == ".stl") {
          stream.open(pathString, std::ios::binary);
          mesh = files.loadSTL(stream);
        } else if (ext == ".3DS" || ext == ".3ds") {
          stream.open(pathString, std::ios::binary);
          mesh = files.load3DS(stream);
        } else if (ext == ".PLY" || ext == ".ply") {
          stream.open(pathString);
          mesh = files.loadPLY(stream);
        }
        stream.close();
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
        mesh_->startPushState();
        _last_loaded_file = pathString;
      } else {
        _last_loaded_file = "";
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

#if _WIN32
  fs::path path = getSaveFilePathCustom("", file_extensions);
#else
  fs::path path = getSaveFilePath("", file_extensions);
#endif

  int err = -1;
  if (!path.empty()) {
    const std::string ext = path.extension().string();
    if (!ext.empty()) {
      if (ext == ".OBJ" || ext == ".obj") {
        std::ofstream file(path.c_str());
        files.saveOBJ(mesh_, file);
      } else if (ext == ".STL" || ext == ".stl") {
        files.saveSTL(mesh_, path.string());
      } else if (ext == ".PLY" || ext == ".ply") {
        std::ofstream file(path.c_str());
        files.savePLY(mesh_, file);
      }
    }
    err = 1;
  }
  return err; // error
}

#if !LM_PRODUCTION_BUILD
#pragma comment( linker, "/subsystem:\"console\" /entry:\"mainCRTStartup\"" )

int main( int argc, char * const argv[] ) {
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
