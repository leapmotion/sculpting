#include "StdAfx.h"
#include "ThreeForm.h"
#include "Files.h"

#define FREEIMAGE_LIB
#include "FreeImage.h"

#include "CameraUtil.h"

const float CAMERA_SPEED = 0.005f;

const float MIN_CAMERA_DIST = 250.0f;
const float MAX_CAMERA_DIST = 350.0f;
const float SPHERE_RADIUS = 50000.0f;
const float MIN_FOV = 50.0f;
const float MAX_FOV = 90.0f;

// Debug output
std::vector<Vector3> debugPoints;
std::vector<Vector3> debugLines;
std::vector<Vector3> debugTriangles;


//*********************************************************
ClayDemoApp::ClayDemoApp()
	: _environment(0)
	, _theta(100.f)
	, _phi(0.f)
	, _draw_ui(true)
	, _mouse_down(false)
	, _fov(60.0f)
	, _cam_dist(MIN_CAMERA_DIST)
	, _exposure(1.0f)
	, _use_ao(false)
	, _only_ao(false)
	, _contrast(1.2f)
	, mesh_(0)
	, symmetry_(false)
  , _last_update_time(0.0)
  , drawOctree_(false)
  , _use_fxaa(true)
{
  _camera_util = new CameraUtil();
}

ClayDemoApp::~ClayDemoApp()
{
	delete _environment;
	if (mesh_) {
		delete mesh_;
	}

  delete _camera_util;
}

void ClayDemoApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 800, 600 );
	//settings->setFrameRate( 60.0f );
	settings->setTitle( "Clay Demo" );
	//enableVerticalSync(true);
}

int ClayDemoApp::loadFile()
{
	// *** open mesh file ***
	std::vector<std::string> file_extensions;
	file_extensions.push_back("obj");
	file_extensions.push_back("stl");
	file_extensions.push_back("3ds");
	fs::path path = getOpenFilePath("", file_extensions);
	
	int err = -1;
	if (!path.empty()) {
		const std::string ext = path.extension().string();
		if (!ext.empty()) {
      //reset flags... not necessary
      Mesh::stateMask_= 1;
      Vertex::tagMask_ = 1;
      Vertex::sculptMask_ = 1;
      Triangle::tagMask_ = 1;

			Files files;
			Mesh* mesh = 0;
			if (ext == ".OBJ" || ext == ".obj") {
				mesh = files.loadOBJ(path.string());
			} else if (ext == ".STL" || ext == ".stl") {
				mesh = files.loadSTL(path.string());
			} else if (ext == ".3DS" || ext == ".3ds") {
				mesh = files.load3DS(path.string());
			}
			if (mesh_) {
				delete mesh_;
			}
			mesh_ = mesh;
			mesh_->startPushState();
			sculpt_.setMesh(mesh_);
			err = 1;
		}
	}

	return err; // error
}

int ClayDemoApp::saveFile()
{
	if (!mesh_) {
		return -1;
	}

	Files files;
	std::vector<std::string> file_extensions;
	file_extensions.push_back("stl");
	fs::path path = getSaveFilePath("", file_extensions);

	int err = -1;
	if (!path.empty()) {
		files.saveSTL(mesh_, path.string());
		err = 1;
	}
	return err; // error
}

void ClayDemoApp::setBrushMode(const std::string& str) {
	if (str == "Grow")
	{
		sculpt_.setSculptMode(Sculpt::INFLATE);
		_brush_color = Color(0.1f, 1.0f, 0.2f);
	}
	else if (str == "Shrink")
	{
		sculpt_.setSculptMode(Sculpt::DEFLATE);
		_brush_color = Color(1.0f, 0.8f, 0.1f);
	}
	else if (str == "Smooth")
	{
		sculpt_.setSculptMode(Sculpt::SMOOTH);
		_brush_color = Color(0.1f, 0.4f, 1.0f);
	}
	else if (str == "Flatten")
	{
		sculpt_.setSculptMode(Sculpt::FLATTEN);
		_brush_color = Color(0.5f, 0.5f, 0.5f);
	} else if (str == "Sweep") {
		sculpt_.setSculptMode(Sculpt::SWEEP);
		_brush_color = Color(0.5f, 0.5f, 0.5f);
	} else if (str == "Push") {
    sculpt_.setSculptMode(Sculpt::PUSH);
    _brush_color = Color(0.5f, 0.5f, 0.5f);
  } else if (str == "Paint") {
    sculpt_.setSculptMode(Sculpt::PAINT);
    _brush_color = Color(0.5f, 0.5f, 0.5f);
  }
}

void ClayDemoApp::setBrushSize(const std::string& str)
{
	if (!_leap_interaction)
	{
		return;
	}
	float size = 0.0f;
  if (str == "X-Small")
  {
    size = 5.0f;
  }
	else if (str == "Small")
	{
		size = 15.0f;
	}
	else if (str == "Medium")
	{
		size = 25.0f;
	}
	else if (str == "Large")
	{
		size = 40.0f;
	}
	_leap_interaction->setBrushRadius(size);
}

void ClayDemoApp::setBrushStrength(const std::string& str)
{
	if (!_leap_interaction)
	{
		return;
	}
	float strength = 0.0f;
	if (str == "Fine")
	{
		strength = 0.2f;
	}
	else if (str == "Medium")
	{
		strength = 0.6f;
	}
	else if (str == "Strong")
	{
		strength = 1.0f;
	}
	_leap_interaction->setBrushStrength(strength);
}

void ClayDemoApp::setEnvironment(const std::string& str)
{
	if (!_environment || _environment->getLoadingState() != Environment::LOADING_STATE_NONE)
	{
		return;
	}
	_loading_thread = thread(&Environment::setEnvironment, _environment, str, _environment->getCurTimeOfDay());
}

void ClayDemoApp::setTimeOfDay(const std::string& str)
{
	if (!_environment || _environment->getLoadingState() != Environment::LOADING_STATE_NONE)
	{
		return;
	}
	Environment::TimeOfDay time = (str == "Noon") ? Environment::TIME_NOON : Environment::TIME_DAWN;
	_loading_thread = thread(&Environment::setEnvironment, _environment, _environment->getCurEnvironmentString(), time);
}

void ClayDemoApp::setMaterial(const std::string& str)
{
	if (str == "Amethyst")
	{
		_ambient_factor = 0.0f;
		_diffuse_factor = 0.55f;
		_reflection_factor = 0.25f;
		_surface_color = Color(0.65f, 0.45f, 0.9f);
		_reflection_bias = 0.0f;
		_refraction_bias = 2.0f;
		_refraction_index = 0.4f;
	}
	else if (str == "Porcelain")
	{
		_ambient_factor = 0.0f;
		_diffuse_factor = 1.0f;
		_reflection_factor = 0.1f;
		_surface_color = Color(1.0f, 0.95f, 0.9f);
		_reflection_bias = 0.5f;
	}
	else if (str == "Glass")
	{
		_ambient_factor = 0.0f;
		_diffuse_factor = 0.15f;
		_reflection_factor = 0.7f;
		_surface_color = Color(0.4f, 0.45f, 0.5f);
		_reflection_bias = 0.0f;
		_refraction_bias = 0.0f;
		_refraction_index = 0.45f;
	}
	else if (str == "Clay")
	{
		_ambient_factor = 0.0f;
		_diffuse_factor = 1.0f;
		_reflection_factor = 0.0f;
		_surface_color = Color(0.7f, 0.6f, 0.3f);
	}
	else if (str == "Plastic")
	{
		_ambient_factor = 0.0f;
		_diffuse_factor = 1.0f;
		_reflection_factor = 0.1f;
		_surface_color = Color(0.0f, 0.8f, 1.0f);
		_reflection_bias = 0.5f;
	}
	else if (str == "Onyx")
	{
		_ambient_factor = 0.0f;
		_diffuse_factor = 1.0;
		_reflection_factor = 0.1f;
		_surface_color = Color(0.05f, 0.06f, 0.08f);
		_reflection_bias = 2.0f;
	}
	else if (str == "Flubber")
	{
		_ambient_factor = 0.2f;
		_diffuse_factor = 0.1f;
		_reflection_factor = 0.5f;
		_surface_color = Color(0.15f, 1.0f, 0.05f);
		_reflection_bias = 0.0f;
		_refraction_bias = 2.0f;
		_refraction_index = 0.8f;
	}
	else if (str == "Steel")
	{
		_ambient_factor = 0.0f;
		_diffuse_factor = 1.0f;
		_reflection_factor = 0.5f;
		_surface_color = Color(0.225f, 0.275f, 0.3f);
		_reflection_bias = 0.0f;
	}
}

void ClayDemoApp::performFileAction(const std::string& str)
{
	if (str == "Load")
	{
		loadFile();
	}
	else if (str == "Save")
	{
		saveFile();
	}
}

void ClayDemoApp::toggleFullscreen(const std::string& str)
{
	bool full = isFullScreen();
	setFullScreen(!full);
}

void ClayDemoApp::toggleWireframe(const std::string& str)
{
	_draw_edges = !_draw_edges;
}

void ClayDemoApp::setAutoSpin(const std::string& str)
{
	if (str == "Off")
	{
		mesh_->setRotationVelocity(0.0f);
	}
	else if (str == "Slow")
	{
    mesh_->setRotationVelocity(1.0f);
	}
	else if (str == "Medium")
	{
    mesh_->setRotationVelocity(3.5f);
	}
	else if (str == "Fast")
	{
    mesh_->setRotationVelocity(10.0f);
	}
}

void ClayDemoApp::setup()
{
	FreeImage_Initialise();
	enableAlphaBlending();
	enableDepthRead();
	enableDepthWrite();
	glCullFace( GL_BACK );
	glEnable(GL_CULL_FACE);

	_ambient_factor = 0.00f;
	_diffuse_factor = 1.0f;
	_reflection_factor = 0.1f;
	_surface_color = Color(1.f,1.f,1.f);
	_reflection_bias = 0;
	_refraction_bias = 0;
	_refraction_index = 0.4f;
	_draw_edges = false;
	_bloom_visible = true;
	_bloom_size = 1.f;
	_bloom_strength = 1.f;
	_bloom_light_threshold = 0.5f;

	_params = params::InterfaceGl::create( getWindow(), "App parameters", toPixels( Vec2i( 200, 400 ) ) );
	_params->minimize();
	_params->addSeparator();	
	_params->addText( "text", "label=`Surface parameters:`" );
	_params->addParam( "Ambient", &_ambient_factor, "min=0.0 max=0.5 step=0.01" );
	_params->addParam( "Diffuse", &_diffuse_factor, "min=0.0 max=1.0 step=0.05" );
	_params->addParam( "Reflection", &_reflection_factor, "min=0.0 max=1.0 step=0.05" );
	_params->addParam( "Color", &_surface_color, "" );
	_params->addParam( "Refraction Index", &_refraction_index, "min=0.0 max=1.0 step=0.01" );
	_params->addParam( "Reflection Bias", &_reflection_bias, "min=0.0 max=3.0 step=0.01" );
	_params->addParam( "Refraction Bias", &_refraction_bias, "min=0.0 max=3.0 step=0.01" );
	_params->addSeparator();	
	_params->addText( "text", "label=`Draw parameters:`" );
	_params->addParam( "Draw edges", &_draw_edges, "" );
	_params->addParam( "Draw AO", &_use_ao, "" );
	_params->addParam( "Only AO", &_only_ao, "" );
	_params->addSeparator();	
	_params->addText( "text", "label=`HDR parameters:`" );
	_params->addParam( "Exposure", &_exposure, "min=0.05 max=8.0 step=0.01" );
	_params->addParam( "Contrast", &_contrast, "min=0.33, max=3.0, step=0.01" );
	_params->addParam( "Show bloom", &_bloom_visible, "" );
	_params->addParam( "Bloom size", &_bloom_size, "min=0.0 max=2.0 step=0.01" );
	_params->addParam( "Bloom strength", &_bloom_strength, "min=0.0 max=1.0 step=0.01" );
	_params->addParam( "Bloom threshold", &_bloom_light_threshold, "min=0.0 max=2.0 step=0.01" );
  _params->addParam( "Use FXAA", &_use_fxaa, "" );

	_environment = new Environment();

	try {
		_light_clamp_shader = gl::GlslProg( loadResource( RES_LIGHT_CLAMP_VERT_GLSL ), loadResource( RES_LIGHT_CLAMP_FRAG_GLSL ) );
		_horizontal_blur_shader = gl::GlslProg( loadResource( RES_BLUR_HORIZONTAL_VERT_GLSL ), loadResource( RES_BLUR_HORIZONTAL_FRAG_GLSL ) );
		_vertical_blur_shader = gl::GlslProg( loadResource( RES_BLUR_VERTICAL_VERT_GLSL ), loadResource( RES_BLUR_VERTICAL_FRAG_GLSL ) );
		_blur_shader = gl::GlslProg( loadResource( RES_BLUR_VERT_GLSL ), loadResource( RES_BLUR_FRAG_GLSL ) );
		_screen_shader = gl::GlslProg( loadResource( RES_SCREEN_VERT_GLSL ), loadResource( RES_SCREEN_FRAG_GLSL ) );
		_material_shader = gl::GlslProg( loadResource( RES_MATERIAL_VERT_GLSL ), loadResource( RES_MATERIAL_FRAG_GLSL ) );
    _wireframe_shader = gl::GlslProg( loadResource( RES_MATERIAL_VERT_GLSL ), loadResource( RES_WIREFRAME_FRAG_GLSL ) );
		_metaball_shader = gl::GlslProg( loadResource( RES_METABALL_VERT_GLSL ), loadResource( RES_METABALL_FRAG_GLSL ) );
		_sky_shader = gl::GlslProg( loadResource( RES_SKY_VERT_GLSL ), loadResource( RES_SKY_FRAG_GLSL ) );
    _fxaa_shader = gl::GlslProg( loadResource( RES_FXAA_VERT_GLSL ), loadResource( RES_FXAA_FRAG_GLSL ) );
	} catch (gl::GlslProgCompileExc e) {
		std::cout << e.what() << std::endl;
		quit();
	}

	sculpt_.clearBrushes();

	_ui = new UserInterface();

	// top level node
	_ui->addElement(UIElement("Menu"));

	// second level nodes
	_ui->addElement(UIElement("Brush"), "Menu");
	_ui->addElement(UIElement("Editing"), "Menu");
	_ui->addElement(UIElement("Environment"), "Menu");
	_ui->addElement(UIElement("Material"), "Menu");

	// Brush nodes
	_ui->addElement(UIElement("Strength"), "Brush");
	_ui->addElement(UIElement("Size"), "Brush");
	_ui->addElement(UIElement("Type"), "Brush");

	// Strength nodes
	_ui->addElement(UIElement("Fine", boost::bind(&ClayDemoApp::setBrushStrength, this, ::_1)), "Strength");
	_ui->addElement(UIElement("Medium", boost::bind(&ClayDemoApp::setBrushStrength, this, ::_1)), "Strength");
	_ui->addElement(UIElement("Strong", boost::bind(&ClayDemoApp::setBrushStrength, this, ::_1)), "Strength");

	// Size nodes
  _ui->addElement(UIElement("X-Small", boost::bind(&ClayDemoApp::setBrushSize, this, ::_1)), "Size");
	_ui->addElement(UIElement("Small", boost::bind(&ClayDemoApp::setBrushSize, this, ::_1)), "Size");
	_ui->addElement(UIElement("Medium", boost::bind(&ClayDemoApp::setBrushSize, this, ::_1)), "Size");
	_ui->addElement(UIElement("Large", boost::bind(&ClayDemoApp::setBrushSize, this, ::_1)), "Size");

	// Type nodes
	_ui->addElement(UIElement("Grow", boost::bind(&ClayDemoApp::setBrushMode, this, ::_1)), "Type");
	_ui->addElement(UIElement("Shrink", boost::bind(&ClayDemoApp::setBrushMode, this, ::_1)), "Type");
	_ui->addElement(UIElement("Smooth", boost::bind(&ClayDemoApp::setBrushMode, this, ::_1)), "Type");
	_ui->addElement(UIElement("Flatten", boost::bind(&ClayDemoApp::setBrushMode, this, ::_1)), "Type");
	_ui->addElement(UIElement("Sweep", boost::bind(&ClayDemoApp::setBrushMode, this, ::_1)), "Type");
  _ui->addElement(UIElement("Push", boost::bind(&ClayDemoApp::setBrushMode, this, ::_1)), "Type");
  _ui->addElement(UIElement("Paint", boost::bind(&ClayDemoApp::setBrushMode, this, ::_1)), "Type");

	// Editing nodes
	_ui->addElement(UIElement("Fullscreen", boost::bind(&ClayDemoApp::toggleFullscreen, this, ::_1)), "Editing");
	_ui->addElement(UIElement("Wireframe", boost::bind(&ClayDemoApp::toggleWireframe, this, ::_1)), "Editing");
	_ui->addElement(UIElement("Load", boost::bind(&ClayDemoApp::performFileAction, this, ::_1)), "Editing");
	_ui->addElement(UIElement("Save", boost::bind(&ClayDemoApp::performFileAction, this, ::_1)), "Editing");
	_ui->addElement(UIElement("Auto-Spin"), "Editing");

	// Environment nodes
	_ui->addElement(UIElement("Scene"), "Environment");
	_ui->addElement(UIElement("Time of Day"), "Environment");

	// Auto-Spin nodes
	_ui->addElement(UIElement("Off", boost::bind(&ClayDemoApp::setAutoSpin, this, ::_1)), "Auto-Spin");
	_ui->addElement(UIElement("Slow", boost::bind(&ClayDemoApp::setAutoSpin, this, ::_1)), "Auto-Spin");
	_ui->addElement(UIElement("Medium", boost::bind(&ClayDemoApp::setAutoSpin, this, ::_1)), "Auto-Spin");
	_ui->addElement(UIElement("Fast", boost::bind(&ClayDemoApp::setAutoSpin, this, ::_1)), "Auto-Spin");

	// Scene nodes
	const std::vector<Environment::EnvironmentInfo>& infos = _environment->getEnvironmentInfos();
	for (size_t i=0; i<infos.size(); i++)
	{
		_ui->addElement(UIElement(infos[i]._name, boost::bind(&ClayDemoApp::setEnvironment, this, ::_1)), "Scene");
	}

	// Time of Day nodes
	_ui->addElement(UIElement("Dawn", boost::bind(&ClayDemoApp::setTimeOfDay, this, ::_1)), "Time of Day");
	_ui->addElement(UIElement("Noon", boost::bind(&ClayDemoApp::setTimeOfDay, this, ::_1)), "Time of Day");

	// Material nodes
	_ui->addElement(UIElement("Amethyst", boost::bind(&ClayDemoApp::setMaterial, this, ::_1)), "Material");
	_ui->addElement(UIElement("Porcelain", boost::bind(&ClayDemoApp::setMaterial, this, ::_1)), "Material");
	_ui->addElement(UIElement("Glass", boost::bind(&ClayDemoApp::setMaterial, this, ::_1)), "Material");
	_ui->addElement(UIElement("Clay", boost::bind(&ClayDemoApp::setMaterial, this, ::_1)), "Material");
	_ui->addElement(UIElement("Plastic", boost::bind(&ClayDemoApp::setMaterial, this, ::_1)), "Material");
	_ui->addElement(UIElement("Onyx", boost::bind(&ClayDemoApp::setMaterial, this, ::_1)), "Material");
	_ui->addElement(UIElement("Flubber", boost::bind(&ClayDemoApp::setMaterial, this, ::_1)), "Material");
	_ui->addElement(UIElement("Steel", boost::bind(&ClayDemoApp::setMaterial, this, ::_1)), "Material");

	_ui->setRootNode("Menu");

	_ui->setShader(&_metaball_shader);

	_controller.setPolicyFlags(Leap::Controller::POLICY_BACKGROUND_FRAMES);
	_controller.addListener(_listener);

	_leap_interaction = new LeapInteraction(&sculpt_, _ui);

	setBrushMode("Sweep");
	setBrushSize("Medium");
	setBrushStrength("Medium");
	setEnvironment("Islands");

	glEnable(GL_FRAMEBUFFER_SRGB);
}

void ClayDemoApp::shutdown() {
	FreeImage_DeInitialise();
}

void ClayDemoApp::resize()
{
	static const int DOWNSCALE_FACTOR = 4;

	int width = getWindowWidth();
	int height = getWindowHeight();
	width = std::max(DOWNSCALE_FACTOR, width);
	height = std::max(DOWNSCALE_FACTOR, height);
	Fbo::Format format;
	format.setColorInternalFormat(GL_RGB32F_ARB);
  format.enableDepthBuffer(false);
	//format.setSamples( 4 ); // uncomment this to enable 4x antialiasing
	//_screen_fbo = Fbo( width, height, format );
	console() << "FBO size: " << width << "    " << height << "\n";

	// set up blur FBOs
	_color_fbo = Fbo( width, height, format );
	_depth_fbo = Fbo( width, height, format );
	_blur_fbo = Fbo( width, height, format );
	_blur_fbo.bindFramebuffer();
	gl::clear();
	_blur_fbo.unbindFramebuffer();

	gl::Fbo::Format hdr_format;
	hdr_format.setColorInternalFormat(GL_RGBA32F_ARB);

	gl::Fbo::Format ldr_format;
	ldr_format.setColorInternalFormat(GL_RGBA32F_ARB);


	_light_clamp_fbo = Fbo( width, height, ldr_format );
	_horizontal_blur_fbo = Fbo( width/DOWNSCALE_FACTOR, height/DOWNSCALE_FACTOR, ldr_format );
	_vertical_blur_fbo = Fbo( width/DOWNSCALE_FACTOR, height/DOWNSCALE_FACTOR, ldr_format );
	_screen_fbo = Fbo( width, height, hdr_format );

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

void ClayDemoApp::mouseDown( MouseEvent event )
{
	_mouse_down = true;

	_current_mouse_pos = _previous_mouse_pos = _initial_mouse_pos = event.getPos();
}

void ClayDemoApp::mouseUp( MouseEvent event )
{
	_mouse_down = false;
}

void ClayDemoApp::mouseDrag( MouseEvent event )
{
	_previous_mouse_pos = _current_mouse_pos;
	_current_mouse_pos = event.getPos();

	updateCamera(float(_current_mouse_pos.x-_previous_mouse_pos.x)*CAMERA_SPEED,float(_current_mouse_pos.y-_previous_mouse_pos.y)*CAMERA_SPEED, 0.f);

  // New camera update.
  _camera_util->RecordUserInput(float(_current_mouse_pos.x-_previous_mouse_pos.x)*CAMERA_SPEED,float(_current_mouse_pos.y-_previous_mouse_pos.y)*CAMERA_SPEED, 0.f);
}

void ClayDemoApp::mouseWheel( MouseEvent event)
{
  float off = event.getWheelIncrement();
  _cam_dist -= 0.1f*event.getWheelIncrement();
  _cam_dist = std::min(MAX_CAMERA_DIST, std::max(0.5f, _cam_dist));
}

void ClayDemoApp::mouseMove( MouseEvent event)
{
	_previous_mouse_pos = _current_mouse_pos;
	_current_mouse_pos = event.getPos();
}

void ClayDemoApp::keyDown( KeyEvent event )
{
	switch( event.getChar() )
	{
		case KeyEvent::KEY_ESCAPE: quit(); break;
		case 'u': _draw_ui = !_draw_ui; break;
		case 'o': drawOctree_ = !drawOctree_; break;
    case 'z': if (event.isControlDown()) { if (mesh_) { mesh_->undo(); } } break;
    case 'y': if (event.isControlDown()) { if (mesh_) { mesh_->redo(); } } break;
    case 's': symmetry_ = !symmetry_; break;
	}
}

void ClayDemoApp::updateCamera(const float _DTheta,const float _DPhi,const float _DFov)
{
	_theta -= _DTheta;
	_phi += _DPhi;
	_fov += _DFov;

	if( _theta<0.f ) _theta += float(M_PI)*2.f;
	if( _theta>=M_PI*2.f ) _theta -= float(M_PI)*2.f;
	_phi = math<float>::clamp(_phi, float(-M_PI)*0.45f, float(M_PI)*0.45f);
	_fov = math<float>::clamp(_fov, 50.f, 90.f);
}



void ClayDemoApp::update()
{
  // Calculate initial camera position
  Vec3f campos;
	campos.x = cosf(_phi)*sinf(_theta)*_cam_dist;
	campos.y = sinf(_phi)*_cam_dist;
	campos.z = cosf(_phi)*cosf(_theta)*_cam_dist;

  // if-conditioning this will disable mouse-based movement, untill we actually handle camera update
  if (!mesh_ || _camera_util->state == CameraUtil::STATE_INVALID) {
    // Init camera
    _camera_util->SetFromStandardCamera(Vector3(campos.ptr()), Vector3(0,0,0), _cam_dist); // up vector assumed to be Vec3f(0,1,0)
  }

  campos = ToVec3f(_camera_util->transform.translation);
  Vector3 up = _camera_util->transform.rotation * Vector3::UnitY();
  Vector3 to = _camera_util->transform.translation +  _camera_util->transform.rotation * Vector3::UnitZ() * -200.0f;

  // Update camera
	_camera.lookAt(campos,ToVec3f(to),ToVec3f(up));
	_camera.setPerspective( _fov, getWindowAspectRatio(), 1.0f, 100000.f );
	_camera.getProjectionMatrix();
	bool supress = _environment->getLoadingState() != Environment::LOADING_STATE_NONE;
	_leap_interaction->processInteraction(_listener, getWindowAspectRatio(), _camera.getModelViewMatrix(), _camera.getProjectionMatrix(), getWindowSize(), supress);

	updateCamera(_leap_interaction->getDTheta(), _leap_interaction->getDPhi(), _leap_interaction->getDZoom());
  // Don't record leap's angles in the user camera
  //_camera_util->RecordUserInput(_leap_interaction->getDTheta(), _leap_interaction->getDPhi(), _leap_interaction->getDZoom());

  _camera_util->RecordUserInput(Vector3(_leap_interaction->getPinchDeltaFromLastCall().ptr()));

  // Camera update.
  if (mesh_) {
    _camera_util->UpdateCamera(mesh_);
  }

	float blend = (_fov-MIN_FOV)/(MAX_FOV-MIN_FOV);
	_cam_dist = blend*(MAX_CAMERA_DIST-MIN_CAMERA_DIST) + MIN_CAMERA_DIST;

  double curTime = ci::app::getElapsedSeconds();

	if (mesh_) {
    const float deltaTime = curTime - _last_update_time;
    mesh_->updateRotation(deltaTime);
		sculpt_.applyBrushes(deltaTime, symmetry_);
	}

  _last_update_time = curTime;
}

void ClayDemoApp::renderSceneToFbo(Camera& _Camera)
{
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

	// draw color pass of skybox
	_environment->bindCubeMap(Environment::CUBEMAP_SKY, 0);
	_sky_shader.bind();
	_sky_shader.uniform("cubemap", 0);
	setViewport( _color_fbo.getBounds() );
	_color_fbo.bindFramebuffer();
	clear();
	setMatrices( _camera );
	gl::drawSphere(Vec3f::zero(), SPHERE_RADIUS, 40);
	_color_fbo.unbindFramebuffer();
	_sky_shader.unbind();
	_environment->unbindCubeMap(0);

	// draw depth pass of skybox
	_environment->bindCubeMap(Environment::CUBEMAP_DEPTH, 0);
	_sky_shader.bind();
	_sky_shader.uniform("cubemap", 0);
	setViewport( _depth_fbo.getBounds() );
	_depth_fbo.bindFramebuffer();
	clear();
	setMatrices( _camera );
	gl::drawSphere(Vec3f::zero(), SPHERE_RADIUS, 40);
	_depth_fbo.unbindFramebuffer();
	_sky_shader.unbind();
	_environment->unbindCubeMap(0);

	int width = getWindowWidth();
	int height = getWindowHeight();

	// bind blur shader and set parameters
	_blur_shader.bind();
	_blur_shader.uniform("color_tex", 0);
	_blur_shader.uniform("depth_tex", 1);
	_blur_shader.uniform("depth_min", depth_min);
	_blur_shader.uniform("depth_max", 0.0f);
	_blur_shader.uniform("offset_min", 0.0f);
	_blur_shader.uniform("offset_max", 0.001f);

	// prepare for first pass (horizontal) blur
	_blur_shader.uniform("offset_mult", Vec2f(1.0f, 0.0f));
	setViewport( _blur_fbo.getBounds() );
	_blur_fbo.bindFramebuffer();
	_color_fbo.bindTexture(0);
	_depth_fbo.bindTexture(1);

	// perform first pass (horizontal) blur
	gl::pushMatrices();
	gl::setMatricesWindow(width, height, false);
	gl::clear();
	gl::drawSolidRect( _blur_fbo.getBounds() );
	gl::popMatrices();
	_color_fbo.unbindTexture();
	_blur_fbo.unbindFramebuffer();

	// prepare for second pass (vertical) blur
	_blur_shader.uniform("offset_mult", Vec2f(0.0f, 1.0f));
	_blur_fbo.bindTexture(0);
	_screen_fbo.bindFramebuffer();
	setViewport( _screen_fbo.getBounds() );

	// perform second pass (vertical) blur
	gl::pushMatrices();
	gl::setMatricesWindow(width, height, false);
	gl::clear();
	color(ColorA::white());
	gl::drawSolidRect( _screen_fbo.getBounds() );
	gl::popMatrices();

	// clean up
	_blur_fbo.unbindTexture();
	_depth_fbo.unbindTexture();
	_blur_shader.unbind();
	setMatrices( _camera );

	// enable depth and culling for rendering mesh
	glEnable(GL_CULL_FACE);
	gl::enableDepthRead();
	gl::enableDepthWrite();
	enableAlphaBlending();

	_environment->bindCubeMap(Environment::CUBEMAP_IRRADIANCE, 0);
	_environment->bindCubeMap(Environment::CUBEMAP_RADIANCE, 1);

  if (mesh_) {
	  _material_shader.bind();
	  GLint vertex = _material_shader.getAttribLocation("vertex");
	  GLint normal = _material_shader.getAttribLocation("normal");
    GLint color = _material_shader.getAttribLocation("color");

    ci::Matrix44f transform = ci::Matrix44f::identity();
    transform = ci::Matrix44f(mesh_->getTransformation().data());
    ci::Matrix44f transformit = transform.inverted().transposed();

	  // draw mesh
	  _material_shader.uniform( "useRefraction", true);
	  _material_shader.uniform( "campos", _Camera.getEyePoint() );
	  _material_shader.uniform( "irradiance", 0 );
	  _material_shader.uniform( "radiance", 1 );
	  _material_shader.uniform( "ambientFactor", _ambient_factor );
	  _material_shader.uniform( "diffuseFactor", _diffuse_factor );
	  _material_shader.uniform( "reflectionFactor", _reflection_factor );
	  _material_shader.uniform( "surfaceColor", _surface_color );
	  _material_shader.uniform( "transform", transform );
	  _material_shader.uniform( "transformit", transformit );
	  _material_shader.uniform( "reflectionBias", _reflection_bias );
	  _material_shader.uniform( "refractionBias", _refraction_bias );
	  _material_shader.uniform( "refractionIndex", _refraction_index );
	  _material_shader.uniform( "numLights", sculpt_.getNumBrushes() );
	  _material_shader.uniform( "brushPositions", sculpt_.brushPositions().data(), sculpt_.getNumBrushes() );
	  _material_shader.uniform( "brushWeights", sculpt_.brushWeights().data(), sculpt_.getNumBrushes() );
    _material_shader.uniform( "brushRadii", sculpt_.brushRadii().data(), sculpt_.getNumBrushes() );
	  _material_shader.uniform( "lightColor", 0.15f*_brush_color );
	  _material_shader.uniform( "lightExponent", 30.0f);
	  _material_shader.uniform( "lightRadius", 3.0f);

	  glPushMatrix();
	  mesh_->draw(vertex, normal, color);
    _material_shader.unbind();

    if (_draw_edges) {
      _wireframe_shader.bind();
	    vertex = _wireframe_shader.getAttribLocation("vertex");
	    _wireframe_shader.uniform( "transform", transform );
	    _wireframe_shader.uniform( "transformit", transformit );
	    _wireframe_shader.uniform( "surfaceColor", Color::black() );
      glLineWidth(2.0f);
      glPolygonMode(GL_FRONT, GL_LINE);
      mesh_->drawVerticesOnly(vertex);
		  glPolygonMode(GL_FRONT, GL_FILL);
      _wireframe_shader.unbind();
	  }
	  glPopMatrix();
  }

	// draw brushes
#if 0
	_material_shader.uniform( "transform", Matrix44f::identity() );
	_material_shader.uniform( "transformit", Matrix44f::identity() );
	_material_shader.uniform( "useRefraction", false);
	_material_shader.uniform( "ambientFactor", 0.15f );
	_material_shader.uniform( "diffuseFactor", 0.15f );
	_material_shader.uniform( "reflectionFactor", 0.2f );
	_material_shader.uniform( "surfaceColor", _brush_color );
	_material_shader.uniform( "reflectionBias", 0.0f );
	_material_shader.uniform( "numLights", 0 );
	const BrushVector& brushes = sculpt_.getBrushes();
	for (size_t i=0; i<brushes.size(); i++) {
		_material_shader.uniform("alphaMult", brushes[i]._weight);
		const Vector3& pos = brushes[i]._position;
		ci::Vec3f temp(pos.x(), pos.y(), pos.z());
		gl::drawSphere(temp, brushes[i]._radius, 30);
	}
	_material_shader.unbind();
#else
	_environment->unbindCubeMap(0);
	_environment->unbindCubeMap(1);
  const BrushVector& brushes = sculpt_.getBrushes();
	for (size_t i=0; i<brushes.size(); i++) {
    ColorA color(_brush_color, 0.25f);
    gl::color(color);
    brushes[i].draw();
	}
#endif

  if (drawOctree_) {
    mesh_->drawOctree();
  }

	if (_draw_ui) 
	{
		disableDepthRead();
		disableDepthWrite();
		setMatricesWindow( getWindowSize() );
		setViewport( getWindowBounds() );
		Matrix33f rot = _camera.getModelViewMatrix().subMatrix33(0, 0).inverted();
		_ui->draw(_environment, rot);
		enableDepthRead();
		enableDepthWrite();
	}

	_screen_fbo.unbindFramebuffer();
}

void ClayDemoApp::createBloom()
{
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);

	SaveFramebufferBinding bindingSaver;

	// *** clamp the light ***
	_light_clamp_fbo.bindFramebuffer();
	setViewport( _light_clamp_fbo.getBounds() );
	clear( Color( 0, 0, 0 ) ); 
	gl::setMatricesWindow(_light_clamp_fbo.getWidth(),_light_clamp_fbo.getHeight(),false);
	_screen_fbo.bindTexture(0);
	_light_clamp_shader.bind();
	_light_clamp_shader.uniform( "input_texture", 0 );
	_light_clamp_shader.uniform( "light_threshold", _bloom_light_threshold );
	gl::drawSolidRect(Rectf(0.0f,0.0f,(float)_light_clamp_fbo.getWidth(),(float)_light_clamp_fbo.getHeight()));
	_light_clamp_shader.unbind();
	_light_clamp_fbo.unbindFramebuffer();

	// *** blur horizontally ***
	_horizontal_blur_fbo.bindFramebuffer();
	setViewport( _horizontal_blur_fbo.getBounds() );
	clear( Color( 0, 0, 0 ) ); 
	gl::setMatricesWindow(_horizontal_blur_fbo.getWidth(),_horizontal_blur_fbo.getHeight(),false);
	_light_clamp_fbo.bindTexture(0);
	_horizontal_blur_shader.bind();
	_horizontal_blur_shader.uniform( "input_texture", 0 );
	_horizontal_blur_shader.uniform( "blurSize",_bloom_size/_horizontal_blur_fbo.getWidth() );
	gl::drawSolidRect(Rectf(0.0f,0.0f,(float)_horizontal_blur_fbo.getWidth(),(float)_horizontal_blur_fbo.getHeight()));
	_horizontal_blur_shader.unbind();
	_horizontal_blur_fbo.unbindFramebuffer();

	// *** blur vertically ***
	_vertical_blur_fbo.bindFramebuffer();
	setViewport( _vertical_blur_fbo.getBounds() );
	clear( Color( 0, 0, 0 ) ); 
	gl::setMatricesWindow(_vertical_blur_fbo.getWidth(),_vertical_blur_fbo.getHeight(),false);
	_horizontal_blur_fbo.bindTexture(0);
	_vertical_blur_shader.bind();
	_vertical_blur_shader.uniform( "input_texture", 0 );
	_vertical_blur_shader.uniform( "blurSize", _bloom_size/_vertical_blur_fbo.getHeight() );
	gl::drawSolidRect(Rectf(0.0f,0.0f,(float)_vertical_blur_fbo.getWidth(),(float)_vertical_blur_fbo.getHeight()));
	_vertical_blur_shader.unbind();
	_vertical_blur_fbo.unbindFramebuffer();

	setViewport( _screen_fbo.getBounds() );
}

void ClayDemoApp::draw()
{
	static const float LOADING_DARKEN_TIME = 2.0f;
	static const float LOADING_LIGHTEN_TIME = 2.0f;
	float exposure_mult = 1.0f;
	const Environment::LoadingState loading_state = _environment->getLoadingState();
	const float loading_time = _environment->getTimeSinceLoadingStateChange();
	if (loading_state == Environment::LOADING_STATE_LOADING)
	{
		float mult = 1.0f - math<float>::clamp(loading_time/LOADING_DARKEN_TIME);
		exposure_mult *= mult;
		
	}
	else if (loading_state == Environment::LOADING_STATE_DONE_LOADING && loading_time > LOADING_DARKEN_TIME)
	{
		_environment->transitionComplete();
		exposure_mult = 0.0f;
		Environment::EnvironmentInfo* info = _environment->getEnvironmentInfoFromString(_environment->getCurEnvironmentString());
		if (info) {
			_bloom_strength = info->_bloom_strength;
			_bloom_light_threshold = info->_bloom_threshold;
			_exposure = info->_exposure;
			_contrast = info->_contrast;
		}
	}
	else if (loading_state == Environment::LOADING_STATE_PROCESSING)
	{
		exposure_mult = 0.0f;
	}
	else if (loading_state == Environment::LOADING_STATE_NONE)
	{
		exposure_mult *= math<float>::clamp(loading_time/LOADING_LIGHTEN_TIME);
	}

	exposure_mult = Utilities::SmootherStep(exposure_mult*exposure_mult);
	
	clear();
	
	const float width = static_cast<float>(getWindowBounds().getWidth());
	const float height = static_cast<float>(getWindowBounds().getHeight());

	if (exposure_mult > 0.0f)
	{
		renderSceneToFbo(_camera);

		if( _bloom_visible )
		{
	//const int blur_iterations = 5;
	//	for(int i=0; i<blur_iterations; ++i)
	//	{
			createBloom();
	//	}
		}

		setMatricesWindow( getWindowWidth(), getWindowHeight(), false);

		glDisable(GL_CULL_FACE);
		_screen_fbo.bindTexture(0);
		_vertical_blur_fbo.bindTexture(1);
		_screen_fbo.getDepthTexture().bind(2);
    if (_use_fxaa) {
      _color_fbo.bindFramebuffer();
    }
		_screen_shader.bind();
		_screen_shader.uniform( "color_texture", 0 );
		_screen_shader.uniform( "bloom_texture", 1 );
		_screen_shader.uniform( "depth_texture", 2 );
		_screen_shader.uniform( "width", width );
		_screen_shader.uniform( "height", height );
		_screen_shader.uniform( "exposure", _exposure * exposure_mult);
		_screen_shader.uniform( "contrast", _contrast );
		_screen_shader.uniform( "use_ao", _use_ao );
		_screen_shader.uniform( "only_ao", _only_ao );
		_screen_shader.uniform( "bloom_strength", _bloom_strength * float(_bloom_visible) );
		_screen_shader.uniform( "vignette_radius", 0.9f * sqrt((width/2)*(width/2) + (height/2)*(height/2)) );
		_screen_shader.uniform( "vignette_strength", 0.75f );
		gl::drawSolidRect(Rectf(0.0f,0.0f,width,height));
		_screen_shader.unbind();
		_screen_fbo.unbindTexture();
		_vertical_blur_fbo.unbindTexture();
		_screen_fbo.getDepthTexture().unbind(2);

    if (_use_fxaa) {
      _color_fbo.unbindFramebuffer();

      _color_fbo.bindTexture(0);
      _fxaa_shader.bind();
      _fxaa_shader.uniform( "textureSampler", 0 );
      _fxaa_shader.uniform( "texcoordOffset", Vec2f(1.0f/width, 1.0f/height) );
      gl::drawSolidRect(Rectf(0,0,width,height));
      _fxaa_shader.unbind();
      _color_fbo.unbindTexture();
    }

    if (mesh_) {
      int tris = mesh_->getNbTriangles();
      int verts = mesh_->getNbVertices();
      std::stringstream ss;
      ss << getAverageFps() << " fps, " << tris << " triangles, " << verts << " vertices";
      glPushMatrix();
      gl::scale(1, -1);
      ci::gl::drawString(ss.str(), Vec2f(5.0f, -(height-5.0f)), ColorA::white(), Font("Arial", 18));
      glPopMatrix();
    }
  }
  else
  {
    setMatricesWindow( getWindowWidth(), getWindowHeight(), false);
    static const float FONT_SIZE = 24.0f;
    Font font("Arial", FONT_SIZE);
    glPushMatrix();
    gl::scale(1, -1);
    gl::drawStringCentered("Loading...", Vec2f(width/2.0f, -height/2.0f), ColorA::white(), font);
    glPopMatrix();
  }

	_params->draw(); // draw the interface
}

#if 1
#pragma comment( linker, "/subsystem:\"console\" /entry:\"mainCRTStartup\"" )

int main( int argc, char * const argv[] ) {
  cinder::app::AppBasic::prepareLaunch();
  cinder::app::AppBasic *app = new ClayDemoApp;
  //cinder::app::RendererRef ren(new RendererGl);
  cinder::app::RendererRef ren(new RendererGl(RendererGl::AA_NONE));
  cinder::app::AppBasic::executeLaunch( app, ren, "ClayDemo");
  cinder::app::AppBasic::cleanupLaunch();
  return 0;
}
#else
//CINDER_APP_NATIVE( ClayDemoApp, RendererGl )
CINDER_APP_NATIVE( ClayDemoApp, RendererGl(RendererGl::AA_NONE) )
#endif
