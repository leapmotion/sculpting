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
#include "DynamicMesh.h"
#include "CreatePrimitives.h"
#include "SmoothSurface.h"
#include "Sculptor.h"
#include "UserInterface.h"
#include "LeapListener.h"
#include "Leap.h"
#include "LeapInteraction.h"
#include "Utilities.h"
#include "cinder/Thread.h"

using namespace ci;
using namespace ci::gl;
using namespace ci::app;
using namespace std;

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
	void renderSceneToFbo(Camera& _Camera);
	void createBloom();
	void draw();

private:

	// *** camera stuff ***
	CameraPersp _camera;
	float _theta;
	float _phi;
	float _fov;
	float _cam_dist;

	// **** mouse stuff ***
	Vec2i _initial_mouse_pos, _current_mouse_pos, _previous_mouse_pos;
	bool _mouse_down;

	// *** scene stuff ***
	Environment* _environment;
	GlslProg _sky_shader;
	GlslProg _material_shader;
	DynamicMesh* _mesh;
	GlslProg _blur_shader;
	Fbo _color_fbo;
	Fbo _depth_fbo;
	Fbo _blur_fbo;
	boost::thread _loading_thread;

	// *** Leap stuff ***
	LeapListener _listener;
	Leap::Controller _controller;
	LeapInteraction* _leap_interaction;

	// *** ui stuff ***
	params::InterfaceGlRef _params;
	float _ambient_factor;
	float _diffuse_factor;
	float _reflection_factor;
	Color _surface_color;
	Color _brush_color;
	bool _draw_edges;
	bool _draw_normals;
	float _minimum_edge_length;
	float _maximum_edge_length;
	float _reflection_bias;
	float _refraction_bias;
	float _refraction_index;
	bool _use_ao;
	bool _only_ao;
	bool _draw_smooth_surface;

	Sculptor* _sculptor;
	UserInterface* _ui;
	GlslProg _metaball_shader;
	bool _draw_ui;

#ifdef USE_SMOOTH_SURFACE
	SmoothSurface* _surface;
#endif
    
	Fbo _screen_fbo;
	Fbo _light_clamp_fbo;
	Fbo _horizontal_blur_fbo;
	Fbo _vertical_blur_fbo;
	GlslProg _light_clamp_shader;
	GlslProg _horizontal_blur_shader;
	GlslProg _vertical_blur_shader;
	GlslProg _screen_shader;
	float _exposure; // hdr exposure
	float _contrast;
	bool _bloom_visible;
	float _bloom_size;
	float _bloom_strength;
	float _bloom_light_threshold;

	// *** transform stuff ***
	Matrix44f _transform;
	Matrix44f _transform_inv;
	Vec3f _transform_translation;
	Vec3f _transform_rotation;
	float _transform_scaling;
	float _rotation_speed;
};

#endif
