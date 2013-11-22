#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "cinder/Cinder.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/Thread.h"
#include <vector>
#include <string>
using namespace cinder;
using namespace cinder::gl;

class CCubeMapProcessor;
struct FIBITMAP;
enum FREE_IMAGE_FORMAT;

class Environment
{
public:

  enum CubeMap {
    CUBEMAP_SKY,
    CUBEMAP_DEPTH,
    CUBEMAP_IRRADIANCE,
    CUBEMAP_RADIANCE,
  };

  enum LoadingState {
    LOADING_STATE_NONE,
    LOADING_STATE_LOADING,
    LOADING_STATE_DONE_LOADING,
    LOADING_STATE_PROCESSING,
    LOADING_STATE_DONE_PROCESSING
  };

  struct EnvironmentInfo {
    // images must be arranged in the following order:
    // x-positive, x-negative, y-positive, y-negative, z-positive, z-negative
    std::string _dawn_images[6];
    std::string _depth_images[6];
    std::string _name;
    gl::Texture* _preview_image;
    float _bloom_strength;
    float _bloom_threshold;
    float _exposure;
    float _contrast;
  };

  Environment();
  virtual ~Environment();
  bool haveEnvironment() const { return !_cur_environment.empty(); }
  void bindCubeMap(CubeMap map, int pos);
  void unbindCubeMap(int pos);
  const std::string& getCurEnvironmentString() const { return _cur_environment; }
  const std::string& getPendingEnvironmentString() const { return _pending_environment; }
  void beginLoading(const std::string& name);
  void finishLoading();
  void beginProcessing();
  void finishProcessing();
  LoadingState getLoadingState() const { return _loading_state; }
  double getLastStateChangeTime() const { return _loading_state_change_time; }
  
  static const std::vector<EnvironmentInfo>& getEnvironmentInfos() { return _environment_infos; }
  static EnvironmentInfo* getEnvironmentInfoFromString(const std::string& name);

private:

  static const int MIPMAP_LEVELS = 6;
  static const int NUM_CHANNELS = 3;
  static const int CUBEMAP_SIDES = 6;

  struct CubemapImages {
    GLuint cubemap;
    GLint internalFormat;
    unsigned int inputSize;
    unsigned int outputSize;
    GLenum format;
    float* images[MIPMAP_LEVELS][CUBEMAP_SIDES];
    bool irradiance;
  };

  bool loadImageSet(std::string* filenames, FIBITMAP** bitmaps, unsigned int* bitmapWidths, unsigned int* bitmapHeights, GLint* internalFormats, GLenum* formats);
  void loadBitmap(std::string* filenames, int _Idx, FIBITMAP** bitmaps, unsigned int* bitmapWidths, unsigned int* bitmapHeights, GLint* internalFormats, GLenum* formats);
  void freeBitmaps(FIBITMAP** bitmaps);
  void processMipmappedCubemap(CubemapImages& cubemapImages);
  void uploadMipmappedCubemap(CubemapImages& cubemapImages);
  void prepareCubemap(GLuint* cubemap, int numLevels);
  void saveImagesToCubemap(GLuint cubemap, GLint internal_format, int miplevel, unsigned int width, unsigned int height, GLenum format, float** images);
  
  static void preparePaths(const std::string& path, std::string* filenames);
  static EnvironmentInfo prepareEnvironmentInfo(const std::string& name, float strength, float thresh, float exposure, float contrast);
  static void createEnvironmentInfos();
  static void createWorkingDirectory();

  CubemapImages irradianceImages;
  CubemapImages radianceImages;

  GLuint _cubemap_sky;
  GLuint _cubemap_depth;
  GLuint _cubemap_irradiance;
  GLuint _cubemap_radiance;

  std::string _cur_environment;
  std::string _pending_environment;
  std::condition_variable _loading_condition;
  std::mutex _loading_mutex;
  LoadingState _loading_state;
  double _loading_state_change_time;

  float* orig_images[6];

  FIBITMAP* bitmaps[6];
  unsigned int bitmap_widths[6];
  unsigned int bitmap_heights[6];
  GLint internal_formats[6];
  GLenum formats[6];

  static CCubeMapProcessor* _cubemap_processor;
  static std::vector<EnvironmentInfo> _environment_infos;
  static std::string working_directory;

};

#endif // #ifndef __ENVIRONMENT_H__
