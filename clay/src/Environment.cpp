#include "StdAfx.h"
#include "Environment.h"
#include "Resources.h"
#include "cinder/app/App.h"
#include "CCubeMapProcessor.h"
#include "Common.h"

#if _WIN32
#include <direct.h>
#else
#include "CoreFoundation/CoreFoundation.h"
#endif

using namespace ci::app;

const int MIPMAP_LEVELS = 6;

CCubeMapProcessor* Environment::_cubemap_processor = new CCubeMapProcessor();
std::vector<Environment::EnvironmentInfo> Environment::_environment_infos;
std::string Environment::working_directory;

Environment::Environment() : _cur_environment(""), _cur_time_of_day(TIME_DAWN),
  _loading_state(LOADING_STATE_NONE), _loading_state_change_time(0.0)
{
  createWorkingDirectory();
  createEnvironmentInfos();
}

Environment::~Environment() { }

void Environment::setEnvironment(const std::string& name, TimeOfDay time) {
  if (name == _cur_environment && time == _cur_time_of_day) {
    return;
  }

  // find environment by this name
  EnvironmentInfo* env = getEnvironmentInfoFromString(name);

  if (!env) {
    // environment with this name not found
    return;
  }

  _loading_state_change_time = getElapsedSeconds();
  _loading_state = LOADING_STATE_LOADING;

  std::string* filenames = (time == TIME_NOON) ? env->_noon_images : env->_dawn_images;
  if (!loadImageSet(filenames, bitmaps, bitmap_widths, bitmap_heights, internal_formats, formats) ||
    !loadImageSet(env->_depth_images, depth_bitmaps, depth_bitmap_widths, depth_bitmap_heights, depth_internal_formats, depth_formats))
  {
    _loading_state_change_time = getElapsedSeconds();
    _loading_state = LOADING_STATE_NONE;
    return;
  }

  _cur_environment = name;
  _cur_time_of_day = time;

  // wait for transition completion
  _loading_state = LOADING_STATE_DONE_LOADING;
  std::unique_lock<std::mutex> lock(_loading_mutex);
  _loading_condition.wait(lock);
}

void Environment::bindCubeMap(CubeMap map, int pos) {
  glActiveTexture(GL_TEXTURE0 + pos);
  switch(map) {
  case CUBEMAP_SKY: glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, _cubemap_sky); break;
  case CUBEMAP_DEPTH: glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, _cubemap_depth); break;
  case CUBEMAP_IRRADIANCE: glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, _cubemap_irradiance); break;
  case CUBEMAP_RADIANCE: glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, _cubemap_radiance); break;
  }
}

void Environment::unbindCubeMap(int pos) {
  glActiveTexture(GL_TEXTURE0 + pos);
  glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, 0);
}

const std::vector<Environment::EnvironmentInfo>& Environment::getEnvironmentInfos() {
  return _environment_infos;
}

Environment::EnvironmentInfo* Environment::getEnvironmentInfoFromString(const std::string& name) {
  for (size_t i=0; i<_environment_infos.size(); i++) {
    if (_environment_infos[i]._name == name) {
      return &_environment_infos[i];
    }
  }
  return 0;
}

void Environment::transitionComplete() {
  std::unique_lock<std::mutex> lock(_loading_mutex);
  _loading_state_change_time = getElapsedSeconds();
  _loading_state = LOADING_STATE_PROCESSING;

  // free old textures
  if (!_cur_environment.empty()) {
    glDeleteTextures(1, &_cubemap_sky);
    glDeleteTextures(1, &_cubemap_depth);
    glDeleteTextures(1, &_cubemap_irradiance);
    glDeleteTextures(1, &_cubemap_radiance);
  }

  // generate OpenGL cubemap textures
  prepareCubemap(&_cubemap_sky, 1);
  prepareCubemap(&_cubemap_depth, 1);
  prepareCubemap(&_cubemap_irradiance, 1);
  prepareCubemap(&_cubemap_radiance, MIPMAP_LEVELS);

  float* images[6];
  int width, height;
  GLint internal_format;
  GLenum format;

  // transfer depth images into OpenGL
  for (int i=0; i<6; i++) {
    images[i] = reinterpret_cast<float*>(FreeImage_GetBits(depth_bitmaps[i]));
  }
  width = depth_bitmap_widths[0];
  height = depth_bitmap_heights[0];
  internal_format = depth_internal_formats[0];
  format = depth_formats[0];
  saveImagesToCubemap(_cubemap_depth, internal_format, 0, width, height, format, images);
  freeBitmaps(depth_bitmaps);

  // transfer sky images into OpenGL
  for (int i=0; i<6; i++) {
    images[i] = reinterpret_cast<float*>(FreeImage_GetBits(bitmaps[i]));
  }
  width = bitmap_widths[0];
  height = bitmap_heights[0];
  internal_format = internal_formats[0];
  format = formats[0];
  saveImagesToCubemap(_cubemap_sky, internal_format, 0, width, height, format, images);

  // generate mipmapped cubemaps of radiance and irradiance from sky images
  static const int DOWNSCALE_FACTOR = 3;
  _cubemap_processor->Clear();
  _cubemap_processor->Init(width, height/DOWNSCALE_FACTOR, MIPMAP_LEVELS, 3);
  generateMipmappedCubemap(_cubemap_irradiance, internal_format, format, width, width/DOWNSCALE_FACTOR, images, true);
  generateMipmappedCubemap(_cubemap_radiance, internal_format, format, width, width/DOWNSCALE_FACTOR, images, false);

  freeBitmaps(bitmaps);

  _loading_state_change_time = getElapsedSeconds();
  _loading_state = LOADING_STATE_NONE;

  _loading_condition.notify_all();
}

Environment::LoadingState Environment::getLoadingState() const {
  return _loading_state;
}

float Environment::getTimeSinceLoadingStateChange() const {
  double cur_time = getElapsedSeconds();
  return static_cast<float>(cur_time - _loading_state_change_time);
}

const std::string& Environment::getCurEnvironmentString() const {
  return _cur_environment;
}

Environment::TimeOfDay Environment::getCurTimeOfDay() const {
  return _cur_time_of_day;
}

bool Environment::loadImageSet(std::string* filenames,
                               FIBITMAP** bitmaps,
                               unsigned int* bitmapWidths,
                               unsigned int* bitmapHeights,
                               GLint* internalFormats,
                               GLenum* formats)
{
  for (int i=0; i<6; i++) {
    loadBitmap(filenames, i, bitmaps, bitmapWidths, bitmapHeights, internalFormats, formats);
  }

  GLint internal_format;
  GLenum format;
  unsigned int width, height;
  for (int i=0; i<6; i++) {
    if (i == 0) {
      internal_format = internalFormats[i];
      format = formats[i];
      width = bitmapWidths[i];
      height = bitmapHeights[i];
    }
    if (!bitmaps[i] || internalFormats[i] != internal_format || formats[i] != format || bitmapWidths[i] != width || bitmapHeights[i] != height) {
      // verify that all images have the same format and size
      freeBitmaps(bitmaps);
      return false;
    }
  }
  return true;
}

void Environment::loadBitmap(std::string* filenames,
                             int _Idx,
                             FIBITMAP** bitmaps,
                             unsigned int* bitmapWidths,
                             unsigned int* bitmapHeights,
                             GLint* internalFormats,
                             GLenum* formats) {
  FREE_IMAGE_FORMAT fifmt = FreeImage_GetFileType(filenames[_Idx].c_str());
  if (fifmt != FIF_EXR) {
    // expect OpenEXR
    return;
  }
  bitmaps[_Idx] = FreeImage_Load(fifmt, filenames[_Idx].c_str());
  if (bitmaps[_Idx]) {
    FREE_IMAGE_TYPE cur_type = FreeImage_GetImageType(bitmaps[_Idx]);
    bool type_supported = (cur_type == FIT_FLOAT || cur_type == FIT_RGBF);
    if (type_supported) {
      // expect floating point images with 1 or 3 channels/pixel
      bitmapWidths[_Idx] = FreeImage_GetWidth(bitmaps[_Idx]);
      bitmapHeights[_Idx] = FreeImage_GetHeight(bitmaps[_Idx]);
      FreeImage_FlipVertical(bitmaps[_Idx]);
      if (cur_type == FIT_FLOAT) {
        internalFormats[_Idx] = GL_R32F;
        formats[_Idx] = GL_RED;
      } else if (cur_type == FIT_RGBF) {
        internalFormats[_Idx] = GL_RGB32F_ARB;
        formats[_Idx] = GL_RGB;
      }
    } else {
      bitmapWidths[_Idx] = 0;
      bitmapHeights[_Idx] = 0;
      FreeImage_Unload(bitmaps[_Idx]);
      bitmaps[_Idx] = 0;
    }
  }
}

void Environment::freeBitmaps(FIBITMAP** bitmaps) {
  for (int i=0; i<6; i++) {
    if (bitmaps[i]) {
      FreeImage_Unload(bitmaps[i]);
      bitmaps[i] = 0;
    }
  }
}

void Environment::generateMipmappedCubemap(GLuint cubemap, GLint internal_format, GLenum format, int input_size, int output_size, float** images, bool irradiance) {
  int numChannels = 3;
  int numLevels = irradiance ? 1 : MIPMAP_LEVELS;

  std::thread threads[6];
  for (int i=0; i<6; i++) {
    threads[i] = std::thread(&CCubeMapProcessor::SetInputFaceData,
      _cubemap_processor,
      i,
      CP_VAL_FLOAT32,
      numChannels,
      input_size*numChannels*4,
      images[i],
      15.0f,
      1.0f,
      1.0f);
  }
  for (int i=0; i<6; i++) {
    threads[i].join();
  }

  bool bUseMultithread = true;
  int FilterTech = CP_FILTER_TYPE_CONE;
  float BaseFilterAngle = 0.0f;
  float MipInitialFilterAngle = 1.0f;
  float MipFilterAngleScale = 2.0f; 
  bool bUseSolidAngleWeighting = true;
  float SpecularPower = 2048.0f;
  float CosinePowerDropPerMip = 0.25;
  int NumMipmap = numLevels;
  int CosinePowerMipmapChainMode = CP_COSINEPOWER_CHAIN_DROP;
  bool bExcludeBase = false;
  bool bIrradianceCubemap = irradiance;
  int LightingModel = false;
  float GlossScale = 10.0f;
  float GlossBias = 1.0f;
  int EdgeFixupTech = CP_FIXUP_BENT;
  //bool bCubeEdgeFixup = true;
  int EdgeFixupWidth = 1; 

  _cubemap_processor->InitiateFiltering(
    BaseFilterAngle,
    MipInitialFilterAngle,
    MipFilterAngleScale,
    FilterTech,
    EdgeFixupTech,
    EdgeFixupWidth,
    bUseSolidAngleWeighting,
    bUseMultithread,
    SpecularPower,
    CosinePowerDropPerMip,
    NumMipmap,
    CosinePowerMipmapChainMode,
    bExcludeBase,
    bIrradianceCubemap,
    LightingModel,
    GlossScale,
    GlossBias
    );

  int cur_size = output_size;
  float* output_images[6];
  for (int i=0; i<numLevels; i++) {
    int numBytes = cur_size*cur_size*numChannels*4;
    for (int j=0; j<6; j++) {
      output_images[j] = new float[numBytes];
      threads[j] = std::thread(&CCubeMapProcessor::GetOutputFaceData,
        _cubemap_processor,
        j,
        i,
        CP_VAL_FLOAT32,
        numChannels,
        cur_size*numChannels*4,
        output_images[j],
        1.0f,
        1.0f);
    }
    for (int j=0; j<6; j++) {
      threads[j].join();
    }

    saveImagesToCubemap(cubemap, internal_format, i, cur_size, cur_size, format, output_images);

    for (int j=0; j<6; j++) {
      delete[] output_images[j];
    }
    cur_size /= 2;
  }
}

void Environment::prepareCubemap(GLuint* cubemap, int numLevels) {
  glGenTextures(1, cubemap);
  glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, *cubemap);
  glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_MAX_LEVEL, numLevels-1);
  glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, 0);
}

void Environment::saveImagesToCubemap(GLuint cubemap, GLint internal_format, int miplevel, unsigned int width, unsigned int height, GLenum format, float** images) {
  glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, cubemap);
  for (int i=0; i<6; i++)
  {
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X_ARB + i, miplevel, internal_format, width, height, 0, format, GL_FLOAT, images[i]);
  }
  glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, 0);
}

Environment::EnvironmentInfo Environment::prepareEnvironmentInfo(const std::string& name, float strength, float thresh, float exposure, float contrast) {
#if _WIN32  

#if LM_PRODUCTION_BUILD
  const std::string ASSETS_PATH = working_directory + "/assets/";
#else
  const std::string ASSETS_PATH = working_directory + "/../../assets/";
#endif

#else
  const std::string ASSETS_PATH = working_directory + "/assets/";
#endif

  EnvironmentInfo info;
  info._name = name;
  preparePaths(ASSETS_PATH + name + "/noon/", info._noon_images);
  preparePaths(ASSETS_PATH + name + "/dawn/", info._dawn_images);
  preparePaths(ASSETS_PATH + name + "/depth/", info._depth_images);
  info._bloom_strength = strength;
  info._bloom_threshold = thresh;
  info._exposure = exposure;
  info._contrast = contrast;
  return info;
}

void Environment::preparePaths(const std::string& path, std::string* filenames) {
  filenames[0] = path + "x-positive.exr";
  filenames[1] = path + "x-negative.exr";
  filenames[2] = path + "y-positive.exr";
  filenames[3] = path + "y-negative.exr";
  filenames[4] = path + "z-positive.exr";
  filenames[5] = path + "z-negative.exr";
}

void Environment::createEnvironmentInfos() {
  static bool created = false;
  if (!created) {
    // set up filenames for the different supported environments
    _environment_infos.push_back(prepareEnvironmentInfo("Islands", 0.6f, 1.0f, 4.5f, 1.01f));
    _environment_infos.push_back(prepareEnvironmentInfo("Arctic", 0.2f, 2.5f, 2.25f, 1.01f));
    _environment_infos.push_back(prepareEnvironmentInfo("Jungle", 0.5f, 0.5f, 4.5f, 1.01f));
    _environment_infos.push_back(prepareEnvironmentInfo("Jungle-Cliff", 0.6f, 0.7f, 4.5f, 1.01f));
    _environment_infos.push_back(prepareEnvironmentInfo("Redwood", 0.6f, 0.7f, 4.5f, 1.01f));
    _environment_infos.push_back(prepareEnvironmentInfo("Desert", 0.5f, 1.5f, 3.5f, 1.01f));
    _environment_infos.push_back(prepareEnvironmentInfo("River", 0.5f, 0.75f, 3.5f, 1.01f));
    _environment_infos.push_back(prepareEnvironmentInfo("Moonscape", 0.3f, 0.5f, 2.25f, 1.01f));
    created = true;
  }
}

void Environment::createWorkingDirectory() {
  static bool created = false;
  if (!created) {
#if _WIN32
    const size_t bufferSize = 1024;
    char buffer[bufferSize];
    if (_getcwd(buffer, bufferSize) != NULL) {
      working_directory = std::string(buffer);
      for (size_t i=0; i<working_directory.size(); i++) {
        if (working_directory[i] == '\\') {
          working_directory[i] = '/';
        }
      }
    }
#else
    CFBundleRef mainBundle = CFBundleGetMainBundle();
    CFURLRef resourcesURL = CFBundleCopyResourcesDirectoryURL(mainBundle);
    char path[PATH_MAX];
    if (CFURLGetFileSystemRepresentation(resourcesURL, TRUE, (UInt8 *)path, PATH_MAX))
    {
      working_directory = std::string(path);
    }
    CFRelease(resourcesURL);
#endif
    created = true;
  }
}
