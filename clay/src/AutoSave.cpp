#include "stdafx.h"
#include "AutoSave.h"
#include "Mesh.h"
#include "Files.h"
#ifdef _WIN32
//#pragma comment (lib, "wintrust.lib")
#else // POSIX
#if __APPLE__
#include <ApplicationServices/ApplicationServices.h>
#include <sys/sysctl.h>
#include <copyfile.h>
#else
#include <fstream>
#endif
#include <unistd.h>
#include <sys/stat.h>
#include <cstdlib>
#include <cstdio>
#endif
#include <sstream>
#include <time.h>

const char AutoSave::PATH_SEPARATOR_WINDOWS = '\\';
const char AutoSave::PATH_SEPARATOR_UNIX = '/';
const double AutoSave::MIN_TIME_BETWEEN_AUTOSAVES = 30.0;

#ifdef _WIN32
const char AutoSave::PATH_SEPARATOR = PATH_SEPARATOR_WINDOWS;
#pragma comment (lib,"UserEnv.lib")
#else // POSIX
const char AutoSave::PATH_SEPARATOR = PATH_SEPARATOR_UNIX;
#endif

#if _WIN32 || __APPLE__
const std::string AutoSave::APPLICATION_DIRECTORY = "Freeform";
#else
const std::string AutoSave::APPLICATION_DIRECTORY[] = ".Freeform";
#endif

AutoSave::AutoSave() : m_shutdown(false), m_savePending(false), m_scale(1.0f), m_lastSaveTime(-MIN_TIME_BETWEEN_AUTOSAVES) {

}

void AutoSave::start() {
  m_saveThread = std::thread(&AutoSave::runMainLoop, this);
#if __APPLE__
  m_saveThread.detach();
#endif
}

void AutoSave::shutdown() {
  std::unique_lock<std::mutex> lock(m_saveMutex);
  m_shutdown = true;
  m_saveCondition.notify_all();
  if (m_saveThread.joinable())
  {
    m_saveThread.join();
  }
}

void AutoSave::triggerAutoSave(Mesh* mesh) {
  std::unique_lock<std::mutex> lock(m_saveMutex);
  const double curTime = ci::app::getElapsedSeconds();
  if (curTime - m_lastSaveTime > MIN_TIME_BETWEEN_AUTOSAVES && mesh->getNbVertices() > 0 && mesh->getNbTriangles() > 0) {
    m_vertices = mesh->getVertices();
    m_triangles = mesh->getTriangles();
    m_scale = mesh->getScale();
    m_savePending = true;
    m_saveCondition.notify_all();
    m_lastSaveTime = curTime;
  }
}

bool AutoSave::haveAutoSave() const {
  const std::string savePath = getAutoSavePath();
  return boost::filesystem::exists(savePath);
}

std::string AutoSave::getAutoSavePath() const {
  return getUserPath("autosave.ply");
}

void AutoSave::deleteAutoSave() {
  boost::filesystem::remove(getAutoSavePath());
}

bool AutoSave::isFirstRun() {
  std::stringstream ss;
  ss << getUserAppDirectory() << PATH_SEPARATOR << APPLICATION_DIRECTORY;
  std::string curPath = ss.str();
  return !boost::filesystem::exists(curPath);
}

void AutoSave::runMainLoop() {
  while (!m_shutdown) {
    checkAutoSave();
  }
}

void AutoSave::checkAutoSave() {
  std::unique_lock<std::mutex> lock(m_saveMutex);
  if (!m_savePending) {
    m_saveCondition.wait(lock);
  }
  if (!m_shutdown) {
    try {
      Files files;
      const std::string savePath = getAutoSavePath();
      std::ofstream file(savePath.c_str());
      if (file) {
        files.savePLY(m_vertices, m_triangles, m_scale, file);
        file.close();
      }
    } catch (...) {}
  }
  m_savePending = false;
}

std::string AutoSave::getUserAppDirectory() {
#if _WIN32
  const char* appdata = getenv("APPDATA");
  return std::string(appdata ? appdata : "C:/");
#else
  const char* home = getenv("HOME");
  std::string appDir(home ? home : "~");
#if __APPLE__
  appDir += "/Library/Application Support";
#endif
  return appDir;
#endif
}

std::string AutoSave::getUserPath(const std::string& filename) {
  std::stringstream ss;
  ss << getUserAppDirectory() << PATH_SEPARATOR << APPLICATION_DIRECTORY;
  std::string curPath = ss.str();
  if (!boost::filesystem::exists(curPath)) {
    boost::filesystem::create_directory(curPath);
  }
  ss << PATH_SEPARATOR << filename;
  curPath = ss.str();
  for (size_t i=0; i<curPath.size(); i++) {
    if (curPath[i] == PATH_SEPARATOR_WINDOWS) {
      curPath[i] = PATH_SEPARATOR_UNIX;
    }
  }
  return curPath;
}
