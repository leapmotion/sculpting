#ifndef __AUTOSAVE_H__
#define __AUTOSAVE_H__

#include <string>
#include <cinder/Thread.h>
#include "Vertex.h"
#include "Triangle.h"

class Mesh;

class AutoSave {

public:

  AutoSave();
  void start();
  void shutdown();
  void triggerAutoSave(Mesh* mesh);
  bool haveAutoSave() const;
  std::string getAutoSavePath() const;
  void deleteAutoSave();

  static bool isFirstRun();

  static const char PATH_SEPARATOR;

private:

  void runMainLoop();
  void checkAutoSave();

  static std::string getUserAppDirectory();
  static std::string getUserPath(const std::string& filename);

  bool m_shutdown;
  VertexVector m_vertices;
  TriangleVector m_triangles;
  float m_scale;
  std::thread m_saveThread;
  std::mutex m_saveMutex;
  std::condition_variable m_saveCondition;
  bool m_savePending;
  double m_lastSaveTime;

  static const std::string APPLICATION_DIRECTORY;
  static const char PATH_SEPARATOR_WINDOWS;
  static const char PATH_SEPARATOR_UNIX;
  static const double MIN_TIME_BETWEEN_AUTOSAVES;

};

#endif
