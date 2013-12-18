#ifndef __PRINT3D_H__
#define __PRINT3D_H__

#include <curl/curl.h>

class Print3D {

public:

  Print3D();
  ~Print3D();

  bool Upload(const std::string& filepath, const std::string& filename);
  void Cancel() { m_abort = true; }
  double Progress() const { return m_progress; }
  void LaunchForm(const std::string& filename, const std::string& name, const std::string& description);

  static void Init();
  static void Cleanup();

private:

  size_t readContent(char* ptr, size_t size, size_t nmemb);
  size_t writeContent(char* ptr, size_t size, size_t nmemb);
  int progressFunction(double dltotal, double dlnow, double ultotal, double ulnow);

  static size_t readContentStatic(char* ptr, size_t size, size_t nmemb, void* userdata);
  static size_t writeContentStatic(char* ptr, size_t size, size_t nmemb, void* userdata);
  static int progressFunctionStatic(void* clientp, double dltotal, double dlnow, double ultotal, double ulnow);

  static std::string createUploadURL(const std::string& username, const std::string& filename, size_t numBytes);

  CURL* m_curl;
  std::string m_data;
  size_t m_readOffset;
  double m_progress;
  bool m_abort;
  std::string m_response;

};

#endif
