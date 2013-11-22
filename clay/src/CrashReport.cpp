/*==================================================================================================================

    Copyright (c) 2010 - 2012 Leap Motion. All rights reserved.

  The intellectual and technical concepts contained herein are proprietary and confidential to Leap Motion, and are
  protected by trade secret or copyright law. Dissemination of this information or reproduction of this material is
  strictly forbidden unless prior written permission is obtained from Leap Motion.

===================================================================================================================*/
#include "CrashReport.h"

#if USE_CRASH_REPORTING

#if _WIN32
#include "client/windows/handler/exception_handler.h"
#include <Userenv.h>
#else
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#if __APPLE__
#include "client/mac/handler/exception_handler.h"
#include "common/mac/MachIPC.h"
#else
#include "client/linux/handler/exception_handler.h"
#endif
#endif
#include "google_breakpad/processor/minidump.h"

#include "boost/filesystem.hpp"
#include <algorithm>
#include <string>
#include <sstream>

namespace CrashReportCallbacks {

#if _WIN32
static bool MDCallback(const wchar_t* dump_dir, const wchar_t* minidump_id,
                       void* context, EXCEPTION_POINTERS* exinfo,
                       MDRawAssertionInfo* assertion, bool success)
{
  std::wstring path(dump_dir);
  path.append(L"\\");
  path.append(minidump_id);
  path.append(L".dmp");

  std::wcout << "fatal error, crash minidump saved to " << path << "\n";

  // CallExtraHandler() is null-safe
  reinterpret_cast<CrashReport*>(context)->CallExtraHandler();

  return true;
}
#elif __APPLE__
static bool MDCallback(const char *dump_dir, const char *file_name,
                       void *context, bool success)
{
  string path(dump_dir);
  path.append("/");
  path.append(file_name);
  path.append(".dmp");

  std::cout << "fatal error, crash minidump saved to " << path << "\n";

  // CallExtraHandler() is null-safe
  reinterpret_cast<CrashReport*>(context)->CallExtraHandler();

  return true;
}
#else
static bool MDCallback(const google_breakpad::MinidumpDescriptor& descriptor,
                       void* context, bool success)
{
  string path(descriptor.path());

  std::cout << "fatal error, crash minidump saved to " << path << "\n";

  // CallExtraHandler() is null-safe
  reinterpret_cast<CrashReport*>(context)->CallExtraHandler();

  return true;
}
#endif
}

const char WIN_PATH_SEPARATOR = '\\';
const char UNIX_PATH_SEPARATOR = '/';

#if _WIN32
  const char PATH_SEPARATOR = WIN_PATH_SEPARATOR;
#else
  const char PATH_SEPARATOR = UNIX_PATH_SEPARATOR;
#endif

#if _WIN32 || __APPLE__
  const char APPLICATION_DIRECTORY[] = "Leap Motion"; // use this directory located under [user]/AppData/Roaming/
#else
  const char APPLICATION_DIRECTORY[] = ".Leap Motion"; // use this directory located under [user]/AppData/Roaming/
#endif

static std::string GetUserAppDirectory() {
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

static std::string GetUserPath(const std::string& filename) {
  std::stringstream ss;
  ss << GetUserAppDirectory() << PATH_SEPARATOR << APPLICATION_DIRECTORY;
  std::string curPath = ss.str();
  if (!boost::filesystem::is_directory(curPath)) {
    boost::filesystem::create_directory(curPath);
  }
  ss << PATH_SEPARATOR << filename;
  curPath = ss.str();
  std::replace( curPath.begin(), curPath.end(), WIN_PATH_SEPARATOR, UNIX_PATH_SEPARATOR);
  return curPath;
}

static std::string GetDumpPath(const std::string& filename) {
#ifdef _WIN32
  CHAR pathBuf[1024];
  DWORD count = 1024;
  std::stringstream ss;
  if (GetAllUsersProfileDirectoryA(pathBuf, &count)) {
    ss << pathBuf << PATH_SEPARATOR << "Leap Motion" << PATH_SEPARATOR;
  }
  std::string curPath = ss.str();
  if (!boost::filesystem::is_directory(curPath)) {
    boost::filesystem::create_directory(curPath);
  }
  ss << filename;
  curPath = ss.str();
  std::replace( curPath.begin(), curPath.end(), WIN_PATH_SEPARATOR, UNIX_PATH_SEPARATOR);
  return curPath;
#else
  return GetUserPath(filename);
#endif
}

static int GetFilesInDirectory(const std::string& path, std::vector<std::string>& filenames, const std::string& extension, int ageInDays) {
  filenames.clear();
  boost::filesystem::path dir(path);
  if (!boost::filesystem::exists(dir)) {
    return 0;
  }
  std::time_t prevTime = ageInDays <= 0 ? time(0) : time(0)-86400*ageInDays;
  boost::filesystem::directory_iterator endIt; // default construction yields past-the-end
  for (boost::filesystem::directory_iterator it(dir); it != endIt; ++it) {
    if (boost::filesystem::is_regular_file(it->status())) {
      if (extension.empty() || !it->path().extension().string().compare(extension)) {
        if (difftime(boost::filesystem::last_write_time(it->path()), prevTime) < 0.0) {
          filenames.push_back(it->path().string());
        }
      }
    }
  }
  return static_cast<int>(filenames.size());
}

CrashReport::CrashReport( ExtraHandler extraHandler ) 
  : m_ExceptionHandler(NULL),
    m_ExtraHandler( extraHandler )
{
  const std::string dump_path = GetDumpPath("");
  std::vector<std::string> files;
  const int count = GetFilesInDirectory(dump_path, files, ".dmp", 0);

  if (count >= 10) {
    std::cout << "At least 10 *.dmp files found under " << dump_path << "\n";
    std::cout << "Will not save more automated crash dumps until these are cleaned up.\n";
  } else {
#if _WIN32
    std::wstring dump_path_wide(dump_path.begin(), dump_path.end());
    m_ExceptionHandler = new google_breakpad::ExceptionHandler(dump_path_wide, NULL,
                                                               CrashReportCallbacks::MDCallback, this,
                                                               google_breakpad::ExceptionHandler::HANDLER_ALL);
#elif __APPLE__
    m_ExceptionHandler = new google_breakpad::ExceptionHandler(dump_path, NULL,
                                                               CrashReportCallbacks::MDCallback, this,
                                                               true, NULL);
#else
    google_breakpad::MinidumpDescriptor descriptor(dump_path.c_str());
    m_ExceptionHandler = new google_breakpad::ExceptionHandler(descriptor, NULL,
                                                               CrashReportCallbacks::MDCallback, this,
                                                               true, -1);
#endif
  }
}

CrashReport::~CrashReport()
{
  delete m_ExceptionHandler;
}

#endif
