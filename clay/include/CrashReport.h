/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#ifndef __CRASHREPORT_H__
#define __CRASHREPORT_H

#if _WIN32
#define USE_CRASH_REPORTING 1
#else
#define USE_CRASH_REPORTING 0
#endif

#if USE_CRASH_REPORTING
namespace google_breakpad {
  class ExceptionHandler;
}
#endif

class CrashReport {
public:
  typedef void (*ExtraHandler)();
public:
  CrashReport(ExtraHandler extraHandler=0);
  ~CrashReport();
  void CallExtraHandler() const;
#if USE_CRASH_REPORTING
private:
  google_breakpad::ExceptionHandler*  m_ExceptionHandler;
  ExtraHandler                        m_ExtraHandler;
#endif
};

#if USE_CRASH_REPORTING
// safe to call with on null instance
inline void CrashReport::CallExtraHandler() const {
  if ( this && m_ExtraHandler ) { 
    (*m_ExtraHandler)(); 
  } 
}
#else
inline CrashReport::CrashReport(ExtraHandler) {}
inline CrashReport::~CrashReport() {}
inline void CrashReport::CallExtraHandler() const {}
#endif

#endif
