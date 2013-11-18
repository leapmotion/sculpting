#pragma once

#include <iostream>
#include <map>
#include "cinder/app/App.h"
#include "DataTypes.h"

// Debug console output
class DemoUtil {
public:
  static std::ostream& LogStream(const char* id, float logPeriod) {
    static std::map<const char*, double> timestamps;
    std::map<const char*, double>::const_iterator it = timestamps.find(id);
    double lastTime = 0.0;
    if (it != timestamps.end()) {
      lastTime = it->second;
    }
    double time = ci::app::getElapsedSeconds();
    if (logPeriod < time - lastTime) 
    {
      timestamps[id] = time;
      return std::cout;
    } else {
      static std::ostream cnul(0);
      cnul.clear();
      return cnul;
    }
  }
};

#define LM_STRING(x) #x
#define LM_LOG_STREAM(file, line, period) DemoUtil::LogStream(file ":" LM_STRING(line), period)

#define LM_LOG LM_LOG_STREAM(__FILE__, __LINE__, 1)
#define LM_LOG2(logPeriod) LM_LOG_STREAM(__FILE__, __LINE__, logPeriod)

// todo: allow logs with id's passed as arguments. Can't do this now as the map stores the pointers to strings.
// so they have to be constant in mem.
//#define LM_LOG_ID(id) LM_LOG_STREAM_ID(__FILE__, __LINE__, id, 1)

template <typename T>
inline T lmClip(T val, T min, T max) {
   val = std::min(val, max);
   val = std::max(min, val);
   return val;
}

template <typename T>
inline T lmInterpolate(lmReal t, const T& v0, const T& v1) {
  return (1-t)*v0+t*v1;
}

#define LM_EPSILON 0.00001f
#define LM_EPSILON_SQR (LM_EPSILON * LM_EPSILON)
#define LM_PI 3.14159265359f
#define LM_2PI (2.0f * 3.14159265359f)
#define LM_DEG (LM_PI / 180.0f) // deg to radians

inline bool lmIsNormalized(const Vector3& v) { lmReal sqn = v.squaredNorm(); return 0.99f < sqn && sqn < 1.01f; }
inline bool lmIsZero(const Vector3& v) { return v.squaredNorm() < LM_EPSILON_SQR; }
inline bool lmIsFinite(const Vector3& v) { return v.norm() < FLT_MAX; }
template <typename T>
inline bool lmIsInRange(const T& v, const T& min, const T& max) { return min <= v && v <= max; }

#define TODO(owner, message) LM_LOG << #owner << ": " << #message << std::endl;

#define LM_PRODUCTION_BUILD 0

#if !LM_PRODUCTION_BUILD
# define LM_BREAK __asm { int 3 }
# define LM_ASSERT(condition, message) if (!(condition)) LM_BREAK;
#else
# define LM_BREAK
# define LM_ASSERT(condition, message)
#endif

#if _WIN32
#pragma warning(disable : 4996) // 'scanf': This function or variable may be unsafe. Consider using scanf_s instead
#endif
