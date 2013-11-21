#pragma once
#ifndef __ReplayUtil_h__
#define __ReplayUtil_h__

#include <vector>
#include <streambuf>
#include <iostream>

#define ENABLE_REPLAY_UTIL 0

#define LM_USE_ITEMS 0

#ifndef LM_BREAK
# define LM_BREAK __asm { int 3 }
#endif
#ifndef LM_ASSERT
# define LM_ASSERT(condition, message) if (!(condition)) LM_BREAK;
#endif


class ReplayUtil
{
public:
  static ReplayUtil& getInstance() { static ReplayUtil instance; return instance; }

  enum Mode {
    MODE_INACTIVE = -1,
    MODE_RECORD,
    MODE_REPLAY
  };

  struct Item {
    union {
      float m_a;
      double m_b;
      int m_c;
      char m_buffer[32];
    } m_value;

    float asFloat() { return m_value.m_a; }
    double asDouble() { return m_value.m_b; }

    friend std::ostream& operator << (std::ostream& os, const Item& i);
    friend std::istream& operator >> (std::istream& is, Item& i);
  };

  ReplayUtil();
  ~ReplayUtil();

  void setMode(Mode mode);
  Mode getMode() const { return m_mode; }

  void loadHistory();
  void clearHistoryFile();
  void saveHistory();

  void addItem(const Item& item);
  const Item& getItem();

  void trackItem(Item& item);

  template<typename T> void addObject( const T& obj );
  template<typename T> void getObject(T* obj);
  template<typename T> void trackObject(T& obj);

  template <typename T> void trackValue(T& value);
  template <typename T> void trackConstValue(const T& item);

  template <typename T> T returnTrackedValue(const T& value);
  template <typename T> T returnTrackedValue();

  template <typename T>
  void assertIdentical(const T& value);

private:
  void initMode();

  Mode m_mode;

  std::vector<Item> m_history;
  int m_historyIdx;

  std::vector<char> m_historyStream;
  int m_historyStreamIdx;

  static const char* configFilename;
  static const char* historyFilename;
};

std::ostream& operator << (std::ostream& os, const ReplayUtil::Item& i);
std::istream& operator >> (std::istream& is, ReplayUtil::Item& i);

#if ENABLE_REPLAY_UTIL

#define LM_START_RECORD() ReplayUtil::getInstance().setMode(ReplayUtil::MODE_RECORD)
#define LM_START_REPLAY() ReplayUtil::getInstance().setMode(ReplayUtil::MODE_REPLAY)

#define LM_TRACK_IS_ACTIVE() (ReplayUtil::getInstance().getMode() != ReplayUtil::MODE_INACTIVE)
#define LM_TRACK_IS_RECORDING() (ReplayUtil::getInstance().getMode() == ReplayUtil::MODE_RECORD)
#define LM_TRACK_IS_REPLAYING() (ReplayUtil::getInstance().getMode() == ReplayUtil::MODE_REPLAY)

#define LM_TRACK_VALUE(value) ReplayUtil::getInstance().trackValue(value)
#define LM_TRACK_CONST_VALUE(value) ReplayUtil::getInstance().trackConstValue(value)

#define LM_RETURN_TRACKED(value) ReplayUtil::getInstance().returnTrackedValue(value)
#define LM_RETURN_TRACKED_NEXT(type) ReplayUtil::getInstance().returnTrackedValue<type>()

#define LM_RETURN_TRACKED_CONDITIONAL(condition, value, type) (condition?LM_RETURN_TRACKED(value):LM_RETURN_TRACKED_NEXT(type))

#define LM_ASSERT_IDENTICAL(value) ReplayUtil::getInstance().assertIdentical(value)

#define LM_TRACK_TODO() 

#else

#define LM_START_RECORD()
#define LM_START_REPLAY()

#define LM_TRACK_IS_ACTIVE() false
#define LM_TRACK_IS_RECORDING() false
#define LM_TRACK_IS_REPLAYING() false

#define LM_TRACK_VALUE(value) value
#define LM_TRACK_CONST_VALUE(value) value

#define LM_RETURN_TRACKED(value) value

#define LM_RETURN_TRACKED_CONDITIONAL(condition, value, type) value // LM_ASSERT(condition)

#define LM_ASSERT_IDENTICAL(value) value

#define LM_TRACK_TODO()

#endif

template <bool b> struct StaticAssert {};
template <> struct StaticAssert<true> { static void Assert() {} };
#define LM_COMPILE_TIME_ASSERT(condition) StaticAssert<condition>::Assert();

//
// Base object storing
//

template<typename T>
void ReplayUtil::addObject( const T& obj )
{
  int streamEndIdx = m_historyStream.size();
  m_historyStream.resize(streamEndIdx + sizeof(obj));
  T* result = reinterpret_cast<T*>(&m_historyStream[0]+streamEndIdx);
  std::memcpy(result, &obj, sizeof(obj));
  //*result = item;
}

template<typename T>
void ReplayUtil::getObject(T* obj)
{
  // assert in bounds
  const T* result = reinterpret_cast<const T*>(&m_historyStream[0]+m_historyStreamIdx);
  m_historyStreamIdx += sizeof(T);
  std::memcpy(obj, result, sizeof(T));
  //std::memcpy(const_cast<void*>(reinterpret_cast<const void*>(obj)), result, sizeof(T));
}

template<typename T>
void ReplayUtil::trackObject(T& obj) {
  if (m_mode == MODE_RECORD) {
    addObject(obj);
    saveHistory();
  } else if (m_mode == MODE_REPLAY) {
    getObject(&obj);
  }
}


//
// User functions
//

template <typename T>
void ReplayUtil::trackValue(T& value) {
#if LM_USE_ITEMS
  Item container;
  LM_COMPILE_TIME_ASSERT(sizeof(Item) >= sizeof(T));
  *reinterpret_cast<T*>(&container) = value;
  trackItem(container);
  value = *reinterpret_cast<T*>(&container);
#else
  trackObject(value);
#endif
}

template <typename T>
void ReplayUtil::trackConstValue(const T& value) {
#if LM_USE_ITEMS
  Item container;
  LM_COMPILE_TIME_ASSERT(sizeof(Item) >= sizeof(T));
  *reinterpret_cast<T*>(&container) = value;
  trackItem(container);
  const_cast<T&>(value) = *reinterpret_cast<T*>(&container);
#else
  trackObject(const_cast<T&>(value));
#endif
}

template <typename T>
T ReplayUtil::returnTrackedValue(const T& value) {
#if LM_USE_ITEMS
  Item container;
  LM_COMPILE_TIME_ASSERT(sizeof(Item) >= sizeof(T));
  *reinterpret_cast<T*>(&container) = value;
  trackItem(container);
  T result  = *reinterpret_cast<T*>(&container);
  return result;
#else
  T result;
  std::memcpy(&result, &value, sizeof(T));
  trackObject(result);
  return result;
#endif
}

template <typename T>
T ReplayUtil::returnTrackedValue() {
  LM_ASSERT(m_mode == MODE_REPLAY, "Trying to extract recorded value (condition not met) during recording.")
#if LM_USE_ITEMS
  trackItem(container);
  T result  = *reinterpret_cast<T*>(&container);
  return result;
#else
  T result;
  trackObject(result);
  return result;
#endif
}

template <typename T>
void ReplayUtil::assertIdentical(const T& value) {
  T copy;
  std::memcpy(&copy, &value, sizeof(T));
  T recorded = returnTrackedValue(value);
  LM_ASSERT(m_mode != MODE_REPLAY || copy == recorded, "LM_ASSERT_IDENTICAL failed.");
}



#endif // __ReplayUtil_h__
