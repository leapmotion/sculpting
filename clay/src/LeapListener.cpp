#include "StdAfx.h"
#include "LeapListener.h"
#include "cinder/app/AppBasic.h"
#include <boost/date_time.hpp>
#include <iostream>

using namespace ci;

LeapListener::LeapListener() : _is_connected(false), _last_frame_time(0.0) { }

void LeapListener::onInit(const Leap::Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void LeapListener::onConnect(const Leap::Controller& controller) {
  _is_connected = true;
  std::cout << "Connected" << std::endl;
}

void LeapListener::onDisconnect(const Leap::Controller& controller) {
  _is_connected = false;
  std::cout << "Disconnected" << std::endl;
}

void LeapListener::onFrame(const Leap::Controller& controller) {
  static const int MAX_FRAME_QUEUE_SIZE = 3; // max number of frames we can be "behind"
  std::lock_guard<std::mutex> lock(_mutex);
  if (!isConnected()) {
    _frameQueue.clear();
    _is_connected = true;
  }
  _frameQueue.push_back(controller.frame());
  if (_frameQueue.size() > MAX_FRAME_QUEUE_SIZE) {
    _frameQueue.pop_front();
  }
  _last_frame_time = ci::app::getElapsedSeconds();
  _condition.notify_all();
}

bool LeapListener::waitForFrame(Leap::Frame& _Frame, int _MillisecondsTimeout) {
  std::unique_lock<std::mutex> lock(_mutex);
#if _WIN32
  if (_frameQueue.empty() && !_condition.timed_wait(lock, boost::posix_time::milliseconds(_MillisecondsTimeout))) {
    return false;
  }
#else
  if (_frameQueue.empty() && _condition.wait_for(lock, std::chrono::milliseconds(_MillisecondsTimeout)) == std::cv_status::timeout) {
    return false;
  }
#endif
  _Frame = _frameQueue.front();
  _frameQueue.pop_front();
  return true;
}

bool LeapListener::isConnected() const {
  static const double CONNECTION_TIMEOUT = 0.5; // seconds until controller is assumed to be dead
  const double curTime = ci::app::getElapsedSeconds();
  return _is_connected && fabs(curTime - _last_frame_time) < CONNECTION_TIMEOUT;
}
