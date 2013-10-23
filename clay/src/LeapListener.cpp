#include "StdAfx.h"
#include "LeapListener.h"
#include "cinder/app/AppBasic.h"
#include <boost/date_time.hpp>
#include <iostream>

using namespace ci;

LeapListener::LeapListener() : _is_connected(false) { }

void LeapListener::onInit(const Leap::Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void LeapListener::onConnect(const Leap::Controller& controller) {
  std::cout << "Connected" << std::endl;
}

void LeapListener::onDisconnect(const Leap::Controller& controller) {
  std::cout << "Disconnected" << std::endl;
}

void LeapListener::onFrame(const Leap::Controller& controller) {
  static const int MAX_FRAME_QUEUE_SIZE = 3; // max number of frames we can be "behind"
  std::lock_guard<std::mutex> lock(_mutex);
  _is_connected = true;
  _frameQueue.push_back(controller.frame());
  if (_frameQueue.size() > MAX_FRAME_QUEUE_SIZE) {
    _frameQueue.pop_front();
  }
  _condition.notify_all();
}

bool LeapListener::waitForFrame(Leap::Frame& _Frame, int _MillisecondsTimeout) {
  std::unique_lock<std::mutex> lock(_mutex);
  if (_frameQueue.empty() && !_condition.timed_wait(lock, boost::posix_time::milliseconds(_MillisecondsTimeout))) {
    return false;
  }
  _Frame = _frameQueue.front();
  _frameQueue.pop_front();
  return true;
}

bool LeapListener::isConnected() const {
  return _is_connected;
}
