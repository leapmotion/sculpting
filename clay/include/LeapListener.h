#ifndef __LEAPLISTENER_H__
#define __LEAPLISTENER_H__

#include "cinder/Thread.h"
#include "Leap.h"
#include <deque>

class LeapListener : public Leap::Listener {

public:

  LeapListener();
  virtual void onInit( const Leap::Controller& );
  virtual void onConnect( const Leap::Controller& );
  virtual void onDisconnect( const Leap::Controller& );
  virtual void onFrame( const Leap::Controller& );
  bool waitForFrame(Leap::Frame& _Frame, int _MillisecondsTimeout);
  bool isConnected() const;
  bool isReceivingFrames() const;

private:

  bool _is_connected;
  double _last_frame_time;
  std::deque<Leap::Frame> _frameQueue;
  std::mutex _mutex;
  std::condition_variable _condition;

};

#endif
