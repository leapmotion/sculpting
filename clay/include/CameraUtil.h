#pragma once
#ifndef __CameraUtil_h__
#define __CameraUtil_h__

#include "DataTypes.h"
#include <vector>

#include "cinder/app/AppNative.h"
#include "cinder/params/Params.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Fbo.h"
#include "Resources.h"
#include "Environment.h"
#include "UserInterface.h"
#include "LeapListener.h"
#include "Leap.h"
#include "LeapInteraction.h"
#include "Utilities.h"
#include "cinder/Thread.h"
#include "Mesh.h"
#include "Sculpt.h"


class Mesh;

struct lmRay {
  lmRay() {}
  lmRay(const Vector3& s, const Vector3& e) : start(s), end(e) {}
  Vector3 start;
  Vector3 end;
  inline Vector3 GetDirection() const { return (end-start).normalized(); }
  inline lmReal GetLength() const { return (end-start).norm(); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct lmRayCastOutput {
  lmRayCastOutput() : triangleIdx(-1), dist(-1.0), fraction(-1.0) {}
  lmRayCastOutput(int triIdx, const Vector3& pt, const Vector3& norm) : triangleIdx(triIdx), position(pt), normal(norm) {}
  int triangleIdx;
  Vector3 position;
  Vector3 normal;
  lmReal dist;
  lmReal fraction;

  inline bool isSuccess() const { return 0 <= triangleIdx; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct lmSurfacePoint {
  Vector3 position;
  Vector3 normal;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Camera object. todo: rename.
class CameraUtil {
public:
  // Init camera at the origin.
  CameraUtil();

  // Set camera from standard parameters: camera-from, camera-to, and up vector.
  void SetFromStandardCamera(const Vector3& from, const Vector3& to, lmReal referenceDistance);

  // Records user mouse input from mouse events.
  // 
  // Accumulates data for later processing in UpdateCamera.
  void RecordUserInput(const float _DTheta,const float _DPhi,const float _DFov);

  // Records user Leap's pinching drag events.
  // 
  // Accumulates data for later processing in UpdateCamera.
  void RecordUserInput(const Vector3& delta);

  // Update camera position.
  void UpdateCamera(const Mesh* mesh);

private:
  // Generates a batch of rays from camera point in the camera's looking direction.
  void GenerateRays(const lmTransform& transform, int castsPerRow, std::vector<lmRay>* rays);

  // todo: Move to an utility.
  void CastRays(const Mesh* mesh, const std::vector<lmRay>& rays, std::vector<lmRayCastOutput>* results);

  // Helper functions.

  // todo: move to MathUtils ?
  static void GetBarycentricCoordinates(const Mesh* mesh, int triIdx, const Vector3& point, Vector3* coordsOut);

  // Correct up vector
  void CorrectCameraUpVector(const Vector3& up);

  // For simplicity.
public:

  // Camera's current transform.
  lmTransform transform;

  // Most recent reference point on the model (used to compute camera's movement).
  // Unused.
  lmSurfacePoint referencePoint;

  // Distance from the surface of the model.
  // Unused.
  lmReal referenceDistance;

  // User input from the last call to CameraUpdate().
  Vector3 userInput;

  // Accumulated user input, waiting to be used over subsequent frames
  Vector3 accumulatedUserInput;

  enum State {
    STATE_INVALID = -1,
    STATE_FREEFLOATING
  };

  // Current state of the camera.
  State state;

public:

  // Settings
  static const int RAY_CAST_BATCH_SIDE = 1;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif // __CameraUtil_h__
