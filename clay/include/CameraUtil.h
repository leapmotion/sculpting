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

#include "Geometry.h" // for GetClosestPointOutput declaration


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
  inline void invalidate() { triangleIdx = -1; dist = -1.0f; fraction = -1.0f; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct lmSurfacePoint {
  Vector3 position;
  Vector3 normal;

  lmSurfacePoint() {}
  lmSurfacePoint(const Vector3& p, const Vector3& n) : position(p), normal(n) {}
  void set(const Vector3& p, const Vector3& n) { position = p; normal = n; }
  void setZero() { position.setZero(); normal.setZero(); }

  void mul(const lmQuat& q) {
    position = q * position;
    normal = q * normal;
  }

  void mul(const lmTransform& t) {
    LM_ASSERT(false, "Never used.");
    position = t.rotation * position + t.translation;
    normal = t.rotation * normal;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


// Camera object. todo: rename.
class CameraUtil {
public:
  struct Params {
    lmReal isoRefDistMultiplier;
    lmReal grav_k;
    lmReal grav_n;

    bool queryTriangles;

    int numRotationClipIterations;

    lmReal isoQueryPaddingRadius;

    bool clipToIsoSurface;
    bool clipCameraMovement;
    lmReal refDistForMovemement;
    bool enableConeClipping;
    lmReal normalConeAngle;
    bool enableMaxReorientationRate;
    lmReal maxReorientationRate;
    lmReal scaleZMovement;

    lmReal minDist;
    lmReal maxDist;
    lmReal speedAtMinDist;
    lmReal speedAtMaxDist;
    bool pinUpVector;
    lmReal smoothingFactor;
    bool drawDebugLines;
    bool drawSphereQueryResults;


    // Detailed camera options
    lmReal inputMultiplier;
    bool invertCameraInput;
    bool enableSmoothing;

    bool forceCameraOrbit;
    bool enableCameraReset;
    bool enableCameraOrbit;

    bool preventCameraInMesh;

    Params() {
      isoRefDistMultiplier = 2.0f;
      grav_k = 0.0001f;
      grav_n = 2.0f;
      queryTriangles = true;
      numRotationClipIterations = 1;
      isoQueryPaddingRadius = 50.0f;
      clipToIsoSurface = false;
      clipCameraMovement = false;
      refDistForMovemement = 50.0f;
      enableConeClipping = false;
      normalConeAngle = 45 * LM_DEG;
      enableMaxReorientationRate = false;
      maxReorientationRate = LM_PI / 2.0f; // 180deg in 2 seconds
      scaleZMovement = 0.75f;

      minDist= 30.0f;
      maxDist = 600.0f;
      speedAtMinDist = 0.5f;
      speedAtMaxDist = 2.5f;
      pinUpVector = true;
      smoothingFactor = 0.1f;
      drawDebugLines = false;
      drawSphereQueryResults = false;

      inputMultiplier = 2.0f;
      invertCameraInput = false;
      enableSmoothing = false;

      forceCameraOrbit = false;
      enableCameraReset = true;
      enableCameraOrbit = true;
    }
  };

  // Init camera at the origin.
  CameraUtil();

  // Set camera from standard parameters: camera-from, camera-to, and up vector.
  void SetFromStandardCamera(const Vector3& from, const Vector3& to, lmReal referenceDistance);

  // Reset camera on the model. 
  // This finds the forwad-facing supporting vertex along the camera directino and relocates the reference point and camera translation.
  void ResetCamera(const Mesh* mesh, const Vector3& cameraDirection, bool keepCloseToMesh = false);

  // Orbit camera around the mesh
  void OrbitCamera(const Mesh* mesh, lmReal deltaTime);

  struct IsoCameraState {
    lmReal refDist;
    lmReal cameraOffsetMultiplier;
    Vector3 refPosition;
    Vector3 refNormal;
    lmSurfacePoint closestPointOnMesh;
    lmReal refPotential; // used when clipping to isosurface
    int numFailedUpdates;
  } isoState;

  lmReal IsoPotential(Mesh* mesh, const Vector3& position, lmReal queryRadius);

  void IsoPotential_row4( Mesh* mesh, const Vector3* positions, lmReal queryRadius, lmReal* potentials );

  Vector3 IsoNormal( Mesh* mesh, const Vector3& position, lmReal queryRadius);

  lmReal IsoQueryRadius(const Mesh* mesh, IsoCameraState* state) const;

  void IsoUpdateCameraTransform(const Vector3& newDirection, IsoCameraState* state, lmReal deltaTime);

  void IsoPreventCameraInMesh(Mesh* mesh, IsoCameraState* state);

  void InitIsoCamera(Mesh* mesh, IsoCameraState* state);

  void IsoCamera(Mesh* mesh, IsoCameraState* state, const Vector3& movement, lmReal deltaTime);

  void IsoCameraConstrainWhenSpinning(Mesh* mesh, IsoCameraState* state);

  void IsoResetIfInsideManifoldMesh(Mesh* mesh, IsoCameraState* isoState);

  void IsoOnMeshUpdateStopped(Mesh* mesh, IsoCameraState* state);

  void IsoMoveTowardsPotential(Mesh* mesh, IsoCameraState* state);

  // Records user mouse input from mouse events.
  // 
  // Accumulates data for later processing in UpdateCamera.
  void RecordUserInput(const float _DTheta,const float _DPhi,const float _DFov);

  // Update camera position.
  void UpdateCamera(Mesh* mesh, Params* paramsInOut);

  lmTransform GetCameraInWorldSpace();

  void UpdateCameraInWorldSpace();

  lmSurfacePoint GetReferencePoint() const;

  lmReal GetReferenceDistance() const;

  lmReal GetMaxDistanceForMesh(const Mesh* mesh) const;

  lmReal GetMeshSize(const Mesh* mesh) const;

private:

  void UpdateMeshTransform(const Mesh* mesh, Params* paramsInOut );

  // Returns true if sphere collides mesh.
  bool CollideCameraSphere(Mesh* mesh, const Vector3& position, lmReal radius);

  bool VerifyCameraMovement(Mesh* mesh, const Vector3& from, const Vector3& to, lmReal radius);

  // Helper functions:
  void CastOneRay(const Mesh* mesh, const lmRay& ray, lmRayCastOutput* result);

  void CastOneRay(const Mesh* mesh, const lmRay& ray, std::vector<lmRayCastOutput>* results, bool collectall = false);

  // Compute camera transform from standard camera vectors: from, to, & assumed up along y-axis.
  //
  // unused
  static void GetTransformFromStandardCamera(const Vector3& from, const Vector3& to, lmTransform& tOut);

  lmSurfacePoint GetClosestSurfacePoint(Mesh* mesh, const Vector3& position, lmReal queryRadius);

  // Helper functions.

  // todo: move to MathUtils ?
  static void GetBarycentricCoordinates(const Mesh* mesh, int triIdx, const Vector3& point, Vector3* coordsOut);

  static void GetNormalAtPoint(const Mesh* mesh, int triIdx, const Vector3& point, Vector3* normalOut);

  // Correct up vector
  void CorrectCameraUpVector(lmReal dt, const Vector3& up);

  // Rotate vectors to mesh's space
  inline Vector3 ToWorldSpace(const Vector3& v);

  // Rotate vectors to mesh's space
  inline Vector3 ToMeshSpace(const Vector3& v);

  inline Vector3 GetCameraDirection() const;

  // For simplicity.
public:

  // Camera's current transform.
  lmTransform m_transform;

  // Camera's current transform.
  lmTransform m_transformInWorldSpaceForGraphics;

  // Mesh's transform
  lmTransform m_meshTransform;

  int m_framesFromLastCollisions;

  // Distance from the surface of the model.
  lmSurfacePoint m_orbitRefPoint;
  lmReal m_orbitDistance;

  // User input from the last call to CameraUpdate().
  Vector3 m_userInput;

  // Accumulated user input, waiting to be used over subsequent frames
  Vector3 m_accumulatedUserInput;

  // Time of last camera update
  lmReal m_lastCameraUpdateTime;

  lmReal m_timeSinceOrbitingStarted;
  lmReal m_timeSinceOrbitingEnded;
  lmReal m_timeSinceCameraUpdateStarted;
  lmReal m_timeOfMovementSinceLastMeshMofification;
  lmReal m_timeOfLastScupt;
  lmReal m_prevTimeOfLastSculpt;
  bool m_justSculpted;
  bool m_forceVerifyPositionAfterSculpting;
  int m_numFramesInsideManifoldMesh;

  Params m_params;

  std::mutex m_transformForGraphicsMutex;
  std::mutex m_userInputMutex;
  std::mutex m_referencePointMutex;

  std::vector<int> m_queryTriangles;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif // __CameraUtil_h__
