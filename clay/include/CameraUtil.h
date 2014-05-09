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
#include "CubeMapManager.h"
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

  // Init camera at the origin.
  CameraUtil();
  
  Vector4 GetIsoStateReferencePosition();
  lmReal IsoQueryRadius(const Mesh* mesh) const;
  
  //Accumulates user input so it can be smoothed and handled in UpdateCamera
  void RecordUserInput(const float _DTheta,const float _DPhi,const float _DFov);

  // Update camera position.
  void UpdateCamera(Mesh* mesh);
  lmTransform GetCameraInWorldSpace();
  lmReal GetReferenceDistance() const;
  
public:
  std::mutex m_referencePointMutex;
  lmReal m_timeOfLastScupt;
  bool m_forceCameraOrbit;

private:
  // Reset camera on the model. 
  // This finds the forwad-facing supporting vertex along the camera directino and relocates the reference point and camera translation.
  void ResetCamera(const Mesh* mesh, const Vector3& cameraDirection, bool keepCloseToMesh = false);
  void OrbitCamera(const Mesh* mesh, lmReal deltaTime);
  lmReal IsoPotential(Mesh* mesh, const Vector3& position, lmReal queryRadius);
  void IsoPotential_row4(Mesh* mesh, const Vector3* positions, lmReal queryRadius, lmReal* potentials);
  Vector3 IsoNormal(Mesh* mesh, const Vector3& position, lmReal queryRadius);
  void IsoUpdateCameraTransform(const Vector3& newDirection, lmReal deltaTime);
  void IsoPreventCameraInMesh(Mesh* mesh);
  void InitIsoCamera(Mesh* mesh);
  void IsoCamera(Mesh* mesh, const Vector3& movement, lmReal deltaTime);
  void IsoCameraConstrainWhenSpinning(Mesh* mesh);
  void IsoResetIfInsideManifoldMesh(Mesh* mesh);
  void IsoOnMeshUpdateStopped(Mesh* mesh);
  void UpdateCameraInWorldSpace();
  lmSurfacePoint GetReferencePoint() const;
  lmReal GetMaxDistanceForMesh(const Mesh* mesh) const;
  lmReal GetMeshSize(const Mesh* mesh) const;

  void UpdateMeshTransform(const Mesh* mesh );

  // Returns true if sphere collides mesh.
  bool CollideCameraSphere(Mesh* mesh, const Vector3& position, lmReal radius);

  bool VerifyCameraMovement(Mesh* mesh, const Vector3& from, const Vector3& to, lmReal radius);

  void CastOneRay(const Mesh* mesh, const lmRay& ray, lmRayCastOutput* result);
  void CastOneRay(const Mesh* mesh, const lmRay& ray, std::vector<lmRayCastOutput>* results, bool collectall = false);

  lmSurfacePoint GetClosestSurfacePoint(Mesh* mesh, const Vector3& position, lmReal queryRadius);

  static void GetBarycentricCoordinates(const Mesh* mesh, int triIdx, const Vector3& point, Vector3* coordsOut);
  static void GetNormalAtPoint(const Mesh* mesh, int triIdx, const Vector3& point, Vector3* normalOut);

  // Correct up vector
  void CorrectCameraUpVector(lmReal dt, const Vector3& up);

  inline Vector3 ToWorldSpace(const Vector3& v);
  inline Vector3 ToMeshSpace(const Vector3& v);

  inline Vector3 GetCameraDirection() const;

private:

  struct IsoCameraState {
    lmReal refDist;
    lmReal cameraOffsetMultiplier; //Might be able to replace with s_isoRefDistMultiplier
    Vector3 refPosition;
    Vector3 refNormal;
    lmSurfacePoint closestPointOnMesh;
    int numFailedUpdates;
  } m_isoState;

  lmTransform m_transform;
  lmTransform m_transformInWorldSpace;

  // Mesh's transform
  lmTransform m_meshTransform;

  // Distance from the surface of the model.
  lmSurfacePoint m_orbitRefPoint;
  lmReal m_orbitDistance;

  // User input from the last call to CameraUpdate().
  Vector3 m_userInput;
  
  lmReal m_lastCameraUpdateTime;
  lmReal m_timeSinceOrbitingStarted;
  lmReal m_timeSinceOrbitingEnded;
  lmReal m_timeSinceCameraUpdateStarted;
  lmReal m_timeOfMovementSinceLastMeshMofification;
  
  lmReal m_prevTimeOfLastSculpt;
  bool m_justSculpted;
  bool m_forceVerifyPositionAfterSculpting;
  int m_numFramesInsideManifoldMesh;

  std::mutex m_transformInWorldSpaceMutex;
  std::mutex m_userInputMutex;
  
  std::vector<int> m_queryTriangles;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif // __CameraUtil_h__
