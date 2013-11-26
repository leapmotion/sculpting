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
class DebugDrawUtil;

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
    bool cameraOverrideIso;
    lmReal isoRefDistMultiplier;
    lmReal grav_k;
    lmReal grav_n;

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

    bool useIsoNormal;
    lmReal isoMultiplier;

    lmReal minDist;
    lmReal maxDist;
    lmReal speedAtMinDist;
    lmReal speedAtMaxDist;
    bool pinUpVector;
    lmReal smoothingFactor;
    bool drawDebugLines;
    bool drawSphereQueryResults;


    // Detailed camera options
    bool useSphereQuery;
    lmReal sphereRadiusMultiplier;
    bool sphereCrawlMode;
    bool userFaultyTriangles;

    bool freeRotationEnabled;
    lmReal freeRotationRatio;
    lmReal inputMultiplier;
    bool invertCameraInput;
    bool enableNormalCorrection;
    bool useAvgNormal;

    bool suppresForwardRotation;
    bool tmpSwitch;

    bool enableSmoothing;
    bool weightNormals;

    bool clipTranslationOnFreeRotate;

    bool walkSmoothedNormals;

    bool overrideNormal;
    bool useClosestPointForEdges;

    bool moveInNormalPlane;
    bool enableBackSnapping;
    bool enableForwardCheckForBackSnapping;

    bool forceCameraOrbit;

    bool enableCameraReset;
    bool enableCameraOrbit;
    bool preventCameraInMesh;
    Params() {
      cameraOverrideIso = true;
      isoRefDistMultiplier = 2.0f;
      grav_k = 0.0001f;
      grav_n = 2.0f;
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


      useIsoNormal = false;

      isoMultiplier = 10.0f;
      minDist= 30.0f;
      maxDist = 600.0f;
      speedAtMinDist = 0.5f;
      speedAtMaxDist = 2.5f;
      pinUpVector = true;
      smoothingFactor = 0.1f;
      drawDebugLines = false;
      drawSphereQueryResults = false;

      useSphereQuery = true;
      sphereRadiusMultiplier = 0.2f;
      sphereCrawlMode = true;
      userFaultyTriangles = true;

      freeRotationEnabled = true;
      freeRotationRatio = 6.0f;
      inputMultiplier = 2.0f;
      invertCameraInput = false;
      enableNormalCorrection = true;
      useAvgNormal = true;

      suppresForwardRotation = true;
      tmpSwitch = false;

      enableSmoothing = false;
      weightNormals = true;

      clipTranslationOnFreeRotate = true;
      walkSmoothedNormals = false;

      overrideNormal = false;
      useClosestPointForEdges = true;

      moveInNormalPlane = false;
      enableBackSnapping = false;
      enableForwardCheckForBackSnapping = false;

      forceCameraOrbit = false;

      enableCameraReset = true;
      enableCameraOrbit = true;

      preventCameraInMesh = false;
    }
  };

  // Init camera at the origin.
  CameraUtil();

  // Set camera from standard parameters: camera-from, camera-to, and up vector.
  void SetFromStandardCamera(const Vector3& from, const Vector3& to, lmReal referenceDistance);

  // Reset camera on the model. 
  // This finds the forwad-facing supporting vertex along the camera directino and relocates the reference point and camera translation.
  void ResetCamera(const Mesh* mesh, const Vector3& cameraDirection);

  // Checks if mesh vertices exist with in a query-sphere's radius from the reference point, and reinitializes camera otherwise.
  void EnsureReferencePointIsCloseToMesh(const Mesh* mesh, Params* paramsInOut);

  // Orbit camera around the mesh
  void OrbitCamera(const Mesh* mesh, lmReal deltaTime);

  struct IsoCameraState {
    lmReal refDist;
    lmReal cameraOffsetMultiplier;
    Vector3 refPosition;
    Vector3 refNormal;
    lmSurfacePoint closestPointOnMesh;
    //lmTransform refTransform;
    lmReal refPotential; // used when clipping to isosurface
    int numFailedUpdates;
  } isoState;

  lmReal IsoPotential(Mesh* mesh, const Vector3& position, lmReal queryRadius);

  void IsoPotential_row4( Mesh* mesh, const Vector3* positions, lmReal queryRadius, lmReal* potentials );

  Vector3 IsoNormal( Mesh* mesh, const Vector3& position, lmReal queryRadius);

  lmReal IsoQueryRadius(IsoCameraState* state) const;

  void IsoUpdateCameraTransform(const Vector3& newDirection, IsoCameraState* state, lmReal deltaTime);

  void IsoUpdateReferencePoint(IsoCameraState* state );

  void IsoPreventCameraInMesh(Mesh* mesh, IsoCameraState* state);

  void InitIsoCamera(Mesh* mesh, IsoCameraState* state);

  void IsoCamera(Mesh* mesh, IsoCameraState* state, const Vector3& movement, lmReal deltaTime);

  void IsoCameraConstrainWhenSpinning(Mesh* mesh, IsoCameraState* state);

  void IsoResetIfInsideManifoldMesh(Mesh* mesh, IsoCameraState* isoState);

  void IsoOnMeshUpdateStopped(Mesh* mesh, IsoCameraState* state);

  // Records user mouse input from mouse events.
  // 
  // Accumulates data for later processing in UpdateCamera.
  void RecordUserInput(const float _DTheta,const float _DPhi,const float _DFov);

  // Update camera position.
  void UpdateCamera(Mesh* mesh, Params* paramsInOut);

  lmTransform GetCameraInWorldSpace();

  void UpdateCameraInWorldSpace();

  // Get radius for mesh spehre queries.
  inline lmReal GetSphereQueryRadius() { return m_params.sphereRadiusMultiplier * m_referenceDistance; }

private:

  void UpdateMeshTransform(const Mesh* mesh, Params* paramsInOut );

  // Returns true if sphere collides mesh.
  bool CollideCameraSphere(Mesh* mesh, const Vector3& position, lmReal radius);

  bool VerifyCameraMovement(Mesh* mesh, const Vector3& from, const Vector3& to, lmReal radius);

  // Helper functions:
  void CastOneRay(const Mesh* mesh, const lmRay& ray, lmRayCastOutput* result);

  void CastOneRay(const Mesh* mesh, const lmRay& ray, std::vector<lmRayCastOutput>* results, bool collectall = false);

  void UpdateParamsToWalkSmoothedNormals(Params* paramsInOut);

  // Compute camera transform from standard camera vectors: from, to, & assumed up along y-axis.
  static void GetTransformFromStandardCamera(const Vector3& from, const Vector3& to, lmTransform& tOut);

  // Gets closest point to the refernce point, given that:
  // the triangle containing the cloest point has at least one of it's veritces withing the radius' distance from the reference point.
  void GetClosestPoint(const Mesh* mesh, const lmSurfacePoint& referencePoint, lmReal radius, const Vector3& cameraDirection, Geometry::GetClosestPointOutput* closestPointOut);

  lmSurfacePoint GetClosestSurfacePoint(Mesh* mesh, const Vector3& position, lmReal queryRadius);

  void FindPointsAheadOfMovement(const Mesh* mesh, const lmSurfacePoint& referencePoint, lmReal radius, const Vector3& movementDirection, std::vector<int>* vertices );

  // Performs a sphere query on the mesh, and returns average normal of the visible surface
  void GetAveragedSurfaceNormal(const Mesh* mesh, const lmSurfacePoint& referencePoint, lmReal radiusSquared, const Vector3& cameraDirection, bool weightNormals, lmSurfacePoint* avgSurfacePoint, lmSurfacePoint* pureAvgSurfacePoint, Geometry::GetClosestPointOutput* closestPointOut);

  // Helper functions.

  // todo: move to MathUtils ?
  static void GetBarycentricCoordinates(const Mesh* mesh, int triIdx, const Vector3& point, Vector3* coordsOut);

  static void GetNormalAtPoint(const Mesh* mesh, int triIdx, const Vector3& point, Vector3* normalOut);

  void GetSmoothedNormalAtPoint(const Mesh* mesh, int triIdx, const Vector3& point, lmReal radius, Vector3* normalOut);

  // Correct orientation.
  void CorrectCameraOrientation(lmReal dt, const Vector3& newNormal);

  // Correct distance from the mesh.
  void CorrectCameraDistance(lmReal dt);

  // Correct up vector
  void CorrectCameraUpVector(lmReal dt, const Vector3& up);

  void RealignRefPtAndCamera();

  void UpdateCameraOrientationFromPositions();

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

  // Most recent reference point on the model (used to compute camera's movement).
  lmSurfacePoint m_referencePoint;

  lmSurfacePoint m_avgVertex;

  int m_framesFromLastCollisions;

  // Point on the mesh closest to the reference point.
  Geometry::GetClosestPointOutput m_closestPoint;

  // Distance from the surface of the model.
  lmReal m_referenceDistance;

  // User input from the last call to CameraUpdate().
  Vector3 m_userInput;

  // Accumulated user input, waiting to be used over subsequent frames
  Vector3 m_accumulatedUserInput;

  // External debug draw util.
  DebugDrawUtil* m_debugDrawUtil;

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

  enum State {
    STATE_INVALID = -1,
    STATE_FREEFLOATING
  };

  // Current state of the camera.
  State state;

  std::mutex m_transformForGraphicsMutex;
  std::mutex m_userInputMutex;
  std::mutex m_referencePointMutex;

  std::vector<int> m_queryTriangles;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif // __CameraUtil_h__
