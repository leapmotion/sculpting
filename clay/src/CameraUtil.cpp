#include "StdAfx.h"
#include "CameraUtil.h"
#include "DebugDrawUtil.h"
#include "Geometry.h"
#include "DataTypes.h"
#include "ReplayUtil.h"

#if LM_PRODUCTION_BUILD
#define LM_LOG_CAMERA_LOGIC 0
#define LM_LOG_CAMERA_LOGIC_2 0
#define LM_LOG_CAMERA_LOGIC_3 0
#define LM_LOG_CAMERA_LOGIC_4 0
#define LM_LOG_CAMERA_LOGIC_5 0
#define LM_DRAW_DEBUG_OBJECTS 0
#else
#define LM_LOG_CAMERA_LOGIC 0
#define LM_LOG_CAMERA_LOGIC_2 0
#define LM_LOG_CAMERA_LOGIC_3 0
#define LM_LOG_CAMERA_LOGIC_4 0
#define LM_LOG_CAMERA_LOGIC_5 0
#define LM_DRAW_DEBUG_OBJECTS 1
#endif

CameraUtil::CameraUtil() {
  m_transform.setIdentity();
  m_meshTransform.setIdentity();
  m_userInput.setZero();
  m_accumulatedUserInput.setZero();
  m_lastCameraUpdateTime = -1.0f;
  m_framesFromLastCollisions = 1000;
  m_timeSinceOrbitingStarted = FLT_MAX;
  m_timeSinceOrbitingEnded = FLT_MAX;
  m_timeSinceCameraUpdateStarted = FLT_MAX;
  m_timeOfMovementSinceLastMeshMofification = FLT_MAX;
  m_timeOfLastScupt = 0.0f;
  m_prevTimeOfLastSculpt = 0.0f;
  m_justSculpted = false;
  m_forceVerifyPositionAfterSculpting = false;
  m_numFramesInsideManifoldMesh = 0;
  m_orbitRefPoint.setZero();
  m_orbitDistance = 0.0f;
}

void CameraUtil::ResetCamera( const Mesh* mesh, const Vector3& cameraDirection, bool keepCloseToMesh /*= false*/ )
{
#if LM_LOG_CAMERA_LOGIC_4
  std::cout << "CAMERA RESET!" << std::endl;
#endif

  const VertexVector& vertices = mesh->getVertices();
  if (vertices.empty()) {
    return;
  }
  LM_ASSERT(vertices.size(), "Mesh has no vertices.");
  LM_ASSERT(lmIsNormalized(cameraDirection), "Camera direction is not normalized.");

  // Iterate through all vertices.
  int idx = -1;
  lmReal minPosDot = FLT_MAX;
  for (unsigned vi = 0; vi < vertices.size(); vi++) {
    const Vertex& vert = vertices[vi];
    lmReal normalDot = vert.normal_.dot(cameraDirection);
    lmReal posDot = vert.dot(cameraDirection);
    if ((idx < 0 || normalDot < 0.0f) && posDot < minPosDot) {
      idx = vi;
      minPosDot = posDot;
    }
  }

  // Set reference point to the selected vertex
  const Vertex& closestVertex = vertices[idx];

  // Rotate camera to the new requested direction
  Vector3 currentNegZ = -1.0f * (m_transform.rotation * Vector3::UnitZ());
  lmQuat correction; correction.setFromTwoVectors(currentNegZ, cameraDirection);

  // Set camera position
  isoState.refNormal = closestVertex.normal_;
  isoState.closestPointOnMesh = lmSurfacePoint(closestVertex, closestVertex.normal_);

  // Hack back the camera prositino after reseting
  isoState.refDist = keepCloseToMesh ? m_params.minDist : (GetMaxDistanceForMesh(mesh)/(1+m_params.isoRefDistMultiplier)*1.0f);
  isoState.refPosition = closestVertex + isoState.refDist * closestVertex.normal_;

  m_transform.rotation = correction * m_transform.rotation;
  m_transform.translation = isoState.refPosition + isoState.refDist * m_params.isoRefDistMultiplier * (m_transform.rotation * Vector3::UnitZ());

  UpdateCameraInWorldSpace();
}

void CameraUtil::GetTransformFromStandardCamera(const Vector3& from, const Vector3& to, lmTransform& tOut) {
  Vector3 dir = to - from;
  Vector3 flat = dir; flat.y() = 0.0f;

  // for orientation: combine rotation around y (vertical) and then adjust pitch
  Vector3 negZ(0.0f, 0.0f, -1.0f);
  lmQuat q1; q1.setFromTwoVectors(negZ, flat); q1.normalize();
  Vector3 pitchV = q1.inverse() * dir;
  lmQuat q2; q2.setFromTwoVectors(negZ, pitchV); q2.normalize();

  tOut.translation = from;
  tOut.rotation = (q1 * q2).normalized();
}

//inline static lmReal TriArea(const Mesh* mesh, const Triangle& tri) {
//  const Vertex& v0 = mesh->getVertex(tri.vIndices_[0]);
//  const Vertex& v1 = mesh->getVertex(tri.vIndices_[1]);
//  const Vertex& v2 = mesh->getVertex(tri.vIndices_[2]);
//
//  const lmReal area = 0.5f * (v1-v0).cross(v2-v0).norm();
//  return area;
//}

inline static Vector3 TriCenter(const Mesh* mesh, const Triangle& tri) {
  const Vertex& v0 = mesh->getVertex(tri.vIndices_[0]);
  const Vertex& v1 = mesh->getVertex(tri.vIndices_[1]);
  const Vertex& v2 = mesh->getVertex(tri.vIndices_[2]);

  Vector3 center = (v0 + v1 + v2) / 3.0f;
  return center;
}

lmSurfacePoint CameraUtil::GetClosestSurfacePoint(Mesh* mesh, const Vector3& position, lmReal queryRadius) {
  std::vector<Octree*> leavesHit;
  m_queryTriangles.clear();
  mesh->getOctree()->intersectSphere(position, queryRadius*queryRadius, leavesHit, m_queryTriangles);

  // Get triangles
  Geometry::GetClosestPointOutput closestPoint;
  closestPoint.distanceSqr = FLT_MAX;

  for (size_t ti = 0; ti < m_queryTriangles.size(); ti++) {
    const Triangle& tri = mesh->getTriangle(m_queryTriangles[ti]);

    Geometry::GetClosestPointOutput output;
    Geometry::GetClosestPointInput input(mesh, &tri, position);
    Geometry::getClosestPoint(input, &output);
    output.triIdx = m_queryTriangles[ti];
    if (output.distanceSqr < closestPoint.distanceSqr) {
      closestPoint = output;
    }
  }

  if (closestPoint.distanceSqr < FLT_MAX) {
    GetNormalAtPoint(mesh, closestPoint.triIdx, closestPoint.position, &closestPoint.normal);
  }

  return lmSurfacePoint(closestPoint.position, closestPoint.normal);
}

void CameraUtil::RecordUserInput(const float _DTheta,const float _DPhi,const float _DFov) {
  std::unique_lock<std::mutex> lock(m_userInputMutex);
  Vector3 movement(50.0f * _DTheta, -50.0f * _DPhi, -_DFov / 100.0f);
  m_userInput += (m_params.invertCameraInput?-1.0f:1.0f) * m_params.inputMultiplier * movement;
}

void CameraUtil::GetBarycentricCoordinates(const Mesh* mesh, int triIdx, const Vector3& point, Vector3* coordsOut) {
  const Triangle& tri = mesh->getTriangle(triIdx);
  Vector3 pointToVertex[3];
  lmReal areas[3];
  for (int i = 0; i < 3; i++) {
    pointToVertex[i] = mesh->getVertex(tri.vIndices_[i]) - point;
  }

  lmReal area = 0.0f;
  for (int i = 0; i < 3; i++) {
    areas[i] = pointToVertex[i].cross(pointToVertex[(i+1)%3]).dot(tri.normal_);
    area += areas[i];
  }

  (*coordsOut)[0] = areas[1] / area;
  (*coordsOut)[1] = areas[2] / area;
  (*coordsOut)[2] = 1 - (*coordsOut)[0] - (*coordsOut)[1];
}

void CameraUtil::GetNormalAtPoint(const Mesh* mesh, int triIdx, const Vector3& point, Vector3* normalOut) {
  Vector3 coords;
  GetBarycentricCoordinates(mesh, triIdx, point, &coords);
  const Triangle& tri = mesh->getTriangle(triIdx);
  const Vertex& v1 = mesh->getVertex(tri.vIndices_[0]);
  const Vertex& v2 = mesh->getVertex(tri.vIndices_[1]);
  const Vertex& v3 = mesh->getVertex(tri.vIndices_[2]);

  Vector3 normal = Vector3::Zero();
  normal += coords[0] * v1.normal_;
  normal += coords[1] * v2.normal_;
  normal += coords[2] * v3.normal_;

  *normalOut = normal.normalized();
}

// Used to collide the camera sphere. Figure out what radius we need ?? maybe same as refence sphere
// Returns true if the sphere collides with the mesh
bool CameraUtil::CollideCameraSphere(Mesh* mesh, const Vector3& position, lmReal radius )
{
  // Get potential triangles from the aabb octree.
  std::vector<Octree*> leavesHit;
  m_queryTriangles.clear();
  mesh->getOctree()->intersectSphere(position,radius*radius,leavesHit,m_queryTriangles);

  // Collide each potential triangle; find real collisions.
  std::vector<Geometry::GetClosestPointOutput> collidingTriangles;
  for (unsigned ti = 0; ti < m_queryTriangles.size(); ti++) {
    int triIdx = m_queryTriangles[ti];
    const Triangle& tri = mesh->getTriangle(triIdx);

    Geometry::GetClosestPointInput input(mesh, &tri, position);
    Geometry::GetClosestPointOutput output;
    Geometry::getClosestPoint_noNormal(input, &output);

    if (output.distanceSqr < radius*radius) {
      collidingTriangles.push_back(output);
      // Cutting things short -- only return if collision happened
      break;
    }
  }

  return collidingTriangles.size() > 0;
}

static lmTransform lmTransformFromMatrix(const Matrix4x4& _r, const Vector3& t) {
  lmTransform result;
  result.translation = t;

  Matrix3x3 r;
  for (int i = 0; i < 9; i++) { r(i%3, i/3) = _r(i%3, i/3); }

  Vector3 rotX = r * Vector3::UnitX();
  lmQuat q0; q0.setFromTwoVectors(Vector3::UnitX(), rotX); q0.normalize();
  Vector3 rotY = r * Vector3::UnitY();
  Vector3 halfWayY = q0 * Vector3::UnitY();
  lmQuat q1; q1.setFromTwoVectors(halfWayY, rotY);
  result.rotation = q1 * q0;
  result.rotation.normalize();

  return result;
}

Vector3 CameraUtil::ToMeshSpace(const Vector3& v) { return m_meshTransform.rotation.inverse() * v; }

Vector3 CameraUtil::ToWorldSpace(const Vector3& v) { return m_meshTransform.rotation * v; }

Vector3 CameraUtil::GetCameraDirection() const { return -1.0f * (m_transform.rotation * Vector3::UnitZ()); };

void CameraUtil::UpdateMeshTransform(const Mesh* mesh, Params* paramsInOut ) {
  m_transform.rotation = m_meshTransform.rotation * m_transform.rotation;
  m_transform.translation = ToWorldSpace(m_transform.translation);
  m_orbitRefPoint.position = ToWorldSpace(m_orbitRefPoint.position);
  m_orbitRefPoint.normal = ToWorldSpace(m_orbitRefPoint.normal);

  isoState.refPosition = ToWorldSpace(isoState.refPosition);
  isoState.refNormal = ToWorldSpace(isoState.refNormal);
  isoState.closestPointOnMesh.position = ToWorldSpace(isoState.closestPointOnMesh.position);
  isoState.closestPointOnMesh.normal = ToWorldSpace(isoState.closestPointOnMesh.normal);

  // Get mesh's transformstion
  m_meshTransform = lmTransformFromMatrix(mesh->getRotationMatrix(), mesh->getTranslation());

  m_transform.rotation = m_meshTransform.rotation.inverse() * m_transform.rotation;
  m_transform.translation = ToMeshSpace(m_transform.translation);
  m_orbitRefPoint.position = ToMeshSpace(m_orbitRefPoint.position);
  m_orbitRefPoint.normal = ToMeshSpace(m_orbitRefPoint.normal);

  isoState.refPosition = ToMeshSpace(isoState.refPosition);
  isoState.refNormal = ToMeshSpace(isoState.refNormal);
  isoState.closestPointOnMesh.position = ToMeshSpace(isoState.closestPointOnMesh.position);
  isoState.closestPointOnMesh.normal = ToMeshSpace(isoState.closestPointOnMesh.normal);
}

static Vector3 lmProjectAlongVec(const Vector3& in, const Vector3& projectionDirection) {
  return in - projectionDirection * in.dot(projectionDirection);
}

void CameraUtil::OrbitCamera( const Mesh* mesh, lmReal deltaTime ) {

  // Horizontal rotate
  static const lmReal ORBIT_RATE = LM_2PI / 40.0f;
  lmSurfacePoint referencePointNew = m_orbitRefPoint;
  {
    lmQuat q(AngleAxis( - ORBIT_RATE * deltaTime, Vector3::UnitY()));
    m_transform.mul(q);
    referencePointNew.mul(q);
  }

  // Orbit camera with raycast

  // Camera position in horizontal plane
  Vector3 camXZ = lmProjectAlongVec(m_transform.translation, Vector3::UnitY());
  // Rotation around Y
  lmQuat q0 = lmQuat::Identity();
  if (!lmIsZero(camXZ)) {
    q0.setFromTwoVectors(Vector3::UnitZ(), camXZ.normalized()); q0.normalize();
  }

  // Oscillating vertical rotation
  static lmReal phase = 0.0f;
  const static lmReal OSCILLATION_RATE = 1.0f / 20.0f;
  phase += LM_2PI * OSCILLATION_RATE * deltaTime;
  lmQuat q1(AngleAxis(std::cos(phase) * 30.0f * LM_DEG, Vector3::UnitX()));

  // Desired camera direction
  lmQuat q = q0 * q1; q.normalize();

  lmReal refPtDistFromOrigin = referencePointNew.position.norm();
  // Do raycast
  if (mesh) {
    const lmReal aabbDiagonal = mesh->getOctree()->getAabbSplit().getDiagonalLength();
    Vector3 rayStart = (q * Vector3::UnitZ()) * aabbDiagonal;
    lmRayCastOutput raycastHit;
    CastOneRay(mesh, lmRay(rayStart, Vector3::Zero()), &raycastHit);

    // if raycast hit (move last reference point, remember last point)
    if (raycastHit.isSuccess()) {
      lmReal distFromOrigin = raycastHit.position.norm();
      if (distFromOrigin + m_params.minDist > refPtDistFromOrigin + m_orbitDistance ) {
        m_orbitDistance = distFromOrigin + m_params.minDist - refPtDistFromOrigin;
      }
    }
  }

  m_orbitRefPoint.position = (q * Vector3::UnitZ()) * refPtDistFromOrigin;
  m_orbitRefPoint.normal = q * Vector3::UnitZ();

  // Fix camera direction & position
  static const lmReal ORBITING_BLEND_IN_TIME = 5.0f;
  const lmReal blendingFactor = lmClip(m_timeSinceOrbitingStarted / ORBITING_BLEND_IN_TIME, 0.0f, 1.0f);

  //transform.rotation = q;
  m_transform.rotation = m_transform.rotation.slerp(blendingFactor*deltaTime, q);
  Vector3 targetTranslation = (m_transform.rotation * Vector3::UnitZ()) * (refPtDistFromOrigin + m_orbitDistance);
  m_transform.translation = lmInterpolate(blendingFactor*deltaTime, m_transform.translation, targetTranslation);
}

void CameraUtil::UpdateCamera( Mesh* mesh, Params* paramsInOut) {

  if (mesh->getVertices().empty())
  {
    // mesh has no vertices. do nothing.
    return; 
  }
  LM_TRACK_VALUE(m_transformInWorldSpaceForGraphics);
  LM_TRACK_VALUE(isoState);

  LM_ASSERT(mesh, "Can't upate the camera without a mesh");
  UpdateMeshTransform(mesh, paramsInOut);

  // Check time
  lmReal dt = 0.0f;
  {
    lmReal prevTime = m_lastCameraUpdateTime;
    lmReal time = lmReal(ci::app::getElapsedSeconds());
    LM_TRACK_VALUE(time);
    LM_ASSERT_IDENTICAL(prevTime);
    m_lastCameraUpdateTime = time;
    if (prevTime < 0.0f) { prevTime = time; }
    dt = time - prevTime;
  }

  this->m_params = *paramsInOut;

  static const Mesh* prevMesh = NULL;
  if (mesh != prevMesh) {
    // Init Iso Camera
    ResetCamera(mesh, -(Vector3::UnitZ() + -0.3f * Vector3::UnitX() + 0.2f * Vector3::UnitY()).normalized());
    InitIsoCamera(mesh, &isoState);
    m_timeSinceCameraUpdateStarted = FLT_MAX;
  }
  prevMesh = mesh;

  m_framesFromLastCollisions++;
  LM_TRACK_VALUE(m_framesFromLastCollisions);
  LM_TRACK_VALUE(m_timeSinceOrbitingStarted);
  LM_TRACK_VALUE(m_timeSinceOrbitingEnded);
  LM_TRACK_VALUE(m_timeOfMovementSinceLastMeshMofification);
  LM_TRACK_VALUE(m_timeOfLastScupt);

  if (m_params.forceCameraOrbit && m_params.enableCameraOrbit) {
    if (m_timeSinceOrbitingStarted == 0.0f) {
      m_orbitRefPoint = GetReferencePoint();
      m_orbitDistance = GetReferenceDistance();
    }

    OrbitCamera(mesh, dt);
    m_timeSinceOrbitingStarted += dt;
    m_timeSinceOrbitingEnded = 0.0f;
    m_timeOfMovementSinceLastMeshMofification = FLT_MAX;
    UpdateCameraInWorldSpace();
    LM_TRACK_VALUE(m_transformInWorldSpaceForGraphics);
    LM_TRACK_VALUE(isoState);
    return;
  }

  m_timeSinceOrbitingStarted = 0.0f;
  m_timeSinceOrbitingEnded += dt;

  if (m_params.drawDebugLines) {
    LM_DRAW_ARROW(Vector3::Zero(), 50.0f * ToMeshSpace(Vector3::UnitX()), lmColor::RED );
    LM_DRAW_ARROW(Vector3::Zero(), 50.0f * ToMeshSpace(Vector3::UnitY()), lmColor::GREEN );
    LM_DRAW_ARROW(Vector3::Zero(), 50.0f * ToMeshSpace(Vector3::UnitZ()), lmColor::BLUE );
    LM_DRAW_ARROW(m_transform.translation, isoState.refPosition, lmColor::YELLOW );
  }
  DebugDrawUtil::getInstance().SwitchBuffers();

  lmReal dtOne = 1.0f;

  LM_TRACK_VALUE(m_userInput);
  Vector3 usedUserInput = m_userInput;
  // Multiply motion by distance:
  {
    std::unique_lock<std::mutex> lock(m_userInputMutex);
    // Accumulate userInput in 2d
    // If smoothing is enabled, only take a small slice of the accumulated user input
    // and use it for camera movement per frame.
    if (m_params.enableSmoothing) {
        m_accumulatedUserInput += m_userInput;
        lmReal useInputFraction = pow(m_params.inputSmoothingPerSecond, dt);
        usedUserInput = (1.0f - useInputFraction) * m_accumulatedUserInput;
        m_accumulatedUserInput -= usedUserInput;
    }
    m_userInput.setZero();

    if (100000000.0f <= usedUserInput.squaredNorm()) {
      // This actually happened when stopping the Leap Service, and starting a different version.
      m_accumulatedUserInput.setZero();
      usedUserInput.setZero();
    }
  }

  // Process smoothed user input
  const lmReal distFraction = (GetReferenceDistance()-m_params.minDist)/(m_params.maxDist/*intentionaly static value and not GetMaxDistFromMesh()*/-m_params.minDist);
  const lmReal movementRatio = m_params.speedAtMinDist + distFraction * (m_params.speedAtMaxDist-m_params.speedAtMinDist);
  Vector3 movementInCamSpace = - usedUserInput * movementRatio;

  isoState.cameraOffsetMultiplier = m_params.isoRefDistMultiplier;
  IsoCamera(mesh, &isoState, movementInCamSpace, dt);
  if (m_params.pinUpVector) { CorrectCameraUpVector(dtOne, Vector3::UnitY()); }
  UpdateCameraInWorldSpace();
  m_prevTimeOfLastSculpt = m_timeOfLastScupt;

  LM_TRACK_VALUE(m_transformInWorldSpaceForGraphics);
  LM_TRACK_VALUE(isoState);

  LM_ASSERT_IDENTICAL(GetReferenceDistance());
}

void CameraUtil::CorrectCameraUpVector(lmReal dt, const Vector3& up) {

  //std::unique_lock<std::mutex> lock(mutex);

  // Camera's xy-plane normal
  Vector3 xyPlaneNormal = m_transform.rotation * Vector3::UnitZ();

  // Take the up vector, and project it onto camera's xy-plane
  const lmReal xyPlaneNormalDotUp = xyPlaneNormal.dot(up);
  Vector3 newUp = up - xyPlaneNormal * xyPlaneNormalDotUp; newUp.normalize();

  // Disable correction, if camera is facing vertically or horizontally. Hard limit.
  if (std::fabs(xyPlaneNormalDotUp) < 0.9) {

    // Calc correction rotation and rate.
    Vector3 oldUp = m_transform.rotation * Vector3::UnitY();
    lmQuat dQ; dQ.setFromTwoVectors(oldUp, newUp); dQ.normalize();
    lmReal adjustOrientationRate = std::pow(0.97f, dt);

    // Apply correction.
    dQ = dQ.slerp(adjustOrientationRate, lmQuat::Identity());
    m_transform.rotation = dQ * m_transform.rotation;
  }
}

lmTransform CameraUtil::GetCameraInWorldSpace()
{
  std::unique_lock<std::mutex> lock(m_transformForGraphicsMutex);
  lmTransform copy = m_transformInWorldSpaceForGraphics;
  return copy;
}

void CameraUtil::UpdateCameraInWorldSpace() {
  lmTransform t;
  t.translation = ToWorldSpace(m_transform.translation);
  t.rotation = m_meshTransform.rotation * m_transform.rotation;
#if LM_LOG_CAMERA_LOGIC_4
  //std::cout <<
  //  "meshQ: " << meshTransform.rotation.x() <<
  //  ", " << meshTransform.rotation.y() <<
  //  ", " << meshTransform.rotation.z() <<
  //  ", " << meshTransform.rotation.w() <<
  //  ", camQ: " << t.rotation.x() <<
  //  ", " << t.rotation.y() <<
  //  ", " << t.rotation.z() <<
  //  ", " << t.rotation.w() <<
  //  ", camT: " << t.translation.x() <<
  //  ", " << t.translation.y() <<
  //  ", " << t.translation.z() << std::endl;
#endif
  std::unique_lock<std::mutex> lock(m_transformForGraphicsMutex);
  m_transformInWorldSpaceForGraphics = t;
}

void CameraUtil::CastOneRay( const Mesh* mesh, const lmRay& ray, lmRayCastOutput* result )
{
  std::vector<lmRayCastOutput> results;
  const bool dontCollectAll = false;
  CastOneRay(mesh, ray, &results, dontCollectAll);
  if (results.size()) {
    *result = results[0];
  } else {
    lmRayCastOutput invalidResult;
    *result = invalidResult;
  }
}

void CameraUtil::CastOneRay( const Mesh* mesh, const lmRay& ray, std::vector<lmRayCastOutput>* results, bool collectall /*= false*/ )
{

  m_queryTriangles.clear();
  mesh->getOctree()->intersectRay(ray.start, ray.GetDirection(), m_queryTriangles);
  lmReal minDist = FLT_MAX;

  lmRayCastOutput rayCastOutput;

  const Vector3 rayDirection = ray.GetDirection();
  const lmReal rayLength = ray.GetLength();


  // Cast ray for each triangle's aabb
  for (size_t ti = 0; ti < m_queryTriangles.size(); ti++) {
    const Triangle& tri = mesh->getTriangle(m_queryTriangles[ti]);
    Vector3 hitPoint;

    bool rayHit = tri.aabb_.intersectRay(ray.start, rayDirection);
    if (rayHit) {
      rayHit = Geometry::intersectionRayTriangle(
        ray.start, ray.end,
        mesh->getVertex(tri.vIndices_[0]),
        mesh->getVertex(tri.vIndices_[1]),
        mesh->getVertex(tri.vIndices_[2]),
        tri.normal_, hitPoint);
    }
    if (rayHit) {
      lmReal dist = (hitPoint-ray.start).dot(rayDirection);
      if ((0 <= dist && dist < minDist) || collectall) {
        minDist = dist;

        rayCastOutput.triangleIdx = m_queryTriangles[ti];
        rayCastOutput.position = hitPoint;
        rayCastOutput.normal = tri.normal_;
        rayCastOutput.dist = dist;
        rayCastOutput.fraction = dist / rayLength;

        if (collectall && lmInRange(rayCastOutput.fraction, 0.0f, 1.0f)) {
          results->push_back(rayCastOutput);
        }
      }
    }
  }

  if (!collectall && minDist < FLT_MAX && lmInRange(rayCastOutput.fraction, 0.0f, 1.0f)) {
    results->push_back(rayCastOutput);
  }
}

bool CameraUtil::VerifyCameraMovement( Mesh* mesh, const Vector3& from, const Vector3& to, lmReal radius )
{
  bool validMovement = false;

  bool cameraCollidesMesh = radius > 0.0f ? CollideCameraSphere(mesh, to, radius) : false;

  if (!cameraCollidesMesh) {

    // Cast a ray between old & new points -- if it crosses the mesh, prevent the movement
    lmRayCastOutput rayCastOutput;
    CastOneRay(mesh, lmRay(from, to), &rayCastOutput);

    if (!rayCastOutput.isSuccess()) {
      validMovement = true;
    }
  }

  return validMovement;
}

lmReal CameraUtil::IsoPotential( Mesh* mesh, const Vector3& position, lmReal queryRadius )
{
  std::vector<Octree*> &leavesHit = mesh->getLeavesUpdate();
  m_queryTriangles.clear();
  mesh->getOctree()->intersectSphere(position,queryRadius*queryRadius,leavesHit, m_queryTriangles);

  //lmReal radius = Get
  lmReal potential = 0.0;
  // process every n-th point
  const int striding = 1;
  const lmReal queryRadiusSqr = queryRadius*queryRadius;

  if (m_params.queryTriangles) {
    const TriangleVector& triangles = mesh->getTriangles();

    for (unsigned ti = 0; ti < m_queryTriangles.size(); ti+= striding)
    {
      const Triangle& tri = triangles[m_queryTriangles[ti]];

      Geometry::GetClosestPointInput input(mesh, &tri, position);
      Geometry::GetClosestPointOutput output;
      Geometry::getClosestPoint_noNormal(input, &output);

      lmReal distSqr = output.distanceSqr;

      if (distSqr < queryRadiusSqr)
      {
        distSqr += m_params.grav_k*m_params.grav_k;

        // avoid calling std::pow
        const lmReal distPowered = distSqr * distSqr * distSqr;
        const lmReal weightDenominator = queryRadiusSqr * queryRadiusSqr * queryRadiusSqr;

        lmReal weight = 1.0f - (distPowered / weightDenominator);
        weight = std::max(0.0f, weight);
        potential += tri.area * weight / distPowered;

        if (m_params.drawDebugLines && m_params.drawSphereQueryResults) {
          LM_DRAW_MESH_TRIANGLE(mesh, tri, lmColor::WHITE);
        }
      }
    }

  } else {

    std::vector<int> selectedVertices;
    mesh->getVerticesFromTriangles(m_queryTriangles, selectedVertices);

    const VertexVector& vertices = mesh->getVertices();

    for (unsigned vi = 0; vi < selectedVertices.size(); vi+= striding)
    {
      const Vertex& vert = vertices[selectedVertices[vi]];

      lmReal distSqr = (position-vert).squaredNorm();

      distSqr += m_params.grav_k*m_params.grav_k;

      if (distSqr < queryRadiusSqr)
      {
        const lmReal distPowered = distSqr * distSqr * distSqr;
        const lmReal weightDenominator = queryRadiusSqr * queryRadiusSqr * queryRadiusSqr;

        lmReal weight = 1.0f - (distPowered / weightDenominator);
        weight = std::max(0.0f, weight);

        potential += weight / distPowered;

        if (m_params.drawSphereQueryResults) {
          LM_DRAW_CROSS(vert, 3.0f, lmColor::WHITE);
        }
      }
    }
  }

  return potential;
}

void CameraUtil::IsoPotential_row4( Mesh* mesh, const Vector3* positions, lmReal queryRadius, lmReal* potentials )
{
  std::vector<Octree*> &leavesHit = mesh->getLeavesUpdate();
  m_queryTriangles.clear();
  mesh->getOctree()->intersectSphere(positions[0],queryRadius*queryRadius,leavesHit, m_queryTriangles);

  for (int i = 0; i < 4; i++) { potentials[i] = 0.0f; }

  // process every n-th point
  static const int striding = 1;
  const lmReal queryRadiusSqr = queryRadius*queryRadius;

  if (m_params.queryTriangles) {
    const TriangleVector& triangles = mesh->getTriangles();

    for (unsigned ti = 0; ti < m_queryTriangles.size(); ti+= striding)
    {
      if (ti < m_queryTriangles.size()-striding) {
        const Triangle& nextTri = triangles[m_queryTriangles[ti+striding]];
        LM_MEM_PREFETCH(&mesh->getVertices()[0]+nextTri.vIndices_[0]);
        LM_MEM_PREFETCH(&mesh->getVertices()[0]+nextTri.vIndices_[1]);
        LM_MEM_PREFETCH(&mesh->getVertices()[0]+nextTri.vIndices_[2]);
      }

      const Triangle& tri = triangles[m_queryTriangles[ti]];

      Geometry::GetClosestPointInput input(mesh, &tri, positions[0]);
      Geometry::GetClosestPointOutput output;
      Geometry::getClosestPoint_noNormal(input, &output);

      lmReal distSqr = output.distanceSqr;

      if (distSqr < queryRadiusSqr)
      {
        distSqr += m_params.grav_k*m_params.grav_k;

        // avoid calling std::pow
        const lmReal distPowered = distSqr * distSqr * distSqr;
        const lmReal weightDenominator = queryRadiusSqr * queryRadiusSqr * queryRadiusSqr;

        lmReal weight = 1.0f - (distPowered / weightDenominator);
        weight = std::max(0.0f, weight);
        potentials[0] += tri.area * weight / distPowered;

        if (m_params.drawDebugLines && m_params.drawSphereQueryResults) {
          LM_DRAW_MESH_TRIANGLE(mesh, tri, lmColor::WHITE);
        }

        for (int i = 1; i < 4; i++) {
          lmReal distSqr =  (positions[i]-output.position).squaredNorm();
          distSqr += m_params.grav_k*m_params.grav_k;

          // avoid calling std::pow
          const lmReal distPowered = distSqr * distSqr * distSqr;
          const lmReal weightDenominator = queryRadiusSqr * queryRadiusSqr * queryRadiusSqr;

          lmReal weight = 1.0f - (distPowered / weightDenominator);
          weight = std::max(0.0f, weight);
          potentials[i] += tri.area * weight / distPowered;
        }
      }
    }
  } else {

    std::vector<int> selectedVertices;
    mesh->getVerticesFromTriangles(m_queryTriangles, selectedVertices);

    const VertexVector& vertices = mesh->getVertices();

    for (unsigned vi = 0; vi < selectedVertices.size(); vi+= striding)
    {
      const Vertex& vert = vertices[selectedVertices[vi]];

      lmReal distSqr = (positions[0]-vert).squaredNorm();
      distSqr += m_params.grav_k*m_params.grav_k;

      if (distSqr < queryRadiusSqr)
      {
        const lmReal distPowered = distSqr * distSqr * distSqr;
        const lmReal weightDenominator = queryRadiusSqr * queryRadiusSqr * queryRadiusSqr;

        lmReal weight = 1.0f - (distPowered / weightDenominator);
        weight = std::max(0.0f, weight);
        potentials[0] += weight / distPowered;

        if (m_params.drawSphereQueryResults) {
          LM_DRAW_CROSS(vert, 3.0f, lmColor::WHITE);
        }

        for(int i = 1; i < 4; i++) {
          lmReal distSqr = (positions[i]-vert).squaredNorm();
          distSqr += m_params.grav_k*m_params.grav_k;

          const lmReal distPowered = distSqr * distSqr * distSqr;
          const lmReal weightDenominator = queryRadiusSqr * queryRadiusSqr * queryRadiusSqr;

          lmReal weight = 1.0f - (distPowered / weightDenominator);
          weight = std::max(0.0f, weight);
          potentials[i] += weight / distPowered;
        }
      }
    }
  }
}

Vector3 CameraUtil::IsoNormal( Mesh* mesh, const Vector3& position, lmReal queryRadius, lmReal* potentialOut /*= NULL*/, lmReal* gradientMagOut /*= NULL*/ )
{
  lmReal epsilon = 0.1f;
  const Vector3& pos = position;
  Vector3 posX = pos; posX.x() += epsilon;
  Vector3 posY = pos; posY.y() += epsilon;
  Vector3 posZ = pos; posZ.z() += epsilon;
  //lmReal potential = IsoPotential(mesh, pos, queryRadius);
  //lmReal dPotentialX = IsoPotential(mesh, posX, queryRadius) - potential;
  //lmReal dPotentialY = IsoPotential(mesh, posY, queryRadius) - potential;
  //lmReal dPotentialZ = IsoPotential(mesh, posZ, queryRadius) - potential;

  Vector3 positions[4] = {pos, posX, posY, posZ };
  lmReal potentials[4];
  IsoPotential_row4(mesh, positions, queryRadius+4*epsilon, potentials);
  lmReal potential = potentials[0];
  lmReal dPotentialX = potentials[1] - potential;
  lmReal dPotentialY = potentials[2] - potential;
  lmReal dPotentialZ = potentials[3] - potential;


  Vector3 negNormal(dPotentialX, dPotentialY, dPotentialZ);
  lmReal gradientMag = negNormal.norm();
  negNormal /= gradientMag;
  gradientMag /= epsilon;

  //std::cout << "Iso potential: " << potential << std::endl;

  Vector3 normal((float)-negNormal.x(), (float)-negNormal.y(), (float)-negNormal.z());

  if (potentialOut) { *potentialOut = potential; }
  if (gradientMagOut) { *gradientMagOut = gradientMag; }

  return normal;
}

lmReal CameraUtil::IsoQueryRadius( const Mesh* mesh, IsoCameraState* state ) const
{
  const lmReal multiplier = 0.5f * GetMeshSize(mesh) / Mesh::globalScale_;
  return std::max(state->refDist + m_params.isoQueryPaddingRadius * multiplier, m_params.isoQueryPaddingRadius * multiplier);
}

void CameraUtil::IsoUpdateCameraTransform( const Vector3& newDirection, IsoCameraState* state, lmReal deltaTime )
{
  lmQuat qCorrection; qCorrection.setFromTwoVectors(GetCameraDirection(), newDirection);

  lmTransform newTransform;
  newTransform.rotation = qCorrection * m_transform.rotation;
  newTransform.translation = state->refPosition - state->cameraOffsetMultiplier * state->refDist * newDirection;

  static const lmReal BLEND_IN_TIME_AFTER_ORBITING = 5.0f;
  lmReal blendingFactor = lmClip(m_timeSinceOrbitingEnded/BLEND_IN_TIME_AFTER_ORBITING, 0.0f, 1.0f);
  blendingFactor *= blendingFactor;

  if (m_timeSinceOrbitingEnded < BLEND_IN_TIME_AFTER_ORBITING * 1.5f) {
    // just use orbit blending
  } else {
    // use sculpting blending
    static const lmReal BLEND_IN_TIME_AFTER_CAMERA_IDLE = 1.0f;
    lmReal blendingFactor2 = lmClip(m_timeOfMovementSinceLastMeshMofification/BLEND_IN_TIME_AFTER_CAMERA_IDLE, 0.0f, 1.0f);
    blendingFactor2 *= blendingFactor2;

    blendingFactor = blendingFactor2;
  }

  m_transform.rotation = m_transform.rotation.slerp(blendingFactor, newTransform.rotation);
  m_transform.translation = lmInterpolate(blendingFactor, m_transform.translation, newTransform.translation);
}

void CameraUtil::IsoPreventCameraInMesh( Mesh* mesh, IsoCameraState* state )
{
  // Raycast from refPoint to camera.
  // reset camera to first hit found.
  lmRayCastOutput raycast;
  lmRay ray(state->refPosition, m_transform.translation);
  CastOneRay(mesh, ray, &raycast);
  if (raycast.isSuccess()) {
#if LM_LOG_CAMERA_LOGIC_4
    std::cout << "rayfraction " << raycast.fraction;
    std::cout << " raydist " << raycast.dist;
    std::cout << " prevdist " << (state->refPosition, m_transform.translation).norm() << std::endl;
    std::cout << "Hugging camera to refPoint (mesh collision)" << std::endl;
#endif
    LM_ASSERT(lmInRange(raycast.fraction, 0.0f, 1.0f), "Inavlid raycast result returned.");
    LM_ASSERT(raycast.dist <= ray.GetLength(), "Raycast result corrupted.");

    // if clip this
    lmReal refPositionFraction = std::max(raycast.fraction, m_params.minDist / state->refDist);
    refPositionFraction = lmClip(refPositionFraction, 0.0f, 1.0f);
    state->refPosition -= state->refNormal * state->refDist * (1.0f-refPositionFraction);
    state->refDist *= refPositionFraction;

    m_transform.translation = raycast.position - ray.GetDirection() * 0.01f;
  }
}

void CameraUtil::InitIsoCamera( Mesh* mesh, IsoCameraState* state )
{
  // get field potential from current position
  lmReal t = 0.5f;
  //state->refPosition = lmInterpolate(t, m_referencePoint.position, m_transform.translation);
  //state->refNormal
  //state->refPotential = IsoPotential(mesh, state->refPosition);
  state->cameraOffsetMultiplier = 1.0f;

  lmReal queryRadius = 10.0f * GetMeshSize(mesh); // everything
  state->closestPointOnMesh = GetClosestSurfacePoint(mesh, state->refPosition, queryRadius);
  state->refDist = (state->closestPointOnMesh.position - state->refPosition).norm();

  state->refNormal = IsoNormal(mesh, state->refPosition, IsoQueryRadius(mesh, state), &state->refPotential, &state->currGradientMag);
  //state->refPotential = IsoPotential(mesh, state->refPosition, IsoQueryRadius(mesh, state));

  state->numFailedUpdates = 0;
  // Remember closest point distance
}

void CameraUtil::IsoCamera( Mesh* mesh, IsoCameraState* state, const Vector3& movement, lmReal deltaTime )
{
  m_justSculpted = (m_prevTimeOfLastSculpt != m_timeOfLastScupt);
  if (m_justSculpted) {
    m_timeOfMovementSinceLastMeshMofification = 0.0f;
    m_forceVerifyPositionAfterSculpting = true;
  }
  LM_TRACK_VALUE(m_justSculpted);
  LM_TRACK_VALUE(m_timeOfMovementSinceLastMeshMofification);
  LM_TRACK_VALUE(m_forceVerifyPositionAfterSculpting);
  LM_ASSERT_IDENTICAL(movement);
  LM_ASSERT_IDENTICAL(deltaTime);

  LM_DRAW_CROSS(state->refPosition, 20.0f, lmColor::GREEN);
  LM_DRAW_CROSS(state->closestPointOnMesh.position, 20.0f, lmColor::RED);
  LM_DRAW_ARROW(state->closestPointOnMesh.position, state->closestPointOnMesh.position + state->closestPointOnMesh.normal * 40.0f, lmColor::RED);

  if (movement.norm() < LM_EPSILON) {
    if (mesh->getRotationVelocity_notSmoothed() > 0.0f) {
      IsoCameraConstrainWhenSpinning(mesh, state);
    }

    IsoUpdateCameraTransform(-state->refNormal, state, deltaTime);
    UpdateCameraInWorldSpace();

    // do nothing.
    return;
  }

  m_timeOfMovementSinceLastMeshMofification += deltaTime;
  LM_TRACK_VALUE(m_timeOfMovementSinceLastMeshMofification);

  LM_TRACK_VALUE(state->numFailedUpdates);

  if(state->numFailedUpdates) {
#if LM_LOG_CAMERA_LOGIC_4
    std::cout << "dist: " << isoState.refDist <<
                 ", pos: " << isoState.refPosition.x() <<
                 ", " << isoState.refPosition.y() <<
                 ", " << isoState.refPosition.z() <<
                 ", nml: " << isoState.refNormal.x() <<
                 ", " << isoState.refNormal.y() <<
                 ", " << isoState.refNormal.z() <<
                 ", mov: " << movement.x() <<
                 ", " << movement.y() <<
                 ", " << movement.z() << std::endl;
#endif
  }

  static const int MAX_NUM_FAILED_UPDATED_BEFORE_CAMERA_RESET = 60;
  if (state->numFailedUpdates++ > MAX_NUM_FAILED_UPDATED_BEFORE_CAMERA_RESET) {
    bool keepCloseToMesh = true;
    ResetCamera(mesh, -Vector3::UnitZ(), keepCloseToMesh);
    state->numFailedUpdates = 0;
  }

  IsoOnMeshUpdateStopped(mesh, state);

  IsoResetIfInsideManifoldMesh(mesh, state);

  // Check if current refPosition is inside the mesh (which may happen sculpting) and correct it.
  {
    int attemptCount = 0;
    lmRayCastOutput raycastOutput;
    CastOneRay(mesh, lmRay(m_transform.translation, state->refPosition), &raycastOutput);
    while (raycastOutput.isSuccess()) {
      LM_ASSERT(lmInRange(raycastOutput.fraction, 0.0f, 1.0f), "Invalid raycast output.");
#if LM_LOG_CAMERA_LOGIC_4
      std::cout << "Moving camera out of mesh!" << std::endl;
#endif
      //state->refPosition = raycastOutput.position + (-1.0f / std::min(raycastOutput.normal.dot(-GetCameraDirection()), 0.2f)) * GetCameraDirection();
      if (GetCameraDirection().dot(raycastOutput.normal) < 0) {
        state->refPosition = raycastOutput.position + m_params.minDist * raycastOutput.normal;
      } else {
        state->refPosition = raycastOutput.position - m_params.minDist * GetCameraDirection();
      }
#if LM_LOG_CAMERA_LOGIC_3
      std::cout << "Fixed ref point in the mesh" << std::endl;
#endif

      if (++attemptCount > 10 ||  (state->refPosition - m_transform.translation).dot(GetCameraDirection()) < 0.0f ) {
        const bool keepCloseToMesh = true;
        ResetCamera(mesh, GetCameraDirection(), keepCloseToMesh);
        break;
      }

      raycastOutput.invalidate();
      CastOneRay(mesh, lmRay(m_transform.translation, state->refPosition), &raycastOutput);
    }
  }

  Vector3 oldRefPosition = state->refPosition;
//  lmReal oldRefDist = state->refDist;

  // Dummy & temp: clip z movement
  Vector3 scaledMovement = movement * std::sqrt(std::sqrt(state->refDist / m_params.refDistForMovemement));
  scaledMovement.z() *= m_params.scaleZMovement;

  Vector3 clippedMovement = m_transform.rotation * scaledMovement;
  //BREAKS: LM_ASSERT_IDENTICAL(clippedMovement);

  // Saftey clip movement so that the camera doesn't go past the safety distance.
  if (lmIsNormalized(state->closestPointOnMesh.normal) && ((state->refPosition - state->closestPointOnMesh.position) + clippedMovement).norm() > GetMaxDistanceForMesh(mesh)/(1+m_params.isoRefDistMultiplier)*1.5f) {
    // Clip to the closest point
    Vector3 newRelPos = (state->refPosition - state->closestPointOnMesh.position) + clippedMovement;
    newRelPos.normalize();
    newRelPos *= GetMaxDistanceForMesh(mesh)/(1+m_params.isoRefDistMultiplier)*1.5f;
    Vector3 newPos = state->closestPointOnMesh.position + newRelPos;
    clippedMovement = newPos - state->refPosition;
  }

  Vector3 newNormal = IsoNormal(mesh, state->refPosition + clippedMovement, IsoQueryRadius(mesh, state));
  if (!lmIsNormalized(newNormal)) {
#if LM_LOG_CAMERA_LOGIC_4
    std::cout << "New IsoNormal not found, broadening the search." << std::endl;
#endif
    lmSurfacePoint closestPoint = GetClosestSurfacePoint(mesh, state->refPosition + clippedMovement, 10.0f*GetMeshSize(mesh));
    if (!lmIsNormalized(closestPoint.normal)) {
#if LM_LOG_CAMERA_LOGIC_4
      std::cout << "Now IsoNormal failed (closest point not found)." << std::endl;
#endif
      return;
    }
    state->refDist = (closestPoint.position - (state->refPosition + clippedMovement)).norm();
    newNormal = IsoNormal(mesh, state->refPosition + clippedMovement, IsoQueryRadius(mesh, state));
    if (!lmIsNormalized(newNormal)) {
#if LM_LOG_CAMERA_LOGIC_4
      std::cout << "Now IsoNormal failed (while closest point was ok)." << std::endl;
#endif
      return;
    }
  }

  if (lmIsNormalized(newNormal)) {
    // clip movement based on normal change.
    Vector3 oldSurfacePosition = state->refPosition - state->refDist * state->refNormal;
    Vector3 newSurfacePosition = state->refPosition + clippedMovement - state->refDist * newNormal;
    lmReal surfaceDist = (newSurfacePosition - oldSurfacePosition).norm();

    if (surfaceDist > clippedMovement.norm()) {
      // Scale movement
      lmReal scale = clippedMovement.norm() / surfaceDist;
      clippedMovement *= scale;
      newNormal = IsoNormal(mesh, state->refPosition + clippedMovement, IsoQueryRadius(mesh, state));
#if LM_LOG_CAMERA_LOGIC_3
      if (scale > 0.0f) {
        std::cout << "Movement scaling: " << scale << std::endl;
      }
#endif
      if (!lmIsNormalized(newNormal)) {
#if LM_LOG_CAMERA_LOGIC_4
        std::cout << "New IsoNormal wrong, after angle-movement clipping." << std::endl;
#endif
        return;
      }
    }
  }

  // Verify movement
  {
    lmReal radiusForCameraSphereCollision = m_params.minDist;
    //lmReal radiusForCameraSphereCollision = 0.0f;
    bool validMovement = VerifyCameraMovement(mesh, state->refPosition, state->refPosition + clippedMovement, radiusForCameraSphereCollision);
    if (!validMovement && clippedMovement.norm() > LM_EPSILON_SQR) {
#if LM_LOG_CAMERA_LOGIC_4
      std::cout << "Invalid camera movement (collision)." << std::endl;
#endif

      const int numCorrections = 5;
      const Vector3 originalMovement = clippedMovement;
      const lmReal originalLength = clippedMovement.norm();
      const Vector3 originalMovementDir = clippedMovement / originalLength;
      lmQuat rotToZ; rotToZ.setFromTwoVectors(originalMovementDir, -GetCameraDirection());
      int i;
      for (i = 0; i < numCorrections && !validMovement; i++)
      {
        const lmReal t = (i+1.0f)/numCorrections;
        lmQuat correction = lmQuat::Identity().slerp(t, rotToZ);
        clippedMovement = correction * originalMovementDir;
        clippedMovement *= (1.0f-(lmReal(i+1.0f)/lmReal(numCorrections+1.0f))) * originalLength;
        //LM_DRAW_ARROW(Vector3::UnitY()*10.0f + state->refPosition, Vector3::UnitY()*10.0f + state->refPosition + clippedMovement*10.0f, lmColor::BLUE);
        validMovement = VerifyCameraMovement(mesh, state->refPosition, state->refPosition + clippedMovement, radiusForCameraSphereCollision);
      }

      if (!validMovement && clippedMovement.norm() > LM_EPSILON_SQR) {
#if LM_LOG_CAMERA_LOGIC_4
        std::cout << "Invalid camera movement (after correcting z)." << std::endl;
#endif
        return;
      } else {
#if LM_LOG_CAMERA_LOGIC_4
        std::cout << "Corrected movement " << i << " times." << std::endl;
#endif
      }
    }
  }

  // Update closest distance: last check
  lmSurfacePoint closestPoint = GetClosestSurfacePoint(mesh, state->refPosition + clippedMovement, state->refDist + clippedMovement.norm());
  if (!lmIsNormalized(closestPoint.normal)) {
#if LM_LOG_CAMERA_LOGIC_4
    std::cout << "Ref Dist too small for queries" << std::endl;
#endif
    closestPoint = GetClosestSurfacePoint(mesh, state->refPosition + clippedMovement, 10.0f * GetMeshSize(mesh));
    if (!lmIsNormalized(closestPoint.normal)) {
#if LM_LOG_CAMERA_LOGIC_
      std::cout << "ClosestPoint normal wrong." << std::endl;
#endif
      return;
    }
  }

  state->refPosition += clippedMovement;
  state->refNormal = newNormal;
  state->closestPointOnMesh = closestPoint;
  state->refDist = (state->closestPointOnMesh.position - state->refPosition).norm();

  LM_ASSERT(lmIsNormalized(state->refNormal), "Iso normal failed.")

  if (mesh->getRotationVelocity_notSmoothed() > 0.0f) {
    IsoCameraConstrainWhenSpinning(mesh, state);
  }

  // Update camera
  IsoUpdateCameraTransform(-state->refNormal, state, deltaTime);

  LM_ASSERT(state->refDist < 10 * GetMeshSize(mesh), "Reference distance exploded.");

  IsoPreventCameraInMesh(mesh, state);

  state->numFailedUpdates = 0;

  lmReal currPotential = 0.0f;
#if LM_LOG_CAMERA_LOGIC_5
  std::cout << "Ref/curr potential: " << state->refPotential << " / " << currPotential << std::endl;
#endif
}

void CameraUtil::IsoCameraConstrainWhenSpinning( Mesh* mesh, IsoCameraState* state )
{
  // Orient normal towards Y-axis
  if (true)
  {
    // Fancy clipping towards Y-axis
    Vector3 fromOrigin = state->refPosition;
    Vector3 fromYAxis = fromOrigin; fromYAxis.y() = 0.0f;
    Vector3 refNormalFromY = state->refNormal; refNormalFromY.y() = 0.0f;
    if (!lmIsZero(fromYAxis) && !lmIsZero(refNormalFromY)) {
      fromYAxis.normalize();
      refNormalFromY.normalize();
      lmQuat correction; correction.setFromTwoVectors(refNormalFromY, fromYAxis);
      state->refNormal = correction * state->refNormal;
    }
  }
  else
  {
    // Just clipping toward origin
    if (!lmIsZero(state->refPosition))
    {
      state->refNormal = state->refPosition.normalized();
    }
  }

  // raycast from camera & move back the refPoint if needed.
  Vector3 rayStart = state->refPosition + state->refNormal * 2.0f * GetMeshSize(mesh);
  Vector3 rayEnd = state->refPosition - state->refNormal * state->refDist;
  lmRayCastOutput raycastOutput;
  lmRay ray(rayStart, rayEnd);
  CastOneRay(mesh, ray, &raycastOutput);
  if (raycastOutput.isSuccess()) {
    LM_ASSERT(lmInRange(raycastOutput.fraction, 0.0f, 1.0f), "Raycast output invalid.");
    state->refPosition = raycastOutput.position - ray.GetDirection() * state->refDist;
  }
}

void CameraUtil::IsoResetIfInsideManifoldMesh(Mesh* mesh, IsoCameraState* isoState) {
  // Only use for manifold meshes..
  lmRay ray0(isoState->refPosition, isoState->refPosition - 10.0f * GetMeshSize(mesh) * isoState->refNormal);
  lmRay ray1(isoState->refPosition, isoState->refPosition + 10.0f * GetMeshSize(mesh) * isoState->refNormal);
  std::vector<lmRayCastOutput> results0;
  std::vector<lmRayCastOutput> results1;
  const bool collectAll = true;
  CastOneRay(mesh, ray0, &results0, collectAll);
  CastOneRay(mesh, ray1, &results1, collectAll);

  LM_TRACK_VALUE(m_numFramesInsideManifoldMesh);
  if ((results0.size() % 2) && (results1.size() % 2)) {
    // Inside mesh && first output
#if LM_LOG_CAMERA_LOGIC_4
    std::cout << "Inside manifold mesh." << std::endl;
#endif
    m_numFramesInsideManifoldMesh++;
    if (m_numFramesInsideManifoldMesh > 60) {
      m_numFramesInsideManifoldMesh = 0;
      ResetCamera(mesh, GetCameraDirection());
    }
  } else {
    m_numFramesInsideManifoldMesh = 0;
  }
}

void CameraUtil::IsoOnMeshUpdateStopped(Mesh* mesh, IsoCameraState* state) {

  //// this is not called when sculpting
  //bool justSculpted = (timeOfLastScupt != state->lastSculptTime); // find
  //state->lastSculptTime = timeOfLastScupt;

  if (m_forceVerifyPositionAfterSculpting) {
    // Just stopped sculpting
    //std::cout << "Stopped sculpting" << std::endl;

    // Raycast towards reference point, if collision, move ref point. remember Distance, and try to preserve it.
    lmRay ray(m_transform.translation, state->refPosition);
    ray.end += ray.GetDirection() * (m_params.minDist * 2.0f + 0.1f);
    lmRayCastOutput raycastOutput;
    CastOneRay(mesh, ray, &raycastOutput);

    if (raycastOutput.isSuccess()) {
#if LM_LOG_CAMERA_LOGIC_4
      std::cout << "Moving reference point" << std::endl;
#endif
      const lmReal refToCamDist = (1+m_params.isoRefDistMultiplier); // convert between refDist and distance to camera
      // Adjust refPoint & ref dist
      //lmReal prevDistToMesh = state->refDist * refToCamDist;
      lmReal currRefDist = raycastOutput.dist / refToCamDist;
      currRefDist = std::max(currRefDist, m_params.minDist * 2.0f + 0.1f);

      state->refDist = currRefDist;
      state->refPosition = raycastOutput.position - ray.GetDirection() * state->refDist;
      // Don't update normal now.

      // We'll need to ease in normal adaption.
    }

    // Readjust normal blending
    lmSurfacePoint closestPoint = GetClosestSurfacePoint(mesh, state->refPosition, (m_params.minDist + 0.1f) * 1.5f);
    lmReal dist = (closestPoint.position - state->refPosition).norm();
    int attempts = 0;
    while((dist < m_params.minDist + 0.1f) && attempts < 10) {
      lmReal diff = m_params.minDist + 0.1f - dist;
      state->refPosition += closestPoint.normal * diff * 1.2f;

      closestPoint = GetClosestSurfacePoint(mesh, state->refPosition, (m_params.minDist + 0.1f) * 1.5f);
      dist = (closestPoint.position - state->refPosition).norm();
      attempts++;
    }

    m_forceVerifyPositionAfterSculpting = false;
  }
}

void CameraUtil::IsoMoveTowardsPotential( Mesh* mesh, IsoCameraState* state )
{
  // use precomputed normal & move reference point towards there

  lmReal currPotential = IsoPotential(mesh, state->refPosition, IsoQueryRadius(mesh, state));
  lmReal err = currPotential - state->refPotential;

  const static lmReal LM_POTENTIAL_QUERY_EPSILON = 0.1f;
  Vector3 testPosition = state->refPosition + state->refNormal * LM_POTENTIAL_QUERY_EPSILON;
  lmReal testPotential = IsoPotential(mesh, testPosition, IsoQueryRadius(mesh, state));

  lmReal deltaPotential = testPotential - currPotential;

  lmReal smoothing = 0.8f; // 1.0f explodes easily
  float correctionRatio = - err / deltaPotential * LM_POTENTIAL_QUERY_EPSILON * smoothing;
  correctionRatio = lmClip(correctionRatio, -10.0f, 10.0f); // use min(minDist, current distance to closest point)

  Vector3 correction = state->refNormal * correctionRatio;
  state->refPosition += correction;

  // todo: udpate query radius ?? can simply add correctionRatio for now & update later.

  state->refNormal = IsoNormal(mesh, state->refPosition, IsoQueryRadius(mesh, state));

  // test: disable z movement from from the hand


  // todo:
  //  - update refPotential whenever needed
  //    - after camera is moved.
}

lmSurfacePoint CameraUtil::GetReferencePoint() const
{
  lmSurfacePoint result;
  result.position = isoState.refPosition - isoState.refNormal * isoState.refDist;
  result.normal = isoState.refNormal;
  return result;
}

lmReal CameraUtil::GetReferenceDistance() const
{
  return isoState.refDist * (1 + m_params.isoRefDistMultiplier);
}

lmReal CameraUtil::GetMaxDistanceForMesh(const Mesh* mesh) const
{
  const lmReal maxDistScale = 0.5f * GetMeshSize(mesh) / Mesh::globalScale_;
  return m_params.maxDist * maxDistScale;
}

lmReal CameraUtil::GetMeshSize(const Mesh* mesh) const {
  lmReal meshSize = mesh->getOctree()->getAabbSplit().getDiagonalLength();
  // Clip max mesh size used for computations.
  meshSize = std::min(meshSize, 3.0f * (2.0f * Mesh::globalScale_));
  return meshSize;
}
