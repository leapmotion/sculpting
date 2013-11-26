#include "StdAfx.h"
#include "CameraUtil.h"
#include "DebugDrawUtil.h"
#include "Geometry.h"
#include "DataTypes.h"

#if LM_PRODUCTION_BUILD
#define LM_LOG_CAMERA_LOGIC 0
#define LM_LOG_CAMERA_LOGIC_2 0
#define LM_LOG_CAMERA_LOGIC_3 0
#define LM_LOG_CAMERA_LOGIC_4 0
#define LM_DRAW_DEBUG_OBJECTS 0
#else
#define LM_LOG_CAMERA_LOGIC 0
#define LM_LOG_CAMERA_LOGIC_2 0
#define LM_LOG_CAMERA_LOGIC_3 0
#define LM_LOG_CAMERA_LOGIC_4 0
#define LM_DRAW_DEBUG_OBJECTS 1
#endif

CameraUtil::CameraUtil() {
  m_transform.setIdentity();
  m_meshTransform.setIdentity();
  m_referencePoint.position.setZero();
  m_referencePoint.normal.setZero(); // invalid
  m_referenceDistance = 0.0f;
  m_userInput.setZero();
  m_accumulatedUserInput.setZero();
  state = STATE_INVALID;
  m_debugDrawUtil = NULL;
  m_lastCameraUpdateTime = -1.0f;
  m_avgVertex.position.setZero();
  m_avgVertex.normal.setZero();
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
}

void CameraUtil::SetFromStandardCamera(const Vector3& from, const Vector3& to, lmReal referenceDistance) {
  //std::unique_lock<std::mutex> lock(transformForGraphicsMutex);
  //GetTransformFromStandardCamera(from, to, transform);
  //this->referenceDistance = referenceDistance;
  state = STATE_FREEFLOATING;
}

void CameraUtil::ResetCamera(const Mesh* mesh, const Vector3& cameraDirection) {
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

  //std::unique_lock<std::mutex> lock(mutex);

  // Set reference point to the selected vertex
  const Vertex& closest = vertices[idx];
  m_referencePoint.position = closest;
  m_referencePoint.normal = closest.normal_;

  // Rotate camera to the new requested direction
  Vector3 currentNegZ = -1.0f * (m_transform.rotation * Vector3::UnitZ());
  lmQuat correction; correction.setFromTwoVectors(currentNegZ, cameraDirection);
  m_transform.rotation = correction * m_transform.rotation;

  // Set camera position
  m_transform.translation = m_referencePoint.position + m_referenceDistance * (m_transform.rotation * Vector3::UnitZ());

  // reset iso camera
  if (m_params.cameraOverrideIso)
  {
    //isoState.refDist = lmClip(isoState.refDist, params.minDist, params.maxDist);
    isoState.refDist = m_params.maxDist/(1+m_params.isoRefDistMultiplier)*1.0f;
    isoState.refNormal = m_referencePoint.normal;
    isoState.refPosition = m_referencePoint.position + isoState.refDist * m_referencePoint.normal;

    m_transform.translation = isoState.refPosition + isoState.refDist * m_params.isoRefDistMultiplier * (m_transform.rotation * Vector3::UnitZ());
  }

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

void CameraUtil::GetClosestPoint(const Mesh* mesh, const lmSurfacePoint& referencePoint, lmReal radius, const Vector3& cameraDirection, Geometry::GetClosestPointOutput* closestPointOut) {
  std::vector<int> verts;
  std::vector<int> tris;
  const_cast<Mesh*>(mesh)->getVerticesInsideSphere(referencePoint.position, radius*radius, *&verts);
  const_cast<Mesh*>(mesh)->getTrianglesFromVertices(verts, tris);

  Geometry::GetClosestPointOutput closestPoint;
  closestPoint.distanceSqr = FLT_MAX;
  int closestTriangleIdx = -1;;

  for (size_t ti = 0; ti < tris.size(); ti++) {
    const Triangle& tri = mesh->getTriangle(tris[ti]);

    // Discard rear-facing triangles
    lmReal camDotTNormal = cameraDirection.dot(tri.normal_);
    if (camDotTNormal < 0.0f) {
      Geometry::GetClosestPointOutput output;
      {
        Geometry::GetClosestPointInput input;
        input.mesh = mesh;
        input.tri = &tri;
        input.point = referencePoint.position;
        Geometry::getClosestPoint(input, &output);
        output.triIdx = tris[ti];
        if (output.distanceSqr < closestPoint.distanceSqr) {
          closestPoint = output;
          closestPoint.normal = tri.normal_;
          closestTriangleIdx = tris[ti];
        }
      }
    }
  }

  if (closestPoint.distanceSqr < FLT_MAX) {
    GetNormalAtPoint(mesh, closestTriangleIdx, closestPoint.position, &closestPoint.normal);
  }

  *closestPointOut = closestPoint;
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

void CameraUtil::FindPointsAheadOfMovement(const Mesh* mesh, const lmSurfacePoint& referencePoint, lmReal radius, const Vector3& movementDirection, std::vector<int>* vertices ) {
  LM_ASSERT(LM_EPSILON * LM_EPSILON < movementDirection.squaredNorm(), "");
  Vector3 movementDir = movementDirection.normalized();
  Vector3 mixedDir = (movementDir + referencePoint.normal).normalized();

  std::vector<int> verts;
  const_cast<Mesh*>(mesh)->getVerticesInsideSphere(referencePoint.position, radius*radius, *&verts);
  for (size_t vi = 0; vi < verts.size(); vi++) {
    Vector3 translationToVert = (mesh->getVertex(verts[vi]) - referencePoint.position);
    Vector3 dirToVert = translationToVert.normalized();
    lmReal dot = movementDir.dot(dirToVert);
    lmReal dotMixed = mixedDir.dot(dirToVert);
    lmReal dotNormal = referencePoint.normal.dot(dirToVert);
    const lmReal dist = translationToVert.norm();
    if ((0.707f < dot && 0.3f * radius < dist && -0.2f < dotNormal ) ||
        (0.707f < dotMixed && 0.1f * radius < dist && dist < 0.3f * radius) ||
        (0.25f < dotNormal && 0.1f * radius < dist && dist < 0.3f * radius)) {
      vertices->push_back(vi);
    }
  }
}

void CameraUtil::GetAveragedSurfaceNormal(const Mesh* mesh, const lmSurfacePoint& referencePoint, lmReal radius, const Vector3& cameraDirection, bool weightNormals, lmSurfacePoint* avgSurfacePoint, lmSurfacePoint* pureAvgSurfacePoint, Geometry::GetClosestPointOutput* closestPointOut) {
  LM_ASSERT(lmIsNormalized(cameraDirection), "Camera direction not normalized.");

  avgSurfacePoint->setZero();
  pureAvgSurfacePoint->setZero();
  closestPointOut->setInvalid();

  Vector3 normal = Vector3::Zero();
  Vector3 position = Vector3::Zero();
  Vector3 pureNormal = Vector3::Zero();
  Vector3 purePosition = Vector3::Zero();

  Geometry::GetClosestPointOutput closestPoint;
  closestPoint.setInvalid();

  std::vector<int> verts;
  const_cast<Mesh*>(mesh)->getVerticesInsideSphere(referencePoint.position, radius*radius, *&verts);

  if (m_params.userFaultyTriangles) {
    std::vector<int> tris;
    const_cast<Mesh*>(mesh)->getTrianglesFromVertices(verts, tris);
    lmReal area = 0.0f;
    lmReal pureArea = 0.0f;

    for (size_t ti = 0; ti < tris.size(); ti++) {
      const Triangle& tri = mesh->getTriangle(tris[ti]);

      // Discard rear-facing triangles
      lmReal camDotTNormal = cameraDirection.dot(tri.normal_);
      Vector3 center = TriCenter(mesh, tri);
      if (camDotTNormal < 0.0f) {
        lmReal weight = weightNormals ? std::max(0.0f, -camDotTNormal) : 1.0f;
        // Calc distance and weight
        lmReal a = tri.area;// TriArea(mesh, tri);

        Geometry::GetClosestPointOutput output;
        {
          Geometry::GetClosestPointInput input;
          input.mesh = mesh;
          input.tri = &tri;
          input.point = referencePoint.position;
          Geometry::getClosestPoint(input, &output);
          output.triIdx = tris[ti];
          if (output.distanceSqr < closestPoint.distanceSqr) {
            closestPoint = output;
          }
#if LM_DRAW_DEBUG_OBJECTS
          if (m_debugDrawUtil && m_params.drawDebugLines && m_params.drawSphereQueryResults) {
            m_debugDrawUtil->DrawTriangle(mesh, tri);
          }
#endif
        }

        // Calc point weight
        lmReal t = std::sqrt(closestPoint.distanceSqr) / radius; // todo remove sqrt later
        t = 1.0f - std::min(1.0f, t);
        t = t*t;

        // Average taking area into account
        normal += t * weight * a * tri.normal_;
        position += t * weight * a * center;
        area += t * weight * a;
        pureNormal += t * a * tri.normal_;
        purePosition += t * a * center;
        pureArea += t * a;
      }
    }

    if (0.0f < area) {
      avgSurfacePoint->position = position / area;
      avgSurfacePoint->normal = normal.normalized();
    }
    if (0.0f < pureArea) {
      pureAvgSurfacePoint->position = purePosition / pureArea;
      pureAvgSurfacePoint->normal = pureNormal.normalized();
    }

  } else {

    lmReal sumWeight = 0.0f;

    // no pure implenetation

    for (size_t vi = 0; vi < verts.size(); vi++) {
      const Vertex& vert = mesh->getVertex(verts[vi]);

      // Discard rear-facing triangles
      lmReal camDotVNormal = cameraDirection.dot(vert.normal_);
      if (camDotVNormal < 0.0f) {
        lmReal weight = weightNormals ? std::max(0.0f, -camDotVNormal) : 1.0f;
        normal += vert.normal_ * weight;
        position += weight * vert;
        sumWeight += weight;
#if LM_DRAW_DEBUG_OBJECTS
        if (m_debugDrawUtil && m_params.drawDebugLines && m_params.drawSphereQueryResults) {
          LM_DRAW_CROSS(vert, 5.0f, lmColor::WHITE);
        }
#endif
      }
    }
    if (0.0f < sumWeight) {
      avgSurfacePoint->position = position / sumWeight;
      avgSurfacePoint->normal = normal.normalized();

      // hack
      *pureAvgSurfacePoint = *avgSurfacePoint;
    }
  }

  if (closestPoint.isValid()) {
    GetNormalAtPoint(mesh, closestPoint.triIdx, closestPoint.position, &closestPoint.normal);
  }
  *closestPointOut = closestPoint;
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

void CameraUtil::GetSmoothedNormalAtPoint(const Mesh* mesh, int triIdx, const Vector3& point, lmReal radius, Vector3* normalOut) {
  Vector3 coords;
  GetBarycentricCoordinates(mesh, triIdx, point, &coords);
  const Triangle& tri = mesh->getTriangle(triIdx);
  const Vertex& v1 = mesh->getVertex(tri.vIndices_[0]);
  const Vertex& v2 = mesh->getVertex(tri.vIndices_[1]);
  const Vertex& v3 = mesh->getVertex(tri.vIndices_[2]);

  lmSurfacePoint avgSurfacePoint[3];
  lmSurfacePoint pureAvgSurfacePoint[3];
  Geometry::GetClosestPointOutput closestPoint[3];
  GetAveragedSurfaceNormal(mesh, lmSurfacePoint(v1, v1.normal_), radius, -1.0f * v1.normal_, true, avgSurfacePoint+0, pureAvgSurfacePoint+0, closestPoint+0);
  GetAveragedSurfaceNormal(mesh, lmSurfacePoint(v2, v2.normal_), radius, -1.0f * v2.normal_, true, avgSurfacePoint+1, pureAvgSurfacePoint+1, closestPoint+1);
  GetAveragedSurfaceNormal(mesh, lmSurfacePoint(v3, v3.normal_), radius, -1.0f * v3.normal_, true, avgSurfacePoint+2, pureAvgSurfacePoint+2, closestPoint+2);

  Vector3 normal = Vector3::Zero();
  normal += coords[0] * avgSurfacePoint[0].normal;
  normal += coords[1] * avgSurfacePoint[1].normal;
  normal += coords[2] * avgSurfacePoint[2].normal;

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
  //std::unique_lock<std::mutex> lock(mutex);

  m_transform.rotation = m_meshTransform.rotation * m_transform.rotation;
  m_transform.translation = ToWorldSpace(m_transform.translation);
  m_referencePoint.position = ToWorldSpace(m_referencePoint.position);
  m_referencePoint.normal = ToWorldSpace(m_referencePoint.normal);

  isoState.refPosition = ToWorldSpace(isoState.refPosition);
  isoState.refNormal = ToWorldSpace(isoState.refNormal);
  isoState.closestPointOnMesh.position = ToWorldSpace(isoState.closestPointOnMesh.position);
  isoState.closestPointOnMesh.normal = ToWorldSpace(isoState.closestPointOnMesh.normal);

  // Get mesh's transformstion
  m_meshTransform = lmTransformFromMatrix(mesh->getRotationMatrix(), mesh->getTranslation());

  m_transform.rotation = m_meshTransform.rotation.inverse() * m_transform.rotation;
  m_transform.translation = ToMeshSpace(m_transform.translation);
  m_referencePoint.position = ToMeshSpace(m_referencePoint.position);
  m_referencePoint.normal = ToMeshSpace(m_referencePoint.normal);

  isoState.refPosition = ToMeshSpace(isoState.refPosition);
  isoState.refNormal = ToMeshSpace(isoState.refNormal);
  isoState.closestPointOnMesh.position = ToMeshSpace(isoState.closestPointOnMesh.position);
  isoState.closestPointOnMesh.normal = ToMeshSpace(isoState.closestPointOnMesh.normal);
}

void CameraUtil::EnsureReferencePointIsCloseToMesh(const Mesh* mesh, Params* paramsInOut) {
  bool resetCamera = false;
  if (m_params.cameraOverrideIso) {
    //// Iso camera
    //std::cout << "Iso cam: dist: " <<
    //             isoState.refDist << ", pos: " << isoState.refPosition.x() << ", " << isoState.refPosition.y() << ", " <<
    //             isoState.refPosition.z() << std::endl;

  } else {
    // Normal camera
    const lmReal radius = GetSphereQueryRadius();
    std::vector<int> verts;
    const_cast<Mesh*>(mesh)->getVerticesInsideSphere(m_referencePoint.position, radius*radius, *&verts);
    resetCamera = !verts.size();
  }

  if (resetCamera && m_params.enableCameraReset ) {
    ResetCamera(mesh, GetCameraDirection());
  }
}

static Vector3 lmProjectAlongVec(const Vector3& in, const Vector3& projectionDirection) {
  return in - projectionDirection * in.dot(projectionDirection);
}

void CameraUtil::OrbitCamera( const Mesh* mesh, lmReal deltaTime ) {

  //std::unique_lock<std::mutex> lock(mutex);

  // Horizontal rotate
  static const lmReal ORBIT_RATE = LM_2PI / 40.0f;
  lmQuat q(AngleAxis( - ORBIT_RATE * deltaTime, Vector3::UnitY()));
  m_transform.mul(q);
  lmSurfacePoint referencePointNew = m_referencePoint;
  referencePointNew.mul(q);

  // Orbit camera with raycast
  if (1) {
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
      Vector3 rayStart = (q * Vector3::UnitZ()) * m_params.maxDist; // + max aabb
      lmRayCastOutput raycastHit;
      CastOneRay(mesh, lmRay(rayStart, Vector3::Zero()), &raycastHit);

      // if raycast hit (move last reference point, remember last point)
      if (raycastHit.isSuccess()) {
        lmReal distFromOrigin = raycastHit.position.norm();
        if (distFromOrigin + m_params.minDist > refPtDistFromOrigin + m_referenceDistance ) {
          m_referenceDistance = distFromOrigin + m_params.minDist - refPtDistFromOrigin;
        }
      }
    }

    // Fix reference point
    {
      std::unique_lock<std::mutex> lock(m_referencePointMutex);
      m_referencePoint.position = (q * Vector3::UnitZ()) * refPtDistFromOrigin;
      m_referencePoint.normal = q * Vector3::UnitZ();
    }
    // Fix camera direction & position
    static const lmReal ORBITING_BLEND_IN_TIME = 5.0f;
    const lmReal blendingFactor = lmClip(m_timeSinceOrbitingStarted / ORBITING_BLEND_IN_TIME, 0.0f, 1.0f);

    //transform.rotation = q;
    m_transform.rotation = m_transform.rotation.slerp(blendingFactor*deltaTime, q);
    Vector3 targetTranslation = (m_transform.rotation * Vector3::UnitZ()) * (refPtDistFromOrigin + m_referenceDistance);
    m_transform.translation = lmInterpolate(blendingFactor*deltaTime, m_transform.translation, targetTranslation);
  }
}

void CameraUtil::UpdateParamsToWalkSmoothedNormals(Params* paramsInOut) {
  paramsInOut->freeRotationEnabled = false;
  paramsInOut->enableNormalCorrection = false;
  paramsInOut->useAvgNormal = false;
  paramsInOut->suppresForwardRotation = false;
  paramsInOut->tmpSwitch = false;
  paramsInOut->walkSmoothedNormals = true;
}

void CameraUtil::UpdateCamera( Mesh* mesh, Params* paramsInOut) {

  if (mesh->getVertices().empty())
  {
    // mesh has no vertices. do nothing.
    return; 
  }

  LM_ASSERT(mesh, "Can't upate the camera without a mesh");
  UpdateMeshTransform(mesh, paramsInOut);

  // Check time
  lmReal dt = 0.0f;
  {
    lmReal prevTime = m_lastCameraUpdateTime;
    lmReal time = lmReal(ci::app::getElapsedSeconds());
    m_lastCameraUpdateTime = time;
    if (prevTime < 0.0f) { prevTime = time; }
    dt = time - prevTime;
  }

  if (!this->m_params.walkSmoothedNormals && paramsInOut->walkSmoothedNormals)
  {
    UpdateParamsToWalkSmoothedNormals(paramsInOut);
  }

  if (!this->m_params.cameraOverrideIso && paramsInOut->cameraOverrideIso) {
    InitIsoCamera(mesh, &isoState);
    paramsInOut->userFaultyTriangles = false;
    paramsInOut->minDist = 10.0f;
    paramsInOut->maxDist = 400.0f;
  }
  this->m_params = *paramsInOut;

  static const Mesh* prevMesh = NULL;
  if (mesh != prevMesh) {
    // Init camera, and all
    //EnsureReferencePointIsCloseToMesh(mesh, paramsInOut);
    ResetCamera(mesh, -(Vector3::UnitZ() + -0.3f * Vector3::UnitX() + 0.2f * Vector3::UnitY()).normalized());
    InitIsoCamera(mesh, &isoState);
    m_timeSinceCameraUpdateStarted = FLT_MAX;
  }
  prevMesh = mesh;

  m_framesFromLastCollisions++;

  if (m_params.forceCameraOrbit && m_params.enableCameraOrbit) {
    OrbitCamera(mesh, dt);
    m_timeSinceOrbitingStarted += dt;
    m_timeSinceOrbitingEnded = 0.0f;
    m_timeOfMovementSinceLastMeshMofification = FLT_MAX;
    UpdateCameraInWorldSpace();
    return;
  } else if (m_params.enableCameraReset) {
    // Don't do this when orbiting.
    EnsureReferencePointIsCloseToMesh(mesh, paramsInOut);
  }

  m_timeSinceOrbitingStarted = 0.0f;
  m_timeSinceOrbitingEnded += dt;

  if (m_debugDrawUtil) {
    if (m_params.drawDebugLines) {
      LM_DRAW_ARROW(Vector3::Zero(), 50.0f * ToMeshSpace(Vector3::UnitX()), lmColor::RED );
      LM_DRAW_ARROW(Vector3::Zero(), 50.0f * ToMeshSpace(Vector3::UnitY()), lmColor::GREEN );
      LM_DRAW_ARROW(Vector3::Zero(), 50.0f * ToMeshSpace(Vector3::UnitZ()), lmColor::BLUE );
      LM_DRAW_ARROW(m_transform.translation, m_referencePoint.position, lmColor::YELLOW );
    }
    m_debugDrawUtil->SwitchBuffers();
  }

  lmReal dtOne = 1.0f;
  lmReal dtZero = 0.0f;

  Vector3 usedUserInput = m_userInput;
  // Multiply motion by distance:
  const lmReal distFraction = (m_referenceDistance-m_params.minDist)/(m_params.maxDist-m_params.minDist);
  const lmReal movementRatio = m_params.speedAtMinDist + distFraction * (m_params.speedAtMaxDist-m_params.speedAtMinDist);
  {
    std::unique_lock<std::mutex> lock(m_userInputMutex);
    // Accumulate userInput in 2d
    if (m_params.enableSmoothing) {
        m_accumulatedUserInput += m_userInput;
        lmReal smoothingValue = pow(m_params.smoothingFactor, dt);
        usedUserInput = (1.0f - smoothingValue) * m_accumulatedUserInput;
        m_accumulatedUserInput -= usedUserInput;
    }
    m_userInput.setZero();

    Vector3 movementInCam = usedUserInput * movementRatio; movementInCam.z() = 0.0f;
    movementInCam = -1.0f * (m_transform.rotation * usedUserInput);

    if (100000000.0f <= movementInCam.squaredNorm()) {
      // This actually happened when stopping the Leap Service, and starting a different version.
      m_accumulatedUserInput.setZero();
      usedUserInput.setZero();
    }
  }

  // Process smoothed user input
  Vector3 deltaAngles = usedUserInput * movementRatio;
  Vector3 movementInCam = usedUserInput * movementRatio; movementInCam.z() = 0.0f;
  Vector3 movementInWorld = - usedUserInput * movementRatio;
  movementInCam = -1.0f * (m_transform.rotation * usedUserInput);

  if (m_params.cameraOverrideIso) {
    isoState.cameraOffsetMultiplier = m_params.isoRefDistMultiplier;
    IsoCamera(mesh, &isoState, movementInWorld, dt);
    if (m_params.pinUpVector) { CorrectCameraUpVector(dtOne, Vector3::UnitY()); }
    UpdateCameraInWorldSpace();
    m_prevTimeOfLastSculpt = m_timeOfLastScupt;

    return;
  }

  if(!m_params.enableSmoothing) {
    dt = dtOne;
  }


  const Vector3 oldOldCameraNormal = m_transform.rotation * Vector3::UnitZ();

  // Attempt to calculate new refernce point and normal
  //
  bool rayHit = false;
//  lmReal refDist = 0.0f;
  const lmTransform oldCamera = this->m_transform;

  if (m_params.sphereCrawlMode) {
    int attampts = 0;

    const Geometry::GetClosestPointOutput oldClosestPoint = this->m_closestPoint;
    Geometry::GetClosestPointOutput& newClosestPoint = this->m_closestPoint;
    const lmSurfacePoint oldReferencePoint = this->m_referencePoint;
    lmSurfacePoint& newReferencePoint = this->m_referencePoint;
    const lmTransform oldCamera = this->m_transform;
    lmTransform& newCamera = this->m_transform;
    const lmSurfacePoint oldAvgVertex = this->m_avgVertex;


    LM_ASSERT(oldReferencePoint.normal.squaredNorm() == 0.0f || lmIsNormalized(oldReferencePoint.normal), "");
    LM_ASSERT(lmIsNormalized(oldCamera.rotation * Vector3::UnitZ()), "");

    if (m_debugDrawUtil) {
      LM_DRAW_ARROW(oldReferencePoint.position, oldReferencePoint.position + oldReferencePoint.normal * 50.0f, lmColor::RED);
    }

    // backwards normal camera snapping.
    bool backSnapped = false;
    if (!m_params.useIsoNormal && m_params.enableBackSnapping &&  LM_EPSILON * LM_EPSILON < oldClosestPoint.normal.squaredNorm() &&
      LM_EPSILON * LM_EPSILON < movementInCam.squaredNorm() ) {
      Vector3 oldCameraNormal = oldCamera.rotation * Vector3::UnitZ();

      LM_DRAW_ARROW(oldClosestPoint.position, oldClosestPoint.position + oldClosestPoint.normal * 50.0f, lmColor::GREEN);

      // take direction of movement & old contact normal
      Vector3 movementDir = movementInCam.normalized();
      Vector3 cross = oldClosestPoint.normal.cross(oldAvgVertex.normal);

      if (LM_EPSILON * LM_EPSILON < cross.squaredNorm()) {
        cross.normalize();
        LM_DRAW_ARROW(oldClosestPoint.position, oldClosestPoint.position + cross * 50.0f, lmColor::GREEN);

        Vector3 sideComponent = cross.dot(oldCameraNormal) * cross;
        Vector3 parallelComponent = oldCameraNormal - sideComponent;
        parallelComponent.normalize();

        if (movementDir.dot(oldClosestPoint.normal) < -0.707f && parallelComponent.dot(oldClosestPoint.normal) < 0.707f) {
          // pulling backwards

          std::vector<int> vertices;
          FindPointsAheadOfMovement(mesh, oldReferencePoint, GetSphereQueryRadius(), movementDir, &vertices);
          bool forwardVerticesDetected = 0 < vertices.size();

          if (!m_params.enableForwardCheckForBackSnapping || !forwardVerticesDetected) {
            // now go form contact normal forward 30 deg.
            lmQuat r(AngleAxis( 3.1415f * 1.0/4.0f, cross));
            Vector3 newN = r * oldClosestPoint.normal;
            newN += sideComponent;
            newN.normalize();
            LM_DRAW_ARROW(oldClosestPoint.position, oldClosestPoint.position + newN * 50.0f, lmColor::BLUE);

            CorrectCameraOrientation(dtZero, newN);
            const_cast<lmTransform&>(oldCamera) = newCamera;

#if LM_LOG_CAMERA_LOGIC
            std::cout << "Normal back-pulled." << std::endl;
#endif
            backSnapped = true;
          }
        }
      }
    }

    // Correct camera back to normal orientation
    if (!m_params.useIsoNormal && !backSnapped && m_framesFromLastCollisions > 2 && m_params.enableNormalCorrection &&  LM_EPSILON * LM_EPSILON < oldReferencePoint.normal.squaredNorm())
    {
      Vector3 oldCameraNormal = oldCamera.rotation * Vector3(0.0f, 0.0f, 1.0f);

      // Plane that connect them, if cross product is significant
      Vector3 cross = oldCameraNormal.cross(oldReferencePoint.normal);

      if (LM_EPSILON * LM_EPSILON < cross.squaredNorm())
      {
        cross.normalize();

        // Decompose motion along those directions (movement is done in oldCamera's plane)
        LM_ASSERT(movementInCam.squaredNorm() < 100000000.0f, "");
        Vector3 sideComponent = cross.dot(movementInCam) * cross;
        Vector3 parallelComponent = movementInCam - sideComponent;

        // Subtract from the 'projected' direction as much as we need, or max.
        lmReal dotMovement = parallelComponent.dot(cross.cross(oldReferencePoint.normal));
        if (0.0f < dotMovement) {
          lmReal maxCameraTranslation = (oldReferencePoint.normal - oldCameraNormal).norm() * m_referenceDistance / m_params.freeRotationRatio;
          lmReal parallelNorm = parallelComponent.norm();

          if (LM_EPSILON < parallelNorm) {
            lmReal translationRatio = std::min(1.0f, maxCameraTranslation / parallelNorm);
            lmReal leftover = 1.0f - translationRatio;

            Vector3 newCameraTransform = m_transform.translation + translationRatio * parallelComponent * m_params.freeRotationRatio ;

            Vector3 newNormal = (newCameraTransform - oldReferencePoint.position).normalized();
            CorrectCameraOrientation(dtZero, newNormal);

            // !! if we have more than 1 attempts.
            const_cast<lmTransform&>(oldCamera) = newCamera;

            // Leave the rest as it was.
            movementInCam = sideComponent + leftover * parallelComponent;
            LM_ASSERT(movementInCam.squaredNorm() < 100000000.0f, "");

#if LM_LOG_CAMERA_LOGIC
            std::cout << "Corrected to normal. Leftover: " << leftover << std::endl;
#endif
          }
        }
      }
    }

    //
    Vector3 movementInNml = movementInCam;
    Vector3 movementInOrgCam = movementInCam;
    lmQuat normToCamera = lmQuat::Identity();
    if (m_params.moveInNormalPlane && oldReferencePoint.normal.squaredNorm() != 0.0f)
    {
      LM_ASSERT(lmIsNormalized(oldReferencePoint.normal), "");

      // Rotation from camera to surface normal
      lmQuat toNmlPlane; toNmlPlane.setFromTwoVectors(oldOldCameraNormal, oldReferencePoint.normal);
      movementInNml = toNmlPlane * movementInCam;
      movementInCam = movementInNml;
      normToCamera = toNmlPlane.inverse();
    }

    const lmReal sphereRadius = GetSphereQueryRadius();

    lmSurfacePoint newAvgVertex;
    lmSurfacePoint pureNewAvgVertex;
    bool normalOkay = false;
    while(!normalOkay && attampts < 16)
    {
      LM_ASSERT(oldReferencePoint.normal.squaredNorm() == 0.0f || lmIsNormalized(oldReferencePoint.normal), "");
      LM_ASSERT(lmIsNormalized(oldCamera.rotation * Vector3::UnitZ()), "");

      // Sphere sweep.
      newReferencePoint.position += movementInCam;
      newCamera.translation += movementInCam;

      // Attempt sphere query
      Vector3 cameraDirection = oldCamera.rotation * Vector3(0.0f, 0.0f, -1.0f);
      GetAveragedSurfaceNormal(mesh, newReferencePoint, sphereRadius, cameraDirection, m_params.weightNormals, &newAvgVertex, &pureNewAvgVertex, &newClosestPoint);
      if (!lmIsNormalized(newClosestPoint.normal)) {
        GetClosestPoint(mesh, newReferencePoint, sphereRadius, cameraDirection, &newClosestPoint);
      }

      newReferencePoint.normal = m_params.useAvgNormal ? newAvgVertex.normal : newClosestPoint.normal;

      if (m_params.useIsoNormal) {
//        Vector3 queryPosition = (referencePoint.position + transform.translation) * 0.5f;
//        Vector3 normal = IsoNormal(mesh, queryPosition, GetSphereQueryRadius());
      }

      normalOkay = lmIsNormalized(newReferencePoint.normal);

      if (normalOkay && m_params.walkSmoothedNormals) {
        GetSmoothedNormalAtPoint(mesh, newClosestPoint.triIdx, newClosestPoint.position, sphereRadius, &newClosestPoint.normal);
        if (lmIsNormalized(oldClosestPoint.normal)) {
          lmReal rotDist = (oldClosestPoint.normal - newClosestPoint.normal).norm();// * referenceDistance;
          if (0.2f < rotDist) {
            movementInCam *= 0.2f / rotDist;
            newCamera = oldCamera;
            newReferencePoint = oldReferencePoint;
          }
        }
      }

      if (normalOkay && m_params.overrideNormal) {
        Vector3 smoothedNormal;
        GetSmoothedNormalAtPoint(mesh, newClosestPoint.triIdx, newClosestPoint.position, sphereRadius, &smoothedNormal);
        newReferencePoint.normal = smoothedNormal;
      }

      bool retryMovement = !normalOkay;

      attampts++;
      if (retryMovement && attampts < 4) {
        movementInCam *= 0.5f;
        newCamera = oldCamera;
        newReferencePoint = oldReferencePoint;
      }
    }

    if (lmIsNormalized(newReferencePoint.normal)) {

      bool forwardVerticesDetected = false;

      // apply quaternion of averaged normals.
      const lmSurfacePoint& oldCalcPoint = oldReferencePoint;
      const lmSurfacePoint& newCalcPoint = newReferencePoint;

      if (!m_params.useIsoNormal &&m_params.suppresForwardRotation && lmIsNormalized(oldCalcPoint.normal) && LM_EPSILON * LM_EPSILON < movementInCam.squaredNorm()) {
        std::vector<int> vertices;
        FindPointsAheadOfMovement(mesh, oldReferencePoint, sphereRadius, movementInCam.normalized(), &vertices);
        forwardVerticesDetected = 0 < vertices.size();

        lmQuat normalChange;
        normalChange.setFromTwoVectors(oldCalcPoint.normal, newCalcPoint.normal);

        // Decompose normal update (if not opposite directions)
        if (-0.9f < oldCalcPoint.normal.dot(newCalcPoint.normal) ) {
          Vector3 deltaNormal = newCalcPoint.normal - oldCalcPoint.normal;
          // Decompose: direction along & perp to motion
          Vector3 movementDir = movementInCam.normalized();
          Vector3 deltaNormalAlongMotion = movementDir.dot(deltaNormal) * movementDir;
          Vector3 deltaNormalPerpMotion = deltaNormal - deltaNormalAlongMotion;

          // Apply the normal delta along only if it's towards the back
          if (m_params.tmpSwitch || forwardVerticesDetected) {
            forwardVerticesDetected = true;
#if LM_LOG_CAMERA_LOGIC
            std::cout << "ORIENTATION backward detected" << std::endl;
#endif
          } else {
#if LM_LOG_CAMERA_LOGIC
            std::cout << "ORIENTATION forward, clipped" << std::endl;
#endif
            // forward normal change -- let the 'explicity camera movement' code handle that
            Vector3 clippedNormal = (oldCalcPoint.normal + deltaNormalPerpMotion).normalized();
            normalChange.setFromTwoVectors(oldCalcPoint.normal, clippedNormal);
          }
        }

        // apply to camera transform
        Vector3 cameraNormal = newCamera.rotation * Vector3::UnitZ();
        Vector3 newNormal = normalChange * cameraNormal; //-- only have it for backward bending angles ?? or when the above rotation is not engaged...
        CorrectCameraOrientation(dtZero, newNormal);

        // check
        Vector3 newCameraNormal = newCamera.rotation * Vector3::UnitZ();
        lmQuat rotCheck; rotCheck.setFromTwoVectors(cameraNormal, newCameraNormal);
        int i = 0; i++;
      }

      // Cast reference point onto the plane(clostestPoint.position, newReferencePoint.normal))
      {
        Vector3 diff = newReferencePoint.position - ((m_params.useAvgNormal && !m_params.useClosestPointForEdges) ? m_avgVertex.position : newClosestPoint.position);
        newReferencePoint.position -= newReferencePoint.normal * diff.dot(newReferencePoint.normal); // smooth it !!
        // this will drift, but will fix a problem for now (todo)
        m_transform.translation -= newReferencePoint.normal * diff.dot(newReferencePoint.normal); // smooth it !!
      }

      if (m_params.freeRotationEnabled) {
        // Look at reference point movement and closest point movement
        Vector3 dRefPt;
        Vector3 dClosestPt;
        if (!m_params.useAvgNormal || m_params.useClosestPointForEdges) {
          dRefPt = newReferencePoint.position - oldReferencePoint.position;
          dRefPt -= m_referencePoint.normal * dRefPt.dot(m_referencePoint.normal);
          dClosestPt = newClosestPoint.position - oldClosestPoint.position;
          dClosestPt -= m_referencePoint.normal * dClosestPt.dot(m_referencePoint.normal);
        } else {
          dRefPt = newReferencePoint.position - oldReferencePoint.position;
          dRefPt -= m_referencePoint.normal * dRefPt.dot(m_referencePoint.normal);  /////////
          dClosestPt = newAvgVertex.position - m_avgVertex.position;
          dClosestPt -= m_referencePoint.normal * dClosestPt.dot(m_referencePoint.normal);
        }

        // Project dCloestPt along dRefPt
        lmReal refDist = dRefPt.norm();
        if (LM_EPSILON < refDist && m_params.clipTranslationOnFreeRotate && !forwardVerticesDetected) { // && directionDot < 0.0f)
          Vector3 projected = dRefPt.dot(dClosestPt) / dRefPt.squaredNorm() * dRefPt ;
          lmReal closestDist = projected.norm();

          // Add camera translation
          lmReal movedFraction = std::min((closestDist+LM_EPSILON)/(refDist+LM_EPSILON), 1.0f);
          lmReal rotationFraction = 1.0f - movedFraction;

          // Use the difference in magnitude to
          // Rotate camera to new position, at the same time: we need to stop updating the normal .....
          //Vector3 correction = ((!params.useAvgNormal || params.useClosestPointForEdges) ? newClosestPoint.position : avgVertex.position) - referencePoint.position; // check that this is (1-rotationFraction) * dRefPt
          Vector3 correction = (-1.0f + movedFraction) * dRefPt;
          {
            std::unique_lock<std::mutex> lock(m_referencePointMutex);
            m_referencePoint.position += correction;;
          }
          m_transform.translation += correction;
#if LM_LOG_CAMERA_LOGIC
          std::cout << "Move fraction: " << movedFraction << std::endl;
#endif
          Vector3 newCameraNormal = (m_transform.translation + m_params.freeRotationRatio * (rotationFraction*(normToCamera*dRefPt)) - m_referencePoint.position).normalized();
          CorrectCameraOrientation(dtZero, newCameraNormal);
        } else {
          // Prevent snapping back to the original positon
//          lmReal rotationFraction = 0.0f;
          Vector3 newCameraNormal = (m_transform.translation - m_referencePoint.position).normalized();
          CorrectCameraOrientation(dtZero, newCameraNormal);
        }
      }

      if (!m_params.freeRotationEnabled && !m_params.suppresForwardRotation) {
        CorrectCameraOrientation(dtZero, m_referencePoint.normal);
      }

      LM_ASSERT(m_referencePoint.normal.squaredNorm() < 2.0f, "Normal exploded.");

      rayHit = true;
      this->m_closestPoint = newClosestPoint;
      this->m_avgVertex = newAvgVertex;
    }
  }

  // Adjust translation to user input.
  if (rayHit) {
    if (m_debugDrawUtil && m_params.drawDebugLines) {
      LM_DRAW_CROSS(m_referencePoint.position, 10.0f, lmColor::WHITE);
      LM_DRAW_ARROW(m_referencePoint.position, m_referencePoint.position + m_referencePoint.normal * 20.0f, lmColor::RED);
    }

    // clean up:
    m_referenceDistance -= deltaAngles.z();
    m_referenceDistance = std::max(m_params.minDist, m_referenceDistance);
    m_referenceDistance = std::min(m_params.maxDist, m_referenceDistance);

    if (!m_params.sphereCrawlMode && !m_params.freeRotationEnabled) {
      CorrectCameraOrientation(dtOne, m_referencePoint.normal);
    }
    CorrectCameraDistance(dt);
  }

  // don't update transform when there's no successful raycast
  if (m_params.pinUpVector) { CorrectCameraUpVector(dtOne, Vector3::UnitY()); }

  if (m_params.preventCameraInMesh) {
    bool validMovement = VerifyCameraMovement(mesh, oldCamera.translation, m_transform.translation, GetSphereQueryRadius());
    if (!validMovement) {
#if LM_LOG_CAMERA_LOGIC_2
      std::cout << "Camera movement clipped." << std::endl;
#endif
      // Disallow camera orientation movement, but allow reference point movement.
      m_transform.translation = oldCamera.translation;
      UpdateCameraOrientationFromPositions();

      m_framesFromLastCollisions = 0;
    }
  }


  // Sync reference point & camera transform
  RealignRefPtAndCamera();

  UpdateCameraInWorldSpace();
}


void CameraUtil::CorrectCameraOrientation(lmReal dt, const Vector3& newNormal) {
  LM_ASSERT(lmIsNormalized(newNormal), "");

  lmReal adjustOrientationRate = 0.0f;
  if (m_params.enableSmoothing && 0.0f < dt) {
    adjustOrientationRate = std::pow(m_params.smoothingFactor, dt);
  }

  // Roation quaternion.
  Vector3 oldZ = m_transform.rotation * Vector3::UnitZ();
  Vector3 newZ = newNormal;

  // Target orientation
  lmQuat dQ; dQ.setFromTwoVectors(oldZ, newZ);

  // Modify camera transform
  dQ = dQ.slerp(adjustOrientationRate, lmQuat::Identity());
  m_transform.rotation = dQ * m_transform.rotation;

  Vector3 cameraFromRefPoint = m_transform.translation - m_referencePoint.position;
  m_transform.translation = m_referencePoint.position + dQ * cameraFromRefPoint;
}

void CameraUtil::CorrectCameraDistance( lmReal dt )
{
  lmReal currentDist = (m_referencePoint.position - m_transform.translation).norm();
  lmReal smoothingValue = std::pow(0.8f, dt);
  lmReal deltaDist = m_referenceDistance - currentDist;
  Vector3 deltaTranslation = (1.0f-smoothingValue) * deltaDist * (m_transform.rotation * Vector3::UnitZ());
  m_transform.translation += deltaTranslation;
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

void CameraUtil::RealignRefPtAndCamera()
{
  lmReal dist = (m_transform.translation - m_referencePoint.position).norm();
  Vector3 cameraNormal = m_transform.rotation * Vector3::UnitZ();
  m_transform.translation = m_referencePoint.position + dist * cameraNormal;
}

void CameraUtil::UpdateCameraOrientationFromPositions()
{
  Vector3 newCameraNormal = m_transform.translation - m_referencePoint.position;
  if (!lmIsZero(newCameraNormal)) {
    newCameraNormal.normalize();
    Vector3 oldCameraNormal = m_transform.rotation * Vector3::UnitZ();
    lmQuat correction; correction.setFromTwoVectors(oldCameraNormal, newCameraNormal);
    m_transform.rotation = correction * m_transform.rotation;

    m_referenceDistance = (m_transform.translation - m_referencePoint.position).norm();

    // Need to clip it though..
    lmClip(m_referenceDistance, m_params.minDist, m_params.maxDist);
  }
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

  //int count[2] = {0,0};

  if (m_params.userFaultyTriangles) {
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

        distSqr = lmClip(distSqr, m_params.grav_k*m_params.grav_k, 1000000.0f*1000000.0f);

        //lmReal distPowered = std::pow(distSqr, (lmReal)params.grav_n/2.0f); // slow?
        //const lmReal weightDenominator = std::pow((lmReal)queryRadius, (lmReal)params.grav_n);

        // avoid calling std::pow
        lmReal distPowered = distSqr;//  std std::pow(distSqr, (lmReal)params.grav_n/2.0f); // slow?
        lmReal weightDenominator = queryRadiusSqr;// std::pow((lmReal)queryRadius, (lmReal)params.grav_n);

        distPowered *= distSqr;
        weightDenominator *= queryRadiusSqr;
        distPowered *= distSqr;
        weightDenominator *= queryRadiusSqr;
#if 0
        if (params.grav_n > 2.0f)
        {
          distPowered *= distSqr;
          weightDenominator *= queryRadiusSqr;
          if (params.grav_n > 4.0f)
          {
            distPowered *= distSqr;
            weightDenominator *= queryRadiusSqr;
          }
        }
#endif

        lmReal weight = 1.0f - (distPowered / weightDenominator);
        weight = lmClip(weight, 0.0f, 1.0f);
        //lmReal area = TriArea(mesh, tri);
        potential += tri.area * weight / distPowered;

#if LM_DRAW_DEBUG_OBJECTS
        if (m_debugDrawUtil && m_params.drawDebugLines && m_params.drawSphereQueryResults) {
          m_debugDrawUtil->DrawTriangle(mesh, tri);
        }
#endif

        //count[1]++;
      }
      //else
      //{
      //  count[0]++;
      //}
    }
    //std::cout << "triangles detailDist / octTreeQueryt: " << count[1] << " / " << count[0]+count[1] << std::endl;

  } else {
    //lmReal minw = 1000000000.0f;
    //lmReal maxw = -1000000000.0f;
    //lmReal mind = 1000000000.0f;
    //lmReal maxd = -1000000000.0f;

    std::vector<int> selectedVertices;
    mesh->getVerticesFromTriangles(m_queryTriangles, selectedVertices);

    const VertexVector& vertices = mesh->getVertices();

    for (unsigned vi = 0; vi < selectedVertices.size(); vi+= striding)
    {
      const Vertex& vert = vertices[selectedVertices[vi]];

      lmReal distSqr = (position-vert).squaredNorm();
      //static const lmReal K = 0.0001f;
      distSqr = lmClip(distSqr, m_params.grav_k*m_params.grav_k, 1000000.0f*1000000.0f);

      if (distSqr < queryRadiusSqr)
      {

        //static const lmReal N = 6.0f;
        //lmReal distPowered = std::pow(distSqr, (lmReal)params.grav_n/2.0f); // slow?
        //const lmReal weightDenominator = std::pow((lmReal)queryRadius, (lmReal)params.grav_n);

        lmReal distPowered = distSqr;// std::pow(distSqr, (lmReal)params.grav_n/2.0f); // slow?
        lmReal weightDenominator = queryRadiusSqr;//std::pow((lmReal)queryRadius, (lmReal)params.grav_n);

        distPowered *= distSqr;
        weightDenominator *= queryRadiusSqr;
        distPowered *= distSqr;
        weightDenominator *= queryRadiusSqr;

        lmReal weight = 1.0f - (distPowered / weightDenominator);
        //minw = std::min(minw, weight);
        //maxw = std::max(maxw, weight);
        //lmReal dist = std::sqrt(distSqr);
        //mind = std::min(mind, dist);
        //maxd = std::max(maxd, dist);
        weight = lmClip(weight, 0.0f, 1.0f);

        potential += weight / distPowered;
        //potential += 1.0 / distPowered;

#if LM_DRAW_DEBUG_OBJECTS
        if (m_params.drawSphereQueryResults) {
          LM_DRAW_CROSS(vert, 3.0f, lmColor::WHITE);
        }
#endif
      }
    }
    //std::cout << "weight min/max: " << minw << " / " << maxw << std::endl;
    //std::cout << "dist min/max/queryr: " << mind << " / " << maxd << " / " << queryRadius << std::endl;
  }

  return potential;
}

void CameraUtil::IsoPotential_row4( Mesh* mesh, const Vector3* positions, lmReal queryRadius, lmReal* potentials )
{
  std::vector<Octree*> &leavesHit = mesh->getLeavesUpdate();
  m_queryTriangles.clear();
  mesh->getOctree()->intersectSphere(positions[0],queryRadius*queryRadius,leavesHit, m_queryTriangles);

  //lmReal radius = Get
  for (int i = 0; i < 4; i++)
  {
    potentials[i] = 0.0f;
  }
  // process every n-th point
  const int striding = 1;
  const lmReal queryRadiusSqr = queryRadius*queryRadius;

  //int count[2] = {0,0};

  if (m_params.userFaultyTriangles) {
    const TriangleVector& triangles = mesh->getTriangles();


    for (unsigned ti = 0; ti < m_queryTriangles.size(); ti+= striding)
    {
      const Triangle& tri = triangles[m_queryTriangles[ti]];

      Geometry::GetClosestPointInput input(mesh, &tri, positions[0]);
      Geometry::GetClosestPointOutput output;
      Geometry::getClosestPoint_noNormal(input, &output);

      lmReal distSqr = output.distanceSqr;

      if (distSqr < queryRadiusSqr)
      {
        distSqr = lmClip(distSqr, m_params.grav_k*m_params.grav_k, 1000000.0f*1000000.0f);

        // avoid calling std::pow
        lmReal distPowered = distSqr;
        lmReal weightDenominator = queryRadiusSqr;

        distPowered *= distSqr;
        weightDenominator *= queryRadiusSqr;
        distPowered *= distSqr;
        weightDenominator *= queryRadiusSqr;
        lmReal weight = 1.0f - (distPowered / weightDenominator);
        weight = lmClip(weight, 0.0f, 1.0f);
        //lmReal area = TriArea(mesh, tri);
        potentials[0] += tri.area * weight / distPowered;

#if LM_DRAW_DEBUG_OBJECTS
        if (m_debugDrawUtil && m_params.drawDebugLines && m_params.drawSphereQueryResults) {
          m_debugDrawUtil->DrawTriangle(mesh, tri);
        }
#endif
        for (int i = 1; i < 4; i++) {
          lmReal distSqr =  (positions[i]-output.position).squaredNorm();
          distSqr = lmClip(distSqr, m_params.grav_k*m_params.grav_k, 1000000.0f*1000000.0f);

          // avoid calling std::pow
          lmReal distPowered = distSqr;
          lmReal weightDenominator = queryRadiusSqr;

          distPowered *= distSqr;
          weightDenominator *= queryRadiusSqr;
          distPowered *= distSqr;
          weightDenominator *= queryRadiusSqr;
          lmReal weight = 1.0f - (distPowered / weightDenominator);
          weight = lmClip(weight, 0.0f, 1.0f);
          //lmReal area = TriArea(mesh, tri);
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
      distSqr = lmClip(distSqr, m_params.grav_k*m_params.grav_k, 1000000.0f*1000000.0f);

      if (distSqr < queryRadiusSqr)
      {
        lmReal distPowered = distSqr;
        lmReal weightDenominator = queryRadiusSqr;

        distPowered *= distSqr;
        weightDenominator *= queryRadiusSqr;
        distPowered *= distSqr;
        weightDenominator *= queryRadiusSqr;

        lmReal weight = 1.0f - (distPowered / weightDenominator);
        weight = lmClip(weight, 0.0f, 1.0f);

        potentials[0] += weight / distPowered;

#if LM_DRAW_DEBUG_OBJECTS
        if (m_params.drawSphereQueryResults) {
          LM_DRAW_CROSS(vert, 3.0f, lmColor::WHITE);
        }
#endif
        for(int i = 1; i < 4; i++) {
          lmReal distSqr = (positions[i]-vert).squaredNorm();
          distSqr = lmClip(distSqr, m_params.grav_k*m_params.grav_k, 1000000.0f*1000000.0f);

          lmReal distPowered = distSqr;
          lmReal weightDenominator = queryRadiusSqr;

          distPowered *= distSqr;
          weightDenominator *= queryRadiusSqr;
          distPowered *= distSqr;
          weightDenominator *= queryRadiusSqr;

          lmReal weight = 1.0f - (distPowered / weightDenominator);
          weight = lmClip(weight, 0.0f, 1.0f);

          potentials[i] += weight / distPowered;

        }
      }
    }
  }
}

Vector3 CameraUtil::IsoNormal( Mesh* mesh, const Vector3& position, lmReal queryRadius )
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
  negNormal.normalize();

  //std::cout << "Iso potential: " << potential << std::endl;

  Vector3 normal((float)-negNormal.x(), (float)-negNormal.y(), (float)-negNormal.z());
  return normal;
}

lmReal CameraUtil::IsoQueryRadius(IsoCameraState* state) const {
  return std::max(state->refDist + m_params.isoQueryPaddingRadius, m_params.isoQueryPaddingRadius);
}

void CameraUtil::IsoUpdateCameraTransform( const Vector3& newDirection, IsoCameraState* state, lmReal deltaTime )
{
  lmQuat qCorrection; qCorrection.setFromTwoVectors(GetCameraDirection(), newDirection);

  //std::unique_lock<std::mutex> lock(mutex);


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

  //transform.rotation = qCorrection * transform.rotation;
  //transform.translation = state->refPosition - state->cameraOffsetMultiplier * state->refDist * newDirection;
}

void CameraUtil::IsoUpdateReferencePoint(IsoCameraState* state ) {
  std::unique_lock<std::mutex> lock(m_referencePointMutex);
  m_referencePoint.position = state->refPosition - state->refNormal * state->refDist;
  m_referencePoint.normal = state->refNormal;
  m_referenceDistance = state->refDist * (1 + m_params.isoRefDistMultiplier);
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
    // Don't know why this fails !?
    LM_ASSERT(raycast.dist <= ray.GetLength(), "Raycast result corrupted.");
    //LM_ASSERT(raycast.dist <= (state->refPosition, transform.translation).norm(), "Camera collision with mesh failed.");

    // if clip this
    lmReal refPositionFraction = std::max(raycast.fraction, m_params.minDist / state->refDist);
    refPositionFraction = lmClip(refPositionFraction, 0.0f, 1.0f);
    state->refPosition -= state->refNormal * state->refDist * (1.0f-refPositionFraction);
    state->refDist *= refPositionFraction;

    //std::unique_lock<std::mutex> lock(mutex);

    m_transform.translation = raycast.position - ray.GetDirection() * 0.01f;
  }
}

void CameraUtil::InitIsoCamera( Mesh* mesh, IsoCameraState* state )
{
  // get field potential from current position
  lmReal t = 0.5f;
  state->refPosition = lmInterpolate(t, m_referencePoint.position, m_transform.translation);
  //state->refPotential = IsoPotential(mesh, state->refPosition);
  state->cameraOffsetMultiplier = 1.0f;

  lmReal queryRadius = 10000.0f; // everything
  state->closestPointOnMesh = GetClosestSurfacePoint(mesh, state->refPosition, queryRadius);
  state->refDist = (state->closestPointOnMesh.position - state->refPosition).norm();

  state->refPotential = IsoPotential(mesh, state->refPosition, IsoQueryRadius(state));

  state->numFailedUpdates = 0;
  // Remember closest point distance

  IsoUpdateReferencePoint(state);
}

void CameraUtil::IsoCamera( Mesh* mesh, IsoCameraState* state, const Vector3& movement, lmReal deltaTime )
{
  m_justSculpted = (m_prevTimeOfLastSculpt != m_timeOfLastScupt);
  if (m_justSculpted) {
    m_timeOfMovementSinceLastMeshMofification = 0.0f;
    m_forceVerifyPositionAfterSculpting = true;
  }

  LM_DRAW_CROSS(state->refPosition, 20.0f, lmColor::GREEN);
  LM_DRAW_CROSS(state->closestPointOnMesh.position, 20.0f, lmColor::RED);
  LM_DRAW_ARROW(state->closestPointOnMesh.position, state->closestPointOnMesh.position + state->closestPointOnMesh.normal * 40.0f, lmColor::RED);

  if (movement.norm() < LM_EPSILON) {
    if (mesh->getRotationVelocity_notSmoothed() > 0.0f) {
      IsoCameraConstrainWhenSpinning(mesh, state);
    }

    IsoUpdateCameraTransform(-state->refNormal, state, deltaTime);
    IsoUpdateReferencePoint(state);
    UpdateCameraInWorldSpace();

    // do nothing.
    return;
  }

  m_timeOfMovementSinceLastMeshMofification += deltaTime;


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
    ResetCamera(mesh, -Vector3::UnitZ());
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
        ResetCamera(mesh, GetCameraDirection());
        state->refPosition = m_referencePoint.position + m_params.minDist * m_referencePoint.normal;
        break;
      }

      raycastOutput.invalidate();
      CastOneRay(mesh, lmRay(m_transform.translation, state->refPosition), &raycastOutput);
    }
  }

  Vector3 oldRefPosition = state->refPosition;
//  lmReal oldRefDist = state->refDist;

  // Dummy & temp: clip z movement
  Vector3 clippedMovement = movement * std::sqrt(std::sqrt(state->refDist / m_params.refDistForMovemement));

  clippedMovement.z() *= m_params.scaleZMovement;

  clippedMovement = m_transform.rotation * clippedMovement;

  // Verify movement
  lmReal radiusForCameraSphereCollision = m_params.minDist;
  //lmReal radiusForCameraSphereCollision = 0.0f;
  bool validMovement = VerifyCameraMovement(mesh, state->refPosition, state->refPosition + clippedMovement, radiusForCameraSphereCollision);
  if (!validMovement && clippedMovement.norm() > LM_EPSILON_SQR) {
#if LM_LOG_CAMERA_LOGIC_4
    std::cout << "Invalid camera movement (collision)." << std::endl;
#endif
    return;
  }

  // Saftey clip movement so that the camera doesn't go past the safety distance.
  if (0) {
    if ((state->refPosition + clippedMovement).norm() > m_params.maxDist) {
      Vector3 newPos = state->refPosition + clippedMovement;
      newPos.normalize();
      newPos *= m_params.maxDist;
      clippedMovement = newPos - state->refPosition;
    }
  } else  {
    if (lmIsNormalized(state->closestPointOnMesh.normal) && ((state->refPosition - state->closestPointOnMesh.position) + clippedMovement).norm() > m_params.maxDist/(1+m_params.isoRefDistMultiplier)*1.5f) {
      // Clip to the closest point
      Vector3 newRelPos = (state->refPosition - state->closestPointOnMesh.position) + clippedMovement;
      newRelPos.normalize();
      newRelPos *= m_params.maxDist/(1+m_params.isoRefDistMultiplier)*1.5f;
      Vector3 newPos = state->closestPointOnMesh.position + newRelPos;
      clippedMovement = newPos - state->refPosition;
    }
  }


  Vector3 newNormal = IsoNormal(mesh, state->refPosition + clippedMovement, IsoQueryRadius(state));
  if (!lmIsNormalized(newNormal)) {
#if LM_LOG_CAMERA_LOGIC_4
    std::cout << "New IsoNormal not found, broadening the search." << std::endl;
#endif
    lmSurfacePoint closestPoint = GetClosestSurfacePoint(mesh, state->refPosition + clippedMovement, 10000.0f);
    if (!lmIsNormalized(closestPoint.normal)) {
#if LM_LOG_CAMERA_LOGIC_4
      std::cout << "Now IsoNormal failed (closest point not found)." << std::endl;
#endif
      return;
    }
    state->refDist = (closestPoint.position - (state->refPosition + clippedMovement)).norm();
    newNormal = IsoNormal(mesh, state->refPosition + clippedMovement, IsoQueryRadius(state));
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
      newNormal = IsoNormal(mesh, state->refPosition + clippedMovement, IsoQueryRadius(state));
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

  // Update closest distance: last check
  lmSurfacePoint closestPoint = GetClosestSurfacePoint(mesh, state->refPosition + clippedMovement, state->refDist + clippedMovement.norm());
  if (!lmIsNormalized(closestPoint.normal)) {
#if LM_LOG_CAMERA_LOGIC_4
    std::cout << "Ref Dist too small for queries" << std::endl;
#endif
    closestPoint = GetClosestSurfacePoint(mesh, state->refPosition + clippedMovement, 10000.0f);
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

  LM_ASSERT(state->refDist < 10000, "Reference distance exploded.")

  IsoPreventCameraInMesh(mesh, state);

  // Display reference sphere (updating point & radius):
  IsoUpdateReferencePoint(state);

  state->numFailedUpdates = 0;
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
  Vector3 rayStart = state->refPosition + state->refNormal * 1000.0f;
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
  lmRay ray0(isoState->refPosition, isoState->refPosition - 10000.0f * isoState->refNormal);
  lmRay ray1(isoState->refPosition, isoState->refPosition + 10000.0f * isoState->refNormal);
  std::vector<lmRayCastOutput> results0;
  std::vector<lmRayCastOutput> results1;
  const bool collectAll = true;
  CastOneRay(mesh, ray0, &results0, collectAll);
  CastOneRay(mesh, ray1, &results1, collectAll);

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
