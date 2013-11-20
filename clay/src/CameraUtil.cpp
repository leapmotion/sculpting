#include "StdAfx.h"
#include "CameraUtil.h"
#include "DebugDrawUtil.h"
#include "Geometry.h"
#include "DataTypes.h"

#define LM_LOG_CAMERA_LOGIC 0
#define LM_LOG_CAMERA_LOGIC_2 0
#define LM_LOG_CAMERA_LOGIC_3 0

CameraUtil::CameraUtil() {
  transform.setIdentity();
  meshTransform.setIdentity();
  referencePoint.position.setZero();
  referencePoint.normal.setZero(); // invalid
  referenceDistance = 0.0f;
  userInput.setZero();
  accumulatedUserInput.setZero();
  state = STATE_INVALID;
  debugDrawUtil = NULL;
  lastUserInputFromVectorTime = -1.0f;
  lastCameraUpdateTime = -1.0f;
  avgVertex.position.setZero();
  avgVertex.normal.setZero();
  framesFromLastCollisions = 1000;
}

void CameraUtil::SetFromStandardCamera(const Vector3& from, const Vector3& to, lmReal referenceDistance) {
  std::unique_lock<std::mutex> lock(mutex);

  GetTransformFromStandardCamera(from, to, transform);

  this->referenceDistance = referenceDistance;
  state = STATE_FREEFLOATING;
}

void CameraUtil::ResetCamera(const Mesh* mesh, const Vector3& cameraDirection) {
  const VertexVector& vertices = mesh->getVertices();
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
  const Vertex& closest = vertices[idx];
  referencePoint.position = closest;
  referencePoint.normal = closest.normal_;

  // Rotate camera to the new requested direction
  Vector3 currentNegZ = -1.0f * (transform.rotation * Vector3::UnitZ());
  lmQuat correction; correction.setFromTwoVectors(currentNegZ, cameraDirection);
  transform.rotation = correction * transform.rotation;

  // Set camera position
  transform.translation = referencePoint.position + referenceDistance * (transform.rotation * Vector3::UnitZ());
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

void CameraUtil::GenerateRays(const lmTransform& transform, int castsPerRow, std::vector<lmRay>* raysOut) {
  // Generate several rays.
  std::vector<Vector3, Eigen::aligned_allocator<Vector3> > rays;
  // Vector3Vector
  for (int i = 0; i < castsPerRow*castsPerRow; i++) {
    rays.push_back(Vector3((-(castsPerRow-1)/2.0f+i%castsPerRow)*4.0f,(-(castsPerRow-1)/2.0f+i/castsPerRow)*4.0f,-100) * 10);
  }
  // transform ray targets
  for (size_t i = 0; i < rays.size(); i++) {
    rays[i] = transform.rotation * rays[i] + transform.translation;
    lmRay ray(transform.translation, rays[i]);
    raysOut->push_back(ray);
  }
  // TODO fix debug lines
  if (0 && debugDrawUtil) {
    std::unique_lock<std::mutex> lock(debugDrawUtil->m_mutex);
    for (size_t ri = 0; ri < rays.size(); ri++) {
      LM_DRAW_POINT(rays[ri], lmColor::CYAN);
    }
  }
}

void CameraUtil::CastRays(const Mesh* mesh, const std::vector<lmRay>& rays, std::vector<lmRayCastOutput>* results) {
  for (size_t ri = 0; ri < rays.size(); ri++) {
    const lmRay& ray = rays[ri];
    std::vector<int> triangles = mesh->getOctree()->intersectRay(ray.start, ray.GetDirection());

    int numAabbSuccess = 0;
    int numTriSuccess = 0;

    std::vector<int> tris2; 
    lmReal minDist = FLT_MAX;
    int minDistIdx = -1;
    lmRayCastOutput rayCastOutput;
    // 0. cast ray for each triangle's aabb
    for (size_t ti = 0; ti < triangles.size(); ti++) {
      const Triangle& tri = mesh->getTriangle(triangles[ti]);
      Vector3 rayOnPlane;
      bool result = tri.aabb_.intersectRay(ray.start, ray.GetDirection());
      if (result) {
        numAabbSuccess++;
        result = Geometry::intersectionRayTriangle(
          ray.start, ray.end, 
          mesh->getVertex(tri.vIndices_[0]),
          mesh->getVertex(tri.vIndices_[1]),
          mesh->getVertex(tri.vIndices_[2]),
          tri.normal_, rayOnPlane);
      }
      if (result) {
        numTriSuccess++;
        //if (tri.aabb_.intersectRay(f, dir)) {
        tris2.push_back(triangles[ti]);
        lmReal dist = (rayOnPlane-ray.start).dot(ray.GetDirection());
        if (dist < minDist) {
          minDistIdx = triangles[ti];
          minDist = dist;

          rayCastOutput.triangleIdx = minDistIdx;
          rayCastOutput.position = rayOnPlane;
          rayCastOutput.normal = tri.normal_;
          rayCastOutput.dist = minDist;
          rayCastOutput.fraction = minDist / ray.GetLength();
        }
      }
    }

    const bool drawClosestOnly = true;
    if (drawClosestOnly) {
      triangles.clear(); 
      results->push_back(rayCastOutput);
      if (minDistIdx != -1) {
        // for debug display only
        triangles.push_back(minDistIdx);
      }
    } else {
      triangles = tris2; // fix: append
    }

    //// Display debug triangles
    //if (debugDrawUtil) {
    //  std::unique_lock<std::mutex> lock(debugDrawUtil->m_mutex);
    //  for (size_t ti = 0; ti < triangles.size(); ti++) {
    //    const Triangle& tri = mesh->getTriangle(triangles[ti]);
    //    debugDrawUtil->DrawTriangle(mesh, tri);
    //  }
    //}
  }
}

inline static lmReal TriArea(const Mesh* mesh, const Triangle& tri) {
  const Vertex& v0 = mesh->getVertex(tri.vIndices_[0]);
  const Vertex& v1 = mesh->getVertex(tri.vIndices_[1]);
  const Vertex& v2 = mesh->getVertex(tri.vIndices_[2]);

  const lmReal area = 0.5f * (v1-v0).cross(v2-v0).norm();
  return area;
}

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
  closestPoint.distance = FLT_MAX;
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
        if (output.distance < closestPoint.distance) {
          closestPoint = output;
          closestPoint.normal = tri.normal_;
          closestTriangleIdx = tris[ti];
        }
      }
    }
  }

  if (closestPoint.distance < FLT_MAX) {
    GetNormalAtPoint(mesh, closestTriangleIdx, closestPoint.position, &closestPoint.normal);
  }

  *closestPointOut = closestPoint;
}

lmSurfacePoint CameraUtil::GetClosestSurfacePoint(Mesh* mesh, const Vector3& position, lmReal queryRadius) {
  std::vector<Octree*> &leavesHit = mesh->getLeavesUpdate();
  std::vector<int> tris = mesh->getOctree()->intersectSphere(position, queryRadius*queryRadius, leavesHit);

  // Get triangles
  Geometry::GetClosestPointOutput closestPoint;
  closestPoint.distance = FLT_MAX;

  for (size_t ti = 0; ti < tris.size(); ti++) {
    const Triangle& tri = mesh->getTriangle(tris[ti]);

    Geometry::GetClosestPointOutput output;
    Geometry::GetClosestPointInput input(mesh, &tri, position);
    Geometry::getClosestPoint(input, &output);
    output.triIdx = tris[ti];
    if (output.distance < closestPoint.distance) {
      closestPoint = output;
    }
  }

  if (closestPoint.distance < FLT_MAX) {
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

  if (params.userFaultyTriangles) {
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
        lmReal a = TriArea(mesh, tri);

        Geometry::GetClosestPointOutput output;
        {
          Geometry::GetClosestPointInput input;
          input.mesh = mesh;
          input.tri = &tri;
          input.point = referencePoint.position;
          Geometry::getClosestPoint(input, &output);
          output.triIdx = tris[ti];
          if (output.distance < closestPoint.distance) {
            closestPoint = output;
          }

          if (debugDrawUtil && params.drawDebugLines && params.drawSphereQueryResults) {
            debugDrawUtil->DrawTriangle(mesh, tri);
          }
        }

        // Calc point weight
        lmReal t = closestPoint.distance / radius;
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

        if (debugDrawUtil && params.drawDebugLines && params.drawSphereQueryResults) {
          LM_DRAW_CROSS(vert, 5.0f, lmColor::WHITE);
        }
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
  std::unique_lock<std::mutex> lock(mutex);
  Vector3 movement(50.0f * _DTheta, -50.0f * _DPhi, -_DFov / 100.0f);
  userInput += (params.invertCameraInput?-1.0f:1.0f) * params.inputMultiplier * movement;
}

#if 0
void CameraUtil::RecordUserInput(const Vector3& deltaPosition, bool controlOn) {
  std::unique_lock<std::mutex> lock(mutex);
  lmReal prevTime = lastUserInputFromVectorTime;
  lmReal time = lmReal(ci::app::getElapsedSeconds());
  lastUserInputFromVectorTime = time;
  if (prevTime < 0.0001f) { prevTime = time; }
  lmReal deltaTime = time-prevTime;

  userInput += deltaPosition;

  if (!controlOn) {
    // fade of velocity
    userInputVelocity = userInputVelocity * 0.9f; // plus constant subtract fro quick 
    LM_LOG << "User input velocity [" << userInputVelocity.x() << ", " << userInputVelocity.y() << ", " << userInputVelocity.z() << "]" << std::endl;

    // fading should be actually along the squared curve
    lmReal newSpeed = std::sqrt(std::max(0.0f, userInputVelocity.squaredNorm() - 1.0f));
    userInputVelocity *= (newSpeed / (userInputVelocity.squaredNorm() + 0.00001f));

    userInput.setZero();
    accumulatedUserInput.setZero();

    userInput = userInputVelocity * deltaTime;
  }

  if (controlOn) {
    if (prevTime < 0.0001f) {
      userInputVelocity.setZero();
    } else {
      userInputVelocity = deltaPosition / deltaTime;
    }
  }
}
#endif

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

void CameraUtil::DebugDrawNormals(const Mesh* mesh, const Params& paramsIn) {
  // Draw normal for every nth triangle
  TriangleVector trangles = const_cast<Mesh*>(mesh)->getTriangles();
  VertexVector vertices = const_cast<Mesh*>(mesh)->getVertices();

  for (size_t i = 0; i < vertices.size(); i+= 1) {
    Vertex vert = vertices[i];

    lmSurfacePoint refPoint(vert, vert.normal_);
    lmSurfacePoint newAvgVertex;
    lmSurfacePoint pureNewAvgVertex;
    Geometry::GetClosestPointOutput newClosestPoint;
    GetAveragedSurfaceNormal(mesh, refPoint, GetSphereQueryRadius(), -1.0f * vert.normal_, paramsIn.weightNormals, &newAvgVertex, &pureNewAvgVertex, &newClosestPoint);

    if(debugDrawUtil) {
      LM_DRAW_ARROW(vert, vert + newAvgVertex.normal * 10.0f, lmColor::WHITE);
    }
  }

  for (size_t i = 0; i < trangles.size(); i++) {
    Triangle tri = trangles[i];
  }
}

void CameraUtil::ExperimentWithIsosurfaces(const Mesh* mesh, Params* paramsInOut) {
  lmReal pmin = FLT_MAX;
  lmReal pmax = 0.0f;

  for (int i = 0; i < 1000; i++) {
    lmReal min = -200.0f;
    lmReal max = 200.0f;

    lmReal x = i/1%10 * 0.1f * (max-min) + min;
    lmReal y = i/10%10 * 0.1f * (max-min) + min;
    lmReal z = i/100%10 * 0.1f * (max-min) + min;

    Vector3 p(x, y, z);
    if (LM_EPSILON * LM_EPSILON < p.squaredNorm()) {
      //LM_PERM_ARROW(p, p + p.normalized() * 20.0f, lmColor::CYAN);
      LM_DRAW_CROSS(p, 3.0f, lmColor::YELLOW);
    }

    if (1) {
      std::vector<int> verts;
      const_cast<Mesh*>(mesh)->getVerticesInsideSphere(p, 50.0f, verts);
      std::vector<int> tris;
      const_cast<Mesh*>(mesh)->getTrianglesFromVertices(verts, tris);

      lmReal eps = 0.5f;
      Vector3 epsXyz[] = { Vector3::Zero(), Vector3::UnitX() * eps, Vector3::UnitY() * eps, Vector3::UnitZ() * eps };
      lmReal potentialXyz[] = { 0.0f, 0.0f, 0.0f, 0.0f };

      for (size_t ti = 0; ti < tris.size(); ti++) {
        const Triangle& tri = mesh->getTriangle(tris[ti]);
        Geometry::GetClosestPointInput input;
        input.mesh = mesh;
        input.tri = &tri;

        for (int ei = 0; ei < 4; ei++) {
          input.point = p + epsXyz[ei];
          Geometry::GetClosestPointOutput output;
          Geometry::getClosestPoint(input, &output);
          lmReal d = output.distance / paramsInOut->isoMultiplier;
          potentialXyz[ei] += 1.0f / (d*d) * TriArea(mesh, tri);
          pmin = std::min(pmin, potentialXyz[ei]);
          pmax = std::max(pmax, potentialXyz[ei]);
        }
      }

      lmReal potential = potentialXyz[0];
      lmReal x = potentialXyz[1] - potential;
      lmReal y = potentialXyz[2] - potential;
      lmReal z = potentialXyz[3] - potential;

      if (tris.size()){
        LM_DRAW_CROSS(p, 3.0f, lmColor::YELLOW);
      }

      Vector3 n(x, y, z);
      if (LM_EPSILON * LM_EPSILON < n.squaredNorm()) {
        n.normalize();
        LM_DRAW_ARROW(p, p + n * 20.0f, lmColor::CYAN);
      } else {
        int hadTriangles = tris.size();
        int i = 0;
      }
    }
  }

#if LM_LOG_CAMERA_LOGIC_3
  std::cout << "min potential " << pmin << std::endl;
  std::cout << "max potential " << pmax << std::endl;
#endif
}

// Used to collide the camera sphere. Figure out what radius we need ?? maybe same as refence sphere
// Returns true if the sphere collides with the mesh
bool CameraUtil::CollideCameraSphere(Mesh* mesh, const Vector3& position, lmReal radius )
{
  // Get potential triangles from the aabb octree.
  std::vector<Octree*> &leavesHit = mesh->getLeavesUpdate();
  std::vector<int> iTrisInCells = mesh->getOctree()->intersectSphere(position,radius*radius,leavesHit);

  // Collide each potential triangle; find real collisions.
  std::vector<Geometry::GetClosestPointOutput> collidingTriangles;
  for (unsigned ti = 0; ti < iTrisInCells.size(); ti++) {
    int triIdx = iTrisInCells[ti];
    const Triangle& tri = mesh->getTriangle(triIdx);

    Geometry::GetClosestPointInput input(mesh, &tri, position);
    Geometry::GetClosestPointOutput output;
    Geometry::getClosestPoint(input, &output);

    if (output.distance < radius) {
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

Vector3 CameraUtil::ToMeshSpace(const Vector3& v) { return meshTransform.rotation.inverse() * v; }

Vector3 CameraUtil::ToWorldSpace(const Vector3& v) { return meshTransform.rotation * v; }

Vector3 CameraUtil::GetCameraDirection() const { return -1.0f * (transform.rotation * Vector3::UnitZ()); };

void CameraUtil::UpdateMeshTransform(const Mesh* mesh, Params* paramsInOut ) {
  transform.rotation = meshTransform.rotation * transform.rotation;
  transform.translation = ToWorldSpace(transform.translation);
  referencePoint.position = ToWorldSpace(referencePoint.position);
  referencePoint.normal = ToWorldSpace(referencePoint.normal);

  isoState.refPosition = ToWorldSpace(isoState.refPosition);
  isoState.refNormal = ToWorldSpace(isoState.refNormal);

  // Get mesh's transformstion
  meshTransform = lmTransformFromMatrix(mesh->getRotationMatrix(), mesh->getTranslation());

  transform.rotation = meshTransform.rotation.inverse() * transform.rotation;
  transform.translation = ToMeshSpace(transform.translation);
  referencePoint.position = ToMeshSpace(referencePoint.position);
  referencePoint.normal = ToMeshSpace(referencePoint.normal);

  isoState.refPosition = ToMeshSpace(isoState.refPosition);
  isoState.refNormal = ToMeshSpace(isoState.refNormal);
}

void CameraUtil::EnsureReferencePointIsCloseToMesh(const Mesh* mesh, Params* paramsInOut) {
  const lmReal radius = GetSphereQueryRadius();
  std::vector<int> verts;
  const_cast<Mesh*>(mesh)->getVerticesInsideSphere(referencePoint.position, radius*radius, *&verts);
  if (!verts.size() && params.enableCameraReset) {
    ResetCamera(mesh, GetCameraDirection());
  }
}

static Vector3 lmProjectAlongVec(const Vector3& in, const Vector3& projectionDirection) {
  return in - projectionDirection * in.dot(projectionDirection);
}

void CameraUtil::OrbitCamera( const Mesh* mesh, lmReal deltaTime ) {

  // Horizontal rotate
  static const lmReal ORBIT_RATE = LM_2PI / 40.0f;
  lmQuat q(AngleAxis( - ORBIT_RATE * deltaTime, Vector3::UnitY()));
  transform.mul(q);
  referencePoint.mul(q);

#if 0
  {
    // Vertical correction & oscillation
    static lmReal phase = 0.0f;
    static lmReal oscillationRate = 1.0f / 20.0f;
    phase += LM_2PI * oscillationRate * deltaTime;
    lmReal angleFromYPlane = std::cos(phase);

    {
      Vector3 refPtDir = referencePoint.position.normalized(); refPtDir.y() = 0.0f;
      if (lmIsZero(refPtDir)) { refPtDir = Vector3::UnitX(); }
      refPtDir.normalize();
      Vector3 newRefPtDir = std::cos(angleFromYPlane) * refPtDir + std::sin(angleFromYPlane) * Vector3::UnitY();
      newRefPtDir.normalize();
      newRefPtDir *= referencePoint.position.norm();
      referencePoint.position = newRefPtDir;
    }

    Vector3 camDir = GetCameraDirection(); camDir.y() = 0.0f;
    if (lmIsZero(camDir)) { camDir = Vector3::UnitX(); }
    camDir.normalize();
    Vector3 newCamDir = std::cos(angleFromYPlane) * camDir - std::sin(angleFromYPlane) * Vector3::UnitY();
    newCamDir.normalize();

    lmQuat correction; correction.setFromTwoVectors(GetCameraDirection(), newCamDir);
    transform.rotation = correction * transform.rotation;
    transform.translation = referencePoint.position - GetCameraDirection() * referenceDistance;
    referencePoint.normal = correction * referencePoint.normal;
  }
#endif

  // Orbit camera with raycast
  if (1) {
    // Camera position in horizontal plane
    Vector3 camXZ = lmProjectAlongVec(transform.translation, Vector3::UnitY());
    // Rotation around Y
    lmQuat q0 = lmQuat::Identity();
    if (!lmIsZero(camXZ)) {
      q0.setFromTwoVectors(Vector3::UnitZ(), camXZ.normalized()); q0.normalize();
    }

    // Rotation around X
    Vector3 camInYZ = q0.inverse() * transform.translation; camInYZ.normalize();
    lmQuat q1Ref; q1Ref.setFromTwoVectors(Vector3::UnitZ(), camInYZ);

    // Oscillating vertical rotation
    static lmReal phase = 0.0f;
    const static lmReal OSCILLATION_RATE = 1.0f / 20.0f;
    phase += LM_2PI * OSCILLATION_RATE * deltaTime;
    //lmReal angleFromYPlane = std::cos(phase);
    lmQuat q1(AngleAxis(std::cos(phase) * 30.0f * LM_DEG, Vector3::UnitX()));

    //// Blend vertical camera movemnt
    //q1.slerp(1.0f - (0.1f * deltaTime), q1Ref);q1.normalize();

    // Desired camera direction
    lmQuat q = q0 * q1; q.normalize();

    lmReal refPtDistFromOrigin = referencePoint.position.norm();
    // Do raycast
    if (mesh) {
      Vector3 rayStart = (q * Vector3::UnitZ()) * params.maxDist;
      std::vector<lmRay> rays; rays.push_back(lmRay(rayStart, Vector3::Zero()));
      std::vector<lmRayCastOutput> hits;
      CastRays(mesh, rays, &hits);
      // if raycast hit (move last reference point, remember last point)
      if (hits[0].isSuccess()) {
        lmReal distFromOrigin = hits[0].position.norm();
        if (distFromOrigin + params.minDist > refPtDistFromOrigin + referenceDistance ) {
          referenceDistance = distFromOrigin + params.minDist - refPtDistFromOrigin;
        }

        //// Update reference point.
        //refPtDistFromOrigin = hits[0].position.norm();
      }
    }

    // Fix reference point
    referencePoint.position = (q * Vector3::UnitZ()) * refPtDistFromOrigin;
    referencePoint.normal = q * Vector3::UnitZ();
    // Fix cameara direction & position
    transform.rotation = q;
    transform.translation = (q * Vector3::UnitZ()) * (refPtDistFromOrigin + referenceDistance);
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

  LM_ASSERT(mesh, "Can't upate the camera without a mesh");
  UpdateMeshTransform(mesh, paramsInOut);

  // Check time
  lmReal dt = 0.0f;
  {
    lmReal prevTime = lastCameraUpdateTime;
    lmReal time = lmReal(ci::app::getElapsedSeconds());
    lastCameraUpdateTime = time;
    if (prevTime < 0.0f) { prevTime = time; }
    dt = time - prevTime;
  }

  std::unique_lock<std::mutex> lock(mutex);
  if (!this->params.walkSmoothedNormals && paramsInOut->walkSmoothedNormals)
  {
    UpdateParamsToWalkSmoothedNormals(paramsInOut);
  }

  if (!this->params.cameraOverrideIso && paramsInOut->cameraOverrideIso) {
    InitIsoCamera(mesh, &isoState);
    paramsInOut->userFaultyTriangles = false;
    paramsInOut->minDist = 10.0f;
    paramsInOut->maxDist = 400.0f;
  }
  this->params = *paramsInOut;

  static const Mesh* prevMesh = NULL;
  if (mesh != prevMesh) {
    // Init camera, and all
    EnsureReferencePointIsCloseToMesh(mesh, paramsInOut);
    InitIsoCamera(mesh, &isoState);
  }
  prevMesh = mesh;

  framesFromLastCollisions++;

  if (params.forceCameraOrbit && params.enableCameraOrbit) {
    OrbitCamera(mesh, dt);
    return;
  } else if (params.enableCameraReset) {
    // Don't do this when orbiting.
    EnsureReferencePointIsCloseToMesh(mesh, paramsInOut);
  }

  //DebugDrawNormals(mesh, paramsIn);
  //ExperimentWithIsosurfaces(mesh, paramsInOut);

  if (debugDrawUtil) {
    if (params.drawDebugLines) {
      LM_DRAW_ARROW(Vector3::Zero(), 50.0f * ToMeshSpace(Vector3::UnitX()), lmColor::RED );
      LM_DRAW_ARROW(Vector3::Zero(), 50.0f * ToMeshSpace(Vector3::UnitY()), lmColor::GREEN );
      LM_DRAW_ARROW(Vector3::Zero(), 50.0f * ToMeshSpace(Vector3::UnitZ()), lmColor::BLUE );
      LM_DRAW_ARROW(transform.translation, referencePoint.position, lmColor::YELLOW );
    }
    debugDrawUtil->SwitchBuffers();
  }

  lmReal dtOne = 1.0f;
  lmReal dtZero = 0.0f;
  if(!params.enableSmoothing) {
    dt = dtOne;
  }

  // Accumulate userInput in 2d
  if (params.enableSmoothing) {
    accumulatedUserInput += userInput;
    lmReal smoothingValue = pow(params.smoothingFactor, dt);
    userInput = (1.0f - smoothingValue) * accumulatedUserInput;
    accumulatedUserInput -= userInput;
  }

  // Multiply motion by distance:
  const lmReal distFraction = (referenceDistance-params.minDist)/(params.maxDist-params.minDist);
  const lmReal movementRatio = params.speedAtMinDist + distFraction * (params.speedAtMaxDist-params.speedAtMinDist);

  // Process smoothed user input
  Vector3 deltaAngles = userInput * movementRatio; 
  Vector3 movementInCam = userInput * movementRatio; movementInCam.z() = 0.0f;
  Vector3 movementInWorld = - userInput * movementRatio;
  movementInCam = -1.0f * (transform.rotation * userInput);
  userInput.setZero();

  const Vector3 oldOldCameraNormal = transform.rotation * Vector3::UnitZ();

  if (100000000.0f <= movementInCam.squaredNorm()) {
    // This actually happened when stopping the Leap Service, and starting a different version.
    accumulatedUserInput.setZero();
    userInput.setZero();
    movementInCam.setZero();
    deltaAngles.setZero();
  }

  if (params.cameraOverrideIso) {
    isoState.cameraOffsetMultiplier = params.isoRefDistMultiplier;
    IsoCamera(mesh, &isoState, movementInWorld);
    if (params.pinUpVector) { CorrectCameraUpVector(dt, Vector3::UnitY()); }
    return;
  }

  // Attempt to calculate new refernce point and normal
  //
  bool rayHit = false;
  lmReal refDist = 0.0f;
  const lmTransform oldCamera = this->transform;

  if (params.sphereCrawlMode) {
    int attampts = 0;

    const Geometry::GetClosestPointOutput oldClosestPoint = this->closestPoint;
    Geometry::GetClosestPointOutput& newClosestPoint = this->closestPoint;
    const lmSurfacePoint oldReferencePoint = this->referencePoint;
    lmSurfacePoint& newReferencePoint = this->referencePoint;
    const lmTransform oldCamera = this->transform;
    lmTransform& newCamera = this->transform;
    const lmSurfacePoint oldAvgVertex = this->avgVertex;


    LM_ASSERT(oldReferencePoint.normal.squaredNorm() == 0.0f || lmIsNormalized(oldReferencePoint.normal), "");
    LM_ASSERT(lmIsNormalized(oldCamera.rotation * Vector3::UnitZ()), "");

    if (debugDrawUtil) {
      LM_DRAW_ARROW(oldReferencePoint.position, oldReferencePoint.position + oldReferencePoint.normal * 50.0f, lmColor::RED);
    }

    // backwards normal camera snapping.
    bool backSnapped = false;
    if (!params.useIsoNormal && params.enableBackSnapping &&  LM_EPSILON * LM_EPSILON < oldClosestPoint.normal.squaredNorm() &&
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

          if (!params.enableForwardCheckForBackSnapping || !forwardVerticesDetected) {
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
    if (!params.useIsoNormal && !backSnapped && framesFromLastCollisions > 2 && params.enableNormalCorrection &&  LM_EPSILON * LM_EPSILON < oldReferencePoint.normal.squaredNorm())
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
          lmReal maxCameraTranslation = (oldReferencePoint.normal - oldCameraNormal).norm() * referenceDistance / params.freeRotationRatio;
          lmReal parallelNorm = parallelComponent.norm();

          if (LM_EPSILON < parallelNorm) {
            lmReal translationRatio = std::min(1.0f, maxCameraTranslation / parallelNorm);
            lmReal leftover = 1.0f - translationRatio;

            Vector3 newCameraTransform = transform.translation + translationRatio * parallelComponent * params.freeRotationRatio ;

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
    if (params.moveInNormalPlane && oldReferencePoint.normal.squaredNorm() != 0.0f)
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
      GetAveragedSurfaceNormal(mesh, newReferencePoint, sphereRadius, cameraDirection, params.weightNormals, &newAvgVertex, &pureNewAvgVertex, &newClosestPoint);
      if (!lmIsNormalized(newClosestPoint.normal)) {
        GetClosestPoint(mesh, newReferencePoint, sphereRadius, cameraDirection, &newClosestPoint);
      }

      newReferencePoint.normal = params.useAvgNormal ? newAvgVertex.normal : newClosestPoint.normal;

      if (params.useIsoNormal) {
        Vector3 queryPosition = (referencePoint.position + transform.translation) * 0.5f;
        Vector3 normal = IsoNormal(mesh, queryPosition, GetSphereQueryRadius());
      }

      normalOkay = lmIsNormalized(newReferencePoint.normal);

      if (normalOkay && params.walkSmoothedNormals) {
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

      if (normalOkay && params.overrideNormal) {
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

      if (!params.useIsoNormal &&params.suppresForwardRotation && lmIsNormalized(oldCalcPoint.normal) && LM_EPSILON * LM_EPSILON < movementInCam.squaredNorm()) {
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
          if (params.tmpSwitch || forwardVerticesDetected) {
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
        Vector3 diff = newReferencePoint.position - ((params.useAvgNormal && !params.useClosestPointForEdges) ? avgVertex.position : newClosestPoint.position);
        newReferencePoint.position -= newReferencePoint.normal * diff.dot(newReferencePoint.normal); // smooth it !!
        // this will drift, but will fix a problem for now (todo)
        transform.translation -= newReferencePoint.normal * diff.dot(newReferencePoint.normal); // smooth it !!
      }

      if (params.freeRotationEnabled) {
        // Look at reference point movement and closest point movement
        Vector3 dRefPt;
        Vector3 dClosestPt;
        if (!params.useAvgNormal || params.useClosestPointForEdges) {
          dRefPt = newReferencePoint.position - oldReferencePoint.position;
          dRefPt -= referencePoint.normal * dRefPt.dot(referencePoint.normal);
          dClosestPt = newClosestPoint.position - oldClosestPoint.position;
          dClosestPt -= referencePoint.normal * dClosestPt.dot(referencePoint.normal);
        } else {
          dRefPt = newReferencePoint.position - oldReferencePoint.position;
          dRefPt -= referencePoint.normal * dRefPt.dot(referencePoint.normal);  /////////
          dClosestPt = newAvgVertex.position - avgVertex.position;
          dClosestPt -= referencePoint.normal * dClosestPt.dot(referencePoint.normal);
        }

        // Project dCloestPt along dRefPt
        lmReal refDist = dRefPt.norm();
        if (LM_EPSILON < refDist && params.clipTranslationOnFreeRotate && !forwardVerticesDetected) { // && directionDot < 0.0f)
          Vector3 projected = dRefPt.dot(dClosestPt) / dRefPt.squaredNorm() * dRefPt ;
          lmReal closestDist = projected.norm();

          // Add camera translation
          lmReal movedFraction = std::min((closestDist+LM_EPSILON)/(refDist+LM_EPSILON), 1.0f);
          lmReal rotationFraction = 1.0f - movedFraction;

          // Use the difference in magnitude to
          // Rotate camera to new position, at the same time: we need to stop updating the normal .....
          //Vector3 correction = ((!params.useAvgNormal || params.useClosestPointForEdges) ? newClosestPoint.position : avgVertex.position) - referencePoint.position; // check that this is (1-rotationFraction) * dRefPt
          Vector3 correction = (-1.0f + movedFraction) * dRefPt;
          referencePoint.position += correction;;
          transform.translation += correction;
#if LM_LOG_CAMERA_LOGIC
          std::cout << "Move fraction: " << movedFraction << std::endl;
#endif
          Vector3 newCameraNormal = (transform.translation + params.freeRotationRatio * (rotationFraction*(normToCamera*dRefPt)) - referencePoint.position).normalized();
          CorrectCameraOrientation(dtZero, newCameraNormal);
        } else {
          // Prevent snapping back to the original positon
          lmReal rotationFraction = 0.0f;
          Vector3 newCameraNormal = (transform.translation - referencePoint.position).normalized();
          CorrectCameraOrientation(dtZero, newCameraNormal);
        }
      }

      if (!params.freeRotationEnabled && !params.suppresForwardRotation) {
        CorrectCameraOrientation(dtZero, referencePoint.normal);
      }

      LM_ASSERT(referencePoint.normal.squaredNorm() < 2.0f, "Normal exploded.");

      rayHit = true;
      this->closestPoint = newClosestPoint;
      this->avgVertex = newAvgVertex;
    }
  }

  // Adjust translation to user input.
  if (rayHit) {
    if (debugDrawUtil && params.drawDebugLines) {
      LM_DRAW_CROSS(referencePoint.position, 10.0f, lmColor::WHITE);
      LM_DRAW_ARROW(referencePoint.position, referencePoint.position + referencePoint.normal * 20.0f, lmColor::RED);
    }

    // clean up:
    referenceDistance -= deltaAngles.z();
    referenceDistance = std::max(params.minDist, referenceDistance);
    referenceDistance = std::min(params.maxDist, referenceDistance);

    if (!params.sphereCrawlMode && !params.freeRotationEnabled) {
      CorrectCameraOrientation(dtOne, referencePoint.normal);
    }
    CorrectCameraDistance(dt);
  }

  // don't update transform when there's no successful raycast
  if (params.pinUpVector) { CorrectCameraUpVector(dt, Vector3::UnitY()); }

  if (params.preventCameraInMesh) {
    bool validMovement = VerifyCameraMovement(mesh, oldCamera.translation, transform.translation, GetSphereQueryRadius());
    if (!validMovement) {
#if LM_LOG_CAMERA_LOGIC_2
      std::cout << "Camera movement clipped." << std::endl;
#endif
      // Disallow camera orientation movement, but allow reference point movement.
      transform.translation = oldCamera.translation;
      UpdateCameraOrientationFromPositions();

      framesFromLastCollisions = 0;
    }
  }


  // Sync reference point & camera transform
  RealignRefPtAndCamera();
}


void CameraUtil::CorrectCameraOrientation(lmReal dt, const Vector3& newNormal) {
  LM_ASSERT(lmIsNormalized(newNormal), "");

  lmReal adjustOrientationRate = 0.0f;
  if (params.enableSmoothing && 0.0f < dt) {
    adjustOrientationRate = std::pow(params.smoothingFactor, dt);
  }

  // Roation quaternion.
  Vector3 oldZ = transform.rotation * Vector3::UnitZ();
  Vector3 newZ = newNormal;

  // Target orientation
  lmQuat dQ; dQ.setFromTwoVectors(oldZ, newZ);

  // Modify camera transform
  dQ = dQ.slerp(adjustOrientationRate, lmQuat::Identity());
  transform.rotation = dQ * transform.rotation;

  Vector3 cameraFromRefPoint = transform.translation - referencePoint.position;
  transform.translation = referencePoint.position + dQ * cameraFromRefPoint;
}

void CameraUtil::CorrectCameraDistance( lmReal dt )
{
  lmReal currentDist = (referencePoint.position - transform.translation).norm();
  lmReal smoothingValue = std::pow(0.8f, dt);
  lmReal deltaDist = referenceDistance - currentDist;
  Vector3 deltaTranslation = (1.0f-smoothingValue) * deltaDist * (transform.rotation * Vector3::UnitZ());
  transform.translation += deltaTranslation;
}

void CameraUtil::CorrectCameraUpVector(lmReal dt, const Vector3& up) {

  // Camera's xy-plane normal
  Vector3 xyPlaneNormal = transform.rotation * Vector3::UnitZ();

  // Take the up vector, and project it onto camera's xy-plane
  const lmReal xyPlaneNormalDotUp = xyPlaneNormal.dot(up);
  Vector3 newUp = up - xyPlaneNormal * xyPlaneNormalDotUp;

  // Disable correction, if camera is facing vertically or horizontally. Hard limit.
  if (std::fabs(xyPlaneNormalDotUp) < 0.9) {

    // Calc correction rotation and rate.
    Vector3 oldUp = transform.rotation * Vector3::UnitY();
    lmQuat dQ; dQ.setFromTwoVectors(oldUp, newUp);
    lmReal adjustOrientationRate = std::pow(0.9f, dt);

    // Apply correction.
    dQ = dQ.slerp(adjustOrientationRate, lmQuat::Identity());
    transform.rotation = dQ * transform.rotation;
  }
}

lmTransform CameraUtil::GetCameraInWorldSpace()
{
  lmTransform t;
  t.translation = ToWorldSpace(transform.translation);
  t.rotation = meshTransform.rotation * transform.rotation;
  return t;
}

lmReal CameraUtil::GetSphereQueryRadius()
{
  return params.sphereRadiusMultiplier * referenceDistance;
}

void CameraUtil::RealignRefPtAndCamera()
{
  lmReal dist = (transform.translation - referencePoint.position).norm();
  Vector3 cameraNormal = transform.rotation * Vector3::UnitZ();
  transform.translation = referencePoint.position + dist * cameraNormal;
}

void CameraUtil::UpdateCameraOrientationFromPositions()
{
  Vector3 newCameraNormal = transform.translation - referencePoint.position;
  if (!lmIsZero(newCameraNormal)) {
    newCameraNormal.normalize();
    Vector3 oldCameraNormal = transform.rotation * Vector3::UnitZ();
    lmQuat correction; correction.setFromTwoVectors(oldCameraNormal, newCameraNormal);
    transform.rotation = correction * transform.rotation;

    referenceDistance = (transform.translation - referencePoint.position).norm();

    // Need to clip it though..
    lmClip(referenceDistance, params.minDist, params.maxDist);
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
  std::vector<int> triangles = mesh->getOctree()->intersectRay(ray.start, ray.GetDirection());
  std::vector<int> hits;
  lmReal minDist = FLT_MAX;

  lmRayCastOutput rayCastOutput;

  // Cast ray for each triangle's aabb
  for (size_t ti = 0; ti < triangles.size(); ti++) {
    const Triangle& tri = mesh->getTriangle(triangles[ti]);
    Vector3 hitPoint;
    bool rayHit = tri.aabb_.intersectRay(ray.start, ray.GetDirection());
    if (rayHit) {
      rayHit = Geometry::intersectionRayTriangle(
        ray.start, ray.end,
        mesh->getVertex(tri.vIndices_[0]),
        mesh->getVertex(tri.vIndices_[1]),
        mesh->getVertex(tri.vIndices_[2]),
        tri.normal_, hitPoint);
    }
    if (rayHit) {
      hits.push_back(triangles[ti]);
      lmReal dist = (hitPoint-ray.start).dot(ray.GetDirection());
      if (dist < minDist || collectall) {
        minDist = dist;

        rayCastOutput.triangleIdx = triangles[ti];
        rayCastOutput.position = hitPoint;
        rayCastOutput.normal = tri.normal_;
        rayCastOutput.dist = dist;
        rayCastOutput.fraction = dist / ray.GetLength();

        if (collectall) {
          results->push_back(rayCastOutput);
        }
      }
    }
  }

  if (!collectall && minDist < FLT_MAX) {
    results->push_back(rayCastOutput);
  }
}

bool CameraUtil::VerifyCameraMovement( Mesh* mesh, const Vector3& from, const Vector3& to, lmReal radius )
{
  bool validMovement = false;

  bool cameraCollidesMesh = CollideCameraSphere(mesh, to, radius);

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

double CameraUtil::IsoPotential( Mesh* mesh, const Vector3& position, lmReal queryRadius )
{
  //lmReal radius = Get
  double potential = 0.0;

  // process every n-th point
  const int striding = 4;

  if (params.userFaultyTriangles) {
    const TriangleVector& triangles = mesh->getTriangles();

    for (unsigned ti = 0; ti < triangles.size(); ti+= striding)
    {
      const Triangle& tri = triangles[ti];

      Geometry::GetClosestPointInput input(mesh, &tri, position);
      Geometry::GetClosestPointOutput output;
      Geometry::getClosestPoint(input, &output);

      double dist = output.distance;
      dist = lmClip(dist, 0.001, 1000000.0);
      potential += 1.0 / (dist*dist);
    }
  } else {
    const VertexVector& vertices = mesh->getVertices();

    for (unsigned vi = 0; vi < vertices.size(); vi+= striding)
    {
      const Vertex& vert = vertices[vi];

      double distSqr = (position-vert).squaredNorm();
      //static const double K = 0.0001f;
      distSqr = lmClip(distSqr, (double)params.grav_k*params.grav_k, 1000000.0*1000000.0);
      //static const double N = 6.0f;
      potential += 1.0 / std::pow(distSqr, (double)params.grav_n);

      if (params.drawSphereQueryResults) {
        LM_DRAW_CROSS(vert, 5.0f, lmColor::WHITE);
      }
    }
  }

  return potential;
}

Vector3 CameraUtil::IsoNormal( Mesh* mesh, const Vector3& position, lmReal queryRadius )
{
  lmReal epsilon = 0.1f;
  const Vector3& pos = position;
  double potential = IsoPotential(mesh, pos, queryRadius);
  Vector3 posX = pos; posX.x() += epsilon;
  Vector3 posY = pos; posY.y() += epsilon;
  Vector3 posZ = pos; posZ.z() += epsilon;
  double dPotentialX = IsoPotential(mesh, posX, queryRadius) - potential;
  double dPotentialY = IsoPotential(mesh, posY, queryRadius) - potential;
  double dPotentialZ = IsoPotential(mesh, posZ, queryRadius) - potential;

  typedef Eigen::Matrix<double, 3, 1> Vector3d;
  Vector3d negNormal(dPotentialX, dPotentialY, dPotentialZ);
  negNormal.normalize();

  //std::cout << "Iso potential: " << potential << std::endl;

  Vector3 normal((float)-negNormal.x(), (float)-negNormal.y(), (float)-negNormal.z());
  return normal;
}

lmReal CameraUtil::IsoQueryRadius(IsoCameraState* state) const {
  return std::min(state->refDist + 50.0f, 50.0f);
}

void CameraUtil::IsoUpdateCameraDirection(const Vector3& newDirection, IsoCameraState* state ) {
  lmQuat qCorrection; qCorrection.setFromTwoVectors(GetCameraDirection(), newDirection);

  transform.rotation = qCorrection * transform.rotation;
  transform.translation = state->refPosition - state->cameraOffsetMultiplier * state->refDist * newDirection;
}

void CameraUtil::InitIsoCamera( Mesh* mesh, IsoCameraState* state )
{
  // get field potential from current position
  lmReal t = 0.5f;
  state->refPosition = lmInterpolate(t, referencePoint.position, transform.translation);
  //state->refPotential = IsoPotential(mesh, state->refPosition);
  state->cameraOffsetMultiplier = 1.0f;

  lmReal queryRadius = 10000.0f; // everything
  state->closestPointOnMesh = GetClosestSurfacePoint(mesh, state->refPosition, queryRadius);
  state->refDist = (state->closestPointOnMesh.position - state->refPosition).norm();

  state->refPotential = IsoPotential(mesh, state->refPosition, IsoQueryRadius(state));

  // Remember closest point distance
}

void CameraUtil::IsoCamera( Mesh* mesh, IsoCameraState* state, const Vector3& movement )
{
  // Check if current refPosition is inside the mesh (which may happen sculpting) and correct it.
  {
    int attemptCount = 0;
    lmRayCastOutput raycastOutput;
    CastOneRay(mesh, lmRay(transform.translation, state->refPosition), &raycastOutput);
    while (raycastOutput.isSuccess()) {
      //state->refPosition = raycastOutput.position + (-1.0f / std::min(raycastOutput.normal.dot(-GetCameraDirection()), 0.2f)) * GetCameraDirection();
      if (GetCameraDirection().dot(raycastOutput.normal) < 0) {
        state->refPosition = raycastOutput.position + params.minDist * raycastOutput.normal;
      } else {
        state->refPosition = raycastOutput.position - params.minDist * GetCameraDirection();
      }
#if LM_LOG_CAMERA_LOGIC_3
      std::cout << "Fixed ref point in the mesh" << std::endl;
#endif

      if (++attemptCount > 10 ||  (state->refPosition - transform.translation).dot(GetCameraDirection()) < 0.0f ) {
        ResetCamera(mesh, GetCameraDirection());
        state->refPosition = referencePoint.position + params.minDist * referencePoint.normal;
        break;
      }

      raycastOutput.invalidate();
      CastOneRay(mesh, lmRay(transform.translation, state->refPosition), &raycastOutput);
    }
  }

  Vector3 oldRefPosition = state->refPosition;
  lmReal oldRefDist = state->refDist;

  // Dummy & temp: clip z movement
  Vector3 clippedMovement = movement * std::sqrt(std::sqrt(state->refDist / params.refDistForMovemement));

  clippedMovement.z() *= params.scaleZMovement;

  clippedMovement = transform.rotation * clippedMovement;

  // Verify movement
  bool validMovement = VerifyCameraMovement(mesh, state->refPosition, state->refPosition + clippedMovement, params.minDist/2.0f);
  if (!validMovement) {
    return;
  }

  // Saftey clip movement so that the camera doesn't go past the safety distance.
  if ((state->refPosition + clippedMovement).norm() > params.maxDist) {
    Vector3 newPos = state->refPosition + clippedMovement;
    newPos.normalize();
    newPos *= params.maxDist;
    clippedMovement = newPos - state->refPosition;
  }

  Vector3 newNormal = IsoNormal(mesh, state->refPosition + clippedMovement, IsoQueryRadius(state));
  if (!lmIsNormalized(newNormal)) {
    return;
  } else {
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
      std::cout << "Movement scaling: " << scale << std::endl;
#endif
      if (!lmIsNormalized(newNormal)) {
        return;
      }
    }
  }

  // Update closest distance: last check
  lmSurfacePoint closestPoint = GetClosestSurfacePoint(mesh, state->refPosition + clippedMovement, state->refDist + clippedMovement.norm());
  if (!lmIsNormalized(state->closestPointOnMesh.normal)) {
    return;
  }

  state->refPosition += clippedMovement;
  state->refNormal = newNormal;
  state->closestPointOnMesh = closestPoint;
  state->refDist = (state->closestPointOnMesh.position - state->refPosition).norm();

  LM_ASSERT(lmIsNormalized(state->refNormal), "Iso normal failed.")

  // Update camera
  IsoUpdateCameraDirection(-state->refNormal, state);

  LM_DRAW_CROSS(state->refPosition, 20.0f, lmColor::GREEN);
  LM_DRAW_CROSS(state->closestPointOnMesh.position, 20.0f, lmColor::RED);
  LM_DRAW_ARROW(state->closestPointOnMesh.position, state->closestPointOnMesh.position + state->closestPointOnMesh.normal * 40.0f, lmColor::RED);

  LM_ASSERT(state->refDist < 10000, "Reference distance exploded.")

  // Display reference sphere (updating point & radius):
  referencePoint.position = state->refPosition - state->refNormal * state->refDist;
  referencePoint.normal = state->refNormal;
  referenceDistance = state->refDist * (1 + params.isoRefDistMultiplier);
}

