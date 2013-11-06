#include "StdAfx.h"
#include "CameraUtil.h"
#include "DebugDrawUtil.h"
#include "Geometry.h"

#define DRAW_SPHERE_QUERY_RESULTS 1
#define DISABLE_LEAP 0

#define PREFER_CLOSEST_NORMAL 0


CameraUtil::CameraUtil() {
  transform.translation.setZero();
  transform.rotation.setIdentity();
  smoothedTransform.translation.setZero();
  smoothedTransform.rotation.setIdentity();
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
}

void CameraUtil::SetFromStandardCamera(const Vector3& from, const Vector3& to, lmReal referenceDistance) {
  std::unique_lock<std::mutex> lock(mutex);

  GetTransformFromStandardCamera(from, to, transform);
  smoothedTransform = transform;

  this->referenceDistance = referenceDistance;
  state = STATE_FREEFLOATING;
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
    std::vector<Vector3>& dbgPoints = debugDrawUtil->GetDebugPoints();
    std::unique_lock<std::mutex> lock(debugDrawUtil->m_mutex);
    dbgPoints.insert(dbgPoints.end(), rays.begin(), rays.end());
    lock.unlock();
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
      if (minDistIdx != -1) {
        results->push_back(rayCastOutput);
        // for debug display only
        triangles.push_back(minDistIdx);
      }
    } else {
      triangles = tris2;
    }

    // Display debug triangles
    if (debugDrawUtil) {
      std::unique_lock<std::mutex> lock(debugDrawUtil->m_mutex);
      for (size_t ti = 0; ti < triangles.size(); ti++) {
        const Triangle& tri = mesh->getTriangle(triangles[ti]);
        std::vector<Vector3>& debugTris = debugDrawUtil->GetDebugTriangles();
        for (int vi = 0; vi < 3; vi++) {
          debugTris.push_back(mesh->getVertex(tri.vIndices_[vi]));
        }
      }
    }
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
  tris = const_cast<Mesh*>(mesh)->getTrianglesFromVertices(verts);

  Geometry::GetClosestPointOutput closestPoint;
  closestPoint.distance = FLT_MAX;
  int closestTriangleIdx = -1;;

  for (size_t ti = 0; ti < tris.size(); ti++) {
    const Triangle& tri = mesh->getTriangle(tris[ti]);

    // Discard rear-facing triangles
    lmReal camDotTNormal = cameraDirection.dot(tri.normal_);
    if (camDotTNormal < 0.0f)
    {
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

  if (closestPoint.distance < FLT_MAX)
  {
    GetNormalAtPoint(mesh, closestTriangleIdx, closestPoint.position, &closestPoint.normal);
  }

  *closestPointOut = closestPoint;
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
    //if (0.707f < dot && 0.5f * radius < translationToVert.norm() ) {
    if ((0.707f < dot || 0.707f < dotMixed) && 0.5f * radius < translationToVert.norm() ) {
      vertices->push_back(vi);
    }
  }
}

void CameraUtil::VecGetAveragedSurfaceNormal(const Mesh* mesh, const lmSurfacePoint& referencePoint, lmReal radius, const Vector3& cameraDirection, bool weightNormals, lmSurfacePoint* avgSurfacePoint, Geometry::GetClosestPointOutput* closestPointOut) {
  avgSurfacePoint->position.setZero();
  avgSurfacePoint->normal.setZero();

  Vector3 normal = Vector3::Zero();
  Vector3 position = Vector3::Zero();

  Geometry::GetClosestPointOutput closestPoint;
  closestPoint.distance = FLT_MAX;
  int closestTriangleIdx = -1;;


  std::vector<int> verts;
  const_cast<Mesh*>(mesh)->getVerticesInsideSphere(referencePoint.position, radius*radius, *&verts);

  if (params.userFaultyTriangles) {

    std::vector<int> tris;
    tris = const_cast<Mesh*>(mesh)->getTrianglesFromVertices(verts);
    lmReal area = 0.0f;

    for (size_t ti = 0; ti < tris.size(); ti++) {
      const Triangle& tri = mesh->getTriangle(tris[ti]);

      // Discard rear-facing triangles
      lmReal camDotTNormal = cameraDirection.dot(tri.normal_);
      Vector3 center = TriCenter(mesh, tri);
      //lmReal camDotCenter = cameraDirection.dot(center - referencePoint.position);
      if (camDotTNormal < 0.0f)// || camDotCenter < 0.0f)
      {
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
            closestTriangleIdx = tris[ti];
          }

#if DRAW_SPHERE_QUERY_RESULTS
          if (debugDrawUtil && params.drawDebugLines) {
            debugDrawUtil->DrawTriangle(mesh, tri);
          }
#endif
        }

        // Calc point weight
        lmReal t = closestPoint.distance / radius;
        t = 1.0f - std::min(1.0f, t);
        t = t*t;
        // Add weigth based on normal
        t = t * weight;
        // Average taking area into account
        normal += t * a * tri.normal_;
        position += t * a * center;
        area += t * a;
      }
    }

    if (0.0f < area)
    {
      avgSurfacePoint->position = position / area;
      avgSurfacePoint->normal = normal.normalized();

//#if DRAW_SPHERE_QUERY_RESULTS
//      if (debugDrawUtil && params.drawDebugLines) {
//        std::vector<Vector3>& lines = debugDrawUtil->GetDebugLines();
//        //lines.push_back(center);
//        //lines.push_back(output.position);
//        debugDrawUtil->DrawCross(position , 10.0f);
//        debugDrawUtil->DrawArrow(position, position + 20.0f * normal);
//      }
//#endif

    }

  } else {

    lmReal sumWeight = 0.0f;

    for (size_t vi = 0; vi < verts.size(); vi++) {
      const Vertex& vert = mesh->getVertex(verts[vi]);

      // Discard rear-facing triangles
      lmReal camDotVNormal = cameraDirection.dot(vert.normal_);
//      lmReal camDotCenter = cameraDirection.dot(vert - referencePoint.position);
      if (camDotVNormal < 0.0f)// || camDotCenter < 0.0f)
      {
        lmReal weight = weightNormals ? std::max(0.0f, -camDotVNormal) : 1.0f;
        normal += vert.normal_ * weight;
        position += weight * vert;
        sumWeight += weight;

#if DRAW_SPHERE_QUERY_RESULTS
        if (debugDrawUtil && params.drawDebugLines) {
          debugDrawUtil->DrawCross(vert, 5.0f);
        }
#endif
      }
    }
    if (0.0f < sumWeight) {
      avgSurfacePoint->position = position / sumWeight;
      avgSurfacePoint->normal = normal.normalized();
    }
  }

  if (closestPoint.distance < FLT_MAX)
  {
    GetNormalAtPoint(mesh, closestTriangleIdx, closestPoint.position, &closestPoint.normal);
  }
  *closestPointOut = closestPoint;
}

void CameraUtil::RecordUserInput(const float _DTheta,const float _DPhi,const float _DFov) {
  std::unique_lock<std::mutex> lock(mutex);
  Vector3 movement(50.0f * _DTheta, -50.0f * _DPhi, -_DFov / 100.0f);
  userInput += movement;
}

void CameraUtil::RecordUserInput(const Vector3& deltaPosition, bool controlOn) {
#if DISABLE_LEAP
  return;
#endif
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
  Geometry::GetClosestPointOutput closestPoint[3];
//  Vector3 normals[3];
  VecGetAveragedSurfaceNormal(mesh, lmSurfacePoint(v1, v1.normal_), radius, -1.0f * v1.normal_, true, avgSurfacePoint+0, closestPoint+0);
  VecGetAveragedSurfaceNormal(mesh, lmSurfacePoint(v2, v2.normal_), radius, -1.0f * v2.normal_, true, avgSurfacePoint+1, closestPoint+1);
  VecGetAveragedSurfaceNormal(mesh, lmSurfacePoint(v3, v3.normal_), radius, -1.0f * v3.normal_, true, avgSurfacePoint+2, closestPoint+2);

  Vector3 normal = Vector3::Zero();
  //normal += coords[0] * v1.normal_;
  //normal += coords[1] * v2.normal_;
  //normal += coords[2] * v3.normal_;
  normal += coords[0] * avgSurfacePoint[0].normal;
  normal += coords[1] * avgSurfacePoint[1].normal;
  normal += coords[2] * avgSurfacePoint[2].normal;

  *normalOut = normal.normalized();
}

void CameraUtil::DebugDrawNormals(const Mesh* mesh, const Params& paramsIn) {
  // Draw normal for every nth triangle
  TriangleVector trangles = const_cast<Mesh*>(mesh)->getTriangles();
  VertexVector vertices = const_cast<Mesh*>(mesh)->getVertices();

  lmReal sphereRadius = params.sphereRadiusMultiplier * referenceDistance;

  for (size_t i = 0; i < vertices.size(); i+= 1) {
    Vertex vert = vertices[i];

    lmSurfacePoint refPoint(vert, vert.normal_);
    lmSurfacePoint newAvgVertex;
    Geometry::GetClosestPointOutput newClosestPoint;
    VecGetAveragedSurfaceNormal(mesh, refPoint, sphereRadius, -1.0f * vert.normal_, paramsIn.weightNormals, &newAvgVertex, &newClosestPoint);

    if(debugDrawUtil) {
      //debugDrawUtil->DrawArrow(vert, vert + vert.normal_ * 10.0f);
      debugDrawUtil->DrawArrow(vert, vert + newAvgVertex.normal * 10.0f);
    }

  }


  for (size_t i = 0; i < trangles.size(); i++) {
    Triangle tri = trangles[i];

  }

}


void CameraUtil::UpdateCamera(const Mesh* mesh, Params* paramsInOut) {

  //DebugDrawNormals(mesh, paramsIn);

  assert(mesh && "Mesh required");
  std::unique_lock<std::mutex> lock(mutex);
  if (!this->params.walkSmoothedNormals && paramsInOut->walkSmoothedNormals)
  {
    paramsInOut->freeRotationEnabled = false;
    paramsInOut->enableNormalCorrection = false;
    paramsInOut->useAvgNormal = false;
    paramsInOut->suppresForwardRotation = false;
    paramsInOut->tmpSwitch = false;
    paramsInOut->walkSmoothedNormals = true;
  }

  this->params = *paramsInOut;

  debugDrawUtil->SwitchBuffers();

  // Check time
  lmReal dt = 0.0f;
  {
    lmReal prevTime = lastCameraUpdateTime;
    lmReal time = lmReal(ci::app::getElapsedSeconds());
    lastCameraUpdateTime = time;
    if (prevTime < 0.0f) { prevTime = time; }
    dt = time - prevTime;
  }
  lmReal dtOne = 1.0f;
  lmReal dtZero = 0.0f;
  if(!params.enableSmoothing)
  {
    dt = dtOne;
  }

  // Accumulate userInput in 2d
  if (params.enableSmoothing)
  {
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
  Vector3 movement = userInput * movementRatio; movement.z() = 0.0f;
  movement = -1.0f * (GetFinalCamera().rotation * userInput);
  userInput.setZero();

  LM_ASSERT(movement.squaredNorm() < 100000000.0f, "");


  // Attempt to calculate new refernce point and normal
  //
  bool rayHit = false;
  lmReal refDist = 0.0f;
  const lmTransform oldCamera = this->transform;

  if (params.sphereCrawlMode)
  {
    int attampts = 0;

    const Geometry::GetClosestPointOutput oldClosestPoint = this->closestPoint;
    Geometry::GetClosestPointOutput& newClosestPoint = this->closestPoint;
    const lmSurfacePoint oldReferencePoint = this->referencePoint;
    lmSurfacePoint& newReferencePoint = this->referencePoint;
    const lmTransform oldCamera = this->transform;
    lmTransform& newCamera = this->transform;

    LM_ASSERT(oldReferencePoint.normal.squaredNorm() == 0.0f || lmIsNormalized(oldReferencePoint.normal), "");
    LM_ASSERT(lmIsNormalized(oldCamera.rotation * Vector3::UnitZ()), "");

    if (debugDrawUtil)
    {
      debugDrawUtil->DrawArrow(oldReferencePoint.position, oldReferencePoint.position + oldReferencePoint.normal * 50.0f);
    }

    // Correct camera back to normal orientation
    if (params.enableNormalCorrection &&  LM_EPSILON * LM_EPSILON < oldReferencePoint.normal.squaredNorm())
    {
      Vector3 oldCameraNormal = oldCamera.rotation * Vector3(0.0f, 0.0f, 1.0f);

      // Plane that connect them, if cross product is significant
      Vector3 cross = oldCameraNormal.cross(oldReferencePoint.normal);

      if (LM_EPSILON * LM_EPSILON < cross.squaredNorm())
      {
        cross.normalize();

        // todo: project movement onto oldCamera's plane, jsut to make sure ?

        // Decompose motion along those directions (movement is done in oldCamera's plane)
        LM_ASSERT(movement.squaredNorm() < 100000000.0f, "");
        Vector3 sideComponent = cross.dot(movement) * cross;
        Vector3 parallelComponent = movement - sideComponent;

        // Subtract from the 'projected' direction as much as we need, or max.
        //
        // Using the conversion param
        //
        // todo: be smarter:
        lmReal dotMovement = parallelComponent.dot(cross.cross(oldReferencePoint.normal));
        if (0.0f < dotMovement) {
          // todo: use fraction of movemnt here !!

          lmReal maxCameraTranslation = (oldReferencePoint.normal - oldCameraNormal).norm() * referenceDistance / params.freeRotationRatio;
          lmReal parallelNorm = parallelComponent.norm();

          if (LM_EPSILON < parallelNorm)
          {
            lmReal translationRatio = std::min(1.0f, maxCameraTranslation / parallelNorm);
            lmReal leftover = 1.0f - translationRatio;

            Vector3 newNormal = (transform.translation + translationRatio * parallelComponent * params.freeRotationRatio /* correction*/ - oldReferencePoint.position).normalized();
            CorrectCameraOrientation(dtZero, newNormal);

            // !! if we have more than 1 attempts.
            const_cast<lmTransform&>(oldCamera) = newCamera;

            // Leave the rest as it was.
            movement = sideComponent + leftover * parallelComponent;
            LM_ASSERT(movement.squaredNorm() < 100000000.0f, "");

            // std::cout << "ORIENTATION corrected" << std::endl;
          }
        }
      }
    }

    lmReal sphereRadius = params.sphereRadiusMultiplier * referenceDistance;

    lmSurfacePoint newAvgVertex;
    bool normalOkay = false;
    while(!normalOkay && attampts < 16)
    {
      LM_ASSERT(oldReferencePoint.normal.squaredNorm() == 0.0f || lmIsNormalized(oldReferencePoint.normal), "");
      LM_ASSERT(lmIsNormalized(oldCamera.rotation * Vector3::UnitZ()), "");

      // Sphere sweep.
      newReferencePoint.position += movement;
      newCamera.translation += movement;

      // Attempt sphere query
      Vector3 cameraDirection = oldCamera.rotation * Vector3(0.0f, 0.0f, -1.0f);
      VecGetAveragedSurfaceNormal(mesh, newReferencePoint, sphereRadius, cameraDirection, params.weightNormals, &newAvgVertex, &newClosestPoint);
      if (!lmIsNormalized(newClosestPoint.normal)) {
        GetClosestPoint(mesh, newReferencePoint, sphereRadius, cameraDirection, &newClosestPoint);
      }


      newReferencePoint.normal = params.useAvgNormal ? newAvgVertex.normal : newClosestPoint.normal;

      normalOkay = lmIsNormalized(newReferencePoint.normal);

      if (normalOkay && params.walkSmoothedNormals) {
        GetSmoothedNormalAtPoint(mesh, newClosestPoint.triIdx, newClosestPoint.position, sphereRadius, &newClosestPoint.normal);
        if (lmIsNormalized(oldClosestPoint.normal)) {
          lmReal rotDist = (oldClosestPoint.normal - newClosestPoint.normal).norm();// * referenceDistance;
          if (0.2f < rotDist) {
            movement *= 0.2f / rotDist;
            newCamera = oldCamera;
            newReferencePoint = oldReferencePoint;

            //lmReal t = 0.2f / rotDist;
            //newClosestPoint = lmInterpolate(t, oldClosestPoint.position, newClosestPoint.position);
          }
        }
      }

      if (normalOkay && params.overrideNormal) {
        Vector3 smoothedNormal;
        // use newClosestPoint, or ReferencePoint
        GetSmoothedNormalAtPoint(mesh, newClosestPoint.triIdx, newClosestPoint.position, sphereRadius, &smoothedNormal);
        //newReferencePoint.normal = newAvgVertex.normal;
        newReferencePoint.normal = smoothedNormal;
      }

      attampts++;
      if (!normalOkay && attampts < 16)
      {
        movement *= 0.5f;
        newCamera = oldCamera;
        newReferencePoint = oldReferencePoint;
      }
    }

    //LM_ASSERT(normalOkay, "Couldn't get the normal.");

    if (lmIsNormalized(newReferencePoint.normal)) {

      bool backwardsNormalChange = false;
      bool fowardVerticesDetected = false;

      // apply quaternion of averaged normals.

#if PREFER_CLOSEST_NORMAL
      const lmSurfacePoint oldCalcPoint(oldClosestPoint.position, oldClosestPoint.normal);
      const lmSurfacePoint newCalcPoint(newClosestPoint.position, newClosestPoint.normal);
#else
      const lmSurfacePoint& oldCalcPoint = oldReferencePoint;
      const lmSurfacePoint& newCalcPoint = newReferencePoint;
#endif

      if (params.suppresForwardRotation && lmIsNormalized(oldCalcPoint.normal) && LM_EPSILON * LM_EPSILON < movement.squaredNorm()) 
      {

        std::vector<int> vertices;
        FindPointsAheadOfMovement(mesh, referencePoint, sphereRadius, movement.normalized(), &vertices);
        fowardVerticesDetected = 0 < vertices.size();

        lmQuat normalChange;
        normalChange.setFromTwoVectors(oldCalcPoint.normal, newCalcPoint.normal);

        // Decompose normal update (if not opposite directions)
        if (-0.9f < oldCalcPoint.normal.dot(newCalcPoint.normal) )
        {
          Vector3 deltaNormal = newCalcPoint.normal - oldCalcPoint.normal;
          // Decompose: direction along & perp to motion
          Vector3 movementDir = movement.normalized();
          Vector3 deltaNormalAlongMotion = movementDir.dot(deltaNormal) * movementDir;
          Vector3 deltaNormalPerpMotion = deltaNormal - deltaNormalAlongMotion;

//          lmReal movementDot = movementDir.dot(deltaNormalAlongMotion);
          // std::cout << "Movement Dot: " << movementDot << std::endl;
          // Apply the normal delta along only if it's towards the back
          //if (params.tmpSwitch || movementDot < 0.0f)
          if (params.tmpSwitch || fowardVerticesDetected)
          {
            if (!params.tmpSwitch) {
              backwardsNormalChange = true;
            }
            // std::cout << "ORIENTATION backward detected" << std::endl;
          }
          else
          {
            // std::cout << "ORIENTATION forward, clipped" << std::endl;

            // forward normal change -- let the 'explicity camera movement' code handle that
            Vector3 clippedNormal = (oldCalcPoint.normal + deltaNormalPerpMotion).normalized();
            normalChange.setFromTwoVectors(oldCalcPoint.normal, clippedNormal);
          }
        }

        // apply to camera transform
        Vector3 cameraNormal = newCamera.rotation * Vector3::UnitZ();
        Vector3 newNormal = normalChange * cameraNormal; //-- only have it for backward bending angles ?? or when the above rotation is not engaged...
        //Vector3 newNormal = normalChange * CalcPoint.normal; //-- only have it for backward bending angles ?? or when the above rotation is not engaged...
        CorrectCameraOrientation(dtZero, newNormal);

        // check
        Vector3 newCameraNormal = newCamera.rotation * Vector3::UnitZ();
        lmQuat rotCheck; rotCheck.setFromTwoVectors(cameraNormal, newCameraNormal);
        int i = 0; i++;
      }

      //
      //
      //


      // Cast reference point onto the plane(clostestPoint.position, newReferencePoint.normal))
      {
        //Vector3 diff = newReferencePoint.position - newClosestPoint.position;
#if PREFER_CLOSEST_NORMAL
        Vector3 diff = newReferencePoint.position - newClosestPoint.position;
#else
        Vector3 diff = newReferencePoint.position - ((params.useAvgNormal && !params.useClosestPointForEdges) ? avgVertex.position : newClosestPoint.position);
#endif
        newReferencePoint.position -= newReferencePoint.normal * diff.dot(newReferencePoint.normal); // smooth it !!
        // this will drift, but will fix a problem for now (todo)
        transform.translation -= newReferencePoint.normal * diff.dot(newReferencePoint.normal); // smooth it !!
      }

      if (params.freeRotationEnabled)
      {
        // Method 2
        //
        // Look at reference point movement and closest point movement
        Vector3 dRefPt;
        Vector3 dClosestPt;
        if (!params.useAvgNormal || params.useClosestPointForEdges)
        {
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
        if (LM_EPSILON < refDist) { // && directionDot < 0.0f)
          Vector3 projected = dRefPt.dot(dClosestPt) / dRefPt.squaredNorm() * dRefPt ;
          lmReal closestDist = projected.norm();

          // Add camera translation
          lmReal movedFraction = std::min((closestDist+LM_EPSILON)/(refDist+LM_EPSILON), 1.0f);
          // std::cout << "MOVED FRACTION: " << movedFraction << std::endl;
          if (backwardsNormalChange) {
            movedFraction = 1.0f;
          }
          lmReal rotationFraction = 1.0f - movedFraction;

          // Use the difference in magnitude to
          //// Rotate camera to new position, at the same time: we need to stop updating the normal .....
          if (params.clipTranslationOnFreeRotate && !backwardsNormalChange)
          {
            if (!params.useAvgNormal || params.useClosestPointForEdges)
            {
              Vector3 correction = newClosestPoint.position - referencePoint.position; // check that this is (1-rotationFraction) * dRefPt
//              lmReal correctionDist = correction.norm();
              referencePoint.position = newClosestPoint.position;
              transform.translation += correction;
            }
            else
            {
              Vector3 correction = newAvgVertex.position - referencePoint.position; // check that this is (1-rotationFraction) * dRefPt
//              lmReal correctionDist = correction.norm();
              referencePoint.position = newAvgVertex.position;
              transform.translation += correction;
            }
          }

          Vector3 newCameraNormal = (transform.translation + params.freeRotationRatio * (rotationFraction*dRefPt) - referencePoint.position).normalized();
          CorrectCameraOrientation(dtZero, newCameraNormal);
        } else {
          // Prevent snapping back to the original positon
          lmReal rotationFraction = 0.0f;
          Vector3 newCameraNormal = (transform.translation + params.freeRotationRatio * (rotationFraction*dRefPt) - referencePoint.position).normalized();
          CorrectCameraOrientation(dtZero, newCameraNormal);
        }
      }

      if (!params.freeRotationEnabled && !params.suppresForwardRotation)
      {
        CorrectCameraOrientation(dtZero, referencePoint.normal);
      }

      LM_ASSERT(referencePoint.normal.squaredNorm() < 2.0f, "Normal exploded.");

      rayHit = true;
      this->closestPoint = newClosestPoint; // remember closest point for the next frame.
      this->avgVertex = newAvgVertex;
    }
  }

  if (!params.sphereCrawlMode || !rayHit)
  {
    std::vector<lmRay> rays;
    GenerateRays(transform, RAY_CAST_BATCH_SIDE, &rays);
    std::vector<lmRayCastOutput> rayCastResults;
    CastRays(mesh, rays, &rayCastResults);

    // Collapse normals and distance and position
    //
    lmReal minDist = FLT_MAX;
    // todo do calculations the same way they're done in MainCameraControl()
    const int numCasts = rayCastResults.size();
    //float weightSum = 0;
    for (int i = 0; i < numCasts; i++) {
      if (rayCastResults[i].isSuccess() && rayCastResults[i].dist < minDist) {
        rayHit = true;
        referencePoint.position = rayCastResults[i].position;
        referencePoint.normal.setZero();
        // Calc normal from the triangle
        Vector3 normal;
        GetNormalAtPoint(mesh, rayCastResults[i].triangleIdx, referencePoint.position, &normal);
        referencePoint.normal += normal;
        refDist = rayCastResults[i].dist;
        minDist = rayCastResults[i].dist;
      }
    }

    if (rayHit && params.useSphereQuery) {
      assert(0.1f < referencePoint.normal.squaredNorm() && referencePoint.normal.squaredNorm() < 2.0f  && "Normal wrong..");
      // Perform a sphere cast and average normal
      Vector3 cameraDirection = transform.rotation * Vector3(0.0f, 0.0f, -1.0f);
      lmReal sphereRadius = params.sphereRadiusMultiplier * referenceDistance;

      Geometry::GetClosestPointOutput closestPoint;
      lmSurfacePoint avgVertex;
      VecGetAveragedSurfaceNormal(mesh, referencePoint, sphereRadius, cameraDirection, params.weightNormals, &avgVertex, &closestPoint);
      if (!lmIsNormalized(closestPoint.normal)) {
        GetClosestPoint(mesh, referencePoint, sphereRadius, cameraDirection, &closestPoint);
      }
      Vector3 avgNormal = avgVertex.normal;

      TODO(adrian, make mesh crawling indepenent of the camera pos);

      referencePoint.normal = lmInterpolate(0.3f, referencePoint.normal, avgNormal).normalized();
      assert(0.1f < referencePoint.normal.squaredNorm() && referencePoint.normal.squaredNorm() < 2.0f  && "Normal wrong..");
    }
  }

  // Adjust translation to user input.
  if (rayHit) {
    if (debugDrawUtil && params.drawDebugLines) {
      debugDrawUtil->DrawCross(referencePoint.position, 10.0f);
      debugDrawUtil->DrawArrow(referencePoint.position, referencePoint.position + referencePoint.normal * 20.0f);
    }

    // clean up:
    referenceDistance -= deltaAngles.z();
    referenceDistance = std::max(params.minDist, referenceDistance);
    referenceDistance = std::min(params.maxDist, referenceDistance);

    // testing settings
    static bool adjustDistance = true;
    static bool adjustOrientation = true;

    if (adjustOrientation) {
      if (!params.sphereCrawlMode && !params.freeRotationEnabled)
      {
        CorrectCameraOrientation(dtOne, referencePoint.normal);
      }
      //else
      //{
      //  Vector3 newCameraNormal = transform.rotation * Vector3::UnitZ();
      //  transform.rotation = oldCamera.rotation;
      //  CorrectCameraOrientation(dtOnedt, newCameraNormal);
      //}
    }
    if (params.sphereCrawlMode) {
      // recalc refdist
      refDist = (referencePoint.position - transform.translation).norm();
    }
    if (adjustDistance) { CorrectCameraDistance(dt, refDist); }
  } else if (0) {
    // backup - no ray hit
    // 0. interaction test -- freefloat'ed orientation around the origin.

    // Create rotations
    lmQuat rotX(AngleAxis(-deltaAngles.y(), Vector3::UnitX()));
    lmQuat rotY(AngleAxis(-deltaAngles.x(), Vector3::UnitY()));

    // Apply them to current viewing matrix
    transform.rotation = transform.rotation * rotX * rotY;
    // Adjust camera position based on the expected distance
    transform.translation = transform.rotation * Vector3(0.0f, 0.0f, referenceDistance);
  }

  // don't update transform when there's no successful raycast
  if (params.pinUpVector) { CorrectCameraUpVector(dt, Vector3::UnitY()); }

  // Sync reference point & camera transform
  lmReal dist = (transform.translation - referencePoint.position).norm();
  Vector3 cameraNormal = transform.rotation * Vector3::UnitZ();
  transform.translation = referencePoint.position + dist * cameraNormal;

  return;
}


lmTransform CameraUtil::GetFinalCamera() {
  // Return either the detailed camera, or the blended hybrid camera
  if (1) {
    return transform;
  } else {
    return GetHybridCameraTransform();
  }
}

void CameraUtil::CorrectCameraOrientation(lmReal dt, const Vector3& newNormal) {
  LM_ASSERT(lmIsNormalized(newNormal), "");

  lmReal adjustOrientationRate = 0.0f;
  if (params.enableSmoothing && 0.0f < dt) {
    adjustOrientationRate = std::pow(params.smoothingFactor, dt);
  }

  // Roation quaternion.
  Vector3 oldZ = transform.rotation * Vector3::UnitZ();
  //Vector3 newZ = referencePoint.normal;
  Vector3 newZ = newNormal;

  // Adjust orientation and angle

  // Target orientation
  lmQuat dQ; dQ.setFromTwoVectors(oldZ, newZ);

  // Modify camera transform
  dQ = dQ.slerp(adjustOrientationRate, lmQuat::Identity());
  transform.rotation = dQ * transform.rotation;

  Vector3 cameraFromRefPoint = transform.translation - referencePoint.position;
  transform.translation = referencePoint.position + dQ * cameraFromRefPoint;
}

void CameraUtil::CorrectCameraDistance(lmReal dt, lmReal currentDistance) {
  lmReal smoothingValue = std::pow(0.8f, dt);
  lmReal deltaDist = referenceDistance - currentDistance;
  Vector3 deltaTranslation = (1.0f-smoothingValue) * deltaDist * (transform.rotation * Vector3::UnitZ()); // 'smoother' movement; add continues velocity though.
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
    lmReal adjustOrientationRate = std::pow(0.9f, dt); //std::max(0.0f, -0.2f + 1.2f * std::fabs(xyPlaneNormalDotUp));

    // Apply correction.
    dQ = dQ.slerp(adjustOrientationRate, lmQuat::Identity());
    transform.rotation = dQ * transform.rotation;
  }
}

lmTransform CameraUtil::GetHybridCameraTransform() {
  lmTransform orbitingTransform;
  GetTransformFromStandardCamera(transform.translation, Vector3::Zero(), orbitingTransform);

  // just blend ??
  // detailed - 0 - 1 - orbiting

  lmReal t = (referenceDistance-params.blendMinDist) / (params.blendMaxDist-params.blendMinDist);
  t = lmClip(t, 0.0f, 1.0f);

  lmTransform tOut;
  // Interpolate position
  //tOut.translation = lmInterpolate(t, transform.translation, orbitingTransform.)
  // -- if you want to interpolate position --- do that before getting the orientation from GetTransformFromStandardCamera.
  tOut.translation = transform.translation;
  // Interpolate orientation
  tOut.rotation = transform.rotation.slerp(t, orbitingTransform.rotation);

  return tOut;
}

lmTransform CameraUtil::GetSmoothedCamera(lmReal dt) {
  return transform;

  //lmReal smoothingRate = 0.9f;
  //lmReal smoothingValue = std::pow(smoothingRate, dt);
  //lmReal t = smoothingValue;

  //smoothedTransform.translation = (1.0f-t)*transform.translation + t*smoothedTransform.translation;
  //smoothedTransform.rotation = transform.rotation.slerp(t, smoothedTransform.rotation);

  //return smoothedTransform;
}
