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
  boost::unique_lock<boost::mutex> lock(mutex);

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
  std::vector<Vector3, Eigen::aligned_allocator<Vector3>> rays;
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
    boost::unique_lock<boost::mutex> lock(debugDrawUtil->m_mutex);
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
      boost::unique_lock<boost::mutex> lock(debugDrawUtil->m_mutex);
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
  for (int vi = 0; vi < verts.size(); vi++) {
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

  Vector3 normal = Vector3::Zero();
  Vector3 position = Vector3::Zero();

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
      if (camDotTNormal < 0.0f)
      {
        lmReal weight = std::max(0.0f, -camDotTNormal);

        // Calc distance and weight
        Vector3 center = TriCenter(mesh, tri);
        lmReal a = TriArea(mesh, tri);

        // Calc point weight
        Vector3 diff = center - referencePoint.position;
        lmReal t = diff.norm() / radius;
        t = 1.0f - std::min(1.0f, t);
        t = t*t;

        // Add weigth based on normal
        // todo:
        t = t * weight;

        // Average taking area into account
        normal += t * a * tri.normal_;
        position += t * a * center;
        area += t * a;
      }

      if (debugDrawUtil && params.drawDebugLines) {
        debugDrawUtil->DrawCross(TriCenter(mesh, tri), 5.0f);
      }

    }
    if (0.0f < area)
    {
      //normal /= area;
      normal.normalize();
      position /= area;
      *avgPosition = position;
    }

  } else {

    lmReal sumWeight = 0.0f;

    for (size_t vi = 0; vi < verts.size(); vi++) {
      const Vertex& vert = mesh->getVertex(verts[vi]);

      // Discard rear-facing triangles
      lmReal camDotVNormal = cameraDirection.dot(vert.normal_);
      if (camDotVNormal < 0.0f)
      {
        lmReal weight = std::max(0.0f, -camDotVNormal);
        normal += vert.normal_ * weight;
        position += weight * vert;
        sumWeight += weight;

        if (debugDrawUtil && params.drawDebugLines) {
          debugDrawUtil->DrawCross(vert, 5.0f);
        }
      }
    }
    if (0.0f < sumWeight) {
      normal.normalize();
      position /= sumWeight;
      *avgPosition = position;
    }
  }
  return normal;
}

void CameraUtil::RecordUserInput(const float _DTheta,const float _DPhi,const float _DFov) {
  boost::unique_lock<boost::mutex> lock(mutex);
  Vector3 movement(50.0f * _DTheta, -50.0f * _DPhi, -_DFov / 100.0f);
  userInput += movement;
}

void CameraUtil::RecordUserInput(const Vector3& deltaPosition, bool controlOn) {
  boost::unique_lock<boost::mutex> lock(mutex);
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

void CameraUtil::UpdateCamera(const Mesh* mesh, const Params& params) {


void CameraUtil::UpdateCamera(const Mesh* mesh, Params* paramsInOut) {

  assert(mesh && "Mesh required");
  boost::unique_lock<boost::mutex> lock(mutex);
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
  {
    lmReal prevTime = lastCameraUpdateTime;
    lmReal time = lmReal(ci::app::getElapsedSeconds());
    lastCameraUpdateTime = time;
    if (prevTime < 0.0f) { prevTime = time; }
  }

  // Accumulate userInput in 2d
  accumulatedUserInput += userInput;
  userInput = 0.1f * accumulatedUserInput;
  accumulatedUserInput -= userInput;

  // Process smoothed user input
  Vector3 deltaAngles = userInput;
  Vector3 movement = -1.0f * (GetFinalCamera().rotation * userInput);
  userInput.setZero();

  // Multiply motion by distance:
  const lmReal distFraction = (referenceDistance-params.minDist)/(params.maxDist-params.minDist);
  const lmReal movementRatio = params.speedAtMinDist + distFraction * (params.speedAtMaxDist-params.speedAtMinDist);

  deltaAngles *= movementRatio;
  movement *= movementRatio;

  lmTransform originalTransform = transform;
  transform.translation += movement;

  // Debug display movement
  if (debugDrawUtil && params.drawDebugLines) {
    Vector3 oldP = transform.translation + transform.rotation * (-100.0f * Vector3::UnitZ());
    Vector3 newP = transform.translation + transform.rotation * (-100.0f * Vector3::UnitZ()) + movement;
    debugDrawUtil->DrawArrow(oldP, newP);
  }

  // Attempt to calculate new refernce point and normal
  //
  bool rayHit = false;
  lmReal refDist = 0.0f;
  if (params.useSphereQueryToMoveRefernecePoint)
  {
    int attampts = 0;
    do
    {
      // Sphere sweep.
      lmSurfacePoint oldRefPt = referencePoint;
      referencePoint.position += movement;

      // Project point onto the model (closest point)
      // Using vertices is not save, 

      // Attempt sphere query
      lmReal sphereRadius = params.sphereRadiusMultiplier * referenceDistance;
      Vector3 cameraDirection = transform.rotation * Vector3(0.0f, 0.0f, -1.0f);
      Vector3 avgPosition;
      Vector3 avgNormal = VecGetAveragedSurfaceNormal(mesh, referencePoint, sphereRadius, cameraDirection, &avgPosition);
      if (lmIsNormalized(avgNormal)) {
        referencePoint.normal = avgNormal;

        // Cast reference point onto the plane
        Vector3 diff = (referencePoint.position - avgPosition);
        referencePoint.position -= avgNormal * diff.dot(avgNormal); // smooth it !!
        // this will drift, but will fix a problem for now (todo)
        transform.translation -= avgNormal * diff.dot(avgNormal); // smooth it !!
        rayHit = true;
        // Correct ref point's normal (need get closest point).
      }
      attampts++;
      if (!rayHit && attampts < 16)
      {
        transform = originalTransform;
        referencePoint.position = oldRefPt.position;
        movement *= 0.5f;
        transform.translation += movement;
      }
    }
    while (!rayHit && attampts < 16);
  }

  if (!params.useSphereQueryToMoveRefernecePoint || !rayHit)
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
    float weightSum = 0;
    for (int i = 0; i < numCasts; i++) {
      if (rayCastResults[i].isSuccess() && rayCastResults[i].dist < minDist) {
        rayHit = true;
        referencePoint.position = rayCastResults[i].position;
        referencePoint.normal.setZero();
        // Calc normal from the triangle
        Vector3 barycentric;
        GetBarycentricCoordinates(mesh, rayCastResults[i].triangleIdx, referencePoint.position, &barycentric);
        const Triangle& tri = mesh->getTriangle(rayCastResults[i].triangleIdx);
        const Vertex& v1 = mesh->getVertex(tri.vIndices_[0]);
        const Vertex& v2 = mesh->getVertex(tri.vIndices_[1]);
        const Vertex& v3 = mesh->getVertex(tri.vIndices_[2]);
        referencePoint.normal += barycentric[0] * v1.normal_;
        referencePoint.normal += barycentric[1] * v2.normal_;
        referencePoint.normal += barycentric[2] * v3.normal_;

        refDist = rayCastResults[i].dist;
        minDist = rayCastResults[i].dist;
      }
    }
    //if (rayHit) {
    //  referencePoint.position = refPosition;
    //  referencePoint.normal = refNormal;
    //}

    if (rayHit && params.useSphereQuery) {
      assert(0.1f < referencePoint.normal.squaredNorm() && referencePoint.normal.squaredNorm() < 2.0f  && "Normal wrong..");
      // Perform a sphere cast and average normal
      Vector3 cameraDirection = transform.rotation * Vector3(0.0f, 0.0f, -1.0f);
      //lmReal sphereRadius = 0.25f * referenceDistance;
      lmReal sphereRadius = params.sphereRadiusMultiplier * referenceDistance;
      Vector3 unusedPosition;
      Vector3 avgNormal = VecGetAveragedSurfaceNormal(mesh, referencePoint, sphereRadius, cameraDirection, &unusedPosition);
      TODO(adrian, make mesh crawling indepenent of the camera pos);
      //Vector3 avgNormal = VecGetAveragedSurfaceNormal(mesh, referencePoint, sphereRadius, -referencePoint.normal, &unusedPosition);
      referencePoint.normal = lmInterpolate(0.3f, referencePoint.normal, avgNormal).normalized();
      assert(0.1f < referencePoint.normal.squaredNorm() && referencePoint.normal.squaredNorm() < 2.0f  && "Normal wrong..");
    }
  }

  // Based on the old/new reference points and their normals determine.
  //  - angled motion
  //  - orientation change

  // 1. reorient camera to half rotation between future and current positions
  //   - determine arc the camera would travel from original position to position right above the newly found
  //     reference point.
  //   - scale the rotation & translation to do. 
  //   - goto (2) raycasts.

  // Adjust translation to user input.
  if (rayHit) {
    if (debugDrawUtil && params.drawDebugLines) {
      debugDrawUtil->DrawCross(referencePoint.position, 10.0f);
      debugDrawUtil->DrawArrow(referencePoint.position, referencePoint.position + referencePoint.normal * 20.0f);
    }

    // clean up:
    //referenceDistance = targetDist;
    referenceDistance -= deltaAngles.z();
    referenceDistance = std::max(params.minDist, referenceDistance);
    referenceDistance = std::min(params.maxDist, referenceDistance);
    LM_LOG << "Reference distance is " << referenceDistance << std::endl;

    // testing settings
    static bool adjustDistance = true;
    static bool adjustOrientation = true;

    if (adjustOrientation) { CorrectCameraOrientation(); }
    if (params.useSphereQueryToMoveRefernecePoint) {
      // recalc refdist
      refDist = (referencePoint.position - transform.translation).norm();
    }
    if (adjustDistance) { CorrectCameraDistance(refDist); }
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
  if (params.pinUpVector) { CorrectCameraUpVector(Vector3::UnitY()); }

  return;

  // 2. raycasts again
  //  - problem: rotation too high, and the new reference point is in the direction opposite of our movement
  //  - problem: surface at new reference point has much different normal 
  //    - make movement smaller, 
  //    - raycast range larger, 
  //    - and raycast density higher.

  // 3. at the end we might also store some type of orientation between the terrain and the camera ... 
  //  - and possibly bend it over time, so that the orientation to ground stays consistent, but
  //    does not move abruptly (hence blending)



  // Verify the camera position is valid and is not, e.g.
  //  - within the mesh
}


lmTransform CameraUtil::GetFinalCamera() {
  // Return either the detailed camera, or the blended hybrid camera
  if (1) {
    return transform;
  } else {
    return GetHybridCameraTransform();
  }
}

void CameraUtil::CorrectCameraOrientation() {
  lmReal adjustOrientationRate = 1.0f;//0.1f;
  if (params.smoothCameraOrientation) {
    adjustOrientationRate = params.smoothingFactor;
  }

  // Roation quaternion.
  Vector3 oldZ = transform.rotation * Vector3::UnitZ();
  Vector3 newZ = referencePoint.normal;

  // Adjust orientation and angle

  // Target orientation
  lmQuat dQ; dQ.setFromTwoVectors(oldZ, newZ);

  // Modify camera transform
  dQ = dQ.slerp((1.0f-adjustOrientationRate), lmQuat::Identity());
  transform.rotation = dQ * transform.rotation;

  Vector3 cameraFromRefPoint = transform.translation - referencePoint.position;
  transform.translation = referencePoint.position + dQ * cameraFromRefPoint;
}

void CameraUtil::CorrectCameraDistance(lmReal currentDistance) {
  lmReal smoothingRate = 0.2f;
  lmReal deltaDist = referenceDistance - currentDistance;
  Vector3 deltaTranslation = smoothingRate * deltaDist * (transform.rotation * Vector3::UnitZ()); // 'smoother' movement; add continues velocity though.
  transform.translation += deltaTranslation;
}

void CameraUtil::CorrectCameraUpVector(const Vector3& up) {

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
    lmReal adjustOrientationRate = 0.1f; //std::max(0.0f, -0.2f + 1.2f * std::fabs(xyPlaneNormalDotUp));

    // Apply correction.
    dQ = dQ.slerp((1.0f-adjustOrientationRate), lmQuat::Identity());
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
