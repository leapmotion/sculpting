#include "StdAfx.h"
#include "Sculpt.h"
#include "Utilities.h"
#include "AutoSave.h"
#include <algorithm>
#include <cinder/gl/gl.h>

float Sculpt::detail_ = 1.0f;
float Sculpt::d2Min_ = 0.0f;
float Sculpt::d2Max_ = 0.0f;
float Sculpt::d2Thickness_ = 0.0f;
float Sculpt::d2Move_ = 0.0f;

/** Constructor */
Sculpt::Sculpt() : mesh_(0), sculptMode_(INVALID), topoMode_(ADAPTIVE), lastSculptTime_(0.0),
  prevSculpt_(false), material_(0), materialColor_(Vector3::Ones()), autoSmoothStrength_(0.15f),
  remeshRadius_(-1.0f), lastUpdateTime_(0.0)
{}

/** Destructor */
Sculpt::~Sculpt()
{}

/** Set sculpting mode */
void Sculpt::setAdaptiveParameters(float radiusSquared)
{
  d2Max_ = radiusSquared*(1.1f-detail_)*0.2f;
  d2Min_ = d2Max_/4.2025f;
  d2Move_ = d2Min_*0.2375f;
  d2Thickness_ = (4.f*d2Move_ + d2Max_/3.f)*1.1f;
}

void Sculpt::remesh(float remeshRadius) {
  const float radius_sq = 200.0f * 200.0f;
  VertexVector &vertices = mesh_->getVertices();
  const int numVertices = vertices.size();
  std::vector<int> vertIndices;
  vertIndices.reserve(numVertices);
  for (int i=0; i<numVertices; i++) {
    vertIndices.push_back(i);
  }
  std::vector<int> iTris;
  mesh_->getTrianglesFromVertices(vertIndices, iTris);

  mesh_->pushState(iTris,vertIndices);

  topo_.init(mesh_, radius_sq, Vector3::Zero());
  setAdaptiveParameters(remeshRadius*remeshRadius);

  switch(topoMode_) {
  case ADAPTIVE :
  case UNIFORMISATION : topo_.uniformisation(iTris, d2Min_, d2Max_); break;
  case DECIMATION : topo_.decimation(iTris, d2Min_); break;
  case SUBDIVISION : topo_.subdivision(iTris, d2Max_); break;
  default : break;
  }

  if(topoMode_==ADAPTIVE) {
    topo_.adaptTopology(iTris, d2Thickness_);
  }

  mesh_->getVerticesFromTriangles(iTris, vertIndices);
  mesh_->updateMesh(iTris, vertIndices);
}

/**
* Sculpt the mesh. Main steps are :
* 1. Take a bigger region topologically (2-ring vertices)
* 2. Subdivide the triangles inside the brush radius (enlarged)
* 3. Cull the vertices if asked (back face culling according to vertex normal)
* 4. Select the vertices to be sculpted (not enlarged). Actually, old vertices to be sculpted
*      are tagged during the picking process, whereas newly created vertices are tagged
*      at the end of the subdivision process, so here, we just have to check the sculpt flag
* 5. Sculpting operator
* 6. Update the mesh (triangle's aabb, normals of triangle/vertices, octree and the VBO's)
*/
void Sculpt::sculptMesh(std::vector<int> &iVertsSelected, const Brush& brush)
{
  VertexVector &vertices = mesh_->getVertices();

  mesh_->getTrianglesFromVertices(iVertsSelected, iTris_);

  //undo-redo
  mesh_->pushState(iTris_,iVertsSelected);

  topo_.init(mesh_, brush._radius_squared, brush._position);
  setAdaptiveParameters(brush._radius_squared);

  switch(topoMode_) {
  case ADAPTIVE :
  case UNIFORMISATION : topo_.uniformisation(iTris_, d2Min_, d2Max_); break;
  case DECIMATION : topo_.decimation(iTris_, d2Min_); break;
  case SUBDIVISION : topo_.subdivision(iTris_, d2Max_); break;
  default : break;
  }

  mesh_->computeTriangleNormals(iTris_);

  mesh_->getVerticesFromTriangles(iTris_, iVertsSelected);

  MaskMatch pred(vertices);
  std::vector<int>::iterator it = std::remove_if(iVertsSelected.begin(), iVertsSelected.end(), pred);
  iVertsSelected.resize(it - iVertsSelected.begin());

  if (!iVertsSelected.empty()) {
    switch(sculptMode_) {
    case INFLATE: draw(mesh_, iVertsSelected, brush, false); break;
    case DEFLATE: draw(mesh_, iVertsSelected, brush, true); break;
    case SMOOTH: smooth(mesh_, iVertsSelected, brush); break;
    case FLATTEN: flatten(mesh_, iVertsSelected, brush); break;
    case SWEEP: sweep(mesh_, iVertsSelected, brush); break;
    case PUSH: push(mesh_, iVertsSelected, brush); break;
    case PAINT: paint(mesh_, iVertsSelected, brush, materialColor_); break;
    case CREASE: crease(mesh_, iVertsSelected, brush); break;
    default: break;
    }

    if (sculptMode_ == DEFLATE || sculptMode_ == SWEEP || sculptMode_ == PUSH || sculptMode_ == INFLATE) {
      Brush autoSmoothBrush(brush);
      autoSmoothBrush._strength *= autoSmoothStrength_;
      smooth(mesh_, iVertsSelected, autoSmoothBrush);
    }
  }

  if (topoMode_==ADAPTIVE) {
    topo_.adaptTopology(iTris_, d2Thickness_);
  }

  mesh_->getVerticesFromTriangles(iTris_, iVertsSelected);

  mesh_->checkVertices(iVertsSelected, d2Min_);

  mesh_->updateMesh(iTris_,iVertsSelected);
}

/** Smooth a group of vertices. New position is given by simple averaging */
void Sculpt::smooth(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush, bool limit)
{
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
  Vector3Vector smoothVerts(nbVerts, Vector3::Zero());
  Vector3Vector smoothColors(nbVerts, Vector3::Zero());
  laplacianSmooth(mesh, iVerts, smoothVerts, smoothColors);

  float dMove = sqrtf(d2Move_);
#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert = vertices[iVerts[i]];
    Vector3 displ = (smoothVerts[i]-vert)*brush._strength;
    vert.material_ = brush._strength*smoothColors[i] + (1.0f-brush._strength)*vert.material_;
    if (limit) {
      float displLength = displ.squaredNorm();
      if (displLength >= d2Move_) {
        displ *= (dMove/std::sqrt(displLength));
      }
    }
    vert += displ;
  }
}

/** Smooth a group of vertices along the plane defined by the normal of the vertex */
void Sculpt::smoothFlat(Mesh* mesh, const std::vector<int> &iVerts)
{
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
  Vector3Vector smoothVerts(nbVerts, Vector3::Zero());
  Vector3Vector smoothColors(nbVerts, Vector3::Zero());
  laplacianSmooth(mesh, iVerts,smoothVerts, smoothColors);

#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert = vertices[iVerts[i]];
    Vector3& vertSmo = smoothVerts[i];
    Vector3& n = vert.normal_;
    float dot = n.dot(vertSmo-vert);
    vert += (vertSmo - dot*n - vert);
    vert.material_ = smoothColors[i];
  }
}

/**
* Draw (inflate/deflate) a group of vertices. Move them according to the normal
* representative of all the sculpting vertices. I couldn't come up with a good
* falloff function, so it's just something that fall off strongly at the border of brush radius
*/
void Sculpt::draw(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush, bool negate)
{
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
  const float dMove = std::sqrt(d2Move_);
  const float deformationIntensity = brush._radius*0.05f;
  const float negationFactor = negate ? -1.0f : 1.0f;
#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert=vertices[iVerts[i]];
    const float strength = brush.strengthAt(vert);
    vert += vert.normal_ * negationFactor * std::min(dMove, deformationIntensity*strength);
  }
}

/** Compute average normal of a group of vertices with culling */
Vector3 Sculpt::areaNormal(Mesh* mesh, const std::vector<int> &iVerts)
{
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
  float areaX=0.f,areaY=0.f,areaZ=0.f;
#pragma omp parallel for reduction(+:areaX,areaY,areaZ)
  for (int i = 0; i<nbVerts; ++i)
  {
    const Vector3& normal = vertices[iVerts[i]].normal_;
    areaX+=normal.x();
    areaY+=normal.y();
    areaZ+=normal.z();
  }
  LM_ASSERT(nbVerts > 0, "Not enough points");
  Vector3 result(areaX, areaY, areaZ);
  float length = result.norm();
  if (length == 0.0f) {
    return Vector3::Zero();
  }
  return result/length;
}

/** Compute average center of a group of vertices with culling */
Vector3 Sculpt::areaCenter(Mesh* mesh, const std::vector<int> &iVerts)
{
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
  float areaX=0.f,areaY=0.f,areaZ=0.f;
#pragma omp parallel for reduction(+:areaX,areaY,areaZ)
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &v = vertices[iVerts[i]];
    areaX+=v.x();
    areaY+=v.y();
    areaZ+=v.z();
  }
  LM_ASSERT(nbVerts > 0, "Not enough points");
  return Vector3(areaX,areaY,areaZ)/static_cast<float>(nbVerts);
}

/**
* Flattening, projection of the sculpting vertex onto a plane
* defined by the barycenter and normals of all the sculpting vertices
*/
void Sculpt::flatten(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush)
{
  Vector3 areaNorm = areaNormal(mesh, iVerts);
  if(areaNorm.squaredNorm()<0.0001f)
    return;
  Vector3 areaPoint = areaCenter(mesh, iVerts);
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
  float deformationIntensity = 0.3f;
  const float dMove = std::sqrt(d2Move_);

#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &v = vertices[iVerts[i]];
    float distance = (v-areaPoint).dot(areaNorm);
    v -= areaNorm * std::min(dMove, distance*deformationIntensity*brush.strengthAt(v));
  }
}

/** Laplacian smooth. Special rule for vertex on the edge of the mesh. */
void Sculpt::laplacianSmooth(Mesh* mesh, const std::vector<int> &iVerts, Vector3Vector &smoothVerts, Vector3Vector &smoothColors)
{
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert = vertices[iVerts[i]];
    const std::vector<int> &ivRing = vert.ringVertices_;
    int nbVRing=ivRing.size();
    if(nbVRing!=(int)vert.tIndices_.size())
    {
      Vector3 center(Vector3::Zero());
      Vector3 color(Vector3::Zero());
      int nbVertEdge = 0;
      for(int j = 0; j<nbVRing; ++j)
      {
        Vertex &ivr = vertices[ivRing[j]];
        if(ivr.tIndices_.size()!=ivr.ringVertices_.size())
        {
          center+=ivr;
          ++nbVertEdge;
          color += ivr.material_;
        }
      }
      LM_ASSERT(nbVertEdge > 0, "Not enough verts");
      if (nbVertEdge > 0) {
        smoothVerts[i]=center/static_cast<float>(nbVertEdge);
        smoothColors[i]=color/static_cast<float>(nbVertEdge);
      }
    }
    else
    {
      Vector3 center(Vector3::Zero());
      Vector3 color(Vector3::Zero());
      for (int j=0;j<nbVRing;++j) {
        center+=vertices[ivRing[j]];
        color+=vertices[ivRing[j]].material_;
      }
      LM_ASSERT(nbVRing > 0, "Not enough verts");
      if (nbVRing > 0) {
        smoothVerts[i]=center/static_cast<float>(nbVRing);
        smoothColors[i]=color/static_cast<float>(nbVRing);
      }
    }
  }
}

/** Sweep deformation */
void Sculpt::sweep(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush)
{
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
  float deformationIntensity = brush._radius*0.0005f;
  const float velMag = brush._velocity.norm();
  const Vector3 normalizedVel = brush._velocity / velMag;
  const float dMove = std::sqrt(d2Move_);

#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert = vertices[iVerts[i]];
    const float strength = brush.strengthAt(vert);
    vert += std::min(dMove, deformationIntensity*strength*velMag)*normalizedVel;
  }
}

void Sculpt::push(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush)
{
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
  const float dMove = std::sqrt(d2Move_);
  const float deformationIntensity = brush._radius*0.25f;
#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert = vertices[iVerts[i]];
    const float strength = brush.strengthAt(vert);
    vert -= std::min(dMove, deformationIntensity*strength)*brush._direction;
  }
}

void Sculpt::crease(Mesh* mesh, const std::vector<int>& iVerts, const Brush& brush) {
  const Vector3 areaNorm = areaNormal(mesh, iVerts);
  if(areaNorm.squaredNorm()<0.0001f)
    return;
  VertexVector& vertices = mesh->getVertices();
  const int nbVerts = iVerts.size();
  const float dMove = std::sqrt(d2Move_);
  const float normalFactor = 30.0f;
  const Vector3& center = brush._position;

  for (int i = 0; i < nbVerts; ++i) {
    Vertex& vert = vertices[iVerts[i]];
    const float strength = brush.strengthAt(vert);
    const Vector3 displ = strength * ((center - vert) + strength*strength*normalFactor*areaNorm);
    float displLength = displ.squaredNorm();
    if (displLength<=d2Move_) {
      vert += displ;
    } else {
      vert += displ.normalized()*dMove;
    }
  }
}

void Sculpt::paint(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush, const Vector3& color) {
  VertexVector &vertices = mesh->getVertices();
  int nbVerts = iVerts.size();
#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert=vertices[iVerts[i]];
    const float changeSpeed = brush.strengthAt(vert);
    vert.material_ = (1.0f-changeSpeed)*vert.material_ + changeSpeed*color;
  }
}

void Sculpt::addBrush(const Vector3& worldPos, const Vector3& pos, const Vector3& dir, const Vector3& vel, float radius, float strength, float activation)
{
  _brushes.push_back(Brush());
  Brush& brush = _brushes.back();
  brush._worldPos = worldPos;
  brush._radius = radius;
  brush._radius_squared = radius*radius;
  brush._length = 0.0f;//30.0f;
  brush._strength = ci::math<float>::clamp(strength);
  brush._position = pos;
  brush._direction = dir;
  brush._velocity = vel;
  brush._activation = ci::math<float>::clamp(activation, 0.00001f, 0.99999f);

  LM_ASSERT(brush._radius > 0.00001f && radius < 9999999.0f, "Bad radius");
  LM_ASSERT(brush._strength >= 0.0f && strength <= 1.0f, "Bad strength");
  LM_ASSERT(fabs(brush._direction.norm() - 1.0f) < 0.0001f, "Bad direction");
  LM_ASSERT(brush._velocity.norm() > 0.00001f && brush._velocity.norm() < 9999999.0f, "Bad velocity");
  LM_ASSERT(brush._activation >= 0.0f && brush._activation <= 1.0f, "Bad activation");
}

void Sculpt::applyBrushes(double curTime, bool symmetry, AutoSave* autoSave)
{
  if (sculptMode_ == INVALID) {
    return;
  }

  static const float DESIRED_ANGLE_PER_SAMPLE = 0.03f;

  std::unique_lock<std::mutex> lock(brushMutex_);
  if (remeshRadius_ > 0) {
    remesh(remeshRadius_);
    remeshRadius_ = -1.0f;
  }

  const Vector3& origin = mesh_->getRotationOrigin();
  const Vector3& axis = mesh_->getRotationAxis();
  const float velocity = mesh_->getRotationVelocity();
  const float deltaTime = static_cast<float>(curTime - lastUpdateTime_);
  const float angle = deltaTime * velocity;
  const int numSamples = velocity > 0.001f ? static_cast<int>(std::ceil(angle / DESIRED_ANGLE_PER_SAMPLE)) : 1;
  const float timePerSample = deltaTime / numSamples;
  const float strengthMult = (1.0f + velocity) / numSamples;

  bool haveSculpt = false;

  double sampleTime = lastUpdateTime_;
  for (int i=0; i<numSamples; i++) {
    sampleTime += timePerSample;
    const Matrix4x4 transformInv = mesh_->getTransformation(sampleTime).inverse();

    for (size_t b=0; b<_brushes.size(); ++b) {
      if (symmetry) {
        brushVertices_.clear();
        Brush brush = _brushes[b].reflected(0).transformed(transformInv).withSpinVelocity(origin, axis, velocity);
        brush._strength *= strengthMult;

        mesh_->getVerticesInsideBrush(brush, brushVertices_);
        if (!brushVertices_.empty()) {
          if (!haveSculpt && !prevSculpt_) {
            mesh_->startPushState();
          }
          haveSculpt = true;
          sculptMesh(brushVertices_, brush);
        }
      }

      brushVertices_.clear();
      Brush brush = _brushes[b].transformed(transformInv).withSpinVelocity(origin, axis, velocity);
      brush._strength *= strengthMult;

      mesh_->getVerticesInsideBrush(brush, brushVertices_);
      if (!brushVertices_.empty()) {
        if (!haveSculpt && !prevSculpt_) {
          mesh_->startPushState();
        }
        haveSculpt = true;
        sculptMesh(brushVertices_, brush);
      }
    }
  }

  if (!haveSculpt && prevSculpt_) {
    autoSave->triggerAutoSave(mesh_);
    mesh_->checkLeavesUpdate();
    material_++;
  }
  mesh_->handleUndoRedo();

  prevSculpt_ = haveSculpt;

  lastUpdateTime_ = curTime;
  if (haveSculpt) {
    lastSculptTime_ = curTime;
  }
}

BrushVector Sculpt::getBrushes() const {
  std::unique_lock<std::mutex> lock(brushMutex_);
  return _brushes;
}

std::mutex& Sculpt::getBrushMutex() {
  return brushMutex_;
}
