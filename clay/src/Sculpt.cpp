#include "StdAfx.h"
#include "Sculpt.h"
#include "Utilities.h"
#include <algorithm>
#include <cinder/gl/gl.h>

/** Constructor */
Sculpt::Sculpt() : mesh_(0), sculptMode_(INVALID), topoMode_(ADAPTIVE),
  detail_(1.0f), d2Min_(0.f), d2Max_(0.f), d2Thickness_(0.f), d2Move_(0.f), lastSculptTime_(0),
  deltaTime_(0.0f), minDetailMult_(0.2f), prevSculpt_(false), material_(0), autoSmoothStrength_(0.25f)
{}

/** Destructor */
Sculpt::~Sculpt()
{}

/** Set sculpting mode */
void Sculpt::setAdaptiveParameters(float radiusSquared)
{
  d2Max_ = radiusSquared*(1.0f-detail_+minDetailMult_)*0.2f;
  d2Min_ = d2Max_/4.2025f;
  d2Move_ = d2Min_*0.2375f;
  d2Thickness_ = (4.f*d2Move_ + d2Max_/3.f)*1.1f;
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

  std::vector<int> iTris = mesh_->getTrianglesFromVertices(iVertsSelected);

  //undo-redo
  mesh_->pushState(iTris,iVertsSelected);

  Topology topo(mesh_,brush._radius_squared,brush._position);
  setAdaptiveParameters(brush._radius_squared);
  switch(topoMode_)
  {
  case ADAPTIVE :
  case UNIFORMISATION : topo.uniformisation(iTris, d2Min_, d2Max_); break;
  case DECIMATION : topo.decimation(iTris, d2Min_); break;
  case SUBDIVISION : topo.subdivision(iTris, d2Max_); break;
  default : break;
  }

  iVertsSelected = mesh_->getVerticesFromTriangles(iTris);

  MaskMatch pred(vertices);
  std::vector<int>::iterator it = std::remove_if(iVertsSelected.begin(), iVertsSelected.end(), pred);
  iVertsSelected.resize(it - iVertsSelected.begin());

  switch(sculptMode_)
  {
  case INFLATE :
  case DEFLATE : draw(iVertsSelected, brush); break;
  case SMOOTH : smooth(iVertsSelected, brush); break;
  case FLATTEN : flatten(iVertsSelected, brush); break;
  case SWEEP : sweep(iVertsSelected, brush); break;
  case PUSH : push(iVertsSelected, brush); break;
  case ERASE :
  case PAINT : paint(iVertsSelected, brush, material_); break;
  default: break;
  }

  if (sculptMode_ == DEFLATE || sculptMode_ == SWEEP || sculptMode_ == PUSH) {
    Brush autoSmoothBrush(brush);
    autoSmoothBrush._strength *= autoSmoothStrength_;
    smooth(iVertsSelected, autoSmoothBrush);
  }

  if(topoMode_==ADAPTIVE)
  {
    topo.adaptTopology(iTris, d2Thickness_);
    iVertsSelected = mesh_->getVerticesFromTriangles(iTris);
  }

  mesh_->updateMesh(iTris,iVertsSelected);
}

/** Smooth a group of vertices. New position is given by simple averaging */
void Sculpt::smooth(const std::vector<int> &iVerts, const Brush& brush)
{
  VertexVector &vertices = mesh_->getVertices();
  int nbVerts = iVerts.size();
  Vector3Vector smoothVerts(nbVerts, Vector3::Zero());
  Vector3Vector smoothColors(nbVerts, Vector3::Zero());
  laplacianSmooth(iVerts,smoothVerts, smoothColors);
  if(topoMode_!=ADAPTIVE)
  {
#pragma omp parallel for
    for (int i = 0; i<nbVerts; ++i)
    {
      Vertex &vert = vertices[iVerts[i]];
      vert += brush._strength*(smoothVerts[i]-vert);
      vert.material_ = brush._strength*smoothColors[i] + (1.0f-brush._strength)*vert.material_;
    }
  }
  else
  {
    float dMove = sqrtf(d2Move_);
#pragma omp parallel for
    for (int i = 0; i<nbVerts; ++i)
    {
      Vertex &vert = vertices[iVerts[i]];
      Vector3 displ = (smoothVerts[i]-vert)*brush._strength;
      vert.material_ = brush._strength*smoothColors[i] + (1.0f-brush._strength)*vert.material_;
      float displLength = displ.squaredNorm();
      if(displLength<=d2Move_)
        vert+=displ;
      else
        vert+=displ.normalized()*dMove;
    }
  }
}

/** Smooth a group of vertices along the plane defined by the normal of the vertex */
void Sculpt::smoothFlat(const std::vector<int> &iVerts)
{
  VertexVector &vertices = mesh_->getVertices();
  int nbVerts = iVerts.size();
  Vector3Vector smoothVerts(nbVerts, Vector3::Zero());
  Vector3Vector smoothColors(nbVerts, Vector3::Zero());
  laplacianSmooth(iVerts,smoothVerts, smoothColors);
#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert = vertices[iVerts[i]];
    Vector3& vertSmo = smoothVerts[i];
    Vector3& n = vert.normal_;
    vertSmo-=n*n.dot(vertSmo-vert);
    vert+=(vertSmo-vert);
    vert.material_ = smoothColors[i];
  }
}

/**
* Draw (inflate/deflate) a group of vertices. Move them according to the normal
* representative of all the sculpting vertices. I couldn't come up with a good
* falloff function, so it's just something that fall off strongly at the border of brush radius
*/
void Sculpt::draw(const std::vector<int> &iVerts, const Brush& brush)
{
  VertexVector &vertices = mesh_->getVertices();
  int nbVerts = iVerts.size();
  float deformationIntensity = brush._radius*0.1f;
  if(topoMode_==ADAPTIVE)
    deformationIntensity = std::min(sqrtf(d2Move_), deformationIntensity);
  if(sculptMode_==DEFLATE)
    deformationIntensity = -deformationIntensity;
#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert=vertices[iVerts[i]];
    vert+=vert.normal_*deformationIntensity*brush.strengthAt(vert);
  }
}

/** Compute average normal of a group of vertices with culling */
Vector3 Sculpt::areaNormal(const std::vector<int> &iVerts)
{
  VertexVector &vertices = mesh_->getVertices();
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
  assert(nbVerts > 0);
  Vector3 result(areaX, areaY, areaZ);
  float length = result.norm();
  if (length == 0.0f) {
    return Vector3::Zero();
  }
  return result/length;
}

/** Compute average center of a group of vertices with culling */
Vector3 Sculpt::areaCenter(const std::vector<int> &iVerts)
{
  VertexVector &vertices = mesh_->getVertices();
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
  assert(nbVerts > 0);
  return Vector3(areaX,areaY,areaZ)/static_cast<float>(nbVerts);
}

/**
* Flattening, projection of the sculpting vertex onto a plane
* defined by the barycenter and normals of all the sculpting vertices
*/
void Sculpt::flatten(const std::vector<int> &iVerts, const Brush& brush)
{
  Vector3 areaNorm = areaNormal(iVerts);
  if(areaNorm.squaredNorm()<0.0001f)
    return;
  Vector3 areaPoint = areaCenter(iVerts);
  VertexVector &vertices = mesh_->getVertices();
  int nbVerts = iVerts.size();
  float deformationIntensity = 0.3f;

#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &v = vertices[iVerts[i]];
    float distance = (v-areaPoint).dot(areaNorm);
    v -= areaNorm*distance*deformationIntensity*brush.strengthAt(v);
  }
}

/** Laplacian smooth. Special rule for vertex on the edge of the mesh. */
void Sculpt::laplacianSmooth(const std::vector<int> &iVerts, Vector3Vector &smoothVerts, Vector3Vector &smoothColors)
{
  VertexVector &vertices = mesh_->getVertices();
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
      if (nbVRing > 0) {
        smoothVerts[i]=center/static_cast<float>(nbVRing);
        smoothColors[i]=color/static_cast<float>(nbVRing);
      }
    }
  }
}

/** Smooth a group of vertices along the plane defined by the normal of the vertex (no openMP) */
void Sculpt::smoothNoMp(const std::vector<int> &iVerts, bool flat)
{
  VertexVector &vertices = mesh_->getVertices();
  int nbVerts = iVerts.size();
  Vector3Vector smoothVerts(nbVerts, Vector3::Zero());
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert = vertices[iVerts[i]];
    const std::vector<int> &ivRing = vert.ringVertices_;
    int nbVRing=ivRing.size();
    if(nbVRing!=(int)vert.tIndices_.size())
    {
      Vector3 center(Vector3::Zero());
      int nbVertEdge = 0;
      for(int j=0;j<nbVRing;++j)
      {
        Vertex &ivr = vertices[ivRing[j]];
        if(ivr.tIndices_.size()!=ivr.ringVertices_.size())
        {
          center+=ivr;
          ++nbVertEdge;
        }
      }
      assert(nbVertEdge > 0);
      smoothVerts[i]=center/static_cast<float>(nbVertEdge);
    }
    else
    {
      Vector3 center(Vector3::Zero());
      for (int j=0;j<nbVRing;++j) {
        center+=vertices[ivRing[j]];
      }
      assert(nbVRing > 0);
      smoothVerts[i]=center/static_cast<float>(nbVRing);
    }
  }
  if(!flat)
  {
    for (int i = 0; i<nbVerts; ++i) {
      vertices[iVerts[i]] = smoothVerts[i];
    }
  }
  else
  {
    for (int i = 0; i<nbVerts; ++i)
    {
      Vertex &vert = vertices[iVerts[i]];
      Vector3& vertSmo = smoothVerts[i];
      Vector3& n = vert.normal_;
      vertSmo -= n*n.dot(vertSmo-vert);
      vert = vertSmo;
    }
  }
}

/** Sweep deformation */
void Sculpt::sweep(const std::vector<int> &iVerts, const Brush& brush)
{
  VertexVector &vertices = mesh_->getVertices();
  int nbVerts = iVerts.size();
  float deformationIntensity = brush._radius*0.0005f;

#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert = vertices[iVerts[i]];
    vert += deformationIntensity*brush.velocityAt(vert);
  }
}

void Sculpt::push(const std::vector<int> &iVerts, const Brush& brush)
{
  VertexVector &vertices = mesh_->getVertices();
  int nbVerts = iVerts.size();
  float deformationIntensity = brush._radius*0.25f;
#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert = vertices[iVerts[i]];
    vert -= deformationIntensity*brush.pushPullAt(vert);
  }
}

void Sculpt::paint(const std::vector<int> &iVerts, const Brush& brush, int material) {
  VertexVector &vertices = mesh_->getVertices();
  int nbVerts = iVerts.size();
  const Vector3 newColor = sculptMode_ == PAINT ? Utilities::colorForIndex(material) : Vector3::Ones();
#pragma omp parallel for
  for (int i = 0; i<nbVerts; ++i)
  {
    Vertex &vert=vertices[iVerts[i]];
    const float changeSpeed = brush.strengthAt(vert);
    vert.material_ = (1.0f-changeSpeed)*vert.material_ + changeSpeed*newColor;
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
  brush._strength = strength;
  brush._position = pos;
  brush._direction = dir;
  brush._velocity = vel;
  brush._activation = activation;
}

void Sculpt::applyBrushes(double curTime, bool symmetry)
{
  if (sculptMode_ == INVALID) {
    return;
  }

  std::unique_lock<std::mutex> lock(brushMutex_);
  mesh_->handleUndoRedo();
  const Matrix4x4 transformInv = mesh_->getInverseTransformation();
  const Vector3& origin = mesh_->getRotationOrigin();
  const Vector3& axis = mesh_->getRotationAxis();
  float velocity = mesh_->getRotationVelocity();

  bool haveSculpt = false;
  for(size_t b=0; b<_brushes.size(); ++b)
  {
    if (symmetry) {
      brushVertices_.clear();
      Brush brush = _brushes[b].reflected(0).transformed(transformInv).withSpinVelocity(origin, axis, velocity);

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

    mesh_->getVerticesInsideBrush(brush, brushVertices_);
    if (!brushVertices_.empty()) {
      if (!haveSculpt && !prevSculpt_) {
        mesh_->startPushState();
      }
      haveSculpt = true;
      sculptMesh(brushVertices_, brush);
    }
  }
  if (!haveSculpt && prevSculpt_) {
    mesh_->checkLeavesUpdate();
    material_++;
  }

  prevSculpt_ = haveSculpt;

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
