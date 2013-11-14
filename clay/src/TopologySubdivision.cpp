#include "StdAfx.h"
#include "Topology.h"

/** Subdivide until the detail every selected triangles comply with a detail level */
void Topology::subdivision(std::vector<int> &iTris, float detailMaxSquared)
{
  int nbTriangles = triangles_.size();
  do
  {
    nbTriangles = triangles_.size();
    subdivide(iTris, detailMaxSquared);
  }
  while(nbTriangles!=(int)triangles_.size());
}

/**
* Subdivide a set of triangle. Main steps are :
* 1. Detect the triangles that need to be split, and at which edge the split should occur
* 2. Subdivide all those triangles (split them in two)
* 3. Take the 2-ring neighborhood of the triangles that have been split
* 4. Fill the triangles (just create an edge where it's needed)
* 5. Smooth newly created vertices (along the plane defined by their own normals)
* 6. Tag the newly created vertices if they are inside the sculpt brush radius
*/
void Topology::subdivide(std::vector<int> &iTris, float detailMaxSquared)
{
  verticesMap_.clear();
  int nbVertsInit = vertices_.size();
  int nbTrisInit = triangles_.size();

  std::vector<int> iTrisSubd, split;

  initSplit(iTris, iTrisSubd, split, detailMaxSquared);
  if(iTrisSubd.size()>20)
    mesh_->expandTriangles(iTrisSubd,3);

  //undo-redo
  std::vector<int> iVertsSubd;
  mesh_->getVerticesFromTriangles(iTrisSubd, iVertsSubd);
  mesh_->pushState(iTrisSubd, iVertsSubd);

  split.resize(iTrisSubd.size(),0);
  subdivideTriangles(iTrisSubd, split, detailMaxSquared);

  std::vector<int> newTriangle;
  int nbTriangles = triangles_.size();
  for(int i = nbTrisInit; i<nbTriangles; ++i)
    newTriangle.push_back(i);

  mesh_->expandTriangles(newTriangle,1);

  //undo-redo
  iTrisSubd = std::vector<int>(newTriangle.begin() + (nbTriangles - nbTrisInit), newTriangle.end());
  mesh_->getVerticesFromTriangles(iTrisSubd, iVertsSubd);
  mesh_->pushState(iTrisSubd, iVertsSubd);

  iTris.insert(iTris.end(),newTriangle.begin(),newTriangle.end());

  std::vector<int> iTrisTemp;
  int nbTrisTemp = iTris.size();
  ++Triangle::tagMask_;
  for(int i=0;i<nbTrisTemp;++i)
  {
    int iTri = iTris[i];
    Triangle &t = triangles_[iTri];
    if(t.tagFlag_==Triangle::tagMask_)
      continue;
    t.tagFlag_ = Triangle::tagMask_;
    iTrisTemp.push_back(iTri);
  }
  iTris = iTrisTemp;

  int nbTrianglesOld = triangles_.size();
  while(newTriangle.size()>0)
    fillTriangles(newTriangle);

  nbTriangles = triangles_.size();
  for(int i = nbTrianglesOld; i<nbTriangles; ++i)
    iTris.push_back(i);

  std::vector<int> vNew;
  int nbVertices = vertices_.size();
  for(int i = nbVertsInit; i<nbVertices; ++i)
    vNew.push_back(i);

  int nbVNew = vNew.size();
  mesh_->expandVertices(vNew,1);
  Sculpt sc;
  sc.setMesh(mesh_);
  sc.smoothFlat(std::vector<int>(vNew.begin() + nbVNew, vNew.end()));

  nbVNew = vNew.size();
#pragma omp parallel for
  for(int i=0;i<nbVNew;++i)
  {
    if((vertices_[vNew[i]]-centerPoint_).squaredNorm()<radiusSquared_)
      vertices_[vNew[i]].sculptFlag_ = Vertex::sculptMask_;
    else
      vertices_[vNew[i]].sculptFlag_ = Vertex::sculptMask_-1;
  }
}

/** Detect which triangles to split and the edge that need to be split */
void Topology::initSplit(std::vector<int> &iTris, std::vector<int> &iTrisSubd, std::vector<int> &split, float detailMaxSquared)
{
  const int nbTris = iTris.size();
  for(int i=0;i<nbTris;++i)
  {
    const int splitNum = findSplit(iTris[i], detailMaxSquared, true);
    if (splitNum > 0) {
      split.push_back(splitNum);
      iTrisSubd.push_back(iTris[i]);
    }
  }
}

/** Find the edge to be split (0 otherwise) */
int Topology::findSplit(int iTri, float detailMaxSquared, bool checkInsideSphere)
{
  const Triangle &t = triangles_[iTri];
  const Vertex &v1 = vertices_[t.vIndices_[0]];
  const Vertex &v2 = vertices_[t.vIndices_[1]];
  const Vertex &v3 = vertices_[t.vIndices_[2]];

  if(checkInsideSphere)
    if(!Geometry::sphereIntersectTriangle(centerPoint_, radiusSquared_*2.f, v1, v2, v3))
      if(!Geometry::pointInsideTriangle(centerPoint_, v1, v2, v3))
        return 0;

  const float length1 = (v1-v2).squaredNorm();
  const float length2 = (v2-v3).squaredNorm();
  const float length3 = (v1-v3).squaredNorm();

  float max = length1;
  int idx = 1;
  if (length2 > max) {
    max = length2;
    idx = 2;
  }
  if (length3 > max) {
    max = length3;
    idx = 3;
  }
  return max > detailMaxSquared ? idx : 0;
}

/** Subdivide all the triangles that need to be subdivided */
void Topology::subdivideTriangles(std::vector<int> &iTrisSubd, std::vector<int> &split, float detailMaxSquared)
{
  int nbTris = iTrisSubd.size();
  for(int i=0; i<nbTris;++i)
  {
    Triangle &t = triangles_[iTrisSubd[i]];
    int iv1 = t.vIndices_[0];
    int iv2 = t.vIndices_[1];
    int iv3 = t.vIndices_[2];
    if(split[i]!=0)
    {
      if(split[i]==1)
        halfEdgeSplit(iTrisSubd[i],iv1,iv2,iv3);
      else if(split[i]==2)
        halfEdgeSplit(iTrisSubd[i],iv2,iv3,iv1);
      else
        halfEdgeSplit(iTrisSubd[i],iv3,iv1,iv2);
    }
    else
    {
      int splitNum = findSplit(iTrisSubd[i], detailMaxSquared);
      if(splitNum==1)
        halfEdgeSplit(iTrisSubd[i],iv1,iv2,iv3);
      else if(splitNum==2)
        halfEdgeSplit(iTrisSubd[i],iv2,iv3,iv1);
      else if(splitNum==3)
        halfEdgeSplit(iTrisSubd[i],iv3,iv1,iv2);
    }
  }
}


/**
* Subdivide one triangle, it simply cut the triangle in two at a given edge.
* The position of the vertex is computed as follow :
* 1. Initial position of the new vertex at the middle of the edge
* 2. Compute normal of the new vertex (average of the two normals of the two vertices defining the edge)
* 3. Compute angle between those two normals
* 4. Move the new vertex along its normal with a strengh proportional to the angle computed at step 3.
*/
void Topology::halfEdgeSplit(int iTri, int iv1, int iv2, int iv3)
{
  Triangle &t = triangles_[iTri];
  Octree *leaf = t.leaf_;
  std::vector<int> &iTrisLeaf = leaf->getTriangles();
  Vertex &v1 = vertices_[iv1];
  Vertex &v2 = vertices_[iv2];
  Vertex &v3 = vertices_[iv3];

  std::pair<std::map<std::pair<int,int>,int>::iterator, bool> pair;
  std::pair<std::pair<int,int>,int> entry;
  entry.first.first = std::min(iv1,iv2);
  entry.first.second = std::max(iv1,iv2);
  entry.second = vertices_.size();
  pair = verticesMap_.insert(entry);
  int ivMid=(*pair.first).second;

  v3.addRingVertex(ivMid);
  int iNewTri = triangles_.size();
  t.vIndices_[0] = iv1;
  t.vIndices_[1] = ivMid;
  t.vIndices_[2] = iv3;
  Triangle newTri = Triangle(Vector3::Zero(),ivMid,iv2,iv3,iNewTri);
  newTri.stateFlag_ = Mesh::stateMask_;
  v3.addTriangle(iNewTri);
  v2.replaceTriangle(iTri,iNewTri);
  newTri.leaf_ = leaf;
  newTri.posInLeaf_ = iTrisLeaf.size();
  if(pair.second) //new vertex
  {
    Vertex vMidTest((v1+v2)*0.5f,vertices_.size());
    vMidTest.material_ = 0.5f*(v1.material_+v2.material_);
    float dot = v1.normal_.dot(v2.normal_);
    float angle;
    if(dot<=-1.f) angle = static_cast<float>(M_PI);
    else if(dot>=1.f) angle = 0.f;
    else angle = acosf(dot);
    vMidTest.normal_ = (v1.normal_+v2.normal_).normalized();
    assert(fabs(vMidTest.normal_.squaredNorm() - 1.0f) < 0.001f);
    Vector3 edge = v1-v2;
    if((edge.dot(v1.normal_)-edge.dot(v2.normal_))>=0.f)
      vMidTest+=vMidTest.normal_*edge.norm()*angle*0.12f;
    else
      vMidTest-=vMidTest.normal_*edge.norm()*angle*0.12f;
    vMidTest.stateFlag_ = Mesh::stateMask_;
    vMidTest.addRingVertex(iv1);
    vMidTest.addRingVertex(iv2);
    vMidTest.addRingVertex(iv3);
    v1.replaceRingVertex(iv2,ivMid);
    v2.replaceRingVertex(iv1,ivMid);
    vMidTest.addTriangle(iTri);
    vMidTest.addTriangle(iNewTri);
    vertices_.push_back(vMidTest);
  }
  else
  {
    Vertex &vm = vertices_[ivMid];
    vm.addRingVertex(iv3);
    vm.addTriangle(iTri);
    vm.addTriangle(iNewTri);
  }
  iTrisLeaf.push_back(iNewTri);
  triangles_.push_back(newTri);
}

/**
* Fill the triangles. It checks if a newly vertex has been created at the middle
* of the edge. If several split are needed, it first chooses the split that minimize
* the valence of the vertex.
*/
void Topology::fillTriangles(std::vector<int> &iTris)
{
  int nbTris = iTris.size();
  std::vector<int> iTrisNext;
  for(int i=0; i<nbTris;++i)
  {
    int iTri = iTris[i];
    Triangle &t = triangles_[iTri];
    int iv1 = t.vIndices_[0];
    int iv2 = t.vIndices_[1];
    int iv3 = t.vIndices_[2];
    Vertex &v1 = vertices_[iv1];
    Vertex &v2 = vertices_[iv2];
    Vertex &v3 = vertices_[iv3];
    bool find1 = false;
    bool find2 = false;
    bool find3 = false;

    std::pair<int,int> entry1;
    entry1.first = std::min(iv1,iv2);
    entry1.second = std::max(iv1,iv2);
    std::map<std::pair<int,int>,int>::iterator it1;
    it1 = verticesMap_.find(entry1);
    find1 = it1!=verticesMap_.end();

    std::pair<int,int> entry2;
    entry2.first = std::min(iv2,iv3);
    entry2.second = std::max(iv2,iv3);
    std::map<std::pair<int,int>,int>::iterator it2;
    it2 = verticesMap_.find(entry2);
    find2 = it2!=verticesMap_.end();

    std::pair<int,int> entry3;
    entry3.first = std::min(iv1,iv3);
    entry3.second = std::max(iv1,iv3);
    std::map<std::pair<int,int>,int>::iterator it3;
    it3 = verticesMap_.find(entry3);
    find3 = it3!=verticesMap_.end();

    int num1 = v1.ringVertices_.size();
    int num2 = v2.ringVertices_.size();
    int num3 = v3.ringVertices_.size();
    int split = 0;
    if(find1)
    {
      if(find2)
      {
        if(find3)
        {
          if(num1<num2 && num1<num3) split = 2;
          else if(num2<num3) split = 3;
          else split = 1;
        }
        else if(num1<num3) split = 2;
        else split = 1;
      }
      else if(find3 && num2<num3) split = 3;
      else split = 1;
    }
    else if(find2)
    {
      if(find3 && num2<num1) split = 3;
      else split = 2;
    }
    else if(find3) split = 3;

    if(split==1)
      fillTriangle(iTri,iv1,iv2,iv3,it1->second);
    else if(split==2)
      fillTriangle(iTri,iv2,iv3,iv1,it2->second);
    else if(split==3)
      fillTriangle(iTri,iv3,iv1,iv2,it3->second);
    else continue;
    iTrisNext.push_back(iTri);
    iTrisNext.push_back(triangles_.size()-1);
  }
  iTris = iTrisNext;
}

/** Fill crack on one triangle */
void Topology::fillTriangle(int iTri, int iv1, int iv2, int iv3, int ivMid)
{
  Triangle &t = triangles_[iTri];
  Octree *leaf = t.leaf_;
  std::vector<int> &iTrisLeaf = leaf->getTriangles();
  Vertex &v2 = vertices_[iv2];
  Vertex &v3 = vertices_[iv3];
  Vertex &vMid = vertices_[ivMid];
  vMid.addRingVertex(iv3);
  v3.addRingVertex(ivMid);
  int iNewTri = triangles_.size();
  vMid.addTriangle(iTri);
  vMid.addTriangle(iNewTri);
  t.vIndices_[0] = iv1;
  t.vIndices_[1] = ivMid;
  t.vIndices_[2] = iv3;
  Triangle newTri = Triangle(Vector3::Zero(),ivMid,iv2,iv3,iNewTri);
  newTri.stateFlag_ = Mesh::stateMask_;
  v3.addTriangle(iNewTri);
  v2.replaceTriangle(iTri,iNewTri);
  newTri.leaf_ = leaf;
  newTri.posInLeaf_ = iTrisLeaf.size();
  iTrisLeaf.push_back(iNewTri);
  triangles_.push_back(newTri);
}
