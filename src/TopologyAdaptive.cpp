#include "StdAfx.h"
#include "Topology.h"
#include "Sculpt.h"

/** Adapt topology */
void Topology::adaptTopology(std::vector<int> &iTris, float d2Thickness)
{
  std::vector<int> iVerts;
  mesh_->getVerticesFromTriangles(iTris, iVerts);
  int nbVerts = iVerts.size();
  std::vector<int> vec;
  int nbVertices = vertices().size();
  for(int i = 0; i<nbVerts; ++i)
  {
    int iVert = iVerts[i];
    if(iVert>=nbVertices)
      continue;
    Vertex &v = vertices()[iVert];
    if((v-centerPoint_).squaredNorm()<radiusSquared_)
      vec.push_back(iVert);
  }
  checkCollisions(vec, d2Thickness);
  std::vector<int> newTris;
  mesh_->getTrianglesFromVertices(iVertsDecimated_, newTris);
  iTris.insert(iTris.end(), newTris.begin(), newTris.end());

  std::vector<int> iTrisTemp;
  int nbTrisTemp = iTris.size();
  int nbTriangles = triangles().size();
  ++Triangle::tagMask_;
  for(int i = 0; i<nbTrisTemp; ++i)
  {
    int iTri = iTris[i];
    if(iTri>=nbTriangles)
      continue;
    Triangle &t = triangles()[iTri];
    if(t.tagFlag_==Triangle::tagMask_)
      continue;
    t.tagFlag_ = Triangle::tagMask_;
    iTrisTemp.push_back(iTri);
  }
  iTris = iTrisTemp;
}

/** Check incoming collisions */
void Topology::checkCollisions(std::vector<int> &iVerts, float d2Thickness)
{
  iVertsDecimated_.clear();
  iTrisToDelete_.clear();
  iVertsToDelete_.clear();
  float r2Thickness = d2Thickness * 0.25f; //squared diameter to radius squared

  Aabb aabb;
  int nbVerts = iVerts.size();
  if(nbVerts>0)
  {
    aabb.min_ = vertices()[iVerts[0]];
    aabb.max_ = vertices()[iVerts[0]];
  }
  for(int i = 0; i<nbVerts; ++i) {
    aabb.expand(vertices()[iVerts[i]]);
  }

  grid_.init(aabb, sqrtf(r2Thickness));
  grid_.build(mesh_, iVerts);

  std::vector<int> iNearVerts;

  for(int i = 0; i<nbVerts; ++i)
  {
    int iVert = iVerts[i];
    Vertex &v = vertices()[iVert];
    if(v.tagFlag_<0)
      continue;
    std::vector<int> &ring = v.ringVertices_;
    int nbRing = ring.size();
    ++Vertex::tagMask_;
    for(int j=0;j<nbRing;++j)
      vertices()[ring[j]].tagFlag_ = Vertex::tagMask_;

    grid_.getNeighborhood(v, iNearVerts);
    int nbNearVerts = iNearVerts.size();
    for(int j = 0; j<nbNearVerts; ++j)
    {
      int jVert = iNearVerts[j];
      if(iVert==jVert)
        continue;
      Vertex &vTest = vertices()[jVert];
      if(vTest.tagFlag_<0 || vTest.tagFlag_==Vertex::tagMask_)
        continue;
      if((v-vTest).squaredNorm()<r2Thickness)
      {
        if(nbRing>(int)vTest.ringVertices_.size())
          vertexJoin(iVert,jVert);
        else
          vertexJoin(jVert,iVert);
        break;
      }
    }
  }

  applyDeletion();

  std::vector<int> iVertsDecimated;
  getValidModifiedVertices(iVertsDecimated);
  iVertsDecimated_ = iVertsDecimated;

  int nbVertsDecimated = iVertsDecimated_.size();
  std::vector<int> vSmooth;
  for(int i=0;i<nbVertsDecimated;++i)
  {
    int iv = iVertsDecimated_[i];
    if(vertices()[iv].sculptFlag_==Vertex::sculptMask_)
      vSmooth.push_back(iv);
  }
  mesh_->expandVertices(vSmooth,1);
  Brush brush;
  brush._strength = 1.0f;
  Sculpt::smooth(mesh_, vSmooth, brush, false);
}

/** Vertex joint */
void Topology::vertexJoin(int iv1, int iv2)
{
  Vertex &v1 = vertices()[iv1];
  Vertex &v2 = vertices()[iv2];

  std::vector<int> &iTris1 = v1.tIndices_;
  std::vector<int> &iTris2 = v2.tIndices_;
  std::vector<int> ring1 = v1.ringVertices_;
  std::vector<int> ring2 = v2.ringVertices_;
  int nbRing1 = ring1.size();
  int nbRing2 = ring2.size();

  //undo-redo
  mesh_->pushState(iTris1, ring1);
  mesh_->pushState(iTris2, ring2);
  if(v1.stateFlag_!=Mesh::stateMask_) { v1.stateFlag_ = Mesh::stateMask_; mesh_->getVerticesState().push_back(v1); }
  if(v2.stateFlag_!=Mesh::stateMask_) { v2.stateFlag_ = Mesh::stateMask_; mesh_->getVerticesState().push_back(v2); }

  std::vector<Edge> edges1,edges2;

  trianglesRotate(v1.tIndices_,iv1,edges1);
  if(!adjustEdgeOrientation(edges1))
    return;

  trianglesRotate(v2.tIndices_,iv2,edges2);
  if(!adjustEdgeOrientation(edges2))
    return;

  std::sort(ring1.begin(),ring1.end());
  std::sort(ring2.begin(),ring2.end());
  std::vector<int> common(std::max(nbRing1, nbRing2),-1);
  std::set_intersection(ring1.begin(),ring1.end(),ring2.begin(),ring2.end(),common.begin());

  int nbCommon = common.size();
  for(int i=0;i<nbCommon;++i)
  {
    if(common[i]==-1)
    {
      common.resize(i);
      break;
    }
  }

  if(common.empty()) {
    connect1Ring(edges1,edges2);
  } else {
    connect1RingCommonVertices(edges1, edges2, common);
  }

  for(int i = 0; i<nbRing1; ++i) mesh_->computeRingVertices(ring1[i]);
  for(int i = 0; i<nbRing2; ++i) mesh_->computeRingVertices(ring2[i]);

  iVertsDecimated_.push_back(iv1);
  iVertsDecimated_.push_back(iv2);
  iVertsToDelete_.push_back(iv1);
  iVertsToDelete_.push_back(iv2);
  v1.tagFlag_ = -1;
  v2.tagFlag_ = -1;

  cleanUpNeighborhood(ring1,ring2);
}

/** Connect two 1-ring with vertices in common */
void Topology::connect1RingCommonVertices(std::vector<Edge> &edges1, std::vector<Edge> &edges2, std::vector<int> &common)
{
  int nbEdges1 = edges1.size();
  int nbEdges2 = edges2.size();
  int nbCommon = common.size();

  ++Vertex::tagMask_;
  for(int i = 0; i<nbCommon; ++i)
    vertices()[common[i]].tagFlag_ = Vertex::tagMask_;

  //delete triangles
  for(int i = 0; i<nbEdges1; ++i)
  {
    Vertex &v1 = vertices()[edges1[i].v1_];
    Vertex &v2 = vertices()[edges1[i].v2_];
    if(v1.tagFlag_==Vertex::tagMask_ && v2.tagFlag_==Vertex::tagMask_)
    {
      int iTri = edges1[i].t_;
      v1.removeTriangle(iTri);
      v2.removeTriangle(iTri);
      triangles()[iTri].tagFlag_ = -1;
      iTrisToDelete_.push_back(iTri);
    }
  }
  for(int i = 0; i<nbEdges2; ++i)
  {
    Vertex &v1 = vertices()[edges2[i].v1_];
    Vertex &v2 = vertices()[edges2[i].v2_];
    if(v1.tagFlag_==Vertex::tagMask_ && v2.tagFlag_==Vertex::tagMask_)
    {
      int iTri = edges2[i].t_;
      v1.removeTriangle(iTri);
      v2.removeTriangle(iTri);
      triangles()[iTri].tagFlag_ = -1;
      iTrisToDelete_.push_back(iTri);
    }
  }

  std::vector<std::vector<Edge> > subEdges1;
  std::vector<std::vector<Edge> > subEdges2;
  subEdges1.push_back(std::vector<Edge>());
  subEdges2.push_back(std::vector<Edge>());

  matchEdgesCommonVertices(edges1, edges2, common.front());

  for(int i = 0; i<nbEdges1; ++i)
  {
    Vertex &v1 = vertices()[edges1[i].v1_];
    Vertex &v2 = vertices()[edges1[i].v2_];
    if(v1.tagFlag_!=Vertex::tagMask_ || v2.tagFlag_!=Vertex::tagMask_ )
      subEdges1.back().push_back(edges1[i]);
    if(v2.tagFlag_==Vertex::tagMask_ && subEdges1.back().size()!=0)
      subEdges1.push_back(std::vector<Edge>());
  }
  for(int i = 0; i<nbEdges2; ++i)
  {
    Vertex &v1 = vertices()[edges2[i].v1_];
    Vertex &v2 = vertices()[edges2[i].v2_];
    if(v1.tagFlag_!=Vertex::tagMask_ || v2.tagFlag_!=Vertex::tagMask_ )
      subEdges2.back().push_back(edges2[i]);
    if(v2.tagFlag_==Vertex::tagMask_ && subEdges2.back().size()!=0)
      subEdges2.push_back(std::vector<Edge>());
  }

  if(subEdges1.back().size()==0)
    subEdges1.pop_back();
  if(subEdges2.back().size()==0)
    subEdges2.pop_back();

  int nbSubEdges1 = subEdges1.size();
  int nbSubEdges2 = subEdges2.size();

  //connect linked edges (loops)
  int i2 = nbSubEdges2-1;
  int i1 = 0;
  int hackCount = 0;
  while(i1<nbSubEdges1 || i2>=0)
  {
    int iv1 = -1;
    int iv2 = -1;
    if (100000 < hackCount++) { break; }
    if(i1<nbSubEdges1)
      iv1 = subEdges1[i1].front().v1_;
    if(i2>=0)
      iv2 = subEdges2[i2].back().v2_;
    if(iv1==iv2)
    {
      if(subEdges1[i1].size()>subEdges2[i2].size())
        connectLinkedEdges(subEdges1[i1], subEdges2[i2]);
      else
        connectLinkedEdges(subEdges2[i2], subEdges1[i1]);
      ++i1;
      --i2;
    }
    else
    {
      for(int i = 0; i<nbEdges1; ++i)
      {
        int ivTest = edges1[i].v1_;
        if(ivTest==iv1)
        {
          connectLoop(subEdges1[i1]);
          ++i1;
          break;
        }
        else if(ivTest==iv2)
        {
          connectLoop(subEdges2[i2]);
          --i2;
          break;
        }
      }
    }
  }
}

/** Connect disconnected 1-ring */
void Topology::connect1Ring(std::vector<Edge> &edges1, std::vector<Edge> &edges2)
{
  matchEdgesNearest(edges1,edges2);
  int nbEdges1 = edges1.size();
  int nbEdges2 = edges2.size();
  LM_ASSERT(nbEdges1 > 0, "Not enough edges");
  LM_ASSERT(nbEdges2 > 0, "Not enough edges");
  float step = static_cast<float>(nbEdges2)/static_cast<float>(nbEdges1);
  int temp = 0;
  if(nbEdges1==nbEdges2)
    temp = -1;
  for(int i=0;i<nbEdges1;++i)
  {
    int j = 0;
    if(i!=0)
      j = (int)(nbEdges2 - step*i);
    triangles()[edges1[i].t_].vIndices_[2] = edges2[j].v1_;
    vertices()[edges2[j].v1_].addTriangle(edges1[i].t_);
    if(j!=temp)
    {
      triangles()[edges2[j].t_].vIndices_[2] = edges1[i].v1_;
      vertices()[edges1[i].v1_].addTriangle(edges2[j].t_);
    }
    temp = j;
  }
}

/** Connect two edges that are linked together at their ends, delete two triangles and connect the edges */
void Topology::connectLinkedEdges(std::vector<Edge> &edges1, std::vector<Edge> &edges2)
{
  int iTri1 = edges2.front().t_;
  vertices()[edges2.front().v1_].removeTriangle(iTri1);
  vertices()[edges2.front().v2_].removeTriangle(iTri1);

  int iTri2 = edges2.back().t_;
  vertices()[edges2.back().v1_].removeTriangle(iTri2);
  vertices()[edges2.back().v2_].removeTriangle(iTri2);

  int nbEdges1 = edges1.size();
  int nbEdges2 = edges2.size();
  float step = (float) nbEdges2/nbEdges1;
  int temp = 0;
  for(int i = 0; i<nbEdges1; ++i)
  {
    int j = (int)(nbEdges2 - step*(i+1));
    if(j<1)
      j = 1;
    if (j < nbEdges2)
    {
      triangles()[edges1[i].t_].vIndices_[2] = edges2[j].v1_;
      vertices()[edges2[j].v1_].addTriangle(edges1[i].t_);
      if(j!=temp && j!=0 && j!=(nbEdges2-1))
      {
        triangles()[edges2[j].t_].vIndices_[2] = edges1[i].v1_;
        vertices()[edges1[i].v1_].addTriangle(edges2[j].t_);
      }
    }
    temp = j;
  }
  triangles()[iTri1].tagFlag_ = -1;
  triangles()[iTri2].tagFlag_ = -1;
  iTrisToDelete_.push_back(iTri1);
  iTrisToDelete_.push_back(iTri2);
}

/** Connect a single loop, delete one triangle and fill the loop */
void Topology::connectLoop(std::vector<Edge> &edges)
{
  int nbEdges = edges.size();
  int iTri = edges.front().t_;
  vertices()[edges.front().v1_].removeTriangle(iTri);
  vertices()[edges.front().v2_].removeTriangle(iTri);
  int iv = edges.front().v1_;
  Vertex &v = vertices()[iv];
  for(int i = 1; i<nbEdges; ++i)
  {
    triangles()[edges[i].t_].vIndices_[2] = iv;
    v.addTriangle(edges[i].t_);
  }
  triangles()[iTri].tagFlag_ = -1;
  iTrisToDelete_.push_back(iTri);
}

/** Change indices order */
void Topology::trianglesRotate(std::vector<int> &iTris, int iv, std::vector<Edge> &edges)
{
  int nbTris = iTris.size();
  for(int i=0;i<nbTris;++i)
  {
    Triangle &t = triangles()[iTris[i]];
    int &iv1 = t.vIndices_[0];
    int &iv2 = t.vIndices_[1];
    int &iv3 = t.vIndices_[2];
    if(iv3!=iv)
    {
      if(iv1==iv)
      {
        iv1 = iv2;
        iv2 = iv3;
        iv3 = iv;
      }
      else
      {
        iv2 = iv1;
        iv1 = iv3;
        iv3 = iv;
      }
    }
    edges.push_back(Edge(iv1,iv2,iTris[i]));
  }
}

/** Adjust edges orientation, it makes a loop. For singular edge and singular vertex, it makes consecutive loops */
bool Topology::adjustEdgeOrientation(std::vector<Edge> &edges)
{
  int nbEdges = edges.size();
  if (nbEdges < 2)
  {
    return false;
  }
  
  int temp = -1;
  int j;
  int vFirst = edges.front().v1_;
  for(int i = 0; i<(nbEdges-1); i++)
  {
    Edge &e = edges[i];
    int vNext = e.v2_;
    temp = -1;
    for(j = i+1; j<nbEdges; ++j)
    {
      if(edges[j].v1_==vNext)
      {
        if(edges[j].v2_!=vFirst)
          temp = j;
        else
        {
          std::swap(edges[i+1],edges[j]);
          ++i;
          if((i+1)<nbEdges)
            vFirst = edges[i+1].v1_;
          break;
        }
      }
    }
    if(j==nbEdges) //non singular vertex
    {
      if(temp==-1)
        return false;
      std::swap(edges[i+1],edges[temp]);
    }
  }
  return true;
}

/** Adjust edges orientation starting from a vertex in common */
void Topology::matchEdgesCommonVertices(std::vector<Edge> &edges1, std::vector<Edge> &edges2, int ivCommon)
{
  int nbEdges1 = edges1.size();
  int match = -1;
  for(int i = 0; i<nbEdges1; ++i)
  {
    if(edges1[i].v1_==ivCommon)
    {
      match = i;
      break;
    }
  }
  LM_ASSERT(match >= 0, "no match");
  std::rotate(edges1.begin(), edges1.begin()+match, edges1.end());

  int nbEdges2 = edges2.size();
  match = -1;
  for(int i = 0; i<nbEdges2; ++i)
  {
    if(edges2[i].v1_==ivCommon)
    {
      match = i;
      break;
    }
  }
  LM_ASSERT(match >= 0, "no match");
  std::rotate(edges2.begin(), edges2.begin()+match, edges2.end());
}

/** Adjust edges orientation taking the closest vertex as a starting point */
void Topology::matchEdgesNearest(std::vector<Edge> &edges1, std::vector<Edge> &edges2)
{
  int nearest = 0;
  Vertex &v = vertices()[edges1.front().v1_];
  float minDist = (v-vertices()[edges2.front().v1_]).squaredNorm();
  int nbEdges2 = edges2.size();
  for(int i = 1; i<nbEdges2; ++i)
  {
    float distTest = (v-vertices()[edges2[i].v1_]).squaredNorm();
    if(distTest<minDist)
    {
      minDist = distTest;
      nearest = i;
    }
  }
  std::rotate(edges2.begin(), edges2.begin()+nearest, edges2.end());
}

/** Clean up neighborhood */
void Topology::cleanUpNeighborhood(const std::vector<int> &ring1, const std::vector<int> &ring2)
{
  int nbRing1 = ring1.size();
  for(int i = 0; i<nbRing1; ++i)
    cleanUpSingularVertex(ring1[i]);
  int nbRing2 = ring2.size();
  for(int i = 0; i<nbRing2; ++i)
    cleanUpSingularVertex(ring2[i]);
}

/** Split a degenerate vertex in two */
void Topology::cleanUpSingularVertex(int iv)
{
  Vertex &v = vertices()[iv];

  if(v.tagFlag_<0) //vertex to be deleted
    return;

  //undo-redo
  mesh_->pushState(v.tIndices_,v.ringVertices_);
  if(v.stateFlag_!=Mesh::stateMask_) { v.stateFlag_ = Mesh::stateMask_; mesh_->getVerticesState().push_back(v); }

  if(deleteVertexIfDegenerate(iv))
    return;

  std::vector<Edge> edges;
  trianglesRotate(v.tIndices_,iv,edges);
  if(!adjustEdgeOrientation(edges))
    return;

  int nbEdges = edges.size();
  int vFirst = edges.front().v1_;
  int endLoop = -1;
  for(int i = 1; i<(nbEdges-1); ++i)
  {
    int vEdge = edges[i].v2_;
    if(vEdge==vFirst)
    {
      endLoop = i;
      break;
    }
  }
  if(endLoop == -1)
    return;

  int ivNew = vertices().size();
  Vertex vNew(v,ivNew);
  vNew.material_ = v.material_;
  LM_ASSERT(fabs(v.normal_.squaredNorm() - 1.0f) < 0.001f, "Bad normal");
  vNew.normal_ = -v.normal_;
  vNew.stateFlag_ = Mesh::stateMask_;

  for(int i = 0; i<=endLoop; ++i)
  {
    int iTri = edges[i].t_;
    vNew.addTriangle(iTri);
    v.removeTriangle(iTri);
    triangles()[iTri].vIndices_[2] = ivNew;
  }

  vertices().push_back(vNew);
  mesh_->computeRingVertices(iv);
  mesh_->computeRingVertices(ivNew);

  std::vector<int> &ring1 = vertices()[ivNew].ringVertices_;
  int nbRing1 = ring1.size();
  for(int i = 0; i<nbRing1; ++i)
    mesh_->computeRingVertices(ring1[i]);

  std::vector<int> &ring2 = vertices()[iv].ringVertices_;
  int nbRing2 = ring2.size();
  for(int i = 0; i<nbRing2; ++i)
    mesh_->computeRingVertices(ring2[i]);

  iVertsDecimated_.push_back(iv);
  iVertsDecimated_.push_back(ivNew);

  std::vector<int> vSmooth;
  vSmooth.push_back(iv);
  vSmooth.push_back(ivNew);

  Brush brush;
  brush._strength = 1.0f;
  Sculpt::smooth(mesh_, vSmooth, brush, false);

  cleanUpSingularVertex(iv);
  cleanUpSingularVertex(ivNew);
}

/** Delete vertex if it is degenerate */
bool Topology::deleteVertexIfDegenerate(int iv)
{
  Vertex &v = vertices()[iv];
  if(v.tagFlag_<0)
    return true;
  Tools::tidy(v.tIndices_);
  int nbTris = v.tIndices_.size();
  if(nbTris==0)
  {
    v.tagFlag_ = -1;
    iVertsToDelete_.push_back(iv);
    iVertsDecimated_.push_back(iv);
    return true;
  }
  else if(nbTris==1)
  {
    int iTri = v.tIndices_[0];
    std::vector<int> verts;
    Triangle &t = triangles()[iTri];
    verts.push_back(t.vIndices_[0]);
    verts.push_back(t.vIndices_[1]);
    verts.push_back(t.vIndices_[2]);
    Tools::tidy(verts);
    int nbVerts = verts.size();
    for(int i = 0; i<nbVerts; ++i)
    {
      int iVert = verts[i];
      if(iVert!=iv)
      {
        vertices()[iVert].removeTriangle(iTri);
        mesh_->computeRingVertices(iVert);
      }
    }
    v.tagFlag_ = -1;
    t.tagFlag_ = -1;
    iTrisToDelete_.push_back(iTri);
    iVertsToDelete_.push_back(iv);
    iVertsDecimated_.push_back(iv);
    for(int i = 0; i<nbVerts; ++i)
    {
      int iVert = verts[i];
      if(iVert!=iv)
      {
        if(vertices()[iVert].tIndices_.size()<3)
          deleteVertexIfDegenerate(iVert);
      }
    }
    return true;
  }
  else if(nbTris==2)
  {
    int iTri1 = v.tIndices_[0];
    std::vector<int> verts1;
    Triangle &t1 = triangles()[iTri1];
    verts1.push_back(t1.vIndices_[0]);
    verts1.push_back(t1.vIndices_[1]);
    verts1.push_back(t1.vIndices_[2]);
    Tools::tidy(verts1);
    int nbVerts1 = verts1.size();
    for(int i = 0; i<nbVerts1; ++i)
    {
      int iVert = verts1[i];
      if(iVert!=iv)
      {
        vertices()[iVert].removeTriangle(iTri1);
        mesh_->computeRingVertices(iVert);
      }
    }

    int iTri2 = v.tIndices_[1];
    std::vector<int> verts2;
    Triangle &t2 = triangles()[iTri2];
    verts2.push_back(t2.vIndices_[0]);
    verts2.push_back(t2.vIndices_[1]);
    verts2.push_back(t2.vIndices_[2]);
    Tools::tidy(verts2);
    int nbVerts2 = verts2.size();
    for(int i = 0; i<nbVerts2; ++i)
    {
      int iVert = verts2[i];
      if(iVert!=iv)
      {
        vertices()[iVert].removeTriangle(iTri2);
        mesh_->computeRingVertices(iVert);
      }
    }

    v.tagFlag_ = -1;
    t1.tagFlag_ = -1;
    t2.tagFlag_ = -1;
    iTrisToDelete_.push_back(iTri1);
    iTrisToDelete_.push_back(iTri2);
    iVertsToDelete_.push_back(iv);
    iVertsDecimated_.push_back(iv);
    for(int i = 0; i<nbVerts1; ++i)
    {
      int iVert = verts1[i];
      if(iVert!=iv)
      {
        if(vertices()[iVert].tIndices_.size()<3)
          deleteVertexIfDegenerate(iVert);
      }
    }
    for(int i = 0; i<nbVerts2; ++i)
    {
      int iVert = verts2[i];
      if(iVert!=iv)
      {
        if(vertices()[iVert].tIndices_.size()<3)
          deleteVertexIfDegenerate(iVert);
      }
    }
    return true;
  }
  return false;
}
