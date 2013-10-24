#include "StdAfx.h"
#include "Topology.h"
#define _USE_MATH_DEFINES
#include <math.h>

/** Decimation */
void Topology::decimation(std::vector<int> &iTris, float detailMinSquared)
{
    float radius = sqrtf(radiusSquared_);

    for(int i = 0; i<(int)iTris.size(); ++i)
    {
        int iTri = iTris[i];
        Triangle &t = triangles_[iTri];
        if(t.tagFlag_<0)
            continue;
        int iv1 = t.vIndices_[0];
        int iv2 = t.vIndices_[1];
        int iv3 = t.vIndices_[2];

        Vertex &v1 = vertices_[iv1];
        Vertex &v2 = vertices_[iv2];
        Vertex &v3 = vertices_[iv3];

        float fallOff = ((v1+v2+v3)/3.f-centerPoint_).squaredNorm();
        if(fallOff<radiusSquared_) {
            fallOff = 1.f;
				} else if(fallOff<radiusSquared_*2.f) {
            fallOff = (sqrtf(fallOff)-radius)/(radius*static_cast<float>(M_SQRT2)-radius);
            fallOff = 3.f*fallOff*fallOff*fallOff*fallOff-4.f*fallOff*fallOff*fallOff+1.f;
        } else {
            continue;
				}

        float length1 = (v2-v1).squaredNorm();
        float length2 = (v2-v3).squaredNorm();
        float length3 = (v1-v3).squaredNorm();
        if(length1<length2 && length1<length3)
        {
            if(length1<detailMinSquared*fallOff)
                decimateTriangles(iTri,findOppositeTriangle(iTri,iv1,iv2),iTris);
        }
        else if(length2<length3)
        {
            if(length2<detailMinSquared*fallOff)
                decimateTriangles(iTri,findOppositeTriangle(iTri,iv2,iv3),iTris);
        }
        else
        {
            if(length3<detailMinSquared*fallOff)
                decimateTriangles(iTri,findOppositeTriangle(iTri,iv1,iv3),iTris);
        }
    }

    Tools::tidy(iTrisToDelete_);
    int nbTrisDelete = iTrisToDelete_.size();
    for(int i=nbTrisDelete-1;i>=0;--i) {
        deleteTriangle(iTrisToDelete_[i]);
    }

    Tools::tidy(iVertsToDelete_);
    int nbVertsToDelete = iVertsToDelete_.size();
    for(int i=nbVertsToDelete-1;i>=0;--i) {
        deleteVertex(iVertsToDelete_[i]);
    }

    std::vector<int> iVertsDecimated;
    int nbVertsDecimated = iVertsDecimated_.size();
    int nbVertices = vertices_.size();
    ++Vertex::tagMask_;
    for(int i=0;i<nbVertsDecimated;++i)
    {
        int iVert = iVertsDecimated_[i];
        if(iVert>=nbVertices) {
            continue;
				}
        Vertex &v = vertices_[iVert];
				if(v.tagFlag_==Vertex::tagMask_) {
            continue;
				}
        v.tagFlag_ = Vertex::tagMask_;
        iVertsDecimated.push_back(iVert);
    }

    std::vector<int> newTris = mesh_->getTrianglesFromVertices(iVertsDecimated);
    iTris.insert(iTris.end(),newTris.begin(),newTris.end());

    std::vector<int> iTrisTemp;
    int nbTris = iTris.size();
    int nbTriangles = triangles_.size();
    ++Triangle::tagMask_;
    for(int i = 0; i<nbTris; ++i)
    {
        int iTri = iTris[i];
        if(iTri>=nbTriangles) {
            continue;
				}
        Triangle &t = triangles_[iTri];
        if(t.tagFlag_==Triangle::tagMask_) {
            continue;
				}
        t.tagFlag_ = Triangle::tagMask_;
        iTrisTemp.push_back(iTri);
    }
    iTris = iTrisTemp;
}

/** Find opposite triangle */
int Topology::findOppositeTriangle(int iTri, int iv1, int iv2)
{
    Vertex &v1 = vertices_[iv1];
    Vertex &v2 = vertices_[iv2];
    std::vector<int> &iTris1 = v1.tIndices_;
    std::vector<int> &iTris2 = v2.tIndices_;
    std::sort(iTris1.begin(),iTris1.end());
    std::sort(iTris2.begin(),iTris2.end());
    std::vector<int> res(iTris1.size(),-1);
    std::set_intersection(iTris1.begin(),iTris1.end(),iTris2.begin(),iTris2.end(),res.begin());
    if(res.size() < 3 || res[2]!=-1) {
        return -1;
		}
		if(res[0]==iTri) {
        return res[1];
		} else {
        return res[0];
		}
}

/** Decimate triangles (find orientation of the 2 triangles) */
void Topology::decimateTriangles(int iTri1, int iTri2, std::vector<int> &iTris)
{
    if(iTri2==-1)
        return;
    Triangle &t1 = triangles_[iTri1];
    Triangle &t2 = triangles_[iTri2];
    int iv11 = t1.vIndices_[0];
    int iv21 = t1.vIndices_[1];
    int iv31 = t1.vIndices_[2];
    int iv12 = t2.vIndices_[0];
    int iv22 = t2.vIndices_[1];
    int iv32 = t2.vIndices_[2];
    if(iv11==iv12)
    {
        if(iv21==iv32) {
            edgeCollapse(iTri1,iTri2,iv11,iv21,iv31,iv22,iTris);
				} else {
            edgeCollapse(iTri1,iTri2,iv11,iv31,iv21,iv32,iTris);
				}
    }
    else if(iv11==iv22)
    {
        if(iv21==iv12) {
            edgeCollapse(iTri1,iTri2,iv11,iv21,iv31,iv32,iTris);
				} else {
            edgeCollapse(iTri1,iTri2,iv11,iv31,iv21,iv12,iTris);
				}
    }
    else if(iv11==iv32)
    {
        if(iv21==iv22) {
            edgeCollapse(iTri1,iTri2,iv11,iv21,iv31,iv12,iTris);
				} else {
            edgeCollapse(iTri1,iTri2,iv11,iv31,iv21,iv22,iTris);
				}
    }
    else if(iv21==iv12) {
        edgeCollapse(iTri1,iTri2,iv31,iv21,iv11,iv22,iTris);
		} else if(iv21==iv22) {
        edgeCollapse(iTri1,iTri2,iv31,iv21,iv11,iv32,iTris);
		} else {
        edgeCollapse(iTri1,iTri2,iv31,iv21,iv11,iv12,iTris);
		}
}

/** Decimate 2 triangles (collapse 1 edge) */
void Topology::edgeCollapse(int iTri1, int iTri2,int iv1, int iv2,int ivOpp1, int ivOpp2, std::vector<int> &iTris)
{
    iVertsDecimated_.push_back(iv1);
    iVertsDecimated_.push_back(iv2);

    Triangle &t1 = triangles_[iTri1];
    Triangle &t2 = triangles_[iTri2];
    Vertex &v1 = vertices_[iv1];
    Vertex &v2 = vertices_[iv2];
    Vector3& n1 = v1.normal_;
    Vertex &vOpp1 = vertices_[ivOpp1];
    Vertex &vOpp2 = vertices_[ivOpp2];
    std::vector<int> &tris1 = v1.tIndices_;
    std::vector<int> &tris2 = v2.tIndices_;
    std::vector<int> &ring1 = v1.ringVertices_;
    std::vector<int> &ring2 = v2.ringVertices_;

    //undo-redo
    mesh_->pushState(tris1,ring1);
    mesh_->pushState(tris2,ring2);

    std::sort(ring1.begin(),ring1.end());
    std::sort(ring2.begin(),ring2.end());
    std::vector<int> res(ring1.size(),-1);
    std::set_intersection(ring1.begin(),ring1.end(),ring2.begin(),ring2.end(),res.begin());

    assert(res.size() >= 2);
    if(res[2]!=-1) //edge flip
    {
        v1.removeTriangle(iTri2);
        v2.removeTriangle(iTri1);
        vOpp1.addTriangle(iTri2);
        vOpp2.addTriangle(iTri1);
        t1.replaceVertex(iv2,ivOpp2);
        t2.replaceVertex(iv1,ivOpp1);
        mesh_->computeRingVertices(iv1);
        mesh_->computeRingVertices(iv2);
        mesh_->computeRingVertices(ivOpp1);
        mesh_->computeRingVertices(ivOpp2);
        cleanUpSingularVertex(iv1);
        cleanUpSingularVertex(iv2);
        cleanUpSingularVertex(ivOpp1);
        cleanUpSingularVertex(ivOpp2);
        return;
    }

    n1 = (v1.normal_+v2.normal_).normalized();
    assert(fabs(n1.squaredNorm() - 1.0f) < 0.0001f); // crash n1 coords set to -1.#IND0000
    ring1.insert(ring1.end(),ring2.begin(),ring2.end());

    v1.removeTriangle(iTri1);
    v1.removeTriangle(iTri2);
    v2.removeTriangle(iTri1);
    v2.removeTriangle(iTri2);
    vOpp1.removeTriangle(iTri1);
    vOpp2.removeTriangle(iTri2);
    tris1.insert(tris1.end(),tris2.begin(),tris2.end());

    int nbTris2 = tris2.size();
    for(int i = 0; i<nbTris2; ++i) {
        triangles_[tris2[i]].replaceVertex(iv2,iv1);
    }

    mesh_->computeRingVertices(iv1);

    Vector3 laplacianPos(Vector3::Zero());
    int nbRing1 = ring1.size();
    for(int i = 0; i<nbRing1; ++i)
    {
        mesh_->computeRingVertices(ring1[i]);
        laplacianPos+=vertices_[ring1[i]];
    }
    assert(nbRing1 > 0);
    laplacianPos/=static_cast<float>(nbRing1);
    v1 = laplacianPos - n1*n1.dot(laplacianPos-v1);

    iTris.insert(iTris.end(),tris1.begin(),tris1.end());
    v2.tagFlag_ = -1;
    t1.tagFlag_ = -1;
    t2.tagFlag_ = -1;
    iVertsToDelete_.push_back(iv2);
    iTrisToDelete_.push_back(iTri1);
    iTrisToDelete_.push_back(iTri2);

    cleanUpSingularVertex(iv1);
}

/** Update last triangle of array and move its position */
void Topology::deleteTriangle(int iTri)
{
    Triangle &t = triangles_[iTri];
    int oldPos = t.posInLeaf_;
    std::vector<int> &iTrisLeaf = t.leaf_->getTriangles();
    int lastTri = iTrisLeaf.back();
    if(iTri!=lastTri)
    {
        iTrisLeaf[oldPos] = lastTri;
        triangles_[lastTri].posInLeaf_ = oldPos;
    }
    iTrisLeaf.pop_back();

    int lastPos = triangles_.size()-1;
    if(lastPos==iTri)
    {
        triangles_.pop_back();
        return;
    }
    Triangle &last = triangles_[lastPos];

    //undo-redo
    if(last.stateFlag_!=Mesh::stateMask_) { last.stateFlag_ = Mesh::stateMask_; mesh_->getTrianglesState().push_back(last); }

    last.id_ = iTri;
    std::vector<int> &iTrisLeafLast = last.leaf_->getTriangles();
    iTrisLeafLast[last.posInLeaf_] = iTri;
    int iv1 = last.vIndices_[0];
    int iv2 = last.vIndices_[1];
    int iv3 = last.vIndices_[2];
    Vertex &v1 = vertices_[iv1];
    Vertex &v2 = vertices_[iv2];
    Vertex &v3 = vertices_[iv3];

    //undo-redo
    if(v1.stateFlag_!=Mesh::stateMask_) { v1.stateFlag_ = Mesh::stateMask_; mesh_->getVerticesState().push_back(v1); }
    if(v2.stateFlag_!=Mesh::stateMask_) { v2.stateFlag_ = Mesh::stateMask_; mesh_->getVerticesState().push_back(v2); }
    if(v3.stateFlag_!=Mesh::stateMask_) { v3.stateFlag_ = Mesh::stateMask_; mesh_->getVerticesState().push_back(v3); }

    v1.replaceTriangle(lastPos,iTri);
    v2.replaceTriangle(lastPos,iTri);
    v3.replaceTriangle(lastPos,iTri);
    iVertsDecimated_.push_back(iv1);
    iVertsDecimated_.push_back(iv2);
    iVertsDecimated_.push_back(iv3);
    triangles_[iTri] = last;

    triangles_.pop_back();
}

/** Update last vertex of array and move its position */
void Topology::deleteVertex(int iVert)
{
    int lastPos = vertices_.size()-1;
    if(iVert==lastPos)
    {
        vertices_.pop_back();
        return;
    }
    Vertex &last = vertices_[lastPos];

    //undo-redo
    if(last.stateFlag_!=Mesh::stateMask_) { last.stateFlag_ = Mesh::stateMask_; mesh_->getVerticesState().push_back(last); }

    last.id_ = iVert;
    std::vector<int> &iTris = last.tIndices_;
    std::vector<int> &ring = last.ringVertices_;
    int nbTris = iTris.size();
    int nbRing = ring.size();
    for(int i=0;i<nbTris;++i)
    {
        Triangle &t = triangles_[iTris[i]];

        //undo-redo
        if(t.stateFlag_!=Mesh::stateMask_) { t.stateFlag_ = Mesh::stateMask_; mesh_->getTrianglesState().push_back(t); }

        t.replaceVertex(lastPos,iVert);
    }
    for(int i=0;i<nbRing;++i)
    {
        Vertex &v = vertices_[ring[i]];

        //undo-redo
        if(v.stateFlag_!=Mesh::stateMask_) { v.stateFlag_ = Mesh::stateMask_; mesh_->getVerticesState().push_back(v); }

        v.replaceRingVertex(lastPos,iVert);
    }
    vertices_[iVert] = last;

    vertices_.pop_back();
}
