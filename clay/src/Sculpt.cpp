#include "Sculpt.h"
#include <algorithm>
#include <cinder/gl/gl.h>

/** Constructor */
Sculpt::Sculpt() : mesh_(0), intensity_(0.5f), sculptMode_(INFLATE), topoMode_(ADAPTIVE), centerPoint_(Vector3::Zero()), culling_(false),
    detail_(1.0f), thickness_(0.5f), d2Min_(0.f), d2Max_(0.f), d2Thickness_(0.f), d2Move_(0.f), sweepCenter_(Vector3::Zero()), sweepDir_(Vector3::Zero()),
    prevTransform_(Matrix4x4::Identity()), deltaTime_(0.0f), minDetailMult_(0.05)
{}

/** Destructor */
Sculpt::~Sculpt()
{}

/** Getters/Setters */
void Sculpt::setMesh(Mesh *mesh) { mesh_ = mesh; }
void Sculpt::setIntensity(float intensity) { intensity_ = intensity; }
void Sculpt::toggleCulling() { culling_ = !culling_; }
void Sculpt::setSculptMode(SculptMode mode) { sculptMode_ = mode; }
void Sculpt::setTopoMode(TopoMode mode) { topoMode_ = mode; }
void Sculpt::setDetail(float detail) { detail_ = detail; }
const Vector3& Sculpt::getSweepCenter() { return sweepCenter_; }
void Sculpt::setSweepCenter(const Vector3& center) { sweepCenter_ = center; }
void Sculpt::setSweepDir(const Vector3& dir) { sweepDir_ = dir; }
bool Sculpt::isSweep() { return sculptMode_==SWEEP; }

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
void Sculpt::sculptMesh(std::vector<int> &iVertsSelected, const Vector3& intersectionPoint, const Vector3& velocity,
                        float radiusSquared, const Vector3& eyeDirection, float strengthMult)
{
    centerPoint_ = intersectionPoint;
    sweepCenter_ = intersectionPoint;
    sweepDir_ = velocity;
    VertexVector &vertices = mesh_->getVertices();

    std::vector<int> iTris = mesh_->getTrianglesFromVertices(iVertsSelected);

    //undo-redo
    mesh_->pushState(iTris,iVertsSelected);

    Topology topo(mesh_,radiusSquared,centerPoint_);
    setAdaptiveParameters(radiusSquared);
    switch(topoMode_)
    {
    case ADAPTIVE :
    case UNIFORMISATION : topo.uniformisation(iTris, d2Min_, d2Max_); break;
    case DECIMATION : topo.decimation(iTris, d2Min_); break;
    case SUBDIVISION : topo.subdivision(iTris, d2Max_); break;
    default : break;
    }

    iVertsSelected = mesh_->getVerticesFromTriangles(iTris);

    int nbVertsSelected = iVertsSelected.size();
    std::vector<int> iVertsSculpt;
    iVertsSculpt.reserve(nbVertsSelected);
    for(int i = 0; i<nbVertsSelected; ++i)
    {
        if(vertices[iVertsSelected[i]].sculptFlag_ == Vertex::sculptMask_)
            iVertsSculpt.push_back(iVertsSelected[i]);
    }

    std::vector<int> iVerts;
    int nbVertsSculpt = iVertsSculpt.size();
    if(culling_)
    {
        for (int i = 0; i<nbVertsSculpt; ++i)
        {
            if(eyeDirection.dot(vertices[iVertsSculpt[i]].normal_)<=0)
                iVerts.push_back(iVertsSculpt[i]);
        }
    }
    else
        iVerts = iVertsSculpt;

    switch(sculptMode_)
    {
    case INFLATE :
    case DEFLATE : draw(iVerts, radiusSquared, intensity_); break;
    case SMOOTH : smooth(iVerts, intensity_); break;
    case FLATTEN : flatten(iVerts,radiusSquared, intensity_); break;
    case SWEEP : sweep(iVerts, radiusSquared, intensity_); break;
    default: break;
    }

    if(topoMode_==ADAPTIVE)
    {
        topo.adaptTopology(iTris, d2Thickness_);
        iVertsSelected = mesh_->getVerticesFromTriangles(iTris);
    }

    mesh_->updateMesh(iTris,iVertsSelected);
}

/** Smooth a group of vertices. New position is given by simple averaging */
void Sculpt::smooth(const std::vector<int> &iVerts, float intensity)
{
    VertexVector &vertices = mesh_->getVertices();
    int nbVerts = iVerts.size();
    Vector3Vector smoothVerts(nbVerts, Vector3::Zero());
    laplacianSmooth(iVerts,smoothVerts);
    if(topoMode_!=ADAPTIVE)
    {
#pragma omp parallel for
        for (int i = 0; i<nbVerts; ++i)
        {
            Vertex &vert = vertices[iVerts[i]];
            Vector3 displ = (smoothVerts[i]-vert)*intensity;
            vert+=displ;
        }
    }
    else
    {
        float dMove = sqrtf(d2Move_);
#pragma omp parallel for
        for (int i = 0; i<nbVerts; ++i)
        {
            Vertex &vert = vertices[iVerts[i]];
            Vector3 displ = (smoothVerts[i]-vert)*intensity;
            float displLength = displ.squaredNorm();
            if(displLength<=d2Move_)
                vert+=displ;
            else
                vert+=displ.normalized()*dMove;
        }
    }
}

/** Smooth a group of vertices along the plane defined by the normal of the vertex */
void Sculpt::smoothFlat(const std::vector<int> &iVerts, float intensity)
{
    VertexVector &vertices = mesh_->getVertices();
    int nbVerts = iVerts.size();
    Vector3Vector smoothVerts(nbVerts, Vector3::Zero());
    laplacianSmooth(iVerts,smoothVerts);
#pragma omp parallel for
    for (int i = 0; i<nbVerts; ++i)
    {
        Vertex &vert = vertices[iVerts[i]];
        Vector3& vertSmo = smoothVerts[i];
        Vector3& n = vert.normal_;
        vertSmo-=n*n.dot(vertSmo-vert);
        vert+=(vertSmo-vert)*intensity;
    }
}

/**
 * Draw (inflate/deflate) a group of vertices. Move them according to the normal
 * representative of all the sculpting vertices. I couldn't come up with a good
 * falloff function, so it's just something that fall off strongly at the border of brush radius
*/
void Sculpt::draw(const std::vector<int> &iVerts, float radiusSquared, float intensity)
{
    VertexVector &vertices = mesh_->getVertices();
    int nbVerts = iVerts.size();
    float radius = sqrtf(radiusSquared);
    float deformationIntensity = intensity*radius*0.1f;
    if(topoMode_==ADAPTIVE)
        deformationIntensity = std::min(sqrtf(d2Move_), deformationIntensity);
    if(sculptMode_==DEFLATE)
        deformationIntensity = -deformationIntensity;
#pragma omp parallel for
    for (int i = 0; i<nbVerts; ++i)
    {
        Vertex &vert=vertices[iVerts[i]];
        float dist = ((vert-centerPoint_).norm()/radius);
        float fallOff = 3.f*dist*dist*dist*dist-4.f*dist*dist*dist+1.f;
        vert+=vert.normal_*deformationIntensity*fallOff;
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
    return Vector3(areaX,areaY,areaZ).normalized();
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
void Sculpt::flatten(const std::vector<int> &iVerts, float radiusSquared, float intensity)
{
    Vector3 areaNorm = areaNormal(iVerts).normalized();
    Vector3 areaPoint = areaCenter(iVerts);
    if(areaNorm.squaredNorm()<0.0001f)
        return;
    VertexVector &vertices = mesh_->getVertices();
    float radius = sqrtf(radiusSquared);
    int nbVerts = iVerts.size();
    float deformationIntensity = intensity * 0.3f;
    if(topoMode_!=ADAPTIVE)
    {
#pragma omp parallel for
        for (int i = 0; i<nbVerts; ++i)
        {
            Vertex &v = vertices[iVerts[i]];
						float distance = (v-areaPoint).dot(areaNorm);
            float dist = ((v-centerPoint_).norm()/radius);
            float fallOff = 3.f*dist*dist*dist*dist-4.f*dist*dist*dist+1.f;
            v-=areaNorm*distance*deformationIntensity*fallOff;
        }
    }
    else
    {
        float dMove = sqrtf(d2Move_);
#pragma omp parallel for
        for (int i = 0; i<nbVerts; ++i)
        {
            Vertex &v = vertices[iVerts[i]];
            float distance = (v-areaPoint).dot(areaNorm);
            float dist = ((v-centerPoint_).norm()/radius);
            float fallOff = 3.f*dist*dist*dist*dist-4.f*dist*dist*dist+1.f;
            v-=areaNorm*std::min(dMove,distance*deformationIntensity*fallOff);
        }
    }
}

/** Laplacian smooth. Special rule for vertex on the edge of the mesh. */
void Sculpt::laplacianSmooth(const std::vector<int> &iVerts, Vector3Vector &smoothVerts)
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
            int nbVertEdge = 0;
            for(int j = 0; j<nbVRing; ++j)
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
void Sculpt::sweep(const std::vector<int> &iVerts, float radiusSquared, float intensity)
{
    VertexVector &vertices = mesh_->getVertices();
    int nbVerts = iVerts.size();
    float radius = sqrtf(radiusSquared);
    float deformationIntensity = intensity*radius*0.025f*deltaTime_;

    if(topoMode_!=ADAPTIVE)
    {
#pragma omp parallel for
        for (int i = 0; i<nbVerts; ++i)
        {
            Vertex &vert = vertices[iVerts[i]];
            float dist = (vert-sweepCenter_).norm()/radius;
            float fallOff = 3.f*dist*dist*dist*dist-4.f*dist*dist*dist+1.f;
            vert+=sweepDir_*deformationIntensity*fallOff;
        }
    }
    else
    {
#pragma omp parallel for
        for (int i = 0; i<nbVerts; ++i)
        {
            Vertex &vert = vertices[iVerts[i]];
            float dist = (vert-sweepCenter_).norm()/radius;
            float fallOff = 3.f*dist*dist*dist*dist-4.f*dist*dist*dist+1.f;
            vert+=sweepDir_*std::min(d2Move_,deformationIntensity*fallOff);
        }
    }
}

void Sculpt::addBrush(const Vector3& pos, const Vector3& dir, const Vector3& vel, const float radius, const float strength, const float weight)
{
	_brushes.push_back(Brush());
	_brushes.back()._radius = radius;
	_brushes.back()._radius_squared = _brushes.back()._radius*_brushes.back()._radius;
	_brushes.back()._strength = weight*strength;
	_brushes.back()._weight = weight;
	_brushes.back()._position = pos;
	_brushes.back()._direction = dir;
  _brushes.back()._velocity = vel;
}

void Sculpt::clearBrushes()
{
    _brushes.clear();
}

void Sculpt::applyBrushes(const Matrix4x4& transform, float deltaTime)
{
  deltaTime_ = deltaTime;
  std::vector<int> vertices;
	// *** first calculate brush positions transformed into object space ***
	for(size_t b=0; b<_brushes.size(); ++b)
	{
    vertices.clear();
		Vector4 temp;
		temp << _brushes[b]._position, 0;
		_brushes[b]._transformed_position = (transform * temp).head<3>();
    Vector3 modelVel = (_brushes[b]._transformed_position - (prevTransform_ * temp).head<3>())/deltaTime;
		_brushes[b]._transformed_radius = _brushes[b]._radius;
		_brushes[b]._transformed_radius_squared = _brushes[b]._transformed_radius*_brushes[b]._transformed_radius;
    temp << _brushes[b]._velocity, 0;
    _brushes[b]._transformed_velocity = (transform * temp).head<3>() + 100.0f*modelVel;

		mesh_->getVerticesInsideSphere(_brushes[b]._transformed_position, _brushes[b]._transformed_radius_squared, vertices);
    if (!vertices.empty()) {
		  sculptMesh(vertices, _brushes[b]._transformed_position, _brushes[b]._transformed_velocity, _brushes[b]._transformed_radius_squared, Vector3::UnitZ(), _brushes[b]._strength);
    }
	}

  prevTransform_ = transform;
}

std::vector<ci::Vec3f> Sculpt::brushPositions() const
{
	std::vector<ci::Vec3f> positions;
	for (size_t i=0; i<_brushes.size(); i++)
	{
		const Vector3& pos = _brushes[i]._position;
		ci::Vec3f temp(pos.x(), pos.y(), pos.z());
		positions.push_back(temp);
	}
	return positions;
}

std::vector<float> Sculpt::brushWeights() const
{
	std::vector<float> weights;
	for (size_t i=0; i<_brushes.size(); i++)
	{
		weights.push_back(_brushes[i]._weight);
	}
	return weights;
}

const BrushVector& Sculpt::getBrushes() const {
	return _brushes;
}
