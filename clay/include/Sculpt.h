#ifndef __SCULPT_H__
#define __SCULPT_H__

#include "DataTypes.h"
#include "Mesh.h"
#include "Topology.h"
#include <vector>

/**
 * Sculpt
 * @author Stéphane GINIER
 */
class Sculpt
{

public:
    enum SculptMode{INFLATE, DEFLATE, SMOOTH, FLATTEN, SWEEP};
    enum TopoMode{DECIMATION, SUBDIVISION, UNIFORMISATION, ADAPTIVE, STATIC};
    Sculpt();
    ~Sculpt();
    void setMesh(Mesh *mesh);
    void setIntensity(float intensity);
    void toggleCulling();
    void setSculptMode(SculptMode mode);
    void setTopoMode(TopoMode mode);
    void setDetail(float detail);
    const Vector3& getSweepCenter();
    void setSweepCenter(const Vector3 &center);
    void setSweepDir(const Vector3 &dir);
    bool isSweep();

    void sculptMesh(std::vector<int> &iVertsSelected, const Vector3 &intersectionPoint, const Vector3& velocity,
                    float radiusSquared, const Vector3 &eyeDirection, float strengthMult);
    void smooth(const std::vector<int> &iVerts, float intensity);
    void smoothFlat(const std::vector<int> &iVerts, float intensity);
    void draw(const std::vector<int> &iVerts, float radiusSquared, float intensity);
    void flatten(const std::vector<int> &iVerts, float radiusSquared, float intensity);
    void smoothNoMp(const std::vector<int> &iVerts, bool flat = false);
    void sweep(const std::vector<int> &iVerts, float radiusSquared, float intensity);

	int getNumBrushes() const { return (int)_brushes.size(); }
	void addBrush(const Vector3& pos, const Vector3& dir, const Vector3& vel, const float radius, const float strength, const float weight);
	void clearBrushes();
	void applyBrushes(const Matrix4x4& transform, float deltaTime, bool symmetry);
	std::vector<ci::Vec3f> brushPositions() const;
	std::vector<float> brushWeights() const;
	const BrushVector& getBrushes() const;
  
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    void setAdaptiveParameters(float radiusSquared);
    Vector3 areaNormal(const std::vector<int> &iVerts);
    Vector3 areaCenter(const std::vector<int> &iVerts);
    void laplacianSmooth(const std::vector<int> &iVerts, Vector3Vector &smoothVerts);
    void reflectBrush(Brush& brush, int axis);
    void transformBrush(const Matrix4x4& transform, float deltaTime, Brush& brush);

private:

    Mesh* mesh_; //selected mesh
    float intensity_; //deformation intensity
    SculptMode sculptMode_; //sculpting mode
    TopoMode topoMode_; //topological mode
    Vector3 centerPoint_; //center of deformation
    bool culling_; //if backface culling is enabled for sculpting
    float detail_; //intensity of details
    float thickness_; //thickness
    float d2Min_; //uniform refinement of mesh (min edge length)
    float d2Max_; //uniform refinement of mesh (max edge length)
    float d2Thickness_; //distance between 2 vertices before split/merge
    float d2Move_; //max displacement of vertices per step
    Vector3 sweepCenter_; //center of sweep tool
    Vector3 sweepDir_; //direction of deformation for sweep tool
    float deltaTime_;
    float minDetailMult_;
    Matrix4x4 prevTransform_;
    bool prevSculpt_;

		BrushVector _brushes;
};

#endif /*__SCULPT_H__*/
