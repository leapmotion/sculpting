#include "DynamicMesh.h"

#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"
#include <set>


/*
void addQuadToDynamicMesh(DynamicMesh& _DynamicMesh,const Vec3f& _P1,const Vec3f& _P2,const Vec3f& _P3,const Vec3f& _P4)
{
	const Vec3f nrm = (Vec3f::cross(_P3-_P1,_P4-_P2)).unit();
	_DynamicMesh._vertices.push_back(DynamicMesh::DynamicMeshVertex(_P1,nrm));
	_DynamicMesh._vertices.push_back(DynamicMesh::DynamicMeshVertex(_P2,nrm));
	_DynamicMesh._vertices.push_back(DynamicMesh::DynamicMeshVertex(_P3,nrm));

	_DynamicMesh._vertices.push_back(DynamicMesh::DynamicMeshVertex(_P1,nrm));
	_DynamicMesh._vertices.push_back(DynamicMesh::DynamicMeshVertex(_P3,nrm));
	_DynamicMesh._vertices.push_back(DynamicMesh::DynamicMeshVertex(_P4,nrm));
}

void addTriangleToDynamicMesh(DynamicMesh& _DynamicMesh,const Vec3f& _P1,const Vec3f& _P2,const Vec3f& _P3)
{
	const Vec3f nrm = (Vec3f::cross(_P2-_P1,_P3-_P1)).unit();
	_DynamicMesh._vertices.push_back(DynamicMesh::DynamicMeshVertex(_P1,nrm));
	_DynamicMesh._vertices.push_back(DynamicMesh::DynamicMeshVertex(_P2,nrm));
	_DynamicMesh._vertices.push_back(DynamicMesh::DynamicMeshVertex(_P3,nrm));
}
*/


// *********************************************************************
// ************************ MARCHIING CUBES  ***************************
// *********************************************************************
int edgeTable[256] = {
	0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
	0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
	0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
	0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
	0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
	0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
	0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
	0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
	0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
	0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
	0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
	0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
	0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
	0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
	0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
	0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
	0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
	0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
	0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
	0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
	0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
	0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
	0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
	0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
	0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
	0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
	0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
	0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
	0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
	0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
	0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
	0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0   
};

int triTable[256][16] = {
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	{3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	{3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	{3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
	{9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	{2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	{8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	{4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
	{3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
	{1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
	{4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	{4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	{5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	{2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
	{9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	{0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	{2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
	{10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
	{5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	{5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	{9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	{1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
	{10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
	{8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
	{2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	{7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	{2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
	{11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
	{5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
	{11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
	{11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
	{9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
	{2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
	{6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	{3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
	{6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	{10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	{6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
	{8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
	{7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
	{3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	{0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
	{9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
	{8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	{5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
	{0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
	{6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
	{10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
	{10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	{8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	{1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
	{0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	{10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
	{3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	{6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
	{9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
	{8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
	{3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	{6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	{0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
	{10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
	{10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	{2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
	{7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	{7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	{2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
	{1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
	{11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
	{8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
	{0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
	{7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	{10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	{2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	{6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
	{7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	{2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
	{10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	{10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
	{0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
	{7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	{6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	{8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
	{9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
	{6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
	{4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
	{10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
	{8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	{0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
	{1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	{8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
	{10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	{4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
	{10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	{11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
	{9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	{6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
	{7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
	{3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
	{7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
	{3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
	{6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
	{9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
	{1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
	{4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
	{7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
	{6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	{3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
	{0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
	{6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
	{0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
	{11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
	{6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
	{5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
	{9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	{1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
	{1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
	{10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
	{0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
	{5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
	{10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
	{11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
	{9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
	{7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
	{2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	{8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
	{9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
	{9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
	{1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	{9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	{5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
	{0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
	{10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
	{2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
	{0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
	{0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
	{9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
	{5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	{3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
	{5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
	{8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	{0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
	{9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
	{1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
	{3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
	{4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
	{9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
	{11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	{11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
	{2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
	{9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
	{3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
	{1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
	{4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	{3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
	{0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
	{1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};

Vec3f VertexInterp(const float _IsoLevel, const Vec3f& _Pos1, const Vec3f& _Pos2, const float _Dist1, const float _Dist2)
{
	const float t = (_IsoLevel-_Dist1)/(_Dist2-_Dist1);
	return _Pos1*(1.f-t) + _Pos2*t;
}

float getField(const Vec3f& _Pos)
{
	return 0.f;
}

Vec3f getFieldNormal(const Vec3f& _Pos)
{
	return Vec3f(0,1,0);
}

void PolygoniseMarchingCubes(const Vec3f& _Size, DynamicMesh* _DynamicMesh, const float _IsoLevel)
{
	// *** calculate bounding box ***
	Vec3f bbmin, bbmax;
//	_Object.getBoundingBox(bbmin,bbmax);
bbmin = Vec3f(-2,-2,-2);
bbmax = Vec3f( 2, 2, 2);
	const Vec3f bbdiff = bbmax-bbmin;

	// *** set up distance grid ***
	const unsigned int num_x = int(_Size[0]);
	const unsigned int num_y = int(_Size[1]);
	const unsigned int num_z = int(_Size[2]);
	struct SamplePoint { Vec3f _pos; float _dist; };
	std::vector<SamplePoint> samples(num_x*num_y*num_z);
	for(unsigned int i=0,z=0; z<num_z; ++z)
	{
		float fz = float(z)/(num_z-1);
		for(unsigned int y=0; y<num_y; ++y)
		{
			float fy = float(y)/(num_y-1);
			for(unsigned int x=0; x<num_x; ++x,++i)
			{
				float fx = float(x)/(num_x-1);
				samples[i]._pos = bbmin + bbdiff*Vec3f(fx,fy,fz);
				samples[i]._dist = getField(samples[i]._pos);
			}
		}
	}

	// *** create triangles ***
	std::vector<Vec3f> vertlist(12);
	int idx[8];
	for(unsigned int z=0; z<num_z-1; ++z)
	{
		for(unsigned int y=0; y<num_y-1; ++y)
		{
			idx[0] = num_x*(y+z*num_y);
			idx[1] = idx[0]+1;
			idx[2] = idx[1]+num_x;
			idx[3] = idx[2]-1;
			idx[4] = idx[0]+num_x*num_y;
			idx[5] = idx[4]+1;
			idx[6] = idx[5]+num_x;
			idx[7] = idx[6]-1;

			for(unsigned int x=0; x<num_x-1; ++x)
			{
				// *** get index from cube corner samples ***
				unsigned char cubeindex = 0;
				unsigned char bit = 1;
				for(int j=0; j<8; ++j) 
				{
					if( samples[idx[j]]._dist<=_IsoLevel ) cubeindex |= bit;
					bit = bit<<1;
				}

				// *** create points along edges ***
				if( edgeTable[cubeindex]&0x001) vertlist[ 0] = VertexInterp(_IsoLevel, samples[idx[0]]._pos, samples[idx[1]]._pos, samples[idx[0]]._dist, samples[idx[1]]._dist);
				if( edgeTable[cubeindex]&0x002) vertlist[ 1] = VertexInterp(_IsoLevel, samples[idx[1]]._pos, samples[idx[2]]._pos, samples[idx[1]]._dist, samples[idx[2]]._dist);
				if( edgeTable[cubeindex]&0x004) vertlist[ 2] = VertexInterp(_IsoLevel, samples[idx[2]]._pos, samples[idx[3]]._pos, samples[idx[2]]._dist, samples[idx[3]]._dist);
				if( edgeTable[cubeindex]&0x008) vertlist[ 3] = VertexInterp(_IsoLevel, samples[idx[3]]._pos, samples[idx[0]]._pos, samples[idx[3]]._dist, samples[idx[0]]._dist);
				if( edgeTable[cubeindex]&0x010) vertlist[ 4] = VertexInterp(_IsoLevel, samples[idx[4]]._pos, samples[idx[5]]._pos, samples[idx[4]]._dist, samples[idx[5]]._dist);
				if( edgeTable[cubeindex]&0x020) vertlist[ 5] = VertexInterp(_IsoLevel, samples[idx[5]]._pos, samples[idx[6]]._pos, samples[idx[5]]._dist, samples[idx[6]]._dist);
				if( edgeTable[cubeindex]&0x040) vertlist[ 6] = VertexInterp(_IsoLevel, samples[idx[6]]._pos, samples[idx[7]]._pos, samples[idx[6]]._dist, samples[idx[7]]._dist);
				if( edgeTable[cubeindex]&0x080) vertlist[ 7] = VertexInterp(_IsoLevel, samples[idx[7]]._pos, samples[idx[4]]._pos, samples[idx[7]]._dist, samples[idx[4]]._dist);
				if( edgeTable[cubeindex]&0x100) vertlist[ 8] = VertexInterp(_IsoLevel, samples[idx[0]]._pos, samples[idx[4]]._pos, samples[idx[0]]._dist, samples[idx[4]]._dist);
				if( edgeTable[cubeindex]&0x200) vertlist[ 9] = VertexInterp(_IsoLevel, samples[idx[1]]._pos, samples[idx[5]]._pos, samples[idx[1]]._dist, samples[idx[5]]._dist);
				if( edgeTable[cubeindex]&0x400) vertlist[10] = VertexInterp(_IsoLevel, samples[idx[2]]._pos, samples[idx[6]]._pos, samples[idx[2]]._dist, samples[idx[6]]._dist);
				if( edgeTable[cubeindex]&0x800) vertlist[11] = VertexInterp(_IsoLevel, samples[idx[3]]._pos, samples[idx[7]]._pos, samples[idx[3]]._dist, samples[idx[7]]._dist);

				// *** create triangles ***
				Vec3f p;
				for(int j=0; triTable[cubeindex][j]!=-1; ) 
				{
/*
					addTriangleToDynamicMesh(_DynamicMesh, vertlist[triTable[cubeindex][j++]], vertlist[triTable[cubeindex][j++]], vertlist[triTable[cubeindex][j++]]);
/*/
					p = vertlist[triTable[cubeindex][j++]]; _DynamicMesh->addVertex(p,getFieldNormal(p));
					p = vertlist[triTable[cubeindex][j++]]; _DynamicMesh->addVertex(p,getFieldNormal(p));
					p = vertlist[triTable[cubeindex][j++]]; _DynamicMesh->addVertex(p,getFieldNormal(p));
//*/
				}
				for(int j=0; j<8; ++j) idx[j]++; // move all indices along by 1
			}
		}
	}
}

#if 0
// *********************************************************************
// ************************ DUAL CONTOURING  ***************************
// *********************************************************************
void PolygoniseDualContouring(const Vec3f& _Size, DynamicMesh& _DynamicMesh, const float _IsoLevel)
{
	// *** calculate bounding box ***
	Vec3f bbmin, bbmax;
	_Object.getBoundingBox(bbmin,bbmax);
	const Vec3f bbdiff = bbmax-bbmin;

	// *** set up distance grid ***
	const unsigned int num_x = int(_Size[0]);
	const unsigned int num_y = int(_Size[1]);
	const unsigned int num_z = int(_Size[2]);
	struct SamplePoint 
	{ 
		Vec3f _pos; // actual grid pos
		bool _hit[3]; // do edge axes cross surface
		Vec3f _p[3]; // positions of where edges of each axis cross surface
		Vec3f _n[3]; // normals of where edges of each axis cross surface
		bool _has_centroid; // has point on surface
		Vec3f _centroid; // position of centroid
		float _dist; // field value at grid pos
	}; 
	std::vector<SamplePoint> samples(num_x*num_y*num_z);
	for(unsigned int i=0,z=0; z<num_z; ++z)
	{
		float fz = float(z)/(num_z-1);
		for(unsigned int y=0; y<num_y; ++y)
		{
			float fy = float(y)/(num_y-1);
			for(unsigned int x=0; x<num_x; ++x,++i)
			{
				float fx = float(x)/(num_x-1);
				samples[i]._pos = bbmin + bbdiff*Vec3f(fx,fy,fz);
				samples[i]._dist = _Object.getField(samples[i]._pos);
				samples[i]._hit[0] = samples[i]._hit[1] = samples[i]._hit[2] = false;
				samples[i]._has_centroid = false;
				samples[i]._centroid = Vec3f(0,0,0);
			}
		}
	}

	// *** calculate positions and normals of where cubes intersect surface ***
	for(unsigned int i=0,z=0; z<num_z; ++z)
	{
		for(unsigned int y=0; y<num_y; ++y)
		{
			for(unsigned int x=0; x<num_x; ++x,++i)
			{
				const int ix = i+1;
				if( x<num_x-1 && samples[i]._dist*samples[ix]._dist<0.f )
				{
					samples[i]._hit[0] = true;
					samples[i]._p[0] = VertexInterp(_IsoLevel,samples[i]._pos,samples[ix]._pos,samples[i]._dist,samples[ix]._dist);
					samples[i]._n[0] = _Object.getNormal(samples[i]._p[0]);
				}
				const int iy = i+num_x;
				if( y<num_y-1 && samples[i]._dist*samples[iy]._dist<0.f )
				{
					samples[i]._hit[1] = true;
					samples[i]._p[1] = VertexInterp(_IsoLevel,samples[i]._pos,samples[iy]._pos,samples[i]._dist,samples[iy]._dist);
					samples[i]._n[1] = _Object.getNormal(samples[i]._p[1]);
				}
				const int iz = i+num_x*num_y;
				if( z<num_z-1 && samples[i]._dist*samples[iz]._dist<0.f )
				{
					samples[i]._hit[2] = true;
					samples[i]._p[2] = VertexInterp(_IsoLevel,samples[i]._pos,samples[iz]._pos,samples[i]._dist,samples[iz]._dist);
					samples[i]._n[2] = _Object.getNormal(samples[i]._p[2]);
				}
			}
		}
	}

	// *** calculate centroid positions for each cube ***
	int idx[8];
	for(unsigned int z=0; z<num_z-1; ++z)
	{
		for(unsigned int y=0; y<num_y-1; ++y)
		{
			idx[0] = num_x*(y+z*num_y);
			idx[1] = idx[0]+1;
			idx[2] = idx[1]+num_x;
			idx[3] = idx[2]-1;
			idx[4] = idx[0]+num_x*num_y;
			idx[5] = idx[4]+1;
			idx[6] = idx[5]+num_x;
			idx[7] = idx[6]-1;
			for(unsigned int x=0; x<num_x-1; ++x)
			{
				std::vector<Vec3f> hit_positions;
				std::vector<Vec3f> hit_normals;

				if( samples[idx[0]]._hit[0] ) { hit_positions.push_back(samples[idx[0]]._p[0]); hit_normals.push_back(samples[idx[0]]._n[0]); }
				if( samples[idx[1]]._hit[1] ) { hit_positions.push_back(samples[idx[1]]._p[1]); hit_normals.push_back(samples[idx[1]]._n[1]); }
				if( samples[idx[3]]._hit[0] ) { hit_positions.push_back(samples[idx[3]]._p[0]); hit_normals.push_back(samples[idx[3]]._n[0]); }
				if( samples[idx[0]]._hit[1] ) { hit_positions.push_back(samples[idx[0]]._p[1]); hit_normals.push_back(samples[idx[0]]._n[1]); }
				if( samples[idx[4]]._hit[0] ) { hit_positions.push_back(samples[idx[4]]._p[0]); hit_normals.push_back(samples[idx[4]]._n[0]); }
				if( samples[idx[5]]._hit[1] ) { hit_positions.push_back(samples[idx[5]]._p[1]); hit_normals.push_back(samples[idx[5]]._n[1]); }
				if( samples[idx[7]]._hit[0] ) { hit_positions.push_back(samples[idx[7]]._p[0]); hit_normals.push_back(samples[idx[7]]._n[0]); }
				if( samples[idx[4]]._hit[1] ) { hit_positions.push_back(samples[idx[4]]._p[1]); hit_normals.push_back(samples[idx[4]]._n[1]); }
				if( samples[idx[0]]._hit[2] ) { hit_positions.push_back(samples[idx[0]]._p[2]); hit_normals.push_back(samples[idx[0]]._n[2]); }
				if( samples[idx[1]]._hit[2] ) { hit_positions.push_back(samples[idx[1]]._p[2]); hit_normals.push_back(samples[idx[1]]._n[2]); }
				if( samples[idx[2]]._hit[2] ) { hit_positions.push_back(samples[idx[2]]._p[2]); hit_normals.push_back(samples[idx[2]]._n[2]); }
				if( samples[idx[3]]._hit[2] ) { hit_positions.push_back(samples[idx[3]]._p[2]); hit_normals.push_back(samples[idx[3]]._n[2]); }

				if( !hit_positions.empty() )
				{
					samples[idx[0]]._has_centroid = true;

//*
					samples[idx[0]]._centroid = (samples[idx[0]]._pos+samples[idx[6]]._pos)*0.5f; // cell centroid starting positon
/*/
					for(size_t j=0; j<hit_positions.size(); ++j)
					{
						samples[idx[0]]._centroid += hit_positions[j];
					}
					samples[idx[0]]._centroid /= (float)hit_positions.size();
//*/

//*
					// *** find centroid point through iteration ***
					const int num_iterations = 100;
					for(int iter=0; iter<num_iterations; ++iter)
					{
						const float f = 1.f-float(iter)/num_iterations;
						for(size_t j=0; j<hit_positions.size(); ++j)
						{
							samples[idx[0]]._centroid += hit_normals[j]*Vec3f::dot(hit_positions[j]-samples[idx[0]]._centroid,hit_normals[j])*(0.5f*f);
						}
						for(int d=0; d<3; ++d)
						{
							if( samples[idx[0]]._centroid[d]<samples[idx[0]]._pos[d] ) samples[idx[0]]._centroid[d] = samples[idx[0]]._pos[d];
							if( samples[idx[0]]._centroid[d]>samples[idx[6]]._pos[d] ) samples[idx[0]]._centroid[d] = samples[idx[6]]._pos[d];
						}
					}
//*/
/*
					// *** find centroid point through QEF solution ***
					float matrix_A[12][3];
					float vector_B[12];

					for(size_t j=0; j<hit_positions.size(); ++j)
					{
						// px,py,pz  is the intersection point
						// nx,ny,nz  is the normal vector at that point
						matrix_A[j][0] = hit_normals[j][0];
						matrix_A[j][1] = hit_normals[j][1];
						matrix_A[j][2] = hit_normals[j][2];

						// compute dot product
						vector_B[j] = Vec3f::dot(hit_positions[j],hit_normals[j]);
					}

					// solve Ax=B using singular value decomposition
					QEF::evaluate(matrix_A, vector_B, (int)hit_positions.size(), &samples[idx[0]]._centroid);
//*/
/*
					// *** find centroid point through QEF solution (eigen) ***
					MatrixXf matrix_A((int)hit_positions.size(),3);
					VectorXf vector_B((int)hit_positions.size());

 					for(size_t j=0; j<hit_normals.size(); ++j)
					{
						// px,py,pz  is the intersection point
						// nx,ny,nz  is the normal vector at that point
						matrix_A(j,0) = hit_normals[j][0];
						matrix_A(j,1) = hit_normals[j][1];
						matrix_A(j,2) = hit_normals[j][2];

						// compute dot product
						vector_B(j) = Vec3f::dot(hit_positions[j],hit_normals[j]);
					}
					VectorXf x = matrix_A.jacobiSvd(ComputeThinU | ComputeThinV).solve(vector_B);
					samples[idx[0]]._centroid = Vec3f(x[0],x[1],x[2]);
//*/
				}
				for(int j=0; j<8; ++j) idx[j]++; // move all indices along by 1
			}
		}
	}

	// *** calculate centroid positions for each cube ***
	for(unsigned int z=0; z<num_z-1; ++z)
	{
		for(unsigned int y=0; y<num_y-1; ++y)
		{
			idx[0] = num_x*(y+z*num_y);
			idx[1] = idx[0]+1;
			idx[2] = idx[1]+num_x;
			idx[3] = idx[2]-1;
			idx[4] = idx[0]+num_x*num_y;
			idx[5] = idx[4]+1;
			idx[6] = idx[5]+num_x;
			idx[7] = idx[6]-1;
			for(unsigned int x=0; x<num_x-1; ++x)
			{
				if( samples[idx[0]]._has_centroid && samples[idx[1]]._has_centroid && samples[idx[2]]._has_centroid && samples[idx[3]]._has_centroid ) 
				{
					addQuadToDynamicMesh(_DynamicMesh, samples[idx[0]]._centroid, samples[idx[1]]._centroid, samples[idx[2]]._centroid, samples[idx[3]]._centroid);
				}

				if( samples[idx[4]]._has_centroid && samples[idx[0]]._has_centroid && samples[idx[3]]._has_centroid && samples[idx[7]]._has_centroid ) 
				{
					addQuadToDynamicMesh(_DynamicMesh, samples[idx[4]]._centroid, samples[idx[0]]._centroid, samples[idx[3]]._centroid, samples[idx[7]]._centroid);
				}

				if( samples[idx[4]]._has_centroid && samples[idx[5]]._has_centroid && samples[idx[1]]._has_centroid && samples[idx[0]]._has_centroid ) 
				{
					addQuadToDynamicMesh(_DynamicMesh, samples[idx[4]]._centroid, samples[idx[5]]._centroid, samples[idx[1]]._centroid, samples[idx[0]]._centroid);
				}

				for(int j=0; j<8; ++j) idx[j]++; // move all indices along by 1
			}
		}
	}

	// *** flip normals to comply with gradient ***
	for(size_t i=0; i<_DynamicMesh._vertices.size(); i+=3)
	{
		Vec3f& vert1 = _DynamicMesh._vertices[i+0]._pos;
		Vec3f& vert2 = _DynamicMesh._vertices[i+1]._pos;
		Vec3f& vert3 = _DynamicMesh._vertices[i+2]._pos;
		const Vec3f mid = (vert1+vert2+vert3)/3;
		const Vec3f nrm = Vec3f::cross(vert2-vert1,vert3-vert1);
		if( Vec3f::dot(nrm,_Object.getNormal(mid))<0.f )
		{
			const Vec3f temp = vert2;
			vert2 = vert3;
			vert3 = temp;
			_DynamicMesh._vertices[i+0]._nrm *= -1.f;
			_DynamicMesh._vertices[i+1]._nrm *= -1.f;
			_DynamicMesh._vertices[i+2]._nrm *= -1.f;
		}
	}

}

#endif


//*************************************************************************************************

struct CSGPolygon;
typedef std::vector<CSGPolygon> CSGPolygonList;


struct Plane
{
	enum TYPE { COPLANAR=0, FRONT, BACK, SPANNING, };
	Vec3f _nrm;
	float _dist;
	static float PLANE_EPSILON;

	Plane() { _nrm = Vec3f(0,0,0); _dist = 0.f; }
	Plane(const Vec3f& _Nrm,const float _Dist) { _nrm = _Nrm.normalized(); _dist = _Dist; }	
	Plane(const Vec3f& _Pos,const Vec3f& _Nrm) { _nrm = _Nrm.normalized(); _dist = _nrm.dot(_Pos); }	
	void fromPoints(const Vec3f& _P0,const Vec3f& _P1,const Vec3f& _P2) { _nrm = ((_P2-_P0).cross(_P1-_P0)).normalized(); _dist = _nrm.dot(_P0); }
	void flip() { _nrm = -_nrm; _dist = -_dist; }
	void split(const CSGPolygon& _Polygon,CSGPolygonList& _CoplanarFront,CSGPolygonList& _CoplanarBack,CSGPolygonList& _Front,CSGPolygonList& _Back);
};

float Plane::PLANE_EPSILON = 1e-8;


// *** forward declarations ***
struct CSGVertex;
struct Plane;
struct CSGPolygon;
struct CSGNode;
struct CGSObject;
struct Mesh;

typedef std::vector<CSGVertex> CSGVertexList;


//****************************************************
struct CSGVertex
{
	Vec3f _pos, _nrm;
	CSGVertex() { }
	CSGVertex(const Vec3f& _Pos,const Vec3f& _Nrm) : _pos(_Pos), _nrm(_Nrm) { }
	CSGVertex interpolate(const CSGVertex& _Other,const float _T) const { return CSGVertex( _pos*(1.f-_T) + _Other._pos*_T, _nrm*(1.f-_T) + _Other._nrm*_T); }
	void flip() { _nrm = -_nrm; }
};


//****************************************************
struct CSGPolygon
{
	CSGVertexList _vertices;
	Plane _plane;
//	shared???
	CSGPolygon() { }
	CSGPolygon(const CSGPolygon& _Polygon) { _plane = _Polygon._plane; _vertices = _Polygon._vertices; } // copy constructor
	CSGPolygon(CSGVertexList& _Vertices);
	void calculateNormal();
	void flip();
};


//****************************************************
struct CSGNode
{
	Plane _plane;
	CSGNode* _front;
	CSGNode* _back;
	CSGPolygonList _polygons;

	CSGNode() : _front(0), _back(0) { }
	CSGNode(const CSGNode& _CSGNode) { _plane = _CSGNode._plane; _front = _CSGNode._front; _back = _CSGNode._back; _polygons = _CSGNode._polygons; } // copy constructor
	CSGNode(CSGPolygonList& _Polygons) : _front(0), _back(0) { if( !_Polygons.empty() ) build(_Polygons); }
	~CSGNode();
	void invert();
	CSGPolygonList clipPolygons(CSGPolygonList& _PolygonList);
	void clipTo(CSGNode* _Node);
	CSGPolygonList allPolygons();
	void mergePolygons();
	void build(CSGPolygonList& _Polygons);
	void draw();
};

//*********************************************************************
struct Polytope
{
	std::vector<Plane> _planes;
	void addPlane(const Vec3f& _Pos,const Vec3f& _Dir);
	bool pointInside(const Vec3f& _Pos,const float _Epsilon=0.001f);
	void createCube();
	void createCone(const int _Slices);
	void createCylinder(const int _Slices);
	void createSphere(const int _NumPoints);
	CSGPolygonList toPolygons();
};


void  Polytope::addPlane(const Vec3f& _Pos,const Vec3f& _Dir) 
{ 
	_planes.push_back(Plane(_Pos,_Dir)); 
}

bool Polytope::pointInside(const Vec3f& _Pos,const float _Epsilon)
{
	for(size_t i=0; i<_planes.size(); ++i)
	{
		if( _Pos.dot(_planes[i]._nrm)-_planes[i]._dist-_Epsilon > 0.f )
		{
			return false;
		}
	}
	return true;
}

void Polytope::createCube()
{
	_planes.clear();
	addPlane(Vec3f(-1,0,0),Vec3f(-1,0,0));
	addPlane(Vec3f( 1,0,0),Vec3f( 1,0,0));
	addPlane(Vec3f(0,-1,0),Vec3f(0,-1,0));
	addPlane(Vec3f(0, 1,0),Vec3f(0, 1,0));
	addPlane(Vec3f(0,0,-1),Vec3f(0,0,-1));
	addPlane(Vec3f(0,0, 1),Vec3f(0,0, 1));
//	addPlane(Vec3f(-1,-1,-1),Vec3f(-1,-1,-1));
}

void Polytope::createCone(const int _Slices)
{
	_planes.clear();
	for(int i=0; i<_Slices; ++i)
	{
		const float f = float(M_PI)*2.f*float(i)/_Slices;
		const Vec3f v = Vec3f(cosf(f),1.f,sinf(f));
		addPlane(Vec3f(0,0,0),v);
	}
	addPlane(Vec3f(0,-1,0),Vec3f(0,-1,0));
}

void Polytope::createCylinder(const int _Slices)
{
	_planes.clear();
	for(int i=0; i<_Slices; ++i)
	{
		const float f = float(M_PI)*2.f*float(i)/_Slices;
		const Vec3f v = Vec3f(cosf(f),0.f,sinf(f));
//		addPlane(v+Vec3f(0,RRAND,0),v+Vec3f(0,RRAND*0.1f,0));
		addPlane(v,v);
	}
	addPlane(Vec3f(0,-1,0),Vec3f(0,-1,0));
	addPlane(Vec3f(0, 1,0),Vec3f(0, 1,0));
}

float radicalInverse_VdC(unsigned int bits) 
{
	bits = (bits << 16u) | (bits >> 16u);
	bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
	bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
	bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
	bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
	return float(bits) * 2.3283064365386963e-10; // / 0x100000000
}

Vec3f hammersley2d(unsigned int i, unsigned int N) 
{
	return Vec3f(float(i)/float(N), radicalInverse_VdC(i),0);
}

void Polytope::createSphere(const int _NumPoints)
{
	_planes.clear();

	std::vector<Vec3f> points(_NumPoints);

	for(int i=0; i<_NumPoints; ++i)
	{
		const float f = float(i)/(_NumPoints);
		const float theta = float(M_PI)*f;
		const float phi = float(M_PI)*2.f*radicalInverse_VdC(i);
		points[i][0] = sin(theta)*cos(phi);
		points[i][1] = cos(theta);
		points[i][2] = sin(theta)*sin(phi);
		points[i].normalize();
	}

	const int num_iter = 100;
	for(int iter=0; iter<num_iter; ++iter)
	{
		const float f = float(num_iter-iter)/num_iter;
		for(size_t i=0; i<points.size(); ++i)
		{
			Vec3f offset(0,0,0);
			for(size_t j=0; j<points.size(); ++j)
			{
				if( i!=j )
				{
					Vec3f v = points[i]-points[j];
					float d = v.length();
					v /= d;
					float factor = 0.2f/(1.f+d*10);
					offset += v*(factor*f);
				}
			}
			points[i] += offset;
			points[i].normalize();
		}
	}

	for(size_t i=0; i<points.size(); ++i) 
	{
		addPlane(points[i],points[i]);
	}
}

CSGPolygonList Polytope::toPolygons()
{
	struct classcomp { bool operator() (const int& lhs, const int& rhs) const {return lhs<rhs;} };

	// *** array to temporarily store vertices ***
	std::vector<Vec3f> vertices;

	// *** array to store which vertices are in each polygon (random order) ***
	std::vector< std::set<int,classcomp> > polyinfo(_planes.size());

	// *** find all vertices of convex volume (intersections of 3 planes which lie on the surface) ***
	for(size_t i=0; i<_planes.size()-2; ++i)
	{
		for(size_t j=i+1; j<_planes.size()-1; ++j)
		{
			if( fabs( _planes[i]._nrm.dot(_planes[j]._nrm) )>1.f-Plane::PLANE_EPSILON ) continue;
			for(size_t k=j+1; k<_planes.size(); ++k)
			{
				// *** check for parallel planes ***
				if( fabs( _planes[j]._nrm.dot(_planes[k]._nrm) )>1.f-Plane::PLANE_EPSILON ) continue;
				if( fabs( _planes[k]._nrm.dot(_planes[i]._nrm) )>1.f-Plane::PLANE_EPSILON ) continue;

				const float denom = _planes[i]._nrm.dot( _planes[j]._nrm.cross(_planes[k]._nrm) );

				if( fabs(denom)<0.00001f ) continue; // no divide by zero

				// *** find intersection point of these 3 planes ***
				const Vec3f numer = 
					_planes[j]._nrm.cross(_planes[k]._nrm) * _planes[i]._dist + 
					_planes[k]._nrm.cross(_planes[i]._nrm) * _planes[j]._dist + 
					_planes[i]._nrm.cross(_planes[j]._nrm) * _planes[k]._dist;

				const Vec3f p = numer/denom;


				// *** test to see if point is inside convex object ***
				bool is_inside = true;
				for(size_t pl=0; pl<_planes.size(); ++pl)
				{
					if( pl!=i && pl!=j && pl!=k ) // dont test connected planes (obviously on surface of those)
					{
						if( p.dot(_planes[pl]._nrm)-_planes[i]._dist > Plane::PLANE_EPSILON )
						{
							is_inside = false; // in front of plane so ignore point
							break;
						}
					}
				}
				if( is_inside )
				{
					bool can_add = true;
					for(size_t vi=0; vi<vertices.size(); ++vi)
					{
						if( (p-vertices[vi]).length() < Plane::PLANE_EPSILON )
						{
							can_add = false;
							polyinfo[i].insert((int)vi);
							polyinfo[j].insert((int)vi);
							polyinfo[k].insert((int)vi);
							break;
						}
					}
					if( can_add )
					{
						polyinfo[i].insert((int)vertices.size());
						polyinfo[j].insert((int)vertices.size());
						polyinfo[k].insert((int)vertices.size());
						vertices.push_back(p);
					}
				}
			}
		}
	}

	CSGPolygonList polygons;
	for(size_t pi=0; pi<_planes.size(); ++pi)
	{
		if( (int)polyinfo[pi].size()<3 ) continue; // didnt find enough vertices for a polygon for this plane

		// *** first find all points on plane ***
		std::vector<Vec3f> polygon_points;

		for(std::set<int,classcomp>::iterator it=polyinfo[pi].begin(); it!=polyinfo[pi].end(); ++it)
		{
			polygon_points.push_back(vertices[*it]);
		}

		// *** find convex hull polygon (slow, but works) ***
		polygons.push_back(CSGPolygon());
		while( !polygon_points.empty() )
		{
			if( polygons.back()._vertices.empty() )
			{
				// to start with, just add first vertex
				polygons.back()._vertices.push_back(CSGVertex(polygon_points[0],_planes[pi]._nrm));
				polygon_points.erase(polygon_points.begin()+0);
			}
			else
			{
				// otherwise, check all remaining for most extreme right, and add it
				int best_point = -1;
				Plane best_plane;
				for(size_t vi=0; vi<polygon_points.size(); ++vi)
				{
					if( best_point==-1 || best_plane._nrm.dot(polygon_points[vi])-best_plane._dist<0.0f )
					{
						best_point = vi;
						const Vec3f latest_point_to_best = polygon_points[best_point]-polygons.back()._vertices.back()._pos;
						best_plane = Plane(polygons.back()._vertices.back()._pos, latest_point_to_best.cross(_planes[pi]._nrm));
					}
				}
				polygons.back()._vertices.push_back(CSGVertex(polygon_points[best_point],_planes[pi]._nrm));
				polygon_points.erase(polygon_points.begin()+best_point);
			}
		}
		polygons.back().calculateNormal();
	}
	return polygons;
}


//****************************************************
struct CSGObject
{
	CSGNode* _root;
	CSGObject() : _root(0) { }
	~CSGObject() { delete _root; }
	void draw();
	void invert() { if( _root ) _root->invert(); }
	void clipTo(CSGObject* _Object) { if( _root ) _root->clipTo(_Object->_root); }
	void mergePolygons() { if( _root ) _root->mergePolygons(); }
	void build(CSGPolygonList& _Polygons) { if( _root ) _root->build(_Polygons); }
	CSGPolygonList allPolygons() { if( _root ) return _root->allPolygons(); return CSGPolygonList(); } // potentially inefficient (extra copy)
	void createUnion(CSGObject* _Other);
	void createSubtraction(CSGObject* _Other);
	void createIntersection(CSGObject* _Other);
};


//****************************************************
void Plane::split(const CSGPolygon& _Polygon,CSGPolygonList& _CoplanarFront,CSGPolygonList& _CoplanarBack,CSGPolygonList& _Front,CSGPolygonList& _Back)
{
    int polygonType = COPLANAR;
    std::vector<int> types;
    // *** check each vertex of polygon to see if in front, behind, or on plane ***
	for(unsigned int i=0; i<_Polygon._vertices.size(); ++i) 
	{
      	const float t = _nrm.dot(_Polygon._vertices[i]._pos) - _dist;
      	const int type = (t<-PLANE_EPSILON) ? BACK : (t>PLANE_EPSILON) ? FRONT : COPLANAR;
      	polygonType |= type;
      	types.push_back(type);
    }
      	
	switch( polygonType ) 
	{
		case COPLANAR:
		{
			if( _nrm.dot(_Polygon._plane._nrm)>0.f )
				_CoplanarFront.push_back(_Polygon);
			else
				_CoplanarBack.push_back(_Polygon);
			break;
		}    	
		case FRONT:
		{
			_Front.push_back(_Polygon);
			break;
		}
      	case BACK:
      	{
        	_Back.push_back(_Polygon);
        	break;
		}
		case SPANNING:
		{
			CSGVertexList f, b;
        	for(unsigned int i=0; i<_Polygon._vertices.size(); ++i) 
		    {
				const int j = (i+1)%_Polygon._vertices.size();
				const CSGVertex& vi = _Polygon._vertices[i];
				const CSGVertex& vj = _Polygon._vertices[j];
				if( types[i]!=BACK ) f.push_back(vi);
				if( types[i]!=FRONT ) b.push_back(vi);
				if( (types[i] | types[j])==SPANNING ) // plane intersects this edge
				{
					// *** find intersection and add to both polygons ***
					const float t = (_dist - _nrm.dot(vi._pos)) / _nrm.dot(vj._pos-vi._pos);
					CSGVertex v = vi.interpolate(vj, t);

					// *** add intersection vertex to front AND back polygons ***
					f.push_back(v); b.push_back(v);
				}
			}
			if( f.size()>=3 ) { _Front.push_back(CSGPolygon(f) ); _Front.back().calculateNormal(); }
			if( b.size()>=3 ) { _Back.push_back(CSGPolygon(b) ); _Back.back().calculateNormal(); }
			break;
		}
    }
}

//****************************************************
CSGObject* createCube(const Vec3f& _Scale,const Vec3f& _Offset);
CSGObject* createCylinder(const Vec3f& _Scale,const Vec3f& _Offset,const int _Slices);
CSGObject* createSphere(const Vec3f& _Scale,const Vec3f& _Offset,const int _Points);


CSGPolygon::CSGPolygon(CSGVertexList& _Vertices) 
{ 
	_vertices = _Vertices; 
	_plane.fromPoints(_vertices[0]._pos,_vertices[1]._pos,_vertices[2]._pos); 
}

void CSGPolygon::calculateNormal()
{
	if( _vertices.size()>=3 )
	{
		// *** find best pair of vectors for finding normal ***
		Vec3f vector1 = (_vertices[1]._pos-_vertices[0]._pos).normalized(), vector2;
		float smallest_fdot = 9999.f;
		for(size_t i=2; i<_vertices.size(); ++i)
		{
			const Vec3f temp_vector = (_vertices[i]._pos-_vertices[0]._pos).normalized();
			const float temp_dot = vector1.dot(temp_vector);
			if( temp_dot<smallest_fdot )
			{
				smallest_fdot = temp_dot;
				vector2 = temp_vector;
			}
		}


//		_plane.fromPoints(_vertices[0]._pos,_vertices[1]._pos,_vertices[2]._pos);
		_plane._nrm = (vector2.cross(vector1)).normalized();
		_plane._dist = _plane._nrm.dot(_vertices[0]._pos);
	}
}

void CSGPolygon::flip() 
{ 
	std::reverse(_vertices.begin(), _vertices.end()); 
	_plane.flip(); 
	for(unsigned int i=0; i<_vertices.size(); ++i) _vertices[i]._nrm = _plane._nrm;
}


//****************************************************
CSGNode::~CSGNode()
{
	delete _front;
	delete _back;
}

void CSGNode::invert()
{
	for(unsigned int i=0; i<_polygons.size(); ++i) _polygons[i].flip();
	_plane.flip();
	if( _front ) _front->invert();
	if( _back ) _back->invert();
	CSGNode* temp = _front; _front = _back; _back = temp; // swap front/back
}

CSGPolygonList CSGNode::clipPolygons(CSGPolygonList& _PolygonList)
{
	if( _plane._nrm.length()<0.1f ) return _PolygonList;
	CSGPolygonList front;
	CSGPolygonList back;
	for(unsigned int i=0; i<_PolygonList.size(); ++i) 
	{
      	_plane.split(_PolygonList[i], front, back, front, back);
    }
	if( _front ) front = _front->clipPolygons(front);
	if( _back )
	{
		back = _back->clipPolygons(back);	
		front.insert(front.end(),back.begin(),back.end()); // concatenate lists
	}
	return front;
}

void CSGNode::clipTo(CSGNode* _Node)
{
    _polygons = _Node->clipPolygons(_polygons);
    if( _front ) _front->clipTo(_Node);
    if( _back ) _back->clipTo(_Node);
}

CSGPolygonList CSGNode::allPolygons()
{
	CSGPolygonList polygons = _polygons;
	if( _front ) 
	{
		CSGPolygonList temp = _front->allPolygons();
		polygons.insert(polygons.end(),temp.begin(),temp.end()); // concatenate lists
	}
	if( _back ) 
	{
		CSGPolygonList temp = _back->allPolygons();
		polygons.insert(polygons.end(),temp.begin(),temp.end()); // concatenate lists
	}		
	return polygons;
}

void CSGNode::mergePolygons()
{
	bool finished = false;
	while(!finished)
	{
		finished = true;
		if( _polygons.size()>1 )
		{
			for(size_t i=0; i<_polygons.size()-1; ++i)
			{
				for(size_t j=i+1; j<_polygons.size(); ++j)
				{
					std::vector< std::pair<int,int> > vertex_pairs;
					for(size_t ii=0; ii<_polygons[i]._vertices.size(); ++ii)
					{
						for(size_t jj=0; jj<_polygons[j]._vertices.size(); ++jj)
						{
							const Vec3f v = _polygons[i]._vertices[ii]._pos-_polygons[j]._vertices[jj]._pos;
							if( v.lengthSquared()<0.0001f )
							{
								vertex_pairs.push_back( std::pair<int,int>(ii,jj) );
							}
						}
					}
					printf("vertex_pairs = %d\n",vertex_pairs.size());
				}
			}
		}
	}

	if( _front ) _front->mergePolygons();
	if( _back ) _back->mergePolygons();
}

void CSGNode::build(CSGPolygonList& _Polygons)
{
    if( _Polygons.empty() ) return;

    if( _plane._nrm.length()<0.1f ) _plane = _Polygons[0]._plane;
	CSGPolygonList front;
	CSGPolygonList back;
	for(size_t i=0; i<_Polygons.size(); ++i) 
	{
      	_plane.split(_Polygons[i], _polygons, _polygons, front, back);
	}
	if( !front.empty() ) 
	{
      	if( !_front ) _front = new CSGNode();
      	_front->build(front);
    }
	if( !back.empty() ) 
	{
      	if( !_back ) _back = new CSGNode();
      	_back->build(back);
    }
}

void CSGNode::draw()
{
//*
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1,1);
	glEnable(GL_LIGHTING);
	for(size_t i=0; i<_polygons.size(); ++i)
	{
		glBegin(GL_TRIANGLE_FAN);
		const Vec3f& nrm = _polygons[i]._plane._nrm;
		glNormal3fv(nrm.ptr());
		for(size_t j=0; j<_polygons[i]._vertices.size(); ++j)
		{
			const Vec3f& pos = _polygons[i]._vertices[j]._pos;
			glVertex3fv(pos.ptr());
		}
		glEnd();
	}
	glDisable(GL_POLYGON_OFFSET_FILL);
//*/
/*
	glDisable(GL_LIGHTING);
	glColor4f(1,1,1,1);
	for(size_t i=0; i<_polygons.size(); ++i)
	{
		glBegin(GL_LINE_LOOP);
		for(size_t j=0; j<_polygons[i]._vertices.size(); ++j)
		{
			const Vec3f& pos = _polygons[i]._vertices[j]._pos;
			glVertex3dv(pos.ptr());
		}
		glEnd();
	}
//*/
	if( _front ) _front->draw();
	if( _back ) _back->draw();
}

//****************************************************
void CSGObject::draw()
{
	if( _root ) _root->draw();
}

void CSGObject::createUnion(CSGObject* _Other)
{
	if( !_root || !_Other->_root ) return;

	_root->clipTo(_Other->_root);
	_Other->_root->clipTo(_root);
	_Other->_root->invert();
	_Other->_root->clipTo(_root);
	_Other->_root->invert();
	_root->build(_Other->_root->allPolygons());

	CSGPolygonList pl = _root->allPolygons();
	_root->build(pl);
//	_root->mergePolygons();
}

void CSGObject::createSubtraction(CSGObject* _Other)
{
	if( !_root || !_Other->_root ) return;

	_root->invert();
	_root->clipTo(_Other->_root);
	_Other->_root->clipTo(_root);
	_Other->_root->invert();
	_Other->_root->clipTo(_root);
	_Other->_root->invert();
	_root->build(_Other->_root->allPolygons());
	_root->invert();

	CSGPolygonList pl = _root->allPolygons();
	_root->build(pl);
//	_root->mergePolygons();
}

void CSGObject::createIntersection(CSGObject* _Other)
{
	if( !_root || !_Other->_root ) return;

	_root->invert();
	_Other->_root->clipTo(_root);
	_Other->_root->invert();
	_root->clipTo(_Other->_root);
	_Other->_root->clipTo(_root);
	_root->build(_Other->_root->allPolygons());
	_root->invert();

	CSGPolygonList pl = _root->allPolygons();
	_root->build(pl);
//	_root->mergePolygons();
}

//****************************************************
CSGObject* createCube(const Vec3f& _Scale,const Vec3f& _Offset)
{
	Polytope polytope;
	polytope.createCube();
//	mesh.createFromPolytope(&polytope);
	CSGPolygonList polygons = polytope.toPolygons();

	CSGObject* cube = new CSGObject();
	cube->_root = new CSGNode();

	for(size_t i=0; i<polygons.size(); ++i) 
	{
		for(size_t j=0; j<polygons[i]._vertices.size(); ++j) 
		{
			polygons[i]._vertices[j]._pos = polygons[i]._vertices[j]._pos*_Scale + _Offset;
		}
		polygons[i].calculateNormal();
	}

	cube->_root->build(polygons);
	return cube;
}

CSGObject* createCylinder(const Vec3f& _Scale,const Vec3f& _Offset,const int _Slices)
{
	Polytope polytope;
	polytope.createCylinder(_Slices);
//	mesh.createFromPolytope(&polytope);
	CSGPolygonList polygons = polytope.toPolygons();

	CSGObject* cylinder = new CSGObject();
	cylinder->_root = new CSGNode();

	for(size_t i=0; i<polygons.size(); ++i) 
	{
		for(size_t j=0; j<polygons[i]._vertices.size(); ++j) 
		{
			polygons[i]._vertices[j]._pos = polygons[i]._vertices[j]._pos*_Scale + _Offset;
		}
		polygons[i].calculateNormal();
	}

	cylinder->_root->build(polygons);
	return cylinder;
};

CSGObject* createSphere(const Vec3f& _Scale,const Vec3f& _Offset,const int _Points)
{
	Polytope polytope;
	polytope.createSphere(_Points);
//	mesh.createFromPolytope(&polytope);
	CSGPolygonList polygons = polytope.toPolygons();

	CSGObject* sphere = new CSGObject();
	sphere->_root = new CSGNode();

	for(size_t i=0; i<polygons.size(); ++i) 
	{
		for(size_t j=0; j<polygons[i]._vertices.size(); ++j) 
		{
			polygons[i]._vertices[j]._pos = polygons[i]._vertices[j]._pos*_Scale + _Offset;
		}
		polygons[i].calculateNormal();
	}

	sphere->_root->build(polygons);
	return sphere;
};




//*******************************************************************************
void DynamicMesh::selfIntersect()
{
/*
	CSGObject* object = new CSGObject();
	object->extractFromMesh(this);
	object->createIntersection(object);
	object->insetIntoMesh(this);
	delete object;
*/
}
