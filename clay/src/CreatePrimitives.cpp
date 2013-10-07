#include "CreatePrimitives.h"
#include "cinder/gl/gl.h"


/***************************************************

Create a Primitive.
These functions work by setting up an unconnected 
list of vertices (assuming triangles are 3 
contiguous vertices in list)
and then a call to createPrimitive() actually
creates the dynamic mesh from the list

***************************************************/

std::vector<Vec3f> primitive_vertices;


DynamicMesh* createPrimitive()
{
	DynamicMesh* mesh = new DynamicMesh();

	// *** make unduplicated vertex list ***
	std::vector<size_t> mapping; // maps duplicate vertex index to unduplicated vertex index
	std::vector<Vec3f> mesh_vertices; // mesh list with no duplicates
	for(size_t i=0; i<primitive_vertices.size(); ++i)
	{
		size_t index = (size_t)mesh_vertices.size();

		bool found_duplicate = false;
		for(size_t j=0; j<mesh_vertices.size(); ++j)
		{
			if( (primitive_vertices[i]-mesh_vertices[j]).length()<0.01f )
			{
				index = j;
				found_duplicate = true;
				break;
			}
		}
		if( !found_duplicate )
		{
			mesh_vertices.push_back(primitive_vertices[i]);
		}
		mapping.push_back(index);
	}

	// *** create mesh vertices ***
	std::vector<DynamicMeshVertex*> mesh_vertex_array; // store pointers in vector for easy lookup
	for(size_t i=0; i<mesh_vertices.size(); ++i)
	{
		mesh_vertex_array.push_back( mesh->addVertex(mesh_vertices[i]) );
	}


	// *** create mesh triangles (3 halfedges) ***
	for(size_t i=0; i<primitive_vertices.size(); i+=3)
	{
		// *** create edges based on unduplicated vertices ***
		DynamicMeshHalfedge* e1 = mesh->addHalfEdge( mesh_vertex_array[ mapping[i+0] ] );
		DynamicMeshHalfedge* e2 = mesh->addHalfEdge( mesh_vertex_array[ mapping[i+1] ] );
		DynamicMeshHalfedge* e3 = mesh->addHalfEdge( mesh_vertex_array[ mapping[i+2] ] );

		// *** connect up edges into triangle ***
		e1->setNext(e2);
		e2->setNext(e3);
		e3->setNext(e1);
	}

	// *** connect up mesh twins ***
	mesh->makeManifold();

	// *** calculate normals ***
	mesh->calculateAllNormals();

	return mesh;
}

void addPrimitiveVertex(const Vec3f& _Pos)
{
	primitive_vertices.push_back(_Pos);
}

//*********************************************
void createSpherePatch(const GLuint _Level,const Vec3f& _Pos1,const Vec3f& _Pos2,const Vec3f& _Pos3)
{
	if( _Level==0 )
	{
		primitive_vertices.push_back(_Pos1);
		primitive_vertices.push_back(_Pos2);
		primitive_vertices.push_back(_Pos3);
	}
	else
	{
		Vec3f new_verts[3];
		new_verts[0] = (_Pos1+_Pos2).normalized();
		new_verts[1] = (_Pos2+_Pos3).normalized();
		new_verts[2] = (_Pos3+_Pos1).normalized();
		createSpherePatch( _Level-1, _Pos1, new_verts[0], new_verts[2] );
		createSpherePatch( _Level-1, new_verts[0], _Pos2, new_verts[1] );
		createSpherePatch( _Level-1, new_verts[2], new_verts[1], _Pos3 );
		createSpherePatch( _Level-1, new_verts[2], new_verts[0], new_verts[1] );
	}
}

DynamicMesh* createSphere(const float _Radius,const int _Level)
{
	std::vector<Vec3f> sphere_vertices;

	// *** add icosahedron vertices ***
	sphere_vertices.push_back(Vec3f(0,1,0));
	float phi = float(M_PI)*1.f/3.f;
	for(int i=0; i<5; ++i)
	{
		float theta = float(M_PI)*2.f*(0.f+i)/5.f;
		sphere_vertices.push_back(Vec3f(cosf(theta),cosf(phi),sinf(theta)));
	}
	phi = float(M_PI)*2.f/3.f;
	for(int i=0; i<5; ++i)
	{
		float theta = float(M_PI)*2.f*(0.5f+i)/5.f;
		sphere_vertices.push_back(Vec3f(cosf(theta),cosf(phi),sinf(theta)));
	}
	sphere_vertices.push_back(Vec3f(0,-1,0));

	// *** add icosahedron faces ***
	for(int i=0; i<5; ++i) 
	{
		const int j = (i+1)%5;
		createSpherePatch( _Level-1, sphere_vertices[0  ], sphere_vertices[j+1], sphere_vertices[i+1] );
		createSpherePatch( _Level-1, sphere_vertices[i+1], sphere_vertices[j+1], sphere_vertices[i+6] );
		createSpherePatch( _Level-1, sphere_vertices[j+1], sphere_vertices[j+6], sphere_vertices[i+6] );
		createSpherePatch( _Level-1, sphere_vertices[i+6], sphere_vertices[j+6], sphere_vertices[11 ] );
	}

	for(size_t i=0; i<primitive_vertices.size(); ++i) 
	{
		primitive_vertices[i].normalize();
		primitive_vertices[i] *= _Radius;
	}

	return createPrimitive();
}


