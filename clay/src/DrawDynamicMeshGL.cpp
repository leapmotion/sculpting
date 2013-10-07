#include "DynamicMesh.h"
#include "cinder/app/AppNative.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"

void DynamicMesh::draw()
{
	/*
	 * assumes mesh is composed only of trinagles
	 * the 3 halfedges that make up a triangle may not be contiguous in the list 
	 * so we use next and next->next to get the edges and their vertices
	 */

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.f,1.f);

	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i) 
	{
		(*i)->setProcessed(false); // clear rendered flag
	}

	glBegin(GL_TRIANGLES);
	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i)
	{
		DynamicMeshHalfedge* edge = (*i);
		if( !edge->getProcessed() )
		{
			// *** draw triangle ***
			for(int j=0; j<3; j++)
			{
				DynamicMeshVertex* vertex = edge->Back();
				glNormal3fv((GLfloat*)&(vertex->getNormal()));
				glVertex3fv((GLfloat*)&(vertex->getPosition()));
				edge->setProcessed(true);
				edge = edge->Next();
			}
		}
	}
	glEnd();
}

void DynamicMesh::drawEdges()
{
	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i) (*i)->setProcessed(false); // set rendered flag

	glBegin(GL_LINES);
	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i)
	{
		DynamicMeshHalfedge* edge = (*i);
		if( !edge->getProcessed() )
		{
			DynamicMeshVertex* back = edge->Back();
			DynamicMeshVertex* front = edge->Front();

			glVertex3fv((GLfloat*)&(back->getPosition()));
			glVertex3fv((GLfloat*)&(front->getPosition()));

			edge->setProcessed(true);
			DynamicMeshHalfedge* twin = edge->Twin();
			if( twin ) twin->setProcessed(true);
		}
	}
	glEnd();
}

void DynamicMesh::drawNormals(const float _Length)
{
	glBegin(GL_LINES);
	for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); ++i)
	{
		DynamicMeshVertex* vertex = (*i);
		glVertex3fv((GLfloat*)&(vertex->getPosition()));
		Vec3f end = vertex->getPosition()+vertex->getNormal()*_Length;
		glVertex3fv((GLfloat*)&end);
	}
	glEnd();
}

