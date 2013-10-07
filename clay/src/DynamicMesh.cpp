#include "DynamicMesh.h"

#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"

//********************** DynamicMeshVertex Methods **********************
void DynamicMeshVertex::calculateNormal()
{
	// *** iterate around the vertex, accumulating normals of attached faces ***
	DynamicMeshHalfedge* starthalfedge = _halfedge->Twin(); 
	if( !starthalfedge ) return; // for non manifold edges
	DynamicMeshHalfedge* lefthalfedge = starthalfedge;
	DynamicMeshHalfedge* righthalfedge;

	// *** zero the normal ***
	Vec3f new_nrm = Vec3f(0,0,0);

	bool all_the_way_round = false;
	int num_contributions = 0;
	int iter = 0;
	while( true )
	{
		righthalfedge = lefthalfedge->Next()->Twin();
		if( !righthalfedge ) break;

		new_nrm += (righthalfedge->Back()->_pos-_pos).cross(lefthalfedge->Back()->_pos-_pos);
		num_contributions++;
		lefthalfedge = righthalfedge;

		if( lefthalfedge==starthalfedge )
		{
			all_the_way_round = true;
			break;
		}

		if( (iter++)>20 ) break;
	} 

	if( !all_the_way_round )
	{
		// *** found an untwinned edge, so now go the other way *** 
		iter = 0;
		righthalfedge = starthalfedge;
		while( true )
		{
			DynamicMeshHalfedge* twin = righthalfedge->Twin();
			if( !twin ) break;
			lefthalfedge = twin->Prev();
			new_nrm += (righthalfedge->Back()->_pos-_pos).cross(lefthalfedge->Back()->_pos-_pos);
			num_contributions++;
			righthalfedge = lefthalfedge;
			if( (iter++)>20 ) break;
		}
	}

	// *** now normalise the normal ***
	if( new_nrm.length()>0.001f ) _nrm = new_nrm.normalized();
}

void DynamicMeshVertex::smooth(const float _Strength)
{
	// *** iterate around the vertex, accumulating normals of attached faces ***
	DynamicMeshHalfedge* starthalfedge = _halfedge; 
//	if( !starthalfedge ) return; // for non manifold edges
	DynamicMeshHalfedge* currenthalfedge = starthalfedge;

	std::vector<DynamicMeshVertex*> neighbours;

	bool all_the_way_round = false;
//	int iter = 0;
	while( true )
	{
		neighbours.push_back(currenthalfedge->Front());

		DynamicMeshHalfedge* twin = currenthalfedge->Twin();
		if( !twin ) break;
		currenthalfedge = twin->Next();

		if( currenthalfedge==starthalfedge )
		{
			all_the_way_round = true;
			break;
		}

//		if( (iter++)>20 ) break;
	} 
	if( !all_the_way_round )
	{
		// *** found an untwinned edge, so now go the other way *** 
//		iter = 0;
		currenthalfedge = starthalfedge->Prev();
		while( true )
		{
			neighbours.push_back(currenthalfedge->Back());

			DynamicMeshHalfedge* twin = currenthalfedge->Twin();
			if( !twin ) break;
			currenthalfedge = twin->Prev();

//			if( (iter++)>20 ) break;
		}
	}
	// *** move vertex towards average of neighbours ***
	if( !neighbours.empty() ) 
	{
//		const float vertex_contribution = 1.f/(neighbours.size()+1);

		// *** zero the average position ***
		Vec3f aver = Vec3f(0,0,0);

		// *** calculate neighbour average position ***
		ci::app::console() << "start:" << "\n";
		for(size_t i=0; i<neighbours.size(); ++i)
		{
			ci::app::console() << "  " << neighbours[i]->getPosition() << "\n";
			aver += neighbours[i]->getPosition();
		}
		aver *= 1.f/float(neighbours.size());
		ci::app::console() << "     " << aver << "\n";
		ci::app::console() << "     " << _pos << "\n";

		// *** weighted vector from position to neighbour average ***
		Vec3f diff = (aver-_pos)*_Strength;

		// *** move vertex most of the way *** 
#if 1
		modifyPosition(_nrm * _nrm.dot(diff));
#else
		modifyPosition(diff*(1.f-vertex_contribution));

		// *** move neighbours a bit the other way (to reduce volume loss) ***
/*
		diff *= -vertex_contribution;
		for(size_t i=0; i<neighbours.size(); ++i)
		{
			neighbours[i]->modifyPosition(diff);
		}
*/
#endif
	}

}


//********************** DynamicMeshHalfedge Methods **********************
void DynamicMeshHalfedge::collapse(DynamicMesh* _Mesh)
{
	if( !_twin ) return; // need to make this method more robust

	// *** get pointers to affected vertices and halfedges ***
	DynamicMeshHalfedge* upperleft  = Next();
	DynamicMeshHalfedge* lowerleft  = Prev();
	DynamicMeshHalfedge* lowerright = _twin->Next();
	DynamicMeshHalfedge* upperright = _twin->Prev();

	DynamicMeshHalfedge* upperlefttwin  = upperleft->Twin();
	DynamicMeshHalfedge* lowerlefttwin  = lowerleft->Twin();
	DynamicMeshHalfedge* lowerrighttwin = lowerright->Twin();
	DynamicMeshHalfedge* upperrighttwin = upperright->Twin();

	DynamicMeshVertex* back  = Back(); 
	DynamicMeshVertex* front = Front(); 
	DynamicMeshVertex* left  = Left(); 
	DynamicMeshVertex* right = Right();

	// *** move 'back' to centre of edge ***
	back->setPosition( (back->getPosition()+front->getPosition())*0.5f );


	//****************************************************************
	//****************************************************************
	// there could be errors with the following if it finds a null twin
	// best to iterate over every single edge really
	//****************************************************************
	//****************************************************************

	// *** traverse half edges connected to 'front' and set their base to 'back' ***
	DynamicMeshHalfedge* fronthalfedge = _twin;
	int iter = 0;
	do
	{
		fronthalfedge->setBack(back);
		fronthalfedge = fronthalfedge->Twin()->Next(); // go to next 'spoke' of front
		if( (iter++)>20 ) break;
	} while( fronthalfedge!=_twin );

	// ** sew up edges ***
/*
	upperlefttwin->joinTwin( lowerlefttwin );
	upperrighttwin->joinTwin( lowerrighttwin );
/*/
	if( upperlefttwin ) upperlefttwin->setTwin( lowerlefttwin );
	if( lowerlefttwin ) lowerlefttwin->setTwin( upperlefttwin );
	if( upperrighttwin ) upperrighttwin->setTwin( lowerrighttwin );
	if( lowerrighttwin ) lowerrighttwin->setTwin( upperrighttwin );
//*/

	// *** make sure left and right 'halfedges' are valid
	upperlefttwin->setBack(left); 
	lowerrighttwin->setBack(right);

	// *** delete ***
	front->setDead(); _Mesh->_num_vertices_to_be_deleted++;

	this->setDead(); _Mesh->_num_halfedges_to_be_deleted++;
	_twin->setDead(); _Mesh->_num_halfedges_to_be_deleted++;
	upperleft->setDead(); _Mesh->_num_halfedges_to_be_deleted++;
	lowerleft->setDead(); _Mesh->_num_halfedges_to_be_deleted++;
	lowerright->setDead(); _Mesh->_num_halfedges_to_be_deleted++;
	upperright->setDead(); _Mesh->_num_halfedges_to_be_deleted++;

	upperleft->setProcessed(true);
	lowerleft->setProcessed(true);
	lowerright->setProcessed(true);
	upperright->setProcessed(true);

}

void DynamicMeshHalfedge::split(DynamicMesh* _Mesh)
{
	if( !_twin ) return; // need to make this method more robust

	// *** get pointers to affected vertices and halfedges ***
	DynamicMeshHalfedge* upperleft  = Next();
//	DynamicMeshHalfedge* lowerleft  = Prev();
	DynamicMeshHalfedge* lowerright = _twin->Next();
	DynamicMeshHalfedge* upperright = _twin->Prev();

	DynamicMeshHalfedge* upperlefttwin = upperleft->Twin();
//	DynamicMeshHalfedge* lowerlefttwin = lowerleft->Twin();
//	DynamicMeshHalfedge* lowerrighttwin = lowerright->Twin();
	DynamicMeshHalfedge* upperrighttwin = upperright->Twin();

	DynamicMeshVertex* back  = Back(); 
	DynamicMeshVertex* front = Front(); 
	DynamicMeshVertex* left  = Left(); 
	DynamicMeshVertex* right = Right();
	const Vec3f mid_pos = (back->getPosition()+front->getPosition())*0.5f;
	const Vec3f mid_nrm = (back->getNormal()+front->getNormal()).normalized();
	DynamicMeshVertex* mid = _Mesh->addVertex( mid_pos, mid_nrm );

	// *** create 2 new faces for upper 2 faces ***
	DynamicMeshHalfedge* newedge = _Mesh->addHalfEdge(mid);
	DynamicMeshHalfedge* newedgenext = _Mesh->addHalfEdge(front);
	DynamicMeshHalfedge* newedgeprev = _Mesh->addHalfEdge(left);

	DynamicMeshHalfedge* newtwin = _Mesh->addHalfEdge(front);
	DynamicMeshHalfedge* newtwinnext = _Mesh->addHalfEdge(mid);
	DynamicMeshHalfedge* newtwinprev = _Mesh->addHalfEdge(right);

	// *** connect up 'nexts' for new faces ***
	newedge->setNext(newedgenext); 
	newedgenext->setNext(newedgeprev); 
	newedgeprev->setNext(newedge); 
	
	newtwin->setNext(newtwinnext); 
	newtwinnext->setNext(newtwinprev); 
	newtwinprev->setNext(newtwin); 

	// *** connect up 'twins' for new faces ***
	newedge->setTwin(newtwin); 
	newtwin->setTwin(newedge); 

	newedgenext->setTwin(upperlefttwin); 
	if(upperlefttwin) upperlefttwin->setTwin(newedgenext); 

	newedgeprev->setTwin(upperleft); 
	if(upperleft) upperleft->setTwin(newedgeprev); 

	newtwinnext->setTwin(upperright); 
	if(upperright) upperright->setTwin(newtwinnext); 

	newtwinprev->setTwin(upperrighttwin); 
	if(upperrighttwin) upperrighttwin->setTwin(newtwinprev); 

	// *** set 'back' for lower face edges ***
	upperleft->setBack(mid); // this is _HalfEdge->front = mid
	_twin->setBack(mid); // this is twin->back = mid
}

void DynamicMeshHalfedge::rotate(DynamicMesh* _Mesh)
{
	if( !_twin ) return; // cant rotate if not manifold

	// *** get pointers to affected vertices and halfedges ***
	DynamicMeshHalfedge* upperleft = Next();
	DynamicMeshHalfedge* lowerleft = Prev();
	DynamicMeshHalfedge* lowerright = _twin->Next();
	DynamicMeshHalfedge* upperright = _twin->Prev();

	DynamicMeshHalfedge* upperlefttwin = upperleft->Twin();
	DynamicMeshHalfedge* lowerlefttwin = lowerleft->Twin();
	DynamicMeshHalfedge* lowerrighttwin = lowerright->Twin();
	DynamicMeshHalfedge* upperrighttwin = upperright->Twin();

	DynamicMeshVertex* back = _back; 
	DynamicMeshVertex* front = Front(); 
	DynamicMeshVertex* left = Left(); 
	DynamicMeshVertex* right = Right();

	// *** set 'twins' for rotated edges ***
	upperleft->setTwin( lowerlefttwin ); 
	if( lowerlefttwin ) lowerlefttwin->setTwin( upperleft ); 

	lowerleft->setTwin( lowerrighttwin ); 
	if( lowerrighttwin ) lowerrighttwin->setTwin( lowerleft ); 

	lowerright->setTwin( upperrighttwin ); 
	if( upperrighttwin ) upperrighttwin->setTwin( lowerright ); 

	upperright->setTwin( upperlefttwin ); 
	if( upperlefttwin ) upperlefttwin->setTwin( upperright ); 

	// *** set 'verts' for rotated edges ***
	upperleft->setBack( left ); 
	lowerleft->setBack( back );
	lowerright->setBack( right ); 
	upperright->setBack( front );

	this->setBack( right ); 
	_twin->setBack( left ); 
}

//********************** DynamicMesh Methods **********************
DynamicMesh::DynamicMesh()
	: _num_vertices_to_be_deleted(0)
	, _num_halfedges_to_be_deleted(0)
	, _dirty(true)
{
}

DynamicMesh::~DynamicMesh()
{
	clear(); // clean up
}

void DynamicMesh::clear()
{
	// *** clear arrays ***
	for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); ++i) delete (*i);
	_vertices.clear();
	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i) delete (*i);
	_halfedges.clear();
}

DynamicMeshVertex* DynamicMesh::addVertex(const Vec3f& _Pos,const Vec3f& _Nrm)
{
	DynamicMeshVertex* vertex = new DynamicMeshVertex();
	vertex->setPosition(_Pos);
	vertex->setNormal(_Nrm);
	_vertices.push_back(vertex);
	return vertex;
}

DynamicMeshHalfedge* DynamicMesh::addHalfEdge(DynamicMeshVertex* _Vert,DynamicMeshHalfedge* _Next,DynamicMeshHalfedge* _Twin)
{
	DynamicMeshHalfedge* halfedge = new DynamicMeshHalfedge();
	halfedge->setBack(_Vert);
	halfedge->setNext(_Next);
	halfedge->setTwin(_Twin);
	if( _Twin ) _Twin->setTwin(halfedge);
	_halfedges.push_back(halfedge);
	return halfedge;
}

void DynamicMesh::deleteDead()
{
	// *** delete dead vertices ***
	if( _num_vertices_to_be_deleted>0 )
	{
		for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); )
		{
			VertexIterator ii = i; ++i;
			if( (*ii)->isDead() ) _vertices.erase(ii);
		}
		_num_vertices_to_be_deleted = 0;
	}

	// *** delete dead halfedges ***
	if( _num_halfedges_to_be_deleted>0 )
	{
		for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); )
		{
			HalfedgeIterator ii = i; ++i;
			if( (*ii)->isDead() ) _halfedges.erase(ii);
		}
		_num_halfedges_to_be_deleted = 0;
	}
}

void DynamicMesh::calculateAllNormals() // faster than calling calculateNormal for each vertex individually
{
	// *** first zero all normals ***
	for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); ++i) (*i)->setNormal( Vec3f(0,0,0) );

	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i) 
	{
		(*i)->setProcessed(false); // clear processed flag
	}

	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i)
	{
		DynamicMeshHalfedge* edge = (*i);
		if( !edge->getProcessed() )
		{
			// *** get the three vertices for this triangle ***
			DynamicMeshVertex* vertex0 = edge->Back(); edge->setProcessed(true); edge = edge->Next();
			DynamicMeshVertex* vertex1 = edge->Back(); edge->setProcessed(true); edge = edge->Next();
			DynamicMeshVertex* vertex2 = edge->Back(); edge->setProcessed(true);

			// *** calculate normal for this triangle ***
			const Vec3f nrm = (vertex1->getPosition()-vertex0->getPosition()).cross(vertex2->getPosition()-vertex0->getPosition() );

			// *** add this normal to vertices' normals ***
			vertex0->modifyNormal( nrm );
			vertex1->modifyNormal( nrm );
			vertex2->modifyNormal( nrm );
		}
	}

	// *** now normalise all normals ***
	for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); ++i) (*i)->setNormal( (*i)->getNormal().normalized() );
}

/***************************************************************
	autogenerate all 'twin' pointers
 ***************************************************************/
bool DynamicMesh::isTriangular()
{
	ci::app::console() << "DynamicMesh::isTriangular(): ";

	// *** are all faces in triples? Do 3 'nexts' make a loop? ***

	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i) 
	{
		(*i)->setProcessed(false); // clear processed flag
	}

	bool istriangular = true;
	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i)
	{
		if( !(*i)->getProcessed() )
		{
			DynamicMeshHalfedge* edge1 = (*i)->Next();
			DynamicMeshHalfedge* edge2 = edge1->Next();
			DynamicMeshHalfedge* edge3 = edge2->Next();
			if( edge3->Next()!=edge1 ) 
			{
				// *** not a triangular loop ***
				istriangular = false;
				break;
			}
			edge1->setProcessed(true);
			edge2->setProcessed(true);
			edge3->setProcessed(true);
		}
	}

	if( istriangular ) 
		ci::app::console() << "YES\n"; 
	else
		ci::app::console() << "NO\n"; 

	return istriangular;
}

bool DynamicMesh::isManifold()
{
	ci::app::console() << "DynamicMesh::isManifold(): ";

	// *** are all twins pointing to each other? ***
	bool ismanifold = true;
	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i)
	{
		if( (*i)->Twin() )
		{
			if( (*i)->Twin()->Twin()!=(*i) )
			{
				// *** edge has twin but it is twinned with another edge ***
				ismanifold = false;
				break;
			}
		}
		else
		{
			// *** edge has no twin ***
			ismanifold = false;
			break;
		}
	}

	if( ismanifold ) 
		ci::app::console() << "YES\n"; 
	else
		ci::app::console() << "NO\n"; 

	return ismanifold;
}

size_t createHashKey(const size_t _P1,const size_t _P2)
{
	return (_P1 + _P2*31) % 1610612741;
}

void DynamicMesh::makeManifold()
{
	ci::app::console() << "DynamicMesh::makeManifold()\n";
//*
	// *** link 'twins' ***
	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i)
	{
		if( !(*i)->Twin() ) // if this twin is empty, try to find it
		{
			for(HalfedgeIterator j=i; j!=_halfedges.end(); ++j)
			{
				if( ((*i)->Back()==(*j)->Front()) && ((*i)->Front()==(*j)->Back()) )
				{
					// *** found twins, link them ***
					(*i)->setTwin( (*j) );
					(*j)->setTwin( (*i) );
				}
			}
		}
	}
/*/
	std::map< size_t, std::vector<DynamicMeshHalfedge*> > hash_map;

	for(HalfedgeIterator e=_halfedges.begin(); e!=_halfedges.end(); ++e)
	{
		DynamicMeshHalfedge* edge = *e;
		const size_t key = createHashKey( size_t(edge->Back()), size_t(edge->Front()) );
		std::vector<DynamicMeshHalfedge*>& elist = hash_map[key];
		elist.push_back(edge);
	}
ci::app::console() << "hash_map.size() " << hash_map.size() << "\n";
	for(HalfedgeIterator e=_halfedges.begin(); e!=_halfedges.end(); ++e)
	{
		DynamicMeshHalfedge* edge = *e;
		const size_t key = createHashKey( size_t(edge->Front()), size_t(edge->Back()) );
		std::vector<DynamicMeshHalfedge*>& elist = hash_map[key];
ci::app::console() << "elist.size() " << elist.size() << "\n";
		for(size_t i=0; i<elist.size(); ++i)
		{
			if( edge!=elist[i] )
			{
				if( (edge->Back()==elist[i]->Front()) && (edge->Front()==elist[i]->Back()) )
				{
					// *** found twins, link them ***
					elist[i]->setTwin( elist[i] );
					edge->setTwin( edge );
				}
			}
		}
	}
	isManifold();
//*/
}

void DynamicMesh::createTetrahedron(const Vec3f& _Pos,const float _Radius)
{
	DynamicMeshVertex* vertices[4];
	DynamicMeshHalfedge* halfedges[12];

	clear(); // clean up

	// *** create vertices ***
	const float one_over_root_two = 1.f/sqrt(2.f);
	vertices[0] = addVertex( Vec3f( -1.f,  0.f, -one_over_root_two )*_Radius );
	vertices[1] = addVertex( Vec3f(  1.f,  0.f, -one_over_root_two )*_Radius );
	vertices[2] = addVertex( Vec3f(  0.f, -1.f,  one_over_root_two )*_Radius );
	vertices[3] = addVertex( Vec3f(  0.f,  1.f,  one_over_root_two )*_Radius );

	// *** create triangles ***
	halfedges[ 0] = addHalfEdge( vertices[0] ); halfedges[ 1] = addHalfEdge( vertices[1] ); halfedges[ 2] = addHalfEdge( vertices[2] );
	halfedges[ 3] = addHalfEdge( vertices[0] ); halfedges[ 4] = addHalfEdge( vertices[3] ); halfedges[ 5] = addHalfEdge( vertices[1] );
	halfedges[ 6] = addHalfEdge( vertices[0] ); halfedges[ 7] = addHalfEdge( vertices[2] ); halfedges[ 8] = addHalfEdge( vertices[3] );
	halfedges[ 9] = addHalfEdge( vertices[1] ); halfedges[10] = addHalfEdge( vertices[3] ); halfedges[11] = addHalfEdge( vertices[2] );

	// *** connect 'nexts' for triangles ***
	halfedges[ 0]->setNext( halfedges[ 1] ); halfedges[ 1]->setNext( halfedges[ 2] ); halfedges[ 2]->setNext( halfedges[ 0] );
	halfedges[ 3]->setNext( halfedges[ 4] ); halfedges[ 4]->setNext( halfedges[ 5] ); halfedges[ 5]->setNext( halfedges[ 3] );
	halfedges[ 6]->setNext( halfedges[ 7] ); halfedges[ 7]->setNext( halfedges[ 8] ); halfedges[ 8]->setNext( halfedges[ 6] );
	halfedges[ 9]->setNext( halfedges[10] ); halfedges[10]->setNext( halfedges[11] ); halfedges[11]->setNext( halfedges[ 9] );

	// autogenerate all the 'twin' pointers
	makeManifold();
}

void DynamicMesh::collapseShortEdges(const float _LowerThreshold)
{
//ci::app::console() << "collapseShortEdges() start: "; isManifold();

	if( _vertices.size()<100 ) return; // leave the poor wee thing alone

	const float lowerthreshold2 = _LowerThreshold*_LowerThreshold; // squared value for optimisation

	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i) (*i)->setProcessed(false); // clear processed flag

	for(HalfedgeReverseIterator ri=_halfedges.rbegin(); ri!=_halfedges.rend(); ++ri) // go backwards because list may grow from back during this loop
	{
		DynamicMeshHalfedge* edge = (*ri);

		if( edge->getProcessed() ) continue;

		if( edge->isDead() ) continue;

		DynamicMeshVertex* back = edge->Back();
		DynamicMeshVertex* front = edge->Front();

		if( back->isDirty() || front->isDirty() )
		{
			const float l2 = (front->getPosition()-back->getPosition()).lengthSquared(); // distance between back and front points squared
			if( l2<lowerthreshold2 ) // edge smaller than lower threshold
			{
				edge->collapse(this);
				_dirty = true;
break;
			}
		}

		edge->setProcessed(true);
		if( edge->Twin() ) edge->Twin()->setProcessed(true);
	}
//ci::app::console() << "collapseShortEdges() end: "; isManifold();
}

void DynamicMesh::splitLongEdges(const float _UpperThreshold)
{
//ci::app::console() << "splitLongEdges() start: "; isManifold();
	const float upperthreshold2 = _UpperThreshold*_UpperThreshold; // squared value for optimisation

	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i) (*i)->setProcessed(false); // clear processed flag

	for(HalfedgeReverseIterator ri=_halfedges.rbegin(); ri!=_halfedges.rend(); ++ri) // go backwards because list may grow from back during this loop
	{
		DynamicMeshHalfedge* edge = (*ri);

		if( edge->getProcessed() ) continue;

		if( edge->isDead() ) continue;

		DynamicMeshVertex* back = edge->Back();
		DynamicMeshVertex* front = edge->Front();

		if( back->isDirty() || front->isDirty() )
		{
			const float l2 = (front->getPosition()-back->getPosition()).lengthSquared(); // distance between back and front points squared
			if( l2>upperthreshold2 ) // edge greater than upper threshold
			{
//static int blob = 0; if( (blob++)<1 )
				edge->split(this);
				_dirty = true;
break;
			}
		}

		edge->setProcessed(true);
		if( edge->Twin() ) edge->Twin()->setProcessed(true);
	}
//ci::app::console() << "splitLongEdges() end: "; isManifold();
}

void DynamicMesh::flipLongFaces()
{
//ci::app::console() << "flipLongFaces() start: "; isManifold();
	if( _vertices.size()<100 ) return; // leave the poor wee thing alone

	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i) (*i)->setProcessed(false); // clear processed flag

	for(HalfedgeReverseIterator ri=_halfedges.rbegin(); ri!=_halfedges.rend(); ++ri) // go backwards because list may grow from back during this loop
	{
		DynamicMeshHalfedge* edge = (*ri);

		if( edge->getProcessed() ) continue;

		if( edge->isDead() ) continue;

		DynamicMeshVertex* back = edge->Back();
		DynamicMeshVertex* front = edge->Front();

		if( back->isDirty() || front->isDirty() )
		{
			DynamicMeshVertex* left = edge->Left();
			DynamicMeshVertex* right = edge->Right();

			const float l2 = (front->getPosition()-back->getPosition()).lengthSquared(); // distance between back and front points squared
			const float k2 = (right->getPosition()-left->getPosition()).lengthSquared(); // distance between left and right points squared
			if( k2<l2*0.8f ) 
			{
				edge->rotate(this); // change back->front to left->right
				_dirty = true;
break;
			}
		}

		edge->setProcessed(true);
		if( edge->Twin() ) edge->Twin()->setProcessed(true);
	}
//ci::app::console() << "flipLongFaces() end: "; isManifold();
}

void DynamicMesh::refineMesh(const float _LowerThreshold,const float _UpperThreshold)
{
	// *** only perform 1 expensive operation per frame ***
	static int mode = 0;
	switch( (mode++)%4 )
	{
		case 0: collapseShortEdges(_LowerThreshold); break;
		case 1: splitLongEdges(_UpperThreshold); break;
		case 2: flipLongFaces(); break;
		case 3: selfIntersect(); break;
	}

	if( _dirty )
	{
		deleteDead();

		// *** clean up dirty vertices ***
		for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); ++i)
		{
			DynamicMeshVertex* vertex = (*i);
			if( vertex->isDirty() ) vertex->calculateNormal();
			vertex->setDirty( false );
		}
		_dirty = false;
	}
//calculateAllNormals();

//	isTriangular();
//	isManifold();
//makeManifold();
//	ci::app::console() << _vertices.size() << " " << _halfedges.size() << "\n"; 
}

void DynamicMesh::refineWholeMesh(const float _LowerThreshold,const float _UpperThreshold)
{
	for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); ++i) (*i)->setDirty( true );
	collapseShortEdges(_LowerThreshold);
//	deleteDead();
	for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); ++i) (*i)->setDirty( true );
	splitLongEdges(_UpperThreshold);
//	deleteDead();
	for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); ++i) (*i)->setDirty( true );
	flipLongFaces();
//	deleteDead();
	for(VertexIterator i=_vertices.begin(); i!=_vertices.end(); ++i) (*i)->setDirty( true );
	selfIntersect();
//	deleteDead();

	calculateAllNormals();
}

int DynamicMesh::loadOBJFile(const std::string& _Filename)
{
	ci::app::console() << "DynamicMesh::loadOBJFile(" << _Filename << ")\n";

	std::ifstream fin( _Filename );
	if( !fin ) 
	{ 
		ci::app::console() << "Can't load obj file\n"; 
		return -1; // error
	}

	clear(); // clear all

	std::vector<DynamicMeshVertex*> vertices;

	do
	{
		char buffer[256];
		fin.getline(buffer,256);
		std::istringstream iss(buffer);
		std::string str;

		iss >> str;
		if( !str.compare("v") ) // new vertex
		{
			Vec3f p;
			for(int k=0; k<3; ++k) iss >> p[k];
			vertices.push_back( addVertex(p /* +ci::randVec3f()*0.1f */) );
		}
		else if( !str.compare("f") ) // new face(s)
		{
			DynamicMeshHalfedge* halfedges[3];
			for(int i=0; i<3; ++i) 
			{
				int vindex;
				iss >> vindex;
				halfedges[i] = addHalfEdge(vertices[vindex-1]);
			}
			halfedges[0]->setNext(halfedges[1]);
			halfedges[1]->setNext(halfedges[2]);
			halfedges[2]->setNext(halfedges[0]);
		}
	} while( !fin.eof() );

	fin.close();

	isTriangular(); // check for triangularness
	makeManifold(); // join up twins
	isManifold(); // check for manifoldness
	calculateAllNormals();
//	refineMesh(0.2f,0.5f);

	return 0;
}

int DynamicMesh::saveOBJFile(const std::string& _Filename)
{
	ci::app::console() << "DynamicMesh::saveOBJFile(" << _Filename << ")\n";

	std::ofstream fout( _Filename );
	if( !fout ) 
	{ 
		ci::app::console() << "Can't save obj file\n"; 
		return -1; // error
	}

	// *** save vertex positions (and store in map for easy indexing) ***
	std::map<DynamicMeshVertex*,int> vert_index_map;
	int index = 0;
	for(VertexList::iterator i=_vertices.begin(); i!=_vertices.end(); ++i)
	{
		Vec3f pos = (*i)->getPosition();
		fout << "v " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
		vert_index_map[(*i)] = index++;
	}

	// *** save triangle data ***
	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i) 
	{
		(*i)->setProcessed(false); // clear rendered flag
	}

	for(HalfedgeIterator i=_halfedges.begin(); i!=_halfedges.end(); ++i)
	{
		DynamicMeshHalfedge* edge = (*i);
		if( !edge->getProcessed() )
		{
			fout << "f ";
			// *** draw triangle ***
			for(int j=0; j<3; j++)
			{
				DynamicMeshVertex* vertex = edge->Back();
				fout << vert_index_map[vertex]+1 << " ";
				edge = edge->Next();
			}
			fout << std::endl;
		}
	}

	fout.close();

	return 0;
}

