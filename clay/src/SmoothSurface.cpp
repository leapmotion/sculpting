#include "SmoothSurface.h"


#ifdef USE_SMOOTH_SURFACE

#define GLEW_STATIC
#include "GL/glew.h"
#include "algebra.h"
#include <iso646.h>
#include <far/meshFactory.h>
#include <osd/vertex.h>
#include <osd/glDrawContext.h>
#include <osd/cpuGLVertexBuffer.h>
#include <osd/cpuComputeController.h>
#include <osd/cpuComputeContext.h>


typedef OpenSubdiv::HbrVertex<OpenSubdiv::OsdVertex> OsdHbrVertex;
typedef OpenSubdiv::HbrFace<OpenSubdiv::OsdVertex> OsdHbrFace;
typedef OpenSubdiv::HbrHalfedge<OpenSubdiv::OsdVertex> OsdHbrHalfedge;
typedef OpenSubdiv::HbrMesh<OpenSubdiv::OsdVertex> OsdHbrMesh;

OpenSubdiv::FarMesh<OpenSubdiv::OsdVertex>* g_farmesh = 0;
OpenSubdiv::OsdCpuGLVertexBuffer* g_vertexBuffer = 0;
OpenSubdiv::OsdGLDrawContext* g_drawContext = 0;
OpenSubdiv::OsdCpuComputeContext* g_osdComputeContext = 0;
OpenSubdiv::OsdCpuComputeController* g_osdComputeController = 0;

void calcNormals(OsdHbrMesh* mesh, const std::vector<float>& pos, std::vector<float>& result) {
  //
  // Get the number of vertices and faces. Notice the naming convention is 
  // different between coarse Vertices and Faces. This may change in the 
  // future (it an artifact of the original renderman code).
  //
  int nverts = mesh->GetNumVertices();
  int nfaces = mesh->GetNumCoarseFaces();

  for (int i = 0; i < nfaces; ++i) {

    OsdHbrFace * f = mesh->GetFace(i);

    float const * p0 = &pos[f->GetVertex(0)->GetID()*3],
                * p1 = &pos[f->GetVertex(1)->GetID()*3],
                * p2 = &pos[f->GetVertex(2)->GetID()*3];

    float n[3];
    cross( n, p0, p1, p2 );

    for (int j = 0; j < f->GetNumVertices(); j++) {
      int idx = f->GetVertex(j)->GetID() * 3;
      result[idx  ] += n[0];
      result[idx+1] += n[1];
      result[idx+2] += n[2];
    }
  }
  for (int i = 0; i < nverts; ++i)
    normalize(&result[i*3]);
}


SmoothSurface::SmoothSurface()
	: g_size(0)
  , g_width(1024)
  , g_height(1024)
  , g_frame(0)
  , g_level(4)
{ 
	static bool glewLoaded = false;
	if (!glewLoaded)
	{
		GLenum result = glewInit();
		if (result != GLEW_OK)
		{
			const GLubyte* error = glewGetErrorString(result);
		}
		glewLoaded = true;
	}
	g_osdComputeController = new OpenSubdiv::OsdCpuComputeController();
}

//*********************************************************************************************************************
void SmoothSurface::createFromDynamicMesh(DynamicMesh& mesh) 
{
//return;
	
  // 
  // Setup an OsdHbr mesh based on the desired subdivision scheme
  //
  static OpenSubdiv::HbrCatmarkSubdivision<OpenSubdiv::OsdVertex>  _catmark;
  OsdHbrMesh *hmesh(new OsdHbrMesh(&_catmark));

#if 0
  int i = 0;
  DynamicMesh::VertexList& vertices = mesh.getVertices();
  for (DynamicMesh::VertexIterator it = vertices.begin(); it != vertices.end(); ++it) {
    DynamicMeshVertex& vertex = **it;

    g_orgPositions.push_back(vertex.getPosition().x);
    g_orgPositions.push_back(vertex.getPosition().y);
    g_orgPositions.push_back(vertex.getPosition().z);

    OpenSubdiv::OsdVertex vert;
    hmesh->NewVertex(i++, vert);
  }
#else
  int level = 4;
  //
  // Now that we have a mesh, we need to add verticies and define the topology.
  // Here, we've declared the raw vertex data in-line, for simplicity
  //
  float verts[] = {    0.000000f, -1.414214f, 1.000000f,
                      1.414214f, 0.000000f, 1.000000f,
                      -1.414214f, 0.000000f, 1.000000f,
                      0.000000f, 1.414214f, 1.000000f,
                      -1.414214f, 0.000000f, -1.000000f,
                      0.000000f, 1.414214f, -1.000000f,
                      0.000000f, -1.414214f, -1.000000f,
                      1.414214f, 0.000000f, -1.000000f
                      };

  //
  // The cube faces are also in-lined, here they are specified as quads
  //
  int faces[] = {
                      0,1,3,2,
                      2,3,5,4,
                      4,5,7,6,
                      6,7,1,0,
                      1,7,5,3,
                      6,0,2,4
                      };

  //
  // Record the original vertex positions and add verts to the mesh.
  //
  // OsdVertex is really just a place holder, it doesn't care what the 
  // position of the vertex is, it's just being used here as a means of
  // defining the mesh topology.
  //
  for (unsigned i = 0; i < sizeof(verts)/sizeof(float); i += 3) 
  {
    g_orgPositions.push_back(verts[i+0]);
    g_orgPositions.push_back(verts[i+1]);
    g_orgPositions.push_back(verts[i+2]);

    OpenSubdiv::OsdVertex vert;
    hmesh->NewVertex(i/3, vert);
  }
#endif

  //
  // Now specify the actual mesh topology by processing the faces array 
  //
  const unsigned VERTS_PER_FACE = 4;
  for (unsigned i = 0; i < sizeof(faces)/sizeof(int); i += VERTS_PER_FACE) {
    //
    // Do some sanity checking. It is a good idea to keep this in your 
    // code for your personal sanity as well.
    //
    // Note that this loop is not changing the HbrMesh, it's purely validating
    // the topology that is about to be created below.
    //
    for (unsigned j = 0; j < VERTS_PER_FACE; j++) {
      OsdHbrVertex * origin      = hmesh->GetVertex(faces[i+j]);
      OsdHbrVertex * destination = hmesh->GetVertex(faces[i+((j+1)%VERTS_PER_FACE)]);
      OsdHbrHalfedge * opposite  = destination->GetEdge(origin);

      if(origin==NULL || destination==NULL) {
        std::cerr << 
          " An edge was specified that connected a nonexistent vertex"
          << std::endl;
        exit(1);
      }

      if(origin == destination) {
        std::cerr << 
          " An edge was specified that connected a vertex to itself" 
          << std::endl;
        exit(1);
      }

      if(opposite && opposite->GetOpposite() ) {
        std::cerr << 
          " A non-manifold edge incident to more than 2 faces was found" 
          << std::endl;
        exit(1);
      }

      if(origin->GetEdge(destination)) {
        std::cerr << 
          " An edge connecting two vertices was specified more than once."
          " It's likely that an incident face was flipped" 
          << std::endl;
        exit(1);
      }
    }
    // 
    // Now, create current face given the number of verts per face and the 
    // face index data.
    //
    /* OsdHbrFace * face = */ hmesh->NewFace(VERTS_PER_FACE, faces+i, 0);

    //
    // If you had ptex data, you would set it here, for example
    //
    /* face->SetPtexIndex(ptexIndex) */

  }

  //
  // Apply some tags to drive the subdivision algorithm. Here we set the 
  // default boundary interpolation mode along with a corner sharpness. See 
  // the API and the renderman spec for the full list of available operations.
  //
  hmesh->SetInterpolateBoundaryMethod( OsdHbrMesh::k_InterpolateBoundaryEdgeOnly );

  OsdHbrVertex * v = hmesh->GetVertex(0);
  v->SetSharpness(2.7f);

  //
  // Finalize the mesh object. The Finish() call is a signal to the internals 
  // that optimizations can be made on the mesh data. 
  //
  hmesh->Finish();

  //
  // Setup some raw vectors of data. Remember that the actual point values were
  // not stored in the OsdVertex, so we keep track of them here instead
  //
  g_normals.resize(g_orgPositions.size(),0.0f);
  calcNormals( hmesh, g_orgPositions, g_normals );

  // 
  // At this point, we no longer need the topological structure of the mesh, 
  // so we bake it down into subdivision tables by converting the HBR mesh 
  // into an OSD mesh. Note that this is just storing the initial subdivision
  // tables, which will be used later during the actual subdivision process.
  //
  // Again, no vertex positions are being stored here, the point data will be 
  // sent to the mesh in updateGeom().
  //
  OpenSubdiv::FarMeshFactory<OpenSubdiv::OsdVertex> meshFactory(hmesh, level);

  g_farmesh = meshFactory.Create();

  g_osdComputeContext = OpenSubdiv::OsdCpuComputeContext::Create(g_farmesh);

  delete hmesh;

  // 
  // Initialize draw context and vertex buffer
  //
  g_vertexBuffer = 
    OpenSubdiv::OsdCpuGLVertexBuffer::Create(6,  /* 3 floats for position, 
                                                  +
                                                  3 floats for normal*/
                                                  g_farmesh->GetNumVertices());

  g_drawContext =
    OpenSubdiv::OsdGLDrawContext::Create(g_farmesh->GetPatchTables(), false);
  g_drawContext->UpdateVertexTexture(g_vertexBuffer);

  // 
  // Setup camera positioning based on object bounds. This really has nothing
  // to do with OSD.
  //
  float centre[] = {g_center.x,g_center.y,g_center.z};
  computeCenterAndSize(g_orgPositions, centre, &g_size);

  //
  // Finally, make an explicit call to updateGeom() to force creation of the 
  // initial buffer objects for the first draw call.
  //
  updateGeom();

  //
  // The OsdVertexBuffer provides GL identifiers which can be bound in the 
  // standard way. Here we setup a single VAO and enable points and normals 
  // as attributes on the vertex buffer and set the index buffer.
  //
  glGenVertexArrays(1, &g_vao);
  glBindVertexArray(g_vao);
  glBindBuffer(GL_ARRAY_BUFFER, g_vertexBuffer->BindVBO());
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glVertexPointer(3, GL_FLOAT, 6*sizeof(GLfloat), (void*)0);
	glNormalPointer(GL_FLOAT, 6*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
  glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_drawContext->GetPatchIndexBuffer());
  glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

//*********************************************************************************************************************
void SmoothSurface::updateGeom() 
{
	g_frame++;
	int nverts = (int)g_orgPositions.size() / 3;

  std::vector<float> vertex;
  vertex.reserve(nverts*6);

  const float *p = &g_orgPositions[0];
  const float *n = &g_normals[0];

  //
  // Apply a simple deformer to the coarse mesh. We save the deformed points 
  // and normals into a separate buffer to avoid accumulation of error. This 
  // loop really has nothing to do with OSD.
  // 
  float r = sin(g_frame*0.01f);
  for (int i = 0; i < nverts; ++i) {
    //float move = 0.05f*cosf(p[0]*20+g_frame*0.01f);
    float ct = cos(p[2] * r);
    float st = sin(p[2] * r);

    vertex.push_back(p[0]*ct + p[1]*st);
    vertex.push_back(-p[0]*st + p[1]*ct);
    vertex.push_back(p[2]);

    //
    // To be completely accurate, we should deform the normals here too, but
    // the original undeformed normals are sufficient for this example 
    //
    vertex.push_back(n[0]);
    vertex.push_back(n[1]);
    vertex.push_back(n[2]);

    p += 3;
    n += 3;
  }

  //
  // Send the animated coarse positions and normals to the vertex buffer.
  //
  //std::cout << vertex.size() << " - " << nverts << std::endl;
  g_vertexBuffer->UpdateData(&vertex[0], 0, nverts);

  //
  // Dispatch subdivision work based on the coarse vertex buffer. At this 
  // point, the assigned dispatcher will queue up work, potentially in many
  // worker threads. If the subdivided data is required for further processing
  // a call to Synchronize() will allow you to block until the worker threads
  // complete.
  //
  g_osdComputeController->Refine(g_osdComputeContext,
    g_farmesh->GetKernelBatches(),
    g_vertexBuffer);

  //
  // The call to Synchronize() is not actually necessary, it's being used
  // here only for illustration. 
  //
  // g_osdComputeController->Synchronize();
}

//*********************************************************************************************************************
void SmoothSurface::displaySmoothSurface() 
{
	updateGeom();

  glBindVertexArray(g_vao);
  glBindBuffer(GL_ARRAY_BUFFER, g_vertexBuffer->BindVBO());
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

  OpenSubdiv::OsdDrawContext::PatchArrayVector const & patches = g_drawContext->patchArrays;
  for (int i=0; i<(int)patches.size(); ++i) {
    OpenSubdiv::OsdDrawContext::PatchArray const & patch = patches[i];

    glDrawElements(GL_QUADS, patch.GetNumIndices(), GL_UNSIGNED_INT, NULL);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

  glBindVertexArray(0);
}

#endif // #ifdef USE_SMOOTH_SURFACE

