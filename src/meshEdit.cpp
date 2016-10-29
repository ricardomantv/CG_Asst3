#include <float.h>
#include <assert.h>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"

namespace CMU462 {

  VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {

    // TODO: (meshEdit)
    // This method should split the given edge and return an iterator to the
    // newly inserted vertex. The halfedge of this vertex should point along
    // the edge that was split, rather than the new edges.

    // Collect halfedges, edges, vertices, faces
    std::vector<HalfedgeIter> h;
    h.push_back(e0->halfedge());
    std::vector<EdgeIter> e;
    e.push_back(e0);
    std::vector<VertexIter> v;
    v.push_back(h[0]->vertex());

    // Collect halfedges, edges, vertices of first face
    HalfedgeIter hiter = h[0]->next();
    do {
      h.push_back(hiter);
      e.push_back(hiter->edge());
      v.push_back(hiter->vertex());
      hiter = hiter->next();
    } while(hiter != h[0]);

    // Collect halfedges, edges, vertices of second face
    hiter = h[0]->twin();
    do {
      h.push_back(hiter);
      if(hiter != h[0]->twin()) {
        // Prevents e0 from being added to e again
        e.push_back(hiter->edge());
        if(hiter != h[0]->twin()->next()) {
          // Prevents v0 and v1 from being added to v again
          v.push_back(hiter->vertex());
        }
      }
      hiter = hiter->next();
    } while(hiter != h[0]->twin());

    // Collect outside halfedges
    for(int i = 1; i < 6; i++) {
      if(i != 3) {
        // Prevent h0 from being added as outside edge
        h.push_back(h[i]->twin());
      }
    }

    FaceIter f0 = h[0]->face();
    FaceIter f1 = h[0]->twin()->face();

    // Create new vertex, edges, halfedges, faces
    v.push_back(newVertex());
    e.push_back(newEdge());
    e.push_back(newEdge());
    e.push_back(newEdge());
    FaceIter f2 = newFace();
    FaceIter f3 = newFace();
    for(int i = 0; i < 6; i++) {
      h.push_back(newHalfedge());
    }

    // Set position of new vertex
    Vector3D pos = e[0]->centroid();
    v[4]->position = pos;

    // Reassign halfedges (outside halfedges only reassign twins)
    h[0]->setNeighbors(h[1], h[3], v[4], e[0], f0);
    h[1]->setNeighbors(h[2], h[6], v[1], e[1], f0);
    h[2]->setNeighbors(h[0], h[11], v[2], e[6], f0);
    h[3]->setNeighbors(h[4], h[0], v[1], e[0], f1);
    h[4]->setNeighbors(h[5], h[15], v[4], e[7], f1);
    h[5]->setNeighbors(h[3], h[9], v[3], e[4], f1);
    h[6]->twin() = h[1];
    h[7]->twin() = h[12];
    h[8]->twin() = h[14];
    h[9]->twin() = h[5];
    h[10]->setNeighbors(h[11], h[13], v[0], e[5], f2);
    h[11]->setNeighbors(h[12], h[2], v[4], e[6], f2);
    h[12]->setNeighbors(h[10], h[7], v[2], e[2], f2);
    h[13]->setNeighbors(h[14], h[10], v[4], e[5], f3);
    h[14]->setNeighbors(h[15], h[8], v[0], e[3], f3);
    h[15]->setNeighbors(h[13], h[4], v[3], e[7], f3);

    // Reassign edges
    e[0]->halfedge() = h[0];
    e[1]->halfedge() = h[1];
    e[2]->halfedge() = h[12];
    e[3]->halfedge() = h[14];
    e[4]->halfedge() = h[5];
    e[5]->halfedge() = h[10];
    e[6]->halfedge() = h[2];
    e[7]->halfedge() = h[15];
    // For loop subdivision
    /*
    e[0]->isNew = false;
    e[1]->isNew = false;
    e[2]->isNew = false;
    e[3]->isNew = false;
    e[4]->isNew = false;
    */
    e[5]->isNew = false;
    e[6]->isNew = true;
    e[7]->isNew = true;

    // Reassign vertices
    v[0]->halfedge() = h[14];
    v[1]->halfedge() = h[1];
    v[2]->halfedge() = h[12];
    v[3]->halfedge() = h[5];
    v[4]->halfedge() = h[0];
    // For loop subdivision
    /*
    v[0]->isNew = false;
    v[1]->isNew = false;
    v[2]->isNew = false;
    v[3]->isNew = false;
    */
    v[4]->isNew = true;

    // Reassign faces
    f0->halfedge() = h[0];
    f1->halfedge() = h[3];
    f2->halfedge() = h[10];
    f3->halfedge() = h[13];

    return v[4];
  }

  VertexIter HalfedgeMesh::collapseEdge(EdgeIter e) {

    // TODO: (meshEdit)
    // This method should collapse the given edge and return an iterator to
    // the new vertex created by the collapse.
    return VertexIter();
  }

  VertexIter HalfedgeMesh::collapseFace(FaceIter f) {

    // TODO: (meshEdit)
    // This method should collapse the given face and return an iterator to
    // the new vertex created by the collapse.
    return VertexIter();
  }

  FaceIter HalfedgeMesh::eraseVertex(VertexIter v) {

    // TODO: (meshEdit)
    // This method should replace the given vertex and all its neighboring
    // edges and faces with a single face, returning the new face.
    return FaceIter();
  }

  FaceIter HalfedgeMesh::eraseEdge( EdgeIter e ) {
    // This method should erase the given edge and return an iterator to the
    // merged face.

    if(e->isBoundary()) {
      // Does nothing if e is a boundary edge
      return e->halfedge()->face();
    }

    std::vector<HalfedgeIter> h;
    h.push_back(e->halfedge());
    // std::vector<VertexIter> v;

    FaceIter f0 = h[0]->face();
    FaceIter f1 = h[0]->twin()->face();

    // Collect halfedges
    HalfedgeIter hiter = h[0]->next();
    do {
      h.push_back(hiter);
      hiter = hiter->next();
    } while(hiter != h[0]);

    hiter = h[0]->twin();
    do {
      h.push_back(hiter);
      hiter = hiter->next();
    } while(hiter != h[0]->twin());

    // Reassign necessary halfedge next pointers
    Size f0_size = f0->degree();
    Size f1_size = f1->degree();
    h[f0_size - 1]->next() = h[0]->twin()->next();
    h[f0_size + f1_size - 1]->next() = h[1];

    // Reassign f0 as face for all connected halfedges
    FaceIter f_new = newFace();
    hiter = h[1];
    do {
      hiter->face() = f_new;
      hiter = hiter->next();
    } while(hiter != h[1]);

    f_new->halfedge() = h[1];

    // Reassign halfedges of vertices of deleted edge
    h[0]->vertex()->halfedge() = h[f0_size + 1];
    h[0]->twin()->vertex()->halfedge() = h[1];

    // Delete unneeded halfedges, edges, faces
    deleteHalfedge(h[0]);
    deleteHalfedge(h[0]->twin());
    deleteEdge(e);
    deleteFace(f0);
    deleteFace(f1);

    return f_new;
  }

  EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
    // This method should flip the given edge and return an iterator to the
    // flipped edge.

    if(e0->isBoundary()) {
      // Don't flip boundary edges
      return e0;
    }

    std::vector<HalfedgeIter> h;
    h.push_back(e0->halfedge());
    std::vector<EdgeIter> e;
    e.push_back(e0);
    std::vector<VertexIter> v;
    v.push_back(h[0]->vertex());

    // Collect faces
    FaceIter f0 = h[0]->face();
    FaceIter f1 = h[0]->twin()->face();

    // Collect halfedges, edges, and vertices of f0
    HalfedgeIter hiter = h[0]->next();
    do {
      h.push_back(hiter);
      e.push_back(hiter->edge());
      v.push_back(hiter->vertex());
      hiter = hiter->next();
    } while(hiter != h[0]);

    // Collect halfedges, edges, and vertices of f1
    hiter = h[0]->twin();
    do {
      h.push_back(hiter);
      if(hiter != h[0]->twin()) {
        // Prevents e0 from being added again to edges
        e.push_back(hiter->edge());
        if(hiter != h[0]->twin()->next()) {
          // Prevents v0 and v1 from being added twice to vertices
          v.push_back(hiter->vertex());
        }
      }
      hiter = hiter->next();
    } while (hiter != h[0]->twin());

    // Collect outside halfedges
    Size f0_size = f0->degree();
    Size f1_size = f1->degree();

    for(int i = 1; i < f0_size + f1_size; i++) {
      if(i != f0_size) {
        // Prevent h0 from being added as outside halfedge
        h.push_back(h[i]->twin());
      }
    }

    // Reassign halfedges, edges, and vertices
    /*
    h[0]->setNeighbors(h[1], h[f0_size], v[f0_size], e[0], f0);
    e[0]->halfedge() = h[0];
    v[f0_size]->halfedge() = h[0];

    for(int i = 1; i < f0_size; i++) {
      HalfedgeIter twin = h[i]->twin();
      h[i]->setNeighbors(h[(i + 1) % f0_size], e[i + 1]->halfedge(), twin->vertex(), e[i + 1], f0);

      // Reassign vertex and edge to moved halfedge
      twin->vertex()->halfedge() = h[i];
      e[i + 1]->halfedge() = h[i];
    }

    h[f0_size]->setNeighbors(h[f0_size + 1], h[0], v[2], e[0], f1);
    v[2]->halfedge() = h[f0_size];

    for(int i = f0_size + 1; i < f0_size + f1_size; i++) {
      HalfedgeIter twin = h[i]->twin();
      int new_e = (i == e.size()) ? 1 : i;
      h[i]->setNeighbors(h[(i + 1) % f0_size + f0_size], e[new_e]->halfedge(), twin->vertex(), e[new_e], f1);

      // Reassign vertex and edge to moved halfedge
      twin->vertex()->halfedge() = h[i];
      e[new_e]->halfedge() = h[i];
    }
    */

    // "Reassign" halfedge next() and face() values (stay the same)
    for(int i = 0; i < h.size(); i++) {
      h[i]->next() = h[i]->next();
      h[i]->face() = h[i]->face();
    }

    // Reassign halfedge twin() values
    for(int i = 0; i < h.size(); i++) {
      int twin;
      if(i == 0 || i == f0_size) {
        twin = f0_size - i;
      }
      else if(i == f0_size + f1_size - 1) {
        twin = i + 1;
      }
      else if(i < f0_size + f1_size){
        // Generic inside halfedge reassign
        if(h[i]->face() == f0) {
          twin = i + f0_size + f1_size;
        } else {
          twin = i + f0_size + f1_size - 1;
        }
      }
      else if(i == f0_size + f1_size){
        twin = i - 1;
      }
      else if(f0_size + f1_size + 1 <= i && i < 2 * f0_size + f1_size){
        // Outside halfedge of f0
        twin = i - f0_size - f1_size;
      }
      else {
        // Outside halfedge of f1
        twin = i - f0_size - f1_size + 1;
      }

      h[i]->twin() = h[twin];
    }

    // Reassign halfedge vertex() values
    for(int i = 0; i < h.size(); i++) {
      VertexIter vert;
      if(i == 0) {
        vert = v[f0_size];
      }
      else if(1 <= i && i < f0_size) {
        vert = v[(i + 1) % f0_size];
      }
      else if(i == f0_size) {
        vert = v[2];
      }
      else if(f0_size + 1 <= i && i < f0_size + f1_size - 1) {
        vert = v[i - 1];
      }
      else if(i == f0_size + f1_size - 1) {
        vert = v[1];
      }
      else {
        // Outside halfedge vertices don't change
        vert = h[i]->vertex();
      }
      h[i]->vertex() = vert;
    }

    // Reassign halfedge edge() values
    for(int i = 0; i < h.size(); i++) {
      EdgeIter edge;
      if(i == 0 || i == f0_size) {
        edge = e[0];
      }
      else if(1 <= i && i < f0_size) {
        edge = e[i];
      }
      else if(f0_size + 1 <= i && i < f0_size + f1_size) {
        edge = e[i - 1];
      }
      else {
        edge = h[i]->edge();
      }
      h[i]->edge() = edge;
    }

    // Reassign edge halfedge() values
    for(int i = 0; i < e.size(); i++) {
      HalfedgeIter half;
      if(i < f0_size) {
        half = h[i];
      } else {
        half = h[i + 1];
      }
      e[i]->halfedge() = half;
    }

    // Reassign vertex halfedge() values
    for(int i = 0; i < v.size(); i++) {
      HalfedgeIter half;
      if(i == f0_size) {
        half = h[0];
      }
      else if(i == 1) {
        half = h[f0_size + f1_size - 1];
      }
      else if(f0_size <= i && i < f0_size + f1_size - 1) {
        half = h[i + 1];
      }
      else {
        // Case for v[0]->v[f0_size], skipping v[1] as separate case
        half = h[(i + f0_size - 1) % f0_size];
      }
      v[i]->halfedge() = half;
    }

    // Reassign face halfedge() values
    f0->halfedge() = h[0];
    f1->halfedge() = h[0]->twin();

    return e[0];
  }

  void HalfedgeMesh::subdivideQuad( bool useCatmullClark )
  {
    // Unlike the local mesh operations (like bevel or edge flip), we will perform
    // subdivision by splitting *all* faces into quads "simultaneously."  Rather
    // than operating directly on the halfedge data structure (which as you've seen
    // is quite difficult to maintain!) we are going to do something a bit nicer:
    //
    //    1. Create a raw list of vertex positions and faces (rather than a full-
    //       blown halfedge mesh).
    //
    //    2. Build a new halfedge mesh from these lists, replacing the old one.
    //
    // Sometimes rebuilding a data structure from scratch is simpler (and even more
    // efficient) than incrementally modifying the existing one.  These steps are
    // detailed below.

    // TODO Step I: Compute the vertex positions for the subdivided mesh.  Here we're
    // going to do something a little bit strange: since we will have one vertex in
    // the subdivided mesh for each vertex, edge, and face in the original mesh, we
    // can nicely store the new vertex *positions* as attributes on vertices, edges,
    // and faces of the original mesh.  These positions can then be conveniently
    // copied into the new, subdivided mesh.
    // [See subroutines for actual "TODO"s]
    if( useCatmullClark )
    {
      computeCatmullClarkPositions();
    }
    else
    {
      computeLinearSubdivisionPositions();
    }

    // TODO Step II: Assign a unique index (starting at 0) to each vertex, edge, and
    // face in the original mesh.  These indices will be the indices of the vertices
    // in the new (subdivided mesh).  They do not have to be assigned in any particular
    // order, so long as no index is shared by more than one mesh element, and the
    // total number of indices is equal to V+E+F, i.e., the total number of vertices
    // plus edges plus faces in the original mesh.  Basically we just need a one-to-one
    // mapping between original mesh elements and subdivided mesh vertices.
    // [See subroutine for actual "TODO"s]
    assignSubdivisionIndices();

    // TODO Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
    // the element indices defined above.  In other words, each new quad should be of
    // the form (i,j,k,l), where i,j,k and l are four of the indices stored on our
    // original mesh elements.  Note that it is essential to get the orientation right
    // here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces should
    // circulate in the same direction as old faces (think about the right-hand rule).
    // [See subroutines for actual "TODO"s]
    vector< vector<Index> > subDFaces;
    vector< Vector3D > subDVertices;
    buildSubdivisionFaceList( subDFaces );
    buildSubdivisionVertexList( subDVertices );

    // TODO Step IV: Pass the list of vertices and quads to a routine that clears the
    // internal data for this halfedge mesh, and builds new halfedge data from scratch,
    // using the two lists.
    rebuild( subDFaces, subDVertices );
  }

  /**
   * Compute new vertex positions for a mesh that splits each polygon
   * into quads (by inserting a vertex at the face midpoint and each
   * of the edge midpoints).  The new vertex positions will be stored
   * in the members Vertex::newPosition, Edge::newPosition, and
   * Face::newPosition.  The values of the positions are based on
   * simple linear interpolation, e.g., the edge midpoints and face
   * centroids.
   */
  void HalfedgeMesh::computeLinearSubdivisionPositions()
  {
    // TODO For each vertex, assign Vertex::newPosition to
    // its original position, Vertex::position.
    for(VertexIter viter = verticesBegin(); viter != verticesEnd(); viter++) {
      viter->newPosition = viter->position;
    }

    // TODO For each edge, assign the midpoint of the two original
    // positions to Edge::newPosition.
    for(EdgeIter eiter = edgesBegin(); eiter != edgesEnd(); eiter++) {
      eiter->newPosition = eiter->centroid();
    }

    // TODO For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::newPosition.  Note
    // that in general, NOT all faces will be triangles!
    for(FaceIter fiter = facesBegin(); fiter != facesEnd(); fiter++) {
      fiter->newPosition = fiter->centroid();
    }
  }

  /**
   * Compute new vertex positions for a mesh that splits each polygon
   * into quads (by inserting a vertex at the face midpoint and each
   * of the edge midpoints).  The new vertex positions will be stored
   * in the members Vertex::newPosition, Edge::newPosition, and
   * Face::newPosition.  The values of the positions are based on
   * the Catmull-Clark rules for subdivision.
   */
  void HalfedgeMesh::computeCatmullClarkPositions()
  {
    // TODO The implementation for this routine should be
    // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules.  (These rules are outlined in the Developer Manual.)

    // TODO face
    for(FaceIter fiter = facesBegin(); fiter != facesEnd(); fiter++) {
      fiter->newPosition = fiter->centroid();
    }

    // TODO edges
    for(EdgeIter eiter = edgesBegin(); eiter != edgesEnd(); eiter++) {
      Vector3D edge_avg = eiter->centroid();
      Vector3D f1_avg = eiter->halfedge()->face()->centroid();
      Vector3D f2_avg = eiter->halfedge()->twin()->face()->centroid();
      float x_avg = (edge_avg.x + f1_avg.x + f2_avg.x) / 3.0f;
      float y_avg = (edge_avg.y + f1_avg.y + f2_avg.y) / 3.0f;
      float z_avg = (edge_avg.z + f1_avg.z + f2_avg.z) / 3.0f;
      eiter->newPosition = Vector3D(x_avg, y_avg, z_avg);
    }

    // TODO vertices
    for(VertexIter viter = verticesBegin(); viter != verticesEnd(); viter++) {
      HalfedgeIter h = viter->halfedge();
      std::vector<Vector3D> edge_pos;
      std::vector<Vector3D> face_pos;
      // Collect average points of surrounding edges and faces
      do {
        edge_pos.push_back(h->edge()->centroid());
        face_pos.push_back(h->face()->centroid());
        h = h->twin()->next();
      } while(h != viter->halfedge());

      // Calculate Q (avg face), R (avg edge), S (vertex)
      Vector3D Q = Vector3D(0.0f, 0.0f, 0.0f);
      size_t f_size = face_pos.size();
      for(int i = 0; i < f_size; i++) {
        Q.x += face_pos[i].x;
        Q.y += face_pos[i].y;
        Q.z += face_pos[i].z;
      }
      Q /= f_size;

      Vector3D R = Vector3D(0.0f, 0.0f, 0.0f);
      size_t e_size = edge_pos.size();
      for(int i = 0; i < e_size; i++) {
        R.x += edge_pos[i].x;
        R.y += edge_pos[i].y;
        R.z += edge_pos[i].z;
      }
      R /= e_size;

      Vector3D S = viter->centroid();
      Size n = viter->degree();

      Vector3D newPos = (Q + 2 * R + (n - 3) * S) / n;
      viter->newPosition = newPos;
    }
  }

  /**
   * Assign a unique integer index to each vertex, edge, and face in
   * the mesh, starting at 0 and incrementing by 1 for each element.
   * These indices will be used as the vertex indices for a mesh
   * subdivided using Catmull-Clark (or linear) subdivision.
   */
  void HalfedgeMesh::assignSubdivisionIndices()
  {
    // TODO Start a counter at zero; if you like, you can use the
    // "Index" type (defined in halfedgeMesh.h)
    Index counter = 0;

    // TODO Iterate over vertices, assigning values to Vertex::index
    for(VertexIter viter = verticesBegin(); viter != verticesEnd(); viter++) {
      viter->index = counter;
      counter++;
    }

    // TODO Iterate over edges, assigning values to Edge::index
    for(EdgeIter eiter = edgesBegin(); eiter != edgesEnd(); eiter++) {
      eiter->index = counter;
      counter++;
    }

    // TODO Iterate over faces, assigning values to Face::index
    for(FaceIter fiter = facesBegin(); fiter != facesEnd(); fiter++) {
      fiter->index = counter;
      counter++;
    }
  }

  /**
   * Build a flat list containing all the vertex positions for a
   * Catmull-Clark (or linear) subdivison of this mesh.  The order of
   * vertex positions in this list must be identical to the order
   * of indices assigned to Vertex::newPosition, Edge::newPosition,
   * and Face::newPosition.
   */
  void HalfedgeMesh::buildSubdivisionVertexList( vector<Vector3D>& subDVertices )
  {
    // TODO Resize the vertex list so that it can hold all the vertices.
    FaceCIter f = facesEnd(); f--;
    size_t size = f->index + 1;
    subDVertices.resize(size);

    // TODO Iterate over vertices, assigning Vertex::newPosition to the appropriate
    // location in the new vertex list.
    for(VertexCIter viter = verticesBegin(); viter != verticesEnd(); viter++) {
      subDVertices[viter->index] = viter->newPosition;
    }

    // TODO Iterate over edges, assigning Edge::newPosition to the appropriate
    // location in the new vertex list.
    for(EdgeCIter eiter = edgesBegin(); eiter != edgesEnd(); eiter++) {
      subDVertices[eiter->index] = eiter->newPosition;
    }

    // TODO Iterate over faces, assigning Face::newPosition to the appropriate
    // location in the new vertex list.
    for(FaceCIter fiter = facesBegin(); fiter != facesEnd(); fiter++) {
      subDVertices[fiter->index] = fiter->newPosition;
    }

  }

  /**
   * Build a flat list containing all the quads in a Catmull-Clark
   * (or linear) subdivision of this mesh.  Each quad is specified
   * by a vector of four indices (i,j,k,l), which come from the
   * members Vertex::index, Edge::index, and Face::index.  Note that
   * the ordering of these indices is important because it determines
   * the orientation of the new quads; it is also important to avoid
   * "bowties."  For instance, (l,k,j,i) has the opposite orientation
   * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
   * will look like a bowtie.
   */
  void HalfedgeMesh::buildSubdivisionFaceList( vector< vector<Index> >& subDFaces )
  {
    // TODO This routine is perhaps the most tricky step in the construction of
    // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
    // vertex positions).  Basically what you want to do is iterate over faces,
    // then for each for each face, append N quads to the list (where N is the
    // degree of the face).  For this routine, it may be more convenient to simply
    // append quads to the end of the list (rather than allocating it ahead of
    // time), though YMMV.  You can of course iterate around a face by starting
    // with its first halfedge and following the "next" pointer until you get
    // back to the beginning.  The tricky part is making sure you grab the right
    // indices in the right order---remember that there are indices on vertices,
    // edges, AND faces of the original mesh.  All of these should get used.  Also
    // remember that you must have FOUR indices per face, since you are making a
    // QUAD mesh!

    // TODO iterate over faces
    // TODO loop around face
    // TODO build lists of four indices for each sub-quad
    // TODO append each list of four indices to face list

    for(FaceCIter fiter = facesBegin(); fiter != facesEnd(); fiter++) {

      HalfedgeCIter hiter = fiter->halfedge();
      Index cur_face = fiter->index;
      do {
        std::vector<Index> cur_quad(4);
        Index cur_edge = hiter->edge()->index;
        Index next_edge = hiter->next()->edge()->index;
        Index next_vert = hiter->next()->vertex()->index;
        cur_quad[0] = cur_edge;
        cur_quad[1] = next_vert;
        cur_quad[2] = next_edge;
        cur_quad[3] = cur_face;
        subDFaces.push_back(cur_quad);
        hiter = hiter->next();
      } while (hiter != fiter->halfedge());
    }

  }

  void HalfedgeMesh::_bevel_fc_reposition_with_dist( vector<Vector3D>& orig, // list of vertex positions of the original face (before bevel)
                                                     vector<HalfedgeIter>& hs, // list of halfedges pointing from the vertices of the new, beveled face to the old, original face
                                                     double shift, // user-requested amount to shift the face in the normal direction
                                                     double inset ) // user-requested amount by which to inset (i.e., grow/shrink) the beveled face
  {
    // TODO Compute new vertex positions for the vertices of the beveled face.
    //
    // These vertices can be accessed via hs[i]->vertex()->position for i = 1, ..., hs.size()-1.
    //
    // The basic strategy here is to loop over the list of outgoing halfedges,
    // and use the preceding and next vertex position from the original mesh
    // (in the orig array) to compute an offset vertex position.
    //
    // Note that there is a 1-to-1 correspondence between halfedges in hs and vertex positions
    // in orig.  So, you can write loops of the form
    //
    // for( int i = 0; i < hs.size(); hs++ )
    // {
    //    Vector3D pi = orig[i]; // get the original vertex position correponding to vertex i
    // }
    //
    for(int i = 0; i < hs.size(); i++) {
      VertexIter v = hs[i]->vertex();
      /*
      FaceIter f = hs[i]->twin()->next()->twin()->face();
      Vector3D scalePos = orig[i] * inset * 10;
      Vector3D fnorm = f->normal();
      Vector3D normShift = fnorm * shift * 100;
      if(shift != 0) {
        cout << "normShift = " << normShift << ", newPos = " << (orig[i] + normShift) << "\n";
      }
      */
      v->position = orig[i];
    }
  }

  void HalfedgeMesh::_bevel_vtx_reposition_with_dist( Vector3D orig, // original vertex position, before the bevel
                                                      vector<HalfedgeIter>& hs, // list of halfedges pointing from the vertices of the new, beveled face to the neighbors of the original vertex
                                                      double inset ) // user-requested amount by which to inset (i.e., grow/shrink) the beveled face
  {
    // TODO Compute new vertex positions for the vertices of the beveled vertex.
    //
    // These vertices can be accessed via hs[i]->vertex()->position for i = 1, ..., hs.size()-1.
    //
    // The basic strategy here is to loop over the list of outgoing halfedges,
    // and use the preceding and next vertex position from the original mesh
    // (in the orig array) to compute an offset vertex position.
    //
    // Note that there is a 1-to-1 correspondence between halfedges in hs and vertex positions
    // in orig.  So, you can write loops of the form
    //
    // for( int i = 0; i < hs.size(); hs++ )
    // {
    //    Vector3D pi = orig[i]; // get the original vertex position correponding to vertex i
    // }
    //
  }

  void HalfedgeMesh::_bevel_edge_reposition_with_dist( vector<Vector3D>& origs,  // list of vertex positions of the neighbors of the two endpoints of the edge, before the bevel
                                                       vector<HalfedgeIter>& hs,  // list of halfedges pointing from the vertices of the new, beveled face to the neighbors of the endpoints of the old, original edge
                                                       double inset) // user-requested amount by which to inset (i.e., grow/shrink) the beveled face
  {
    // TODO Compute new vertex positions for the vertices of the beveled edge.
    //
    // These vertices can be accessed via hs[i]->vertex()->position for i = 1, ..., hs.size()-1.
    //
    // The basic strategy here is to loop over the list of outgoing halfedges,
    // and use the preceding and next vertex position from the original mesh
    // (in the orig array) to compute an offset vertex position.
    //
    // Note that there is a 1-to-1 correspondence between halfedges in hs and vertex positions
    // in orig.  So, you can write loops of the form
    //
    // for( int i = 0; i < hs.size(); hs++ )
    // {
    //    Vector3D pi = orig[i]; // get the original vertex position correponding to vertex i
    // }
    //
  }

  FaceIter HalfedgeMesh::bevelVertex(VertexIter v) {

    // TODO This method should replace the vertex v with a face, corresponding to a bevel operation.
    // It should return the new face.  NOTE: This method is responsible for updating the *connectivity*
    // of the mesh only---it does not need to update the vertex positions.  These positions will be
    // updated in HalfedgeMesh::_bevel_vtx_reposition_with_dist (which you also have to implement!)

    return facesBegin();
  }

  FaceIter HalfedgeMesh::bevelEdge(EdgeIter e) {

    // TODO This method should replace the edge e with a face, corresponding to a bevel operation.
    // It should return the new face.  NOTE: This method is responsible for updating the *connectivity*
    // of the mesh only---it does not need to update the vertex positions.  These positions will be
    // updated in HalfedgeMesh::_bevel_vtx_reposition_with_dist (which you also have to implement!)

    return facesBegin();
  }

  FaceIter HalfedgeMesh::bevelFace(FaceIter f) {

    // TODO This method should replace the face f with an additional, inset face (and ring of faces around it),
    // corresponding to a bevel operation. It should return the new face.  NOTE: This method is responsible for
    // updating the *connectivity* of the mesh only---it does not need to update the vertex positions.  These
    // positions will be updated in HalfedgeMesh::_bevel_vtx_reposition_with_dist (which you also have to
    // implement!)

    // Collect halfedges, edges, vertices
    std::vector<HalfedgeIter> h;
    h.push_back(f->halfedge());
    std::vector<EdgeIter> e;
    e.push_back(h[0]->edge());
    std::vector<VertexIter> v;
    v.push_back(h[0]->vertex());
    std::vector<FaceIter> fs;
    fs.push_back(f);

    Size f0_size = f->degree();

    HalfedgeIter hiter = h[0]->next();
    do {
      h.push_back(hiter);
      e.push_back(hiter->edge());
      v.push_back(hiter->vertex());
      hiter = hiter->next();
    } while (hiter != h[0]);

    // Create new faces, edges, and vertices
    for(int i = 0; i < f0_size; i++) {
      fs.push_back(newFace()); // new face
      v.push_back(newVertex()); // vertices of new face
      e.push_back(newEdge()); // connection edges from old face to new face
      e.push_back(newEdge()); // edges of new face
    }
    fs.push_back(newFace()); // extra face to replace old face

    // Create new halfedges
    for(int i = 0; i < f0_size + 1; i++) {
      // 3 halfedges for each new surrounding face
      h.push_back(newHalfedge());
      h.push_back(newHalfedge());
      h.push_back(newHalfedge());
      if(i == 0) {
        // Add one more halfedge for new inside face
        h.push_back(newHalfedge());
      }
    }

    // I'll need these a lot and I'm too lazy to type them repeatedly
    int f0s2 = f0_size * 2;
    int f0s3 = f0_size * 3;
    int f0s4 = f0_size * 4;
    int f0s5 = f0_size * 5;

    // Reassign halfedge next() values
    for(int i = 0; i < f0s4; i++) {
      h[i]->next() = h[(i + f0_size) % f0s4];
    }

    for(int i = f0s4; i < f0s5; i++) {
      h[i]->next() = h[(i + 1) % f0_size + f0s4];
    }

    // Reassign halfedge twin() values
    for(int i = 0; i < f0s5; i++) {
      if(i < f0_size) {
        h[i]->twin() = h[i]->twin();
      }
      else if(f0_size <= i && i < f0s2) {
        int twin = (i == f0s2 - 1) ? i + f0_size + 1 : i + f0s2 + 1;
        h[i]->twin() = h[twin];// (i == f0s2 - 1) ? h[i + f0_size + 1] : h[i + f0s2 + 1];
      }
      else if(f0s2 <= i && i < f0s3) {
        int twin = i + f0s2;
        h[i]->twin() = h[twin];// h[i + f0s2];
      }
      else if(f0s3 <= i && i < f0s4) {
        int twin = (i == f0s3) ? i - f0_size - 1 : i - f0s2 - 1;
        h[i]->twin() = h[twin];// (i == f0s3) ? h[i - f0_size - 1] : h[i - f0s2 - 1];
      }
      else {
        int twin = i - f0s2;
        h[i]->twin() = h[twin];// h[i - f0s2];
      }
    }

    // Reassign halfedge vertex() values
    for(int i = 0; i < f0s5; i++) {
      if(i < f0_size) {
        h[i]->vertex() = h[i]->vertex();
      }
      else if(f0_size <= i && i < f0s2) {
        int vert = (i + 1) % f0_size;
        h[i]->vertex() = v[vert];// v[(i + 1) % f0_size];
      }
      else if(f0s2 <= i && i < f0s3) {
        int vert = (i == f0s3 - 1) ? f0_size : i - f0_size + 1;
        h[i]->vertex() = v[vert];// v[(i - f0_size + 1)];
      }
      else if(f0s3 <= i && i < f0s4) {
        int vert = i - f0s2;
        h[i]->vertex() = v[vert];// v[(i - f0s2)];
      }
      else {
        int vert = i - f0s3;
        h[i]->vertex() = v[vert]; // v[(i - f0s3)];
      }
    }

    // Reassign halfedge edge() values
    for(int i = 0; i < f0s5; i++) {
      if(i < f0s3) {
        h[i]->edge() = e[i];
      } else{
        h[i]->edge() = h[i]->twin()->edge();
      }
    }

    // Reassign halfedge face() values
    for(int i = 0; i < f0s5; i++) {
      if(i < f0s4) {
        int face = (i % f0_size) + 1;
        h[i]->face() = fs[face];// fs[(i % f0_size) + 1];
      } else {
        int face = fs.size() - 1;
        h[i]->face() = fs[fs.size() - 1];
      }
    }

    // Reassign edge halfedge() values
    for(int i = 0; i < e.size(); i++) {
      e[i]->halfedge() = h[i];
    }

    // Reassign vertex halfedge() values
    for(int i = 0; i < v.size(); i++) {
      if(i < f0_size) {
        v[i]->halfedge() = h[i];
      } else {
        v[i]->halfedge() = h[i + f0s2];
      }
    }

    // Reassign face halfedge() values
    for(int i = 0; i < fs.size(); i++) {
      if(i != fs.size() - 1) {
        fs[i]->halfedge() = h[i];
      } else{
        fs[i]->halfedge() = h[f0s4];
      }
    }

    // Delete old face
    deleteFace(f);

    return fs[fs.size() - 1];
  }

  void HalfedgeMesh::splitPolygons(vector<FaceIter>& fcs) {
    for (auto f : fcs) splitPolygon(f);
  }

  void HalfedgeMesh::splitPolygon(FaceIter f) {
    Size f0_size = f->degree();
    if(f0_size <= 3) {
      // Ignore faces that are already triangles or invalid somehow
      return;
    }

    std::vector<HalfedgeIter> h;
    h.push_back(f->halfedge());
    std::vector<VertexIter> v;
    v.push_back(h[0]->vertex());

    // Collect halfedges and vertices
    HalfedgeIter hiter = h[0]->next();
    do {
      h.push_back(hiter);
      v.push_back(hiter->vertex());
      hiter = hiter->next();
    } while(hiter != h[0]);

    // Create new halfedges, edge, and face
    h.push_back(newHalfedge());
    h.push_back(newHalfedge());
    EdgeIter new_e = newEdge();
    FaceIter f1 = newFace();

    // Reassign pointers for new components & some existing ones
    h[f0_size]->setNeighbors(h[0], h[f0_size + 1], v[2], new_e, f);
    h[f0_size + 1]->setNeighbors(h[2], h[f0_size], v[0], new_e, f1);
    h[1]->next() = h[f0_size];
    h[f0_size - 1]->next() = h[f0_size + 1];
    v[2]->halfedge() = h[f0_size];
    v[0]->halfedge() = h[f0_size + 1];
    new_e->halfedge() = h[f0_size];
    f->halfedge() = h[f0_size];
    f1->halfedge() = h[f0_size + 1];

    hiter = h[0];
    do {
      hiter->face() = f;
      hiter = hiter->next();
    } while(hiter != h[0]);

    hiter = h[2];
    do {
      hiter->face() = f1;
      hiter = hiter->next();
    } while(hiter != h[2]);

    splitPolygon(f1);
  }

  EdgeRecord::EdgeRecord(EdgeIter& _edge) : edge(_edge) {

    // TODO: (meshEdit)
    // Compute the combined quadric from the edge endpoints.
    // -> Build the 3x3 linear system whose solution minimizes the quadric error
    //    associated with these two endpoints.
    // -> Use this system to solve for the optimal position, and store it in
    //    EdgeRecord::optimalPoint.
    // -> Also store the cost associated with collapsing this edg in
    //    EdgeRecord::Cost.

  }

  void MeshResampler::upsample(HalfedgeMesh& mesh)
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
  {

    // TODO: (meshEdit)
    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::newPosition.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::newPosition.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::isNew. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::position.

    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using the Loop subdivision rule.
    for(VertexIter viter = mesh.verticesBegin(); viter != mesh.verticesEnd(); viter++) {
      Size n = viter->degree();
      float u = (n == 3) ? 3.0f/16.0f : 3.0f/8.0f;
      Vector3D vert_sum = Vector3D(0.0f, 0.0f, 0.0f);
      HalfedgeIter hiter = viter->halfedge();
      do {
        VertexIter neigh_v = hiter->next()->vertex();
        vert_sum += neigh_v->position;
        hiter = hiter->twin()->next();
      } while(hiter != viter->halfedge());

      viter->newPosition = (1 - n * u) * (viter->position) + u * vert_sum;
      viter->isNew = false;
    }

    // Next, compute the updated vertex positions associated with edges.
    for(EdgeIter eiter = mesh.edgesBegin(); eiter != mesh.edgesEnd(); eiter++) {
      HalfedgeIter hiter = eiter->halfedge();
      Vector3D A = hiter->vertex()->position;
      Vector3D B = hiter->twin()->vertex()->position;
      Vector3D C = hiter->next()->next()->vertex()->position;
      Vector3D D = hiter->twin()->next()->next()->vertex()->position;

      eiter->newPosition = (3.0f/8.0f) * (A + B) + (1.0f/8.0f) * (C + D);
      eiter->isNew = false;
    }

    // Next, we're going to split every edge in the mesh, in any order.  For future
    // reference, we're also going to store some information about which subdivided
    // edges come from splitting an edge in the original mesh, and which edges are new.
    // In this loop, we only want to iterate over edges of the original mesh---otherwise,
    // we'll end up splitting edges that we just split (and the loop will never end!)

    Size num_edges = mesh.nEdges();
    EdgeIter eiter = mesh.edgesBegin();
    int count = 0;
    while(count != num_edges) {
      EdgeIter next = eiter;
      next++;
      if(!(eiter->isNew)) {
        mesh.splitEdge(eiter);
      }
      eiter = next;
      count++;
    }

    // Finally, flip any new edge that connects an old and new vertex.
    for(EdgeIter eiter = mesh.edgesBegin(); eiter != mesh.edgesEnd(); eiter++) {
      VertexIter v0 = eiter->halfedge()->vertex();
      VertexIter v1 = eiter->halfedge()->twin()->vertex();
      if((v0->isNew != v1->isNew) && eiter->isNew) {
        // This is broken and I'm not really sure why...
        mesh.flipEdge(eiter);
      }
    }

    // Copy the updated vertex positions to the subdivided mesh.
    for(VertexIter viter = mesh.verticesBegin(); viter != mesh.verticesEnd(); viter++) {
      if(!viter->isNew) {
        viter->position = viter->newPosition;
      } else {
        viter->position = viter->halfedge()->edge()->newPosition;
      }
    }
  }

  void MeshResampler::downsample(HalfedgeMesh& mesh)
  {

    // TODO: (meshEdit)
    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in Face::quadric
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in Vertex::quadric
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an EdgeRecord for each edge and sticking it in the
    //    queue.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

  }

  void MeshResampler::resample(HalfedgeMesh& mesh) {

    // TODO: (meshEdit)
    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

  }

} // namespace CMU462
