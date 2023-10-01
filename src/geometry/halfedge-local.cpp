
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/


/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius) {
	//faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3) return std::nullopt;


	std::vector< VertexRef > face_vertices;
	//In order to make the first edge point in the +x direction, first vertex should
	// be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s) {
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	//assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false); //the face to return
	FaceRef boundary = emplace_face(true); //the boundary loop around the face

	std::vector< HalfedgeRef > face_halfedges; //will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s) {
		//will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s+1)%sides];

		//h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		//t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		//e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); //false: non-sharp

		//set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f,-1.0f);

		//thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0) face->halfedge = h;
		if (s + 1 == sides) boundary->halfedge = t;

		//halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s) {
		face_halfedges[s]->next = face_halfedges[(s+1)%sides];
		face_halfedges[(s+1)%sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}


/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = emplace_face();
    HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); //unchanged

	//n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;
	
	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e); // unchanged
	//h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	//t->face unchanged


	// Phase 4: Delete unused elements
    erase_face(f_not_used);
    erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}


/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e) {
	// A2L2 (REQUIRED): split_edge

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	HalfedgeRef hNext = h->next;
	HalfedgeRef tNext = t->next;
	HalfedgeRef hNextNext = h->next->next;
	HalfedgeRef tNextNext = t->next->next;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;
	VertexRef v1Next = t->next->next->vertex;
	VertexRef v2Next = h->next->next->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	//new elements on edge e
	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal
	
	if(e->on_boundary()){
		//new elements on the new edge 1
		EdgeRef eNew = emplace_edge();// the edge split face1
		eNew->sharp = e->sharp;

		HalfedgeRef hNew = emplace_halfedge();//halfedges on the new edge 1
		HalfedgeRef tNew = emplace_halfedge();

		FaceRef fNew = emplace_face();

		// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

		vm->halfedge = h2;
		e2->halfedge = h2;
		assert(e->halfedge == h); //unchanged

		fNew->halfedge = (h->face->boundary)?hNew:tNew;

		if(h->face->boundary){
			v1Next->halfedge = hNew;
			eNew->halfedge = hNew;//end point hasn't been assigned
			t->next->next = hNew;
			tNext->face = fNew;
			t->face->halfedge = t;
		}
		else{
			v2Next->halfedge = tNew;
			eNew->halfedge = tNew;//end point hasn't been assigned
			h->next->next = tNew;
			hNext->face = fNew;
			h->face->halfedge = h;
		}	

		//h2, t2 start assigning
		h2->twin = t;
		h2->next = h->next;
		h2->vertex = vm;
		h2->edge = e2;
		h2->face = (h->face->boundary)?h->face:fNew;

		t2->twin = h;
		t2->next = t->next;
		t2->vertex = vm;
		t2->edge = e;
		t2->face = (h->face->boundary)?fNew:t->face;
		
		h->twin = t2;
		h->next = (h->face->boundary)?h2:hNew;
		assert(h->vertex == v1); // unchanged
		assert(h->edge == e); // unchanged
		//h->face unchanged

		t->twin = h2;
		t->next = (h->face->boundary)?tNew:t2;
		assert(t->vertex == v2); // unchanged
		t->edge = e2;
		//t->face unchanged

		//h_e1, t_e1, fNew1 start assigning
		hNew->twin = tNew;
		hNew->next = (h->face->boundary)?t2:hNextNext;
		hNew->vertex = (h->face->boundary)?v1Next:vm;
		hNew->edge = eNew;
		hNew->face = (h->face->boundary)?fNew:h->face;

		tNew->twin = hNew;
		tNew->next = (h->face->boundary)?tNextNext:h2;
		tNew->vertex = (h->face->boundary)?vm:v2Next;
		tNew->edge = eNew;
		tNew->face = (h->face->boundary)?t->face:fNew;

		//validate
		assert(tNextNext->face==t->face);
		assert(hNextNext->face==h->face);
		assert(tNextNext->next->face==t->face);
		assert(hNextNext->next->face==h->face);
		
		// Phase 5: Return the correct iterator
		return vm;
	}
	else{
		//new elements on the new edge 1
		EdgeRef eSf1 = emplace_edge();// the edge split face1
		eSf1->sharp = e->sharp;

		HalfedgeRef h_e1 = emplace_halfedge();//halfedges on the new edge 1
		HalfedgeRef t_e1 = emplace_halfedge();

		//new elements on the new edge2
		EdgeRef eSf2 = emplace_edge();//the edge spit face2
		eSf2->sharp = e->sharp;

		HalfedgeRef h_e2 = emplace_halfedge();//halfedges on the new edge 2
		HalfedgeRef t_e2 = emplace_halfedge();

		//new 2 faces
		FaceRef fNew1 = emplace_face();
		FaceRef fNew2 = emplace_face(); 

		// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

		vm->halfedge = h2;
		e2->halfedge = h2;
		assert(e->halfedge == h); //unchanged

		fNew1->halfedge = h_e1;
		fNew2->halfedge = h_e2;

		v1Next->halfedge = h_e1;
		eSf1->halfedge = h_e1;//end point hasn't been assigned
		t->next->next = h_e1;

		v2Next->halfedge = h_e2;
		eSf2->halfedge = h_e2;//end point hasn't been assigned
		h->next->next = h_e2;
			
		//h2, t2 start assigning
		h2->twin = t;
		h2->next = h->next;
		h2->vertex = vm;
		h2->edge = e2;
		h2->face = fNew2;

		t2->twin = h;
		t2->next = t->next;
		t2->vertex = vm;
		t2->edge = e;
		t2->face = fNew1;
		
		h->twin = t2;
		h->next = t_e2;
		assert(h->vertex == v1); // unchanged
		assert(h->edge == e); // unchanged
		//h->face unchanged

		t->twin = h2;
		t->next = t_e1;
		assert(t->vertex == v2); // unchanged
		t->edge = e2;
		//t->face unchanged

		//h_e1, t_e1, fNew1 start assigning
		h_e1->twin = t_e1;
		h_e1->next = t2;
		h_e1->vertex = v1Next;
		h_e1->edge = eSf1;
		h_e1->face = fNew1;

		t_e1->twin = h_e1;
		t_e1->next = tNextNext;
		t_e1->vertex = vm;
		t_e1->edge = eSf1;
		t_e1->face = t->face;

		//h_e2, t_e2, fNew2 start assigning
		h_e2->twin = t_e2;
		h_e2->next = h2;
		h_e2->vertex = v2Next;
		h_e2->edge = eSf2;
		h_e2->face = fNew2;

		t_e2->twin = h_e2;
		t_e2->next = hNextNext;
		t_e2->vertex = vm;
		t_e2->edge = eSf2;
		t_e2->face = h->face;

		hNext->face = fNew2;
		tNext->face = fNew1;
		//validate
		assert(tNextNext->face==t->face);
		assert(hNextNext->face==h->face);
		assert(tNextNext->next->face==t->face);
		assert(hNextNext->next->face==h->face);
		assert(t2->face==fNew1);
		assert(h2->face==fNew2);
		assert(t_e1->face==t->face);
		assert(h_e1->face==fNew1);
		
		// Phase 5: Return the correct iterator
		return vm;
	}
}


/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
	// A2Lx4 (OPTIONAL): inset vertex
	
	(void)f;
    return std::nullopt;
}


/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v) {
	//A2Lx5 (OPTIONAL): Bevel Vertex
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
    return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e) {
	//A2Lx6 (OPTIONAL): Bevel Edge
	// Reminder: This function does not update the vertex positions.
	// remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
    return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f) {
	//A2L4: Extrude Face
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in extrude_helper (A2L4h)

	(void)f;
    return std::nullopt;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e) {
	//A2L1: Flip Edge
	if(e->on_boundary()){
		return std::nullopt;
	}

	//collect data
	HalfedgeRef h1 = e->halfedge;
	HalfedgeRef h2 = h1->twin;
	VertexRef v1 = h1->next->vertex;
	VertexRef v2 = h2->next->vertex;
	VertexRef v3 = h1->next->next->vertex;
	VertexRef v4 = h2->next->next->vertex;
	FaceRef f1 = h1->face;
	FaceRef f2 = h2->face;

	//find h1,h2's previews halfedge
	HalfedgeRef h1Prev = h1;
	while(h1Prev->next != h1){
		h1Prev = h1Prev->next;
		
	}

	HalfedgeRef h2Prev = h2;
	while(h2Prev->next != h2){
		h2Prev = h2Prev->next;
	}

	//disconnect
	HalfedgeRef h1Next = h1->next;
	HalfedgeRef h2Next = h2->next;
	HalfedgeRef h1NextNext = h1->next->next;
	HalfedgeRef h2NextNext = h2->next->next;
	h1Prev->next = h2Next;
	h2Prev->next = h1Next;
	v1->halfedge = h1Next;
	v2->halfedge = h2Next;

	//reconnect
	h1->vertex = v4;
	h2->vertex = v3;
	h1->next = h1NextNext;
	h2->next = h2NextNext;
	h1Next->next = h2;
	h2Next->next = h1;
	h1Next->face = f2;
	h2Next->face = f1;

    return e;
}


/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face) {
	//A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; //TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v) {
	// A2Lx1 (OPTIONAL): Dissolve Vertex

    return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e) {
	// A2Lx2 (OPTIONAL): dissolve_edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data
	
    return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e) {
	//A2L3: Collapse Edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)
	
    return std::nullopt;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f) {
	//A2Lx3 (OPTIONAL): Collapse Face

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

    return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2) {
	//A2Lx8: Weld Edges

	//Reminder: use interpolate_data() to merge bone_weights data on vertices!

    return std::nullopt;
}



/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance) {
	//A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper
	
	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
	
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 * 
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink) {
	//A2L4h: Extrude Positions Helper

	//General strategy:
	// use mesh navigation to get starting positions from the surrounding faces,
	// compute the centroid from these positions + use to shrink,
	// offset by move
	
}

