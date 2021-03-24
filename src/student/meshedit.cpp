
#include <queue>
#include <set>
#include <unordered_map>

#include <iostream>     // for debug

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

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
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {
    
    if(v -> on_boundary()){ return std::nullopt; }
    
    // collect neighbours info
    std::vector<HalfedgeRef> h_list;    // HALFEDGES
    std::vector<VertexRef> v_list;      // VERTICES
    std::vector<EdgeRef> e_list;        // EDGES
    std::vector<FaceRef> f_list;        // FACES
    
    // HALFEDGES
    size_t num_neighbourHalfedge = 0;   // record number of halfedge connected to the vertex
    std::vector<int> num_edgeInFace;    // count number of halfedges in each neighbour faces not connected to the vertex, CUMULATIVE
    num_edgeInFace.push_back(0);
    
    HalfedgeRef h = v -> halfedge();
    do{
        h_list.push_back(h);
        h_list.push_back(h -> twin());
        h = h -> twin() -> next();
        
        num_neighbourHalfedge++;
    }while(h != v -> halfedge());
    int count_edge = 0;
    do{
        HalfedgeRef h_temp = h;
        do{
            h_temp = h_temp -> next();
            h_list.push_back(h_temp);
            h_list.push_back(h_temp -> twin());
            
            count_edge++;
        }while(h_temp -> next() -> next() != h);
        
        num_edgeInFace.push_back(count_edge);   // cumulative
        h = h -> twin() -> next();
    }while(h != v -> halfedge());
    
    num_edgeInFace.push_back(num_edgeInFace[0]);    // handle loop
    

    // VERTICES
    v_list.push_back(v);
    do{
        HalfedgeRef h_temp = h;
        do{
            h_temp = h_temp -> next();
            v_list.push_back(h_temp -> vertex());
        }while(h_temp -> next() -> next() != h);
        h = h -> twin() -> next();
    }while(h != v -> halfedge());
    
    // EDGES
    do{
        e_list.push_back(h -> edge());
        h = h -> twin() -> next();
    }while(h != v -> halfedge());
    do{
        HalfedgeRef h_temp = h;
        do{
            h_temp = h_temp -> next();
            e_list.push_back(h_temp -> edge());
        }while(h_temp -> next() -> next() != h);
        h = h -> twin() -> next();
    }while(h != v -> halfedge());
    
    // FACES
    do{
        f_list.push_back(h -> face());
        h = h -> twin() -> next();
    }while(h != v -> halfedge());
    
    
    //std::cout << "h_list.size(): " << h_list.size() << std::endl;
    //std::cout << "num_neighbourHalfedge: " << num_neighbourHalfedge << std::endl;
    
    // reassign elements
    // HALFEDGES
    for(size_t count = 0; count < num_neighbourHalfedge; count++){
        unsigned int id_from = (num_neighbourHalfedge + num_edgeInFace[count + 1]) * 2;
        unsigned int id_to = (num_neighbourHalfedge + num_edgeInFace[count]) * 2;
        
        if(id_from >= h_list.size()){
            id_from = id_from % h_list.size() + num_neighbourHalfedge * 2; // prevent overflow, loop back
        }
        
        //std::cout << "id_from: " << id_from << " id_to: " << id_to << std::endl;
        
        h_list[id_from] -> next() = h_list[id_to];
        h_list[id_from] -> twin() = h_list[id_from + 1];
        h_list[id_from] -> face() = f_list[0];
        
        h_list[id_from + 1] -> twin() = h_list[id_from];
        
    }
    
    //std::cout << "v_list.size(): " << v_list.size() << std::endl;
        
    // VERTICES
    // should not changes, will remove v0
    for(size_t count = 1; count < v_list.size(); count++){
        unsigned int halfedgeId = (num_neighbourHalfedge + num_edgeInFace[count - 1]) * 2;
        unsigned int vertexId = count;
        
        v_list[vertexId] -> halfedge() = h_list[halfedgeId];
        
        //std::cout << "vertexId: " << vertexId << " halfedgeId: " << halfedgeId <<  std::endl;
        
    }
    
    // EDGES
    // should not changes, will remove those connected to v
    
    // FACES
    f_list[0] -> halfedge() = h_list[(num_neighbourHalfedge + 1) * 2];
    
    /*
    for(size_t count = 1; count < f_list.size(); count++){
        unsigned int halfedgeId = (num_neighbourHalfedge + num_edgeInFace[count - 1]) * 2;
        unsigned int faceId = count;
        
        f_list[vertexId] -> halfedge() = h_list[halfedgeId];
        
        std::cout << "faceId: " << faceId << " halfedgeId: " << halfedgeId <<  std::endl;
        
    }
     */
    
    
    // remove elements
    // HALFEDGES
    for(size_t count = 0; count < num_neighbourHalfedge; count++){
        erase(h_list[count * 2]);
        erase(h_list[count * 2 + 1]);
    }
    
    // VERTICES
    erase(v_list[0]);
    
    // EDGES
    for(size_t count = 0; count < num_neighbourHalfedge; count++){
        erase(e_list[count]);
    }
    
    // FACES
    for(size_t count = 1; count < f_list.size(); count++){
        erase(f_list[count]);
    }
    
    return f_list[0];
    
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    // check if edge on boundary
    if(e -> on_boundary()){ return std::nullopt; }
    // check if endpoints of the edge on boundary
    if(e -> halfedge() -> vertex() -> on_boundary()){ return std::nullopt; }
    if(e -> halfedge() -> twin() -> vertex() -> on_boundary()){ return std::nullopt; }
    
    // collect neighbours info
    std::vector<HalfedgeRef> h_listTop, h_listDown;    // HALFEDGES
    std::vector<VertexRef> v_listTop, v_listDown;      // VERTICES
    std::vector<EdgeRef> e_listTop, e_listDown;        // EDGES
    std::vector<FaceRef> f_listTop, f_listDown;        // FACES
   
 
    // halfedge of e0
    HalfedgeRef h0 = e -> halfedge(),
                h1 = e -> halfedge() -> twin();
    
    HalfedgeRef hLeft = h0 -> next(),
                hRight = h1 -> next();
    
    // vertex of e0
    VertexRef v0 = h0 -> vertex(),
              v1 = h1 -> vertex();
    
    // left - top
    do{
        // HALFEDGES
        h_listTop.push_back(hLeft);
        h_listTop.push_back(hLeft -> twin());
        // VERTICES
        v_listTop.push_back(hLeft -> twin() -> vertex());
        // EDGES
        e_listTop.push_back(hLeft -> edge());
        // FACES
        f_listTop.push_back(hLeft -> face());
        
        hLeft = hLeft -> twin() -> next();
    }while(hLeft != h1);
    hLeft = h0 -> next();    // reset
    // right - down
    do{
        // HALFEDGES
        h_listDown.push_back(hRight);
        h_listDown.push_back(hRight -> twin());
        // VERTICES
        v_listDown.push_back(hRight -> twin() -> vertex());
        // EDGES
        e_listDown.push_back(hRight -> edge());
        // FACES
        f_listDown.push_back(hRight -> face());
        
        hRight = hRight -> twin() -> next();
    }while(hRight != h0);
    hRight = h1 -> next();    // reset
    
    // check whether edge on triangle
    bool isLeftTriangle = false, isRightTriangle = true;
    isLeftTriangle = (h_listTop[0] -> next() == h_listDown[h_listDown.size() - 1]);
    isRightTriangle = (h_listDown[0] -> next() == h_listTop[h_listTop.size() - 1]);

    
    // create new element
    VertexRef v_new = new_vertex();
    v_new -> pos = ((v0 -> pos) + (v1 -> pos)) / 2;
    v_new -> halfedge() = hLeft;

    // reassign element
    size_t num_EdgeTop = h_listTop.size() / 2;
    size_t num_EdgeDown = h_listDown.size() / 2;
    // top
    h_listTop[num_EdgeTop * 2 - 1] -> next() = h_listDown[0];
    for(size_t count = 0; count < num_EdgeTop; count++){
        // HALFEDGES
        h_listTop[count * 2] -> vertex() = v_new;
        h_listTop[count * 2] -> twin() = h_listTop[count * 2 + 1];
        h_listTop[count * 2 + 1] -> twin() = h_listTop[count * 2];
        // VERTICES
        v_listTop[count] -> halfedge() = h_listTop[count * 2 + 1];
        // EDGES
        e_listTop[count] -> halfedge() = h_listTop[count * 2];
        // FACES
        f_listTop[count] -> halfedge() = h_listTop[count * 2];
    }
    // down
    h_listDown[num_EdgeDown * 2 - 1] -> next() = h_listTop[0];
    for(size_t count = 0; count < num_EdgeDown; count++){
        // HALFEDGES
        h_listDown[count * 2] -> vertex() = v_new;
        h_listDown[count * 2] -> twin() = h_listDown[count * 2 + 1];
        h_listDown[count * 2 + 1] -> twin() = h_listDown[count * 2];
        // VERTICES
        v_listDown[count] -> halfedge() = h_listDown[count * 2 + 1];
        // EDGES
        e_listDown[count] -> halfedge() = h_listDown[count * 2];
        // FACES
        f_listDown[count] -> halfedge() = h_listDown[count * 2];
    }
    hLeft -> vertex() = v_new;
    hRight -> vertex() = v_new;
    // handle triangle case
    if(isLeftTriangle){
        // HALFEDGE
        h_listTop[0] -> next() = h_listDown[h_listDown.size() - 2] -> next();
        h_listTop[0] -> face() = f_listDown[f_listDown.size() - 1];
        h_listTop[0] -> twin() = h_listTop[1];
        
        h_listDown[h_listDown.size() - 3] -> next() =  h_listTop[0];
        
        // VERTICES
        v_listDown[v_listDown.size() - 1] -> halfedge() = h_listTop[1];
        
        // EDGES
        
        // FACES
        f_listDown[f_listDown.size() - 1] -> halfedge() = h_listTop[0];
    }
    if(isRightTriangle){
        // HALFEDGE
        h_listDown[0] -> next() = h_listTop[h_listTop.size() - 2] -> next();
        h_listDown[0] -> face() = f_listTop[f_listTop.size() - 1];
        h_listDown[0] -> twin() = h_listDown[1];
        
        h_listTop[h_listTop.size() - 3] -> next() =  h_listDown[0];
        
        // VERTICES
        v_listTop[v_listTop.size() - 1] -> halfedge() = h_listDown[1];
        
        // EDGES
        
        // FACES
        f_listTop[f_listTop.size() - 1] -> halfedge() = h_listDown[0];
    }
    
    
    // erase element
    // VERTICES
    erase(v0);
    erase(v1);
    // HALFEDGE
    erase(h0);
    erase(h1);
    // EDGE
    erase(e);
    // handle triangle case
    if(isLeftTriangle){
        // HALFEDGE
        erase(h_listDown[h_listDown.size() - 2]);
        erase(h_listDown[h_listDown.size() - 1]);
        // EDGES
        erase(e_listDown[e_listDown.size() - 1]);
        // FACES
        erase(f_listTop[0]);
    }
    if(isRightTriangle){
        // HALFEDGE
        erase(h_listTop[h_listTop.size() - 2]);
        erase(h_listTop[h_listTop.size() - 1]);
        // EDGES
        erase(e_listTop[e_listTop.size() - 1]);
        // FACES
        erase(f_listDown[0]);
    }
    
    return v_new;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {
    
    // check if edge on boundary
    if(e -> on_boundary()){ return std::nullopt; }
    
    // collect neighbours info
    std::vector<HalfedgeRef> h_list;    // HALFEDGES
    std::vector<VertexRef> v_list;      // VERTICES
    std::vector<EdgeRef> e_list;        // EDGES
    std::vector<FaceRef> f_list;        // FACES
    
    // HALFEDGE
    HalfedgeRef h = e -> halfedge();
    size_t num_halfedgeLeft = 0, num_halfedgeRight = 0; // count number of halfedge at left and right side of h0; left + right == total
    // inner circle
    do{
        h_list.push_back(h);
        h = h -> next();
        num_halfedgeLeft++;
    }while(h != e -> halfedge());
    h = h -> twin();
    do{
        h_list.push_back(h);
        h = h -> next();
        num_halfedgeRight++;
    }while(h != e -> halfedge() -> twin());
    h = e -> halfedge();    // reset pointer
    // outer circle
    h = h -> next();        // skip h0
    do{
        h_list.push_back(h -> twin());
        h = h -> next();
    }while(h != e -> halfedge());
    h = h -> twin() -> next();  // skip twin of h0
    do{
        h_list.push_back(h -> twin());
        h = h -> next();
    }while(h != e -> halfedge() -> twin());
    h = e -> halfedge();    // reset pointer
    
    // VERTICES
    size_t num_verticesLeft = 0, num_verticesRight = 0; // count number of vertices at left and right side of h0; left + right == total
    do{
        v_list.push_back(h -> vertex());
        h = h -> next();
        num_verticesLeft++;
    }while(h != e -> halfedge());
    h = h -> twin() -> next() -> next();    // skip v0 and v1
    do{
        v_list.push_back(h -> vertex());
        h = h -> next();
        num_verticesRight++;
    }while(h != e -> halfedge() -> twin());
    h = e -> halfedge();    // reset pointer
    
    // EDGES
    size_t num_edgeLeft = 0, num_edgeRight = 0; // count number of vertices at left and right side of h0; left + right == total
    do{
        e_list.push_back(h -> edge());
        h = h -> next();
        num_edgeLeft++;
    }while(h != e -> halfedge());
    h = h -> twin() -> next();  // skip e0
    do{
        e_list.push_back(h -> edge());
        h = h -> next();
        num_edgeRight++;
    }while(h != e -> halfedge() -> twin());
    h = e -> halfedge();        // reset pointer
    
    // FACES
    // only two faces in the neighbour
    f_list.push_back(h -> face());
    f_list.push_back(h -> twin() -> face());
    
    // reassign elements
    // HALFEDGE
    h_list[0] -> next() = h_list[2];
    h_list[0] -> vertex() = v_list[num_verticesLeft];
    
    h_list[1] -> next() = h_list[num_halfedgeLeft];
    h_list[1] -> face() = f_list[1];
    
    h_list[num_halfedgeLeft - 1] -> next() = h_list[num_halfedgeLeft + 1];
    
    h_list[num_halfedgeLeft] -> next() = h_list[num_halfedgeLeft + 2];
    h_list[num_halfedgeLeft] -> vertex() = v_list[2];
    
    h_list[num_halfedgeLeft + 1] -> next() = h_list[0];
    h_list[num_halfedgeLeft + 1] -> face() = f_list[0];
    
    h_list[num_halfedgeLeft + num_halfedgeRight - 1] -> next() = h_list[1];
    
    // VERTICES
    v_list[0] -> halfedge() = h_list[num_halfedgeLeft + 1];
    v_list[1] -> halfedge() = h_list[1];
    
    // EDGES
    // not changed
    
    // FACES
    f_list[0] -> halfedge() = h_list[0];
    f_list[1] -> halfedge() = h_list[num_halfedgeLeft];
    
    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    // restricted to triangles
    
    // check if edge on boundary
    if(e -> on_boundary()){ return std::nullopt; }
    
    // collect neighbours info
    std::vector<HalfedgeRef> h_list;    // HALFEDGES
    std::vector<VertexRef> v_list;      // VERTICES
    std::vector<EdgeRef> e_list;        // EDGES
    std::vector<FaceRef> f_list;        // FACES
    
    // HALFEDGE
    HalfedgeRef h = e -> halfedge();
    size_t num_halfedgeLeft = 0, num_halfedgeRight = 0; // count number of halfedge at left and right side of h0; left + right == total
    // inner circle
    do{
        h_list.push_back(h);
        h = h -> next();
        num_halfedgeLeft++;
    }while(h != e -> halfedge());
    h = h -> twin();
    do{
        h_list.push_back(h);
        h = h -> next();
        num_halfedgeRight++;
    }while(h != e -> halfedge() -> twin());
    h = e -> halfedge();    // reset pointer
    // outer circle
    h = h -> next();        // skip h0
    do{
        h_list.push_back(h -> twin());
        h = h -> next();
    }while(h != e -> halfedge());
    h = h -> twin() -> next();  // skip twin of h0
    do{
        h_list.push_back(h -> twin());
        h = h -> next();
    }while(h != e -> halfedge() -> twin());
    h = e -> halfedge();    // reset pointer
    
    // VERTICES
    size_t num_verticesLeft = 0, num_verticesRight = 0; // count number of vertices at left and right side of h0; left + right == total
    do{
        v_list.push_back(h -> vertex());
        h = h -> next();
        num_verticesLeft++;
    }while(h != e -> halfedge());
    h = h -> twin() -> next() -> next();    // skip v0 and v1
    do{
        v_list.push_back(h -> vertex());
        h = h -> next();
        num_verticesRight++;
    }while(h != e -> halfedge() -> twin());
    h = e -> halfedge();    // reset pointer
    
    // EDGES
    size_t num_edgeLeft = 0, num_edgeRight = 0; // count number of vertices at left and right side of h0; left + right == total
    do{
        e_list.push_back(h -> edge());
        h = h -> next();
        num_edgeLeft++;
    }while(h != e -> halfedge());
    h = h -> twin() -> next();  // skip e0
    do{
        e_list.push_back(h -> edge());
        h = h -> next();
        num_edgeRight++;
    }while(h != e -> halfedge() -> twin());
    h = e -> halfedge();        // reset pointer
    
    // FACES
    // only two faces in the neighbour
    f_list.push_back(h -> face());
    f_list.push_back(h -> twin() -> face());
    
    
    // check whether in triangle, proceed only in triangle
    if(num_halfedgeLeft > 3 || num_halfedgeRight > 3){ return std::nullopt; }
    
    // create new vertex and related elements
    VertexRef v_new = new_vertex();
    
    HalfedgeRef h_new0 = new_halfedge(),
                h_new1 = new_halfedge(),
                h_new2 = new_halfedge(),
                h_new3 = new_halfedge(),
                h_new4 = new_halfedge(),
                h_new5 = new_halfedge();
    
    EdgeRef e_new0 = new_edge(),
            e_new1 = new_edge(),
            e_new2 = new_edge();
    
    FaceRef f_new0 = new_face(),
            f_new1 = new_face();
    
    
    // reassign elements
    // VERTICES
    v_new -> pos = ((v_list[0] -> pos) + (v_list[1] -> pos)) / 2;
    v_new -> halfedge() = h_new0;
    
    // HALFEDGES
    h_new0 -> next() = h_list[2];
    h_new0 -> twin() = h_new1;
    h_new0 -> vertex() = v_new;
    h_new0 -> edge() = e_new0;
    h_new0 -> face() = f_list[0];
    
    h_list[0] -> next() = h_new0;
    h_list[0] -> twin() = h_new5;
    ////
    h_new1 -> next() = h_new4;
    h_new1 -> twin() = h_new0;
    h_new1 -> vertex() = v_list[2];
    h_new1 -> edge() = e_new0;
    h_new1 -> face() = f_new0;
    
    h_new4 -> next() = h_list[1];
    h_new4 -> twin() = h_list[3];
    h_new4 -> vertex() = v_new;
    h_new4 -> edge() = e_new1;
    h_new4 -> face() = f_new0;
    
    h_list[1] -> next() = h_new1;
    h_list[1] -> face() = f_new0;
    ////
    h_new2 -> next() = h_list[5];
    h_new2 -> twin() = h_new3;
    h_new2 -> vertex() = v_new;
    h_new2 -> edge() = e_new2;
    h_new2 -> face() = f_list[1];
    
    h_list[3] -> next() = h_new2;
    h_list[3] -> twin() = h_new4;
    h_list[3] -> edge() = e_new1;
    ////
    h_new3 -> next() = h_new5;
    h_new3 -> twin() = h_new2;
    h_new3 -> vertex() = v_list[3];
    h_new3 -> edge() = e_new2;
    h_new3 -> face() = f_new1;
    
    h_new5 -> next() = h_list[4];
    h_new5 -> twin() = h_list[0];
    h_new5 -> vertex() = v_new;
    h_new5 -> edge() = e_list[0];
    h_new5 -> face() = f_new1;
    
    h_list[4] -> next() = h_new3;
    h_list[4] -> face() = f_new1;
    
    // EDGES
    e_new0 -> halfedge() = h_new0;
    e_new1 -> halfedge() = h_new4;
    e_new2 -> halfedge() = h_new2;
    e_list[0] -> halfedge() = h_list[0];
    
    // FACES
    f_new0 -> halfedge() = h_new1;
    f_new1 -> halfedge() = h_new5;
    f_list[0] -> halfedge() = h_list[0];
    f_list[1] -> halfedge() = h_list[3];
    
    return v_new;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."
    
    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."
    
    // collect neighbour information
    std::vector<HalfedgeRef> h_list;    // HALFEDGES
    std::vector<HalfedgeRef> h_newList;
    std::vector<VertexRef> v_list;      // VERTICES
    std::vector<VertexRef> v_newList;
    std::vector<EdgeRef> e_list;        // EDGES
    std::vector<EdgeRef> e_newList;
    std::vector<FaceRef> f_newList;        // FACES
    
    HalfedgeRef h = f -> halfedge();
    do{
        // HALFEDGES
        h_list.push_back(h);
        // VERTICES
        v_list.push_back(h -> vertex());
        // EDGES
        e_list.push_back(h -> edge());      // not used
        
        h = h -> next();
    }while(h != f -> halfedge());
    h = f -> halfedge();                // reset
    
    size_t num_edge = h_list.size();    // number of edges of the face to be beveled
    
    
    // create new elements
    // HALFEDGES
    for(size_t count = 0; count < 4 * num_edge; count++){
        h_newList.push_back(new_halfedge());
    }
    // EDGES
    for(size_t count = 0; count < 2 * num_edge; count++){
        e_newList.push_back(new_edge());
    }
    // VERTICES
    for(size_t count = 0; count < num_edge; count++){
        v_newList.push_back(new_vertex());
    }
    // FACES
    for(size_t count = 0; count < num_edge; count++){
        f_newList.push_back(new_face());
    }
    f_newList.push_back(new_face());
    
    
    // reassign elements
    // HALFEDGES
    for(size_t count = 0; count < num_edge; count++){
        h_list[count] -> next() = h_newList[count * 4];
        h_list[count] -> face() = f_newList[count];
        
        h_newList[count * 4] -> next() = h_newList[count * 4 + 2];
        h_newList[count * 4] -> twin() = h_newList[count * 4 + 1];
        h_newList[count * 4] -> vertex() = v_list[(count + 1) % num_edge];
        h_newList[count * 4] -> edge() = e_newList[count * 2];
        h_newList[count * 4] -> face() = f_newList[count];
        
        h_newList[count * 4 + 1] -> next() = h_list[(count + 1) % num_edge];
        h_newList[count * 4 + 1] -> twin() = h_newList[count * 4];
        h_newList[count * 4 + 1] -> vertex() = v_newList[count];
        h_newList[count * 4 + 1] -> edge() = e_newList[count * 2];
        h_newList[count * 4 + 1] -> face() = f_newList[(count + 1) % num_edge];
        
        h_newList[count * 4 + 2] -> next() = h_newList[((count + num_edge - 1) * 4 + 1) % (num_edge * 4)];
        h_newList[count * 4 + 2] -> twin() = h_newList[count * 4 + 3];
        h_newList[count * 4 + 2] -> vertex() = v_newList[count];
        h_newList[count * 4 + 2] -> edge() = e_newList[count * 2 + 1];
        h_newList[count * 4 + 2] -> face() = f_newList[count];
        
        h_newList[count * 4 + 3] -> next() = h_newList[((count + 1) * 4 + 3) % (num_edge * 4)];
        h_newList[count * 4 + 3] -> twin() = h_newList[count * 4 + 2];
        h_newList[count * 4 + 3] -> vertex() = v_newList[(count + num_edge - 1) % num_edge];
        h_newList[count * 4 + 3] -> edge() = e_newList[count * 2 + 1];
        h_newList[count * 4 + 3] -> face() = f_newList[f_newList.size() - 1];
    }
    // EDGES
    for(size_t count = 0; count < num_edge; count++){
        e_newList[count * 2] -> halfedge() = h_newList[count * 4];
        e_newList[count * 2 + 1] -> halfedge() = h_newList[count * 4 + 2];
    }
    // VERTICES
    for(size_t count = 0; count < num_edge; count++){
        v_newList[count] -> halfedge() = h_newList[count * 4 + 2];
        v_newList[count] -> pos = v_list[(count + 1) % num_edge] -> pos;
    }
    // FACES
    for(size_t count = 0; count < num_edge; count++){
        f_newList[count] -> halfedge() = h_list[count];
    }
    f_newList[f_newList.size() - 1] -> halfedge() = h_newList[3];
    
    
    // erase element
    erase(f);
    
    
    return f_newList[f_newList.size() - 1];
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {
    
    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());
    
    Vec3 p0 = start_positions[0];
    Vec3 p2 = start_positions[2];
    if(tangent_offset >= (p0 - p2).norm() / 2.0f){
        return;
    }
    
    size_t num_edge = new_halfedges.size();
    for(size_t i = 0; i < num_edge; i++)
    {
        Vec3 p = start_positions[i];                // vertex position
        Vec3 p_left = start_positions[(i+num_edge-1) % num_edge]; // left neighbour position
        Vec3 p_right = start_positions[(i+1) % num_edge];  // right neighbour position
        
        Vec3 p_proj = dot(p - p_left, (p_right - p_left).unit()) * (p_right - p_left).unit(); // projection vector
        Vec3 p_normal = (p - p_left) - p_proj;
        
        Vec3 p_end = p_proj + p_normal + p_normal.unit() * tangent_offset;   // offset on tangent
        p_end = p_end + (face -> normal()).unit() * normal_offset;             // offset on face normal
            
        new_halfedges[i] -> vertex() -> pos = p_end + p_left;
        
    }
    
    return;
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {
        
    // For each face...    
    std::vector<FaceRef> f_oldList;
    Size countFace = 0;
    // record non-triangle faces
    for(FaceRef f = faces_begin(); f != faces_end(); f++){
        if(f -> degree() == 3){
            continue; // skip triangle faces
            
        }else{
            f_oldList.push_back(f);
            countFace++;
        }
}
    
    for(Size countId = 0; countId < countFace; countId++){
        
        FaceRef f = f_oldList[countId];
        
        // collect neighbours info
        std::vector<HalfedgeRef> h_list, h_newList;    // HALFEDGES
        std::vector<VertexRef> v_list;      // VERTICES
        std::vector<EdgeRef> e_list, e_newList;        // EDGES
        std::vector<FaceRef> f_newList;        // FACES
        
        HalfedgeRef h = f -> halfedge();
        do{
            // HALFEDGES
            h_list.push_back(h);
            // VERTICES
            v_list.push_back(h -> vertex());
            // EDGES
            e_list.push_back(h -> edge());     // not used
            
            h = h -> next();
        }while(h != f -> halfedge());
        size_t num_edge = e_list.size();
        
        // create new elements
        // HALFEDGES
        for(size_t count = 0; count < 2 * (num_edge - 3); count++){
            h_newList.push_back(new_halfedge());
        }
    
        // EDGES
        for(size_t count = 0; count < num_edge - 3; count++){
            e_newList.push_back(new_edge());
        }
        // FACES
        //f_newList.push_back(f);
        for(size_t count = 0; count < num_edge - 2; count++){
            f_newList.push_back(new_face());
        }
    
        
        // reassign elements
        // HALFEDGES
        for(size_t count = 0; count < num_edge - 3; count++){
            h_newList[2 * count] -> next() = h_list[count + 2];
            h_newList[2 * count] -> twin() = h_newList[2 * count + 1];
            h_newList[2 * count] -> vertex() = v_list[0];
            h_newList[2 * count] -> edge() = e_newList[count];
            h_newList[2 * count] -> face() = f_newList[count + 1];
            
            if(count == 0){
                h_newList[2 * count + 1] -> next() = h_list[0];
            }else{
                h_newList[2 * count + 1] -> next() = h_newList[2 * (count - 1)];
            }
            h_newList[2 * count + 1] -> twin() = h_newList[2 * count];
            h_newList[2 * count + 1] -> vertex() = v_list[count + 2];
            h_newList[2 * count + 1] -> edge() = e_newList[count];
            h_newList[2 * count + 1] -> face() = f_newList[count];
        }
    
        h_list[0] -> face() = f_newList[0];
        
        h_list[num_edge - 1] -> next() = h_newList[2 * (num_edge - 3 - 1)];
        h_list[num_edge - 1] -> face() = f_newList[f_newList.size() - 1];
        
        h_list[num_edge - 2] -> face() = f_newList[f_newList.size() - 1];
        
        for(size_t count = 0; count < num_edge - 3; count++){
            h_list[count + 1] -> next() = h_newList[2 * count + 1];
            h_list[count + 1] -> face() = f_newList[count];
        }
    
        // VERTEX
        for(size_t count = 0; count < num_edge; count++){
            v_list[count] -> halfedge() = h_list[count];
        }
    
        // EDGES
        for(size_t count = 0; count < num_edge - 3; count++){
            e_newList[count] -> halfedge() = h_newList[2 * count];
        }
        // FACES
        for(size_t count = 0; count < num_edge - 2; count++){
            f_newList[count] -> halfedge() = h_list[count + 1];
        }
        
        //std::cout<< "h_list[1] -> face() is f_newList[0]: " << ((h_list[1] -> face()) == f_newList[0]) << std::endl;
        //std::cout<< "h_newList[3] -> next() is h_newList[0]: " << ((h_newList[3] -> next()) == h_newList[0]) << std::endl;
        
        // erase element
        erase(f);
        
        // validate operation for current face
        validate();
    }
    
    return;
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    
    // TODO: implement this
    
    // VERTICES
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++){
        v -> new_pos = v -> pos;
    }
    
    // EDGES
    for(EdgeRef e = edges_begin(); e != edges_end(); e++){
        HalfedgeRef h1 = e -> halfedge();
        HalfedgeRef h2 = h1 -> twin();
        VertexRef v1 = h1 -> vertex(), v2 = h2 -> vertex();
        
        e -> new_pos = ((v1 -> pos) + (v2 -> pos)) / 2;
    }
    
    // FACES
    for(FaceRef f = faces_begin(); f != faces_end(); f++){
        HalfedgeRef h = f -> halfedge();
        Vec3 avgPos(0.0f,0.0f,0.0f);
        int edgeCount = 0;
        do{
            avgPos += h -> vertex() -> pos;
            edgeCount++;
            h = h -> next();
        }while(h != f -> halfedge());
        
        avgPos /= edgeCount;
        
        f -> new_pos = avgPos;
    }
    
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)
    
    // Faces
    //Vec3 avgFacePos(0,0,0);
    //Size numFace = n_faces();
    for(FaceRef f = faces_begin(); f != faces_end(); f++){
        HalfedgeRef h = f -> halfedge();
        Vec3 avgPos(0.0f,0.0f,0.0f);
        do{
            avgPos += h -> vertex() -> pos;
            h = h -> next();
        }while(h != f -> halfedge());
        
        avgPos /= f -> degree();
        
        f -> new_pos = avgPos;
        //avgFacePos += avgPos;
    }
    //avgFacePos /= numFace;
    
    // Edges
    for(EdgeRef e = edges_begin(); e != edges_end(); e++){
        HalfedgeRef h1 = e -> halfedge();
        HalfedgeRef h2 = h1 -> twin();
        FaceRef f1 = h1 -> face(), f2 = h2 -> face();
        
        e -> new_pos = ((f1 -> new_pos) + (f2 -> new_pos) + (e -> center())) / 3;
        
    }
    
    // Vertices
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++){
        int n = v -> degree();
        Vec3 S = v -> pos;
        Vec3 Q(0,0,0), R(0,0,0);
        HalfedgeRef h = v -> halfedge();
        
        do{
            Q += h -> face() -> new_pos;
            R += h -> edge() -> center();
            
            h = h -> twin() -> next();
        }while(h != v -> halfedge());
        
        Q /= n;
        R /= n;
        
        v -> new_pos = (Q + 2*R + (n - 3)*S) / n;
        
    }
    
    
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {
    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flag Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.
    
    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
    
        
    // Assume all faces in mesh are triangles
    
    // computer new position of existed vertices
    Size numVertex = n_vertices();
    VertexRef v = vertices_begin();
    for(Size count = 0; count < numVertex; count++, v++){
        int n = v -> degree();
        float u;
        if(n == 3){
            u = 3.0f / 16.0f;
        }else{
            u = 3.0f / (8.0f * (float)n);
        }
        
        Vec3 sumPos(0,0,0);
        HalfedgeRef h_temp = v -> halfedge();
        do{
            sumPos += h_temp -> twin() -> vertex() -> pos;
            h_temp = h_temp -> twin() -> next();
        }while(h_temp != v -> halfedge());
        
        v -> new_pos = (1.0f - n * u) * (v -> pos) + u * sumPos;
        v -> is_new = false;    // still belongs to original mesh
                
    }
    
    // compute new edge position using new vertices position
    Size numEdge = n_edges();
    EdgeRef e = edges_begin();
    for(Size count = 0; count < numEdge; count++, e++){
        VertexRef v1 = e -> halfedge() -> vertex();
        VertexRef v2 = e -> halfedge() -> twin() -> vertex();
        Vec3 newPos = ((v1 -> new_pos) + (v2 -> new_pos)) / 2.0f;
        
        e -> new_pos = newPos;
        e -> is_new = false;
    }
    
    // splitting edges
    e = edges_begin();   // reset
    for(Size count = 0; count < numEdge; count++){
        // record next edge
        EdgeRef nextEdge = e;
        nextEdge++;
        
        // split the edge
        VertexRef v1 = e -> halfedge() -> vertex();             // record endpoints
        VertexRef v2 = e -> halfedge() -> twin() -> vertex();
        
        auto ref = split_edge(e);   // split edge
        VertexRef v_new = ref.value();
                
        // flag new vertex
        v_new -> is_new = true;
        v_new -> new_pos = e -> new_pos;
        
        // flag new edge
        HalfedgeRef h_temp = v_new -> halfedge();
        do{
            if((h_temp -> twin() -> vertex() == v1) || (h_temp -> twin() -> vertex() == v2)){
                h_temp -> edge() -> is_new = false;
            }else{
                h_temp -> edge() -> is_new = true;
            }
            
            h_temp = h_temp -> twin() -> next();
        }while(h_temp != v_new -> halfedge());
        
        e = nextEdge;    // move to next edge
    }
    
    // flip edges, no new edges added
    for(e = edges_begin(); e != edges_end(); e++){
        if(!(e -> is_new)){ continue; }     // skip old edges

        VertexRef v1 = e -> halfedge() -> vertex();
        VertexRef v2 = e -> halfedge() -> twin() -> vertex();

        while((v1 -> is_new) != (v2 -> is_new)){
            auto ref = flip_edge(e);
            EdgeRef e_temp = ref.value();

            v1 = e_temp -> halfedge() -> vertex();
            v2 = e_temp -> halfedge() -> twin() -> vertex();
        }

    }
    
    // update vertex position
    for(v = vertices_begin(); v != vertices_end(); v++){
        v -> pos = v -> new_pos;
    }
    
    // validate operation
    validate();
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
