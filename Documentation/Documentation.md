# Conceptual_Map

Class responsible to create a interface to the TopoMap (Graph Map) and the Concepts associated with each Vertex

## TopoMap (Graph Map)

The graph was implemented using the Boost Adjacent List library.

Boost adjacency_list: https://www.boost.org/doc/libs/1_55_0/libs/graph/doc/adjacency_list.html

Additions to the data stored in the TopoMap can be done in the structures that defines the Vertices and Edges.

### VertexData

VertexData:\
|  const long index = 0;\
|  Concept this_thing;\
|  boost::container::list<Concept> related_things;


- index: Index of the vertice
- this_thing: Is the concept of the current Vertice. Should be a semantic_type::Location
- related_things: Linked list of Concepts detected while in the current Vertex Location

Boost Container: https://www.boost.org/doc/libs/1_80_0/doc/html/container.html

### EdgeData

EdgeData:\
|  const double distance = 0;\
|  double modifier = 1;\
|  double cost = 0;

- distance: Distance between Vertices
- modifier: Modifier utilized to determine the cost of traversing this edge
- cost: distance*modifier

# Concept
