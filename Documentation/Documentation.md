# Topological_Map

Class responsible to create a interface to the TopoMap (Graph Map) and the things associated with each Vertex

- Has a tf buffer to track robot position

## Methods:

### set_logger()
Reference to the logger object of the node

### add_vertex()
Given the current postion <span style="color:red">pos</span>, if the distance beteween the last node and the current position is grater than <span style="color:blue">VERTEX_DISTANCE</span>, a new vertex related to <span style="color:red">pos</span> will be added and a edge will be established beteween the last and current vertices.

### export_TopoGraph()
Export a .png graph for visualization of the map

### save_map() and load_map()
Serialization of objects

## TopoMap (Graph Map)

The graph was implemented using the Boost Adjacent List library.

Boost adjacency_list: https://www.boost.org/doc/libs/1_55_0/libs/graph/doc/adjacency_list.html

Additions to the data stored in the TopoMap can be done in the structures that defines the Vertices and Edges.

### VertexData

VertexData:\
|  const long index = 0;\
|  geometry_msgs::msg::Point pos;\
|  thing this_thing;\
|  boost::container::list<thing> related_things;


- index: Index of the vertex
- pos: position of the vertex
- this_thing: Is the thing of the current Vertex. Should be a semantic_type_t::Location
- related_things: Linked list of things detected while in the current Vertex Location

Boost Container: https://www.boost.org/doc/libs/1_80_0/doc/html/container.html

### EdgeData

EdgeData:\
|  const double distance = 0;\
|  double modifier = 1;\
|  double cost = 0;

- distance: Distance between Vertices
- modifier: Modifier utilized to determine the cost of traversing this edge
- cost: distance*modifier

# thing
