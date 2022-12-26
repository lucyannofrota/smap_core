void Conceptual_Map::add_vertex(const geometry_msgs::msg::Point & pos)
{
  static geometry_msgs::msg::Point *initial_point = NULL;
  static bool initialization = true;
  if (initial_point == NULL) {
    initial_point = new geometry_msgs::msg::Point;
    initial_point->x = pos.x;
    initial_point->y = pos.y;
    initial_point->z = pos.z;
    return;
  }


  double distance;
  if (initialization) {
    distance = sqrt(
      pow(initial_point->x - pos.x, 2) +
      pow(initial_point->y - pos.y, 2) +
      pow(initial_point->z - pos.z, 2)
    );

    if (distance < VERTEX_DISTANCE) {return;} else {initialization = false;}
  }


  long v_index = 0;
  size_t idx;
  static bool loaded = false;
  if (current_vertex == NULL) {
    auto v_pair = boost::vertices(Semantic_Graph);
    for (auto iter = v_pair.first; iter != v_pair.second; iter++) {
      loaded = true;
      distance = sqrt(
        pow(Semantic_Graph[*iter].pos.x - pos.x, 2) +
        pow(Semantic_Graph[*iter].pos.y - pos.y, 2) +
        pow(Semantic_Graph[*iter].pos.z - pos.z, 2)
      );
      if (Semantic_Graph[*iter].index > v_index) {v_index = Semantic_Graph[*iter].index;}
      if (distance <= VERTEX_DISTANCE) {
        current_vertex = &(Semantic_Graph[*iter]);
        return;
      }
      v_index++;
    }
    idx = this->_add_vertex(
      v_index,
      pos,
      Concept(),
      std::list<Concept>()
    );

    // RCLCPP_DEBUG(this->get_logger(), "First Vertex added [%f,%f,%f]",pos.x,pos.y,pos.z);
    current_vertex = &(Semantic_Graph[idx]);
    return;
  }


  v_index = current_vertex->index + 1;

  distance = sqrt(
    pow(current_vertex->pos.x - pos.x, 2) +
    pow(current_vertex->pos.y - pos.y, 2) +
    pow(current_vertex->pos.z - pos.z, 2)
  );


  VertexData * closest = _get_edge_match(pos);

  if (closest == NULL) {
    RCLCPP_DEBUG(this->get_logger(), "closest == NULL");
    if (distance < VERTEX_DISTANCE) {return;} else {
      idx = this->_add_vertex(
        v_index,
        pos,
        Concept(),
        std::list<Concept>()
      );
      previous_vertex = current_vertex;
      current_vertex = &(Semantic_Graph[idx]);
    }
  } else { // closest != NULL
    RCLCPP_DEBUG(this->get_logger(), "closest != NULL");
    if (previous_vertex == NULL) {
      RCLCPP_DEBUG(this->get_logger(), "previous_vertex == NULL");
      if (loaded) {previous_vertex = closest;} else {
        if (distance < VERTEX_DISTANCE) {return;} else {
          idx = this->_add_vertex(
            v_index,
            pos,
            Concept(),
            std::list<Concept>()
          );
          previous_vertex = current_vertex;
          current_vertex = &(Semantic_Graph[idx]);
        }
      }
    } else {
      RCLCPP_DEBUG(this->get_logger(), "previous_vertex != NULL");
      if (closest->index == previous_vertex->index) {
        if (distance < VERTEX_DISTANCE) {return;} else {
          idx = this->_add_vertex(
            v_index,
            pos,
            Concept(),
            std::list<Concept>()
          );
          previous_vertex = current_vertex;
          current_vertex = &(Semantic_Graph[idx]);
        }
      } else {
        previous_vertex = current_vertex;
        current_vertex = closest;
      }

    }
  }

  distance = sqrt(
    pow(current_vertex->pos.x - previous_vertex->pos.x, 2) +
    pow(current_vertex->pos.y - previous_vertex->pos.y, 2) +
    pow(current_vertex->pos.z - previous_vertex->pos.z, 2)
  );
  RCLCPP_DEBUG(
    this->get_logger(), "Edge added %i->%i [%4.1f,%4.1f,%4.1f]->[%4.1f,%4.1f,%4.1f]", previous_vertex->index, current_vertex->index,
    previous_vertex->pos.x, previous_vertex->pos.y, previous_vertex->pos.z,
    current_vertex->pos.x, current_vertex->pos.y, current_vertex->pos.z
  );


  RCLCPP_DEBUG(this->get_logger(), "try edge dist=%4.1f", distance);

  _add_edge(
    _get_TopoMap_iterator(previous_vertex),
    _get_TopoMap_iterator(current_vertex)
  );
}