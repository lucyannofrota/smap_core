#ifndef LABEL_WRITERS_HPP_
#define LABEL_WRITERS_HPP_

template<class Name>
class vertex_label_writer
{
public:
  vertex_label_writer(Name vertex_map)
  : name(vertex_map) {}
  template<class VertexOrEdge>
  void operator()(std::ostream & out, const VertexOrEdge & v) const
  {
    std::pair<std::string,std::string> g_pair = get(name, v).get_vertex_representation();
    out << "[label=" << boost::escape_dot_string(g_pair.first) << ",color=\""+g_pair.second+"\"]";
  }

private:
  Name name;
};
template<class Name>
inline vertex_label_writer<Name>
make_vertex_label_writer(Name n)
{
  return vertex_label_writer<Name>(n);
}


//////////////////////////////


template<class Name_1, class Name_2>
class cost_label_writer
{
public:
  cost_label_writer(Name_1 p1,Name_2 p2)
  : prop1(p1), prop2(p2) {}
  template<class VertexOrEdge>
  void operator()(std::ostream & out, const VertexOrEdge & v) const
  {
    char buf[16];
    sprintf(buf, "%6.2f",round((boost::get(prop1, v) * boost::get(prop2, v) * 100)) / 100);
    out << "[label=" << 
      std::string(buf)
     << "]";
  }

private:
  Name_1 prop1;
  Name_2 prop2;
};
template<class Name_1, class Name_2>
inline cost_label_writer<Name_1,Name_2>
make_cost_label_writer(Name_1 n1,Name_2 n2)
{
  return cost_label_writer<Name_1,Name_2>(n1,n2);
}


#endif  // LABEL_WRITERS_HPP_