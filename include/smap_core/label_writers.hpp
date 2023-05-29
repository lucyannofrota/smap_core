#ifndef LABEL_WRITERS_HPP_
#define LABEL_WRITERS_HPP_

#include <list>
#include <utility>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace smap
{

template<class Name_1, class Name_2>
class vertex_label_writer
{
public:
  vertex_label_writer(Name_1 n1, Name_2 n2)
  : name(n1), pos(n2) {}
  template<class VertexOrEdge>
  void operator()(std::ostream & out, const VertexOrEdge & v)
  {
    std::pair<std::string, std::string> g_pair = get(name, v).get_vertex_representation();
    geometry_msgs::msg::Point point = get(pos, v);
    // add_pos
    out << "[label=" << boost::escape_dot_string(
      add_pos(
        add_cout(g_pair.first), point
      )
    ) << ",color=\"" +
    g_pair.second + "\"]";
  }
  std::string add_cout(const std::string cname)
  {
    std::list<std::pair<std::string, int>>::iterator it;
    if (find(cname, it)) {
      (*it).second++;
      return cname + "_" + std::to_string((*it).second-1);
    }
    classes.push_back(std::pair<std::string, int>{cname, 1});
    return cname + std::string("_0");
  }
  std::string add_pos(const std::string text, const geometry_msgs::msg::Point point)
  {
    char nuns[6*3+2*3];
    sprintf(nuns,"%6.1f, %6.1f, %6.1f",
      point.x,
      point.y,
      point.z
    );
    return text + std::string(
      "\n["+
      std::string(nuns)+
      "]"
    );
  }
  bool find(const std::string cname, std::list<std::pair<std::string, int>>::iterator & it)
  {
    size_t id = 0;
    for (it = classes.begin(); it != classes.end(); it++, id++) {
      if ((*it).first == cname) {
        return true;
      }
    }
    return false;
  }

private:
  Name_1 name;
  Name_2 pos;
  std::list<std::pair<std::string, int>> classes;
};

template<class Name_1, class Name_2>
inline vertex_label_writer<Name_1, Name_2>
make_vertex_label_writer(Name_1 n1, Name_2 n2)
{
  return vertex_label_writer<Name_1, Name_2>(n1,n2);
}


//////////////////////////////


template<class Name_1, class Name_2>
class cost_label_writer
{
public:
  cost_label_writer(Name_1 p1, Name_2 p2)
  : prop1(p1), prop2(p2) {}
  template<class VertexOrEdge>
  void operator()(std::ostream & out, const VertexOrEdge & v) const
  {
    char buf[16];
    sprintf(buf, "%6.2f", round((boost::get(prop1, v) * boost::get(prop2, v) * 100)) / 100);
    out << "[label=" <<
      std::string(buf) <<
      "]";
  }

private:
  Name_1 prop1;
  Name_2 prop2;
};
template<class Name_1, class Name_2>
inline cost_label_writer<Name_1, Name_2>
make_cost_label_writer(Name_1 n1, Name_2 n2)
{
  return cost_label_writer<Name_1, Name_2>(n1, n2);
}

}

#endif  // LABEL_WRITERS_HPP_
