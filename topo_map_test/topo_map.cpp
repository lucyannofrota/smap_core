#include <boost/graph/adjacency_list.hpp>
#include <cmath>
// #include <boost/graph/properties.hpp>
// #include <boost/graph/named_function_params.hpp>
// #include <boost/property_map/property_map.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>

// TODO: Change point_t to point_t

struct point_t
{
    float x;
    float y;
    float z;
};

struct thing_t
{
    std::pair< std::string, std::string > get_vertex_representation()
    {
        return std::pair< std::string, std::string >( std::string( "Corridor" ), std::string( "red" ) );
    }
};

struct vertex_data_t
{
    size_t index;
    point_t pos;
    thing_t this_thing;

    friend class boost::serialization::access;

    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        // ar &index;
        ar& pos.x;
        ar& pos.y;
        ar& pos.z;
        // ar &thing;
        // TODO: add parameters
    }
};

struct edge_data_t
{
    double distance = 0;
    double modifier = 1;

    double get_cost( void ) { return round( distance * modifier * 100 ) / 100.0; }

    friend class boost::serialization::access;

    template< class Archive >
    inline void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar& distance;
        ar& modifier;
    }
};

typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, vertex_data_t, edge_data_t > graph_t;

// https://stackoverflow.com/questions/71488845/how-to-configure-boostgraph-to-use-my-own-stable-index-for-vertices
// traits
template<>
struct boost::graph::internal_vertex_name< vertex_data_t >
{
    struct type
    {
        using result_type = size_t;

        const result_type& operator()( const vertex_data_t& bundle ) const { return bundle.index; }
    };
};

template<>
struct boost::graph::internal_vertex_constructor< vertex_data_t >
{
    struct type
    {
      private:

        using extractor = typename internal_vertex_name< vertex_data_t >::type;
        using name_t    = std::decay_t< typename extractor::result_type >;

      public:

        using argument_type = name_t;
        using result_type   = vertex_data_t;

        result_type operator()( const name_t& index ) const { return { index }; }
    };
};

namespace smap
{

#define OUTPUT_PATH std::string( "./" )
#define VERTEX_DISTANCE 1
#define NEW_EDGE_FACTOR 0.95  // Must be > 1

template< class prop_1, class prop_2 >
class vertex_label_writer
{
  public:

    vertex_label_writer( prop_1 p1, prop_2 p2 ) : thing( p1 ), pos( p2 ) {}

    template< class VertexOrEdge >
    void operator()( std::ostream& out, const VertexOrEdge& v )
    {
        std::pair< std::string, std::string > g_pair = get( thing, v ).get_vertex_representation();
        point_t point                                = get( pos, v );
        // add_pos
        out << "[label=" << boost::escape_dot_string( add_pos( add_cout( g_pair.first ), point ) )
            << ",color=\"" + g_pair.second + "\"]";
    }

    std::string add_cout( const std::string cname )
    {
        std::list< std::pair< std::string, int > >::iterator it;
        if( find( cname, it ) )
        {
            ( *it ).second++;
            return cname + "_" + std::to_string( ( *it ).second - 1 );
        }
        classes.push_back( std::pair< std::string, int > { cname, 1 } );
        return cname + std::string( "_0" );
    }

    std::string add_pos( const std::string text, const point_t point )
    {
        char nuns[ 6 * 3 + 2 * 3 ];
        sprintf( nuns, "%6.1f, %6.1f, %6.1f", point.x, point.y, point.z );
        return text + std::string( "\n[" + std::string( nuns ) + "]" );
    }

    bool find( const std::string cname, std::list< std::pair< std::string, int > >::iterator& it )
    {
        size_t id = 0;
        for( it = classes.begin(); it != classes.end(); it++, id++ )
            if( ( *it ).first == cname ) return true;
        return false;
    }

  private:

    prop_1 thing;
    prop_2 pos;
    std::list< std::pair< std::string, int > > classes;
};

template< class prop_1, class prop_2 >
inline vertex_label_writer< prop_1, prop_2 > make_vertex_label_writer( prop_1 p1, prop_2 p2 )
{
    return vertex_label_writer< prop_1, prop_2 >( p1, p2 );
}

template< class prop_1, class prop_2 >
class cost_label_writer
{
  public:

    cost_label_writer( prop_1 p1, prop_2 p2 ) : prop1( p1 ), prop2( p2 ) {}

    template< class VertexOrEdge >
    void operator()( std::ostream& out, const VertexOrEdge& v ) const
    {
        char buf[ 16 ];
        sprintf( buf, "%6.2f", round( ( boost::get( prop1, v ) * boost::get( prop2, v ) * 100 ) ) / 100 );
        out << "[label=" << std::string( buf ) << "]";
    }

  private:

    prop_1 prop1;
    prop_2 prop2;
};

template< class prop_1, class prop_2 >
inline cost_label_writer< prop_1, prop_2 > make_cost_label_writer( prop_1 p1, prop_2 p2 )
{
    return cost_label_writer< prop_1, prop_2 >( p1, p2 );
}

class topo_map
{
  private:

    friend class boost::serialization::access;

    graph_t graph;

    size_t previous_idx = -1;
    size_t current_idx  = -1;

    size_t v_index      = 1;

    bool publish_vertex = false, publish_edge = false;

    inline size_t _add_vertex( size_t v_index, const point_t& pos )
    {
        size_t ret     = boost::add_vertex( vertex_data_t { v_index, pos, thing_t() }, this->graph );
        publish_vertex = true;
        _append_vertex_marker( pos );
        // TODO: To complete ROS
        // RCLCPP_INFO(this->get_logger(), "Vertex added (%li) [%4.1f,%4.1f,%4.1f]",
        // v_index, pos.x, pos.y, pos.z);
        return ret;
    }

    inline void _append_vertex_marker( const point_t& pos )
    {
        // TODO: To complete ROS
        // static bool init_flag = true;
        // static int id = 0;
        // // for(auto it = vertex_marker.points.begin(); it !=
        // vertex_marker.points.end(); it++){
        // //   if((it->x == pos.x) && (it->y == pos.y) && (it->z == pos.z)) return;
        // // }
        // vertex_marker.header.stamp = this->get_clock().get()->now();
        // vertex_marker.id = id;
        // id++;
        // vertex_marker.points.push_back(pos);
        // if (init_flag)
        // {
        //     init_flag = false;
        //     vertex_marker.action = visualization_msgs::msg::Marker::MODIFY;
        // }
        // publish_vertex = true;
    }

    inline void _append_edge_marker( const point_t& pos1, const point_t& pos2 )
    {
        // TODO: To complete ROS
        // static bool init_flag = true;
        // static int id = 0;

        // edge_marker.header.stamp = this->get_clock().get()->now();
        // edge_marker.id = id;
        // id++;
        // if (init_flag)
        // {
        //     edge_marker.points.push_back(pos1);
        //     edge_marker.points.push_back(pos2);
        //     init_flag = false;
        //     edge_marker.action = visualization_msgs::msg::Marker::MODIFY;
        // }
        // else
        // {
        //     edge_marker.points.push_back(pos2);
        // }
        // publish_edge = true;
    }

    inline void add_edge( const size_t& previous, const size_t& current )
    {
        vertex_data_t prev, cur;
        this->get_vertex( previous, prev );
        this->get_vertex( current, cur );
        double distance = sqrt(
            pow( prev.pos.x - cur.pos.x, 2 ) + pow( prev.pos.y - cur.pos.y, 2 ) + pow( prev.pos.z - cur.pos.z, 2 ) );
        for( auto e: boost::make_iterator_range( boost::out_edges( this->_get_vertex( current ), this->graph ) ) )
            if( boost::target( e, this->graph ) == this->_get_vertex( previous ) ) return;
        boost::add_edge(
            this->_get_vertex( previous ), this->_get_vertex( current ),
            // previous,
            // current,
            { distance, 1 }, this->graph );
        this->publish_edge = true;
        this->_append_edge_marker( prev.pos, cur.pos );
        // TODO: To complete ROS
        // RCLCPP_INFO(
        //     this->get_logger(), "Edge added %i->%i
        //     [%4.1f,%4.1f,%4.1f]->[%4.1f,%4.1f,%4.1f]", previous, current,
        //     Graph[previous].pos.x, Graph[previous].pos.y, Graph[previous].pos.z,
        //     Graph[current].pos.x, Graph[current].pos.y,
        //     Graph[current].pos.z);
    }

    size_t _get_vertex( const size_t& v_index )
    {
        if( auto v = this->graph.vertex_by_property( { v_index } ) ) return *v;
        // Case vertex dont't exists
        // TODO: To complete ROS
        printf( "Vertex not found!\n" );
        return -1;
    }

    void get_vertex( const size_t& v_index, vertex_data_t& vertex )
    {
        if( auto v = this->graph.vertex_by_property( { v_index } ) ) vertex = this->graph[ *v ];
        else
        {
            // Case vertex dont't exists
            // TODO: To complete ROS
            printf( "Vertex not found!\n" );
            vertex.index = -1;
        }
    }

    size_t get_closest_vertex( const point_t pos, double& min )
    {
        auto pair      = boost::vertices( this->graph );
        size_t closest = -1;
        min            = DBL_MAX;
        double distance;
        for( auto it = pair.first; it != pair.second; ++it )
        {
            distance = topo_map::_calc_distance( pos, this->graph[ *it ].pos );
            if( distance < min )
            {
                min     = distance;
                closest = this->graph[ *it ].index;
            }
        }

        return closest;
    }

    inline static double _calc_distance( const point_t& p1, const point_t& p2 )
    {
        return sqrt( pow( p1.x - p2.x, 2 ) + pow( p1.y - p2.y, 2 ) + pow( p1.z - p2.z, 2 ) );
    }

  public:

    void add_vertex( const point_t& pos ) { this->add_vertex( pos, this->current_idx, this->previous_idx ); }

    void add_vertex( const point_t& pos, size_t& current, size_t& previous )
    {
        printf( "pos: [%6.1f,%6.1f,%6.1f]\n", pos.x, pos.y, pos.z );
        double dist_current;

        size_t idx;

        printf( "add_vertex\n" );
        // Initialization
        if( current == -1 )
        {
            printf( "\tInitialization\n" );
            printf( "\t\tcurrent -1\n" );
            printf( "\t\tadding Vertex\n" );
            idx     = this->_add_vertex( this->v_index++, pos );
            current = this->graph[ idx ].index;
            this->print_vertex( std::string( "\t\t" ), current );
            return;
        }

        vertex_data_t cur, clo;
        size_t closest = this->get_closest_vertex( pos, dist_current );
        this->get_vertex( current, cur );
        this->get_vertex( closest, clo );

        if( ( previous != -1 ) && ( closest != -1 ) && ( closest != current ) && ( closest != previous )
            && ( dist_current <= VERTEX_DISTANCE * NEW_EDGE_FACTOR ) )
        {
            printf( "\tConnection\n" );
            this->print_vertex( std::string( "\t\tCurrent: " ), current );
            this->print_vertex( std::string( "\t\tPrevious: " ), previous );
            this->print_vertex( std::string( "\t\tClosest: " ), closest );
            this->add_edge( current, closest );
            previous = current;
            current  = closest;
            this->print_vertex( std::string( "\t\t/Current: " ), current );
            this->print_vertex( std::string( "\t\t/Previous: " ), previous );
            this->print_vertex( std::string( "\t\t/Closest: " ), closest );
            return;
        }

        // New vertex
        if( dist_current < VERTEX_DISTANCE * NEW_EDGE_FACTOR )
        {
            // Connection
            printf( "\tToo Close\n" );
            return;
        }

        printf( "\tNew vertex\n" );

        printf( "\t\tadding Vertex\n" );
        this->print_vertex( std::string( "\t\t\tCurrent: " ), current );
        this->print_vertex( std::string( "\t\t\tPrevious: " ), previous );
        this->print_vertex( std::string( "\t\t\tClosest: " ), closest );
        idx      = this->_add_vertex( this->v_index++, pos );
        previous = current;
        current  = this->graph[ idx ].index;
        this->print_vertex( std::string( "\t\t\t/Current: " ), current );
        this->print_vertex( std::string( "\t\t\t/Previous: " ), previous );
        this->print_vertex( std::string( "\t\t\t/Closest: " ), closest );

        printf( "\t\tadding Edge\n" );
        this->print_vertex( std::string( "\t\t\tCurrent: " ), current );
        this->print_vertex( std::string( "\t\t\tPrevious: " ), previous );
        this->print_vertex( std::string( "\t\t\tClosest: " ), closest );
        this->add_edge( previous, current );
        this->print_vertex( std::string( "\t\t\t/Current: " ), current );
        this->print_vertex( std::string( "\t\t\t/Previous: " ), previous );
        this->print_vertex( std::string( "\t\t\t/Closest: " ), closest );
    }

    void add_object( const point_t& pos )
    {
        // TODO: Continue ROS
        // TODO: create callback group. Should be mutually exclusive

        if( boost::num_vertices( this->graph ) == 0 ) return;

        size_t current = -1, previous = -1;
        double distance, t;
        // previous will be the closest point in this case
        previous = this->get_closest_vertex( pos, distance );
        vertex_data_t pre;
        this->get_vertex( previous, pre );

        if( distance < 2 * VERTEX_DISTANCE )
        {
            // this->append_object();
        }

        int n_new_vertex, count = 1;
        point_t new_point, base_point;
        base_point = pre.pos;
        // new_point = pre.pos;
        new_point.z  = 1;  // TODO: ROS

        n_new_vertex = floor( distance / VERTEX_DISTANCE );
        t            = ( VERTEX_DISTANCE ) / distance;
        printf( "n: %i|t: %6.1f\n", n_new_vertex, t );
        while( distance > VERTEX_DISTANCE && n_new_vertex > 0 )
        {
            new_point.x = ( 1 - t * count ) * base_point.x + t * count * pos.x;
            new_point.y = ( 1 - t * count ) * base_point.y + t * count * pos.y;

            int idx     = this->_add_vertex( this->v_index++, new_point );
            current     = this->graph[ idx ].index;

            this->add_edge( previous, current );

            previous = current;
            this->get_vertex( previous, pre );
            distance = this->_calc_distance( pre.pos, pos );
            n_new_vertex--;
            count++;
        }
    }

    void print_vertex( const std::string& prefix, const size_t& idx )
    {
        if( auto v = this->graph.vertex_by_property( { idx } ) )
        {
            printf(
                "%s (%i) [%4.1f,%4.1f,%4.1f]\n", prefix.c_str(), (int) this->graph[ *v ].index, this->graph[ *v ].pos.x,
                this->graph[ *v ].pos.y, this->graph[ *v ].pos.z );
        }
    };

    void print_graph( void ) { boost::print_graph( this->graph, boost::get( &vertex_data_t::index, this->graph ) ); }

    void export_graph( void ) { this->export_graph( "topo_map" ); }

    void export_graph( const std::string& f_name )
    {
        std::ofstream dotfile( OUTPUT_PATH + f_name + ".dot" );

        // auto ids = get(&vertex_data_t::index, this->graph);
        // // std::map<&vertex_data_t::index> ids;

        // size_t num = 0;
        // for (auto u : boost::make_iterator_range(vertices(this->graph)))
        // {
        //     put(ids, u, num++);
        // }

        // boost::default_writer w;

        write_graphviz(
            dotfile, this->graph,
            make_vertex_label_writer(
                boost::get( &vertex_data_t::this_thing, this->graph ), boost::get( &vertex_data_t::pos, this->graph ) ),
            make_cost_label_writer(
                boost::get( &edge_data_t::distance, this->graph ),
                boost::get( &edge_data_t::modifier, this->graph ) ) );

        if( std::system( ( "dot -Tpng " + OUTPUT_PATH + f_name + ".dot > " + OUTPUT_PATH + f_name + ".png" ).c_str() )
            == 0 )
        {
            if( std::system( ( "rm " + OUTPUT_PATH + f_name + ".dot" ).c_str() ) == 0 )
            {
                // https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
                printf(
                    "\033[42m[Export Complete]\033[0m png file successfully "
                    "exported to: %s.png\n",
                    std::string( OUTPUT_PATH + f_name ).c_str() );
            }
        }
    }
};
}  // namespace smap

int main( void )
{
    smap::topo_map map;
    size_t current = -1, previous = -1;
    graph_t g;
    map.add_vertex( { 1, 2, 1 } );
    map.add_vertex( { 1, 1, 1 } );
    map.add_vertex( { 2, 1, 1 } );
    map.add_vertex( { 2, 2, 1 } );
    map.add_vertex( { 1, 2, 1 } );
    map.add_vertex( { 1.5, 3, 1 } );
    map.add_vertex( { 2, 2, 1 } );

    map.add_object( { 7, 7, 1 } );

    map.add_vertex( { 2.3, 3.6, 1 } );

    map.add_object( { -25, -25, 1 } );

    map.add_vertex( { 2.3, 5, 1 } );

    map.print_graph();
    map.export_graph();

    // auto x = boost::add_vertex(
    //     {5,
    //      {1, 2, 3},
    //      thing_t()},
    //     g);
    // auto y = boost::add_vertex(
    //     {2,
    //      {2, 2, 2},
    //      thing_t()},
    //     g);

    // boost::print_graph(g, boost::get(&vertex_data_t::index, g));
    // auto b = g.vertex_by_property({2});
    // std::cout << g[*b].pos.x << std::endl;
    // std::cout << x << std::endl;
    // std::cout << y << std::endl;
    // assert(x == *g.vertex_by_property(g[x]));
    // vertex_data_t a = vertex_data_t{3};
    // printf("a: %i [%f,%f,%f]\n", (int)a.index, a.pos.x, a.pos.y, a.pos.z);
    // // boost::print_graph(g, *g.vertex_by_property({5}));
    // if (auto v = g.vertex_by_property({7}))
    // {
    //     std::cout << g[*v].index << std::endl;
    // }
    // (*g.named_vertices.find(5));
    // assert(x == *g.named_vertices.find(5));

    // TODO: adaptar classe para trabalhar com essas definições
    // boost::print_graph(g, g.vertex_by_property({5}));
    // auto x = boost::add_vertex(
    //     {5,
    //      {1, 2, 3},
    //      thing_t()},
    //     g);
    // map.add_vertex({1, 2, 3}, current, previous);
    // map.export_graph();

    return 0;
}
