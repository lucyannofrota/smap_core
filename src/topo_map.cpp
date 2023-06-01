#include "../include/smap_core/topo_map.hpp"

namespace smap
{
bool topo_map::add_edge( const size_t& previous, const size_t& current )
{
    vertex_data_t prev, cur;
    this->get_vertex( previous, prev );
    this->get_vertex( current, cur );
    double distance =
        sqrt( pow( prev.pos.x - cur.pos.x, 2 ) + pow( prev.pos.y - cur.pos.y, 2 ) + pow( prev.pos.z - cur.pos.z, 2 ) );
    for( auto e: boost::make_iterator_range( boost::out_edges( this->_get_vertex( current ), this->graph ) ) )
        if( boost::target( e, this->graph ) == this->_get_vertex( previous ) ) return false;
    boost::add_edge( this->_get_vertex( previous ), this->_get_vertex( current ), { distance, 1 }, this->graph );
    this->markers.append_edge( prev.pos, cur.pos );
    RCLCPP_INFO(
        this->get_logger(), "Edge added %i->%i [ % 4.1f, % 4.1f, % 4.1f ] -> [ % 4.1f, % 4.1f, % 4.1f ] ", previous,
        current, prev.pos.x, prev.pos.y, prev.pos.z, cur.pos.x, cur.pos.y, cur.pos.z );
    return true;
}

size_t topo_map::get_closest_vertex( const geometry_msgs::msg::Point pos, double& min )
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

void topo_map::add_vertex( const geometry_msgs::msg::Point& pos, size_t& current, size_t& previous )
{
    printf( "pos: [%6.1f,%6.1f,%6.1f]\n", pos.x, pos.y, pos.z );
    double dist_current;

    size_t idx;

    printf( "add_vertex\n" );
    // Initialization
    if( current == (size_t) -1 )
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

    if( ( previous != (size_t) -1 ) && ( closest != (size_t) -1 ) && ( closest != current )
        && ( dist_current <= VERTEX_DISTANCE * NEW_EDGE_FACTOR ) )
    {
        printf( "\tConnection/Relocation\n" );
        if( ( closest != previous ) )
        {
            printf( "\tConnection\n" );
            this->print_vertex( std::string( "\t\tCurrent: " ), current );
            this->print_vertex( std::string( "\t\tPrevious: " ), previous );
            this->print_vertex( std::string( "\t\tClosest: " ), closest );
            this->add_edge( current, closest );
        }
        else printf( "\tRelocation\n" );
        previous = current;
        current  = closest;
        this->print_vertex( std::string( "\t\t/Current: " ), current );
        this->print_vertex( std::string( "\t\t/Previous: " ), previous );
        this->print_vertex( std::string( "\t\t/Closest: " ), closest );
        this->export_graph();
        return;
    }

    // New vertex
    if( dist_current < VERTEX_DISTANCE * NEW_EDGE_FACTOR )
    {
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
    this->export_graph();
}

void topo_map::add_object( const geometry_msgs::msg::Pose pose )
{
    // TODO: create callback group. Should be mutually exclusive
    printf( "add_object\n" );
    if( boost::num_vertices( this->graph ) == 0 ) return;

    size_t current = -1, previous = -1;
    double distance, t;
    // previous will be the closest point in this case
    previous = this->get_closest_vertex( pose.position, distance );
    vertex_data_t pre;
    this->get_vertex( previous, pre );

    if( distance < 2 * VERTEX_DISTANCE )
    {
        // TODO: Implement method
        // this->append_object();
        printf( "append_object\n" );
        return;
    }

    int n_new_vertex, count = 1;
    geometry_msgs::msg::Point new_point, base_point;
    base_point   = pre.pos;
    new_point    = pre.pos;

    n_new_vertex = floor( distance / VERTEX_DISTANCE );
    t            = ( VERTEX_DISTANCE ) / distance;
    printf( "Create support nodes\nn: %i|t: %6.1f\n", n_new_vertex, t );
    while( distance > VERTEX_DISTANCE && n_new_vertex > 0 )
    {
        new_point.x = ( 1 - t * count ) * base_point.x + t * count * pose.position.x;
        new_point.y = ( 1 - t * count ) * base_point.y + t * count * pose.position.y;

        int idx     = this->_add_vertex( this->v_index++, new_point );
        current     = this->graph[ idx ].index;

        this->add_edge( previous, current );

        previous = current;
        this->get_vertex( previous, pre );
        distance = this->_calc_distance( pre.pos, pose.position );
        n_new_vertex--;
        count++;
    }
}

}  // namespace smap

int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );

    std::shared_ptr< smap::topo_map > node = std::make_shared< smap::topo_map >();

    try
    {
        rclcpp::spin( node );
    }
    catch( std::exception& e )
    {
        std::cout << "SMAP Exception!" << std::endl;
        std::cout << e.what() << std::endl;
    }
    rclcpp::shutdown();

    return 0;
}
