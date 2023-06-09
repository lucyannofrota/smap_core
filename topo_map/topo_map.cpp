#include "topo_map.hpp"

namespace smap
{

void topo_map::observation_callback( const smap_interfaces::msg::SmapObservation::SharedPtr observation )
{
    // TODO: Add message "no classes registered"
    RCLCPP_DEBUG( this->get_logger(), "0. Check graph integrity" );
    if( boost::num_vertices( this->graph ) == 0 || this->reg_classes == nullptr || this->reg_detectors == nullptr )
    {
        RCLCPP_WARN( this->get_logger(), "No detectors registered." );
        return;
    }

    // TODO: Add mutex

    // 1. Get all adjacent vertexes 3 layers deep
    RCLCPP_DEBUG( this->get_logger(), "1. Get all adjacent vertexes 3 layers deep" );
    std::vector< size_t > idxs_checked, idxs_checking, idxs_to_check;  // Using iterator graph idxs (not v_indexes)
    double min_distance;
    idxs_checking.push_back(
        this->_get_vertex( this->get_closest_vertex( observation->object.pose.pose.position, min_distance ) ) );
    for( size_t i = 0; i <= 3; i++ )
    {
        for( auto checking: idxs_checking )
        {
            idxs_checked.push_back( checking );
            for( auto to_check: boost::make_iterator_range(
                     boost::adjacent_vertices( boost::vertex( checking, this->graph ), this->graph ) ) )
            {
                if( std::find( idxs_checked.begin(), idxs_checked.end(), to_check ) != idxs_checked.end() ) continue;
                else idxs_to_check.push_back( to_check );
            }
        }
    }

    // 2. Filter possible vertexes
    RCLCPP_DEBUG( this->get_logger(), "2. Filter possible vertexes" );
    std::vector< std::vector< thing* > > candidates;
    // int server_class_id = -1;
    for( auto checking = idxs_checked.begin(); checking != idxs_checked.end(); checking++ )
    {
        // Check list of objects inside each vertex
        std::vector< thing* > local_candidates;
        local_candidates.clear();
        bool has_candidate = false;
        for( auto obj: this->graph[ *checking ].related_things )
        {
            // Check labels
            if( !obj.label_is_equal( observation->object.module_id, observation->object.label ) ) continue;

            // Check active cone
            if( abs( rad2deg(
                    this->compute_direction( observation->object.pose.pose.position, observation->robot_pose.pose ) ) )
                > ACTIVE_FOV_H )
                continue;

            // Check position
            if( this->_calc_distance( observation->object.pose.pose.position, obj.pos ) > OBJECT_ERROR_DISTANCE )
                continue;

            local_candidates.push_back( &obj );
            has_candidate = true;
        }
        if( !has_candidate ) idxs_checked.erase( checking );
        else candidates.push_back( local_candidates );
    }

    // 3. Verify the existence of the detector
    auto det       = this->reg_detectors->begin();
    bool det_found = false;
    for( ; det != this->reg_detectors->end(); det++ )
    {
        // printf( "det id: %i |%i\n", observation->object.module_id, det->id );
        if( det->id == observation->object.module_id )
        {
            det_found = true;
            break;
        }
    }
    if( !det_found )  // return if no detector is found
    {
        RCLCPP_WARN( this->get_logger(), "Detector not found!" );
        return;
    }

    // 4. Update vertex
    // If true add object otherwise update an existing one
    RCLCPP_DEBUG( this->get_logger(), "3. Update vertex" );
    if( candidates.size() == 0 ) this->add_object( observation, *det );
    else
    {
        // Select the closest object
        RCLCPP_DEBUG( this->get_logger(), "Select the closest object" );
        thing* closest = nullptr;
        for( auto c: candidates )
        {
            for( auto lc: c )
            {
                if( closest == nullptr )
                {
                    closest      = lc;
                    min_distance = this->_calc_distance( observation->object.pose.pose.position, lc->pos );
                    continue;
                }

                if( this->_calc_distance( observation->object.pose.pose.position, lc->pos ) < min_distance )
                {
                    closest      = lc;
                    min_distance = this->_calc_distance( observation->object.pose.pose.position, lc->pos );
                }
            }
        }
        // TODO: Implement object update
        RCLCPP_DEBUG( this->get_logger(), "Object update" );
        closest->update(
            smap::semantic_type_t::OBJECT, observation->object, min_distance, (double) observation->direction, *det );
    }
}

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
    if( prev.strong_vertex && cur.strong_vertex ) this->markers.append_edge( prev.pos, cur.pos );
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

void topo_map::add_vertex( const geometry_msgs::msg::Point& pos, size_t& current, size_t& previous, bool strong_vertex )
{
    double dist_current;

    size_t idx;

    // printf( "add_vertex\n" );
    // Initialization
    if( current == (size_t) -1 )
    {
        // printf( "\tInitialization\n" );
        // printf( "\t\tcurrent -1\n" );
        // printf( "\t\tadding Vertex\n" );
        idx     = this->_add_vertex( this->v_index++, pos, strong_vertex );
        current = this->graph[ idx ].index;
        // this->print_vertex( std::string( "\t\t" ), current );
        return;
    }

    vertex_data_t cur, clo;
    size_t closest = this->get_closest_vertex( pos, dist_current );
    this->get_vertex( current, cur );
    this->get_vertex( closest, clo );

    if( ( previous != (size_t) -1 ) && ( closest != (size_t) -1 ) && ( closest != current )
        && ( dist_current <= VERTEX_DISTANCE * NEW_EDGE_FACTOR ) )
    {
        // printf( "\tConnection/Relocation\n" );
        if( ( closest != previous ) )
        {
            // printf( "\tConnection\n" );
            // this->print_vertex( std::string( "\t\tCurrent: " ), current );
            // this->print_vertex( std::string( "\t\tPrevious: " ), previous );
            // this->print_vertex( std::string( "\t\tClosest: " ), closest );
            this->add_edge( current, closest );
        }
        // else printf( "\tRelocation\n" );
        previous = current;
        current  = closest;
        // this->print_vertex( std::string( "\t\t/Current: " ), current );
        // this->print_vertex( std::string( "\t\t/Previous: " ), previous );
        // this->print_vertex( std::string( "\t\t/Closest: " ), closest );
        // this->export_graph();
        return;
    }

    // New vertex
    if( dist_current < VERTEX_DISTANCE * NEW_EDGE_FACTOR )
    {
        // printf( "\tToo Close\n" );
        return;
    }

    // printf( "\tNew vertex\n" );

    if( dist_current > VERTEX_DISTANCE * NEW_EDGE_FACTOR )
    {
        // printf( "\t\tadding distant Vertex\n" );
        int n_new_vertex, count = 1;
        double t;
        geometry_msgs::msg::Point new_point, base_point;
        vertex_data_t pre;
        previous = current;
        this->get_vertex( previous, pre );
        base_point   = pre.pos;
        new_point    = pre.pos;

        n_new_vertex = floor( dist_current / VERTEX_DISTANCE );
        t            = ( VERTEX_DISTANCE ) / dist_current;
        // printf( "\t\t\tCreate support nodes\nn: %i|t: %6.1f\n", n_new_vertex, t );
        while( dist_current > VERTEX_DISTANCE && n_new_vertex > 0 )
        {
            new_point.x = ( 1 - t * count ) * base_point.x + t * count * pos.x;
            new_point.y = ( 1 - t * count ) * base_point.y + t * count * pos.y;

            int idx     = this->_add_vertex( this->v_index++, new_point, strong_vertex );
            current     = this->graph[ idx ].index;

            this->add_edge( previous, current );

            previous = current;
            this->get_vertex( previous, pre );
            dist_current = this->_calc_distance( pre.pos, pos );
            n_new_vertex--;
            count++;
        }
        return;
    }

    // printf( "\t\tadding close Vertex\n" );
    // this->print_vertex( std::string( "\t\t\tCurrent: " ), current );
    // this->print_vertex( std::string( "\t\t\tPrevious: " ), previous );
    // this->print_vertex( std::string( "\t\t\tClosest: " ), closest );
    idx      = this->_add_vertex( this->v_index++, pos, strong_vertex );
    previous = current;
    current  = this->graph[ idx ].index;
    // this->print_vertex( std::string( "\t\t\t/Current: " ), current );
    // this->print_vertex( std::string( "\t\t\t/Previous: " ), previous );
    // this->print_vertex( std::string( "\t\t\t/Closest: " ), closest );

    // printf( "\t\tadding Edge\n" );
    // this->print_vertex( std::string( "\t\t\tCurrent: " ), current );
    // this->print_vertex( std::string( "\t\t\tPrevious: " ), previous );
    // this->print_vertex( std::string( "\t\t\tClosest: " ), closest );
    this->add_edge( previous, current );
    // this->print_vertex( std::string( "\t\t\t/Current: " ), current );
    // this->print_vertex( std::string( "\t\t\t/Previous: " ), previous );
    // this->print_vertex( std::string( "\t\t\t/Closest: " ), closest );
    // this->export_graph();
}

// void topo_map::add_object( const smap_interfaces::msg::SmapObject& object )
void topo_map::add_object( const smap_interfaces::msg::SmapObservation::SharedPtr observation, detector_t& det )
// void topo_map::add_object( const smap_interfaces::msg::SmapObject& object, double& angle ) // TODO: Revert
{
    // TODO: create callback group. Should be mutually exclusive
    printf( "add_object\n" );
    // if( boost::num_vertices( this->graph ) == 0 ) return;

    size_t current = -1, previous = -1;
    double distance, t;
    // previous will be the closest point in this case
    previous = this->get_closest_vertex( observation->object.pose.pose.position, distance );
    vertex_data_t pre;
    this->get_vertex( previous, pre );

    if( distance < 2 * VERTEX_DISTANCE )
    {
        // TODO: Implement method
        printf( "append_object\n" );
        thing new_thing( &( this->reg_classes ) );
        // probabilities
        // *this->reg_detectors[]
        // TODO: Get indexes based on the detector idx key

        // Search for the detector related to the observation

        // printf( "-> Classes: \n" );
        // // for( auto cls: *this->reg_classes )
        // //     printf(
        // //         "\t[%2i] (%s) | (%s) [%2i,%2i]\n", cls.first, ( *this->reg_detectors )[ cls.second.second ],
        // //         cls.first, cls.second.first, cls.second.second );
        // for( auto c: det.classes )
        //     // (*this->reg_classes)[ c.second ]
        //     printf(
        //         "\t[%2i] (%s) | [%2i,%2i]\n", c.first, c.second.c_str(), ( *this->reg_classes )[ c.second ].first,
        //         ( *this->reg_classes )[ c.second ].second );

        new_thing.update(
            smap::semantic_type_t::OBJECT, observation->object, distance, observation->direction,
            det );  // TODO: Revert
        // pre.related_things.push_back( new_thing );
        // this->append_object();
        // printf( "append_object\n" );
        return;
    }

    int n_new_vertex, count = 1;
    geometry_msgs::msg::Point new_point, base_point;
    base_point   = pre.pos;
    new_point    = pre.pos;

    n_new_vertex = floor( distance / VERTEX_DISTANCE );
    t            = ( VERTEX_DISTANCE ) / distance;
    // printf( "Create support nodes\nn: %i|t: %6.1f\n", n_new_vertex, t );
    while( distance > VERTEX_DISTANCE && n_new_vertex > 0 )
    {
        new_point.x = ( 1 - t * count ) * base_point.x + t * count * observation->object.pose.pose.position.x;
        new_point.y = ( 1 - t * count ) * base_point.y + t * count * observation->object.pose.pose.position.y;

        int idx     = this->_add_vertex( this->v_index++, new_point, false );
        current     = this->graph[ idx ].index;

        this->add_edge( previous, current );

        previous = current;
        this->get_vertex( previous, pre );
        distance = this->_calc_distance( pre.pos, observation->object.pose.pose.position );
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
