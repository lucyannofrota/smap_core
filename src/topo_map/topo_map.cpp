#include "topo_map.hpp"

namespace smap
{

void topo_map::observation_callback( const smap_interfaces::msg::SmapObservation::SharedPtr observation )
{
    // TODO: observation subtractive behaviors
    // TODO: Migrate object between nodes
    RCLCPP_DEBUG( this->get_logger(), "observation_callback" );
    RCLCPP_DEBUG( this->get_logger(), "0. Check graph integrity" );
    if( boost::num_vertices( this->graph ) == 0 || this->reg_classes == nullptr || this->reg_detectors == nullptr )
    {
        RCLCPP_WARN( this->get_logger(), "No detectors registered." );
        return;
    }

    // 1. Get all adjacent vertexes 3 layers deep
    RCLCPP_DEBUG( this->get_logger(), "Get all adjacent vertexes 3 layers deep" );
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
        idxs_checking = idxs_to_check;
    }
    RCLCPP_DEBUG( this->get_logger(), "1.1 idxs_checked size: %i", (int) idxs_checked.size() );
    // 2. Filter possible vertexes

    RCLCPP_DEBUG( this->get_logger(), "2. Filter possible vertexes" );
    std::vector< std::pair< size_t, std::vector< thing* > > > candidates;
    RCLCPP_DEBUG( this->get_logger(), "2.1.1 idxs_checked.size(): %i", (int) idxs_checked.size() );
    std::vector< thing* > local_candidates;
    for( auto checking: idxs_checked )
    {
        // Check list of objects inside each vertex
        local_candidates.clear();
        bool has_candidate = false;

        RCLCPP_DEBUG(
            this->get_logger(), "2.1.2 this->graph[ *checking ].related_things.size(): %i",
            (int) this->graph[ checking ].related_things.size() );
        for( auto& obj: this->graph[ checking ].related_things )
        {
            // Check labels
            if( !obj.label_is_equal( observation->object.module_id, observation->object.label ) )
            {
                RCLCPP_DEBUG( this->get_logger(), "2.2.1 Check label: FAIL" );
                continue;
            }
            else RCLCPP_DEBUG( this->get_logger(), "2.2.1 Check label: PASS" );

            // Check active cone
            if( abs( rad2deg(
                    this->compute_direction( observation->object.pose.pose.position, observation->robot_pose.pose ) ) )
                > ACTIVE_FOV_H )
            {

                RCLCPP_DEBUG(
                    this->get_logger(), "2.2.2 Check active cone [val: %7.2f|rad2deg: %7.2f|abs: %7.2f]: FAIL",
                    this->compute_direction( observation->object.pose.pose.position, observation->robot_pose.pose ),
                    rad2deg( this->compute_direction(
                        observation->object.pose.pose.position, observation->robot_pose.pose ) ),
                    abs( rad2deg( this->compute_direction(
                        observation->object.pose.pose.position, observation->robot_pose.pose ) ) ) );
                // RCLCPP_DEBUG(
                //     this->get_logger(), "2.2 Check active cone [val: %7.2f|rad2deg: %7.2f|abs: %7.2f]: FAIL",
                //     this->compute_direction( observation->object.pose.pose.position, observation->robot_pose.pose
                //     ), rad2deg( this->compute_direction(
                //         observation->object.pose.pose.position, observation->robot_pose.pose ) ),
                //     abs( rad2deg( this->compute_direction(
                //         observation->object.pose.pose.position, observation->robot_pose.pose ) ) ) );
                continue;
            }
            else RCLCPP_DEBUG( this->get_logger(), "2.2.2 Check active cone: PASS" );

            // Check position
            if( !( ( ( observation->object.pose.pose.position.x
                       > observation->object.aabb.min.point.x - OBJECT_TRACKING_TOLERANCE )
                     && ( observation->object.pose.pose.position.x
                          < observation->object.aabb.max.point.x + OBJECT_TRACKING_TOLERANCE ) )
                   && ( ( observation->object.pose.pose.position.y
                          > observation->object.aabb.min.point.y - OBJECT_TRACKING_TOLERANCE )
                        && ( observation->object.pose.pose.position.y
                             < observation->object.aabb.max.point.y + OBJECT_TRACKING_TOLERANCE ) )
                   && ( ( observation->object.pose.pose.position.z
                          > observation->object.aabb.min.point.z - OBJECT_TRACKING_TOLERANCE )
                        && ( observation->object.pose.pose.position.z
                             < observation->object.aabb.max.point.z + OBJECT_TRACKING_TOLERANCE ) ) )
                && ( this->_calc_distance( observation->object.pose.pose.position, obj.pos ) > OBJECT_ERROR_DISTANCE ) )
            {
                RCLCPP_DEBUG( this->get_logger(), "2.3 Check position: FAIL" );
                continue;
            }
            else RCLCPP_DEBUG( this->get_logger(), "2.3 Check position: PASS" );

            local_candidates.push_back( &obj );
            has_candidate = true;
        }
        // Update candidates
        RCLCPP_DEBUG( this->get_logger(), "2.2.4 Update candidates" );
        if( has_candidate )
            candidates.push_back( std::pair< size_t, std::vector< thing* > >( checking, local_candidates ) );
    }
    // RCLCPP_DEBUG( this->get_logger(), "2| candidates size: %i", (int) candidates.size() );
    RCLCPP_DEBUG( this->get_logger(), "2.3 candidates size: %i", (int) candidates.size() );

    // 3. Verify the existence of the detector
    auto det       = this->reg_detectors->begin();
    bool det_found = false;
    for( ; det != this->reg_detectors->end(); det++ )
    {
        if( det->id == observation->object.module_id )
        {
            det_found = true;
            break;
        }
    }
    if( !det_found )  // return if no detector is found
    {
        RCLCPP_WARN( this->get_logger(), "No detector was found!" );
        return;
    }

    // 4. Update vertex
    // If true add object otherwise update an existing one
    // geometry_msgs::msg::Point o_pos;
    thing *closest = nullptr, *closest_valid = nullptr;
    size_t vert_idx = 0, vert_idx_valid = 0;
    RCLCPP_DEBUG( this->get_logger(), "3. Update vertex" );
    if( candidates.size() == 0 )
    {
        RCLCPP_DEBUG( this->get_logger(), "3.1.1 Object add" );
        closest = &( this->add_object( observation, *det ) );
    }
    else
    {
        // Select the closest object
        RCLCPP_DEBUG( this->get_logger(), "3.1.2.1 Select the closest object" );

        // for( auto c: candidates )
        // {
        //     //
        //     //
        //     // c.
        // }

        for( auto c: candidates )
        {
            for( auto lc: c.second )
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
                    vert_idx     = c.first;
                    min_distance = this->_calc_distance( observation->object.pose.pose.position, lc->pos );
                }
            }
        }
        RCLCPP_DEBUG( this->get_logger(), "Object update" );
        closest->update(
            observation->object.probability_distribution, observation->object.pose.pose.position,
            std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point >(
                observation->object.aabb.min.point, observation->object.aabb.max.point ),
            observation->object.aabb.confidence, min_distance, (double) observation->direction, *det );
    }

    // Object combination
    double min_distance_valid = DBL_MAX;
    for( auto c: candidates )
    {
        for( auto lc: c.second )
        {
            if( ( this->_calc_distance( closest->pos, lc->pos ) < min_distance_valid ) && ( lc->id != closest->id ) )
            {
                closest_valid      = lc;
                vert_idx_valid     = c.first;
                min_distance_valid = this->_calc_distance( closest->pos, lc->pos );
            }
        }
    }
    if( ( min_distance_valid < OBJECT_ERROR_DISTANCE * 0.8 ) && closest_valid->id != closest->id )
    {
        int pred_obj_idx = ( closest->id < closest_valid->id ? closest->id : closest_valid->id );
        if( vert_idx == vert_idx_valid )
        {
            // Same vertex
            this->graph[ vert_idx_valid ].related_things.remove_if(
                [ &closest_valid ]( thing& th ) { return th.id == closest_valid->id; } );
            closest->id = pred_obj_idx;
        }
        else
        {
            // Different vertex
            for( auto& th: this->graph[ vert_idx_valid ].related_things )
                if( th.id == closest_valid->id )
                {
                    th    = *closest;
                    th.id = pred_obj_idx;
                    this->graph[ vert_idx ].related_things.remove_if(
                        [ &closest ]( thing& th ) { return th.id == closest->id; } );
                    closest  = &th;
                    vert_idx = vert_idx_valid;
                    break;
                }
        }
    }

    // If necessary, move the object to another vertex
    std::vector< std::pair< size_t, double > > distances;
    // min_distance = this->_calc_distance( closest->pos, this->graph[ vert_idx ].pos );
    std::pair< size_t, double > min_vertex = std::pair< size_t, double >( vert_idx, DBL_MAX );
    for( auto& c: candidates )
    {
        if( this->_calc_distance( closest->pos, this->graph[ c.first ].pos ) < min_vertex.second )
        {
            min_vertex.first  = c.first;
            min_vertex.second = this->_calc_distance( closest->pos, this->graph[ c.first ].pos );
            // distances.push_back( std::pair< size_t, double >(
            // c.first, this->_calc_distance( closest->pos, this->graph[ c.first ].pos ) ) );
        }
    }
    if( min_vertex.second < this->_calc_distance( closest->pos, this->graph[ vert_idx ].pos ) )
    {
        this->graph[ min_vertex.first ].related_things.push_back( *closest );
        this->graph[ vert_idx ].related_things.remove_if( [ &closest ]( thing& th ) { return th.id == closest->id; } );
        // closest became invalid at this point!
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

    // Initialization
    if( current == (size_t) -1 )
    {
        idx     = this->_add_vertex( this->v_index++, pos, strong_vertex );
        current = this->graph[ idx ].index;
        return;
    }

    vertex_data_t cur, clo;
    size_t closest = this->get_closest_vertex( pos, dist_current );
    this->get_vertex( current, cur );
    this->get_vertex( closest, clo );

    if( ( previous != (size_t) -1 ) && ( closest != (size_t) -1 ) && ( closest != current )
        && ( dist_current <= VERTEX_DISTANCE * NEW_EDGE_FACTOR ) )
    {
        if( ( closest != previous ) ) this->add_edge( current, closest );

        previous = current;
        current  = closest;

        return;
    }

    // New vertex
    if( dist_current < VERTEX_DISTANCE * NEW_EDGE_FACTOR ) return;

    if( dist_current > VERTEX_DISTANCE * NEW_EDGE_FACTOR )
    {

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

    idx      = this->_add_vertex( this->v_index++, pos, strong_vertex );
    previous = current;
    current  = this->graph[ idx ].index;

    this->add_edge( previous, current );
}

thing& topo_map::add_object( const smap_interfaces::msg::SmapObservation::SharedPtr observation, detector_t& det )
{
    // TODO: create callback group. Should be mutually exclusive
    RCLCPP_DEBUG( this->get_logger(), "add_object" );

    // if( boost::num_vertices( this->graph ) == 0 ) return;

    size_t current = -1, previous = -1;
    double distance, t;
    // previous will be the closest point in this case
    previous = this->get_closest_vertex( observation->object.pose.pose.position, distance );
    vertex_data_t pre;
    this->get_vertex( previous, pre );

    if( distance > 2 * VERTEX_DISTANCE )
    {

        int n_new_vertex, count = 1;
        geometry_msgs::msg::Point new_point, base_point;
        base_point   = pre.pos;
        new_point    = pre.pos;

        n_new_vertex = floor( distance / VERTEX_DISTANCE );
        t            = ( VERTEX_DISTANCE ) / distance;
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

    // current = this->get_closest_vertex( observation->object.pose.pose.position, distance );

    RCLCPP_DEBUG( this->get_logger(), "append_object" );
    thing new_thing( &( this->reg_classes ) );
    // probabilities
    // *this->reg_detectors[]

    // Search for the detector related to the observation

    new_thing.set(
        smap::semantic_type_t::OBJECT, observation->object.probability_distribution,
        observation->object.pose.pose.position,
        std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point >(
            observation->object.aabb.min.point, observation->object.aabb.max.point ),
        observation->object.aabb.confidence, distance, observation->direction, det );

    this->graph[ _get_vertex( this->get_closest_vertex( observation->object.pose.pose.position, distance ) ) ]
        .related_things.push_back( new_thing );

    thing* th_ret = nullptr;
    for( auto& th:
         this->graph[ _get_vertex( this->get_closest_vertex( observation->object.pose.pose.position, distance ) ) ]
             .related_things )
    {
        if( th.id == new_thing.id )
        {
            th_ret = &th;
            break;
        }
    }
    return *th_ret;
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
