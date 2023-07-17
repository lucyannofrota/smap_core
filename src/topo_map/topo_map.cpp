#include "topo_map.hpp"

#include "../../include/smap_core/count_time.hpp"
#include "../object_estimator/include/depth_map.hpp"

// STL
#include <algorithm>
#include <array>

// BOOST
#include <boost/range/adaptor/indexed.hpp>

// #include "../pcl_processing/include/pcl_processing.hpp"

namespace smap
{

// inline double& depth_map_indexer(
//     std_msgs::msg::Float64MultiArray& occ_mat, const size_t& r, const size_t& c, const size_t& lims,
//     const size_t& comp )
// {
//     return occ_mat.data
//         [ occ_mat.layout.dim[ 0 ].stride * r + occ_mat.layout.dim[ 1 ].stride * c
//           + occ_mat.layout.dim[ 2 ].stride * lims + comp ];
// }

void topo_map::observation_callback( const smap_interfaces::msg::SmapObservation::SharedPtr observation )
{
    // TODO: observation subtractive behaviors
    if( this->reg_classes == nullptr || this->reg_detectors == nullptr ) return;
    count_time timer;
    RCLCPP_DEBUG( this->get_logger(), "observation_callback" );
    RCLCPP_DEBUG( this->get_logger(), "0. Check graph integrity" );
    if( boost::num_vertices( this->graph ) == 0 ) return;

    // 1. Get all adjacent vertexes 3 layers deep (In relation to the object position)
    std::vector< size_t > valid_idxs;
    this->get_adjacent_vertices( valid_idxs, observation->object.pose.pose.position, 3 );

    // 2. Filter possible vertexes
    std::vector< std::pair< size_t, std::vector< thing* > > > candidates;
    this->filter_vertices(
        valid_idxs, candidates, observation->object.module_id, observation->object.label,
        observation->object.pose.pose.position, observation->robot_pose.pose );

    // 3. Verify the existence of the detector
    std::vector< detector_t >::iterator det;
    if( !this->is_detector_valid( observation->object.module_id, det ) )  // return if no detector is found
    {
        RCLCPP_WARN( this->get_logger(), "No detector was found!" );
        return;
    }
    // 4. Update vertex
    // If true add object otherwise update an existing one
    std::pair< size_t, thing* > closest( 0, nullptr ), closest_valid( 0, nullptr );
    // thing *closest = nullptr, *closest_valid = nullptr;
    // size_t vert_idx = 0, vert_idx_valid = 0;
    this->vertex_transaction( observation, candidates, det, closest );

    // Object combination
    // this->object_combination( candidates, closest, closest_valid );

    // If necessary, move the object to another vertex
    this->object_vert_move( valid_idxs, closest );

    const char str[] = "observation_callback";
    timer.print_time( this->get_logger(), str );
}

void topo_map::depth_map_callback( const smap_interfaces::msg::DepthMap::SharedPtr msg )
{
    if( this->reg_classes == nullptr || this->reg_detectors == nullptr || boost::num_vertices( this->graph ) == 0 )
        return;
    RCLCPP_DEBUG( this->get_logger(), "Occlusion_callback" );
    static depth_map_t depth_map;
    from_msg( *msg, depth_map );

    // 1. Get all adjacent vertexes 5 layers deep
    RCLCPP_DEBUG( this->get_logger(), "1. Get all adjacent vertexes 5 layers deep" );
    std::vector< size_t > valid_idxs;
    this->get_adjacent_vertices( valid_idxs, msg->camera_pose.position, 5 );
    if( valid_idxs.empty() ) return;

    // 2. Filter possible vertexes
    RCLCPP_DEBUG( this->get_logger(), "2. Filter possible vertexes" );
    // std::vector< thing* > candidates;
    for( const auto& element: valid_idxs )
    {
        for( auto& object: this->graph[ element ].related_things )
        {
            // 2.1. Check active cone
            RCLCPP_DEBUG( this->get_logger(), "2.1. Check active cone" );
            // printf(
            //     "1 : %f|2 : %f\n", rad2deg( this->compute_object_direction( object.pos, msg->camera_pose ) ),
            //     rad2deg( this->compute_corner_direction( msg->camera_pose, object.pos ) ) );
            if( abs( rad2deg( this->compute_object_direction( object.pos, msg->camera_pose ) ) ) > ACTIVE_FOV_H )
                continue;

            // 2.2. Check cell occlusion
            RCLCPP_DEBUG( this->get_logger(), "2.2. Check occlusion" );
            // 2.2.1. Get AABB corners
            std::array< geometry_msgs::msg::Point, 8 > AABB;
            set_AABB( AABB, object.aabb.first, object.aabb.second );

            // 2.2.2. Get correspondent AABB corners in depth_map indexes
            std::array< std::pair< std::array< long, 2 >, double >, 8 > object_corner_indexes = {
                {{ { { std::numeric_limits< long >::max(), std::numeric_limits< long >::min() } },
std::numeric_limits< double >::infinity() },
                 { { { std::numeric_limits< long >::max(), std::numeric_limits< long >::min() } },
                 std::numeric_limits< double >::infinity() },
                 { { { std::numeric_limits< long >::max(), std::numeric_limits< long >::min() } },
                 std::numeric_limits< double >::infinity() },
                 { { { std::numeric_limits< long >::max(), std::numeric_limits< long >::min() } },
                 std::numeric_limits< double >::infinity() },
                 { { { std::numeric_limits< long >::max(), std::numeric_limits< long >::min() } },
                 std::numeric_limits< double >::infinity() },
                 { { { std::numeric_limits< long >::max(), std::numeric_limits< long >::min() } },
                 std::numeric_limits< double >::infinity() },
                 { { { std::numeric_limits< long >::max(), std::numeric_limits< long >::min() } },
                 std::numeric_limits< double >::infinity() },
                 { { { std::numeric_limits< long >::max(), std::numeric_limits< long >::min() } },
                 std::numeric_limits< double >::infinity() }}
            };
            for( const auto& corner: AABB | boost::adaptors::indexed( 0 ) )
            {
                // 2.2.2.1 Compute the angle from the camera to the corner
                double camera_to_corner_angle = this->compute_corner_direction( msg->camera_pose, corner.value() );
                // 2.2.2.2 Loop through all cells of the depth_map
                for( const auto& o_map_row: depth_map | boost::adaptors::indexed( 0 ) )
                {
                    for( const auto& o_map_col: o_map_row.value() | boost::adaptors::indexed( 0 ) )
                    {
                        if( !is_valid( o_map_col.value()[ 2 ] ) ) continue;
                        // 2.2.2.2.1 Compute the angle from the camera to the depth_map cell
                        double camera_to_panel_angle =
                            this->compute_corner_direction( msg->camera_pose, o_map_col.value()[ 2 ] );
                        // 2.2.2.2.2 Compute the difference between camera_to_corner_angle and camera_to_panel_angle
                        double angle_difference = atan2(
                            sin( camera_to_corner_angle - camera_to_panel_angle ),
                            cos( camera_to_corner_angle - camera_to_panel_angle ) );
                        if( ( abs( angle_difference ) < object_corner_indexes[ corner.index() ].second )
                            && ( ( object.aabb.first.z < o_map_col.value()[ 2 ].z )
                                 && ( object.aabb.second.z > o_map_col.value()[ 2 ].z ) ) )
                        {
                            object_corner_indexes[ corner.index() ].first  = { o_map_row.index(), o_map_col.index() };
                            object_corner_indexes[ corner.index() ].second = abs( angle_difference );
                        }
                    }
                }
            }

            // 2.2.3.1 Get the minimum and maximum corners in the depth_map plane
            std::array< long, 2 > mins = { DEPTH_MAP_ROWS, DEPTH_MAP_COLS }, maxs = { 0, 0 };
            for( const auto& cell: object_corner_indexes )
            {
                // mins
                if( cell.first[ 0 ] < mins[ 0 ] ) mins[ 0 ] = cell.first[ 0 ];
                if( cell.first[ 1 ] < mins[ 1 ] ) mins[ 1 ] = cell.first[ 1 ];

                // maxs
                if( cell.first[ 0 ] > maxs[ 0 ] ) maxs[ 0 ] = cell.first[ 0 ];
                if( cell.first[ 1 ] > maxs[ 1 ] ) maxs[ 1 ] = cell.first[ 1 ];
            }

            if( this->selected_panels_pub->get_subscription_count() > 0 )
            {
                this->panels_maker.id = 0;
                this->panels_maker_array.markers.clear();
                for( long occ_row = mins[ 0 ]; occ_row < maxs[ 0 ]; occ_row++ )
                {
                    for( long occ_col = mins[ 1 ]; occ_col < maxs[ 1 ]; occ_col++ )
                    {
                        if( !is_valid( depth_map[ occ_row ][ occ_col ][ 0 ] )
                            || !is_valid( depth_map[ occ_row ][ occ_col ][ 1 ] )
                            || !is_valid( depth_map[ occ_row ][ occ_col ][ 2 ] ) )
                            continue;
                        this->panels_maker.pose.position = depth_map[ occ_row ][ occ_col ][ 2 ];
                        this->panels_maker.scale.x =
                            abs( depth_map[ occ_row ][ occ_col ][ 1 ].x - depth_map[ occ_row ][ occ_col ][ 0 ].x );
                        this->panels_maker.scale.y =
                            abs( depth_map[ occ_row ][ occ_col ][ 1 ].y - depth_map[ occ_row ][ occ_col ][ 0 ].y );
                        this->panels_maker.scale.z =
                            abs( depth_map[ occ_row ][ occ_col ][ 1 ].z - depth_map[ occ_row ][ occ_col ][ 0 ].z );
                        this->panels_maker_array.markers.push_back( this->panels_maker );
                        this->panels_maker.id++;
                    }
                }
                this->selected_panels_pub->publish( this->panels_maker_array );
            }

            // 2.2.3.2 Compute the number of cells in occlusion
            size_t cells_before = 0, cells_in = 0, cells_after = 0;
            double distance_camera_to_object = gPoint_distance( msg->camera_pose.position, object.pos );
            for( long occ_row = mins[ 0 ]; occ_row < maxs[ 0 ]; occ_row++ )
            {
                for( long occ_col = mins[ 1 ]; occ_col < maxs[ 1 ]; occ_col++ )
                {
                    if( !is_valid( depth_map[ occ_row ][ occ_col ][ 0 ] )
                        || !is_valid( depth_map[ occ_row ][ occ_col ][ 1 ] )
                        || !is_valid( depth_map[ occ_row ][ occ_col ][ 2 ] ) )
                        continue;
                    // 2.2.3.2.1 Compute the the distance between cell centroids and the object centroid with tolerance
                    // distance_cell_to_object =
                    // gPoint_distance( depth_map[ occ_row ][ occ_col ][ 2 ], object.pos )
                    double distance_camera_to_cell =
                               gPoint_distance( msg->camera_pose.position, depth_map[ occ_row ][ occ_col ][ 2 ] ),
                           object_tolerance_radius = ( gPoint_distance( object.aabb.second, object.aabb.first ) / 2 )
                                                   * OCCLUSION_OBJECT_DISTANCE_TOLERANCE_FACTOR;
                    if( object_tolerance_radius > OCCLUSION_OBJECT_DISTANCE_TOLERANCE_MAX )
                        object_tolerance_radius = OCCLUSION_OBJECT_DISTANCE_TOLERANCE_MAX;
                    // 2.2.3.2.2-1 cell before obj
                    if( distance_camera_to_cell < distance_camera_to_object - object_tolerance_radius )
                    {
                        cells_before++;
                        continue;
                    }
                    // 2.2.3.2.2-2 cell close to obj
                    if( ( distance_camera_to_cell >= distance_camera_to_object - object_tolerance_radius )
                        && ( distance_camera_to_cell <= distance_camera_to_object + object_tolerance_radius ) )
                    {
                        cells_in++;
                        continue;
                    }
                    // 2.2.3.2.2-3 cell after obj
                    if( distance_camera_to_cell > distance_camera_to_object + object_tolerance_radius )
                    {
                        cells_after++;
                        continue;
                    }
                }
            }

            // 2.3. Classify the object as occluded or non-occluded and update
            size_t cells_total = cells_before + cells_in + cells_after;
            if( cells_total == 0 ) return;
            // 2.3.1 Check for occluded objects
            if( ( cells_before / ( cells_total * 1.0 ) ) > OCCLUSION_MAX_PERCENTAGE )
            {
                // 2.3.1-1 Object occluded
                // Decay the histogram probabilities
                // TODO: Consider a small decay

                object.observations->register_obs(
                    distance_camera_to_object, this->compute_corner_direction( msg->camera_pose, object.pos ), false );
                continue;
            }
            // 2.3.1-2 Object not occluded
            // 2.3.2 Reduce the object confidence in proportion to (cells_before + cells_in + cells_after)
            object.decay_confidence(
                distance_camera_to_object, ( cells_before + cells_after ) / ( cells_total * 1.0 ) );
            // this->observations.register_obs( distance, angle, false );
        }
    }

    // TODO: Use the occlusion map

    // for( size_t r = 0; r < msg->layout.dim[ 0 ].size; r++ )
    // {
    //     for( size_t c = 0; c < msg->layout.dim[ 1 ].size; c++ )
    //     {
    //         for( size_t lims = 0; lims < msg->layout.dim[ 2 ].size; lims++ )
    //         {
    //             printf( "[" );
    //             for( size_t comps = 0; comps < msg->layout.dim[ 3 ].size; comps++ )
    //                 printf( "%6.2f,", depth_map_indexer( *msg, r, c, lims, comps ) );
    //             printf( "]|" );
    //         }
    //         printf( "]\t" );
    //     }
    //     printf( "\n" );
    // }
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
    min            = std::numeric_limits< double >::max();
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

        // Consider strong_vertex where the robot has already passed
        this->graph[ _get_vertex( closest ) ].strong_vertex = true;

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
    thing new_thing( this->reg_classes, ++this->thing_id_count );
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
