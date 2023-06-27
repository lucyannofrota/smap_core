#include "topo_map.hpp"

#include "../../include/smap_core/count_time.hpp"
#include "../object_estimator/include/occlusion_map.hpp"

// #include "../pcl_processing/include/pcl_processing.hpp"

namespace smap
{

// inline double& occlusion_map_indexer(
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
    count_time timer;
    RCLCPP_DEBUG( this->get_logger(), "observation_callback" );
    RCLCPP_DEBUG( this->get_logger(), "0. Check graph integrity" );
    if( boost::num_vertices( this->graph ) == 0 || this->reg_classes == nullptr || this->reg_detectors == nullptr )
    {
        RCLCPP_WARN( this->get_logger(), "No detectors registered." );
        return;
    }

    // 1. Get all adjacent vertexes 3 layers deep
    std::vector< size_t > valid_idxs;
    this->get_adjacent_vertices( valid_idxs, observation->object.pose.pose.position );

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
    timer.print_time( str );
}

void topo_map::occlusion_map_callback( const smap_interfaces::msg::OcclusionMap::SharedPtr msg )
{
    static occlusion_map_t occlusion_map;
    from_msg( *msg, occlusion_map );
    //
    //
    printf( "\n\nTOPO_MAP\n\n\n" );
    for( auto& row: occlusion_map )
    {
        for( auto& col: row )
        {
            for( auto& lim: col ) printf( "[%6.2f,%6.2f,%6.2f]|", lim.x, lim.y, lim.z );
            printf( "\t" );
        }
        printf( "\n" );
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
    //                 printf( "%6.2f,", occlusion_map_indexer( *msg, r, c, lims, comps ) );
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
