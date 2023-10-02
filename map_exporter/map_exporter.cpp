#include "include/map_exporter/map_exporter.hpp"

// SMAP
// #include "topo_map/topo_map.hpp"

// #include "topo_map/thing.hpp"

namespace smap
{

void map_exporter::export_map( graph_t& graph, double confidence_threshold )
{
    if( this->reg_classes == nullptr || this->map.empty() ) return;
    printf( "MAP_EXP B\n" );
    std::vector< signed char > aux;
    {
        // Copy map
        std::lock_guard< std::mutex > lk( this->m_gmap );
        aux.assign( map.begin(), map.end() );  // Copy
    }
    // for( const auto& class_register: *this->reg_classes )
    // {
    //     //
    //     //

    // }
    printf( "MAP_EXP C\n" );
    std::vector< signed char > g_smap;
    bool map_used = true;
    for( const auto& class_registered: *this->reg_classes )
    {
        if( map_used )
        {
            g_smap.assign( aux.begin(), aux.end() );  // Copy
            map_used = false;
        }
        for( const auto& e: boost::make_iterator_range( boost::vertices( graph ) ) )
        {
            for( const auto& r_thing: graph[ e ].related_things )
            {
                if( !r_thing.is_valid( confidence_threshold ) ) continue;
                if( class_registered.first != r_thing.get_label().first ) continue;
                map_used = true;

                // Get corners
                std::array< std::array< double, 2 >, 4 > corners;
                corners[ 0 ] = { r_thing.aabb.first.x, r_thing.aabb.first.y };
                corners[ 1 ] = { r_thing.aabb.first.x, r_thing.aabb.second.y };
                corners[ 2 ] = { r_thing.aabb.second.x, r_thing.aabb.second.y };
                corners[ 3 ] = { r_thing.aabb.second.x, r_thing.aabb.first.y };

                this->mark_rectangle( g_smap, corners );
            }
        }
        if( map_used )
        {
            std::string file_name = path_name + class_registered.first;
            RCLCPP_WARN( this->get_logger(), "Saving: %s", file_name );
            this->save_map( file_name, g_smap );
        }
    }
    printf( "MAP_EXP E\n" );
}

void map_exporter::draw_rectangle(
    std::vector< signed char >& map, const std::array< std::array< double, 2 >, 4 >& corners )
{
    int max_elements         = metadata.height * metadata.width;
    const auto outp_function = [ &map, this, &max_elements ]( const unsigned int& x, const unsigned int& y ) {
        unsigned int index = ( (unsigned int) ( x + y * this->metadata.width ) );
        if( x >= this->metadata.width || y >= this->metadata.height ) return;
        map[ index ] = 100;
    };
    bresenham(
        outp_function, (unsigned int) corners[ 0 ][ 0 ], (unsigned int) corners[ 0 ][ 1 ],
        (unsigned int) corners[ 1 ][ 0 ], (unsigned int) corners[ 1 ][ 1 ] );
    bresenham(
        outp_function, (unsigned int) corners[ 1 ][ 0 ], (unsigned int) corners[ 1 ][ 1 ],
        (unsigned int) corners[ 2 ][ 0 ], (unsigned int) corners[ 2 ][ 1 ] );
    bresenham(
        outp_function, (unsigned int) corners[ 2 ][ 0 ], (unsigned int) corners[ 2 ][ 1 ],
        (unsigned int) corners[ 3 ][ 0 ], (unsigned int) corners[ 3 ][ 1 ] );
    bresenham(
        outp_function, (unsigned int) corners[ 3 ][ 0 ], (unsigned int) corners[ 3 ][ 1 ],
        (unsigned int) corners[ 0 ][ 0 ], (unsigned int) corners[ 0 ][ 1 ] );
}

void map_exporter::mark_rectangle( std::vector< signed char >& map, std::array< std::array< double, 2 >, 4 > corners )
{
    // Transform points
    auto transform = [ this ]( std::array< double, 2 > point ) {
        point[ 0 ] = ( ( point[ 0 ] - this->metadata.origin.position.x ) / this->metadata.resolution );
        point[ 1 ] = ( ( point[ 1 ] - this->metadata.origin.position.y ) / this->metadata.resolution );
        return point;
    };
    for( int i = 0; i < 4; i++ ) corners[ i ] = transform( corners[ i ] );

    std::array< double, 2 > center = { 0, 0 };
    for( const auto& corner: corners )
    {
        center[ 0 ] += corner[ 0 ];
        center[ 1 ] += corner[ 1 ];
    }
    center[ 0 ] /= 4.0;
    center[ 1 ] /= 4.0;
    int n_draws  = 5 * ceil( 1.0 / metadata.resolution );
    std::array< std::array< double, 2 >, 4 > aux_corners;
    for( int i = 0; i < n_draws; i++ )
    {
        for( int j = 0; j < 4; j++ )
        {
            aux_corners[ j ][ 0 ] = corners[ j ][ 0 ] - ( ( corners[ j ][ 0 ] - center[ 0 ] ) / n_draws ) * i;
            aux_corners[ j ][ 1 ] = corners[ j ][ 1 ] - ( ( corners[ j ][ 1 ] - center[ 1 ] ) / n_draws ) * i;
        }
        this->draw_rectangle( map, aux_corners );
    }
}

void map_exporter::og_callback( const nav_msgs::msg::OccupancyGrid::SharedPtr og )
{
    // if( this->reg_classes == nullptr ) return;
    {
        std::lock_guard< std::mutex > lk( this->m_gmap );
        // Copy Map
        map.resize( og->info.height * og->info.width );
        this->metadata = og->info;

        // nav_msgs::msg::OccupancyGrid occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
        // occupancy_grid_msg.header                       = og->header;
        // occupancy_grid_msg.info                         = og->info;
        // occupancy_grid_msg.data.resize( og->info.width * og->info.height );

        for( unsigned int y = 0; y < og->info.height; y++ )
        {
            for( unsigned int x = 0; x < og->info.width; x++ )
            {
                unsigned int i = x + ( og->info.height - y - 1 ) * og->info.width;
                this->map[ i ] = og->data[ i ];
            }
        }
        // og_pub->publish( occupancy_grid_msg );

        // Changing map
        for( unsigned int y = 0; y < og->info.height; y++ )
        {
            for( unsigned int x = 0; x < og->info.width; x++ )
            {
                unsigned int i = x + ( og->info.height - y - 1 ) * og->info.width;
                if( this->map[ i ] >= 25 )
                {                        // (occ,255] Object
                    this->map[ i ] = 0;  // 100 // -1
                }
            }
        }

        // Publish occupancy_grid
        std::string file_name = path_name + std::string( "og_map" );
        this->save_map( file_name, this->map );

        // Publish occupancy_grid_center
    }
    // std::array< std::array< double, 2 >, 4 > corners;
    // corners[ 0 ][ 0 ] = 1;
    // corners[ 0 ][ 1 ] = 0;
    // corners[ 1 ][ 0 ] = 3;
    // corners[ 1 ][ 1 ] = 2;
    // corners[ 2 ][ 0 ] = 5;
    // corners[ 2 ][ 1 ] = 0;
    // corners[ 3 ][ 0 ] = 3;
    // corners[ 3 ][ 1 ] = -2;

    // this->mark_rectangle( aux, og->info, corners );

    // file_name = path_name + std::string( "og_center" );
    // this->save_map( file_name, aux, og->info, occupancy_grid_msg );
    // og_center_pub->publish( occupancy_grid_msg );
}

void map_exporter::save_map(
    const std::string& file_name, std::vector< signed char >& map, nav_msgs::msg::OccupancyGrid& occupancy_grid_msg )

{
    // https://github.com/ros-planning/navigation/blob/noetic-devel/map_server/src/map_saver.cpp
    // Map
    std::string _file_name = file_name + std::string( ".pgm" );
    FILE* output           = fopen( _file_name.c_str(), "w" );
    if( !output )
    {
        RCLCPP_ERROR( this->get_logger(), "Couldn't save map [%s]", _file_name.c_str() );
        return;
    }
    fprintf(
        output, "P5\n# CREATOR: map_exporter.cpp %.3f m/pix\n%d %d\n255\n", metadata.resolution, metadata.width,
        metadata.height );
    for( unsigned int y = 0; y < metadata.height; y++ )
    {
        for( unsigned int x = 0; x < metadata.width; x++ )
        {
            unsigned int i = x + ( metadata.height - y - 1 ) * metadata.width;
            if( map[ i ] >= 0 && map[ i ] <= 25 )
            {  // Unknown
                fputc( 254, output );
                occupancy_grid_msg.data[ i ] = map[ i ];
            }
            else if( map[ i ] >= 25 )
            {  // (occ,255] Object
                fputc( 000, output );
                occupancy_grid_msg.data[ i ] = map[ i ];
            }
            else
            {  // occ [0.25,0.65] Free
                fputc( 205, output );
                occupancy_grid_msg.data[ i ] = map[ i ];
            }
        }
    }
    fclose( output );

    // Metadata
    _file_name = file_name + std::string( ".yaml" );
    output     = fopen( _file_name.c_str(), "w" );
    if( !output )
    {
        RCLCPP_ERROR( this->get_logger(), "Couldn't save map metadata [%s]", _file_name.c_str() );
        return;
    }
    geometry_msgs::msg::Quaternion orientation = metadata.origin.orientation;
    tf2::Matrix3x3 mat( tf2::Quaternion( orientation.x, orientation.y, orientation.z, orientation.w ) );
    double yaw, pitch, roll;
    mat.getEulerYPR( yaw, pitch, roll );

    fprintf(
        output,
        "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
        _file_name.c_str(), metadata.resolution, metadata.origin.position.x, metadata.origin.position.y, yaw );

    fclose( output );
}

void map_exporter::save_map( const std::string& file_name, std::vector< signed char >& map )

{
    // https://github.com/ros-planning/navigation/blob/noetic-devel/map_server/src/map_saver.cpp
    // Map
    std::string _file_name = file_name + std::string( ".pgm" );
    FILE* output           = fopen( _file_name.c_str(), "w" );
    if( !output )
    {
        RCLCPP_ERROR( this->get_logger(), "Couldn't save map [%s]", _file_name.c_str() );
        return;
    }
    fprintf(
        output, "P5\n# CREATOR: map_exporter.cpp %.3f m/pix\n%d %d\n255\n", metadata.resolution, metadata.width,
        metadata.height );
    for( unsigned int y = 0; y < metadata.height; y++ )
    {
        for( unsigned int x = 0; x < metadata.width; x++ )
        {
            unsigned int i = x + ( metadata.height - y - 1 ) * metadata.width;
            if( map[ i ] >= 0 && map[ i ] <= 25 )
            {  // Unknown
                fputc( 254, output );
            }
            else if( map[ i ] >= 25 )
            {  // (occ,255] Object
                fputc( 000, output );
            }
            else
            {  // occ [0.25,0.65] Free
                fputc( 205, output );
            }
        }
    }
    fclose( output );

    // Metadata
    _file_name = file_name + std::string( ".yaml" );
    output     = fopen( _file_name.c_str(), "w" );
    if( !output )
    {
        RCLCPP_ERROR( this->get_logger(), "Couldn't save map metadata [%s]", _file_name.c_str() );
        return;
    }
    geometry_msgs::msg::Quaternion orientation = metadata.origin.orientation;
    tf2::Matrix3x3 mat( tf2::Quaternion( orientation.x, orientation.y, orientation.z, orientation.w ) );
    double yaw, pitch, roll;
    mat.getEulerYPR( yaw, pitch, roll );

    fprintf(
        output,
        "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
        _file_name.c_str(), metadata.resolution, metadata.origin.position.x, metadata.origin.position.y, yaw );

    fclose( output );
}

}  // namespace smap

// int main( int argc, char** argv )
// {
//     rclcpp::init( argc, argv );

// std::shared_ptr< smap::map_exporter > node = std::make_shared< smap::map_exporter >();

// try
// {
//     rclcpp::spin( node );
// }
// catch( std::exception& e )
// {
//     std::cout << "SMAP Exception!" << std::endl;
//     std::cout << e.what() << std::endl;
// }
// rclcpp::shutdown();

// return 0;
// }
