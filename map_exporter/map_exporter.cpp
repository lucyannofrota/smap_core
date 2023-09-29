#include "include/map_exporter/map_exporter.hpp"

namespace smap
{

void map_exporter::draw_rectangle(
    std::vector< signed char >& map, const nav_msgs::msg::MapMetaData& metadata,
    const std::array< std::array< float, 2 >, 4 >& corners ) const
{
    int max_elements         = metadata.height * metadata.width;
    const auto outp_function = [ &map, &metadata, &max_elements ]( const unsigned int& x, const unsigned int& y ) {
        unsigned int index = ( (unsigned int) ( x + y * metadata.width ) );
        if( x >= metadata.width || x < 0 || y >= metadata.height || y < 0 ) return;
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

void map_exporter::mark_rectangle(
    std::vector< signed char >& map, const nav_msgs::msg::MapMetaData& metadata,
    std::array< std::array< float, 2 >, 4 > corners ) const
{
    // Transform points
    auto transform = [ metadata ]( std::array< float, 2 > point ) {
        point[ 0 ] = ( ( point[ 0 ] - metadata.origin.position.x ) / metadata.resolution );
        point[ 1 ] = ( ( point[ 1 ] - metadata.origin.position.y ) / metadata.resolution );
        return point;
    };
    for( int i = 0; i < 4; i++ ) corners[ i ] = transform( corners[ i ] );

    std::array< float, 2 > center = { 0, 0 };
    for( const auto& corner: corners )
    {
        center[ 0 ] += corner[ 0 ];
        center[ 1 ] += corner[ 1 ];
    }
    center[ 0 ] /= 4.0;
    center[ 1 ] /= 4.0;
    int n_draws  = 5 * ceil( 1.0 / metadata.resolution );
    std::array< std::array< float, 2 >, 4 > aux_corners;
    for( int i = 0; i < n_draws; i++ )
    {
        for( int j = 0; j < 4; j++ )
        {
            aux_corners[ j ][ 0 ] = corners[ j ][ 0 ] - ( ( corners[ j ][ 0 ] - center[ 0 ] ) / n_draws ) * i;
            aux_corners[ j ][ 1 ] = corners[ j ][ 1 ] - ( ( corners[ j ][ 1 ] - center[ 1 ] ) / n_draws ) * i;
        }
        this->draw_rectangle( map, metadata, aux_corners );
    }
}

void map_exporter::og_callback( const nav_msgs::msg::OccupancyGrid::SharedPtr og )
{
    // Copy Map
    std::vector< signed char > map, aux;
    map.resize( og->info.height * og->info.width );

    nav_msgs::msg::OccupancyGrid occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
    occupancy_grid_msg.header                       = og->header;
    occupancy_grid_msg.info                         = og->info;
    occupancy_grid_msg.data.resize( og->info.width * og->info.height );

    for( unsigned int y = 0; y < og->info.height; y++ )
    {
        for( unsigned int x = 0; x < og->info.width; x++ )
        {
            unsigned int i = x + ( og->info.height - y - 1 ) * og->info.width;
            map[ i ]       = og->data[ i ];
        }
    }

    const std::string path_name = std::string( "/workspace/src/smap/smap_core/maps/" );

    // Publish occupancy_grid
    std::string file_name = path_name + std::string( "og_map" );
    this->save_map( file_name, map, og->info, occupancy_grid_msg );
    og_pub->publish( occupancy_grid_msg );

    // Changing map
    for( unsigned int y = 0; y < og->info.height; y++ )
    {
        for( unsigned int x = 0; x < og->info.width; x++ )
        {
            unsigned int i = x + ( og->info.height - y - 1 ) * og->info.width;
            if( map[ i ] >= 25 )
            {                  // (occ,255] Object
                map[ i ] = 0;  // 100 // -1
            }
        }
    }

    // Publish occupancy_grid_center
    aux.assign( map.begin(), map.end() );  // Copy
    std::array< std::array< float, 2 >, 4 > corners;
    corners[ 0 ][ 0 ] = 1;
    corners[ 0 ][ 1 ] = 0;
    corners[ 1 ][ 0 ] = 3;
    corners[ 1 ][ 1 ] = 2;
    corners[ 2 ][ 0 ] = 5;
    corners[ 2 ][ 1 ] = 0;
    corners[ 3 ][ 0 ] = 3;
    corners[ 3 ][ 1 ] = -2;

    this->mark_rectangle( aux, og->info, corners );

    file_name = path_name + std::string( "og_center" );
    this->save_map( file_name, aux, og->info, occupancy_grid_msg );
    og_center_pub->publish( occupancy_grid_msg );
}

void map_exporter::save_map(
    const std::string& file_name, std::vector< signed char >& map, const nav_msgs::msg::MapMetaData& metadata,
    nav_msgs::msg::OccupancyGrid& occupancy_grid_msg )

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

}  // namespace smap

int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );

    std::shared_ptr< smap::map_exporter > node = std::make_shared< smap::map_exporter >();

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
