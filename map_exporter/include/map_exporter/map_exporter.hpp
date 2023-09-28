#ifndef SMAP_CORE__MAP_EXPORTER_HPP_
#define SMAP_CORE__MAP_EXPORTER_HPP_

// C
#include <cstdio>

// STL
#include <memory>
#include <vector>

// BOOST

// ROS
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

// SMAP

// compute linear index for given map coords
#define MAP_IDX( width, r, c ) ( ( width ) * ( r ) + ( c ) )

namespace smap
{

// template< typename Functor >
// void save_og_cell( Functor functor )
// {
//     cout << functor( 10 ) << endl;
// }

class map_exporter : public rclcpp::Node

{
  private:

    rclcpp::Subscription< nav_msgs::msg::OccupancyGrid >::SharedPtr og_sub =
        this->create_subscription< nav_msgs::msg::OccupancyGrid >(
            std::string( "/map" ), 5, std::bind( &map_exporter::og_callback, this, std::placeholders::_1 ) );

    rclcpp::Publisher< nav_msgs::msg::OccupancyGrid >::SharedPtr og_pub =
        this->create_publisher< nav_msgs::msg::OccupancyGrid >(
            std::string( this->get_namespace() ) + std::string( "/map_exporter/occupancy_grid" ), 5 );

    rclcpp::Publisher< nav_msgs::msg::OccupancyGrid >::SharedPtr og_center_pub =
        this->create_publisher< nav_msgs::msg::OccupancyGrid >(
            std::string( this->get_namespace() ) + std::string( "/map_exporter/occupancy_grid_center" ), 5 );

  public:

    map_exporter( void ) : Node( "map_exporter" )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing map_exporter" );
        //
    }

    ~map_exporter( void )
    {
        RCLCPP_WARN( this->get_logger(), "Saving Maps" );

        // og_sub = this->create_subscription< nav_msgs::msg::OccupancyGrid >(
        //     std::string( "/map" ), 2, std::bind( &map_exporter::export_maps, this, std::placeholders::_1 ) );

        // while( !this->map_exported )
        // {
        //     RCLCPP_WARN( this->get_logger(), "EXPORTING MAP" );
        //     this->ending = true;
        //     std::this_thread::yield();
        // }
        // RCLCPP_WARN( this->get_logger(), "TOPO-" );
        // this->export_graph( "TopoGraph" );

        //
    }

    void og_callback( const nav_msgs::msg::OccupancyGrid::SharedPtr og )
    {
        // Copy Map
        std::vector< signed char > map, aux;
        map.resize( og->info.height * og->info.width );
        aux.resize( og->info.height * og->info.width );

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

        // Publish occupancy_grid_center
        for( unsigned int y = 0; y < og->info.height; y++ )
        {
            for( unsigned int x = 0; x < og->info.width; x++ )
            {
                //
                unsigned int i = x + ( og->info.height - y - 1 ) * og->info.width;
            }
        }

        file_name = path_name + std::string( "og_center" );
        this->save_map( file_name, map, og->info, occupancy_grid_msg );
        og_center_pub->publish( occupancy_grid_msg );
    }

    // void save_og_cell(
    //     const std::vector< signed char >& map, const unsigned int& x, const unsigned int& y, const unsigned int& i,
    //     FILE* output ) const
    // {
    //     (void) x;
    //     (void) y;
    //     if( map[ i ] >= 0 && map[ i ] <= 25 )
    //     {  // Unknown
    //         fputc( 254, output );
    //     }
    //     else if( map[ i ] >= 25 )
    //     {  // (occ,255] Object
    //         fputc( 000, output );
    //     }
    //     else
    //     {  // occ [0.25,0.65] Free
    //         fputc( 205, output );
    //     }
    // }

    // void save_c_origin_cell(
    //     const std::vector< signed char >& map, const unsigned int& x, const unsigned int& y, const unsigned int& i,
    //     FILE* output, const nav_msgs::msg::MapMetaData& metadata ) const
    // {
    //     (void) x;
    //     (void) y;
    //     if( map[ i ] >= 0 && map[ i ] <= 25 )
    //     {  //
    //         fputc( 254, output );
    //     }
    //     else if( map[ i ] >= 25 )
    //     {  // (occ,255]
    //         fputc( 000, output );
    //     }
    //     else
    //     {  // occ [0.25,0.65]
    //         fputc( 205, output );
    //     }
    // }

    void save_map(
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

    // void _save_metadata( const std::string& file_name, const nav_msgs::msg::MapMetaData& metadata )
    // {
    //     FILE* output = fopen( file_name.c_str(), "w" );
    //     if( !output )
    //     {
    //         RCLCPP_ERROR( this->get_logger(), "Couldn't save map metadata [%s]", file_name.c_str() );
    //         return;
    //     }
    //     geometry_msgs::msg::Quaternion orientation = metadata.origin.orientation;
    //     tf2::Matrix3x3 mat( tf2::Quaternion( orientation.x, orientation.y, orientation.z, orientation.w ) );
    //     double yaw, pitch, roll;
    //     mat.getEulerYPR( yaw, pitch, roll );

    // fprintf(
    //     output,
    //     "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
    //     file_name.c_str(), metadata.resolution, metadata.origin.position.x, metadata.origin.position.y, yaw );

    // fclose( output );
    // }

    // void __save_map(
    //     std::vector< signed char >& map, const nav_msgs::msg::MapMetaData& metadata,
    //     nav_msgs::msg::OccupancyGrid& occupancy_grid_msg )
    // {
    //     // Map
    //     const std::string file_path = std::string( "/workspace/src/smap/smap_core/maps/" );
    //     std::string file_name       = file_path + std::string( "og_map" ) + std::string( ".pgm" );
    //     RCLCPP_WARN( this->get_logger(), "Writing og_map to %s", file_name.c_str() );
    //     // this->__save_map( file_name, map, metadata.width, metadata.height, metadata.resolution, occupancy_grid_msg
    //     );

    // // Metadata
    // file_name = file_path + std::string( "og_map" ) + std::string( ".yaml" );
    // RCLCPP_WARN( this->get_logger(), "Writing og_map to %s", file_name.c_str() );
    // this->_save_metadata( file_name, metadata );
    // //
    // }
};
}  // namespace smap

#endif  // SMAP_CORE__MAP_EXPORTER_HPP_
