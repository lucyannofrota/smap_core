#include "topo_marker.hpp"

namespace smap
{

topo_marker::topo_marker( void )
{
    // Marker msg initialization
    // Vertex
    this->vertex.header.frame_id = "/map";
    this->vertex.ns              = "vertices";
    this->vertex.type            = visualization_msgs::msg::Marker::POINTS;
    this->vertex.action          = visualization_msgs::msg::Marker::MODIFY;
    this->vertex.scale.x         = 0.075 * 4;
    this->vertex.scale.y         = 0.075 * 4;
    this->vertex.scale.z         = 0.075 * 4;
    this->vertex.color.r         = 102.0 / ( 102.0 + 51.0 );
    this->vertex.color.g         = 51.0 / ( 102.0 + 51.0 );
    this->vertex.color.b         = 0.0;
    this->vertex.color.a         = 1.0;

    // Histogram
    this->histogram.header.frame_id = "/map";
    this->histogram.ns              = "histogram";
    this->histogram.type            = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    this->histogram.action          = visualization_msgs::msg::Marker::MODIFY;
    this->histogram.scale.x         = 1;
    this->histogram.scale.y         = 1;
    this->histogram.scale.z         = 1;
    this->histogram.color.r         = 255;
    this->histogram.color.g         = 0;
    this->histogram.color.b         = 0;
    this->histogram.color.a         = 1;
    // std_msgs::msg::ColorRGBA c;
    // c.r = 255;
    // c.g = 0;
    // c.b = 0;
    // c.a = 1;
    // for( int i = 0; i < HISTOGRAM_BINS * 3; i++ ) this->histogram.colors.push_back( c );

    // const float theta[ 3 ]          = { 0, 2.0944, 4.1888 };
    // float t;
    // std::vector< geometry_msgs::msg::Point > points_aux;
    // geometry_msgs::msg::Point pt;
    // for( int i = 0; i < HISTOGRAM_BINS; i++ )
    // {
    //     t    = ( 2 * M_PI / HISTOGRAM_BINS );
    //     pt.x = () points_aux.push_back( pt );
    // }

    // this->histogram.colors
    // this->vertex.scale.x         = 0.075 * 4;
    // this->vertex.scale.y         = 0.075 * 4;
    // this->vertex.scale.z         = 0.075 * 4;
    // this->vertex.color.r = 102.0 / ( 102.0 + 51.0 );
    // this->vertex.color.g = 51.0 / ( 102.0 + 51.0 );
    // this->vertex.color.b = 0.0;
    // this->vertex.color.a = 1.0;

    // Edge
    this->edge.header.frame_id    = "/map";
    this->edge.ns                 = "edges";
    this->edge.type               = visualization_msgs::msg::Marker::LINE_LIST;
    this->edge.action             = visualization_msgs::msg::Marker::MODIFY;
    this->edge.scale.x            = 0.007;
    this->edge.scale.y            = 0.007;
    this->edge.scale.z            = 0.007;
    this->edge.color.r            = 1.0;
    this->edge.color.g            = 0.0;
    this->edge.color.b            = 0.0;
    this->edge.pose.orientation.x = 0;
    this->edge.pose.orientation.y = 0;
    this->edge.pose.orientation.z = 0;
    this->edge.pose.orientation.w = 1.0;
    this->edge.color.a            = 1.0;

    // Label
    this->label.header.frame_id    = "/map";
    this->label.ns                 = "labels";
    this->label.type               = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    this->label.action             = visualization_msgs::msg::Marker::MODIFY;
    this->label.scale.x            = 0.05;
    this->label.scale.y            = 0.05;
    this->label.scale.z            = 0.05;
    this->label.color.r            = 0.0;
    this->label.color.g            = 1.0;
    this->label.color.b            = 0.0;
    this->label.pose.orientation.x = 0;
    this->label.pose.orientation.y = 0;
    this->label.pose.orientation.z = 0;
    this->label.pose.orientation.w = 1.0;
    this->label.color.a            = 1.0;
}

void topo_marker::_append_histogram( void )
{
    // HISTOGRAM_BINS
}

void topo_marker::_gen_triangles(
    float r, std::vector< geometry_msgs::msg::Point >& points, std::vector< std_msgs::msg::ColorRGBA >& colors )
{
    const float thetas[ 3 ] = { 0, 2.0944, 4.1888 };
    std::vector< geometry_msgs::msg::Point > points_aux;
    geometry_msgs::msg::Point p;
    p.z = 0;
    float t;
    for( int i = 0; i < HISTOGRAM_BINS; i++ )
    {
        t   = ( 2 * M_PI / HISTOGRAM_BINS ) * i;
        p.x = ( r + R_TRIANGLES ) * cos( t );
        p.y = ( r + R_TRIANGLES ) * sin( t );
        points_aux.push_back( p );
    }

    int j    = 0;
    auto cit = colors.begin();
    for( auto e: points_aux )
    {
        for( int i = 0; i < 3; i++, cit++ )
        {
            t   = ( 2 * M_PI / HISTOGRAM_BINS ) * j;
            p.x = e.x + R_TRIANGLES * cos( thetas[ i ] + t );
            p.y = e.y + R_TRIANGLES * sin( thetas[ i ] + t );
            points.push_back( p );
            // ( *cit ).r = 255;
            // ( *cit ).g = 0;
            // ( *cit ).b = 0;
            // ( *cit ).a = 1;
        }
        j++;
    }
}

// void topo_marker::publish_markers( void )
// {
//     const std::lock_guard< std::mutex > lock( this->mutex );
//     this->pub->publish( this->array );

// // bool init_flag            = true;
// // this->vertex.header.stamp = clock->now();
// // this->pub->publish( this->vertex );
// // this->edge.header.stamp = clock->now();
// // this->pub->publish( this->edge );
// // this->label.header.stamp = clock->now();
// // for( auto e: this->vertex_data )
// // {
// //     this->label.id            = e.id;     // std::get< 0 >( e );
// //     this->label.pose.position = e.pos;    // std::get< 1 >( e );
// //     this->label.text          = e.label;  // std::get< 2 >( e );
// //     this->pub->publish( this->label );
// //     // for( auto obj: e.objects )
// //     // {
// //     this->_gen_triangles( 1, this->histogram.points, this->histogram.colors );
// //     this->histogram.pose.position = e.pos;
// //     this->pub->publish( this->histogram );
// //     // }
// // }
// }

void topo_marker::update_markers( void )
{
    const std::lock_guard< std::mutex > lock( this->mutex );
    this->array.markers.clear();
}
}  // namespace smap
