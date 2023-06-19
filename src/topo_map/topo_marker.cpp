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

    // aabb
    this->aabb.header.frame_id    = "/map";
    this->aabb.ns                 = "aabb";
    this->aabb.type               = visualization_msgs::msg::Marker::CUBE;
    this->aabb.action             = visualization_msgs::msg::Marker::MODIFY;
    this->aabb.pose.orientation.x = 0;
    this->aabb.pose.orientation.y = 0;
    this->aabb.pose.orientation.z = 0;
    this->aabb.pose.orientation.w = 1.0;
    this->aabb.color.r            = 0.0;
    this->aabb.color.g            = 0.0;
    this->aabb.color.b            = 1.0;
    this->aabb.color.a            = 0.5;

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

    // aabb label
    this->aabb_label         = this->label;
    this->aabb_label.ns      = "object_label";
    this->aabb_label.color.r = 1.0;
    this->aabb_label.color.g = 0.0;
    this->aabb_label.color.b = 1.0;
    this->aabb_label.color.a = 1.0;

    // Generating triangles
    float t;
    triangles_t triangle;
    const float thetas[ 3 ] = { 0, 2.0944, 4.1888 };
    for( int i = 0; i < HISTOGRAM_BINS; i++ )
    {
        for( int j = 0; j < 3; j++ )
        {
            t                = ( 2 * M_PI / HISTOGRAM_BINS ) * i;
            triangle.point.x = R_TRIANGLES * cos( thetas[ j ] + t );
            triangle.point.y = R_TRIANGLES * sin( thetas[ j ] + t );
            this->triangles_base.push_back( triangle );
        }
    }
}

// void topo_marker::_append_histogram( void )
// {
//     // HISTOGRAM_BINS
// }

std_msgs::msg::ColorRGBA topo_marker::histogram_color_picker( double min, double max, double value )
{
    std_msgs::msg::ColorRGBA color;
    color.b     = 0.0;
    double diff = max - min, ct = diff / 2, alpha = abs( value - ct ) / ( diff / 2 );

    color.a = alpha > HISTOGRAM_MARKER_ALPHA_LIM ? alpha : 0;
    color.r = 1.0 - value / diff;
    color.g = 0.0 + value / diff;
    return color;
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

void topo_marker::update_markers( const graph_t& graph )
{
    const std::lock_guard< std::mutex > lock( this->mutex );
    this->array.markers.clear();
    this->vertex.points.clear();
    this->edge.points.clear();
    this->histogram.points.clear();
    this->histogram.colors.clear();

    // Vertices
    geometry_msgs::msg::Point up_point, aux_point, d_point;
    up_point.x  = 0;
    up_point.y  = 0;
    up_point.z  = 0.2;

    aux_point.x = 0;
    aux_point.y = 0;
    aux_point.z = 0.1;

    int obj_id  = 0;
    for( auto e: boost::make_iterator_range( boost::vertices( graph ) ) )
    {
        printf( "update_markers()| n_objects: %i\n", (int) graph[ e ].related_things.size() );

        // edge
        for( auto edg: boost::make_iterator_range( boost::out_edges( e, graph ) ) )
        {
            this->edge.points.push_back( graph[ e ].pos );
            this->edge.points.push_back( graph[ edg.m_target ].pos );
        }

        // vertex
        this->vertex.points.push_back( graph[ e ].pos );
        // label
        this->label.pose.position = graph[ e ].pos + up_point;
        this->label.text = graph[ e ].this_thing.get_label() + std::string( "_" ) + std::to_string( graph[ e ].index );
        this->label.id   = graph[ e ].index;
        this->label.header.stamp = clock->now();
        this->array.markers.push_back( this->label );
        // histogram
        double offset_theta, r;
        // thing r_thing;
        // r_thing.pos = graph[ e ].pos;
        // // r_thing.pos.y         = graph[ e ].pos.y;
        // // r_thing.pos.z         = graph[ e ].pos.z;
        // r_thing.aabb.first.x  = 1.51;
        // r_thing.aabb.first.y  = 1.51;
        // r_thing.aabb.first.z  = 1.1;
        // r_thing.aabb.second.x = 1.0;
        // r_thing.aabb.second.y = 1.0;
        // r_thing.aabb.second.z = 0.9;
        for( auto& r_thing: graph[ e ].related_things )
        {
            // Histogram
            int j = 0;
            std_msgs::msg::ColorRGBA color;
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 1.0;
            auto it = this->triangles_base.begin();
            for( int i = 0; i < HISTOGRAM_BINS; i++ )
            {
                offset_theta = ( 2 * M_PI / HISTOGRAM_BINS ) * i;
                for( j = 0; j < 3; j++, it++ )
                {
                    d_point     = abs( r_thing.aabb.second - r_thing.aabb.first );
                    r           = ( d_point.x > d_point.y ? d_point.x : d_point.y ) * 0.7;
                    aux_point.x = ( r + R_TRIANGLES ) * cos( offset_theta );
                    aux_point.y = ( r + R_TRIANGLES ) * sin( offset_theta );

                    this->histogram.points.push_back( r_thing.pos + it->point + aux_point );
                    this->histogram.colors.push_back(
                        this->histogram_color_picker( 0, 1, log_odds_inv( r_thing.observations.histogram[ i ] ) ) );
                }
            }

            this->histogram.id           = obj_id;
            this->histogram.header.stamp = clock->now();
            this->array.markers.push_back( this->histogram );

            // aabb
            this->aabb.id            = obj_id;
            this->aabb.header.stamp  = clock->now();
            this->aabb.scale         = vec3_abs( r_thing.aabb.second - r_thing.aabb.first );
            this->aabb.pose.position = r_thing.pos;
            this->array.markers.push_back( this->aabb );

            // aabb label
            this->aabb_label.pose.position    = r_thing.pos + ( up_point * 0.5 );
            this->aabb_label.pose.position.z += this->aabb.scale.z / 2;
            this->aabb_label.text             = r_thing.get_label();
            this->aabb_label.id               = obj_id;
            this->aabb_label.header.stamp     = clock->now();
            this->array.markers.push_back( this->aabb_label );

            obj_id++;
        }
    }

    this->vertex.header.stamp = clock->now();
    this->edge.header.stamp   = clock->now();

    this->array.markers.push_back( this->edge );
    this->array.markers.push_back( this->vertex );
    this->pub->publish( this->array );
}
}  // namespace smap