#include "topo_marker.hpp"

namespace smap
{

void topo_marker::publish_markers( void )
{
    bool init_flag            = true;
    this->vertex.header.stamp = clock->now();
    vertex_pub->publish( this->vertex );
    this->edge.header.stamp = clock->now();
    edge_pub->publish( this->edge );
    this->label.header.stamp = clock->now();
    for( auto e: this->vertex_label )
    {
        this->label.id            = std::get< 0 >( e );
        this->label.pose.position = std::get< 1 >( e );
        this->label.text          = std::get< 2 >( e );
        label_pub->publish( this->label );
    }
    if( init_flag )
    {
        this->vertex.action = visualization_msgs::msg::Marker::MODIFY;
        this->edge.action   = visualization_msgs::msg::Marker::MODIFY;
        init_flag           = false;
    }
}

}  // namespace smap
