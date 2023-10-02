#ifndef LABEL_WRITERS_HPP_
#define LABEL_WRITERS_HPP_

namespace smap
{

template< class this_thing_t, class pos_t, class related_things_t >
class vertex_label_writer
{
  public:

    vertex_label_writer( this_thing_t this_thing, pos_t pos_v, related_things_t related_things_v ) :
        thing( this_thing ), pos( pos_v ), related_things( related_things_v )
    {
    }

    template< class VertexOrEdge >
    void operator()( std::ostream& out, const VertexOrEdge& v )
    {
        std::pair< std::string, std::string > g_pair = get( thing, v ).get_vertex_representation();
        geometry_msgs::msg::Point point              = get( pos, v );
        auto relate_things_list                      = get( related_things, v );

        out << " [label="
            << boost::escape_dot_string( add_pos( std::string( "\n\t" ), add_cout( g_pair.first ), point ) )
            << ", color=\"" + g_pair.second + "\"]";

        for( auto& r_thing: relate_things_list )
        {
            std::pair< std::string, std::string > rg_pair = r_thing.get_vertex_representation();
            out << ";\n\t" << std::to_string( count ) << std::string( "obj" ) << std::to_string( obj_count )
                << "  [label="
                << boost::escape_dot_string(
                       add_pos( std::string( "\n\t\t" ), add_cout( rg_pair.first ), r_thing.pos ) )
                << ",color=\"" + rg_pair.second + "\"]";
        }
        count++;
    }

    std::string add_cout( const std::string cname )
    {
        std::list< std::pair< std::string, int > >::iterator it;
        if( find( cname, it ) )
        {
            ( *it ).second++;
            return cname + "_" + std::to_string( ( *it ).second );
        }
        classes.push_back( std::pair< std::string, int > { cname, 1 } );
        return cname + std::string( "_1" );
    }

    std::string add_pos( const std::string prefix, const std::string text, const geometry_msgs::msg::Point point )
    {
        char nuns[ 6 * 3 + 2 * 3 ];
        sprintf( nuns, "%6.1f, %6.1f, %6.1f", point.x, point.y, point.z );
        return text + prefix + std::string( "[" + std::string( nuns ) + "]" );
    }

    bool find( const std::string cname, std::list< std::pair< std::string, int > >::iterator& it )
    {
        size_t id = 0;
        for( it = classes.begin(); it != classes.end(); it++, id++ )
            if( ( *it ).first == cname ) return true;
        return false;
    }

  private:

    this_thing_t thing;
    pos_t pos;
    std::list< std::pair< std::string, int > > classes;
    related_things_t related_things;
    size_t count = 0, obj_count = 0;
};

template< class this_thing_t, class pos_t, class related_things_t >
inline vertex_label_writer< this_thing_t, pos_t, related_things_t > make_vertex_label_writer(
    this_thing_t this_thing, pos_t pos, related_things_t related_things )
{
    return vertex_label_writer< this_thing_t, pos_t, related_things_t >( this_thing, pos, related_things );
}

template< class distance_t, class modifier_t >
class cost_label_writer
{
  public:

    cost_label_writer( distance_t distance_v, modifier_t modifier_v ) : distance( distance_v ), modifier( modifier_v )
    {
    }

    template< class VertexOrEdge >
    void operator()( std::ostream& out, const VertexOrEdge& v ) const
    {
        char buf[ 16 ];
        sprintf( buf, "%6.2f", round( ( boost::get( distance, v ) * boost::get( modifier, v ) * 100 ) ) / 100 );
        out << "[label=" << std::string( buf ) << "]";
    }

  private:

    distance_t distance;
    modifier_t modifier;
};

template< class distance_t, class modifier_t >
inline cost_label_writer< distance_t, modifier_t > make_cost_label_writer(
    distance_t distance_v, modifier_t modifier_v )
{
    return cost_label_writer< distance_t, modifier_t >( distance_v, modifier_v );
}
}  // namespace smap

#endif  // LABEL_WRITERS_HPP_
