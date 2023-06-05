#include "../include/smap_core/thing.hpp"

namespace smap
{

std::string thing::get_label( void ) { return std::string( "Corridor" ); }

std::pair< std::string, std::string > thing::get_vertex_representation()
{
    return std::pair< std::string, std::string >( std::string( "Corridor" ), std::string( "red" ) );
}

void thing::update( semantic_type_t type, double angle )
{
    (void) angle;
    static bool set = false;
    if( !set )
    {
        set        = true;
        this->type = type;
        // TODO: initialize histogram
        // this->histogram
    }
    else
    {
        // TODO: define else
    }
}

}  // namespace smap
