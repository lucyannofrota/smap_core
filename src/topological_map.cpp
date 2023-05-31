#include "../include/smap_core/topological_map.hpp"

// #include "rcl/error_handling.h"
// #include <rmw/types.h>

namespace smap
{

void topological_map::on_process( void )  // Pooling
{
    if( publish_vertex )
    {
        publisher_marker->publish( vertex_marker );
        RCLCPP_DEBUG( this->get_logger(), "Vertex Marker Published" );
        publish_vertex = false;
    }
    if( publish_edge )
    {
        publisher_marker->publish( edge_marker );
        RCLCPP_DEBUG( this->get_logger(), "Edge Marker Published" );
        publish_edge = false;
    }
}

void topological_map::add_vertex( const geometry_msgs::msg::Point& pos, VertexData*& current, VertexData*& previous )
{
    // add vertex based on the current pose

    double dist_current;
    VertexData* closest;
    this->_get_closest_vertex( closest, pos );

    size_t idx;
    auto print_vertex = []( std::string prefix, VertexData& v ) {
        // if(&v == nullptr) printf("%s nullptr\n",prefix.c_str());
        printf( "%s (%i) [%4.1f,%4.1f,%4.1f]\n", prefix.c_str(), (int) v.index, v.pos.x, v.pos.y, v.pos.z );
    };

    printf( "add_vertex\n" );
    // Initialization
    printf( "\tInitialization\n" );
    if( current == nullptr )
    {
        printf( "\t\tcurrent null\n" );
        if( closest == nullptr )
        {
            printf( "\t\tclosest null\n" );
            printf( "\t\tadding Vertex\n" );
            idx     = this->_add_vertex( this->v_index++, pos );
            current = &( Graph[ idx ] );
            print_vertex( std::string( "\t\t" ), *current );
            return;
        }
        else
        {
            printf( "\t\tclosest != nullptr\n" );
            print_vertex( std::string( "\t\tb: " ), *current );
            current = closest;
            print_vertex( std::string( "\t\ta: " ), *current );
        }
    }

    // Connection
    printf( "\tConnection\n" );
    if( ( closest != nullptr ) && ( closest != current ) )
    {
        print_vertex( std::string( "\t\t\tCurrent: " ), *current );
        // print_vertex(std::string("\t\t\tPrevious: "),*previous);
        print_vertex( std::string( "\t\t\tClosest: " ), *closest );
        printf( "\t\tadding Edge\n" );
        _add_edge( _get_TopoMap_index( current ), _get_TopoMap_index( closest ) );
        previous = current;
        current  = closest;
        print_vertex( std::string( "\t\t\t/Current: " ), *current );
        // print_vertex(std::string("\t\t\t/Previous: "),*previous);
        print_vertex( std::string( "\t\t\t/Closest: " ), *closest );
    }

    // New vertex
    printf( "\tNew vertex\n" );
    dist_current =
        sqrt( pow( current->pos.x - pos.x, 2 ) + pow( current->pos.y - pos.y, 2 ) + pow( current->pos.z - pos.z, 2 ) );

    if( dist_current < VERTEX_DISTANCE )
    {
        printf( "\t\tToo close\n" );
        return;
    }
    printf( "\t\tadding Vertex\n" );
    print_vertex( std::string( "\t\t\tCurrent: " ), *current );
    // print_vertex(std::string("\t\t\tPrevious: "),*previous);
    print_vertex( std::string( "\t\t\tClosest: " ), *closest );
    idx      = _add_vertex( this->v_index++, pos );
    previous = current;
    current  = &( Graph[ idx ] );
    print_vertex( std::string( "\t\t\t/Current: " ), *current );
    // print_vertex(std::string("\t\t\t/Previous: "),*previous);
    print_vertex( std::string( "\t\t\t/Closest: " ), *closest );

    printf( "\t\tadding Edge\n" );
    print_vertex( std::string( "\t\t\tCurrent: " ), *current );
    // print_vertex(std::string("\t\t\tPrevious: "),*previous);
    print_vertex( std::string( "\t\t\tClosest: " ), *closest );
    printf( "\t\tadding Edge\n" );
    _add_edge( _get_TopoMap_index( previous ), _get_TopoMap_index( current ) );
    print_vertex( std::string( "\t\t\t/Current: " ), *current );
    // print_vertex(std::string("\t\t\t/Previous: "),*previous);
    print_vertex( std::string( "\t\t\t/Closest: " ), *closest );
}

void topological_map::add_vertex_( const geometry_msgs::msg::Point& pos, VertexData** current, VertexData** previous )
{
    // add vertex based on the current pose

    double dist_current;
    VertexData* closest = _get_valid_close_vertex( pos, NEW_EDGE_FACTOR );
    size_t idx;
    auto print_vertex = []( std::string prefix, VertexData& v ) {
        printf( "%s (%i) [%4.1f,%4.1f,%4.1f]\n", prefix.c_str(), (int) v.index, v.pos.x, v.pos.y, v.pos.z );
    };
    if( ( *current ) == nullptr )
    {
        printf( "current null\n" );
        if( closest == nullptr )
        {
            printf( "closest == nullptr\n" );
            idx          = this->_add_vertex( this->v_index++, pos );
            ( *current ) = &( Graph[ idx ] );
            return;
        }
        else
        {
            printf( "closest != nullptr\n" );
            ( *current ) = closest;
        }
    }

    print_vertex( std::string( "" ), ( **current ) );

    if( ( closest != nullptr ) && ( closest != ( *current ) ) )
    {
        printf( "add_edge\n" );
        // this->export_TopoGraph(std::string("TopoGraph_-1"));
        // printf("if ((closest != nullptr) && (closest != (*current)))\n");
        // printf("\tcurrent: %li\n",(*current)->index);
        // printf("\tprevious: %li\n",(*previous) == nullptr ? 9999 : (*previous)->index);
        // printf("\tclosest: %li\n",closest->index);
        // static int b = 1;
        printf( "\n\n\n\n\n" );
        printf( "Graph: \n" );
        auto pair = boost::vertices( this->Graph );
        for( auto it = pair.first; it != pair.second; ++it ) print_vertex( "\t", this->Graph[ *it ] );

        print_vertex( std::string( "Closest: " ), *closest );
        // print_vertex(std::string("Current: "), (*current));

        _add_edge( _get_TopoMap_index( ( *current ) ), _get_TopoMap_index( closest ) );
        ( *previous ) = ( *current );
        ( *current )  = closest;
        // printf("\t/current: %li| [%4.1f,%4.1f]\n",(*current)->index, (*current)->pos.x,(*current)->pos.y);
        // printf("\t/previous: %i| [%4.1f,%4.1f]\n",(*previous) == nullptr ? 9999 : (int)(*previous)->index,
        // (*previous)->pos.x,(*previous)->pos.y); printf("\t/closest: %li| [%4.1f,%4.1f]\n",closest->index,
        // closest->pos.x,closest->pos.y); this->export_TopoGraph(std::string("TopoGraph_")+std::to_string(b++));
        printf( "\n\n\n\n\n" );
        return;
    }
    printf( "FALSE add_edge\n" );

    dist_current = sqrt(
        pow( ( *current )->pos.x - pos.x, 2 ) + pow( ( *current )->pos.y - pos.y, 2 )
        + pow( ( *current )->pos.z - pos.z, 2 ) );

    if( dist_current < VERTEX_DISTANCE )
    {
        printf( "dist_current < VERTEX_DISTANCE\n" );
        return;
    }
    printf( "FALSE dist_current < VERTEX_DISTANCE\n" );
    idx           = _add_vertex( this->v_index++, pos );
    ( *previous ) = ( *current );
    ( *current )  = &( Graph[ idx ] );

    _add_edge( _get_TopoMap_index( ( *previous ) ), _get_TopoMap_index( ( *current ) ) );
}

void topological_map::object_callback( const smap_interfaces::msg::SmapObject::SharedPtr object )
{
    // TODO: create callback group. Should be mutually exclusive
    if( boost::num_vertices( this->Graph ) == 0 ) return;

    VertexData *closest = nullptr, *current = this->current_vertex, *previous = nullptr;
    double distance, t;
    auto calc_closest = [ & ]( const smap_interfaces::msg::SmapObject::SharedPtr object ) {
        // Find the closes vertex relative to the object position
        // Don't consider distances through z axis
        auto pair  = boost::vertices( this->Graph );
        double min = DBL_MAX;
        for( auto it = pair.first; it != pair.second; it++ )
        {
            distance = sqrt(
                pow( object->obj_pose.pose.position.x - this->Graph[ *it ].pos.x, 2 )
                + pow( object->obj_pose.pose.position.y - this->Graph[ *it ].pos.y, 2 ) );
            if( distance < min )
            {
                min     = distance;
                closest = &( this->Graph[ *it ] );
            }
        }
        return closest;
    };

    auto calc_dist = []( const geometry_msgs::msg::Point& pt1, const geometry_msgs::msg::Point& pt2 ) {
        return sqrt( pow( pt1.x - pt2.x, 2 ) + pow( pt1.y - pt2.y, 2 ) );
    };

    // Find the closes vertex relative to the object position
    // Don't consider distances through z axis
    // auto pair = boost::vertices(this->Graph);
    // VertexData *closest = nullptr, *prev = nullptr;
    // double distance, min = DBL_MAX;
    // for (auto it = pair.first; it != pair.second; it++)
    // {
    //   distance = sqrt(
    //       pow(object->obj_pose.pose.position.x - this->Graph[*it].pos.x, 2) +
    //       pow(object->obj_pose.pose.position.y - this->Graph[*it].pos.y, 2));
    //   if (distance < min)
    //   {
    //     min = distance;
    //     closest = &(this->Graph[*it]);
    //   }
    // }

    closest = calc_closest( object );
    printf( "object_callback: current (%i)\n", (int) current->index );
    printf( "object_callback: closest (%i)\n", (int) closest->index );

    // Create new vertex to approach the object if the distance greater then VERTEX_DISTANCE * NEW_EDGE_FACTOR
    distance         = calc_dist( object->obj_pose.pose.position, closest->pos );

    int n_new_vertex = 1;
    geometry_msgs::msg::Point new_point;
    new_point.z = 0;

    // 1 < NEW_EDGE_FACTOR < 2
    if( distance < VERTEX_DISTANCE * ( 2 - NEW_EDGE_FACTOR ) ) this->append_object();

    int v = 0;

    while( distance > VERTEX_DISTANCE && n_new_vertex > 0 )
    {
        this->export_TopoGraph( std::string( "TopoGraph_" ) + std::to_string( v++ ) );
        n_new_vertex = floor( distance / ( VERTEX_DISTANCE * NEW_EDGE_FACTOR ) );
        t            = ( VERTEX_DISTANCE * NEW_EDGE_FACTOR ) / distance;
        new_point.x  = ( 1 - t ) * closest->pos.x + t * object->obj_pose.pose.position.x;
        new_point.y  = ( 1 - t ) * closest->pos.y + t * object->obj_pose.pose.position.y;

        // add vertex
        // add_vertex(
        //   new_point,
        //   current,
        //   previous
        // );

        int idx  = this->_add_vertex( this->v_index++, new_point );
        previous = current;
        current  = &( Graph[ idx ] );

        this->_add_edge( this->_get_TopoMap_index( previous ), this->_get_TopoMap_index( current ) );

        // TODO: Add print graph to the destructor
        // TODO: Debug the add_vertex method

        // int idx = this->add_vertex

        // /add vertex

        // printf("/cv: %p\n",(void*)this->current_vertex);
        this->export_TopoGraph( std::string( "TopoGraph_" ) + std::to_string( v++ ) );

        closest  = calc_closest( object );
        distance = calc_dist( object->obj_pose.pose.position, closest->pos );  // verify
    }

    // while (distance > VERTEX_DISTANCE * NEW_EDGE_FACTOR && n_new_vertex > 0)
    // {
    //   this->export_TopoGraph("TopoGraph_0");
    //   n_new_vertex = floor(distance / (VERTEX_DISTANCE * NEW_EDGE_FACTOR));
    //   t = (VERTEX_DISTANCE * NEW_EDGE_FACTOR) / distance;
    //   new_point.x = (1 - t) * closest->pos.x + t * object->obj_pose.pose.position.x;
    //   new_point.y = (1 - t) * closest->pos.y + t * object->obj_pose.pose.position.y;
    //   printf("cv: %p\n",(void*)this->current_vertex);

    // // add vertex

    // // VertexData *closest = _get_valid_close_vertex(pos, NEW_EDGE_FACTOR);
    // closest = calc_closest(object);

    // this->add_vertex(
    //     new_point,
    //     &closest,
    //     &prev
    // );

    // // add vertex

    // printf("/cv: %p\n",(void*)this->current_vertex);
    // distance = sqrt(
    //     pow(object->obj_pose.pose.position.x - new_point.x, 2) +
    //     pow(object->obj_pose.pose.position.y - new_point.y, 2));
    // this->export_TopoGraph("TopoGraph_1");
    // break;
    // }

    // if(distance > VERTEX_DISTANCE && distance <= VERTEX_DISTANCE * NEW_EDGE_FACTOR){

    // }
}

void topological_map::export_TopoGraph( const std::string& f_name )
{
    std::ofstream dotfile( OUTPUT_PATH + f_name + ".dot" );

    write_graphviz(
        dotfile, Graph,
        make_vertex_label_writer( boost::get( &VertexData::this_thing, Graph ), boost::get( &VertexData::pos, Graph ) ),
        make_cost_label_writer( boost::get( &EdgeData::distance, Graph ), boost::get( &EdgeData::modifier, Graph ) ) );

    if( std::system( ( "dot -Tpng " + OUTPUT_PATH + f_name + ".dot > " + OUTPUT_PATH + f_name + ".png" ).c_str() )
        == 0 )
    {
        if( std::system( ( "rm " + OUTPUT_PATH + f_name + ".dot" ).c_str() ) == 0 )
        {
            // https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
            RCLCPP_INFO(
                this->get_logger(), "\033[42m[Export Complete]\033[0m png file successfully exported to: %s.png",
                std::string( OUTPUT_PATH + f_name ).c_str() );
        }
    }
}

void topological_map::save_map( topological_map& obj ) { topological_map::save_map( obj, DEFAULT_FILE_NAME ); }

void topological_map::save_map( topological_map& obj, std::string file_name )
{
    std::ofstream ofs( SAVE_LOAD_PATH + file_name );
    boost::archive::text_oarchive oa( ofs );
    oa << obj;
    ofs.close();
}

void topological_map::load_map( topological_map& obj ) { topological_map::load_map( obj, DEFAULT_FILE_NAME ); }

void topological_map::load_map( topological_map& obj, std::string file_name )
{
    // TODO: Test load
    std::ifstream ifs( SAVE_LOAD_PATH + file_name );
    boost::archive::text_iarchive ia( ifs );
    ia >> obj;
    obj.previous_vertex = nullptr;
    obj.current_vertex  = nullptr;
    ifs.close();
}

}  // namespace smap

int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );

    std::shared_ptr< smap::topological_map > node = std::make_shared< smap::topological_map >();

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
