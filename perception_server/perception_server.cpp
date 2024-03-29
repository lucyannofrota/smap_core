#include "include/perception_server/perception_server.hpp"

namespace smap
{

std::map< std::string, std::pair< int, int > > perception_server::add_detector( detector_t& new_detector )
{
    // id
    new_detector.id = ( this->detectors->size() > 0 ) ? this->detectors->size() + 1 : 1;

    // classes
    std::map< std::string, std::pair< int, int > > new_classes;
    int l_id = 0;
    for( const auto& e: *( this->classes ) )
        if( e.second.first > l_id ) l_id = e.second.first;
    for( const auto& detector_cls: new_detector.classes )
    {
        // Check if exists
        bool rep = false;
        for( const auto& server_cls: *( this->classes ) )
        {
            if( detector_cls.second == server_cls.first )
            {
                rep = true;
                break;
            }
        }
        // append
        if( !rep )
        {
            new_classes[ detector_cls.second ]        = std::pair< int, int >( l_id, detector_cls.first );
            ( *this->classes )[ detector_cls.second ] = std::pair< int, int >( l_id, detector_cls.first );
            this->n_classes++;
        }
        l_id++;
    }

    // detector
    this->detectors->push_back( new_detector );
    return new_classes;
}

void perception_server::AddPerceptionModule_callback(
    const std::shared_ptr< smap_interfaces::srv::AddPerceptionModule::Request > request,
    const std::shared_ptr< smap_interfaces::srv::AddPerceptionModule::Response > response )
{
    RCLCPP_INFO( this->get_logger(), "add_perception_module request received." );

    response->is_new  = true;
    response->success = true;

    // Check if request is empty or if type is invalid
    if( ( request->name.empty() ) || ( request->type.empty() ) || ( request->architecture.empty() )
        || ( ( request->type != "object" ) && ( request->type != "place" ) ) )
    {
        response->success = false;
        RCLCPP_WARN( this->get_logger(), "Invalid request received from client." );
        return;
    }

    // Filling detector structure
    detector_t det;
    det.name = request->name;
    if( request->type == "object" ) det.type = detector_type::object;
    else
    {
        if( request->type == "place" ) det.type = detector_type::place;
        else
        {
            response->success = false;
            return;
        }
    }
    det.architecture = request->architecture;
    det.n_classes    = request->n_classes;
    if( ( request->classes.empty() )
        || ( request->n_classes != request->classes.size()
             || ( request->classes.size() != request->class_ids.size() ) ) )
    {
        response->success = false;
        RCLCPP_WARN( this->get_logger(), "Invalid classes list." );
        return;
    }
    for( size_t cls_i = 0; cls_i < request->n_classes; cls_i++ )
        det.classes[ request->class_ids[ cls_i ] ] = request->classes[ cls_i ];

    // Check if the detector is already registered
    for( const auto& detector: ( *this->detectors ) )
    {
        if( detector.name == request->name )
        {
            response->is_new = false;

            response->success =
                ( ( detector.type == det.type ) && ( detector.architecture == det.architecture )
                  && ( detector.n_classes == det.n_classes ) );

            size_t eq_classes = 0;
            for( const auto& cls: detector.classes )
            {
                for( const auto& r_cls: det.classes )
                {
                    if( cls == r_cls )
                    {
                        eq_classes++;
                        break;
                    }
                }
            }
            response->success &= ( eq_classes == detector.n_classes );

            if( response->success )
            {
                response->module_id = detector.id;
                RCLCPP_WARN(
                    this->get_logger(), "Client requesting connection with duplicate detector name. The id of the "
                                        "first entry will be sent." );
            }
            else
                RCLCPP_WARN(
                    this->get_logger(),
                    "Client requesting connection with duplicate detector name and different classes." );
            return;
        }
    }

    auto new_classes    = this->add_detector( det );

    response->module_id = det.id;

    // Logging
    RCLCPP_INFO( this->get_logger(), "Request successfully processed." );
    RCLCPP_INFO( this->get_logger(), "Module (%s) added with %i classes:", request->name.c_str(), request->n_classes );

    RCLCPP_INFO( this->get_logger(), "\t%i new classes added:", new_classes.size() );
    this->print_classes( "\t\t", new_classes );

    RCLCPP_INFO( this->get_logger(), "Server Classes [%i]:", this->classes->size() );
    this->print_classes( "\t\t", this->classes );
    //
}

void perception_server::ListClasses_callback(
    const std::shared_ptr< smap_interfaces::srv::SmapClasses::Request > request,
    const std::shared_ptr< smap_interfaces::srv::SmapClasses::Response > response ) const
{
    RCLCPP_INFO( this->get_logger(), "list_classes request received." );

    // Check module id
    if( request->module_id == 0 )
    {
        response->n_classes = this->classes->size();
        response->classes.clear();
        for( const auto& cls: ( *this->classes ) ) response->classes.push_back( cls.first );
    }
    else
    {
        for( const auto& detector: ( *this->detectors ) )
        {
            if( detector.id == request->module_id )
            {
                response->n_classes = detector.n_classes;
                for( const auto& cls: detector.classes ) response->classes.push_back( cls.second );
                return;
            }
        }
        RCLCPP_INFO( this->get_logger(), "Requested module does not exist." );
    }
}

void perception_server::observations_callback( const smap_interfaces::msg::SmapObservation::SharedPtr object ) const
{
    // object->bb_2d
    (void) object;
}

void perception_server::print_classes( std::string pref, const std::map< int, std::string >& classes ) const
{
    for( const auto& cls: classes )
        RCLCPP_INFO( this->get_logger(), "%s%4i| %s", pref.c_str(), cls.first, cls.second.c_str() );
}

void perception_server::print_classes(
    std::string pref, const std::map< std::string, std::pair< int, int > >& classes ) const
{
    RCLCPP_INFO( this->get_logger(), "%so idx|Server|Detector|Label", pref.c_str() );
    int i = 0;
    for( const auto& cls: classes )
        RCLCPP_INFO(
            this->get_logger(), "%s %4i|  %4i|    %4i| %s", pref.c_str(), i++, cls.second.first, cls.second.second,
            cls.first.c_str() );
}

void perception_server::print_classes(
    std::string pref, const std::shared_ptr< std::map< std::string, std::pair< int, int > > >& classes ) const
{
    RCLCPP_INFO( this->get_logger(), "%so idx|Server|Detector|Label", pref.c_str() );
    int i = 0;
    for( const auto& cls: *classes )
        RCLCPP_INFO(
            this->get_logger(), "%s %4i|  %4i|    %4i| %s", pref.c_str(), i++, cls.second.first, cls.second.second,
            cls.first.c_str() );
}

void perception_server::print_classes( std::string pref, const std::vector< std::string >& classes ) const
{
    int idx = 0;
    for( const auto& cls: classes ) RCLCPP_INFO( this->get_logger(), "%s%4i| %s", pref.c_str(), idx++, cls.c_str() );
}

void perception_server::print_detector( std::string pref, const detector_t& det ) const
{
    RCLCPP_INFO(
        this->get_logger(),
        "%sDet:\n%s\tarch: %s\n%s\tname: %s\n%s\tid: %i\n%s\ttype: %i\n%s\tn_classes: %i\n%s\tclasses:", pref.c_str(),
        pref.c_str(), det.architecture.c_str(), pref.c_str(), det.name.c_str(), pref.c_str(), det.id, pref.c_str(),
        det.type, pref.c_str(), (int) det.n_classes, pref.c_str() );
    perception_server::print_classes( pref, det.classes );
}

void perception_server::print_server_data( void ) const
{
    RCLCPP_INFO( this->get_logger(), "Detectors:" );
    for( const auto& detector: ( *this->detectors ) ) perception_server::print_detector( "\t", detector );
    RCLCPP_INFO( this->get_logger(), "Classes:\n" );
    perception_server::print_classes( "", this->classes );
}

}  // namespace smap
