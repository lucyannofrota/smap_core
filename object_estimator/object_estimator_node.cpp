#include "include/object_estimator/imgui_vars.hpp"
#include "include/object_estimator/object_estimator.hpp"

// #include "parameter_tuning.hpp"

// #include <math.h>

// // ImGui
// #include "imgui/backends/imgui_impl_opengl3.h"
// #include "imgui/backends/imgui_impl_sdl2.h"
// #include "imgui/imgui.h"

// #include <SDL2/SDL.h>
// #include <SDL2/SDL_opengl.h>

// // SMAP
// #include "smap_base/aux_functions.hpp"

// // PCL
// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>

// using namespace std::chrono_literals;

// TODO: Check the slider parameters to avoid seg fault

int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );

    std::shared_ptr< smap::object_estimator > node = std::make_shared< smap::object_estimator >();

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
