#include "include/object_estimator/imgui_vars.hpp"
#include "include/object_estimator/object_estimator.hpp"
#include "parameter_tuning.hpp"

#include <math.h>

// ImGui
#include "imgui/backends/imgui_impl_opengl3.h"
#include "imgui/backends/imgui_impl_sdl2.h"
#include "imgui/imgui.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

// SMAP
// #include "../pcl_processing/include/pcl_processing.hpp"
// #include "pcl_processing/pcl_processing.hpp"
#include "pcl_processing/depth_map.hpp"
#include "smap_base/aux_functions.hpp"

// PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

// plot_vec total_thread_time;
// plot_vec box_filter_plot, roi_filter_plot, voxelization_plot, sof_filter_plot, euclidean_clustering_plot,
//     total_filter_plot;
// plot_vec centroid_plot, transform_plot, total_estimation_plot;

int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );

    std::shared_ptr< smap::object_estimator > node = std::make_shared< smap::object_estimator >();

    // auto ui = std::make_shared<std::future<void>>(
    //   std::async(
    //     std::launch::async,
    //     &imgui_thread,
    //   )
    // );
    auto plot_times_lambda = []( plot_vec& vec ) {  // Plot times
        static char overlay[ 32 ];
        float vals[ 128 ] = { 0 };
        vec.get_float_arr_av( vals );
        sprintf( overlay, "cur: %4.1f ms| avg %4.1f ms", vals[ 127 ], vec.get_average() );
        ImGui::PlotLines( "Execution time", vals, IM_ARRAYSIZE( vals ), 0, overlay, 0.0f, 200.0f, ImVec2( 0, 100.0f ) );
    };

    auto lambda = [ &node, &plot_times_lambda ]( void ) {  // ui lambda function
        ImGui::Begin( "Object Pose Estimator Parameters" );

        if( ImGui::CollapsingHeader( "Filtering" ) )
        {  // Filtering
            ImGui::Indent();

            if( ImGui::CollapsingHeader( "Box Filter" ) )
            {  // box filter
                ImGui::Indent();
                plot_times_lambda( box_filter_plot );
                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "Region Of Interest Filter" ) )
            {  // roi_filter
                ImGui::Indent();
                static bool roi_filter = true;
                if( ImGui::Checkbox( "ROI Filter", &roi_filter ) ) node->roi_filt = roi_filter;

                ImGui::Text( "pcl_lim" );
                ImGui::Text( "   Distance limits applied to the cloud." );

                ImGui::SliderFloat( "min", &( node->pcl_lims->first ), 0.0f, node->pcl_lims->second );
                ImGui::SliderFloat( "max", &( node->pcl_lims->second ), node->pcl_lims->first, 10.0f );

                plot_times_lambda( roi_filter_plot );
                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "PCL Voxelization" ) )
            {  // pcl_voxelization
                ImGui::Indent();
                static bool voxelization = true;
                if( ImGui::Checkbox( "Voxelization", &voxelization ) ) node->voxelization = voxelization;
                ImGui::Text( "LeafSize" );
                ImGui::Text( "   Greater values increase the size of the voxels (filter more points)" );
                ImGui::SliderFloat( "LeafSize", &( node->leaf_size ), 0.0f, 0.05f );

                plot_times_lambda( voxelization_plot );
                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "Statistical Outlier Filter" ) )
            {  // pcl_voxelization
                ImGui::Indent();
                static bool sof = true;
                if( ImGui::Checkbox( "SOF Filter", &sof ) ) node->sof = sof;
                ImGui::Text( "MeanK" );
                ImGui::Text( "   Number of neighbors to evaluate" );
                ImGui::SliderInt( "MeanK", &( node->mean_k ), 0, 100 );

                ImGui::Text( "Mu" );
                ImGui::Text( "   Local standard deviation" );
                ImGui::SliderFloat( "Mu", &( node->mu ), 0.0f, 2.0f );

                plot_times_lambda( sof_filter_plot );
                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "Euclidean Clustering" ) )
            {  // pcl_voxelization
                ImGui::Indent();
                static bool euclidean_clustering = true;
                if( ImGui::Checkbox( "Clustering", &euclidean_clustering ) )
                    node->euclidean_clust = euclidean_clustering;

                ImGui::Text( "Cluster Tolerance" );
                ImGui::Text( "   Max space between points" );
                ImGui::SliderFloat( "ClusterTolerance", &( node->ClusterTolerance ), 0.0f, 1.0f );

                plot_times_lambda( euclidean_clustering_plot );
                ImGui::Unindent();
            }

            plot_times_lambda( total_filter_plot );

            ImGui::Unindent();
        }

        if( ImGui::CollapsingHeader( "Parameter Estimation" ) )
        {  // Parameter estimation
            ImGui::Indent();

            if( ImGui::CollapsingHeader( "Centroid and Limits" ) )
            {  // Parameter estimation
                ImGui::Indent();

                plot_times_lambda( centroid_plot );

                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "Transform" ) )
            {  // Transform coordinates to map reference
                ImGui::Indent();

                plot_times_lambda( transform_plot );

                ImGui::Unindent();
            }

            plot_times_lambda( total_estimation_plot );

            ImGui::Unindent();
        }

        ImGui::Text( "Total execution time" );
        plot_times_lambda( total_thread_time );

        static int clicked = 0;
        if( ImGui::Button( "Lock PCL" ) ) clicked++;
        if( clicked & 1 )
        {
            ImGui::SameLine();
            ImGui::Text( "PCL locked!" );
            node->pcl_lock = true;
        }
        else node->pcl_lock = false;

        ImGui::End();

    };

    std::thread ui( imgui_thread< decltype( lambda ) >, lambda );

    ui.detach();

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
