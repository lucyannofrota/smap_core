#ifndef IMGUI_VARS__
#define IMGUI_VARS__

#include "smap_base/counters.hpp"

extern plot_vec total_thread_time;
extern plot_vec box_filter_plot, roi_filter_plot, voxelization_plot, sof_filter_plot, euclidean_clustering_plot,
    total_filter_plot;
extern plot_vec centroid_plot, transform_plot, total_estimation_plot;

#endif  // IMGUI_VARS__
