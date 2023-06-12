#ifndef SMAP_CORE__DETECTOR_DESCRIPTOR_HPP_
#define SMAP_CORE__DETECTOR_DESCRIPTOR_HPP_

#include "../pch/pch.hpp"

enum detector_type
{
    object,
    place
};

struct detector_t
{
    std::string name         = "";
    int id                   = 0;
    detector_type type       = detector_type::object;
    std::string architecture = "";
    size_t n_classes         = 0;
    std::map< int, std::string > classes;
};

#endif  // SMAP_CORE__DETECTOR_DESCRIPTOR_HPP_
