#ifndef SMAP_CORE__DETECTOR_DESCRIPTOR_HPP_
#define SMAP_CORE__DETECTOR_DESCRIPTOR_HPP_

#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>
#include <map>
#include <string>

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

    friend class boost::serialization::access;

    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        ar & version;
        ar & name;
        ar & id;
        ar & type;
        ar & architecture;
        ar & n_classes;
        ar & classes;
    }
};

#endif  // SMAP_CORE__DETECTOR_DESCRIPTOR_HPP_
