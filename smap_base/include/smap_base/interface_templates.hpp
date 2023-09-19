#ifndef SMAP_CORE__TEMPLATES_HPP_
#define SMAP_CORE__TEMPLATES_HPP_

// STL
#include <cmath>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// geometry_msgs::msg::Point operators

//+
inline geometry_msgs::msg::Point operator+( const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b )
{
    geometry_msgs::msg::Point ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    return ret;
}

inline geometry_msgs::msg::Point operator+=( geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b )
{
    a.x = a.x + b.x;
    a.y = a.y + b.y;
    a.z = a.z + b.z;
    return a;
}

//-
inline geometry_msgs::msg::Point operator-( const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b )
{
    geometry_msgs::msg::Point ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    return ret;
}

//*
inline geometry_msgs::msg::Point operator*( const geometry_msgs::msg::Point& a, const double b )
{
    geometry_msgs::msg::Point ret;
    ret.x = a.x * b;
    ret.y = a.y * b;
    ret.z = a.z * b;
    return ret;
}

inline geometry_msgs::msg::Point operator*=( geometry_msgs::msg::Point& a, const double b )
{
    a.x = a.x * b;
    a.y = a.y * b;
    a.z = a.z * b;
    return a;
}

inline geometry_msgs::msg::Point operator*=( geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b )
{
    a.x = a.x * b.x;
    a.y = a.y * b.y;
    a.z = a.z * b.z;
    return a;
}

// /
inline geometry_msgs::msg::Point operator/( const geometry_msgs::msg::Point& a, const double b )
{
    geometry_msgs::msg::Point ret;
    ret.x = a.x / b;
    ret.y = a.y / b;
    ret.z = a.z / b;
    return ret;
}

// ==
inline bool operator==( geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b )
{
    bool ret  = false;
    ret      &= a.x == b.x;
    ret      &= a.y == b.y;
    ret      &= a.z == b.z;
    return ret;
}

// abs()
inline geometry_msgs::msg::Point abs( const geometry_msgs::msg::Point a )
{
    geometry_msgs::msg::Point ret;
    ret.x = abs( a.x );
    ret.y = abs( a.y );
    ret.z = abs( a.z );
    return ret;
}

inline geometry_msgs::msg::Vector3 vec3_abs( const geometry_msgs::msg::Point a )
{
    geometry_msgs::msg::Vector3 ret;
    ret.x = abs( a.x );
    ret.y = abs( a.y );
    ret.z = abs( a.z );
    return ret;
}

// sqrt
inline double gPoint_distance( geometry_msgs::msg::Point a, geometry_msgs::msg::Point b )
{
    return sqrt( pow( a.x - b.x, 2 ) + pow( a.y - b.y, 2 ) + pow( a.z - b.z, 2 ) );
}

// norm

inline double norm( const double& x, const double& y, const double& z )
{
    return sqrt( pow( x, 2 ) + pow( y, 2 ) + pow( z, 2 ) );
}

#endif  // SMAP_CORE__TEMPLATES_HPP_
