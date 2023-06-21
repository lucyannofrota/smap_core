#ifndef SMAP_CORE__COUNT_TIME_HPP_
#define SMAP_CORE__COUNT_TIME_HPP_

// C
#include <stdlib.h>

// STL
#include <chrono>
#include <list>

// ROS
#include <rclcpp/rclcpp.hpp>

class plot_vec : public std::list< int >
{
  private:

    const size_t _max_size = 128;
    float average;

  public:

    inline void push_back( const int& element )
    {
        if( std::list< int >::size() >= _max_size ) std::list< int >::pop_front();
        std::list< int >::push_back( element );
    }

    inline void get_float_arr_av( float* ptr )
    {
        auto it       = this->begin();
        this->average = 0;
        for( int i = 0; it != this->end(); ++it, i++ )
        {
            ptr[ i ]       = *it;
            this->average += *it;
        }
        this->average /= this->size();
    }

    inline float get_average( void ) { return this->average; }
};

class count_time
{

  private:

    std::chrono::_V2::system_clock::time_point start, stop;

  public:

    count_time( void ) { this->start = std::chrono::high_resolution_clock::now(); }

    inline int get_time( void )
    {
        stop = std::chrono::high_resolution_clock::now();
        return ( std::chrono::duration_cast< std::chrono::milliseconds >( stop - start ) ).count();
    }

    inline void print_time( const char* str ) { printf( "%s %ims\n", this->get_time() ); }

    inline void print_time( const rclcpp::Logger& logger, const char* str )
    {
        RCLCPP_DEBUG( logger, "%s %ims", this->get_time() );
    }

    inline void get_time( const rclcpp::Logger& logger, const char* str, plot_vec& vec )
    {
        stop = std::chrono::high_resolution_clock::now();

        vec.push_back( ( std::chrono::duration_cast< std::chrono::milliseconds >( stop - start ) ).count() );

        RCLCPP_DEBUG(
            logger, "%s %ims", str,
            ( std::chrono::duration_cast< std::chrono::milliseconds >( stop - start ) ).count() );
    }
};

#endif  // SMAP_CORE__COUNT_TIME_HPP_
