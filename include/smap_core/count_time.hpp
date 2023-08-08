#ifndef SMAP_CORE__COUNT_TIME_HPP_
#define SMAP_CORE__COUNT_TIME_HPP_

#define ARRAY_SIZE 16384

// C
#include <stdlib.h>

// STL
#include <array>
#include <chrono>
#include <fstream>
#include <list>
#include <memory>
#include <string>

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

    std::chrono::_V2::system_clock::time_point beginning, end;
    std::string file_name;
    std::unique_ptr< std::array< int64_t, ARRAY_SIZE > > exec_times;
    size_t count = 0;

  public:

    count_time( void ) { this->beginning = std::chrono::high_resolution_clock::now(); }

    count_time( const std::string& name ) : file_name( name )
    {
        exec_times = std::make_unique< std::array< int64_t, ARRAY_SIZE > >();
        exec_times->fill( 0 );
    }

    ~count_time()
    {
        if( this->exec_times ) this->export_times( this->file_name );
    }

    inline int get_time( void )
    {
        end = std::chrono::high_resolution_clock::now();
        return ( std::chrono::duration_cast< std::chrono::milliseconds >( end - beginning ) ).count();
    }

    inline void print_time( const char* str ) { printf( "%s %ims\n", str, this->get_time() ); }

    inline void print_time( const rclcpp::Logger& logger, const char* str )
    {
        RCLCPP_DEBUG( logger, "%s %ims", str, this->get_time() );
    }

    inline void get_time( const rclcpp::Logger& logger, const char* str, plot_vec& vec )
    {
        end = std::chrono::high_resolution_clock::now();

        vec.push_back( ( std::chrono::duration_cast< std::chrono::milliseconds >( end - beginning ) ).count() );

        RCLCPP_DEBUG(
            logger, "%s %ims", str,
            ( std::chrono::duration_cast< std::chrono::milliseconds >( end - beginning ) ).count() );
    }

    inline void start( void ) { this->beginning = std::chrono::high_resolution_clock::now(); }

    inline void stop( void )
    {
        this->exec_times->at( count++ ) =
            ( std::chrono::duration_cast< std::chrono::nanoseconds >( end - beginning ) ).count();
    }

    inline void export_times( const std::string& name )
    {
        std::cout << "Exporting times " << name << "." << std::endl;
        std::ofstream file;
        file.open( name, std::ios_base::trunc );
        if( file.is_open() )
        {
            file << "Exec Times in nanonseconds [ns] | [samples:" << this->count << "]" << std::endl;
            for( const auto& item: *this->exec_times ) file << item << std::endl;
            file.close();
        }
    }
};

#endif  // SMAP_CORE__COUNT_TIME_HPP_
