#ifndef SMAP_CORE__COUNT_TIME_HPP_
#define SMAP_CORE__COUNT_TIME_HPP_

#define ARRAY_SIZE 8192

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

class counter
{
  protected:

    std::unique_ptr< std::array< int64_t, ARRAY_SIZE > > vals_array;
    size_t count = 0;

  public:

    std::string file_name;

    counter( void ) {}

    counter( const std::string& name ) : file_name( name )
    {
        vals_array = std::make_unique< std::array< int64_t, ARRAY_SIZE > >();
        vals_array->fill( 0 );
    }

    virtual ~counter()
    {
        // std::string header = "Values [samples:" + std::to_string( this->count ) + "]";
        // if( this->vals_array ) this->export_vals( header, this->file_name );
    }

    void append( int64_t val ) { this->vals_array->at( count++ ) = val; }

    virtual void export_lines( std::ofstream& file ) {}

    inline void export_vals( const std::string& header, const std::string& name )
    {
        std::cout << "Exporting values " << name << "." << std::endl;
        std::ofstream file;
        file.open( name, std::ios_base::trunc );
        if( file.is_open() )
        {
            file << header << std::endl;
            // file << "Exec Times in nanonseconds [ns] | [samples:" << this->count << "]" << std::endl;
            // for( const auto& item: *this->vals_array ) file << item << std::endl;
            export_lines( file );
            // for( const auto& item: *this->vals_array ) this->export_line( file );
            file.close();
        }
    }
};

class count_time : counter
{

  private:

    std::chrono::_V2::system_clock::time_point beginning, end;
    // std::string file_name;
    // std::unique_ptr< std::array< int64_t, ARRAY_SIZE > > vals_array;
    // size_t count = 0;

  public:

    count_time( void ) { this->beginning = std::chrono::high_resolution_clock::now(); }

    count_time( const std::string& name ) : counter( name ) {}

    // count_time( const std::string& name ) : counter::file_name( name )
    // {
    //     vals_array = std::make_unique< std::array< int64_t, ARRAY_SIZE > >();
    //     vals_array->fill( 0 );
    // }

    ~count_time()
    {
        std::string header = "Exec Times in nanoseconds [ns] | [samples:" + std::to_string( this->count ) + "]";
        if( this->vals_array ) this->export_vals( header, this->file_name );
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
        this->end = std::chrono::high_resolution_clock::now();

        vec.push_back(
            ( std::chrono::duration_cast< std::chrono::milliseconds >( this->end - this->beginning ) ).count() );

        RCLCPP_DEBUG(
            logger, "%s %ims", str,
            ( std::chrono::duration_cast< std::chrono::milliseconds >( this->end - this->beginning ) ).count() );
    }

    inline void start( void ) { this->beginning = std::chrono::high_resolution_clock::now(); }

    inline void stop( void )
    {
        this->end = std::chrono::high_resolution_clock::now();
        this->append(
            ( std::chrono::duration_cast< std::chrono::nanoseconds >( this->end - this->beginning ) ).count() );
    }

    inline void export_lines( std::ofstream& file )
    {
        for( const auto& item: *this->vals_array ) file << item << std::endl;
    }

    // inline void export_times( const std::string& name )
    // {
    //     std::cout << "Exporting times " << name << "." << std::endl;
    //     std::ofstream file;
    //     file.open( name, std::ios_base::trunc );
    //     if( file.is_open() )
    //     {
    //         file << "Exec Times in nanoseconds [ns] | [samples:" << this->count << "]" << std::endl;
    //         for( const auto& item: *this->vals_array ) file << item << std::endl;
    //         file.close();
    //     }
    // }
};

class count_points : counter
{
  protected:

    int v1, v2;

    std::unique_ptr< std::array< std::tuple< int64_t, int64_t >, ARRAY_SIZE > > vals_array;

  public:

    count_points( const std::string& name ) : counter( name )
    {
        vals_array = std::make_unique< std::array< std::tuple< int64_t, int64_t >, ARRAY_SIZE > >();
        vals_array->fill( std::tuple< int64_t, int64_t >( 0, 0 ) );
    }

    ~count_points()
    {
        std::string header = "PCL Decay | [samples:" + std::to_string( this->count ) + "]";
        if( this->vals_array ) this->export_vals( header, this->file_name );
    }

    void start( int val ) { this->v1 = val; }

    void stop( int val )
    {
        this->v2 = val;
        this->append( this->v1, this->v2 );
    }

    void append( int64_t val1, int64_t val2 )
    {
        this->vals_array->at( count++ ) = std::tuple< int64_t, int64_t >( val1, val2 );
    }

    inline void export_lines( std::ofstream& file )
    {
        for( const auto& item: *this->vals_array )
            file << std::get< 0 >( item ) << " - " << std::get< 1 >( item ) << std::endl;
    }
};

#endif  // SMAP_CORE__COUNT_TIME_HPP_
