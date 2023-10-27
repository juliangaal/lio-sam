//
// Created by xiang on 2021/10/11.
//

#ifndef FASTER_LIO_UTILS_H
#define FASTER_LIO_UTILS_H

#include <glog/logging.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <string>

#include "color.h"

namespace faster_lio
{
/// timer
class Timer
{
public:
  struct TimerRecord
  {
    TimerRecord() = default;
    TimerRecord( const std::string& name, double time_usage )
    {
      func_name_ = name;
      time_usage_in_ms_.emplace_back( time_usage );
    }
    std::string         func_name_;
    std::vector<double> time_usage_in_ms_;
  };

  /**
     * call F and save its time usage
     * @tparam F
     * @param func
     * @param func_name
     */
  template <class F>
  static void Evaluate( F&& func, const std::string& func_name )
  {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::forward<F>( func )();
    auto t2        = std::chrono::high_resolution_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2 - t1 ).count() * 1000;

    if ( records_.find( func_name ) != records_.end() )
    {
      records_[ func_name ].time_usage_in_ms_.emplace_back( time_used );
    }
    else
    {
      records_.insert( { func_name, TimerRecord( func_name, time_used ) } );
    }

    std::cout << func_name << " ----> " << time_used << "\n";
  }

  /// print the run time
  static void PrintAll()
  {
    std::cout << BOLDGREEN << ">>> ===== Printing run time =====" << std::endl;
    for ( const auto& r : records_ )
    {
      std::cout << "> [ " << r.first << " ] average time usage: "
                << std::accumulate( r.second.time_usage_in_ms_.begin(), r.second.time_usage_in_ms_.end(), 0.0 ) / double( r.second.time_usage_in_ms_.size() )
                << " ms , called times: " << r.second.time_usage_in_ms_.size() << "\n";
    }
    std::cout << ">>> ===== Printing run time end =====" << RESET << std::endl;
  }

  /// dump to a log file
  static void DumpIntoFile( const std::string& file_name )
  {
    std::ofstream ofs( file_name, std::ios::out );
    if ( !ofs.is_open() )
    {
      LOG( ERROR ) << "Failed to open file: " << file_name;
      return;
    }
    else
    {
      LOG( INFO ) << "Dump Time Records into file: " << file_name;
    }

    size_t max_length = 0;
    for ( const auto& iter : records_ )
    {
      ofs << iter.first << ", ";
      if ( iter.second.time_usage_in_ms_.size() > max_length )
      {
        max_length = iter.second.time_usage_in_ms_.size();
      }
    }
    ofs << std::endl;

    for ( size_t i = 0; i < max_length; ++i )
    {
      for ( const auto& iter : records_ )
      {
        if ( i < iter.second.time_usage_in_ms_.size() )
        {
          ofs << iter.second.time_usage_in_ms_[ i ] << ",";
        }
        else
        {
          ofs << ",";
        }
      }
      ofs << std::endl;
    }
    ofs.close();
  }

  /// get the average time usage of a function
  static double GetMeanTime( const std::string& func_name )
  {
    if ( records_.find( func_name ) == records_.end() )
    {
      return 0.0;
    }

    auto r = records_[ func_name ];
    return std::accumulate( r.time_usage_in_ms_.begin(), r.time_usage_in_ms_.end(), 0.0 ) /
           double( r.time_usage_in_ms_.size() );
  }

  /// clean the records
  static void Clear() { records_.clear(); }

private:
  static std::map<std::string, TimerRecord> records_;
};

}  // namespace faster_lio


namespace lin
{
class Timer
{
private:
  std::chrono::high_resolution_clock::time_point start, end;
  std::chrono::duration<double, std::milli>      time_consumed_ms;

public:
  double time_consumed_ms_double;
  Timer()
  {
  }

  ~Timer()
  {
  }

  void tic()
  {
    start = std::chrono::high_resolution_clock::now();
  }

  void toc()
  {
    end                     = std::chrono::high_resolution_clock::now();
    time_consumed_ms        = end - start;
    time_consumed_ms_double = time_consumed_ms.count();
  }

  void show()
  {
    std::cout << "Time: " << time_consumed_ms.count() << "\t\tms" << std::endl;
  }
};
}  // namespace lin

#endif  // FASTER_LIO_UTILS_H
