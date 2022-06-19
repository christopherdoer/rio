// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <numeric>

#include "rio_utils/simple_profiler.h"

using namespace rio;

SimpleProfiler::SimpleProfiler(const bool is_on) : kPrefix{"[SimpleProfiler]: "}, is_on_{is_on}, next_id_{0} {}

void SimpleProfiler::start(const std::string& key)
{
  if (is_on_)
    start_times_[key] = std::chrono::system_clock::now();
}

float SimpleProfiler::stopWithRuntimeMs(const std::string& key)
{
  if (stop(key))
    return profile_data_[key].execution_ms.back();
  return 0.0;
}

bool SimpleProfiler::stop(const std::string& key)
{
  if (!is_on_)
    return false;

  auto stop  = std::chrono::system_clock::now();
  auto start = start_times_[key];
  if (start.time_since_epoch().count() == 0)
  {
    ROS_ERROR_STREAM(kPrefix << key << " is ended but has been started.");
    return false;
  }
  else
  {
    const float ms = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count() / 1.0e6f;
    auto& elem     = profile_data_[key];
    elem.execution_ms.emplace_back(ms);
    if (elem.id == -1)
      elem.id = next_id_++;
  }

  return true;
}

RuntimeStatistics SimpleProfiler::getStatistics(const std::string& key)
{
  if (!is_on_)
    return RuntimeStatistics();

  auto data = profile_data_.find(key);

  if (data == profile_data_.end())
  {
    ROS_ERROR_STREAM(kPrefix << key << " has not been profiled.");
    return RuntimeStatistics();
  }

  return calculateProfileStatistics(key);
}

std::string SimpleProfiler::toString(const uint indent)
{
  std::vector<std::string> str_tmp(profile_data_.size());
  RuntimeStatistics overall;
  std::string indent_str(indent + 2, ' ');

  for (const auto& elem : profile_data_)
  {
    std::stringstream ss;
    calculateProfileStatistics(elem.first);
    ss << indent_str << elem.first << ": " << elem.second.statistics.toStringMs() << "\n";
    str_tmp.at(elem.second.id) = ss.str();

    overall.mean_ms += elem.second.statistics.mean_ms;
    overall.max_ms += elem.second.statistics.max_ms;
    overall.min_ms += elem.second.statistics.min_ms;
    overall.total_ms += elem.second.statistics.total_ms;
  }

  std::stringstream ss;
  for (const auto& str : str_tmp) ss << str;

  // overall
  ss << std::string(indent, ' ') << "total"
     << ": " << overall.toStringMs() << "\n";

  return ss.str();
}

std::string SimpleProfiler::toMarkdownTable()
{
  std::vector<std::string> str_tmp(profile_data_.size());
  RuntimeStatistics overall;

  for (const auto& elem : profile_data_)
  {
    std::stringstream ss;
    calculateProfileStatistics(elem.first);
    ss << "|" << elem.first << "|" << elem.second.statistics.mean_ms << "|" << elem.second.statistics.max_ms << "|"
       << elem.second.statistics.min_ms << "|\n";
    str_tmp.at(elem.second.id) = ss.str();

    overall.mean_ms += elem.second.statistics.mean_ms;
    overall.max_ms += elem.second.statistics.max_ms;
    overall.min_ms += elem.second.statistics.min_ms;
  }

  std::stringstream ss;
  ss << "|name|mean_ms|max_ms|min_ms| \n|-----|-----|-----|-----|\n";
  for (const auto& str : str_tmp) ss << str;

  // overall
  ss << "|total|" << overall.mean_ms << "|" << overall.max_ms << "|" << overall.min_ms << "|\n";

  return ss.str();
}

RuntimeStatistics SimpleProfiler::calculateProfileStatistics(const std::string& key)
{
  if (!is_on_)
    return RuntimeStatistics();

  auto data = profile_data_.find(key);

  if (data == profile_data_.end())
  {
    ROS_ERROR_STREAM(kPrefix << key << " has not been profiled.");
    return RuntimeStatistics();
  }
  else
  {
    data->second.statistics.max_ms =
        *std::max_element(data->second.execution_ms.begin(), data->second.execution_ms.end());
    data->second.statistics.min_ms =
        *std::min_element(data->second.execution_ms.begin(), data->second.execution_ms.end());
    data->second.statistics.mean_ms =
        std::accumulate(data->second.execution_ms.begin(), data->second.execution_ms.end(), 0.0f) /
        data->second.execution_ms.size();
    data->second.statistics.total_ms =
        std::accumulate(data->second.execution_ms.begin(), data->second.execution_ms.end(), 0.0f) / 1.0e3;
    return data->second.statistics;
  }
}

float SimpleProfiler::getTotalRuntime()
{
  RuntimeStatistics overall;
  for (const auto& elem : profile_data_)
  {
    calculateProfileStatistics(elem.first);
    overall.total_ms += elem.second.statistics.total_ms;
  }

  return overall.total_ms;
}
