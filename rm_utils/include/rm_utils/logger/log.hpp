// Created by Chengfu Zou
// Copyright (C) Aurora Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_UTILS_LOGGER_LOGGER_HPP_
#define RM_UTILS_LOGGER_LOGGER_HPP_

#include <rm_utils/logger/logger_pool.hpp>
#include <rm_utils/logger/types.hpp>

#define AURORA_REGISTER_LOGGER(name, path, level)                           \
  do {                                                                      \
    aurora::logger::LoggerPool::registerLogger(                             \
      name, path, aurora::logger::LogLevel::level, DATE_DIR | DATE_SUFFIX); \
  } while (0)

#define AURORA_LOG(name, level, ...)                                     \
  do {                                                                   \
    aurora::logger::LoggerPool::getLogger(name).log(level, __VA_ARGS__); \
  } while (0)

#define AURORA_DEBUG(name, ...) AURORA_LOG(name, aurora::logger::LogLevel::DEBUG, __VA_ARGS__)

#define AURORA_INFO(name, ...) AURORA_LOG(name, aurora::logger::LogLevel::INFO, __VA_ARGS__)

#define AURORA_WARN(name, ...) AURORA_LOG(name, aurora::logger::LogLevel::WARN, __VA_ARGS__)

#define AURORA_ERROR(name, ...) AURORA_LOG(name, aurora::logger::LogLevel::ERROR, __VA_ARGS__)
#define AURORA_FATAL(name, ...) AURORA_LOG(name, aurora::logger::LogLevel::FATAL, __VA_ARGS__)

#endif  // RM_UTILS_LOGGER_LOGGER_HPP_
