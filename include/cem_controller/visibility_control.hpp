// Copyright 2024 MAINTAINER
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CEM_CONTROLLER__VISIBILITY_CONTROL_HPP_
#define CEM_CONTROLLER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(CEM_CONTROLLER_BUILDING_DLL) || defined(CEM_CONTROLLER_EXPORTS)
    #define CEM_CONTROLLER_PUBLIC __declspec(dllexport)
    #define CEM_CONTROLLER_LOCAL
  #else  // defined(CEM_CONTROLLER_BUILDING_DLL) || defined(CEM_CONTROLLER_EXPORTS)
    #define CEM_CONTROLLER_PUBLIC __declspec(dllimport)
    #define CEM_CONTROLLER_LOCAL
  #endif  // defined(CEM_CONTROLLER_BUILDING_DLL) || defined(CEM_CONTROLLER_EXPORTS)
#elif defined(__linux__)
  #define CEM_CONTROLLER_PUBLIC __attribute__((visibility("default")))
  #define CEM_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define CEM_CONTROLLER_PUBLIC __attribute__((visibility("default")))
  #define CEM_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // CEM_CONTROLLER__VISIBILITY_CONTROL_HPP_
