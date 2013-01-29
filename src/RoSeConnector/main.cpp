/*
 * Copyright (c) 2013, Fraunhofer FKIE
 *
 * Authors: Bastian Gaspers
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the Fraunhofer FKIE nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * This file is part of the StructureColoring ROS package.
 *
 * The StructureColoring ROS package is free software:
 * you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The StructureColoring ROS package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with The StructureColoring ROS package.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <structureColoring/RoSeConnector/StrColRoSeService.h>

static RoSe::Service::Ptr service;

static void ctrlc(int)
{
  if (service.ptr() != 0) service->dispose();
  std::cout << "CTRL-C pressed" << std::endl;
}

int main (int argc, char** argv)
{
  struct sigaction sa;
  memset (&sa, 0, sizeof(sa));
  sa.sa_handler = ctrlc;
  sa.sa_flags = SA_RESETHAND;
  sigaction (SIGINT, &sa, 0);
  sigaction (SIGTERM, &sa, 0);
  int ac=1;
  ros::init(ac, argv, "StructureColoring_RoSeService");

  try {
    service = RoSe::Service::create (argc, argv, StrColRoSeService::create);
    service->suspend();
    return 0;
  } catch (RoSe::ConfigError& e) {
    std::cerr << "Configuration Error: " << e.what() << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
  return 1;
}
