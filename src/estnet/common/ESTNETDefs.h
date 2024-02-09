//
// Copyright (C) 2020 Computer Science VII: Robotics and Telematics -
// Julius-Maximilians-Universitaet Wuerzburg
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
#ifndef ESTNET_DEFS_H_
#define ESTNET_DEFS_H_



#ifndef CONTACT_PLAN_TOOLS
#include <omnetpp.h>

#include "estnet/global_config.h"

#endif
#if defined(ESTNET_EXPORT)
#  define ESTNET_API OPP_DLLEXPORT
#elif defined(TESTLIB_IMPORT)
#  define ESTNET_API OPP_DLLIMPORT
#else
#  define ESTNET_API
#endif

#endif /* ESTNET_DEFS_H_ */
