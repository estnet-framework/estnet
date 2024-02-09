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

#ifndef __UTILS_BYTE_HELPERS_H__
#define __UTILS_BYTE_HELPERS_H__

#include <stdint.h>

namespace estnet {

/** @brief Converts the given uint16 to individual bytes,
 *  assumes bytes array has 2 bytes of memory allocated
 */
void uint16ToBytes(uint16_t i, uint8_t *bytes);
/** @brief Converts the given uint32 to individual bytes,
 *  assumes bytes array has 4 bytes of memory allocated
 */
void uint32ToBytes(uint32_t i, uint8_t *bytes);
/** @brief Converts the given int32 to individual bytes,
 *  assumes bytes array has 4 bytes of memory allocated
 */
void int32ToBytes(int32_t i, uint8_t *bytes);

/** @brief Converts the the first two bytes of the given
 *  byte array to an uint16
 */
uint16_t bytesToUint16(uint8_t *bytes);

}  // namespace estnet

#endif
