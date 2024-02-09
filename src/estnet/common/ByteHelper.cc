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

#include "ByteHelpers.h"
#include "estnet/common/ESTNETDefs.h"

namespace estnet {

void ESTNET_API uint16ToBytes(uint16_t i, uint8_t *bytes) {
    bytes[0] = (i >> 8) & 0xFF;
    bytes[1] = (i >> 0) & 0xFF;
}

void ESTNET_API uint32ToBytes(uint32_t i, uint8_t *bytes) {
    bytes[0] = (i >> 24) & 0xFF;
    bytes[1] = (i >> 16) & 0xFF;
    bytes[2] = (i >> 8) & 0xFF;
    bytes[3] = (i >> 0) & 0xFF;
}

void ESTNET_API int32ToBytes(int32_t i, uint8_t *bytes) {
    uint32ToBytes(i, bytes);
}

uint16_t ESTNET_API bytesToUint16(uint8_t *bytes) {
    uint16_t i = 0;
    i += ((bytes[0] & 0xFF) << 8);
    i += ((bytes[1] & 0xFF) << 0);
    return i;
}

}  // namespace estnet
