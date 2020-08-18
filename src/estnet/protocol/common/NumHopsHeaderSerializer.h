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

#ifndef __PROTOCOLS_NUM_HOPS_HEADER_SERIALIZER_H
#define __PROTOCOLS_NUM_HOPS_HEADER_SERIALIZER_H

#include <inet/common/packet/serializer/FieldsChunkSerializer.h>
#include "NumHopsHeader_m.h"

using namespace inet;

namespace estnet {

/**
 * Converts between CompassAbstractProtocolHeader and binary network byte order representation of the header.
 */
class ESTNET_API NumHopsHeaderSerializer: public FieldsChunkSerializer {
protected:
    virtual void serialize(MemoryOutputStream &stream,
            const Ptr<const Chunk> &chunk) const override;
    virtual const Ptr<Chunk> deserialize(MemoryInputStream &stream) const
            override;

public:
    NumHopsHeaderSerializer() :
            FieldsChunkSerializer() {
    }
};

}  // namespace estnet

#endif
