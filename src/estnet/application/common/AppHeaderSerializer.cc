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

#include "AppHeaderSerializer.h"

#include <inet/common/packet/serializer/ChunkSerializerRegistry.h>

namespace estnet {

Register_Serializer(AppHeader, AppHeaderSerializer);

void AppHeaderSerializer::serialize(MemoryOutputStream &stream,
        const Ptr<const Chunk> &chunk) const {
    auto appHeader = staticPtrCast<const AppHeader>(chunk);
    stream.writeUint16Be(appHeader->getSourceAppID());
    stream.writeUint16Be(appHeader->getDestAppID());
    stream.writeUint32Be(appHeader->getPktId());
}

const Ptr<Chunk> AppHeaderSerializer::deserialize(
        MemoryInputStream &stream) const {
    auto appHeader = makeShared<AppHeader>();
    appHeader->setSourceAppID(stream.readUint16Be());
    appHeader->setDestAppID(stream.readUint16Be());
    appHeader->setPktId(stream.readUint32Be());
    return appHeader;
}

}  // namespace estnet
