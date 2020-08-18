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

#include "extMessageHandler.h"

#include "estnet/common/rapidjson/document.h" // rapidjson's DOM-style API
#include "estnet/common/rapidjson/writer.h"
#include "estnet/common/rapidjson/stringbuffer.h"
#include "estnet/common/rapidjson/filereadstream.h"
#include "estnet/common/rapidjson/error/en.h"

using namespace rapidjson;

namespace estnet {

int extMessageHandler::handleExtMessage(std::string const &message) {

    Document document;

    // Read json from file
    FILE *fp;
    // example_JSON_oneConnection
    if ((fp =
            fopen(
                    (std::string(getenv("HOME"))
                            + std::string(
                                    "/Downloads/example_JSON_oneConnection.json")).c_str(),
                    "r")) == NULL) // non-Windows use "r"
    {
        std::cout << "Error: " << strerror(errno) << endl;
    }

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    if (document.ParseStream(is).HasParseError()) {
        fprintf(stderr, "\nError(offset %u): %s\n",
                (unsigned) document.GetErrorOffset(),
                GetParseError_En(document.GetParseError()));
    }
    fclose(fp);

    // Convert JSON document to string
    StringBuffer strbuf;
    Writer<StringBuffer> writer(strbuf);
    document.Accept(writer);

    for (auto &m : document.GetObject()) {
        // Using a reference for consecutive access is handy and faster.
        const Value &a = document[m.name.GetString()];
    }

    return 0;
}

}  // namespace estnet
