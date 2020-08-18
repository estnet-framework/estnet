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

#include "TextMessageApp.h"

namespace estnet {

Define_Module(TextMessageApp);

TextMessageApp::TextMessageApp() :
        _wrongTextMessagesRcvd(0) {

}

void TextMessageApp::initialize(int stage) {
    BasicApp::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL) {
        this->_textMessage = std::string(par("textMessage").stringValue());

        for (int i = 0; i < par("multiplier").intValue() - 1; i++) {
            this->_textMessage += std::string(par("textMessage").stringValue());
        }
    }
}

void TextMessageApp::finish() {
    BasicApp::finish();
    EV_INFO << "Received packet with wrong message: "
                   << this->_wrongTextMessagesRcvd << endl;

}

void TextMessageApp::recvPacketInternal(omnetpp::cMessage *msg) {
    BasicApp::recvPacketInternal(msg);
    auto pkt = recvPacket(msg);
    auto numHopsHeader = pkt->popAtBack<NumHopsHeader>(inet::B(1));
    auto payload = pkt->peekDataAsBytes()->getBytes();
    auto message = std::string(payload.begin(), payload.end());
    auto expectedMessage = par("expectedMessage").stdstringValue();
    for (int i = 0; i < par("multiplier").intValue() - 1; i++) {
        expectedMessage += std::string(par("expectedMessage").stringValue());
    }

    EV_INFO << "Received packet with following message: " << message << endl;
    if (strcmp(message.c_str(), expectedMessage.c_str()) != 0) {
        EV_ERROR << "This is not the expected message!" << endl;
        this->_wrongTextMessagesRcvd++;
    }

}

inet::Packet* TextMessageApp::generatePacketPayload() {
    // dummy packet without real payload data
    std::vector<uint8_t> payload;
    for (size_t i = 0; i < _textMessage.length(); i++) {
        payload.push_back((uint8_t) _textMessage[i]);
    }
    auto data = inet::makeShared<inet::BytesChunk>(payload);
    inet::Packet *newPacket = new inet::Packet();
    newPacket->insertAtBack(data);
    return newPacket;
}

} //namespace
