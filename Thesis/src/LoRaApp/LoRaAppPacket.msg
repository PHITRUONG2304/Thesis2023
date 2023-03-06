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

//
// TODO generated message class
//

import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;
import inet.linklayer.common.MacAddress;

namespace flora;

enum AppPacketType
{
    JOIN_INVITE = 1;
    JOIN_REQUEST = 2;
    JOIN_ACCEPT_REPLY = 3;
    JOIN_REFUSE_REPLY = 4;
    HEAT_INFO = 5;
    DATA_PACKET = 6;
    FORWARD_PACKET = 7;
    UNABLE_RECEIVE_MORE = 8;
    RECEIVED_SUCCESS = 9;
}

class LoRaOptions {
    double LoRaTP = -1;
    double LoRaCF = -1;
    int LoRaSF = -1;
    double LoRaBW = -1;
    int LoRaCR = -1;
    bool UseHeader = true;
    bool ADRACKReq = false;
}

class LoRaAppPacket extends inet::FieldsChunk {
    int msgType @enum(AppPacketType);
    simtime_t generatedTime;
    double PRR;
    simtime_t timeToGW;
    int slot; 
    int payload;
    uint32_t seqNum;
    int hopNum;
    inet::MacAddress originAddress;
    LoRaOptions options;
}
