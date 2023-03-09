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
import inet.linklayer.common.MacAddress;
import LoRaAppPacket;

namespace flora;


//
// TODO generated message class
//

enum mCommandType
{
    SEND_INVITATION			= 101;
    SEND_REQUEST			= 102;
    SEND_ACCEPT_ACK			= 103;
    SEND_REFUSE_ACK			= 104;
    UPDATE_NEIGHBOR_INFO	= 105;
    SEND_UPDATE_HEAT		= 106;
    START_GENERATE_PACKET	= 107;
    SEND_DATA_PACKET		= 108;
    SEND_ACK_SUCCESS		= 109;
    SEND_ACK_FAIL			= 110;
}

message mCommand extends cMessage {
    inet::MacAddress address;
//    for setup communication slot
    int slot;
    
//    for update HEAT and broadcast information
    double PRR;
    simtime_t timeToGW;
    
//    for Data packet
    int payload;
    uint32_t seqNum;
    int hopNum;
    LoRaAppPacket *data;
}
