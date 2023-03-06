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


namespace flora;

//
// TODO generated message class
//

enum mCommandType
{
    SEND_INVITATION = 1;
    SEND_REQUEST = 2;
    SEND_ACCEPT_ACK = 3;
    SEND_REFUSE_ACK = 4;
    UPDATE_NEIGHBOR_INFO = 5;
    SEND_UPDATE_HEAT = 6;
}

message mCommand extends cMessage {
    inet::MacAddress address;
    int slot;
    
    double PRR;
    simtime_t timeToGW;
}
