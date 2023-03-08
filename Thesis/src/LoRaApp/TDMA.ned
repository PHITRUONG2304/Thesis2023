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

package flora.LoRaApp;

simple TDMA
{
    parameters:
        int max_neighbors = default(10);
        double timeslot_size @unit(s) = default(5000ms);
        double timeout_grant @unit(s) = default(500ms);
        double send_again_interval @unit(s) = default(200ms);
        double transmittion_time @unit(s) = default(80ms);
        int send_again_times = default(3);
        
    gates:        
        input From_Receiver @labels(TDMA/R2T);
        
        output To_Transmitter @labels(TDMA/T2D);
        output To_Heat @labels(TDMA/T2H);
        output To_Container @labels(TDMA/T2C);
}

