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
        double timeout_request @unit(s) = default(200ms);
        double transmittion_time @unit(s) = default(80ms);
        
    gates:
        input socketIn @labels(TDMA/lowerIn);
        output Deliverer_O @labels(TDMA/TDMAtoDELIVERER);
        output Heat_O @labels(TDMA/TMDAtoHEAT);
        output Container_O @labels(TDMA/TDMAtoCONTAINER);
}

