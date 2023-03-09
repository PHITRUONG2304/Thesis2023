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

simple Container
{
    parameters:
        int max_neighbors = default(10);
		int queue_length = default(500);
		double generate_packet_interval @unit(s) = default(1000s);
		double start_generate @unit(s) = default(5000s);
		double timeoutACK @unit(s) = default(200ms);
		int send_again_times = default(3);
		bool iAmGateway = default(false);
		
	gates:
        input From_Receiver @labels(Container/R2C);
        input From_TDMA @labels(Container/T2C);
        output To_Transmitter@labels(Container/C2D);
        
}
