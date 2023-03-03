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

import inet.applications.contract.IApp;
import flora.LoRaApp.TDMA;
import flora.LoRaApp.Deliverer;
import flora.LoRaApp.ReviseHEAT;
import flora.LoRaApp.Container;

//
// TODO auto-generated module
//
module SimpleLoRaApp like IApp
{
    gates:
        input socketIn @labels(LoRaAppPacket/up);
        output socketOut @labels(LoRaAppPacket/down);

    submodules:
        TDMA: TDMA {
            @display("p=127,90;i=block/cogwheel");
        }

        Deliverer: Deliverer {
            @display("p=127,190;i=block/transport");
        }

        ReviseHEAT: ReviseHEAT {
            @display("p=32,90;i=block/app");
        }

        Container: Container {
            @display("p=227,90;i=block/buffer");
        }

    connections allowunconnected:
        socketOut <-- Deliverer.socketOut;
        socketIn --> TDMA.socketIn;
        
        TDMA.Container_O --> Container.TDMA_I;
        TDMA.Heat_O --> ReviseHEAT.TDMA_I;

        Deliverer.TDMA_I <-- TDMA.Deliverer_O;
        Deliverer.Heat_I <-- ReviseHEAT.Deliverer_O;
		Deliverer.Container_I <-- Container.Deliverer_O;
}
