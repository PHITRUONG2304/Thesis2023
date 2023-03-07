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
            @display("p=141,35;i=block/cogwheel");
        }

        Deliverer: Deliverer {
            @display("p=141,188;i=block/transport");
        }

        ReviseHEAT: ReviseHEAT {
            @display("p=40,95;i=block/app");
        }

        Container: Container {
            @display("p=245,95;i=block/buffer");
        }

    connections allowunconnected:

        Deliverer.lowerLayerOut --> socketOut;
        Deliverer.lowerLayerIn <-- socketIn;

        Deliverer.To_Container --> Container.From_Deliverer;
        Deliverer.To_Heat --> ReviseHEAT.From_Deliverer;
        Deliverer.To_TDMA --> TDMA.From_Deliverer;

        Deliverer.From_Container <-- Container.To_Deliverer;
        Deliverer.From_Heat <-- ReviseHEAT.To_Deliverer;
        Deliverer.From_TDMA <-- TDMA.To_Deliverer;

        TDMA.To_Container --> Container.From_TDMA;
        TDMA.To_Heat --> ReviseHEAT.From_TDMA;

}
