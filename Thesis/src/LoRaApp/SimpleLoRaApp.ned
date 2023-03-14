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
import flora.LoRaApp.ReviseHEAT;
import flora.LoRaApp.Container;
import flora.LoRaApp.Receiver;
import flora.LoRaApp.Transmitter;
import flora.LoRaApp.NeighborTable;

//
// TODO auto-generated module
//
module SimpleLoRaApp like IApp
{
    gates:
        input socketIn @labels(LoRaAppPacket/up);
        output socketOut @labels(LoRaAppPacket/down);

    submodules:
        Receiver: Receiver {
            @display("p=142,66;i=block/transport");
        }

        TDMA: TDMA {
            @display("p=142,158;i=block/cogwheel");
        }

        ReviseHEAT: ReviseHEAT {
            @display("p=43,158;i=block/app");
        }

        Container: Container {
            @display("p=248,158;i=block/buffer");
        }

        Transmitter: Transmitter {
            @display("p=142,262;i=block/transport");
        }

        NeighborTable: NeighborTable {
            @display("p=407,262;i=block/table2");
        }

        StatisticalModule: StatisticalModule {
            @display("p=407,33;i=device/pc");
        }

    connections allowunconnected:
        socketIn --> Receiver.lowerLayerIn;

        Receiver.To_Heat --> ReviseHEAT.From_Receiver;
        Receiver.To_TDMA --> TDMA.From_Receiver;
        Receiver.To_Container --> Container.From_Receiver;

        TDMA.To_Container --> Container.From_TDMA;
        TDMA.To_Heat --> ReviseHEAT.From_TDMA;

        ReviseHEAT.To_Transmitter --> Transmitter.From_Heat;
        TDMA.To_Transmitter --> Transmitter.From_TDMA;
        Container.To_Transmitter --> Transmitter.From_Container;

        Transmitter.lowerLayerOut --> socketOut;

}
