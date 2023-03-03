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

package flora.LoraNode;

import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;
import inet.applications.contract.IApp;
import flora.LoRa.LoRaNic;
import inet.common.lifecycle.NodeStatus;


module LoRaGW
{
    parameters:
        int numApps = default(0);
        string deploymentType = default("");
        double maxGatewayDistance = default(320.0);
        double gatewayX = default(320.0);
        double gatewayY = default(320.0);      
        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        @display("bgb=482,313;i=device/accesspoint;is=vs");
        @statistic[coordX](title="coord X"; source="xCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
        @statistic[coordY](title="coord Y"; source="yCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
        @statistic[coordZ](title="coord Z"; source="zCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
        bool hasStatus = default(false);
    submodules:
        interfaceTable: InterfaceTable {
            @display("p=63,49");
        }
        status: NodeStatus if hasStatus {
            @display("p=63,151;is=s");
        }
        mobility: <default("StationaryMobility")> like IMobility {
            @display("p=63,103;is=s");
        }
        LoRaNic: LoRaNic {
            @display("p=137,239");
        }
        app[numApps]: <default("SimpleLoRaAppGW")> like IApp {
            parameters:
                @display("p=378,135,row,150");
        }
    connections allowunconnected:
        for i=0..numApps-1 {
            app[i].socketOut --> LoRaNic.upperLayerIn;
            LoRaNic.upperLayerOut --> app[i].socketIn;
        }
}
