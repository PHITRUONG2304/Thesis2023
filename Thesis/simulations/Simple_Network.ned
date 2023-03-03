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

package flora.simulations;

import inet.applications.udpapp.UdpBasicApp;
import inet.node.inet.Router;
import inet.node.internetcloud.InternetCloud;
import flora.LoRaPhy.LoRaMedium;
import flora.LoraNode.LoRaNode;
import inet.node.inet.StandardHost;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth1G;
import inet.common.scenario.ScenarioManager;
import inet.visualizer.canvas.integrated.IntegratedCanvasVisualizer;
import inet.visualizer.contract.IIntegratedVisualizer;
//
// TODO auto-generated type
//
network Simple_Network
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(40000);
        int networkSizeY = default(40000);
        @display("bgb=40000,20000");
    submodules:
        Nodes[numberOfNodes]: LoRaNode {
            @display("p=5001,2720.544;is=s");
        }
        Gateway[numberOfGateways]: LoRaNode {
            @display("bgb=783.96,587.688;i=device/antennatower;is=m;p=19801.979,11401.14");
        }
        LoRaMedium: LoRaMedium {
            @display("p=19301.93,2150.2148");
        }
        scenarioManager: ScenarioManager {
            @display("p=15501.55,2200.22");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=11751.175,2200.22");
        }
//        visualizer: <default(firstAvailableOrEmpty("IntegratedCanvasVisualizer"))> like IIntegratedVisualizer if typename != "" {
//            @display("p=22802.28,2200.22");
//        }

    connections:
}
