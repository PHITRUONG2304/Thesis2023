package flora.simulations;

import inet.applications.udpapp.UdpBasicApp;
import inet.node.inet.Router;
import inet.node.internetcloud.InternetCloud;
import flora.LoRaPhy.LoRaMedium;
import flora.LoraNode.LoRaNode;
import flora.LoraNode.LoRaGW;
import inet.node.inet.StandardHost;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth1G;

@license(LGPL);
network LoRaNetworkTest
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(500);
        int networkSizeY = default(500);
        @display("bgb=562,417");
    submodules:
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=400,304");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=157,238;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=400,148");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=318,27");
        }
    connections:
}