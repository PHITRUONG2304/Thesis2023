/*
 * StatisticalModule.h
 *
 *  Created on: Mar 10, 2023
 *      Author: PHITRUONG
 */

#ifndef LORAAPP_STATISTICALMODULE_H_
#define LORAAPP_STATISTICALMODULE_H_


#include <omnetpp.h>
#include "inet/common/packet/Packet.h"
#include "LoRaAppPacket_m.h"
#include <queue>

using namespace omnetpp;
using namespace inet;
using namespace std;

namespace flora
{
    struct StatisticalInformation
    {
        MacAddress address;
        int numReceived;
        int numSent;
        int numResent;
        int numSentSucess;
        int maxCurrentHops;
        int *numReceiveAccordingToHops;


    };
    class NeighborTable;
    class StatisticalModule : public cSimpleModule
    {

    private:
        int maxNeighbors;
        int currentNeighbors;
        StatisticalInformation *neighbors;
        NeighborTable *neighborInfo;

        void printNeighborsInformation();
        int isAlreadyExist(MacAddress address);
        void ensureHopNumStorge(MacAddress address, int hopNum);
        int addNewNeighbor(MacAddress address);

    protected:
        void initialize(int stage) override;
        void finish() override;

    public:
        void updateStatisticalLoRaData(MacAddress address, bool sucess, bool again = false);
        void updateStatisticalLoRaData(MacAddress address, int hopNum);

    };

}


#endif /* LORAAPP_STATISTICALMODULE_H_ */
