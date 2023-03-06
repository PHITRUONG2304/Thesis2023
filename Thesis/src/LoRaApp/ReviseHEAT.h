/*
 * ReviseHEAT.h
 *
 *  Created on: Feb 26, 2023
 *      Author: PHITRUONG
 */

#ifndef __REVISEHEAT_H__
#define __REVISEHEAT_H__

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"

#include "LoRaAppPacket_m.h"
#include "TDMA.h"
#include "Deliverer.h"
#include "LoRa/LoRaRadio.h"
#include "mCommand_m.h"
#include <queue>

using namespace omnetpp;
using namespace inet;
using namespace std;

namespace flora
{

    struct HEAT_Field
    {
        double PRR;
        simtime_t timeToGW;
    };

    struct Neighbor_Info
    {
        MacAddress addr;
        double PRR;
        HEAT_Field heatValue;
        simtime_t timestamp;
    };

    class Container;

    class ReviseHEAT : public cSimpleModule, public ILifecycle
    {
    private:
        bool iAmGateway;
        int max_neighbors;
        HEAT_Field current_HEAT;
        int currentNeighbors;
        Neighbor_Info *neighbor_Table;
        int currentPath = -1;
        bool updatedInCurrentSlot = false;

        TDMA *myTDMA;
        Deliverer *myDeliverer;
        Container *myContainer;


    protected:
        void initialize(int stage) override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void finish() override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        virtual void handleMessage(cMessage *msg) override;
        void calculateHEATField();
        void sortNeighborTable();

    public:
        HEAT_Field getCurrentHEAT();
        void updateNeighborTable(MacAddress addr, double PRR, simtime_t timeToGW, simtime_t timestamp);
        void addNewNeighborIntoTable(MacAddress addr, double PRR, simtime_t timeToGW, simtime_t timestamp);
        int isAlreadyExistNeighbor(MacAddress addr);
        MacAddress getCurrentPathToGW();
        bool canUpdate() { return (current_HEAT.PRR != 0) ? true : false; }
        void printTable();
        simtime_t getCurrentTimeToGW();
    };

}

#endif /* LORAAPP_REVISEHEAT_H_ */
