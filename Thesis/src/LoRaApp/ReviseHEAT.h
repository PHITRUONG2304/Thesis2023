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
#include "NeighborTable.h"
#include "LoRa/LoRaRadio.h"
#include "mCommand_m.h"
#include <queue>

using namespace omnetpp;
using namespace inet;
using namespace std;

namespace flora
{

    class Container;

    struct CurrentHEAT{
        MacAddress addr;
        HEAT_Field currentValue;
    };

    class ReviseHEAT : public cSimpleModule, public ILifecycle
    {
    private:
        bool iAmGateway;
        CurrentHEAT currentHEAT;
        NeighborTable *neighborTable;
        int currentPath = -1;
        int sendAgainTimes = 0;

        TDMA *myTDMA;
        Container *myContainer;

        mCommand *updateAgain;


    protected:
        void initialize(int stage) override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void finish() override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        virtual void handleMessage(cMessage *msg) override;
//        Tinh toan gia tri HEAT hien tai
        void calculateHEATField();
        void sortNeighborTable(NeighborHEATTable *table);

    public:
        CurrentHEAT getCurrentHEAT() { return this->currentHEAT; }
        void updateNeighborTable(MacAddress addr, double PRR, simtime_t timeToGW);
        void addNewNeighborIntoTable(MacAddress addr, double PRR, simtime_t timeToGW, simtime_t timestamp);
        int isAlreadyExistNeighbor(MacAddress addr);
        MacAddress getCurrentPathToGW();
//        Kiem tra xem hien tai co the gui thong tin HEAT cho cac neighbor khac hay chua
        bool canUpdate() { return (currentHEAT.addr != MacAddress::UNSPECIFIED_ADDRESS) ? true : false; }
//        Lay thoi gian goi tin den duoc gateway (bao gom waiting time and transmission time)
        simtime_t getCurrentTimeToGW();
    };

}

#endif /* LORAAPP_REVISEHEAT_H_ */
