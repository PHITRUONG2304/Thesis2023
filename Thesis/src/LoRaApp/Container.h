/*
 * Container.h
 *
 *  Created on: Feb 25, 2023
 *      Author: PHITRUONG
 */

#ifndef __CONTAINER_H__
#define __CONTAINER_H__

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"

#include "LoRaAppPacket_m.h"
#include "mCommand_m.h"
#include "TDMA.h"
#include "ReviseHEAT.h"
#include "NeighborTable.h"
#include "../LoRa/LoRaMac.h"

#include <queue>

using namespace omnetpp;
using namespace inet;
using namespace std;

namespace flora
{
    class ReviseHEAT;

    enum State
    {
        NORMAL,
        WAITINGACK
    };

    class Container : public cSimpleModule, public ILifecycle
    {
    private:
        int max_communication_neighbors;
        int currentNeighbors;
        NeighborTable *neighborTable;
        State fsmState = NORMAL;
        ReviseHEAT *myHEATer;
        bool iAmGateway;

        MacAddress waitingAckFrom = MacAddress::UNSPECIFIED_ADDRESS;
        int sentTimes = 0;

        cMessage *generatePacket;
        cMessage *timeoutWaitACK;

//        For statistics
        LoRaMac *macLayer;
        int rPacketNum = 0;

    protected:
        void initialize(int stage) override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void finish() override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        void handleMessage(cMessage *msg) override;
        void handleSelfMessage(cMessage *msg);
        void stateTransitions(cMessage *msg);
        void handleWithFsm(cMessage *msg);

    public:
        bool disPatchPacket(LoRaAppPacket *packet);
        bool disPatchPacket(Packet *packet);
        void generatingPacket();
        bool canAddMore(MacAddress addr){ return this->neighborTable->isFull(addr); }
        bool forwardDataPacket(MacAddress addr);
        bool forwardDataPacket(Packet *pkt);
        void sendACKCommand(Packet *pkt, bool status);
        bool updateNeighborInfo(Packet *pkt);
        bool updateSendPacketState(MacAddress addr, int state);
    };

}

#endif /* LORAAPP_CONTAINER_H_ */
