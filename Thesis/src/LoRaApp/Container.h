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
#include "StatisticalModule.h"

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
        uint32_t prevSeqNum = INT32_MAX;

        MacAddress waitingAckFrom = MacAddress::UNSPECIFIED_ADDRESS;
        int sentTimes = 0;
        int maxSendTimes;

        cMessage *generatePacket;
        cMessage *timeoutWaitACK;

//        For statistics
        LoRaMac *macLayer;
        int generatedPacketNum = 0;
        int generatedPacketFailNum = 0;
        StatisticalModule *sMachine;

    protected:
        void initialize(int stage) override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void finish() override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        void handleMessage(cMessage *msg) override;
        void handleSelfMessage(cMessage *msg);
        void handleWithFsm(cMessage *msg);

    public:
        void generatingPacket();

        bool disPatchPacket(LoRaAppPacket *packet);
        bool disPatchPacket(Packet *packet);

        bool forwardDataPacket(MacAddress addr, /*for statics*/ bool again = false);

        void sendACKCommand(Packet *pkt, bool status);

        bool updateNeighborInfo(Packet *pkt);
        bool updateSentPacketState(MacAddress addr, bool state);
//        for statistic
        void updateStatisticalData(Packet *packet);
    };

}

#endif /* LORAAPP_CONTAINER_H_ */
