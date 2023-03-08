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
        DISPATCHING,
        ACK_FAIL,
        ACK_SUCCESS,
        FORWARDING
    };

    class Container : public cSimpleModule, public ILifecycle
    {
    private:
        int max_communication_neighbors;
        int currentNeighbors;
        NeighborTable *neighborTable;
        State fsmState = NORMAL;
        ReviseHEAT *myHEATer;

        cMessage *generatePacket;

    protected:
        void initialize(int stage) override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void finish() override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        void handleMessage(cMessage *msg) override;

    public:
        void handleWithFsm(cMessage *msg);
        bool disPatchPacket(LoRaAppPacket *packet);
        bool disPatchPacket(Packet *packet);
        void generatingPacket();
        bool canAddMore(MacAddress addr){ return this->neighborTable->isFull(addr); }
        bool forwardDataPacket(MacAddress addr);
    };

}

#endif /* LORAAPP_CONTAINER_H_ */
