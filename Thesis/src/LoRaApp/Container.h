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
#include "TDMA.h"
#include "ReviseHEAT.h"
#include "LoRa/LoRaRadio.h"
#include <queue>

using namespace omnetpp;
using namespace inet;
using namespace std;

namespace flora
{
    enum ADD_PACKET_RESULT
    {
        FAIL,
        SUCCESS,
        EXISTED
    };

    class ReviseHEAT;

    class DataQueue
    {
    private:
        uint32_t length;
        uint32_t maxlength;
        queue<LoRaAppPacket> receivedPackets;
        vector<uint32_t> receivedSeqNumber;

        bool isPacketAlreadyExist(uint32_t seqNum);
        bool isPacketAlreadyExist(LoRaAppPacket packet);

    public:
        bool isFull() { return this->maxlength == this->length; }
        bool isEmpty() { return this->length == 0; }
        DataQueue(uint32_t maxlength = 50);
        ADD_PACKET_RESULT addPacket(LoRaAppPacket packet);
        LoRaAppPacket peekPacket();
        void removePacket(uint32_t index = 0);
    };

    enum State
    {
        NORMAL,
        DISPATCHING,
        ACK_FAIL,
        ACK_SUCCESS,
        DISPATCH_AGAIN
    };

    struct Container_El
    {
        MacAddress addr;
        DataQueue Dqueue;
    };

    class Container : public cSimpleModule, public ILifecycle
    {
    private:
        uint8_t max_communication_neighbors;
        uint8_t currentNeighbors;
        Container_El *neighborData;
        State fsmState = NORMAL;
        ReviseHEAT *myHEATer;

    protected:
        void initialize(int stage) override;
        void finish() override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        void handleMessage(cMessage *msg) override;

    public:
        void handleWithFsm(LoRaAppPacket *packet = NULL);
        void handleWithFsmWhenReceivePacket(LoRaAppPacket *packet);
        bool disPatchPacket(LoRaAppPacket packet);
        void generatingPacket();
        bool canAddMore(MacAddress addr);
    };

}

#endif /* LORAAPP_CONTAINER_H_ */
