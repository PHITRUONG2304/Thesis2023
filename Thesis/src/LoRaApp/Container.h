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
        queue<Packet* > receivedPackets;
        vector<uint32_t> receivedSeqNumber;

        bool isPacketAlreadyExist(uint32_t seqNum);
        bool isPacketAlreadyExist(Packet *packet);

    public:
        bool isFull() { return this->maxlength == this->length; }
        bool isEmpty() { return this->length == 0; }
        DataQueue(uint32_t maxlength = 50);
        ADD_PACKET_RESULT addPacket(Packet* packet);
        Packet *peekPacket();
        void popPacket();
        void removePacket(uint32_t index = 0);
        bool isJustSentPacket(uint32_t seqNum);
    };

    enum State
    {
        NORMAL,
        DISPATCHING,
        ACK_FAIL,
        ACK_SUCCESS,
        FORWARDING
    };

    struct Container_El
    {
        MacAddress addr;
        DataQueue Dqueue;
    };

    class Container : public cSimpleModule, public ILifecycle
    {
    private:
        int max_communication_neighbors;
        int currentNeighbors;
        Container_El *neighborData;
        State fsmState = NORMAL;
        ReviseHEAT *myHEATer;
        TDMA *myTDMA;

        cMessage *generatePacket;

    protected:
        void initialize(int stage) override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void finish() override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        void handleMessage(cMessage *msg) override;
        bool isAlreadyExistNeighbor(MacAddress addr);
        void addNewNeighborContainer(MacAddress addr);
        bool isJustSentPacket(MacAddress src, uint32_t seqNum);

    public:
        void handleWithFsm(cMessage *msg);
        State getNextState(cMessage *msg);
        void executeAction();
        bool disPatchPacket(Packet *packet);
        void generatingPacket();
        bool canAddMore(MacAddress addr);
        bool forwardDataPacket(MacAddress addr);
        void printTable();
    };

}

#endif /* LORAAPP_CONTAINER_H_ */
