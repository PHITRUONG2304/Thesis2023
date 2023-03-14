#ifndef __TDMA_H__
#define __TDMA_H__

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"
#include "inet/linklayer/common/MacAddressTag_m.h"

#include "LoRaAppPacket_m.h"
#include "LoRa/LoRaMacControlInfo_m.h"
#include "LoRa/LoRaRadio.h"
#include "mCommand_m.h"
#include "NeighborTable.h"
#include <queue>

using namespace omnetpp;
using namespace inet;

namespace flora {

enum TDMA_STATE {
    IDLE,
    REQUESTING,
    WAITING_FOR_REQUEST
};

class RandomRequestMessage : public cMessage
{
private:
    int slot;
    MacAddress addr;
public:
    RandomRequestMessage(const char *name = nullptr, MacAddress addr = MacAddress::UNSPECIFIED_ADDRESS, int slot = 0): cMessage(name)
    {
        this->slot = slot;
        this->addr = addr;
    }
    MacAddress getAddress() const {return this->addr;}
    int getRequestSlot() const {return this->slot;}
};

class TDMA : public cSimpleModule, public ILifecycle
{
    private:
        int current_slot;
        simtime_t timeslot_start_time;
        simtime_t timeslotSize;
        simtime_t periodTimeForUpdate;
        int maxNeighbors;

        NeighborTable* neighborTable;
        TDMA_STATE state;
        bool stopBroadcast = false;

        cMessage *changeSlot;
        cMessage *grantFreeSlot;
        cMessage *timeOutGrant;
        RandomRequestMessage *randomRequest;
        cMessage *timeoutForUpdate;
        cMessage *stopBroadcastMsg;
        int request_again_times;
        int maxSendAgain;
        simtime_t sendAgainTime;

    protected:
        virtual void initialize(int stage) override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void finish() override;

        virtual void handleMessage(cMessage *msg) override;
        virtual void handleSelfMessage(cMessage *msg);
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        void removeNeighbor(MacAddress addr) { this->neighborTable->removeNeighbor(addr); }
    public:

        void handleWithFsm(Packet *pkt);
        bool isAvailableSlot(int slot);
        void updateCommunicationSlot(MacAddress addr, int slotNumber, bool waitUpdateFrom) { this->neighborTable->addNewCommunicationSlot(addr, slotNumber, waitUpdateFrom); }
        simtime_t getShortestWaitingTime(MacAddress dest);
        bool hasEstablishedCommunicationWith(MacAddress src) { return this->neighborTable->isAlreadyExistNeighbor(src); }
        bool isBetterCommunicationSlot(MacAddress src, simtime_t timeToGW) {return this->neighborTable->isBetterSlot(src, timeToGW); }
        int getCurrentSlot(){return this->current_slot;}
        MacAddress getCurrentNeighborCommunicating() { return this->neighborTable->getCommunicationNeighbor(current_slot); }
        simtime_t getCurrentTimeSlotStartTime(){return this->timeslot_start_time;}
        bool needUpdateBack() { return this->neighborTable->waitUpdateInThisSlot(current_slot); }
        void updateNeighborInfo(Packet *pkt);

};

}



#endif
