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
#include <queue>

using namespace omnetpp;
using namespace inet;

namespace flora {

enum TDMA_STATE {
    IDLE,
    REQUESTING,
    WAITING_FOR_REQUEST
};

struct Communication_Slot {
    MacAddress addr;
    int slot = 0;
    simtime_t timestamp = 0;
    bool isBusy;
    bool waitUpdateFrom;
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
        int max_communication_neighbors;
        int currentNeighbors;
        simtime_t timeslotSize;
        Communication_Slot* connectedNeighbors;
        TDMA_STATE state;

        cMessage *changeSlot;
        cMessage *grantFreeSlot;
        cMessage *timeOutGrant;
        RandomRequestMessage *randomRequest;
        cMessage *randomUpdate;
        int request_again_times;

    protected:
        virtual void initialize(int stage) override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void finish() override;

        virtual void handleMessage(cMessage *msg) override;
        virtual void handleSelfMessage(cMessage *msg);
        virtual void handlePacketFromLowerLayer(Packet *packet);
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        void printTable();
        void removeNeighbor(MacAddress addr);
    public:

        void handleWithFsm(Packet *pkt);
        bool isAvailableSlot(int slot, MacAddress src);
        void updateCommunicationSlot(MacAddress addr, int slotNumber, simtime_t timestamp, bool waitUpdateFrom);
        simtime_t getShortestWaitingTime(MacAddress dest);
        bool hasEstablishedCommunicationWith(MacAddress src);
        int getCurrentSlot(){return this->current_slot;}
        simtime_t getCurrentTimeSlotStartTime(){return this->timeslot_start_time;}
        bool needUpdateMore(MacAddress addr);

};

}



#endif
