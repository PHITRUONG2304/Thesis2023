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
#include "LoRaAppPacket_m.h"
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
    uint8_t slot = 0;
    simtime_t timestamp = 0;
    bool isBusy;
};

class RandomRequestMessage : public cMessage
{
private:
    uint8_t slot;
    MacAddress addr;
public:
    RandomRequestMessage(const char *name = nullptr, MacAddress addr = MacAddress::UNSPECIFIED_ADDRESS, uint8_t slot = 0): cMessage(name)
    {
        this->slot = slot;
        this->addr = addr;
    }
    MacAddress getAddress() const {return this->addr;}
    uint8_t getRequestSlot() const {return this->slot;}
};

class TDMA : public cSimpleModule, public ILifecycle
{
    private:
        uint8_t current_slot;
        simtime_t timeslot_start_time;
        uint8_t max_communication_neighbors;
        uint8_t currentNeighbors;
        simtime_t timeslotSize;
        Communication_Slot* connectedNeighbors;
        TDMA_STATE state;

        cMessage *grantFreeSlot;
        cMessage *changeSlot;
        cMessage *timeOutGrant;
        RandomRequestMessage *randomRequest;

    protected:
        virtual void initialize(int stage) override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void finish() override;

        virtual void handleMessage(cMessage *msg) override;
        virtual void handleSelfMessage(cMessage *msg);
        virtual void handlePacketFromLowerLayer(Packet *packet);
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;
        void printTable();
        void removeNeighborCommunication(MacAddress src);
    public:

        void handleWithFsm(Packet *pkt);
        bool isAvailableSlot(int slot, MacAddress src);
        void updateCommunicationSlot(MacAddress addr, uint8_t slotNumber, simtime_t timestamp);
        simtime_t getShortestWaitingTime(MacAddress src, MacAddress dest);
        bool hasEstablishedCommunicationWith(MacAddress src);
        uint8_t getCurrentSlot(){return this->current_slot;}

};

}



#endif
