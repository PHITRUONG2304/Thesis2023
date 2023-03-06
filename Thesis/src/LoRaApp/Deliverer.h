#ifndef __DELIVERER_H__
#define __DELIVERER_H__

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"

#include "LoRaAppPacket_m.h"
#include "mCommand_m.h"
#include "TDMA.h"
#include "LoRa/LoRaMacControlInfo_m.h"
#include "LoRa/LoRaRadio.h"
#include <queue>

using namespace omnetpp;
using namespace inet;


namespace flora
{

struct NeighborsQuality
{
    MacAddress addr;
    bool lastState[100];
};
class Deliverer : public cSimpleModule, public ILifecycle
{
    protected:
        void initialize(int stage) override;
        void finish() override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;

        void handleMessageFromUpperLayer(cMessage *msg);

        //LoRa parameters control
        LoRaRadio *loRaRadio;
        TDMA *myTDMA;
        int currentNeighbors;
        NeighborsQuality *neighborsQuality;

        void setSF(int SF);
        int getSF();
        void setTP(int TP);
        double getTP();
        void setCR(int CR);
        int getCR();
        void setCF(units::values::Hz CF);
        units::values::Hz getCF();
        void setBW(units::values::Hz BW);
        units::values::Hz getBW();

    public:
        Deliverer() {}
        simsignal_t LoRa_AppPacketSent;

//      ****************************************************** EDIT *****************************************************     //
        void sendHEAT_packet(MacAddress destination, double PRR, simtime_t timeToGW);

        void sendDATA_packet(MacAddress destination, LoRaAppPacket packet);
        void sendACK_packet(MacAddress destination, bool success);

        void sendFreeslot_packet(int slot, double myPRR, simtime_t timeToGW);
        void requestCommunicationSlot(MacAddress destination, int slot);
        void sendACKrequestSlot(MacAddress destination, int slot, bool accept, double myPRR = 0, simtime_t timeToGW = 0);

        double getNeighborQuality(MacAddress addr);
        bool isAlreadyExistNeighbor(MacAddress addr);
        void addNewNeighbor(MacAddress addr);
        void updateNeighborQuality(MacAddress addr, bool state);
        void printTable();
        void removeNeighbor(MacAddress addr);
//      ****************************************************** EDIT *****************************************************     //
};
}


#endif
