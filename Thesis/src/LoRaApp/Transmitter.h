/*
 * Transmitter.h
 *
 *  Created on: Mar 7, 2023
 *      Author: PHITRUONG
 */

#ifndef LORAAPP_TRANSMITTER_H_
#define LORAAPP_TRANSMITTER_H_

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"

#include "LoRaAppPacket_m.h"
#include "mCommand_m.h"
#include "LoRa/LoRaMacControlInfo_m.h"
#include "LoRa/LoRaRadio.h"
#include <queue>

using namespace omnetpp;
using namespace inet;


namespace flora
{

class Transmitter : public cSimpleModule, public ILifecycle
{
    protected:
        void initialize(int stage) override;
        void finish() override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;

        void handleCommandFromUpperLayer(cMessage *msg);

        //LoRa parameters control
        LoRaRadio *loRaRadio;

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
        Transmitter() {}
        simsignal_t LoRa_AppPacketSent;

//      ****************************************************** EDIT *****************************************************     //
        void sendHEAT_packet(MacAddress destination, double PRR, simtime_t timeToGW);

        void sendDATA_packet(MacAddress address, void *data);
        void sendACK_packet(MacAddress destination, uint32_t seqNum, bool success);

        void sendFreeslot_packet(int slot, double myPRR, simtime_t timeToGW);
        void requestCommunicationSlot(MacAddress destination, int slot);
        void sendACKrequestSlot(MacAddress destination, int slot, bool accept, double myPRR = 0, simtime_t timeToGW = 0);

//      ****************************************************** EDIT *****************************************************     //
};
}



#endif /* LORAAPP_TRANSMITTER_H_ */
