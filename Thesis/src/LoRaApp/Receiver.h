/*
 * Receiver.h
 *
 *  Created on: Mar 7, 2023
 *      Author: PHITRUONG
 */

#ifndef LORAAPP_RECEIVER_H_
#define LORAAPP_RECEIVER_H_

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"

#include "LoRaAppPacket_m.h"
#include "mCommand_m.h"
#include "TDMA.h"
#include "LoRa/LoRaMacControlInfo_m.h"

using namespace omnetpp;
using namespace inet;


namespace flora
{

class Receiver : public cSimpleModule, public ILifecycle
{
    protected:
        void initialize(int stage) override;
        void finish() override;
        int numInitStages() const override { return NUM_INIT_STAGES; }
        void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;

        TDMA *myTDMA;
    public:
        Receiver() {}
};
}



#endif /* LORAAPP_RECEIVER_H_ */
