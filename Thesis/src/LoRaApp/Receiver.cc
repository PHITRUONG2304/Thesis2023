/*
 * Receiver.cc
 *
 *  Created on: Mar 7, 2023
 *      Author: PHITRUONG
 */

#include "Receiver.h"
#include "LoRa/LoRaTagInfo_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"

namespace flora
{
    Define_Module(Receiver);
    void Receiver::initialize(int stage)
    {
        cSimpleModule::initialize(stage);
        if (stage == INITSTAGE_LOCAL)
        {
            EV << "Initializing stage " << stage << ", at Receiver\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER)
        {
            EV << "Initializing stage " << stage << ", at Receiver\n";

        }
    }

    void Receiver::finish()
    {

    }

    bool Receiver::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
    {
        Enter_Method_Silent();

        throw cRuntimeError("Unsupported life-cycle operation '%s'", operation->getClassName());
        return true;
    }

    void Receiver::handleMessage(cMessage *msg)
    {
        if(msg->isSelfMessage())
        {
            throw "Message don't know";
        }
        else
        {
            switch(msg->getKind())
            {
            case JOIN_INVITE:
            case JOIN_REQUEST:
            case JOIN_ACCEPT_REPLY:
            case JOIN_REFUSE_REPLY:
                send(msg->dup(), "To_TDMA");
                break;

            case HEAT_INFO:
                send(msg->dup(), "To_Heat");
                break;

            case DATA_PACKET:
            case FORWARD_PACKET:
            case UNABLE_RECEIVE_MORE:
            case RECEIVED_SUCCESS:
                send(msg->dup(), "To_Container");
                break;


            }
        }
        delete msg;
    }
}
