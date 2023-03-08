/*
 * Container.cc
 *
 *  Created on: Feb 25, 2023
 *      Author: PHITRUONG
 */

#include "Container.h"
#include "ReviseHEAT.h"
#include "inet/mobility/static/StationaryMobility.h"
#include "../LoRa/LoRaTagInfo_m.h"
#include "inet/common/packet/Packet.h"
#include <iomanip>

namespace flora
{
    Define_Module(Container);

    void Container::initialize(int stage)
    {
        cSimpleModule::initialize(stage);
        if (stage == INITSTAGE_LOCAL)
        {
            EV << "Initializing stage " << stage << ", at Container\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER)
        {
            EV << "Initializing stage " << stage << ", at Container\n";
            this->max_communication_neighbors = par("max_neighbors").intValue();
            this->currentNeighbors = 0;

            this->neighborTable = check_and_cast<NeighborTable *>(getParentModule()->getSubmodule("NeighborTable"));
            this->myHEATer = check_and_cast<ReviseHEAT *>(getParentModule()->getSubmodule("ReviseHEAT"));

            generatePacket = new cMessage("Generate packet");
            scheduleAt(simTime() + par("start_generate"), generatePacket);
        }
    }

    void Container::finish()
    {
//        printTable();
//        delete[] neighborData;
    }

    bool Container::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
    {
        Enter_Method_Silent();

        throw cRuntimeError("Unsupported life-cycle operation '%s'", operation->getClassName());
        return true;
    }

    void Container::handleMessage(cMessage *msg)
    {
        EV <<"I am received a command from lower layer\n";
        if(msg->isSelfMessage())
        {
            if(msg == generatePacket)
            {
                this->generatingPacket();
                scheduleAt(simTime() + par("generate_packet_interval"), generatePacket->dup());
            }
        }
        else
        {
            if(msg->getArrivalGate() == gate("From_TDMA"))
            {
                if(msg->getKind() == START_GENERATE_PACKET)
                {
                    if(!generatePacket->isScheduled())
                    {
                        generatePacket = new cMessage("Generate packet");
                        scheduleAt(simTime(), generatePacket);
                    }
                }
                else if(msg->getKind() == SEND_DATA_PACKET)
                {
                    auto command = check_and_cast<mCommand *>(msg);
                    int result = this->forwardDataPacket(command->getAddress());
                    if(result)
                        this->fsmState = FORWARDING;
                    else
                        this->fsmState = NORMAL;
                }
            }
            else if (msg->getArrivalGate() == gate("From_Deliverer"))
            {
                handleWithFsm(msg->dup());
            }
        }
        delete msg;
    }

    void Container::handleWithFsm(cMessage *msg)
    {
        switch (this->fsmState)
        {
        case NORMAL:
            break;

        case DISPATCHING:
            if(this->disPatchPacket(check_and_cast<Packet *>(msg)))
                this->fsmState = ACK_SUCCESS;
            else
                this->fsmState = ACK_FAIL;
            break;

        case ACK_FAIL:
            // send function ack about dispatch fail and
            // attach new path (HEAT, timetoGW)
            cout << "ACK Fail" << endl;

            // completed send ACK
            // TODO
            break;

        case ACK_SUCCESS:
            // send function ack about dispatch success
            cout << "ACK Success" << endl;
            break;

        case FORWARDING:
            break;


        }
    }

    bool Container::disPatchPacket(LoRaAppPacket *packet)
    {
        packet->setHopNum(packet->getHopNum() + 1);
        packet->setPayload(packet->getPayload());
        packet->setSeqNum(packet->getSeqNum());

        MacAddress destAddr = MacAddress::UNSPECIFIED_ADDRESS;
        destAddr = myHEATer->getCurrentPathToGW();

        return this->neighborTable->addNewPacketTo(destAddr, packet);
    }

    bool Container::disPatchPacket(Packet *packet)
    {
        const auto &pkt = packet->peekAtFront<LoRaAppPacket>();

        auto newPkt = new LoRaAppPacket();
        newPkt->setHopNum(pkt->getHopNum());
        newPkt->setPayload(pkt->getPayload());
        newPkt->setSeqNum(pkt->getSeqNum());

        return this->disPatchPacket(newPkt);
    }

    void Container::generatingPacket()
    {
        auto newPkt = new LoRaAppPacket();
        newPkt->setHopNum(0);
        newPkt->setPayload(rand());
        newPkt->setSeqNum(intrand(UINT32_MAX));

        this->disPatchPacket(newPkt);
    }

    bool Container::forwardDataPacket(MacAddress addr)
    {
        if(this->neighborTable->isEmpty(addr))
            return false;
//        TODO
        return true;
    }
}
