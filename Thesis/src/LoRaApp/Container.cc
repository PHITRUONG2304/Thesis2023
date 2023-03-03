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

namespace flora
{
    Define_Module(Container);

    DataQueue::DataQueue(uint32_t maxlength)
    {
        this->maxlength = maxlength;
        this->length = 0;
    }

    bool DataQueue::isPacketAlreadyExist(uint32_t seqNum)
    {
        for (auto it = this->receivedSeqNumber.begin(); it != this->receivedSeqNumber.end(); it++)
        {
            if (*it == seqNum)
                return true;
        }
        return false;
    }
    bool DataQueue::isPacketAlreadyExist(LoRaAppPacket packet)
    {
        for (auto it = this->receivedSeqNumber.begin(); it != this->receivedSeqNumber.end(); it++)
        {
            if (*it == packet.getSeqNum())
                return true;
        }
        return false;
    }

    ADD_PACKET_RESULT DataQueue::addPacket(LoRaAppPacket packet)
    {
        if (this->isPacketAlreadyExist(packet))
            return EXISTED;
        else if (this->isFull())
            return FAIL;

        this->receivedPackets.push(packet);
        this->length++;
        this->receivedSeqNumber.push_back(packet.getSeqNum());

        return SUCCESS;
    }

    LoRaAppPacket DataQueue::peekPacket()
    {
        if (this->isEmpty())
            return LoRaAppPacket();

        return this->receivedPackets.front();
    }

    void DataQueue::removePacket(uint32_t index)
    {
        if (isEmpty())
            return;

        for (auto it = this->receivedSeqNumber.begin(); it != this->receivedSeqNumber.end(); it++)
        {
            if (index == 0)
            {
                this->receivedSeqNumber.erase(it);
                break;
            }
            index--;
        }
    }

    void Container::initialize(int stage)
    {
        cSimpleModule::initialize(stage);
        if (stage == INITSTAGE_LOCAL)
        {
            EV << "Initializing stage " << stage << "at Container\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER)
        {
            EV << "Initializing stage " << stage << "at Container\n";
            this->max_communication_neighbors = par("max_neighbors").intValue();
            this->currentNeighbors = 0;

            this->neighborData = new Container_El[this->max_communication_neighbors];
            for (uint8_t i = 0; i < this->max_communication_neighbors; i++)
            {
                this->neighborData[i].addr = MacAddress::UNSPECIFIED_ADDRESS;
                this->neighborData[i].Dqueue = DataQueue(par("queue_length").intValue());
            }
            this->myHEATer = check_and_cast<ReviseHEAT *>(getParentModule()->getSubmodule("ReviseHEAT"));
        }
    }

    void Container::finish()
    {
        //    delete[] neighborData;
    }

    bool Container::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
    {
        Enter_Method_Silent();

        throw cRuntimeError("Unsupported life-cycle operation '%s'", operation->getClassName());
        return true;
    }

    void Container::handleMessage(cMessage *msg)
    {
    }

    void Container::handleWithFsm(LoRaAppPacket *packet)
    {
        switch (this->fsmState)
        {
        case NORMAL:
            if (packet != NULL)
            {
                this->fsmState = DISPATCHING;
            }
            break;

        case DISPATCHING:
            if (this->disPatchPacket(*(packet)))
                this->fsmState = ACK_SUCCESS;
            else
                this->fsmState = ACK_FAIL;
            break;

        case ACK_FAIL:
            // send function ack about dispatch fail and
            // attach new path (HEAT, timetoGW)
            cout << "ACK Fail" << endl;

            // completed send ACK
            this->fsmState = DISPATCH_AGAIN;
            break;

        case ACK_SUCCESS:
            // send function ack about dispatch success
            cout << "ACK Success" << endl;
            break;

        case DISPATCH_AGAIN:
            if (packet->getMsgType() == (AppPacketType)ACCEPT_ANOTHER_PATH)
                this->disPatchPacket(*(packet));
            break;
        }
    }

    void Container::handleWithFsmWhenReceivePacket(LoRaAppPacket *packet)
    {
    }

    bool Container::disPatchPacket(LoRaAppPacket packet)
    {
        MacAddress destAddr = MacAddress::UNSPECIFIED_ADDRESS;
        destAddr = myHEATer->getCurrentPathToGW();

        if (destAddr == MacAddress::UNSPECIFIED_ADDRESS)
            return false;

        for (uint8_t i = 0; i < this->currentNeighbors; i++)
        {
            if (this->neighborData[i].addr == destAddr)
            {
                ADD_PACKET_RESULT state = this->neighborData[i].Dqueue.addPacket(packet);
                if (state == FAIL)
                    return false;
                else
                    return true;
            }
        }

        return false;
    }

    bool Container::canAddMore(MacAddress addr)
    {
        bool result;
        for (int i = 0; i < this->currentNeighbors; i++)
        {
            if (neighborData[i].addr == addr)
                result = !(neighborData[i].Dqueue.isFull());
        }
        return result;
    }

    void Container::generatingPacket()
    {
        LoRaAppPacket newPacket;

        newPacket.setHopNum(0);
        newPacket.setMsgType(DATA_PACKET);
        newPacket.setPayload(rand());

        // set timestamp when generating packet
        newPacket.setGeneratedTime(simTime());

        this->disPatchPacket(newPacket);
    }
}
