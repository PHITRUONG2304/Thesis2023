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
            EV << "Initializing stage " << stage << ", at Container\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER)
        {
            EV << "Initializing stage " << stage << ", at Container\n";
            this->max_communication_neighbors = par("max_neighbors").intValue();
            this->currentNeighbors = 0;

            this->neighborData = new Container_El[this->max_communication_neighbors];
            for (int i = 0; i < this->max_communication_neighbors; i++)
            {
                this->neighborData[i].addr = MacAddress::UNSPECIFIED_ADDRESS;
                this->neighborData[i].Dqueue = DataQueue(par("queue_length").intValue());
            }
            this->myHEATer = check_and_cast<ReviseHEAT *>(getParentModule()->getSubmodule("ReviseHEAT"));
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
        if(msg->isPacket())
        {

        }
        else
        {
            if(msg->getKind() == UPDATE_NEIGHBOR_INFO)
            {
                auto command = check_and_cast<mCommand *>(msg);
                this->addNewNeighborContainer(command->getAddress());
            }
        }
    }

    bool Container::isAlreadyExistNeighbor(MacAddress addr)
    {
        for(int i=0; i < this->currentNeighbors; i++)
        {
            if(this->neighborData[i].addr == addr)
                return true;
        }
        return false;
    }
    void Container::addNewNeighborContainer(MacAddress addr)
    {
        if(!isAlreadyExistNeighbor(addr))
        {
            this->neighborData[currentNeighbors].addr = addr;
            currentNeighbors += 1;
        }
        else
            EV <<"Neighbor already exists" << endl;
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
            // TODO
            break;

        case ACK_SUCCESS:
            // send function ack about dispatch success
            cout << "ACK Success" << endl;
            break;
        }
    }

    bool Container::disPatchPacket(LoRaAppPacket packet)
    {
        MacAddress destAddr = MacAddress::UNSPECIFIED_ADDRESS;
        destAddr = myHEATer->getCurrentPathToGW();

        if (destAddr == MacAddress::UNSPECIFIED_ADDRESS)
            return false;

        for (int i = 0; i < this->currentNeighbors; i++)
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

    void Container::printTable()
    {
        EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << "Address" <<std::left << std::setw(2) << " " << endl;

        for(int i = 0; i < this->currentNeighbors; i++){
                EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << this->neighborData[i].addr <<std::left << std::setw(2) << " " << endl;
        }
    }
}
