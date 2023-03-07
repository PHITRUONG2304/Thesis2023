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

    bool DataQueue::isPacketAlreadyExist(Packet *packet)
    {
        const auto &pkt = packet->peekAtFront<LoRaAppPacket>();
        for (auto it = this->receivedSeqNumber.begin(); it != this->receivedSeqNumber.end(); it++)
        {
            if (*it == pkt->getSeqNum())
                return true;
        }
        return false;
    }

    bool DataQueue::isJustSentPacket(uint32_t seqNum)
    {
        auto packet = peekPacket();
        const auto &pkt = packet->peekAtFront<LoRaAppPacket>();
        if(pkt->getSeqNum() == seqNum)
            return true;

        return false;
    }

    ADD_PACKET_RESULT DataQueue::addPacket(Packet *packet)
    {
        const auto &pkt = packet->peekAtFront<LoRaAppPacket>();
        if (this->isPacketAlreadyExist(packet))
            return EXISTED;
        else if (this->isFull())
            return FAIL;

        this->receivedPackets.push(packet);
        this->length++;
        this->receivedSeqNumber.push_back(pkt->getSeqNum());

        return SUCCESS;
    }

    Packet* DataQueue::peekPacket()
    {
        return this->receivedPackets.front();
    }

    void DataQueue::removePacket(uint32_t index)
    {
        if (isEmpty())
            return;
        this->receivedPackets.pop();
        this->receivedSeqNumber.erase(receivedSeqNumber.begin());
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
            this->myTDMA = check_and_cast<TDMA *>(getParentModule()->getSubmodule("TDMA"));

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
                if(msg->getKind() == UPDATE_NEIGHBOR_INFO)
                {
                    auto command = check_and_cast<mCommand *>(msg);
                    this->addNewNeighborContainer(command->getAddress());
                }
                else if(msg->getKind() == START_GENERATE_PACKET)
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

    void Container::handleWithFsm(cMessage *msg)
    {
        switch (this->fsmState)
        {
        case NORMAL:
            break;

        case DISPATCHING:
            if (this->disPatchPacket(check_and_cast<Packet *>(msg)))
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

            auto packet = check_and_cast<Packet* >(msg);
            auto addr = packet->getTag<MacAddressInd>();

        }
    }

    bool Container::disPatchPacket(Packet *packet)
    {
        MacAddress destAddr = MacAddress::UNSPECIFIED_ADDRESS;
        destAddr = myHEATer->getCurrentPathToGW();

        if (destAddr == MacAddress::UNSPECIFIED_ADDRESS)
            return false;

        for (int i = 0; i < this->currentNeighbors; i++)
        {
            if (this->neighborData[i].addr == destAddr)
            {
                ADD_PACKET_RESULT result = this->neighborData[i].Dqueue.addPacket(packet);
                if (result == FAIL)
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
        auto pktData = new Packet("Sensor Data");
        pktData->setKind(DATA_PACKET);

        auto payload = makeShared<LoRaAppPacket>();
        payload->setMsgType(DATA_PACKET);

        payload->setHopNum(0);
        payload->setMsgType(DATA_PACKET);
        payload->setPayload(rand());

        // set timestamp when generating packet
        payload->setGeneratedTime(simTime());
        pktData->insertAtBack(payload);

        this->disPatchPacket(pktData);
    }


    bool Container::forwardDataPacket(MacAddress addr)
    {
        for(int i=0; i<this->currentNeighbors; i++)
        {
            if(this->neighborData[i].addr == addr)
                if(!this->neighborData[i].Dqueue.isEmpty())
                {
                    Packet *dataPkt = this->neighborData[i].Dqueue.peekPacket();
                    auto loraTag = dataPkt->addTagIfAbsent<LoRaTag>();
                    loraTag->setDestination(addr);

                    send(dataPkt, "To_Deliverer");

                    return true;
                }
        }
        return false;
    }

    bool Container::isJustSentPacket(MacAddress src, uint32_t seqNum)
    {
        for(int i=0; i < this->currentNeighbors; i++)
        {
            if(this->neighborData[i].addr == src)
                return this->neighborData[i].Dqueue.isJustSentPacket(seqNum);
        }
        throw "Not find neighbor address";
    }

    void Container::printTable()
    {
        EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << "Address" <<std::left << std::setw(2) << " " << endl;

        for(int i = 0; i < this->currentNeighbors; i++){
                EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << this->neighborData[i].addr <<std::left << std::setw(2) << " " << endl;
        }
    }
}
