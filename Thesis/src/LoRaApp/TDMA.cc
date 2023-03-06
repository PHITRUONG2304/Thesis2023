/*
 * TDMA.cc
 *
 *  Created on: Feb 25, 2023
 *      Author: PHITRUONG
 */

#include "TDMA.h"
#include "LoRa/LoRaTagInfo_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include <iomanip>

namespace flora{
Define_Module(TDMA);

void TDMA::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        EV << "Initializing stage " << stage << ", at TDMA\n";
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        EV << "Initializing stage " << stage << ", at TDMA\n";
        this->max_communication_neighbors = par("max_neighbors").intValue();
        this->current_slot = -1;
        this->currentNeighbors = 0;
        this->timeslotSize = par("timeslot_size");

        this->connectedNeighbors = new Communication_Slot[this->max_communication_neighbors];
        for (int i = 0; i < this->max_communication_neighbors; i++)
        {
           this->connectedNeighbors[i].addr = MacAddress::UNSPECIFIED_ADDRESS;
           this->connectedNeighbors[i].slot = -1;
           this->connectedNeighbors[i].timestamp = 0;
           this->connectedNeighbors[i].isBusy = false;
           this->connectedNeighbors[i].waitUpdateFrom = false;
        }

        this->state = TDMA_STATE::IDLE;
        this->timeslot_start_time = simTime();

        changeSlot = new cMessage("Change slot");
        scheduleAt(simTime() + par("timeslot_size"), changeSlot);
    }
}

void TDMA::finish()
{
//    delete[] connectedNeighbors;
    EV << "The current neighbors are: " << int(this->currentNeighbors) << endl;
    printTable();
}

void TDMA::handleWithFsm(Packet *pkt)
{
    auto addr = pkt->getTag<MacAddressInd>();

    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();

    switch (this->state)
    {
    case IDLE:
        if(pkt->getKind() == JOIN_INVITE)
        {
            if(!hasEstablishedCommunicationWith(addr->getSrcAddress()))
            {
                randomRequest = new RandomRequestMessage("random request message", addr->getSrcAddress(), packet->getSlot());
                scheduleAt(uniform(simTime(), simTime() + par("timeout_grant")), randomRequest);

                request_again_times = 0;
                this->state = REQUESTING;
            }
        }
        break;

    case REQUESTING:
        if(pkt->getKind() == JOIN_ACCEPT_REPLY)
        {
            if(isAvailableSlot(packet->getSlot(), addr->getSrcAddress()))
            {
                updateCommunicationSlot(addr->getSrcAddress(), packet->getSlot(), simTime(), true);

                mCommand *updateNeighbor_command = new mCommand("Update Neighbor's information");
                updateNeighbor_command->setKind(UPDATE_NEIGHBOR_INFO);
                updateNeighbor_command->setAddress(addr->getSrcAddress());
                updateNeighbor_command->setPRR(packet->getPRR());
                updateNeighbor_command->setTimeToGW(packet->getTimeToGW());

//                send command for Container to update new neighbor if not exist
                send(updateNeighbor_command, "Container_O");
//                send command for Deliverer to update new neighbor if not exist
                send(updateNeighbor_command->dup(), "Deliverer_O");
//                send command for HEAT to update HEAT information
                send(updateNeighbor_command->dup(), "Heat_O");

                if(grantFreeSlot != nullptr)
                {
                    cancelAndDelete(grantFreeSlot);
                    grantFreeSlot = nullptr;
                }

                if(randomRequest != nullptr)
                {
                    cancelAndDelete(randomRequest);
                    randomRequest = nullptr;
                }
            }
        }
        else if(pkt->getKind() == JOIN_REFUSE_REPLY)
        {
            EV <<"Setup communication slot is not complete\n";
            if(randomRequest != nullptr)
            {
                cancelAndDelete(randomRequest);
                randomRequest = nullptr;
            }
        }
        break;

    case WAITING_FOR_REQUEST:
        if(pkt->getKind() == JOIN_REQUEST)
        {
            EV <<"Received join_request slot at TDMA module\n";
            if(isAvailableSlot(packet->getSlot(), addr->getSrcAddress()))
            {
                updateCommunicationSlot(addr->getSrcAddress(), packet->getSlot(), simTime(), false);

                mCommand *ACK_command = new mCommand("Accept to communicate in this time-slot");
                ACK_command->setKind(SEND_ACCEPT_ACK);
                ACK_command->setAddress(addr->getSrcAddress());
                ACK_command->setPRR(packet->getPRR());
                ACK_command->setTimeToGW(packet->getTimeToGW());
                ACK_command->setSlot(current_slot);

                mCommand *updateNeighbor_command = new mCommand("Update Neighbor's information");
                updateNeighbor_command->setKind(UPDATE_NEIGHBOR_INFO);
                updateNeighbor_command->setAddress(addr->getSrcAddress());

//                send command for HEAT to send an accept acknowledge - attach current my HEAT info
                send(ACK_command, "Heat_O");

//                send command for Deliverer to update new neighbor if not exist
                send(updateNeighbor_command, "Deliverer_O");

//                send command for Container to update new neighbor if not exist
                send(updateNeighbor_command->dup(), "Container_O");
            }
            else
            {
                mCommand *ACK_command = new mCommand("Refuse to communicate in this time-slot");
                ACK_command->setKind(SEND_REFUSE_ACK);
                ACK_command->setAddress(addr->getSrcAddress());
                ACK_command->setSlot(this->current_slot);

//                send command for Deliverer to send a refuse acknowledge that communicate in this time-slot
                send(ACK_command, "Deliverer_O");
            }
        }

    default:
        break;
    }
}

bool TDMA::isAvailableSlot(int slot, MacAddress src)
{
    if(slot == -1)
        return false;
    if(hasEstablishedCommunicationWith(src))
        removeNeighbor(src);

    return (slot == this->current_slot) &&
            (!this->connectedNeighbors[slot].isBusy || this->connectedNeighbors[slot].addr == src);
}

void TDMA::updateCommunicationSlot(MacAddress addr, int slotNumber, simtime_t timestamp, bool waitUpdateFrom)
{
    if(!this->connectedNeighbors[slotNumber].isBusy)
    {
        this->connectedNeighbors[slotNumber].addr = addr;
        this->connectedNeighbors[slotNumber].slot = slotNumber;
        this->connectedNeighbors[slotNumber].timestamp = timestamp;
        this->connectedNeighbors[slotNumber].isBusy = true;
        this->connectedNeighbors[slotNumber].waitUpdateFrom = waitUpdateFrom;
        currentNeighbors++;
    }
}

simtime_t TDMA::getShortestWaitingTime(MacAddress dest)
{
    int indexDest = -1;

    for (int i = 0; i < this->max_communication_neighbors; i++)
        if (this->connectedNeighbors[i].addr == dest)
            indexDest = i;

    if (indexDest == -1)
        return -1;

    if (this->connectedNeighbors[indexDest].slot >= this->current_slot)
        return (this->connectedNeighbors[indexDest].slot - this->current_slot) * this->timeslotSize;

    else
        return (this->connectedNeighbors[indexDest].slot + this->max_communication_neighbors - this->current_slot) * this->timeslotSize;
}

bool TDMA::hasEstablishedCommunicationWith(MacAddress src)
{
    for(int i=0; i<this->max_communication_neighbors; i++)
    {
        if(this->connectedNeighbors[i].addr == src)
            return true;
    }
    return false;
}

void TDMA::removeNeighbor(MacAddress addr)
{
    for(int i=0; i<this->max_communication_neighbors; i++)
    {

        if(this->connectedNeighbors[i].addr == addr)
        {
            this->connectedNeighbors[i].addr = MacAddress::UNSPECIFIED_ADDRESS;
            this->connectedNeighbors[i].slot = 0;
            this->connectedNeighbors[i].timestamp = 0;
            this->connectedNeighbors[i].isBusy = false;
            break;
        }

    }
}

void TDMA::handleMessage(cMessage *msg)
{
    if(msg->isSelfMessage())
        handleSelfMessage(msg);

    else
        handlePacketFromLowerLayer(dynamic_cast<Packet *>(msg));

}

void TDMA::handleSelfMessage(cMessage *msg)
{
//    handle self message
    if(msg == changeSlot)
    {
        this->current_slot = (this->current_slot + 1) % max_communication_neighbors;
        this->timeslot_start_time = simTime();
        this->state = IDLE;

        EV <<"The current slot is " << this->current_slot << endl;

        if(!this->connectedNeighbors[current_slot].isBusy)
        {
            grantFreeSlot = new cMessage("Broadcast free slot message");
            scheduleAt(uniform(simTime(), simTime() + par("timeslot_size") - par("timeout_grant")), grantFreeSlot);
        }
        else
        {
            if(!this->connectedNeighbors[current_slot].waitUpdateFrom)
            {
                randomUpdate = new cMessage("Update HEAT value message");
                scheduleAt(uniform(simTime(), simTime() + par("timeslot_size")), randomUpdate);
            }
        }

        scheduleAt(simTime() + par("timeslot_size"), changeSlot);
    }
    else if(msg == grantFreeSlot)
    {
        mCommand *myCommand = new mCommand("Broadcast join invitation");
        myCommand->setKind(SEND_INVITATION);
        send(myCommand, "Heat_O");

        this->state = WAITING_FOR_REQUEST;

        //schedule to avoid avoid waiting requests forever
        timeOutGrant = new cMessage("Timeout waiting for request");
        scheduleAt(simTime() + par("timeout_grant"), timeOutGrant);

        delete msg;
        grantFreeSlot = nullptr;
    }
    else if(msg == timeOutGrant)
    {
        this->state = IDLE;
        delete msg;
        timeOutGrant = nullptr;
    }
    else if(msg == randomRequest)
    {
        auto request = check_and_cast<RandomRequestMessage *>(msg);
        mCommand *command = new mCommand("Send request slot");
        command->setKind(SEND_REQUEST);
        command->setAddress(request->getAddress());
        command->setSlot(request->getRequestSlot());

        send(command, "Deliverer_O");

        if(++request_again_times < par("request_again_times").intValue())
            scheduleAt(simTime() + par("request_again_interval"), randomRequest);
        else
        {
            delete msg;
            randomRequest = nullptr;
        }
    }
    else if(msg == randomUpdate)
    {
        mCommand *command = new mCommand("Send update HEAT value command");
        command->setKind(SEND_UPDATE_HEAT);
        command->setAddress(this->connectedNeighbors[current_slot].addr);
        command->setSlot(current_slot);

        send(command, "Heat_O");

        delete msg;
        randomRequest = nullptr;
    }
    else
    {
        EV <<"Don't know message\n";
        delete msg;
    }

}

void TDMA::handlePacketFromLowerLayer(Packet *packet)
{
    const auto & pkt = packet->peekAtFront<LoRaAppPacket>();
//    if(pkt->getSlot() != this->current_slot)
//        return;

    switch(packet->getKind())
    {
    case JOIN_INVITE:
    case JOIN_REQUEST:
    case JOIN_ACCEPT_REPLY:
    case JOIN_REFUSE_REPLY:
        handleWithFsm(packet);
        break;

    case HEAT_INFO:
        send(packet, "Heat_O");
        break;

    case DATA_PACKET:
    case FORWARD_PACKET:
    case UNABLE_RECEIVE_MORE:
    case RECEIVED_SUCCESS:
        send(packet, "Container_O");
        break;


    }

}
bool TDMA::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    //throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void TDMA::printTable()
{
    EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << "Address" <<std::left << std::setw(2) << " " <<
                std::left << std::setw(2) << "|" << std::left << std::setw(4) << "slot" <<std::left << std::setw(2) << " " <<
                std::left << std::setw(2) << "|" << std::left << std::setw(14) << "waitUpdateFrom" <<std::left << std::setw(2) << " " << endl;

    for(int i = 0; i < this->max_communication_neighbors; i++){
        if(this->connectedNeighbors[i].slot == -1)
            continue;
            EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << this->connectedNeighbors[i].addr <<std::left << std::setw(2) << " " <<
                    std::left << std::setw(2) << "|" << std::left << std::setw(4) << int(this->connectedNeighbors[i].slot) <<std::left << std::setw(2) << " " <<
                    std::left << std::setw(2) << "|" << std::left << std::setw(14) << this->connectedNeighbors[i].waitUpdateFrom <<std::left << std::setw(2) << " " << endl;
    }
}

bool TDMA::needUpdateMore(MacAddress addr)
{
    for(int i=0; i<this->currentNeighbors; i++)
    {
        if(this->connectedNeighbors[i].addr == addr)
            return connectedNeighbors[i].waitUpdateFrom;
    }
    return false;
}

}
