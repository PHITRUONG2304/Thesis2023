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

namespace flora
{
    Define_Module(TDMA);

    void TDMA::initialize(int stage)
    {
        cSimpleModule::initialize(stage);
        if (stage == INITSTAGE_LOCAL)
        {
            EV << "Initializing stage " << stage << ", at TDMA\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER)
        {
            EV << "Initializing stage " << stage << ", at TDMA\n";
            this->current_slot = -1;
            this->timeslotSize = par("timeslot_size");
            this->timeslot_start_time = simTime();

            this->state = TDMA_STATE::IDLE;
            this->neighborTable = check_and_cast<NeighborTable* >(getParentModule()->getSubmodule("NeighborTable"));

            changeSlot = new cMessage("Change slot");
            scheduleAt(simTime() + par("timeslot_size"), changeSlot);
        }
    }

    void TDMA::finish()
    {
        //    delete[] connectedNeighbors;
    }

    void TDMA::handleMessage(cMessage *msg)
    {
        if (msg->isSelfMessage())
            handleSelfMessage(msg);
        else
        {
            handleWithFsm(dynamic_cast<Packet *>(msg));
            delete msg;
        }
    }

    void TDMA::handleSelfMessage(cMessage *msg)
    {
        //    handle self message
        if (msg == changeSlot)
        {
            this->current_slot = (this->current_slot + 1) % par("max_neighbors").intValue();
            this->timeslot_start_time = simTime();
            this->state = IDLE;
            EV <<"The current slot is: " << this->current_slot << endl;
            if (!this->neighborTable->isBusySlot(current_slot))
            {
                grantFreeSlot = new cMessage("Broadcast free slot message");
                scheduleAt(uniform(simTime(), simTime() + par("timeslot_size") - par("timeout_grant")), grantFreeSlot);
            }
            else
            {
                if (!this->neighborTable->waitUpdateInThisSlot(current_slot))
                {
                    mCommand *command = new mCommand("Send update HEAT value command");
                    command->setKind(SEND_UPDATE_HEAT);
                    command->setAddress(getCurrentNeighborCommunicating());
                    command->setSlot(current_slot);
                    send(command, "To_Heat");

                }

                timeoutForUpdate = new cMessage("Schedule to send stop update HEAT value command");
                scheduleAt(simTime() + par("timeslot_size").doubleValue()/10, timeoutForUpdate);
            }

            scheduleAt(simTime() + par("timeslot_size"), changeSlot);
        }
        else if (msg == grantFreeSlot)
        {
            mCommand *myCommand = new mCommand("Broadcast join invitation");
            myCommand->setKind(SEND_INVITATION);
            myCommand->setSlot(current_slot);
            send(myCommand, "To_Heat");

            this->state = WAITING_FOR_REQUEST;

            // schedule to avoid avoid waiting requests forever
            timeOutGrant = new cMessage("Timeout waiting for request");
            scheduleAt(simTime() + par("timeout_grant"), timeOutGrant);

            delete msg;
            grantFreeSlot = nullptr;
        }
        else if (msg == timeOutGrant)
        {
            this->state = IDLE;

            delete msg;
            timeOutGrant = nullptr;
        }
        else if (msg == randomRequest)
        {
            auto request = check_and_cast<RandomRequestMessage *>(msg);
            mCommand *command = new mCommand("Send request slot");
            command->setKind(SEND_REQUEST);
            command->setAddress(request->getAddress());
            command->setSlot(request->getRequestSlot());
            send(command, "To_Transmitter");

            if (++request_again_times < par("send_again_times").intValue())
                scheduleAt(simTime() + par("send_again_interval"), randomRequest);
            else
            {
                this->state = IDLE;
                delete msg;
                randomRequest = nullptr;
            }
        }
        else if (msg == timeoutForUpdate)
        {
            mCommand *sendDataCommand = new mCommand("Start sending packet if there are packets to serve");
            sendDataCommand->setKind(SEND_DATA_PACKET);
            sendDataCommand->setAddress(getCurrentNeighborCommunicating());
            send(sendDataCommand, "To_Container");

            delete msg;
            timeoutForUpdate = nullptr;
        }
        else
        {
            EV << "Don't know message\n";
            delete msg;
        }
    }

    bool TDMA::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
    {
        Enter_Method_Silent();

        // throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
        return true;
    }

    void TDMA::handleWithFsm(Packet *pkt)
    {
        auto addr = pkt->getTag<MacAddressInd>();
        const auto &packet = pkt->peekAtFront<LoRaAppPacket>();
        EV <<"The current state is " << this->state << endl;
        switch (this->state)
        {
        case IDLE:
            if (pkt->getKind() == JOIN_INVITE)
            {
                if (!hasEstablishedCommunicationWith(addr->getSrcAddress()))
                {
                    randomRequest = new RandomRequestMessage("random request message", addr->getSrcAddress(), packet->getSlot());
                    scheduleAt(uniform(simTime(), simTime() + par("timeout_grant")), randomRequest);

                    request_again_times = 0;
                    this->state = REQUESTING;
                }
            }
            break;

        case REQUESTING:
            if (pkt->getKind() == JOIN_ACCEPT_REPLY)
            {
                EV << "Communication slot setup success\n";
                updateCommunicationSlot(addr->getSrcAddress(), packet->getSlot(), true);
                updateNeighborInfo(pkt->dup());

                if (grantFreeSlot != nullptr)
                {
                    cancelAndDelete(grantFreeSlot);
                    grantFreeSlot = nullptr;
                }

                if (randomRequest != nullptr)
                {
                    cancelAndDelete(randomRequest);
                    randomRequest = nullptr;
                }

                this->state = IDLE;
            }
            else if (pkt->getKind() == JOIN_REFUSE_REPLY)
            {
                EV << "Communication slot setup failed\n";
                if (randomRequest != nullptr)
                {
                    cancelAndDelete(randomRequest);
                    randomRequest = nullptr;
                }
                this->state = IDLE;
            }
            break;

        case WAITING_FOR_REQUEST:
            if (pkt->getKind() == JOIN_REQUEST)
            {
                EV << "Received join_request slot at TDMA module\n";
                if (isAvailableSlot(packet->getSlot(), addr->getSrcAddress()))
                {
                    updateCommunicationSlot(addr->getSrcAddress(), packet->getSlot(), false);
                    updateNeighborInfo(pkt->dup());

                    mCommand *ACK_command = new mCommand("Accept to communicate in this time-slot");
                    ACK_command->setKind(SEND_ACCEPT_ACK);
                    ACK_command->setAddress(addr->getSrcAddress());
                    ACK_command->setSlot(current_slot);

                    send(ACK_command, "To_Heat");
                }
                else
                {
                    mCommand *ACK_command = new mCommand("Refuse to communicate in this time-slot");
                    ACK_command->setKind(SEND_REFUSE_ACK);
                    ACK_command->setAddress(addr->getSrcAddress());
                    ACK_command->setSlot(this->current_slot);

                    // send command for Deliverer to send a refuse acknowledge that communicate in this time-slot
                    send(ACK_command, "To_Transmitter");
                }
            }
            break;

        default:
            break;
        }
    }

    bool TDMA::isAvailableSlot(int slot, MacAddress src)
    {
        if (slot == -1)
            return false;

        if (this->neighborTable->isAlreadyExistNeighbor(src))
            this->neighborTable->removeNeighbor(src);

        return !(this->neighborTable->isBusySlot(slot));
    }

    simtime_t TDMA::getShortestWaitingTime(MacAddress dest)
    {
        int slotDest = this->neighborTable->getCommunicationSlot(dest);

        if (slotDest == -1)
            return -1;
        if (slotDest >= this->current_slot)
            return (slotDest - this->current_slot) * this->timeslotSize;
        else
            return (slotDest + par("max_neighbors").intValue() - this->current_slot) * this->timeslotSize;
    }

    void TDMA::updateNeighborInfo(Packet *pkt)
    {
        auto addr = pkt->getTag<MacAddressInd>();
        const auto &packet = pkt->peekAtFront<LoRaAppPacket>();

        mCommand *updateNeighbor_command = new mCommand("Update Neighbor's information");
        updateNeighbor_command->setKind(UPDATE_NEIGHBOR_INFO);
        updateNeighbor_command->setAddress(addr->getSrcAddress());
        updateNeighbor_command->setPRR(packet->getPRR());
        updateNeighbor_command->setTimeToGW(packet->getTimeToGW());

        send(updateNeighbor_command, "To_Heat");
    }

}
