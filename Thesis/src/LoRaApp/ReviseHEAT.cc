/*
 * ReviseHEAT.cc
 *
 *  Created on: Feb 26, 2023
 *      Author: PHITRUONG
 */

#include "ReviseHEAT.h"
#include "Container.h"
#include "inet/mobility/static/StationaryMobility.h"
#include "../LoRa/LoRaTagInfo_m.h"
#include "inet/common/packet/Packet.h"
#include <iomanip>

namespace flora
{
    Define_Module(ReviseHEAT);

    void ReviseHEAT::initialize(int stage)
    {
        cSimpleModule::initialize(stage);

        if (stage == INITSTAGE_LOCAL)
        {
            EV << "Initializing stage " << stage << ", at ReviseHEAT module\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER)
        {
            EV << "Initializing stage " << stage << ", at ReviseHEAT module\n";
            this->iAmGateway = getParentModule()->par("iAmGateway").boolValue();
            this->maxSendAgainTimes = getParentModule()->par("send_again_times").intValue();
            this->sendAgainTime = getParentModule()->par("send_again_interval");

            this->currentHEAT = { MacAddress::UNSPECIFIED_ADDRESS, { par("initialPRR").doubleValue(), par("timeToGW") } };
            this->neighborTable = check_and_cast<NeighborTable* >(getParentModule()->getSubmodule("NeighborTable"));
            this->myTDMA = check_and_cast<TDMA *>(getParentModule()->getSubmodule("TDMA"));
            this->myContainer = check_and_cast<Container *>(getParentModule()->getSubmodule("Container"));

        }
    }

    void ReviseHEAT::finish()
    {
        EV <<"My current HEAT value is: " << this->currentHEAT.addr << ", " << this->currentHEAT.currentValue.PRR << ", " << this->currentHEAT.currentValue.timeToGW << endl;
    }

    bool ReviseHEAT::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
    {
        Enter_Method_Silent();

        throw cRuntimeError("Unsupported life-cycle operation '%s'", operation->getClassName());
        return true;
    }

    void ReviseHEAT::handleMessage(cMessage *msg)
    {
        if (msg->isSelfMessage())
            handleSelfMessage(msg);

        else
        {
            if (msg->getArrivalGate() == gate("From_Receiver"))
            {
                auto packet = check_and_cast<Packet *>(msg);
                auto addr = packet->getTag<MacAddressInd>();
                const auto &pkt = packet->peekAtFront<LoRaAppPacket>();

//                debug
                EV << "Source address= " << addr->getSrcAddress() << ", PRR= " << pkt->getPRR() << ", timeToGW= " << pkt->getTimeToGW() << endl;
//                end

                updateNeighborTable(addr->getSrcAddress(), pkt->getPRR(), pkt->getTimeToGW());

                if(this->myTDMA->needUpdateBack())
                {
                    mCommand *myCommand = new mCommand("Send update HEAT value");
                    myCommand->setKind(SEND_UPDATE_HEAT);
                    myCommand->setPRR(this->currentHEAT.currentValue.PRR);
                    myCommand->setTimeToGW(this->getCurrentTimeToGW());
                    myCommand->setSlot(pkt->getSlot());
                    myCommand->setAddress(addr->getSrcAddress());

                    send(myCommand, "To_Transmitter");
                }
                else
                {
                    if(updateAgain != nullptr)
                    {
                        cancelAndDelete(updateAgain);
                        updateAgain = nullptr;
                    }
                }
            }
            else if (msg->getArrivalGate() == gate("From_TDMA"))
            {
                EV <<"Curren HEAT value " << this->currentHEAT.addr << ", PRR=" << this->currentHEAT.currentValue.PRR << ", timeToGW=" << this->currentHEAT.currentValue.timeToGW << endl;
                auto command = check_and_cast<mCommand *>(msg);

                if (command->getKind() == SEND_INVITATION)
                {
                    if (canUpdate() || iAmGateway)
                    {
                        EV << "Receive command: send join invitation to neighbor nodes\n";
                        mCommand *myCommand = new mCommand("Broadcast join invitation");
                        myCommand->setKind(SEND_INVITATION);
                        myCommand->setPRR(this->currentHEAT.currentValue.PRR);
                        myCommand->setTimeToGW(this->getCurrentTimeToGW());
                        myCommand->setSlot(command->getSlot());

                        send(myCommand, "To_Transmitter");
                    }
                }
                else if(command->getKind() == SEND_ACCEPT_ACK)
                {
                    EV << "Receive command: Update neighbor's HEAT information\n";
                    this->updateNeighborTable(command->getAddress(), command->getPRR(), command->getTimeToGW());

                    mCommand *myCommand = new mCommand("Send accept message to communicate in this time-slot");
                    myCommand->setKind(SEND_ACCEPT_ACK);
                    myCommand->setPRR(this->currentHEAT.currentValue.PRR);
                    myCommand->setTimeToGW(this->getCurrentTimeToGW());
                    myCommand->setSlot(command->getSlot());
                    myCommand->setAddress(command->getAddress());
                    send(myCommand, "To_Transmitter");

                }
                else if(command->getKind() == UPDATE_NEIGHBOR_INFO)
                {
                    EV << "Receive command: Update neighbor's HEAT information\n";
                    this->updateNeighborTable(command->getAddress(), command->getPRR(), command->getTimeToGW());
                }
                else if(command->getKind() == SEND_UPDATE_HEAT)
                {
                    mCommand *myCommand = new mCommand("Send update HEAT value");
                    myCommand->setKind(SEND_UPDATE_HEAT);
                    myCommand->setPRR(this->currentHEAT.currentValue.PRR);
                    myCommand->setTimeToGW(this->getCurrentTimeToGW());
                    myCommand->setSlot(command->getSlot());
                    myCommand->setAddress(command->getAddress());

                    send(myCommand, "To_Transmitter");

                    updateAgain = new mCommand("Schedule an update in case your neighbor doesn't get it");
                    updateAgain->setAddress(command->getAddress());
                    updateAgain->setSlot(command->getSlot());

                    scheduleAt(simTime() + sendAgainTime, updateAgain);
                    this->sendAgainTimes = 0;
                }
            }
            delete msg;
        }
    }

    void ReviseHEAT::handleSelfMessage(cMessage *msg)
    {
        if(msg == updateAgain)
        {
            if(++sendAgainTimes < maxSendAgainTimes)
            {
                auto command = check_and_cast<mCommand *>(msg);

                mCommand *myCommand = new mCommand("Send update HEAT value");
                myCommand->setKind(SEND_UPDATE_HEAT);
                myCommand->setPRR(this->currentHEAT.currentValue.PRR);
                myCommand->setTimeToGW(this->getCurrentTimeToGW());
                myCommand->setSlot(command->getSlot());
                myCommand->setAddress(command->getAddress());
                send(myCommand, "To_Transmitter");

                scheduleAt(simTime() + sendAgainTime, updateAgain);
            }
            else
            {
                delete msg;
                updateAgain = nullptr;
            }
        }
    }

    void ReviseHEAT::updateNeighborTable(MacAddress addr, double PRR, simtime_t timeToGW)
    {
        this->neighborTable->updateHEATValue(addr, PRR, timeToGW);
        this->calculateHEATField();
        EV <<"Curren HEAT value " << this->currentHEAT.addr << ", PRR=" << this->currentHEAT.currentValue.PRR << ", timeToGW=" << this->currentHEAT.currentValue.timeToGW << endl;
    }

    void ReviseHEAT::updateSentState(MacAddress addr, bool state)
    {
        this->neighborTable->updateSendPacketState(addr, state);
        this->calculateHEATField();
    }

    void ReviseHEAT::calculateHEATField()
    {
        if(iAmGateway)
            return;

        NeighborHEATTable *table;
        table = this->neighborTable->getCurrentHEATTable();
        this->sortNeighborTable(table);

        this->currentHEAT.addr = MacAddress::UNSPECIFIED_ADDRESS;
        this->currentHEAT.currentValue = {0, -1};

        for (int i = 0; i < this->neighborTable->getConnectedCurrentNeighbors(); i++)
        {
            if(!table[i].overload && table[i].PRR > 0)
            {
                this->currentHEAT.addr = table[i].addr;
                this->currentHEAT.currentValue = {table[i].PRR, table[i].timeToGW};
                break;
            }
        }

        delete table;

    }

    int compareValues(const void *a, const void *b)
    {
        double PRRThroughA = ((NeighborHEATTable *)a)->PRR;
        double PRRThroughB = ((NeighborHEATTable *)b)->PRR;

        simtime_t timeToGWThroughA = ((NeighborHEATTable *)a)->timeToGW;
        simtime_t timeToGWThroughB = ((NeighborHEATTable *)b)->timeToGW;

        if (PRRThroughA > PRRThroughB)
            return -1;
        else if (PRRThroughA < PRRThroughB)
            return 1;
        else
        {
            if (timeToGWThroughA > timeToGWThroughB)
                return 1;
            else if (timeToGWThroughA < timeToGWThroughB)
                return -1;
            else
                return 0;
        }
    }

    void ReviseHEAT::sortNeighborTable(NeighborHEATTable *table)
    {
        qsort(table, this->neighborTable->getConnectedCurrentNeighbors(), sizeof(NeighborHEATTable), compareValues);
    }

    void ReviseHEAT::recalculateCurrentHEAT()
    {
        this->calculateHEATField();
    }

    simtime_t ReviseHEAT::getCurrentTimeToGW()
    {
        if(iAmGateway)
            return 0;

        return this->currentHEAT.currentValue.timeToGW + this->myTDMA->getShortestWaitingTime(currentHEAT.addr);
    }
}
