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
            this->iAmGateway = par("iAmGateway");

            this->currentHEAT = { MacAddress::UNSPECIFIED_ADDRESS, { par("initialPRR").doubleValue(), par("timeToGW") }};

            this->neighborTable = check_and_cast<NeighborTable* >(getParentModule()->getSubmodule("NeighborTable"));
            this->myTDMA = check_and_cast<TDMA *>(getParentModule()->getSubmodule("TDMA"));
            this->myContainer = check_and_cast<Container *>(getParentModule()->getSubmodule("Container"));

        }
    }

    void ReviseHEAT::finish()
    {
        //    delete[] neighbor_Table;
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
        {
            if(msg == updateAgain)
            {
                if(this->sendAgainTimes < par("send_again_times").intValue())
                {
                    auto command = check_and_cast<mCommand *>(msg);

                    mCommand *myCommand = new mCommand("Send update HEAT value");
                    myCommand->setKind(SEND_UPDATE_HEAT);
                    myCommand->setPRR(this->currentHEAT.currentValue.PRR);
                    myCommand->setTimeToGW(this->getCurrentTimeToGW());
                    myCommand->setSlot(command->getSlot());
                    myCommand->setAddress(command->getAddress());
                    send(myCommand, "To_Transmitter");

                    scheduleAt(simTime() + par("send_again_interval"), updateAgain);
                    this->sendAgainTimes += 1;
                }
                else
                {
                    delete msg;
                    updateAgain = nullptr;
                }
            }
        }
        else
        {
            if (msg->getArrivalGate() == gate("From_Receiver"))
            {
                if(msg->isPacket())
                {
                    auto packet = check_and_cast<Packet *>(msg);
                    auto addr = packet->getTag<MacAddressInd>();
                    const auto &pkt = packet->peekAtFront<LoRaAppPacket>();

                    EV << "Source address= " << addr->getSrcAddress() << ", PRR= " << pkt->getPRR() << ", timeToGW= " << pkt->getTimeToGW() << endl;

                    updateNeighborTable(addr->getSrcAddress(), pkt->getPRR(), pkt->getTimeToGW());
                    calculateHEATField();

                    if(this->myTDMA->needUpdateBack())
                    {
                        mCommand *myCommand = new mCommand("Send update HEAT value");
                        myCommand->setKind(SEND_UPDATE_HEAT);
                        myCommand->setPRR(this->currentHEAT.currentValue.PRR);
                        myCommand->setTimeToGW(this->getCurrentTimeToGW());
                        myCommand->setSlot(this->myTDMA->getCurrentSlot());
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
            }
            else if (msg->getArrivalGate() == gate("From_TDMA"))
            {
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
                    this->calculateHEATField();

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
                    this->calculateHEATField();
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

                    scheduleAt(simTime() + par("send_again_interval"), updateAgain);
                    this->sendAgainTimes = 0;
                }
            }
            delete msg;
        }
    }

    void ReviseHEAT::updateNeighborTable(MacAddress addr, double PRR, simtime_t timeToGW)
    {
        this->neighborTable->updateHEATValue(addr, PRR, timeToGW);
    }

    void ReviseHEAT::calculateHEATField()
    {
        if(iAmGateway)
            return;

        NeighborHEATTable *table;
        table = this->neighborTable->getCurrentHEATTable();
        this->sortNeighborTable(table);
        for (int i = 0; i < this->neighborTable->getConnectedCurrentNeighbors(); i++)
        {
            if(!table[i].overload)
            {
                this->currentHEAT.addr = table[i].addr;
                this->currentHEAT.currentValue = {table[i].PRR, table[i].timeToGW};
                break;
            }
        }
    }

    int compareValues(const void *a, const void *b)
    {
        double PRRThroughA = ((NeighborHEATTable *)a)->PRR;
        double PRRThroughB = ((NeighborHEATTable *)b)->PRR;

        simtime_t timeToGWThroughA = ((NeighborHEATTable *)a)->timeToGW;
        simtime_t timeToGWThroughB = ((NeighborHEATTable *)b)->timeToGW;

        if (PRRThroughA > PRRThroughB)
        {
            return -1;
        }
        else if (PRRThroughA < PRRThroughB)
        {
            return 1;
        }
        else
        {
            if (timeToGWThroughA > timeToGWThroughB)
            {
                return 1;
            }
            else if (timeToGWThroughA < timeToGWThroughB)
            {
                return -1;
            }
            else
                return 0;
        }
    }

    void ReviseHEAT::sortNeighborTable(NeighborHEATTable *table)
    {
        qsort(table, this->neighborTable->getConnectedCurrentNeighbors(), sizeof(NeighborHEATTable), compareValues);
    }

    MacAddress ReviseHEAT::getCurrentPathToGW()
    {
        return this->currentHEAT.addr;
    }

    simtime_t ReviseHEAT::getCurrentTimeToGW()
    {
        if(iAmGateway)
            return 0;

        return this->currentHEAT.currentValue.timeToGW + this->myTDMA->getShortestWaitingTime(currentHEAT.addr);
    }
}
