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

            this->max_neighbors = par("maxNeighbour").intValue();
            this->currentNeighbors = 0;
            this->current_HEAT.PRR = par("initialPRR").doubleValue();
            this->current_HEAT.timeToGW = par("timeToGW");

            this->neighbor_Table = new Neighbor_Info[max_neighbors];
            for (int i = 0; i < this->max_neighbors; i++)
            {
                this->neighbor_Table[i].addr = MacAddress::UNSPECIFIED_ADDRESS;
                this->neighbor_Table[i].PRR = 0.0;
                this->neighbor_Table[i].heatValue.PRR = 0.0;
                this->neighbor_Table[i].heatValue.timeToGW = 0.0;
                this->neighbor_Table[i].timestamp = 0.0;
            }
            this->myTDMA = check_and_cast<TDMA *>(getParentModule()->getSubmodule("TDMA"));
            this->myDeliverer = check_and_cast<Deliverer *>(getParentModule()->getSubmodule("Deliverer"));
            this->myContainer = check_and_cast<Container *>(getParentModule()->getSubmodule("Container"));
        }
    }

    void ReviseHEAT::finish()
    {
        //    delete[] neighbor_Table;
        printTable();
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
            EV <<"Receive self message" << endl;
        }
        else
        {
            if (msg->getArrivalGate() == gate("TDMA_I"))
            {
                if(msg->isPacket())
                {
                    auto packet = check_and_cast<Packet *>(msg);
                    auto addr = packet->getTag<MacAddressInd>();
                    const auto &pkt = packet->peekAtFront<LoRaAppPacket>();
                    EV << "Source address= " << addr->getSrcAddress() << ", PRR= " << pkt->getPRR() << ", timeToGW= " << pkt->getTimeToGW() << endl;

                    updateNeighborTable(addr->getSrcAddress(), pkt->getPRR(), pkt->getTimeToGW(), simTime());
                    calculateHEATField();

                    if(this->myTDMA->needUpdateMore(addr->getSrcAddress()))
                    {
                        mCommand *myCommand = new mCommand("Send update HEAT value");
                        myCommand->setKind(SEND_UPDATE_HEAT);
                        myCommand->setPRR(this->current_HEAT.PRR);
                        myCommand->setTimeToGW(this->getCurrentTimeToGW());
                        myCommand->setSlot(this->myTDMA->getCurrentSlot());
                        myCommand->setAddress(addr->getSrcAddress());

                        send(myCommand, "Deliverer_O");
                    }
                }
                else
                {
                    auto command = check_and_cast<mCommand *>(msg);
                    if (command->getKind() == SEND_INVITATION)
                    {
                        if (canUpdate())
                        {
                            EV << "Receive command: send join invitation to neighbor nodes\n";
                            mCommand *myCommand = new mCommand("Broadcast join invitation");
                            myCommand->setKind(SEND_INVITATION);
                            myCommand->setPRR(this->current_HEAT.PRR);
                            myCommand->setTimeToGW(this->getCurrentTimeToGW());

                            send(myCommand, "Deliverer_O");
                        }
                    }
                    else if(command->getKind() == SEND_ACCEPT_ACK)
                    {
                        EV << "Receive command: Update neighbor's HEAT information\n";
                        this->updateNeighborTable(command->getAddress(), command->getPRR(), command->getTimeToGW(), simTime());
                        this->calculateHEATField();

                        mCommand *myCommand = new mCommand("Send accept message to communicate in this time-slot");
                        myCommand->setKind(SEND_ACCEPT_ACK);
                        myCommand->setPRR(this->current_HEAT.PRR);
                        myCommand->setTimeToGW(this->getCurrentTimeToGW());
                        myCommand->setSlot(command->getSlot());
                        myCommand->setAddress(command->getAddress());

                        send(myCommand, "Deliverer_O");
                    }
                    else if(command->getKind() == UPDATE_NEIGHBOR_INFO)
                    {
                        EV << "Receive command: Update neighbor's HEAT information\n";
                        this->updateNeighborTable(command->getAddress(), command->getPRR(), command->getTimeToGW(), simTime());
                        this->calculateHEATField();
                    }
                    else if(command->getKind() == SEND_UPDATE_HEAT)
                    {
                        mCommand *myCommand = new mCommand("Send update HEAT value");
                        myCommand->setKind(SEND_UPDATE_HEAT);
                        myCommand->setPRR(this->current_HEAT.PRR);
                        myCommand->setTimeToGW(this->getCurrentTimeToGW());
                        myCommand->setSlot(command->getSlot());
                        myCommand->setAddress(command->getAddress());

                        send(myCommand, "Deliverer_O");
                    }
                }
            }
        }
        delete msg;
    }

    int ReviseHEAT::isAlreadyExistNeighbor(MacAddress addr)
    {
        for(int i = 0; i < this->currentNeighbors; i++)
        {
            if(this->neighbor_Table[i].addr == addr)
                return i;
        }
        return -1;
    }

    void ReviseHEAT::addNewNeighborIntoTable(MacAddress addr, double PRR, simtime_t timeToGW, simtime_t timestamp)
    {

        HEAT_Field HEATOfNeighbor = {PRR, timeToGW};
        this->neighbor_Table[currentNeighbors].addr = addr;
        this->neighbor_Table[currentNeighbors].heatValue = HEATOfNeighbor;
        this->neighbor_Table[currentNeighbors].timestamp = timestamp;

        currentNeighbors++;

    }

    void ReviseHEAT::updateNeighborTable(MacAddress addr, double PRR, simtime_t timeToGW, simtime_t timestamp)
    {

        int index = isAlreadyExistNeighbor(addr);
        if(index == -1)
            addNewNeighborIntoTable(addr, PRR, timeToGW, timestamp);
        else
        {
            HEAT_Field HEATOfNeighbor = {PRR, timeToGW};
            this->neighbor_Table[index].addr = addr;
            this->neighbor_Table[index].heatValue = HEATOfNeighbor;
            this->neighbor_Table[index].timestamp = timestamp;
        }

    }

    void ReviseHEAT::calculateHEATField()
    {
        if(iAmGateway)
            return;

        for (int i = 0; i < this->currentNeighbors; i++)
            this->neighbor_Table[i].PRR = this->neighbor_Table[i].heatValue.PRR * this->myDeliverer->getNeighborQuality(neighbor_Table[i].addr);

        this->sortNeighborTable();

        for (int i = 0; i < this->currentNeighbors; i++)
        {
            if (myContainer->canAddMore(neighbor_Table[i].addr))
            {
                this->currentPath = i;
                break;
            }
        }
        current_HEAT.PRR = this->neighbor_Table[currentPath].PRR;
        current_HEAT.timeToGW = this->neighbor_Table[currentPath].heatValue.timeToGW;
    }

    int compareValues(const void *a, const void *b)
    {

        double PRRThroughA = ((Neighbor_Info *)a)->PRR;
        double PRRThroughB = ((Neighbor_Info *)b)->PRR;

        simtime_t timeToGWThroughA = ((Neighbor_Info *)a)->heatValue.timeToGW;
        simtime_t timeToGWThroughB = ((Neighbor_Info *)b)->heatValue.timeToGW;

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

    void ReviseHEAT::sortNeighborTable()
    {
        qsort(this->neighbor_Table, this->currentNeighbors, sizeof(Neighbor_Info), compareValues);
    }

    HEAT_Field ReviseHEAT::getCurrentHEAT()
    {
        HEAT_Field tempHEAT;
        tempHEAT.PRR = this->neighbor_Table[currentPath].PRR;
        tempHEAT.timeToGW = this->neighbor_Table[currentPath].heatValue.timeToGW;

        return tempHEAT;
    }
    MacAddress ReviseHEAT::getCurrentPathToGW()
    {
        return (this->currentPath != -1) ? this->neighbor_Table[currentPath].addr : MacAddress::UNSPECIFIED_ADDRESS;
    }

    void ReviseHEAT::printTable()
    {
        EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << "Address" <<std::left << std::setw(2) << " " <<
                    std::left << std::setw(2) << "|" << std::left << std::setw(8) << "PRR" <<std::left << std::setw(2) << " " <<
                    std::left << std::setw(2) << "|" << std::left << std::setw(8) << "PRRof" <<std::left << std::setw(2) << " " <<
                    std::left << std::setw(2) << "|" << std::left << std::setw(12) << "TimetoGWof" <<std::left << std::setw(2) << " " << endl;

        for(int i = 0; i < this->currentNeighbors; i++){
                EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << this->neighbor_Table[i].addr <<std::left << std::setw(2) << " " <<
                        std::left << std::setw(2) << "|" << std::left << std::setw(8) << this->neighbor_Table[i].PRR <<std::left << std::setw(2) << " " <<
                        std::left << std::setw(2) << "|" << std::left << std::setw(8) << this->neighbor_Table[i].heatValue.PRR <<std::left << std::setw(2) << " " <<
                        std::left << std::setw(2) << "|" << std::left << std::setw(12) << this->neighbor_Table[i].heatValue.timeToGW <<std::left << std::setw(2) << " " << endl;
        }
    }
    simtime_t ReviseHEAT::getCurrentTimeToGW()
    {
        if(iAmGateway)
            return 0;

        return this->myTDMA->getShortestWaitingTime(neighbor_Table[currentPath].addr) + neighbor_Table[currentPath].heatValue.timeToGW;
    }
}
