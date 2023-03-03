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

namespace flora
{
    Define_Module(ReviseHEAT);

    void ReviseHEAT::initialize(int stage)
    {
        cSimpleModule::initialize(stage);
        if (stage == INITSTAGE_LOCAL)
        {
            EV << "Initializing stage " << stage << "at ReviseHEAT module\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER)
        {
            EV << "Initializing stage " << stage << "at ReviseHEAT module\n";
            this->iAmGateway = par("iAmGateway");

            this->max_neighbors = par("max_neigbors");
            this->currentNeighbors = 0;
            this->current_HEAT.PRR = par("initialPRR").doubleValue();
            this->current_HEAT.timeToGW = par("timeToGW");

            this->neighbor_Table = new Neighbor_Info[max_neighbors];
            for (uint8_t i = 0; i < this->max_neighbors; i++)
            {
                this->neighbor_Table[i].addr = MacAddress::UNSPECIFIED_ADDRESS;
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
        }
        else
        {
            if (msg->getArrivalGate() == gate("TDMA_I"))
            {
                auto command = check_and_cast<mCommand *>(msg);
                if (command->getKind() == SEND_INVITATION)
                {
                    if (canUpdate())
                    {
                        EV << "Send join invitation to neighbor nodes\n";
                        mCommand *myCommand = new mCommand("Broadcast join invitation");
                        myCommand->setKind(SEND_INVITATION);
                        myCommand->setPRR(this->current_HEAT.PRR);
                        myCommand->setTimeToGW(this->current_HEAT.timeToGW);

                        send(myCommand, "Deliverer_O");
                    }
                }
            }
        }
    }

    void ReviseHEAT::updateNeighborTable(MacAddress addr, double PRR, double timeToGW, double timestamp)
    {
        if (this->currentNeighbors + 1 == this->max_neighbors)
            return;

        HEAT_Field HEATOfNeighbor = {PRR, timeToGW};
        this->neighbor_Table[currentNeighbors].addr = addr;
        this->neighbor_Table[currentNeighbors].heatValue = HEATOfNeighbor;
        this->neighbor_Table[currentNeighbors].timestamp = timestamp;

        currentNeighbors++;
    }

    void ReviseHEAT::calculateHEATField()
    {
        for (uint8_t i = 0; i < this->currentNeighbors; i++)
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

}
