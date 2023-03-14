/*
 * StatisticalModule.cc
 *
 *  Created on: Mar 10, 2023
 *      Author: PHITRUONG
 */

#include "StatisticalModule.h"
#include "NeighborTable.h"
#include <iomanip>

namespace flora
{
    Define_Module(StatisticalModule);

    void StatisticalModule::initialize(int stage)
    {
        EV << "Initial statistical module successful\n";
        this->maxNeighbors = par("initialNeighbors").intValue();
        this->currentNeighbors = 0;

        this->neighbors = new StatisticalInformation[maxNeighbors];
        for(int i = 0; i < this->maxNeighbors; i++)
        {
            this->neighbors[i].address = MacAddress::UNSPECIFIED_ADDRESS;
            this->neighbors[i].maxCurrentHops = par("initialHopNum").intValue();
            this->neighbors[i].numSent = 0;
            this->neighbors[i].numResent = 0;
            this->neighbors[i].numSentSucess = 0;
            this->neighbors[i].numReceived = 0;
            this->neighbors[i].numReceiveAccordingToHops = new int[neighbors[i].maxCurrentHops];
            for(int j = 0; j < this->neighbors[i].maxCurrentHops; j++)
                this->neighbors[i].numReceiveAccordingToHops[j] = 0;
        }

        this->neighborInfo = check_and_cast<NeighborTable *>(getParentModule()->getSubmodule("NeighborTable"));
    }

    void StatisticalModule::finish()
    {
        printNeighborsInformation();

        for(int i = 0; i < this->maxNeighbors; i++)
            delete[] neighbors[i].numReceiveAccordingToHops;

        delete[] neighbors;

    }

    int StatisticalModule::isAlreadyExist(MacAddress address)
    {
        for(int i = 0; i < this->maxNeighbors; i++)
        {
            if(this->neighbors[i].address == address)
                return i;
        }

        return -1;
    }

    int StatisticalModule::addNewNeighbor(MacAddress address)
    {
        this->neighbors[currentNeighbors].address = address;
        currentNeighbors += 1;

        return currentNeighbors - 1;
    }

    void StatisticalModule::ensureHopNumStorge(MacAddress address, int hopNum)
    {
        for(int i = 0; i < this->maxNeighbors; i++)
        {
            if(this->neighbors[i].address == address)
            {
                if(this->neighbors[i].maxCurrentHops >= hopNum)
                    return;

                int *newNumReceiveAccordingToHops = new int[hopNum];
                for(int j = 0; j < this->neighbors[i].maxCurrentHops; j++)
                        newNumReceiveAccordingToHops[j] = this->neighbors[i].numReceiveAccordingToHops[j];

                for(int j = this->neighbors[i].maxCurrentHops; j < hopNum; j++)
                        newNumReceiveAccordingToHops[j] = 0;

                neighbors[i].maxCurrentHops = hopNum;
                delete[] neighbors[i].numReceiveAccordingToHops;

                neighbors[i].numReceiveAccordingToHops = newNumReceiveAccordingToHops;
                break;
            }
        }
    }

    void StatisticalModule::updateStatisticalLoRaData(MacAddress address, bool sucess, bool again)
    {
        int index = this->isAlreadyExist(address);
        if(index == -1)
            index = this->addNewNeighbor(address);

        if(again)
            this->neighbors[index].numResent += 1;
        else{
            if(!sucess)
                this->neighbors[index].numSent += 1;
            else
                this->neighbors[index].numSentSucess += 1;
        }

    }

    void StatisticalModule::updateStatisticalLoRaData(MacAddress address, int hopNum)
    {
        int index = this->isAlreadyExist(address);
        if(index == -1)
            index = this->addNewNeighbor(address);

        this->ensureHopNumStorge(address, hopNum);

        this->neighbors[index].numReceived += 1;
        this->neighbors[index].numReceiveAccordingToHops[hopNum-1] += 1;
        EV_INFO <<"The current neighbor is: " << this->currentNeighbors << endl;

    }

    void StatisticalModule::printNeighborsInformation()
    {
        this->neighborInfo->printTable();
        EV_INFO << endl;
        int maxHops = 0;
        for(int i=0; i<this->currentNeighbors; i++)
            maxHops = ( maxHops < neighbors[i].maxCurrentHops ) ? neighbors[i].maxCurrentHops : maxHops;

        EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << "Address" <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(12) << "numReceived" <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(14) << "numSentSucess" <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(10) << "numResent" <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(8) << "maxHops" <<std::left << std::setw(2) << " ";
        for(int i=0; i < maxHops; i++)
            EV_INFO << std::left << std::setw(2) << "|" << std::left << i + 1 << std::setw(8) << " Hops" <<std::left << std::setw(2) << " ";
        EV_INFO << endl;

        for(int i = 0; i < this->currentNeighbors; i++){
            EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << this->neighbors[i].address <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(12) << this->neighbors[i].numReceived <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(14) << this->neighbors[i].numSentSucess <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(10) << this->neighbors[i].numResent <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(8) << this->neighbors[i].maxCurrentHops <<std::left << std::setw(2) << " ";
            for(int j=0; j < this->neighbors[i].maxCurrentHops; j++)
                EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(9) << this->neighbors[i].numReceiveAccordingToHops[j] <<std::left << std::setw(2) << " ";
            EV_INFO << endl;
        }
    }
}
