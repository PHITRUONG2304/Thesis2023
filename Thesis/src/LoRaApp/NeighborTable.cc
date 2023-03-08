/*
 * NeighborTable.cc
 *
 *  Created on: Mar 7, 2023
 *      Author: PHITRUONG
 */

#include "NeighborTable.h"
#include <iomanip>


namespace flora
{
    Define_Module(NeighborTable);

    DataQueue::DataQueue(int32_t maxlength)
    {
        this->maxlength = maxlength;
        this->length = 0;
    }


    bool DataQueue::addPacket(LoRaAppPacket *packet)
    {
        if (this->isFull())
            return false;

        this->receivedPackets.push(packet);
        this->length++;
        return true;
    }

    LoRaAppPacket* DataQueue::peekPacket()
    {
        return this->receivedPackets.front();
    }

    void DataQueue::popPacket()
    {
        if (isEmpty()) return;

        this->receivedPackets.pop();
        this->length--;
    }
    void DataQueue::clear()
    {
        int number = this->length;
        for(int i=0; i < number; i++)
            popPacket();
    }

    void NeighborTable::initialize(int stage)
    {
        cSimpleModule::initialize(stage);
        EV << "Initializing NeighborTable\n";

        this->max_communication_neighbors = par("max_neighbors").intValue();
        this->current_neighbors = 0;

        this->neighborTable = new NeighborInformation[max_communication_neighbors];
        for (int i = 0; i < this->max_communication_neighbors; i++)
        {
            this->neighborTable[i].address = MacAddress::UNSPECIFIED_ADDRESS;
            this->neighborTable[i].timestamp = 0.0;

            this->neighborTable[i].lastState = new bool[par("last_packet_number").intValue()];
            for(int j=0; j < par("last_packet_number").intValue(); j++)
                this->neighborTable[i].lastState[j] = true;

            this->neighborTable[i].dataQueue = new DataQueue(par("queue_length").intValue());
            this->neighborTable[i].isBusy = false;
        }
    }

    void NeighborTable::finish()
    {
        printTable();
    }

    bool NeighborTable::isAlreadyExistNeighbor(MacAddress address)
    {
        for(int i = 0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
                return true;
        }
        return false;
    }

    bool NeighborTable::addNewCommunicationSlot(MacAddress address, int slot, bool waitUpdate)
    {
        if(this->neighborTable[slot].isBusy || (slot >= this->max_communication_neighbors))
            return false;

        this->neighborTable[slot].address = address;
        this->neighborTable[slot].isBusy = true;
        this->neighborTable[slot].waitUpdateFrom = waitUpdate;

        this->current_neighbors++;

        return true;
    }

    void NeighborTable::removeNeighbor(MacAddress address)
    {
        for(int i=0; i<this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
            {
                this->neighborTable[i].address = MacAddress::UNSPECIFIED_ADDRESS;
                this->neighborTable[i].isBusy = false;

                break;
            }
        }
    }

    int NeighborTable::getCommunicationSlot(MacAddress address)
    {
        int slot = -1;
        for(int i=0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
            {
                slot = i;
                break;
            }
        }

        return slot;
    }


    void NeighborTable::updateHEATValue(MacAddress address, double PRR, simtime_t timeToGW)
    {
        for(int i=0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
            {
                this->neighborTable[i].heatValue.PRR = PRR;
                this->neighborTable[i].heatValue.timeToGW = timeToGW;
                this->neighborTable[i].timestamp = simTime();
                break;
            }
        }
    }

    NeighborHEATTable* NeighborTable::getCurrentHEATTable()
    {
        NeighborHEATTable *tempTable = new NeighborHEATTable[current_neighbors + 1];

        int index = 0;
        for(int i=0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].isBusy)
            {
                tempTable[index].addr = this->neighborTable[i].address;
                tempTable[index].timeToGW = this->neighborTable[i].heatValue.timeToGW;
                tempTable[index].PRR = this->neighborTable[i].heatValue.PRR * getNeighborQuality(neighborTable[i].address);
                tempTable[index].overload = this->neighborTable[i].dataQueue->isFull();
                index++;
            }
        }
        return tempTable;
    }

    void NeighborTable::updateSendPacketState(MacAddress address, bool state)
    {
        for(int i=0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
            {
                for (int j = 0; j < par("last_packet_number").intValue() - 1; j++)
                    this->neighborTable[i].lastState[j] = this->neighborTable[i].lastState[j + 1];

                this->neighborTable[i].lastState[99] = state;
                break;
            }
        }
    }
    bool NeighborTable::addNewPacketTo(MacAddress address, LoRaAppPacket *packet)
    {
        for(int i=0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
                return neighborTable[i].dataQueue->addPacket(packet);
        }

        throw "Don't have the respective neighbor";
    }

    LoRaAppPacket* NeighborTable::peekPacket(MacAddress address)
    {
        for(int i=0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
            {
                if(!neighborTable[i].dataQueue->isEmpty())
                    return neighborTable[i].dataQueue->peekPacket();
                else
                    return nullptr;
            }
        }

        throw "Don't have the respective neighbor";
    }

    void NeighborTable::popPacket(MacAddress address)
    {
        for(int i=0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
            {
                if(!neighborTable[i].dataQueue->isEmpty())
                    neighborTable[i].dataQueue->popPacket();

                break;
            }
        }
    }

    bool NeighborTable::isFull(MacAddress address)
    {
        for(int i=0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
            {
                return this->neighborTable[i].dataQueue->isFull();
            }
        }
        throw "Don't have the respective neighbor";
    }

    bool NeighborTable::isEmpty(MacAddress address)
    {
        for(int i=0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == address)
            {
                return this->neighborTable[i].dataQueue->isEmpty();
            }
        }
        throw "Don't have the respective neighbor";
    }

    void NeighborTable::printTable()
    {
        EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << "Address" <<std::left << std::setw(2) << " "
                << std::left << std::setw(2) << "|" << std::left << std::setw(8) << "PRR" <<std::left << std::setw(2) << " "
                << std::left << std::setw(2) << "|" << std::left << std::setw(8) << "timeToGW" <<std::left << std::setw(2) << " "
                << std::left << std::setw(2) << "|" << std::left << std::setw(8) << "waitU" <<std::left << std::setw(2) << " "
                << endl;

        for(int i = 0; i < this->max_communication_neighbors; i++){
            if(this->neighborTable[i].isBusy)
                EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << this->neighborTable[i].address <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(8) << this->neighborTable[i].heatValue.PRR <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(8) << this->neighborTable[i].heatValue.timeToGW <<std::left << std::setw(2) << " "
                        << std::left << std::setw(2) << "|" << std::left << std::setw(8) << this->neighborTable[i].waitUpdateFrom <<std::left << std::setw(2) << " "
                        << endl;
        }
    }

    double NeighborTable::getNeighborQuality(MacAddress addr)
    {
        int success = 0;
        for(int i = 0; i < this->max_communication_neighbors; i++)
        {
            if(this->neighborTable[i].address == addr)
            {
                for(int j = 0; j < par("last_packet_number").intValue(); j++)
                {
                    success += (neighborTable[i].lastState[j])? 1 : 0;
                }
                break;
            }
        }

        return success/100.0;
    }
}
