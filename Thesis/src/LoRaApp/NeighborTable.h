/*
 * NeighborTable.h
 *
 *  Created on: Mar 7, 2023
 *      Author: PHITRUONG
 */

#ifndef LORAAPP_NEIGHBORTABLE_H_
#define LORAAPP_NEIGHBORTABLE_H_

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"

#include "LoRaAppPacket_m.h"
#include <queue>

using namespace omnetpp;
using namespace inet;
using namespace std;

namespace flora
{
    struct NeighborHEATTable
    {
        MacAddress addr;
        double PRR;
        simtime_t timeToGW;
        bool overload;
    };

    struct HEAT_Field
    {
        double PRR;
        simtime_t timeToGW;
    };

    class DataQueue
    {
    private:
        int32_t length;
        int32_t maxlength;
        queue <LoRaAppPacket* > receivedPackets;

    public:
        DataQueue(int32_t maxlength = 50);
        ~DataQueue() {}
        bool isFull() { return this->maxlength == this->length; }
        bool isEmpty() { return this->length == 0; }
        bool addPacket(LoRaAppPacket* packet);
        LoRaAppPacket *peekPacket();
        void popPacket();
        void clear();
    };

    struct NeighborInformation
    {
//        Address of a respective neighbor
        MacAddress address;
        HEAT_Field heatValue;
        simtime_t timestamp;
//        To decide to update first or wait
        bool waitUpdateFrom;

//        The queue will store data packet
        DataQueue *dataQueue;
//        Status of the last n packets sent
        bool *lastState;
//        The slot has setup
        bool isBusy;
    };


    class NeighborTable : public cSimpleModule
    {
    private:
        NeighborInformation *neighborTable;
        int max_communication_neighbors;
        int current_neighbors;

    protected:
        void initialize(int stage) override;
        void finish() override;

        void printTable();

    public:
        NeighborTable() {}
//        For TDMA
        bool isAlreadyExistNeighbor(MacAddress address);
        bool addNewCommunicationSlot(MacAddress address, int slot, bool waitUpdate);
        void removeNeighbor(MacAddress address);
        bool isBusySlot(int slot){ return this->neighborTable[slot].isBusy; }
        bool waitUpdateInThisSlot(int slot) { return this->neighborTable[slot].waitUpdateFrom; }
        MacAddress getCommunicationNeighbor(int slot) { return this->neighborTable[slot].address; }
        int getCommunicationSlot(MacAddress address);
//        For ReviseHEAT
        void updateHEATValue(MacAddress address, double PRR, simtime_t timeToGW);
        NeighborHEATTable *getCurrentHEATTable();
//        For Container
        bool addNewPacketTo(MacAddress address, LoRaAppPacket *packet);
        LoRaAppPacket *peekPacket(MacAddress address);
        void popPacket(MacAddress address);
        void updateSendPacketState(MacAddress address, bool state);
        bool isFull(MacAddress address);
        bool isEmpty(MacAddress address);
        int getConnectedCurrentNeighbors(){ return current_neighbors; }
        double getNeighborQuality(MacAddress addr);
    };
}



#endif /* LORAAPP_NEIGHBORTABLE_H_ */
