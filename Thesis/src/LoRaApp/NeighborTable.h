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
#include "StatisticalModule.h"
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
        int32_t maxlength;
        queue <LoRaAppPacket* > receivedPackets;

    public:
        DataQueue(int32_t maxlength = 50);
        ~DataQueue() {}
        bool isFull() { return this->maxlength == this->receivedPackets.size(); }
        bool isEmpty() { return receivedPackets.empty(); }
        bool addPacket(LoRaAppPacket* packet);
        LoRaAppPacket *peekPacket();
        int32_t length() { return this->receivedPackets.size(); }
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
        DataQueue dataQueue;
//        Status of the last n packets sent
        bool *lastState;
//        The slot has setup
        bool isBusy;

//        for testing
        int rPacketNum;
    };


    class NeighborTable : public cSimpleModule
    {
        friend class StatisticalModule;
    private:
        NeighborInformation *neighborTable;
        int max_communication_neighbors;
        int current_neighbors;

    protected:
        void initialize(int stage) override;
        void finish() override;


    public:
        void printTable();
        NeighborTable() {}
//        For TDMA
//        Kiem tra xem neighbor da ton tai trong neighbor table chua
        bool isAlreadyExistNeighbor(MacAddress address);
//        Kiem tra slot giao tiep nay thoi gian co tot hon khong
        bool isBetterSlot(MacAddress src, simtime_t timeToGW);
//        Them moi mot neighbor da thiet lap giao tiep neu chua ton tai
        bool addNewCommunicationSlot(MacAddress address, int slot, bool waitUpdate);
//        Xoa 1 neighbor khoi bang neighbor table - dung cho khi neighbor mat ket noi hoac dung cho giai thuat TDMA
        void removeNeighbor(MacAddress address);
//        Kiem tra xem slot nay da duoc thiet lap ket noi voi neighbor nao chua
        bool isBusySlot(int slot){ return this->neighborTable[slot].isBusy; }
//        Dung cho cap nhat de khong dung do
        bool waitUpdateInThisSlot(int slot) { return this->neighborTable[slot].waitUpdateFrom; }
//        Lay ra dia chi cua neighbor giao tiep o 1 slot cu the
        MacAddress getCommunicationNeighbor(int slot) { return this->neighborTable[slot].address; }
//        Lay so thu tu slot giao tiep cua 1 neighbor
        int getCommunicationSlot(MacAddress address);

//        For ReviseHEAT
//        cap nhat thong tin HEAT vao trong neighbor table
        void updateHEATValue(MacAddress address, double PRR, simtime_t timeToGW);
//        Lay thong tin de tin toan gia tri HEAT hien tai
        NeighborHEATTable *getCurrentHEATTable();

//        For Container
//        Them goi tin moi vao 1 neighbor cu the
        bool addNewPacketTo(MacAddress address, LoRaAppPacket *packet);
//        Lay goi tin de chuan bi chuyen tiep cho neighbor
        LoRaAppPacket *peekPacket(MacAddress address);
//        Xoa goi tin khi goi tin da gui thanh cong
        void popPacket(MacAddress address);
//        Cap nhat trang thai goi tin da gui den 1 neighbor (success, fail)
        void updateSendPacketState(MacAddress address, bool state);
//        Queue chua goi tin day hay chua
        bool isFull(MacAddress address);
//        Queueuchua goi tin co goi tin hay khong
        bool isEmpty(MacAddress address);
//        Lay ra so luong neighbors da thiet lap ket noi
        int getConnectedCurrentNeighbors(){ return current_neighbors; }
//        Tinh toan chat luong cua 1 neighbor
        double getNeighborQuality(MacAddress addr);
//        chuyen goi tin sang hang doi khac de chuyen cho neighbor moi
        void changePath(MacAddress oldPath, MacAddress newPath);
    };
}



#endif /* LORAAPP_NEIGHBORTABLE_H_ */
