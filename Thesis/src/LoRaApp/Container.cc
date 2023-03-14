/*
 * Container.cc
 *
 *  Created on: Feb 25, 2023
 *      Author: PHITRUONG
 */

#include "Container.h"
#include "ReviseHEAT.h"
#include "inet/mobility/static/StationaryMobility.h"
#include "../LoRa/LoRaTagInfo_m.h"
#include "inet/common/packet/Packet.h"
#include <iomanip>

namespace flora
{
    Define_Module(Container);

    void Container::initialize(int stage)
    {
        cSimpleModule::initialize(stage);
        if (stage == INITSTAGE_LOCAL)
        {
            EV << "Initializing stage " << stage << ", at Container\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER)
        {
            EV << "Initializing stage " << stage << ", at Container\n";
            this->max_communication_neighbors = getParentModule()->par("max_neighbors").intValue();
            this->currentNeighbors = 0;
            this->iAmGateway = getParentModule()->par("iAmGateway").boolValue();
            this->maxSendTimes = getParentModule()->par("send_again_times").intValue();

            this->neighborTable = check_and_cast<NeighborTable *>(getParentModule()->getSubmodule("NeighborTable"));
            this->myHEATer = check_and_cast<ReviseHEAT *>(getParentModule()->getSubmodule("ReviseHEAT"));
            if(!iAmGateway)
            {
                generatePacket = new cMessage("Generate packet");
                scheduleAt(simTime() + par("start_generate"), generatePacket);
            }
            this->fsmState = NORMAL;
//            For statistics
            this->macLayer = check_and_cast<LoRaMac* >(getParentModule()->getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
            this->sMachine = check_and_cast<StatisticalModule *>(getParentModule()->getSubmodule("StatisticalModule"));
        }
    }

    void Container::finish()
    {
        if(!iAmGateway)
        {
            EV <<"The number of successfully generated and dispatched packets is: " << this->generatedPacketNum << endl;
            EV <<"The number of failed generated and dispatched packets is: " << this->generatedPacketFailNum << endl;
            cancelAndDelete(generatePacket);
        }
    }

    bool Container::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
    {
        Enter_Method_Silent();

        throw cRuntimeError("Unsupported life-cycle operation '%s'", operation->getClassName());
        return true;
    }

    void Container::handleMessage(cMessage *msg)
    {
        if (msg->isSelfMessage())
            handleSelfMessage(msg);
        else
        {
            handleWithFsm(msg);
            delete msg;
        }
    }

    void Container::handleSelfMessage(cMessage *msg)
    {
        if (msg == generatePacket)
        {
            this->generatingPacket();
            scheduleAt(simTime() + par("generate_packet_interval"), generatePacket);
        }
        else if (msg == timeoutWaitACK)
        {
            if(++sentTimes < maxSendTimes)
            {
                this->forwardDataPacket(waitingAckFrom, true);
                scheduleAt(simTime() + par("timeoutACK"), timeoutWaitACK);
            }
            else
            {
                delete msg;

                if(this->updateSentPacketState(waitingAckFrom, false))
                {
                    if(this->forwardDataPacket(waitingAckFrom))
                    {
                        this->fsmState = WAITINGACK;

                        sentTimes = 0;
                        timeoutWaitACK = new cMessage("Timeout for waiting an acknowledge packet");
                        scheduleAt(simTime() + par("timeoutACK"), timeoutWaitACK);
                        return;
                    }
                }
                this->fsmState = NORMAL;
            }
        }
    }

    void Container::handleWithFsm(cMessage *msg)
    {
        EV <<"The current state is: " << this->fsmState << endl;
        switch (this->fsmState)
        {
        case NORMAL:
            if(msg->getKind() == SEND_DATA_PACKET)
            {
                EV <<"This is normal state, and address = " << (check_and_cast<mCommand* >(msg))->getAddress() << endl;
                if(this->forwardDataPacket(check_and_cast<mCommand* >(msg)->getAddress()))
                {
                    this->fsmState = WAITINGACK;
                    sentTimes = 0;
                    timeoutWaitACK = new cMessage("Timeout for waiting an acknowledge packet");
                    scheduleAt(simTime() + par("timeoutACK"), timeoutWaitACK);
                }
            }
            else if(msg->getKind() == DATA_PACKET)
            {
                this->updateStatisticalData(check_and_cast<Packet *>(msg));

                if (iAmGateway)
                    this->sendACKCommand(check_and_cast<Packet *>(msg), true);
                else
                {
                    if(this->disPatchPacket(check_and_cast<Packet *>(msg)))
                        this->sendACKCommand(check_and_cast<Packet *>(msg), true);
                    else
                        this->sendACKCommand(check_and_cast<Packet *>(msg), false);
                }
            }
            break;

        case WAITINGACK:
            if(msg->getKind() == RECEIVED_SUCCESS)
            {
                EV << "This is a receive success messsage" << endl;
                if(timeoutWaitACK->isScheduled())
                    cancelAndDelete(timeoutWaitACK);

                this->updateSentPacketState(waitingAckFrom, true);
                this->sMachine->updateStatisticalLoRaData(waitingAckFrom, true);

                if(forwardDataPacket(waitingAckFrom))
                {
                    sentTimes = 0;
                    timeoutWaitACK = new cMessage("Timeout for waiting an acknowledge packet");
                    scheduleAt(simTime() + par("timeoutACK"), timeoutWaitACK);
                }
                else
                    this->fsmState = NORMAL;

            }
            else if(msg->getKind() == UNABLE_RECEIVE_MORE)
            {
                if(timeoutWaitACK->isScheduled())
                    cancelAndDelete(timeoutWaitACK);

                if(this->updateNeighborInfo(check_and_cast<Packet *>(msg)))
                {
                    if(forwardDataPacket(waitingAckFrom))
                    {
                        sentTimes = 0;
                        timeoutWaitACK = new cMessage("Timeout for waiting an acknowledge packet");
                        scheduleAt(simTime() + par("timeoutACK"), timeoutWaitACK);
                        return;
                    }
                }
                this->fsmState = NORMAL;
            }

            break;
        }
    }

    bool Container::disPatchPacket(LoRaAppPacket *packet)
    {
        packet->setHopNum(packet->getHopNum() + 1);
        packet->setPayload(packet->getPayload());
        packet->setSeqNum(packet->getSeqNum());

        MacAddress destAddr = MacAddress::UNSPECIFIED_ADDRESS;
        destAddr = myHEATer->getCurrentPathToGW();

        if(destAddr == MacAddress::UNSPECIFIED_ADDRESS)
           return false;

        return this->neighborTable->addNewPacketTo(destAddr, packet);
    }

    bool Container::disPatchPacket(Packet *packet)
    {
        const auto &pkt = packet->peekAtFront<LoRaAppPacket>();
//        check seqNum of previous packet

        auto myCurrentHEAT = this->myHEATer->getCurrentHEAT();
        if(myCurrentHEAT.currentValue.PRR != pkt->getPRR() || myHEATer->getCurrentTimeToGW() != pkt->getTimeToGW())
            return false;

        if(pkt->getOriginAddress() != macLayer->getAddress())
        {
            if(prevSeqNum == pkt->getSeqNum())
                return true;
            prevSeqNum = pkt->getSeqNum();
        }

        auto newPkt = new LoRaAppPacket();
        newPkt->setHopNum(pkt->getHopNum());
        newPkt->setPayload(pkt->getPayload());
        newPkt->setSeqNum(pkt->getSeqNum());
        newPkt->setOriginAddress(pkt->getOriginAddress());


        return this->disPatchPacket(newPkt);
    }

    void Container::generatingPacket()
    {
        auto newPkt = new LoRaAppPacket();
        newPkt->setHopNum(0);
        newPkt->setPayload(rand());
        newPkt->setSeqNum(intrand(UINT32_MAX));
        newPkt->setOriginAddress(macLayer->getAddress());
        newPkt->setGeneratedTime(simTime());

        if(this->disPatchPacket(newPkt))
            this->generatedPacketNum += 1;
        else
            this->generatedPacketFailNum += 1;
    }

    bool Container::forwardDataPacket(MacAddress addr, /*for statics*/ bool again)
    {
        if (this->neighborTable->isEmpty(addr))
            return false;

        auto dataPacket = this->neighborTable->peekPacket(addr);
        auto neighborHEAT = this->neighborTable->getHEATof(addr);
        mCommand *sendDataCommand = new mCommand("Give command for Transmitter send Data packet");
        sendDataCommand->setKind(SEND_DATA_PACKET);
        sendDataCommand->setData(dataPacket);
        sendDataCommand->setPRR(neighborHEAT->PRR);
        sendDataCommand->setTimeToGW(neighborHEAT->timeToGW);
        sendDataCommand->setAddress(addr);

        send(sendDataCommand, "To_Transmitter");
        delete neighborHEAT;

        waitingAckFrom = addr;
        this->sMachine->updateStatisticalLoRaData(addr, false, again);
//        TODO
        return true;
    }
    void Container::sendACKCommand(Packet *pkt, bool status)
    {
        auto addr = pkt->getTag<MacAddressInd>();
        const auto &packet = pkt->peekAtFront<LoRaAppPacket>();

        mCommand *ackCommand = new mCommand("Give command for Transmitter send acknowledge packet");
        ackCommand->setKind(status ? SEND_ACK_SUCCESS : SEND_ACK_FAIL);
        ackCommand->setAddress(addr->getSrcAddress());
        ackCommand->setSeqNum(packet->getSeqNum());
        if(!status)
        {
            this->myHEATer->recalculateCurrentHEAT();
            auto myCurrentHEAT = this->myHEATer->getCurrentHEAT();
            ackCommand->setPRR(myCurrentHEAT.currentValue.PRR);
            ackCommand->setTimeToGW(this->myHEATer->getCurrentTimeToGW());
        }

        send(ackCommand, "To_Transmitter");
    }

    bool Container::updateNeighborInfo(Packet *pkt)
    {
        auto addr = pkt->getTag<MacAddressInd>();
        const auto &packet = pkt->peekAtFront<LoRaAppPacket>();

        MacAddress oldDest = this->myHEATer->getCurrentPathToGW();
        this->myHEATer->updateNeighborTable(addr->getSrcAddress(), packet->getPRR(), packet->getTimeToGW());
        MacAddress newDest = this->myHEATer->getCurrentPathToGW();

//        chuyen goi tin sang hang doi khac de chuyen cho neighbor moi
        this->neighborTable->changePath(oldDest, newDest);

        return (oldDest == newDest);
    }

    bool Container::updateSentPacketState(MacAddress addr, bool state)
    {
        if (state)
            this->neighborTable->popPacket(addr);

        MacAddress oldDest = this->myHEATer->getCurrentPathToGW();
        this->myHEATer->updateSentState(addr, state);
        MacAddress newDest = this->myHEATer->getCurrentPathToGW();

//        chuyen goi tin sang hang doi khac de chuyen cho neighbor moi
        this->neighborTable->changePath(oldDest, newDest);

        return (oldDest == newDest);
    }
    void Container::updateStatisticalData(Packet *packet)
    {
        auto addr = packet->getTag<MacAddressInd>();
        const auto &pkt = packet->peekAtFront<LoRaAppPacket>();

        this->sMachine->updateStatisticalLoRaData(pkt->getOriginAddress(), pkt->getHopNum());
    }
}
