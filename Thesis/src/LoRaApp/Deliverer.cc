/*
 * Deliverer.cc
 *
 *  Created on: Feb 25, 2023
 *      Author: PHITRUONG
 */

#include "Deliverer.h"
#include "inet/mobility/static/StationaryMobility.h"
#include "../LoRa/LoRaTagInfo_m.h"
#include "inet/common/packet/Packet.h"
#include <iomanip>


namespace flora
{
    Define_Module(Deliverer);

    void Deliverer::initialize(int stage)
    {
        cSimpleModule::initialize(stage);
        if (stage == INITSTAGE_LOCAL) {
            EV << "Initializing stage " << stage << ", at Deliverer\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER) {
            EV << "Initializing stage " << stage << ", at Deliverer\n";
            bool isOperational;
            NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
            isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
            if (!isOperational)
                throw cRuntimeError("This module doesn't support starting in node DOWN state");

            LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

            //LoRa physical layer parameters
            loRaRadio = check_and_cast<LoRaRadio *>(getParentModule()->getParentModule()->getSubmodule("LoRaNic")->getSubmodule("radio"));
            loRaRadio->loRaTP = par("initialLoRaTP").doubleValue();
    //        setTP(par("initialLoRaTP").doubleValue());
    //        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
            loRaRadio->loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
    //        loRaSF = par("initialLoRaSF");
            loRaRadio->loRaSF = par("initialLoRaSF");
    //        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
            loRaRadio->loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
    //        loRaCR = par("initialLoRaCR");
            loRaRadio->loRaCR = par("initialLoRaCR");
    //        loRaUseHeader = par("initialUseHeader");
            loRaRadio->loRaUseHeader = par("initialUseHeader");

            myTDMA = check_and_cast<TDMA *>(getParentModule()->getSubmodule("TDMA"));
            this->currentNeighbors = 0;

            neighborsQuality = new NeighborsQuality[par("maxNeighbours").intValue()];
            for(int i=0; i < par("maxNeighbours").intValue(); i++)
            {
                this->neighborsQuality[i].addr = MacAddress::UNSPECIFIED_ADDRESS;
                for (int j = 0; j < 100; j++)
                    this->neighborsQuality[i].lastState[j] = true;
            }
        }
    }

    void Deliverer::finish()
    {
        cModule *host = getContainingNode(this);
        StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
        Coord coord = mobility->getCurrentPosition();
        recordScalar("positionX", coord.x);
        recordScalar("positionY", coord.y);
        recordScalar("finalTP", getTP());
        recordScalar("finalSF", getSF());
//        printTable();
    //    delete[] neighborsQuality;
    }

    void Deliverer::handleMessage(cMessage *msg)
    {
        if (msg->isSelfMessage()) {
            EV << "Hello windows!" << endl;
        }
        else
        {
            handleMessageFromUpperLayer(msg);
            delete msg;
        }
    }

    void Deliverer::handleMessageFromUpperLayer(cMessage *msg)
    {
        EV <<"This is handleMessageFromUpperLayer function!\n";
        EV << "This is packet from TDMA module\n" << endl;
        auto command = dynamic_cast<mCommand *>(msg);
        if(msg->getKind() == UPDATE_NEIGHBOR_INFO)
        {
            if(!isAlreadyExistNeighbor(command->getAddress()))
                this->addNewNeighbor(command->getAddress());
        }
        else if(msg->getKind() == SEND_INVITATION)
        {
            sendFreeslot_packet(this->myTDMA->getCurrentSlot(), command->getPRR(), command->getTimeToGW());
        }
        else if(msg->getKind() == SEND_REQUEST)
        {
            requestCommunicationSlot(command->getAddress(), command->getSlot());
        }
        else if(msg->getKind() == SEND_ACCEPT_ACK)
        {
            sendACKrequestSlot(command->getAddress(), command->getSlot(), true, command->getPRR(), command->getTimeToGW());
        }
        else if(msg->getKind() == SEND_REFUSE_ACK)
        {
            sendACKrequestSlot(command->getAddress(), command->getSlot(), false);
        }
        else if(msg->getKind() == SEND_UPDATE_HEAT)
        {
            sendHEAT_packet(command->getAddress(), command->getPRR(), command->getTimeToGW());
        }

    //    delete msg;
    }

    bool Deliverer::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
    {
        Enter_Method_Silent();

        throw cRuntimeError("Unsupported life-cycle operation '%s'", operation->getClassName());
        return true;
    }

    void Deliverer::sendHEAT_packet(MacAddress destination, double PRR, simtime_t timeToGW)
    {
        auto pktHEAT_Info = new Packet("HEAT Information");
        pktHEAT_Info->setKind(HEAT_INFO);

        auto payload = makeShared<LoRaAppPacket>();
        payload->setChunkLength(B(par("dataSize").intValue()));
        payload->setMsgType(HEAT_INFO);

        payload->setPRR(PRR);
        payload->setTimeToGW(timeToGW);

        auto loraTag = pktHEAT_Info->addTagIfAbsent<LoRaTag>();
        loraTag->setBandwidth(getBW());
        loraTag->setCenterFrequency(getCF());
        loraTag->setSpreadFactor(getSF());
        loraTag->setCodeRendundance(getCR());
        loraTag->setPower(mW(math::dBmW2mW(getTP())));
        loraTag->setDestination(destination);

        pktHEAT_Info->insertAtBack(payload);
        send(pktHEAT_Info, "socketOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Deliverer::sendDATA_packet(MacAddress destination, LoRaAppPacket packet)
    {
        auto pktData = new Packet("Sensor Data");
        pktData->setKind(DATA_PACKET);

        auto payload = makeShared<LoRaAppPacket>(packet);
        payload->setMsgType(DATA_PACKET);

        auto loraTag = pktData->addTagIfAbsent<LoRaTag>();
        loraTag->setBandwidth(getBW());
        loraTag->setCenterFrequency(getCF());
        loraTag->setSpreadFactor(getSF());
        loraTag->setCodeRendundance(getCR());
        loraTag->setPower(mW(math::dBmW2mW(getTP())));
        loraTag->setDestination(destination);

        pktData->insertAtBack(payload);
        send(pktData, "socketOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Deliverer::sendACK_packet(MacAddress destination, bool success)
    {

    }

    void Deliverer::sendFreeslot_packet(int slot, double myPRR, simtime_t timeToGW)
    {
        auto pktFreeSlot = new Packet("Join Invitation");
        pktFreeSlot->setKind(JOIN_INVITE);

        auto payload = makeShared<LoRaAppPacket>();
        payload->setChunkLength(B(par("dataSize").intValue()));
        payload->setMsgType(JOIN_INVITE);

        payload->setMsgType(JOIN_INVITE);
        payload->setSlot(slot);
        payload->setPRR(myPRR);
        payload->setTimeToGW(timeToGW);

        auto loraTag = pktFreeSlot->addTagIfAbsent<LoRaTag>();
        loraTag->setBandwidth(getBW());
        loraTag->setCenterFrequency(getCF());
        loraTag->setSpreadFactor(getSF());
        loraTag->setCodeRendundance(getCR());
        loraTag->setPower(mW(math::dBmW2mW(getTP())));
        loraTag->setDestination(MacAddress::BROADCAST_ADDRESS);

        pktFreeSlot->insertAtBack(payload);

        send(pktFreeSlot, "socketOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Deliverer::requestCommunicationSlot(MacAddress destination, int slot)
    {
        auto pktJoinRequest = new Packet("Join Request");
        pktJoinRequest->setKind(JOIN_REQUEST);

        auto payload = makeShared<LoRaAppPacket>();
        payload->setChunkLength(B(par("dataSize").intValue()));
        payload->setMsgType(JOIN_REQUEST);

        payload->setSlot(slot);

        auto loraTag = pktJoinRequest->addTagIfAbsent<LoRaTag>();
        loraTag->setBandwidth(getBW());
        loraTag->setCenterFrequency(getCF());
        loraTag->setSpreadFactor(getSF());
        loraTag->setCodeRendundance(getCR());
        loraTag->setPower(mW(math::dBmW2mW(getTP())));
        loraTag->setDestination(destination);

        pktJoinRequest->insertAtBack(payload);
        send(pktJoinRequest, "socketOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Deliverer::sendACKrequestSlot(MacAddress destination, int slot, bool accept, double myPRR, simtime_t timeToGW)
    {
        auto pktJoinReply = new Packet("Join Reply");
        pktJoinReply->setKind(accept?JOIN_ACCEPT_REPLY:JOIN_REFUSE_REPLY);

        auto payload = makeShared<LoRaAppPacket>();
        payload->setChunkLength(B(par("dataSize").intValue()));

        payload->setMsgType(accept?JOIN_ACCEPT_REPLY:JOIN_REFUSE_REPLY);
        payload->setSlot(slot);
        payload->setPRR(myPRR);
        payload->setTimeToGW(timeToGW);

        auto loraTag = pktJoinReply->addTagIfAbsent<LoRaTag>();
        loraTag->setBandwidth(getBW());
        loraTag->setCenterFrequency(getCF());
        loraTag->setSpreadFactor(getSF());
        loraTag->setCodeRendundance(getCR());
        loraTag->setPower(mW(math::dBmW2mW(getTP())));
        loraTag->setDestination(destination);

        pktJoinReply->insertAtBack(payload);
        send(pktJoinReply, "socketOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Deliverer::setSF(int SF) {
        loRaRadio->loRaSF = SF;
    }

    int Deliverer::getSF() {
        return loRaRadio->loRaSF;
    }

    void Deliverer::setTP(int TP) {
        loRaRadio->loRaTP = TP;
    }

    double Deliverer::getTP() {
        return loRaRadio->loRaTP;
    }

    void Deliverer::setCF(units::values::Hz CF) {
        loRaRadio->loRaCF = CF;
    }

    units::values::Hz Deliverer::getCF() {
        return loRaRadio->loRaCF;
    }

    void Deliverer::setBW(units::values::Hz BW) {
        loRaRadio->loRaBW = BW;
    }

    units::values::Hz Deliverer::getBW() {
        return loRaRadio->loRaBW;
    }

    void Deliverer::setCR(int CR) {
        loRaRadio->loRaCR = CR;
    }

    int Deliverer::getCR() {
        return loRaRadio->loRaCR;
    }


    bool Deliverer::isAlreadyExistNeighbor(MacAddress addr)
    {
        for(int i=0; i< this->currentNeighbors; i++)
        {
            if(this->neighborsQuality[i].addr == addr)
                return true;
        }
        return false;
    }
    void Deliverer::addNewNeighbor(MacAddress addr)
    {
        this->neighborsQuality[currentNeighbors++].addr = addr;
    }

    double Deliverer::getNeighborQuality(MacAddress addr)
    {
        int success = 0;
        for(int i = 0; i < this->currentNeighbors; i++)
        {
            if(this->neighborsQuality[i].addr == addr)
            {
                for (auto state:this->neighborsQuality[i].lastState)
                {
                    if (state)
                        success++;
                }
            }
        }

        return success/100.0;
    }

    void Deliverer::updateNeighborQuality(MacAddress addr, bool state)
    {
        for(int i = 0; i < this->currentNeighbors; i++)
        {
            if(this->neighborsQuality[i].addr == addr)
            {
                for (int j = 0; i < 99; i++)
                {
                    this->neighborsQuality[i].lastState[j] = this->neighborsQuality[i].lastState[j + 1];
                }
                this->neighborsQuality[i].lastState[99] = state;
            }
        }
    }

    void Deliverer::printTable()
    {
        EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << "Address" <<std::left << std::setw(2) << " " <<
                    std::left << std::setw(2) << "|" << std::left << std::setw(8) << "Quality" <<std::left << std::setw(2) << " " << endl;

        for(int i = 0; i < this->currentNeighbors; i++){
                EV_INFO << std::left << std::setw(2) << "|" << std::left << std::setw(17) << this->neighborsQuality[i].addr <<std::left << std::setw(2) << " " <<
                        std::left << std::setw(2) << "|" << std::left << std::setw(8) << this->getNeighborQuality(this->neighborsQuality[i].addr) <<std::left << std::setw(2) << " " << endl;
        }
    }

}

