/*
 * Transmitter.cc
 *
 *  Created on: Mar 7, 2023
 *      Author: PHITRUONG
 */

#include "Transmitter.h"
#include "LoRa/LoRaTagInfo_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"

namespace flora
{
Define_Module(Transmitter);
    void Transmitter::initialize(int stage)
    {
        cSimpleModule::initialize(stage);
        if (stage == INITSTAGE_LOCAL)
        {
            EV << "Initializing stage " << stage << ", at Transmitter\n";
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER)
        {
            EV << "Initializing stage " << stage << ", at Transmitter\n";
            bool isOperational;
            NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
            isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
            if (!isOperational)
                throw cRuntimeError("This module doesn't support starting in node DOWN state");

            LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

            //LoRa physical layer parameters
            loRaRadio = check_and_cast<LoRaRadio *>(getParentModule()->getParentModule()->getSubmodule("LoRaNic")->getSubmodule("radio"));
            loRaRadio->loRaTP = par("initialLoRaTP").doubleValue();
            loRaRadio->loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
            loRaRadio->loRaSF = par("initialLoRaSF");
            loRaRadio->loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
            loRaRadio->loRaCR = par("initialLoRaCR");
            loRaRadio->loRaUseHeader = par("initialUseHeader");
        }
    }

    void Transmitter::finish()
    {

    }

    bool Transmitter::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
    {
        Enter_Method_Silent();

        throw cRuntimeError("Unsupported life-cycle operation '%s'", operation->getClassName());
        return true;
    }

    void Transmitter::handleMessage(cMessage *msg)
    {
        if(msg->isSelfMessage())
        {
            throw "Message don't know";
        }
        else
        {
            auto command = check_and_cast<mCommand *>(msg);
            switch(command->getKind())
            {
            case SEND_INVITATION:
                EV << command->getSlot() << ", " << command->getPRR() << ", " << command->getTimeToGW() << endl;
                sendFreeslot_packet(command->getSlot(), command->getPRR(), command->getTimeToGW());
                break;
            case SEND_REQUEST:
                requestCommunicationSlot(command->getAddress(), command->getSlot());
                break;
            case SEND_ACCEPT_ACK:
                sendACKrequestSlot(command->getAddress(), command->getSlot(), true, command->getPRR(), command->getTimeToGW());
                break;
            case SEND_REFUSE_ACK:
                sendACKrequestSlot(command->getAddress(), command->getSlot(), false, command->getPRR(), command->getTimeToGW());
                break;
            case UPDATE_NEIGHBOR_INFO:
                throw "UPDATE Neighbor info";
                break;
            case SEND_UPDATE_HEAT:
                sendHEAT_packet(command->getAddress(), command->getPRR(), command->getTimeToGW());
                break;
            case SEND_DATA_PACKET:
                break;
            case SEND_ACK:
                break;
            }
        }
    }

    void Transmitter::sendHEAT_packet(MacAddress destination, double PRR, simtime_t timeToGW)
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
        send(pktHEAT_Info, "lowerLayerOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Transmitter::sendDATA_packet(Packet *packet)
    {
        auto loraTag = packet->addTagIfAbsent<LoRaTag>();
        loraTag->setBandwidth(getBW());
        loraTag->setCenterFrequency(getCF());
        loraTag->setSpreadFactor(getSF());
        loraTag->setCodeRendundance(getCR());
        loraTag->setPower(mW(math::dBmW2mW(getTP())));

        send(packet, "lowerLayerOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Transmitter::sendACK_packet(MacAddress destination, bool success)
    {

    }

    void Transmitter::sendFreeslot_packet(int slot, double myPRR, simtime_t timeToGW)
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

        send(pktFreeSlot, "lowerLayerOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Transmitter::requestCommunicationSlot(MacAddress destination, int slot)
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
        send(pktJoinRequest, "lowerLayerOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Transmitter::sendACKrequestSlot(MacAddress destination, int slot, bool accept, double myPRR, simtime_t timeToGW)
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
        send(pktJoinReply, "lowerLayerOut");
        emit(LoRa_AppPacketSent, getSF());
    }

    void Transmitter::setSF(int SF) {
        loRaRadio->loRaSF = SF;
    }

    int Transmitter::getSF() {
        return loRaRadio->loRaSF;
    }

    void Transmitter::setTP(int TP) {
        loRaRadio->loRaTP = TP;
    }

    double Transmitter::getTP() {
        return loRaRadio->loRaTP;
    }

    void Transmitter::setCF(units::values::Hz CF) {
        loRaRadio->loRaCF = CF;
    }

    units::values::Hz Transmitter::getCF() {
        return loRaRadio->loRaCF;
    }

    void Transmitter::setBW(units::values::Hz BW) {
        loRaRadio->loRaBW = BW;
    }

    units::values::Hz Transmitter::getBW() {
        return loRaRadio->loRaBW;
    }

    void Transmitter::setCR(int CR) {
        loRaRadio->loRaCR = CR;
    }

    int Transmitter::getCR() {
        return loRaRadio->loRaCR;
    }
}

