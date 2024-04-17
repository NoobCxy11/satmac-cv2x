//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//
// This file is an extension of SimuLTE
// Author: Brian McCarthy
// email : b.mccarthy@cs.ucc.ie


#include <math.h>
#include <assert.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include "stack/phy/layer/SatPhy.h"
#include "stack/phy/packet/LteFeedbackPkt.h"
#include "stack/d2dModeSelection/D2DModeSelectionBase.h"
#include "stack/phy/packet/SpsCandidateResources.h"
#include "stack/phy/packet/cbr_m.h"

Define_Module(SatPhy);

SatPhy::SatPhy()
{
    handoverStarter_ = NULL;
    handoverTrigger_ = NULL;
}

SatPhy::~SatPhy()
{
}

void SatPhy::initialize(int stage)
{
    if (stage != inet::INITSTAGE_NETWORK_LAYER_2)
        LtePhyUeD2D::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL)
    {
        adjacencyPSCCHPSSCH_             = par("adjacencyPSCCHPSSCH");
        //randomScheduling_                = par("randomScheduling");
        sensingWindowSizeOverride_       = par("sensingWindowSizeOverride");
        pStep_                           = par("pStep");
        selectionWindowStartingSubframe_ = par("selectionWindowStartingSubframe");
        numSubchannels_                  = par("numSubchannels");
//        numSubchannels_ = 1;
        subchannelSize_                  = par("subchannelSize");
        d2dDecodingTimer_                = NULL;
        transmitting_                    = false;
        beginTransmission_               = false;
        rssiFiltering_                   = par("rssiFiltering");
        rsrpFiltering_                   = par("rsrpFiltering");

        checkAwareness_                  = par("checkAwareness");

        int thresholdRSSI                = par("thresholdRSSI");

        thresholdRSSI_ = (-112 + 2 * thresholdRSSI);

        d2dTxPower_ = par("d2dTxPower");
        if (d2dTxPower_ <= 0){
            d2dTxPower_ = txPower_;
        }

        // The threshold has a size of 64, and allowable values of 0 - 66
        // Deciding on this for now as it makes the most sense (low priority for both then more likely to take it)
        // High priority for both then less likely to take it.
        for (int i = 1; i < 66; i++)
        {
            ThresPSSCHRSRPvector_.push_back(i);
        }

        // SCI stats
        sciSent                     = registerSignal("sciSent");

        sciReceived                 = registerSignal("sciReceived");
        sciDecoded                  = registerSignal("sciDecoded");
        csrReschedule               = registerSignal("csrReschedule");

        sciFailedDueToProp          = registerSignal("sciFailedDueToProp");
        sciFailedDueToInterference  = registerSignal("sciFailedDueToInterference");
        sciUnsensed                 = registerSignal("sciUnsensed");
        sciFailedHalfDuplex         = registerSignal("sciFailedHalfDuplex");

        txRxDistanceSCI             = registerSignal("txRxDistanceSCI");

        sciReceived_ = 0;
        sciDecoded_ = 0;

        sciFailedHalfDuplex_ = 0;
        sciFailedDueToProp_ = 0;
        sciFailedDueToInterference_ = 0;
        sciUnsensed_ = 0;


        // TB Stats
        tbSent                             = registerSignal("tbSent");

        tbReceived                         = registerSignal("tbReceived");
        tbDecoded                          = registerSignal("tbDecoded");

        //periodic                           = registerSignal("periodic");

        tbFailedDueToNoSCI                 = registerSignal("tbFailedDueToNoSCI");
        tbFailedButSCIReceived             = registerSignal("tbFailedButSCIReceived");
        tbAndSCINotReceived                = registerSignal("tbAndSCINotReceived");

        tbFailedHalfDuplex                 = registerSignal("tbFailedHalfDuplex");
        tbFailedDueToProp                  = registerSignal("tbFailedDueToProp");
        tbFailedDueToInterference          = registerSignal("tbFailedDueToInterference");

        tbFailedDueToPropIgnoreSCI         = registerSignal("tbFailedDueToPropIgnoreSCI");
        tbFailedDueToInterferenceIgnoreSCI = registerSignal("tbFailedDueToInterferenceIgnoreSCI");
        tbDecodedIgnoreSCI                 = registerSignal("tbDecodedIgnoreSCI");

        txRxDistanceTB                     = registerSignal("txRxDistanceTB");

        collisionCount              = registerSignal("collisionCount");

        tbReceived_ = 0;
        tbDecoded_ = 0;

        tbFailedDueToNoSCI_ = 0;
        tbFailedButSCIReceived_ = 0;
        tbAndSCINotReceived_ = 0;
        tbFailedHalfDuplex_ = 0;
        tbFailedDueToProp_ = 0;
        tbFailedDueToInterference_ = 0;
        tbFailedDueToPropIgnoreSCI_ = 0;
        tbFailedDueToInterferenceIgnoreSCI_ = 0;
        tbDecodedIgnoreSCI_ = 0;

        // General stats

        cbr                         = registerSignal("cbr");
        //cbrPscch                    = registerSignal("cbrPscch");
        threshold                   = registerSignal("threshold");

        subchannelReceived          = registerSignal("subchannelReceived");
        subchannelsUsed             = registerSignal("subchannelsUsed");
        senderID                    = registerSignal("senderID");
        subchannelSent              = registerSignal("subchannelSent");
        subchannelsUsedToSend       = registerSignal("subchannelsUsedToSend");
        interPacketDelay            = registerSignal("interPacketDelay");

        awareness1sStat             = registerSignal("awareness1sStat");
        awareness500msStat          = registerSignal("awareness500msStat");
        awareness200msStat          = registerSignal("awareness200msStat");

        posX                        = registerSignal("posX");
        posY                        = registerSignal("posY");

        subchannelReceived_ = 0;
        subchannelsUsed_ = 0;

        sensingWindowFront_ = 0; // Will ensure when we first update the sensing window we don't skip over the first element

        cbrCountDown_ = intuniform(0, 1000);
    }
    else if (stage == INITSTAGE_NETWORK_LAYER_2)
    {
        // Need to start initialising the sensingWindow
        deployer_ = getDeployer();
        int index = intuniform(0, binder_->phyPisaData.maxChannel() - 1);
        deployer_->lambdaInit(nodeId_, index);
        deployer_->channelUpdate(nodeId_, intuniform(1, binder_->phyPisaData.maxChannel2()));

        nodeId_ = getAncestorPar("macNodeId");

        initialiseSensingWindow();
    }
}

void SatPhy::handleSelfMessage(cMessage *msg)
{
    if (msg->isName("d2dDecodingTimer"))
    {
        std::sort(begin(sciInfo_), end(sciInfo_), [](const std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> &t1,
        const std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> &t2) {
            return get<5>(t1) < get<5>(t2); // or use a custom compare function
        });

        std::sort(begin(tbInfo_), end(tbInfo_), [](const std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> &t1,
        const std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> &t2) {
            return get<5>(t1) < get<5>(t2); // or use a custom compare function
        });

        std::vector<int> missingTbs;
        for (int i=0; i<sciInfo_.size(); i++){
            bool foundTB=false;
            LteAirFrame* sciFrame = get<0>(sciInfo_[i]);
            UserControlInfo* sciInfo = check_and_cast<UserControlInfo*>(sciFrame->removeControlInfo());
            for (int j=0; j<tbInfo_.size();j++){
                LteAirFrame* tbFrame = get<0>(tbInfo_[j]);
                UserControlInfo* tbInfo = check_and_cast<UserControlInfo*>(tbFrame->removeControlInfo());
                if (sciInfo->getSourceId() == tbInfo->getSourceId()){
                    foundTB = true;
                    tbFrame->setControlInfo(tbInfo);
                    break;
                }
                tbFrame->setControlInfo(tbInfo);
            }
            if (!foundTB){
                missingTbs.push_back(sciInfo_.size() - 1 - i);
            }
            sciFrame->setControlInfo(sciInfo);
        }

        while (!sciInfo_.empty()){
            std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> sciTuple = sciInfo_.back();
            // Get received SCI and it's corresponding RsrpVector
            LteAirFrame* frame = get<0>(sciTuple);
            std::vector<double> rsrpVector = get<1>(sciTuple);
            std::vector<double> rssiVector = get<2>(sciTuple);
            std::vector<double> sinrVector = get<3>(sciTuple);
            double attenuation = get<4>(sciTuple);

            sciInfo_.pop_back();

            UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(frame->removeControlInfo());

            // decode the selected frame
            decodeAirFrame(frame, lteInfo, rsrpVector, rssiVector, sinrVector, attenuation);

            emit(sciReceived, sciReceived_);
            emit(sciUnsensed, sciUnsensed_);
            emit(sciDecoded, sciDecoded_);
            emit(sciFailedDueToProp, sciFailedDueToProp_);
            emit(sciFailedDueToInterference, sciFailedDueToInterference_);
            emit(sciFailedHalfDuplex, sciFailedHalfDuplex_);
            emit(subchannelReceived, subchannelReceived_);
            emit(subchannelsUsed, subchannelsUsed_);

            sciReceived_ = 0;
            sciDecoded_ = 0;
            sciFailedHalfDuplex_ = 0;
            sciFailedDueToInterference_ = 0;
            sciFailedDueToProp_ = 0;
            subchannelReceived_ = 0;
            subchannelsUsed_ = 0;
            sciUnsensed_ = 0;

        }
        int countTbs = 0;
        while (!tbInfo_.empty()){
            auto missingPosition = std::find(missingTbs.begin(), missingTbs.end(), countTbs);
            if(missingPosition != missingTbs.end()) {
                missingTbs.erase(missingPosition);
                // This corresponds to where we are missing a TB, record results as being negative to identify this.
                emit(txRxDistanceTB, -1);
                emit(tbReceived, -1);
                emit(tbDecoded, -1);
                emit(tbFailedDueToNoSCI, -1);
                emit(tbFailedDueToProp, -1);
                emit(tbFailedDueToInterference, -1);
                emit(tbFailedButSCIReceived, -1);
                emit(tbFailedHalfDuplex, -1);
                //emit(periodic, -1);

                emit(tbFailedDueToPropIgnoreSCI ,-1);
                emit(tbFailedDueToInterferenceIgnoreSCI ,-1);
                emit(tbDecodedIgnoreSCI ,-1);
            } else {
                std::tuple<LteAirFrame*, std::vector<double>, std::vector<double>, std::vector<double>, double, double> tbTuple = tbInfo_.back();
                LteAirFrame* frame = get<0>(tbTuple);
                std::vector<double> rsrpVector = get<1>(tbTuple);
                std::vector<double> rssiVector = get<2>(tbTuple);
                std::vector<double> sinrVector = get<3>(tbTuple);
                double attenuation = get<4>(tbTuple);

                tbInfo_.pop_back();

                UserControlInfo *lteInfo = check_and_cast<UserControlInfo *>(frame->removeControlInfo());

                // decode the selected frame
                decodeAirFrame(frame, lteInfo, rsrpVector, rssiVector, sinrVector, attenuation);

                emit(tbReceived, tbReceived_);
                emit(tbDecoded, tbDecoded_);
                emit(tbFailedDueToNoSCI, tbFailedDueToNoSCI_);
                emit(tbFailedDueToProp, tbFailedDueToProp_);
                emit(tbFailedDueToInterference, tbFailedDueToInterference_);
                emit(tbFailedButSCIReceived, tbFailedButSCIReceived_);
                emit(tbFailedHalfDuplex, tbFailedHalfDuplex_);
                //emit(periodic, int(lteInfo->getPeriodic()));

                emit(tbFailedDueToPropIgnoreSCI ,tbFailedDueToPropIgnoreSCI_);
                emit(tbFailedDueToInterferenceIgnoreSCI ,tbFailedDueToInterferenceIgnoreSCI_);
                emit(tbDecodedIgnoreSCI ,tbDecodedIgnoreSCI_);

                tbReceived_ = 0;
                tbDecoded_ = 0;
                tbFailedDueToNoSCI_ = 0;
                tbFailedButSCIReceived_ = 0;
                tbFailedHalfDuplex_ = 0;
                tbFailedDueToProp_ = 0;
                tbFailedDueToInterference_ = 0;

                tbFailedDueToPropIgnoreSCI_ = 0;
                tbFailedDueToInterferenceIgnoreSCI_ = 0;
                tbDecodedIgnoreSCI_ = 0;
            }
            countTbs++;
        }
        if (!missingTbs.empty()){
            for(int i=0; i<missingTbs.size(); i++){
                emit(txRxDistanceTB, -1);
                emit(tbReceived, -1);
                emit(tbDecoded, -1);
                emit(tbFailedDueToNoSCI, -1);
                emit(tbFailedDueToProp, -1);
                emit(tbFailedDueToInterference, -1);
                emit(tbFailedButSCIReceived, -1);
                emit(tbFailedHalfDuplex, -1);
                //emit(periodic, -1);

                emit(tbFailedDueToPropIgnoreSCI ,-1);
                emit(tbFailedDueToInterferenceIgnoreSCI ,-1);
                emit(tbDecodedIgnoreSCI ,-1);
            }
        }
        std::vector<cPacket*>::iterator it;
        for(it=scis_.begin();it!=scis_.end();it++)
        {
            delete(*it);
        }
        sciInfo_.clear();
        tbInfo_.clear();
        scis_.clear();
        delete msg;
        d2dDecodingTimer_ = NULL;
    }
    else if (msg->isName("updateSubframe"))
    {
        transmitting_ = false;
        if (beginTransmission_){
            transmitting_ = true;
            beginTransmission_ = false;
        }
        updateSubframe();
        delete msg;
    }
    else
        LtePhyUe::handleSelfMessage(msg);
}

// TODO: ***reorganize*** method
void SatPhy::handleAirFrame(cMessage* msg)
{
    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    connectedNodeId_ = masterId_;
    LteAirFrame* frame = check_and_cast<LteAirFrame*>(msg);
    EV << "SatPhy: received new LteAirFrame with ID " << frame->getId() << " from channel" << endl;
    //Update coordinates of this user
    if (lteInfo->getFrameType() == HANDOVERPKT)
    {
        // check if handover is already in process
        if (handoverTrigger_ != NULL && handoverTrigger_->isScheduled())
        {
            delete lteInfo;
            delete frame;
            return;
        }

        handoverHandler(frame, lteInfo);
        return;
    }

    // HACK: if this is a multicast connection, change the destId of the airframe so that upper layers can handle it
    // All packets in mode 4 are multicast
    lteInfo->setDestId(nodeId_);

    // send H-ARQ feedback up
    if (lteInfo->getFrameType() == HARQPKT || lteInfo->getFrameType() == GRANTPKT || lteInfo->getFrameType() == RACPKT || lteInfo->getFrameType() == D2DMODESWITCHPKT)
    {
        handleControlMsg(frame, lteInfo);
        return;
    }

    // this is a DATA packet

    // if not already started, auto-send a message to signal the presence of data to be decoded
    if (d2dDecodingTimer_ == NULL)
    {
        d2dDecodingTimer_ = new cMessage("d2dDecodingTimer");
        d2dDecodingTimer_->setSchedulingPriority(10);          // last thing to be performed in this TTI
        scheduleAt(NOW, d2dDecodingTimer_);
    }

    Coord myCoord = getCoord();
    // Only store frames which are within 200m over this the interference caused is negligible.
    if (myCoord.distance(lteInfo->getCoord()) < 1000) {
        // store frame, together with related control info
        frame->setControlInfo(lteInfo);

        // Capture the Airframe for decoding later
        storeAirFrame(frame);
    } else {
        delete lteInfo;
        delete frame;
    }
}

void SatPhy::handleUpperMessage(cMessage* msg)
{

    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    LteAirFrame* frame;

    if(lteInfo->getFrameType() == ASKCOORDINATEPKT)
    {
        inet::Coord myCoord = getCoord();
        Coordinate* coordinate = new Coordinate("Coordinate Information");
        coordinate->setX(myCoord.x);
        coordinate->setY(myCoord.y);
        //coordinate->setControlInfo(lteInfo);
        send(coordinate, upperGateOut_);
        delete lteInfo;
        //delete coordinate;
        return;
    }

    if (lteInfo->getFrameType() == GRANTPKT)
    {
        // Generate CSRs or save the grant use when generating SCI information
        SatmacSchedulingGrant* grant = check_and_cast<SatmacSchedulingGrant*>(msg);

        sciGrant_ = grant;
        lteInfo->setUserTxParams(sciGrant_->getUserTxParams()->dup());
        lteInfo->setGrantedBlocks(sciGrant_->getGrantedBlocks());
        lteInfo->setTotalGrantedBlocks(sciGrant_->getTotalGrantedBlocks());
        lteInfo->setDirection(D2D_MULTI);

        availableRBs_ = sendSciMessage(msg, lteInfo);

        SatmacPacket* satmacPkt = new SatmacPacket("SATMAC Message");

        m_frame_len = grant->getFrameLength();
        m_channel_num = grant->getChannelNumber();
        int globleSti = grant->getGlobleSti();
        SlotTag slotTag;
        for(int i = 0; i < m_frame_len; i++)
        {
            std::vector<SlotTagForSending> v;
            for(int j = 0; j < m_channel_num; j++)
            {
                SlotTagForSending stfs;
                stfs.busy = grant->getSlotTag()[i][j].busy;
                stfs.sti = grant->getSlotTag()[i][j].sti;
                stfs.count_2hop = grant->getSlotTag()[i][j].count_2hop;
                stfs.psf = grant->getSlotTag()[i][j].psf;
                v.push_back(stfs);
            }
            slotTag.push_back(v);
        }
        satmacPkt->setFrameLength(m_frame_len);
        satmacPkt->setChannelNumber(m_channel_num);
        satmacPkt->setGlobleSti(globleSti);
        satmacPkt->setSlotTag(slotTag);
        //satmacPkt->setHaveTbFrame(sciGrant_->getHaveTbFrame());
        UserControlInfo* SATMACInfo = lteInfo->dup();

        SATMACInfo->setFrameType(SATMACPKT);
        SATMACInfo->setGrantedBlocks(availableRBs_);
        //SATMACInfo->setTotalGrantedBlocks(sciGrant_->getTotalGrantedBlocks());
        //frame = new LteAirFrame("satmac-packet");
        beginTransmission_ = true;   // 用于模拟半双工，在解码的时候如果本节点在传输中则无法接受数据包

        frame = prepareAirFrame(satmacPkt, SATMACInfo);
        // emit(fiSent, 1);
        sendBroadcast(frame);
        delete sciGrant_;
        delete lteInfo;

        for (int i=0; i<numSubchannels_; i++)
        {
            // Mark all the subchannels as not sensed
            sensingWindow_[sensingWindowFront_][i]->setSensed(false);
        }

        return;
    }
    else if (lteInfo->getFrameType() == HARQPKT)
    {
        frame = new LteAirFrame("harqFeedback-grant");
    }

    // if this is a multicast/broadcast connection, send the frame to all neighbors in the hearing range
    // otherwise, send unicast to the destination

//    EV << "SatPhy::handleUpperMessage - " << nodeTypeToA(nodeType_) << " with id " << nodeId_
//       << " sending message to the air channel. Dest=" << lteInfo->getDestId() << endl;
//
//    // Mark that we are in the process of transmitting a packet therefore when we go to decode messages we can mark as failure due to half duplex
//    beginTransmission_ = true;
//
//    lteInfo->setGrantedBlocks(availableRBs_);
//
//    frame = prepareAirFrame(msg, lteInfo);
//
//    emit(tbSent, 1);
//
//    if (lteInfo->getDirection() == D2D_MULTI)
//        sendBroadcast(frame);
//    else
//        sendUnicast(frame);
}

RbMap SatPhy::sendSciMessage(cMessage* msg, UserControlInfo* lteInfo)
{
    RbMap rbMap = lteInfo->getGrantedBlocks();
    UserControlInfo* SCIInfo = lteInfo->dup();
    LteAirFrame* frame = NULL;

    RbMap sciRbs;
    RbMap allRbs = rbMap;
    if (adjacencyPSCCHPSSCH_)
    {
        // Adjacent mode

        // Setup so SCI gets 2 RBs from the grantedBlocks.
        RbMap::iterator it;
        std::map<Band, unsigned int>::iterator jt;
        //for each Remote unit used to transmit the packet
        int allocatedBlocks = 0;
        for (it = rbMap.begin(); it != rbMap.end(); ++it)
        {
            if (allocatedBlocks == 2)
            {
                break;
            }
            //for each logical band used to transmit the packet
            for (jt = it->second.begin(); jt != it->second.end(); ++jt)
            {
                Band band = jt->first;

                if (allocatedBlocks == 2)
                {
                    // Have all the blocks allocated to the SCI so can move on.
                    break;
                }

                if (jt->second == 0) // this Rb is not allocated
                    continue;
                else
                {
                    // RB is allocated to grant, now give it to the SCI.
                    if (jt->second == 1){
                        jt->second = 0;
                        sciRbs[it->first][band] = 1;
                        ++allocatedBlocks;
                    }
                }
            }
        }
    }
    else
    {
        // Non-Adjacent mode

        // Take 2 rbs from the inital RBs available to nodes.
        int startingRB = sciGrant_->getStartingSubchannel() * 2;

        for (Band b = startingRB; b <= startingRB + 1 ; b++)
        {
            sciRbs[MACRO][b] = 1;
            allRbs[MACRO][b] = 1;
        }
    }

    // Store the RBs used for transmission. For interference computation
    UsedRBs info;
    info.time_ = NOW;
    info.rbMap_ = allRbs;

    usedRbs_.push_back(info);

    std::vector<UsedRBs>::iterator it = usedRbs_.begin();
    while (it != usedRbs_.end())  // purge old allocations
    {
        if (it->time_ < NOW)
            usedRbs_.erase(it++);
        else
            ++it;
    }
    lastActive_ = NOW;

    SCIInfo->setFrameType(SCIPKT);
    SCIInfo->setGrantedBlocks(sciRbs);
    SCIInfo->setTotalGrantedBlocks(sciGrant_->getTotalGrantedBlocks());
    //SCIInfo->setGrantStartTime(sciGrant_->getStartTime());

    /*
     * Need to prepare the airframe were sending
     * Ensure that it will fit into it's grant
     * if not don't send anything except a break reservation message
     */
    EV << NOW << " SatPhy::handleUpperMessage - message from stack" << endl;

    // create LteAirFrame and encapsulate the received packet
    SidelinkControlInformation* SCI = createSCIMessage();
    LteAirFrame* sciFrame = prepareAirFrame(SCI, SCIInfo);

    emit(sciSent, 1);
    emit(subchannelSent, sciGrant_->getStartingSubchannel());
    emit(subchannelsUsedToSend, sciGrant_->getNumSubchannels());
    sendBroadcast(sciFrame);

//    delete sciGrant_;
//    delete lteInfo;

    return (rbMap);
}





SidelinkControlInformation* SatPhy::createSCIMessage()
{
    EV << NOW << " SatPhy::createSCIMessage - Start creating SCI..." << endl;

    SidelinkControlInformation* sci = new SidelinkControlInformation("SCI Message");

    /*
     * Priority (based on upper layer)
     * 0-7
     * Mapping unknown, so possibly just take the priority max and min and map to 0-7
     * This needs to be integrated from the application layer.
     * Will take this from the scheduling grant.
     */
    sci->setPriority(1);

    /* Resource Interval
     *
     * 0 -> 16
     * 0 = not reserved
     * 1 = 100ms (1) RRI [Default]
     * 2 = 200ms (2) RRI
     * ...
     * 10 = 1000ms (10) RRI
     * 11 = 50ms (0.5) RRI
     * 12 = 20ms (0.2) RRI
     * 13 - 15 = Reserved
     *
     */
    if (sciGrant_->getExpiration() != 0)
    {
        if (sciGrant_->getPeriod() == 50){
            sci->setResourceReservationInterval(11);
        } else if (sciGrant_->getPeriod() == 20) {
            sci->setResourceReservationInterval(12);
        } else {
            sci->setResourceReservationInterval(sciGrant_->getPeriod() / 100);
        }
    }
    else
    {
        sci->setResourceReservationInterval(0);
    }
    /* frequency Resource Location
     * Based on another parameter RIV
     * but size is
     * Log2(Nsubchannel(Nsubchannel+1)/2) (rounded up)
     * 0 - 8 bits
     * 0 - 256 different values
     *
     * Based on TS36.213 14.1.1.4C
     *     if SubchannelLength -1 < numSubchannels/2
     *         RIV = numSubchannels(SubchannelLength-1) + subchannelIndex
     *     else
     *         RIV = numSubchannels(numSubchannels-SubchannelLength+1) + (numSubchannels-1-subchannelIndex)
     */
    unsigned int riv;
    //
    if (sciGrant_->getNumSubchannels() -1 <= (numSubchannels_/2))
    {
        // RIV calculation for less than half+1
        riv = ((numSubchannels_ * (sciGrant_->getNumSubchannels() - 1)) + sciGrant_->getStartingSubchannel());
    }
    else
    {
        // RIV calculation for more than half size
        riv = ((numSubchannels_ * (numSubchannels_ - sciGrant_->getNumSubchannels() + 1)) + (numSubchannels_ - 1 - sciGrant_->getStartingSubchannel()));
    }

    sci->setFrequencyResourceLocation(riv);

    /* TimeGapRetrans
     * 1 - 15
     * ms from init to retrans
     */
    sci->setTimeGapRetrans(sciGrant_->getTimeGapTransRetrans());


    /* mcs
     * 5 bits
     * 26 combos
     * Technically the MAC layer determines the MCS that it wants the message sent with and as such it will be in the packet
     */
    sci->setMcs(sciGrant_->getMcs());

    /* retransmissionIndex
     * 0 / 1
     * if 0 retrans is in future/this is the first transmission
     * if 1 retrans is in the past/this is the retrans
     */
    if (sciGrant_->getRetransmission())
    {
        sci->setRetransmissionIndex(1);
    }
    else
    {
        sci->setRetransmissionIndex(0);
    }

    /* Filler up to 32 bits
     * Can just set the length to 32 bits and ignore the issue of adding filler
     */
    sci->setBitLength(32);

    return sci;
}

LteAirFrame* SatPhy::prepareAirFrame(cMessage* msg, UserControlInfo* lteInfo){
    // Helper function to prepare airframe for sending.
    LteAirFrame* frame = new LteAirFrame("airframe");

    frame->encapsulate(check_and_cast<cPacket*>(msg));
    frame->setSchedulingPriority(airFramePriority_);
    frame->setDuration(TTI);

    lteInfo->setCoord(getRadioPosition());

    lteInfo->setTxPower(txPower_);
    lteInfo->setD2dTxPower(d2dTxPower_);
    frame->setControlInfo(lteInfo);

    return frame;
}

void SatPhy::storeAirFrame(LteAirFrame* newFrame)
{
    // implements the capture effect
    // store the frame received from the nearest transmitter
    UserControlInfo* newInfo = check_and_cast<UserControlInfo*>(newFrame->getControlInfo());
    Coord myCoord = getCoord();

    std::tuple<std::vector<double>, double> rsrpAttenuation = channelModel_->getRSRP_D2D(newFrame, newInfo, nodeId_, myCoord);
    std::vector<double> rsrpVector = get<0>(rsrpAttenuation);
    double attenuation = get<1>(rsrpAttenuation);

    // Seems we don't really actually need the enbId, I have set it to 0 as it is referenced but never used for calc
    std::tuple<std::vector<double>, std::vector<double>> rssiSinrVectors = channelModel_->getRSSI_SINR(newFrame, newInfo, nodeId_, myCoord, 0, rsrpVector);

    std::vector<double> rssiVector = get<0>(rssiSinrVectors);
    std::vector<double> sinrVector = get<1>(rssiSinrVectors);

    int countAssignedRbs = 0;
    double avgSinr = 0.0;
    RbMap grantedBlocks = newInfo->getGrantedBlocks();

    RbMap::iterator it;
    std::map<Band, unsigned int>::iterator jt;
    //for each Remote unit used to transmit the packet
    for (it = grantedBlocks.begin(); it != grantedBlocks.end(); ++it) {
        //for each logical band used to transmit the packet
        for (jt = it->second.begin(); jt != it->second.end(); ++jt) {
            //this Rb is not allocated
            if (jt->second == 0)
                continue;
            avgSinr += sinrVector[jt->first];
            countAssignedRbs++;
        }
    }

    avgSinr = avgSinr / countAssignedRbs;

    // Need to be able to figure out which subchannel is associated to the Rbs in this case
    if (newInfo->getFrameType() == SCIPKT){
        sciInfo_.push_back(std::make_tuple(newFrame, rsrpVector, rssiVector, sinrVector, attenuation, avgSinr));
    }  else{
        tbInfo_.push_back(std::make_tuple(newFrame, rsrpVector, rssiVector, sinrVector, attenuation, avgSinr));
    }
}

void SatPhy::decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo, std::vector<double> &rsrpVector, std::vector<double> &rssiVector, std::vector<double> &sinrVector, double &attenuation)
{
    EV << NOW << " SatPhy::decodeAirFrame - Start decoding..." << endl;

    // apply decider to received packet
    bool interference_result = false;
    bool prop_result = false;

    cPacket* pkt = frame->decapsulate();

    if(lteInfo->getFrameType() == SCIPKT)
    {
        double pkt_dist = getCoord().distance(lteInfo->getCoord());
        emit(txRxDistanceSCI, pkt_dist);

        SidelinkControlInformation *sci = check_and_cast<SidelinkControlInformation *>(pkt);
        std::tuple<int, int> indexAndLength = decodeRivValue(sci, lteInfo);
        int subchannelIndex = std::get<0>(indexAndLength);
        int lengthInSubchannels = std::get<1>(indexAndLength);

        subchannelReceived_ = subchannelIndex;
        subchannelsUsed_ = lengthInSubchannels;
        emit(senderID, lteInfo->getSourceId());

        if (!transmitting_)
        {

            sciReceived_ += 1;

            bool notSensed = false;
            double erfParam = (lteInfo->getD2dTxPower() - attenuation - -90.5) / (3 * sqrt(2));
            double erfValue = erf(erfParam);
            double packetSensingRatio = 0.5 * (1 + erfValue);
            double er = dblrand(1);

            if (er >= packetSensingRatio){
                // Packet was not sensed so mark as such and delete it.
                lteInfo->setDeciderResult(false);
                sciUnsensed_ += 1;
            } else {
                std::tuple<bool, bool> res = channelModel_->error_Mode4(frame, lteInfo, rsrpVector, sinrVector, 0);
                prop_result = get<0>(res);
                interference_result = get<1>(res);

                RbMap::iterator mt;
                std::map<Band, unsigned int>::iterator nt;
                RbMap usedRbs = lteInfo->getGrantedBlocks();
                std::vector < Subchannel * > currentSubframe = sensingWindow_[sensingWindowFront_];
                Subchannel *currentSubchannel = currentSubframe[subchannelIndex];
                std::vector<Band>::iterator lt;
                std::vector <Band> allocatedBands = currentSubchannel->getOccupiedBands();
                for (lt = allocatedBands.begin(); lt != allocatedBands.end(); lt++) {
                    // Record RSRP and RSSI for this band depending if it was used or not
                    bool used = false;

                    //for each Remote unit used to transmit the packet
                    for (mt = usedRbs.begin(); mt != usedRbs.end(); ++mt) {
                        //for each logical band used to transmit the packet
                        for (nt = mt->second.begin(); nt != mt->second.end(); ++nt) {
                            if (nt->first == *lt) {
                                currentSubchannel->addRsrpValue(rsrpVector[(*lt)], (*lt));
                                currentSubchannel->addRssiValue(rssiVector[(*lt)], (*lt));
                                used = true;
                                break;
                            }
                        }
                    }
                }

                // Need to ensure that we haven't previously decoded a higher SINR packet.
                if (interference_result & !sensingWindow_[sensingWindowFront_][subchannelIndex]->getReserved()) {
                    for (int i = subchannelIndex; i < subchannelIndex + lengthInSubchannels; i++) {
                        Subchannel *currentSubchannel = currentSubframe[i];
                        // Record the SCI info in the subchannel.
                        currentSubchannel->setPriority(sci->getPriority());
                        currentSubchannel->setResourceReservationInterval(sci->getResourceReservationInterval());
                        currentSubchannel->setFrequencyResourceLocation(sci->getFrequencyResourceLocation());
                        currentSubchannel->setTimeGapRetrans(sci->getTimeGapRetrans());
                        currentSubchannel->setMcs(sci->getMcs());
                        currentSubchannel->setRetransmissionIndex(sci->getRetransmissionIndex());
                        currentSubchannel->setSciSubchannelIndex(subchannelIndex);
                        currentSubchannel->setSciLength(lengthInSubchannels);
                        currentSubchannel->setReserved(true);
                    }
                    lteInfo->setDeciderResult(true);
                    sciDecoded_ += 1;
                } else if (!prop_result) {
                    lteInfo->setDeciderResult(false);
                    sciFailedDueToProp_ += 1;
                } else {
                    lteInfo->setDeciderResult(false);
                    // Just ensure the result is recorded as false
                    interference_result = false;
                    sciFailedDueToInterference_ += 1;
                }
            }
            pkt->setControlInfo(lteInfo);
            scis_.push_back(pkt);
        }
        else
        {
            sciFailedHalfDuplex_ += 1;
            delete lteInfo;
            delete pkt;
        }
        delete frame;
    }
    else
    {
        double pkt_dist = getCoord().distance(lteInfo->getCoord());
        emit(txRxDistanceTB, pkt_dist);

        if(!transmitting_){

            tbReceived_ += 1;

            // Have a TB want to make sure we have the SCI for it.
            bool foundCorrespondingSci = false;
            bool sciDecodedSuccessfully = false;
            SidelinkControlInformation *correspondingSCI;
            UserControlInfo *sciInfo;
            std::vector<cPacket *>::iterator it;
            for (it = scis_.begin(); it != scis_.end(); it++) {
                sciInfo = check_and_cast<UserControlInfo*>((*it)->removeControlInfo());
                // if the SCI and TB have same source then we have the right SCI
                if (sciInfo->getSourceId() == lteInfo->getSourceId()) {
                    //Successfully received the SCI
                    foundCorrespondingSci = true;

                    correspondingSCI = check_and_cast<SidelinkControlInformation*>(*it);

                    if (sciInfo->getDeciderResult()) {
                        sciDecodedSuccessfully = true;
                    }

                    if (lteInfo->getDirection() == D2D_MULTI) {
                        std::tuple<bool, bool> res = channelModel_->error_Mode4(frame, lteInfo, rsrpVector, sinrVector, correspondingSCI->getMcs());
                        prop_result = get<0>(res);
                        interference_result = get<1>(res);
                    }

                    // Remove the SCI
                    scis_.erase(it);
                    break;
                } else {
                    (*it)->setControlInfo(sciInfo);
                }
            }
            if (!foundCorrespondingSci || !sciDecodedSuccessfully) {
                tbFailedDueToNoSCI_ += 1;
                if (!prop_result) {
                    tbFailedDueToPropIgnoreSCI_ += 1;
                } else if (!interference_result) {
                    tbFailedDueToInterferenceIgnoreSCI_ += 1;
                } else {
                    tbDecodedIgnoreSCI_ += 1;
                }

            } else if (!prop_result) {
                tbFailedButSCIReceived_ += 1;
                tbFailedDueToProp_ += 1;
                tbFailedDueToPropIgnoreSCI_ += 1;
            } else if (!interference_result) {
                tbFailedButSCIReceived_ += 1;
                tbFailedDueToInterference_ += 1;
                tbFailedDueToInterferenceIgnoreSCI_ += 1;
            } else {
                tbDecoded_ += 1;
                tbDecodedIgnoreSCI_ += 1;
                std::map<MacNodeId, simtime_t>::iterator jt = previousTransmissionTimes_.find(lteInfo->getSourceId());
                if ( jt != previousTransmissionTimes_.end() ) {
                    simtime_t elapsed_time = NOW - jt->second;
                    emit(interPacketDelay, elapsed_time);
                }
                previousTransmissionTimes_[lteInfo->getSourceId()] = NOW;
            }
            if (foundCorrespondingSci) {
                // Need to get the map only for the RBs used for transmission
                RbMap::iterator mt;
                std::map<Band, unsigned int>::iterator nt;
                RbMap usedRbs = lteInfo->getGrantedBlocks();

                // Now need to find the associated Subchannels, record the RSRP and RSSI for the message and go from there.
                // Need to again do the RIV steps
                std::tuple<int, int> indexAndLength = decodeRivValue(correspondingSCI, sciInfo);
                int subchannelIndex = std::get<0>(indexAndLength);
                int lengthInSubchannels = std::get<1>(indexAndLength);

                std::vector <Subchannel *> currentSubframe = sensingWindow_[sensingWindowFront_];
                for (int i = subchannelIndex; i < subchannelIndex + lengthInSubchannels; i++) {
                    Subchannel *currentSubchannel = currentSubframe[i];
                    std::vector<Band>::iterator lt;
                    std::vector <Band> allocatedBands = currentSubchannel->getOccupiedBands();
                    for (lt = allocatedBands.begin(); lt != allocatedBands.end(); lt++) {
                        // Record RSRP and RSSI for this band depending if it was used or not
                        bool used = false;

                        //for each Remote unit used to transmit the packet
                        for (mt = usedRbs.begin(); mt != usedRbs.end(); ++mt) {
                            //for each logical band used to transmit the packet
                            for (nt = mt->second.begin(); nt != mt->second.end(); ++nt) {
                                if (nt->first == *lt) {
                                    currentSubchannel->addRsrpValue(rsrpVector[(*lt)], (*lt));
                                    currentSubchannel->addRssiValue(rssiVector[(*lt)], (*lt));
                                    used = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                // Need to delete the message now
                delete correspondingSCI;
                delete sciInfo;
            }
        }
        else{
            tbFailedHalfDuplex_ += 1;
        }

        delete frame;

        // send decapsulated message along with result control info to upperGateOut_
        lteInfo->setDeciderResult(interference_result);
        if(!interference_result)
            emit(collisionCount,1);
        pkt->setControlInfo(lteInfo);
        send(pkt, upperGateOut_);

        if (getEnvir()->isGUI())
            updateDisplayString();
    }

    // update statistics
    if (interference_result)
    {
        numAirFrameReceived_++;
    }
    else
    {
        numAirFrameNotReceived_++;
    }

    EV << "Handled LteAirframe with ID " << frame->getId() << " with result "
       << (interference_result ? "RECEIVED" : "NOT RECEIVED") << endl;
}

std::tuple<int,int> SatPhy::decodeRivValue(SidelinkControlInformation* sci, UserControlInfo* sciInfo)
{
    EV << NOW << " SatPhy::decodeRivValue - Decoding RIV value of SCI allows correct placement in sensing window..." << endl;
    //UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(pkt->removeControlInfo());
    RbMap rbMap = sciInfo->getGrantedBlocks();
    RbMap::iterator it;
    std::map<Band, unsigned int>::iterator jt;
    Band startingBand;
    bool bandNotFound = true;

    it = rbMap.begin();

    while (it != rbMap.end() && bandNotFound )
    {
        for (jt = it->second.begin(); jt != it->second.end(); ++jt)
        {
            Band band = jt->first;
            if (jt->second == 1) // this Rb is not allocated
            {
                startingBand = band;
                bandNotFound= false;
                break;
            }
        }
    }

    // Get RIV first as this is common
    unsigned int RIV = sci->getFrequencyResourceLocation();

    // Get the subchannel Index (allows us later to calculate the length of the message
    int subchannelIndex;
    if (adjacencyPSCCHPSSCH_)
    {
        // If adjacent: Subchannel Index = band//subchannelSize
        subchannelIndex = startingBand/subchannelSize_;
    }
    else
    {
        // If non-adjacent: Subchannel Index = band/2
        subchannelIndex = startingBand/2;
    }

    // Based on TS36.213 14.1.1.4C
    // if SubchannelLength -1 < numSubchannels/2
    // RIV = numSubchannels(SubchannelLength-1) + subchannelIndex
    // else
    // RIV = numSubchannels(numSubchannels-SubchannelLength+1) + (numSubchannels-1-subchannelIndex)
    // RIV limit
    // log2(numSubchannels(numSubchannels+1)/2)
    double subchannelLOverHalf = (numSubchannels_ + 2 + (( -1 - subchannelIndex - RIV )/numSubchannels_));
    double subchannelLUnderHalf = (RIV + numSubchannels_ - subchannelIndex)/numSubchannels_;
    int lengthInSubchannels;

    // First the number has to be whole in both cases, it's length + subchannelIndex must be less than the number of subchannels
    if (floor(subchannelLOverHalf) == subchannelLOverHalf && subchannelLOverHalf <= numSubchannels_ && subchannelLOverHalf + subchannelIndex <= numSubchannels_)
    {
        lengthInSubchannels = subchannelLOverHalf;
    }
    // Same as above but also the length must be less than half + 1
    else if (floor(subchannelLUnderHalf) == subchannelLUnderHalf && subchannelLUnderHalf + subchannelIndex <= numSubchannels_ && subchannelLUnderHalf <= numSubchannels_ /2 + 1)
    {
        lengthInSubchannels = subchannelLUnderHalf;
    }
    return std::make_tuple(subchannelIndex, lengthInSubchannels);
}


void SatPhy::updateSubframe()
{
    // Ensure for every 1ms we record the position of the vehicle
    emit(posX, getCoord().x);
    emit(posY, getCoord().y);

    int sensingWindowLength = pStep_ * 10;
    if (sensingWindowSizeOverride_ > 0){
        sensingWindowLength = sensingWindowSizeOverride_;
    }

    // Increment the pointer to the next element in the sensingWindow
    if (sensingWindowFront_ < sensingWindowLength - 1) {
        ++sensingWindowFront_;
    }
    else{
        // Front has gone over the end of the sensing window reset it.
        sensingWindowFront_ = 0;
    }


    // First find the subframe that we want to look at i.e. the front one I imagine
    // If the front isn't occupied then skip on
    // If it is occupied, pop it off, update it and push it back
    // All good then.

    std::vector<Subchannel*> subframe = sensingWindow_[sensingWindowFront_];

    if (subframe.at(0)->getSubframeTime() <= NOW - SimTime(sensingWindowLength, SIMTIME_MS) - TTI)
    {
        std::vector<Subchannel*>::iterator it;
        for (it=subframe.begin(); it!=subframe.end(); it++)
        {
            (*it)->reset(NOW - TTI);
        }
    }

    cMessage* updateSubframe = new cMessage("updateSubframe");
    updateSubframe->setSchedulingPriority(0);        // Generate the subframe at start of next TTI
    scheduleAt(NOW + TTI, updateSubframe);
}

void SatPhy::initialiseSensingWindow()
{
    EV << NOW << " SatPhy::initialiseSensingWindow - creating subframes to be added to sensingWindow..." << endl;

    simtime_t subframeTime = NOW - TTI;

    int sensingWindowLength = pStep_ * 10;
    if (sensingWindowSizeOverride_ > 0){
        sensingWindowLength = sensingWindowSizeOverride_;
    }

    // Reserve the full size of the sensing window (might improve efficiency).
    sensingWindow_.reserve(sensingWindowLength);

    while(sensingWindow_.size() < sensingWindowLength)
    {
        std::vector<Subchannel*> subframe;
        subframe.reserve(numSubchannels_);
        Band band = 0;

        if (!adjacencyPSCCHPSSCH_)
        {
            // This assumes the bands only every have 1 Rb (which is fine as that appears to be the case)
            band = numSubchannels_*2;
        }
        for (int i = 0; i < numSubchannels_; i++) {
            Subchannel *currentSubchannel = new Subchannel(subchannelSize_, subframeTime);
            // Need to determine the RSRP and RSSI that corresponds to background noise
            // Best off implementing this in the channel model as a method.

            std::vector <Band> occupiedBands;

            int overallCapacity = 0;
            // Ensure the subchannel is allocated the correct number of RBs
            while (overallCapacity < subchannelSize_ && band < getBinder()->getNumBands()) {
                // This acts like there are multiple RBs per band which is not allowed.
                occupiedBands.push_back(band);
                ++overallCapacity;
                ++band;
            }
            currentSubchannel->setOccupiedBands(occupiedBands);
            subframe.push_back(currentSubchannel);
        }
        sensingWindow_.push_back(subframe);
        subframeTime += TTI;
    }
    // Send self message to trigger another subframes creation and insertion. Need one for every TTI
    cMessage* updateSubframe = new cMessage("updateSubframe");
    updateSubframe->setSchedulingPriority(0);        // Generate the subframe at start of next TTI
    scheduleAt(NOW + TTI, updateSubframe);
}

int SatPhy::translateIndex(int fallBack) {
    if (fallBack > sensingWindowFront_){
        int max = 10 * pStep_;
        if (sensingWindowSizeOverride_ > 0){
            max = sensingWindowSizeOverride_;
        }
        int fromMax = fallBack - sensingWindowFront_;
        return max - fromMax;
    } else{
        return sensingWindowFront_ - fallBack;
    }
}

void SatPhy::finish()
{
    if (getSimulation()->getSimulationStage() != CTX_FINISH)
    {
        // do this only at deletion of the module during the simulation
        //LtePhyUe::finish();
        LteAmc *amc = getAmcModule(masterId_);
        if (amc != NULL)
        {
            amc->detachUser(nodeId_, UL);
            amc->detachUser(nodeId_, DL);
            amc->detachUser(nodeId_, D2D);
        }

        // binder call
        binder_->unregisterNextHop(masterId_, nodeId_);

        // deployer call
        deployer_->detachUser(nodeId_);
    }

    std::vector<std::vector<Subchannel *>>::iterator it;
    for (it=sensingWindow_.begin();it!=sensingWindow_.end();it++)
    {
        std::vector<Subchannel *>::iterator jt;
        for (jt=it->begin();jt!=it->end();jt++)
        {
            delete (*jt);
        }
    }
    sensingWindow_.clear();
}
