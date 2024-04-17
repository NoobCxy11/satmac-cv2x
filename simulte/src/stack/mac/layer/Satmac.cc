//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

/**
 * LteMacVUeMode4 is a new model which implements the functionality of LTE Mode 4 as per 3GPP release 14
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#include "stack/mac/buffer/harq/LteHarqBufferRx.h"
#include "stack/mac/buffer/LteMacQueue.h"
#include "stack/mac/buffer/harq_d2d/LteHarqBufferRxD2DMirror.h"
#include "stack/mac/scheduler/LteSchedulerUeUl.h"
#include "stack/phy/packet/SpsCandidateResources.h"
#include "stack/phy/packet/cbr_m.h"
#include "stack/phy/layer/Subchannel.h"
#include "Satmac.h"
#include "stack/mac/amc/AmcPilotD2D.h"
#include "common/LteCommon.h"
#include "stack/phy/layer/LtePhyBase.h"
#include "inet/networklayer/common/InterfaceEntry.h"
#include "inet/common/ModuleAccess.h"
#include "inet/networklayer/ipv4/IPv4InterfaceData.h"
#include "stack/mac/amc/LteMcs.h"
#include <map>

Define_Module(Satmac);

Satmac::Satmac() :
    LteMacUeRealisticD2D()
{
}

Satmac::~Satmac()
{
}

void Satmac::initialize(int stage)
{
    if (stage !=inet::INITSTAGE_NETWORK_LAYER_3)
    LteMacUeRealisticD2D::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL)
    {
        parseUeTxConfig(par("txConfig").xmlValue());
        //parseCbrTxConfig(par("txConfig").xmlValue());
        //parseRriConfig(par("txConfig").xmlValue());
        //resourceReservationInterval_ = validResourceReservationIntervals_.at(0);
        subchannelSize_ = par("subchannelSize");
        numSubchannels_ = par("numSubchannels");
        //probResourceKeep_ = par("probResourceKeep");
        usePreconfiguredTxParams_ = par("usePreconfiguredTxParams");
        reselectAfter_ = par("reselectAfter");
        //useCBR_ = par("useCBR");
        packetDropping_ = par("packetDropping");
        //adjacencyPSCCHPSSCH_ = par("adjacencyPSCCHPSSCH");
        //randomScheduling_ = par("randomScheduling");
        maximumCapacity_ = 0;
        //cbr_=0;
        currentCw_=0;
        missedTransmissions_=0;

        expiredGrant_ = false;

        currentCbrIndex_ = defaultCbrIndex_;

        // Satmac initialize
        int64_t temp_slotTime = par("SlotTime");
        m_slotTime = SimTime(temp_slotTime,SIMTIME_US);
        m_frame_len = par("FrameLen");
        slot_lifetime_frame_ = par("SlotLife");
        c3hop_threshold_ = par("C3HThreshold");
        adj_free_threshold_ = par("AdjThreshold");
        random_bch_if_single_switch_ = par("RandomBchIfSingle");
        choose_bch_random_switch_ = par("ChooseBchRandomSwitch");
        //vemac_mode_ = par("VemacMode");
        adj_ena_ = par("AdjEnable");
        adj_ena_ = 1;
        adj_frame_ena_ = par("AdjFrameEnable");
        adj_frame_ena_ = 0;
        adj_frame_lower_bound_ = par("AdjFrameLowerBound");
        adj_frame_upper_bound_ = par("AdjFrameUpperBound");
        slot_memory_ = par("SlotMemory");
        frameadj_exp_ratio_ = par("FrameadjExpRatio");
        frameadj_cut_ratio_ths_ = par("FrameadjCutRatioThs");
        frameadj_cut_ratio_ehs_ = par("FrameadjCutRatioEhs");
        global_sti = nodeId_;
        //NS_LOG_FUNCTION (this); // 需要改
        //m_traceOutFile = "lpf-output.txt";
        m_wifimaclow_flag = 0;
        geoHash_NQueens = true;
        m_queue = new LteMacQueue(queueSize_);
        channel_num_ = -1;
        if(geoHash_NQueens)
            m_channel_num = 3;
        else
            m_channel_num = 1;
        //m_channel_num = 3;
        //m_channel_num = 8;
        //m_channel_num = 1;
        //pduMakeCounter = 100;
        adj_single_slot_ena_ = 0;
        bch_slot_lock_ = 5;
        adj_ena_ = 1;
        adj_free_threshold_ = 5;
        //adj_frame_ena_ = 1;
        adj_frame_lower_bound_  = 32;
        adj_frame_upper_bound_ = 128;
        slot_memory_ = 1;
        frameadj_cut_ratio_ths_ = 0.4;
        frameadj_cut_ratio_ehs_ = 0.6;
        frameadj_exp_ratio_ = 0.9;
        testmode_init_flag_ = 0;
        random_bch_if_single_switch_ = 0;
        choose_bch_random_switch_ = 1;
        slot_adj_candidate_ = -1;
        channelNumber_adj = -1;
        vemac_mode_ = 0;

        // frameadj_cut_ratio_ths_ = 0.4;
        // frameadj_cut_ratio_ehs_ = 0.6;
        // frameadj_exp_ratio_ = 0.9;
        dataPktExist = false;
        total_slot_count_ = 0;
        slot_count_ = 0;
        slot_num_ = (slot_count_+1)% m_frame_len; //slot_num_初始化为当前的下一个时隙。

        global_psf = 0;
        collected_fi_ = new Frame_info(512, 1);  //帧长=512
        collected_fi_->sti = nodeId_;
        received_fi_list_= NULL;

        node_state_ = NODE_INIT;
        slot_state_ = BEGINING;
        collision_count_ = 0;
        localmerge_collision_count_ = 0;
        request_fail_times = 0;
        waiting_frame_count = 0;
        frame_count_ = 0;
        this->enable=0;
        this->packet_sended = 0;
        this->packet_received =0;

        recv_fi_count_ = 0;
        send_fi_count_ = 0;

        mbr::MbrSumo *map = mbr::MbrSumo::GetInstance();
        if (!map->isInitialized())
        {
            //map->Initialize("/home/cxy/latest_simulte/simulte/simulations/Mode4/highway/006vpm/highway.net.xml", "");
            map->Initialize("/home/cxy/latest_simulte/simulte/simulations/Mode4/highway/006vpm/highway.net.xml", "");
            map->setInitialized(true);
        }

        // Register the necessary signals for this simulation

        accessCollision         = registerSignal("accessCollision");
        grantStartTime          = registerSignal("grantStartTime");
        //takingReservedGrant     = registerSignal("takingReservedGrant");
        grantBreak              = registerSignal("grantBreak");
        grantBreakTiming        = registerSignal("grantBreakTiming");
        grantBreakSize          = registerSignal("grantBreakSize");
        droppedTimeout          = registerSignal("droppedTimeout");
        grantBreakMissedTrans   = registerSignal("grantBreakMissedTrans");
        missedTransmission      = registerSignal("missedTransmission");
        selectedMCS             = registerSignal("selectedMCS");
        selectedSubchannelIndex = registerSignal("selectedSubchannelIndex");
        selectedNumSubchannels  = registerSignal("selectedNumSubchannels");
        maximumCapacity         = registerSignal("maximumCapacity");
        grantRequests           = registerSignal("grantRequests");
        packetDropDCC           = registerSignal("packetDropDCC");
        macNodeID               = registerSignal("macNodeID");
        //rrcSelected             = registerSignal("resourceReselectionCounter");
        //retainGrant             = registerSignal("retainGrant");

        // Satmac Register
        macRcvdSatmacPkt        = registerSignal("macRcvdSatmacPkt");
        macpktSent              = registerSignal("macpktSent");
    }
    else if (stage == inet::INITSTAGE_NETWORK_LAYER_3)
    {
        deployer_ = getDeployer();
        numAntennas_ = getNumAntennas();
        mcsScaleD2D_ = deployer_->getMcsScaleUl();
        d2dMcsTable_.rescale(mcsScaleD2D_);

        if (usePreconfiguredTxParams_)
        {
            preconfiguredTxParams_ = getPreconfiguredTxParams();
        }

        // LTE UE Section
        nodeId_ = getAncestorPar("macNodeId");

        emit(macNodeID, nodeId_);

        /* Insert UeInfo in the Binder */
        ueInfo_ = new UeInfo();
        ueInfo_->id = nodeId_;            // local mac ID
        ueInfo_->cellId = cellId_;        // cell ID
        ueInfo_->init = false;            // flag for phy initialization
        ueInfo_->ue = this->getParentModule()->getParentModule();  // reference to the UE module

        // Get the Physical Channel reference of the node
        ueInfo_->phy = check_and_cast<LtePhyBase*>(ueInfo_->ue->getSubmodule("lteNic")->getSubmodule("phy"));

        binder_->addUeInfo(ueInfo_);
    }
}

void Satmac::parseUeTxConfig(cXMLElement* xmlConfig)
{
    if (xmlConfig == 0)
    throw cRuntimeError("No sidelink configuration file specified");

    // Get channel Model field which contains parameters fields
    cXMLElementList ueTxConfig = xmlConfig->getElementsByTagName("userEquipment-txParameters");

    if (ueTxConfig.empty())
        throw cRuntimeError("No userEquipment-txParameters configuration found in configuration file");

    if (ueTxConfig.size() > 1)
        throw cRuntimeError("More than one userEquipment-txParameters configuration found in configuration file.");

    cXMLElement* ueTxConfigData = ueTxConfig.front();

    ParameterMap params;
    getParametersFromXML(ueTxConfigData, params);

    //get lambda max threshold
    ParameterMap::iterator it = params.find("minMCS-PSSCH");
    if (it != params.end())
    {
        minMCSPSSCH_ = (int)it->second;
    }
    else
        minMCSPSSCH_ = (int)par("minMCSPSSCH");
    it = params.find("maxMCS-PSSCH");
    if (it != params.end())
    {
        maxMCSPSSCH_ = (int)it->second;
    }
    else
        maxMCSPSSCH_ = (int)par("minMCSPSSCH");
    it = params.find("minSubchannel-NumberPSSCH");
    if (it != params.end())
    {
        minSubchannelNumberPSSCH_ = (int)it->second;
    }
    else
        minSubchannelNumberPSSCH_ = (int)par("minSubchannelNumberPSSCH");
    it = params.find("maxSubchannel-NumberPSSCH");
    if (it != params.end())
    {
        maxSubchannelNumberPSSCH_ = (int)it->second;
    }
    else
        maxSubchannelNumberPSSCH_ = (int)par("maxSubchannelNumberPSSCH");
    it = params.find("allowedRetxNumberPSSCH");
    if (it != params.end())
    {
        allowedRetxNumberPSSCH_ = (int)it->second;
    }
    else
        allowedRetxNumberPSSCH_ = (int)par("allowedRetxNumberPSSCH");
}


int Satmac::getNumAntennas()
{
    /* Get number of antennas: +1 is for MACRO */
    return deployer_->getNumRus() + 1;
}

void Satmac::macPduMake()
{
    int64 size = 0;

    macPduList_.clear();

    // In a D2D communication if BSR was created above this part isn't executed
    // Build a MAC PDU for each scheduled user on each codeword
    LteMacScheduleList::const_iterator it;
    for (it = scheduleList_->begin(); it != scheduleList_->end(); it++)
    {
        LteMacPdu* macPkt;
        cPacket* pkt;

        MacCid destCid = it->first.first;
        Codeword cw = it->first.second;

        // get the direction (UL/D2D/D2D_MULTI) and the corresponding destination ID
        LteControlInfo* lteInfo = &(connDesc_.at(destCid));
        MacNodeId destId = lteInfo->getDestId();
        Direction dir = (Direction)lteInfo->getDirection();

        std::pair<MacNodeId, Codeword> pktId = std::pair<MacNodeId, Codeword>(destId, cw);
        unsigned int sduPerCid = it->second;

        MacPduList::iterator pit = macPduList_.find(pktId);

        if (sduPerCid == 0)
        {
            continue;
        }

        // No packets for this user on this codeword
        if (pit == macPduList_.end())
        {
            // Always goes here because of the macPduList_.clear() at the beginning
            // Build the Control Element of the MAC PDU
            UserControlInfo* uinfo = new UserControlInfo();
            uinfo->setSourceId(getMacNodeId());
            uinfo->setDestId(destId);
            uinfo->setLcid(MacCidToLcid(destCid));
            uinfo->setDirection(dir);
            uinfo->setLcid(MacCidToLcid(SHORT_BSR));

            // First translate MCS to CQI
            SatmacSchedulingGrant* satmacGrant = check_and_cast<SatmacSchedulingGrant*>(schedulingGrant_);

            if (usePreconfiguredTxParams_)
            {
                //UserTxParams* userTxParams = preconfiguredTxParams_;
                uinfo->setUserTxParams(preconfiguredTxParams_->dup());
                satmacGrant->setUserTxParams(preconfiguredTxParams_->dup());
            }
            else
                uinfo->setUserTxParams(satmacGrant->getUserTxParams()->dup());

            // Create a PDU
            macPkt = new LteMacPdu("LteMacPdu");
            macPkt->setHeaderLength(MAC_HEADER);
            macPkt->setControlInfo(uinfo);
            macPkt->setTimestamp(NOW);
            macPduList_[pktId] = macPkt;
        }
        else
        {
            // Never goes here because of the macPduList_.clear() at the beginning
            macPkt = pit->second;
        }

        while (sduPerCid > 0)
        {
            // Add SDU to PDU
            // Find Mac Pkt
            if (mbuf_.find(destCid) == mbuf_.end())
                throw cRuntimeError("Unable to find mac buffer for cid %d", destCid);

            if (mbuf_[destCid]->empty())
                throw cRuntimeError("Empty buffer for cid %d, while expected SDUs were %d", destCid, sduPerCid);

            pkt = mbuf_[destCid]->popFront();

            // multicast support
            // this trick gets the group ID from the MAC SDU and sets it in the MAC PDU
            int32 groupId = check_and_cast<LteControlInfo*>(pkt->getControlInfo())->getMulticastGroupId();
            if (groupId >= 0) // for unicast, group id is -1
                check_and_cast<LteControlInfo*>(macPkt->getControlInfo())->setMulticastGroupId(groupId);

            drop(pkt);

            macPkt->pushSdu(pkt);
            sduPerCid--;
        }

        // consider virtual buffers to compute BSR size
        size += macBuffers_[destCid]->getQueueOccupancy();

        if (size > 0)
        {
            // take into account the RLC header size
            if (connDesc_[destCid].getRlcType() == UM)
                size += RLC_HEADER_UM;
            else if (connDesc_[destCid].getRlcType() == AM)
                size += RLC_HEADER_AM;
        }
    }

    // Put MAC PDUs in H-ARQ buffers
    MacPduList::iterator pit;
    for (pit = macPduList_.begin(); pit != macPduList_.end(); pit++)
    {
        MacNodeId destId = pit->first.first;
        Codeword cw = pit->first.second;
        // Check if the HarqTx buffer already exists for the destId
        // Get a reference for the destId TXBuffer
        LteHarqBufferTx* txBuf;
        HarqTxBuffers::iterator hit = harqTxBuffers_.find(destId);
        if ( hit != harqTxBuffers_.end() )
        {
            // The tx buffer already exists
            txBuf = hit->second;
        }
        else
        {
            // The tx buffer does not exist yet for this mac node id, create one
            LteHarqBufferTx* hb;
            // FIXME: hb is never deleted
            UserControlInfo* info = check_and_cast<UserControlInfo*>(pit->second->getControlInfo());
            if (info->getDirection() == UL)
                hb = new LteHarqBufferTx((unsigned int) ENB_TX_HARQ_PROCESSES, this, (LteMacBase*) getMacByMacNodeId(destId));
            else // D2D or D2D_MULTI
                hb = new LteHarqBufferTxD2D((unsigned int) ENB_TX_HARQ_PROCESSES, this, (LteMacBase*) getMacByMacNodeId(destId));
            harqTxBuffers_[destId] = hb;
            txBuf = hb;
        }

        // search for an empty unit within current harq process
        UnitList txList = txBuf->getEmptyUnits(currentHarq_);
        EV << "LteMacUeRealisticD2D::macPduMake - [Used Acid=" << (unsigned int)txList.first << "] , [curr=" << (unsigned int)currentHarq_ << "]" << endl;

        //Get a reference of the LteMacPdu from pit pointer (extract Pdu from the MAP)
        LteMacPdu* macPkt = pit->second;

        EV << "LteMacUeRealisticD2D: pduMaker created PDU: " << macPkt->info() << endl;

        if (txList.second.empty())
        {
            EV << "LteMacUeRealisticD2D() : no available process for this MAC pdu in TxHarqBuffer" << endl;
            delete macPkt;
        }
        else
        {
            //Insert PDU in the Harq Tx Buffer
            //txList.first is the acid
            txBuf->insertPdu(txList.first,cw, macPkt);
        }
    }
}

UserTxParams* Satmac::getPreconfiguredTxParams()
{
    UserTxParams* txParams = new UserTxParams();

    // default parameters for D2D
    txParams->isSet() = true;
    txParams->writeTxMode(SINGLE_ANTENNA_PORT0);
    Rank ri = 1;                                              // rank for TxD is one
    txParams->writeRank(ri);
    txParams->writePmi(intuniform(1, pow(ri, (double) 2)));   // taken from LteFeedbackComputationRealistic::computeFeedback

    BandSet b;
    for (Band i = 0; i < deployer_->getNumBands(); ++i) b.insert(i);
    txParams->writeBands(b);

    RemoteSet antennas;
    antennas.insert(MACRO);
    txParams->writeAntennas(antennas);

    return txParams;
}

void Satmac::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage())
    {
        LteMacUeRealisticD2D::handleMessage(msg);
        return;
    }

    cPacket* pkt = check_and_cast<cPacket *>(msg);
    cGate* incoming = pkt->getArrivalGate();

    if (incoming == down_[IN])
    {
        if(strcmp(pkt->getName(), "Coordinate Information") == 0)
        {
            Coordinate* coordinate = check_and_cast<Coordinate*>(msg);
            this->x = coordinate->getX();
            this->y = coordinate->getY();
            delete pkt;
            return;
        }
        UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->getControlInfo());
        if (lteInfo->getFrameType() == SATMACPKT)
        {
            emit(macRcvdSatmacPkt,1);
            SatmacPacket* satmacPkt = check_and_cast<SatmacPacket*>(msg);
            int recv_fi_frame_fi = 0;
            Frame_info *fi_recv;

            if (adj_frame_ena_)
                recv_fi_frame_fi = satmacPkt->getFrameLength();
            else
                recv_fi_frame_fi = m_frame_len;

            fi_recv = this->get_new_FI(recv_fi_frame_fi);
            fi_recv->sti = satmacPkt->getGlobleSti();
            fi_recv->frame_len = recv_fi_frame_fi;
            //fi_recv->channel_num = this->m_channel_num;
            fi_recv->recv_slot = this->slot_count_;
            fi_recv->valid_time = this->m_frame_len;
            fi_recv->remain_time = fi_recv->valid_time;

            //fi_recv->slot_describe[0][0].busy = 1;
            //获取slot_tag
            slot_tag **fi_local_= fi_recv->slot_describe;
            for(int i = 0; i < recv_fi_frame_fi; i++)
            {
                for(int j = 0; j < m_channel_num; j++)
                {
                    fi_local_[i][j].busy = satmacPkt->getSlotTag()[i][j].busy;
                    fi_local_[i][j].sti = satmacPkt->getSlotTag()[i][j].sti;
                    fi_local_[i][j].count_2hop = satmacPkt->getSlotTag()[i][j].count_2hop;
                    fi_local_[i][j].psf = satmacPkt->getSlotTag()[i][j].psf;
                }
            }
            EV << "This node " << nodeId_ <<endl;
            EV<<"SATMAC: received satmacPkt ！" <<endl;
            EV << "From node " << satmacPkt->getGlobleSti()<<endl;
            EV << "Frame length is " << satmacPkt->getFrameLength()<<endl;
            EV << "This is FI information:" << endl;
            for(int i = 0; i < recv_fi_frame_fi; i++)
            {
                for(int j = 0; j < m_channel_num; j++)
                {
                    EV << "This slot " <<"[" << i << "][" << j << "]" << "is occuried by " << fi_local_[i][j].sti << "\t"
                            << "2HOP is " << fi_local_[i][j].count_2hop<< ", state is " << (int)fi_local_[i][j].busy << endl;
                }
            }
            delete pkt;
            return;
        }
    }
    else if (incoming == up_[IN])
    {
//        if (strcmp(pkt->getName(), "newDataPkt")== 0)
//        {
//            FlowControlInfoNonIp* lteInfo = check_and_cast<FlowControlInfoNonIp*>(pkt->removeControlInfo());
//            receivedTime_ = NOW;
//            simtime_t elapsedTime = receivedTime_ - lteInfo->getCreationTime();
//            remainingTime_ = lteInfo->getDuration() - (elapsedTime.dbl() * 1000);
//
//            if (schedulingGrant_ == NULL)
//            {
//                macGenerateSchedulingGrant(remainingTime_, lteInfo->getPriority(), pkt->getBitLength());
//            }
//            else if ((schedulingGrant_ != NULL && periodCounter_ > remainingTime_))
//            {
//                emit(grantBreakTiming, 1);
//                delete schedulingGrant_;
//                schedulingGrant_ = NULL;
//                macGenerateSchedulingGrant(remainingTime_, lteInfo->getPriority(), pkt->getBitLength());
//            }
//            else
//            {
//                LteMode4SchedulingGrant* mode4Grant = check_and_cast<LteMode4SchedulingGrant*>(schedulingGrant_);
//                mode4Grant->setSpsPriority(lteInfo->getPriority());
//                // Need to get the creation time for this
//                mode4Grant->setMaximumLatency(remainingTime_);
//            }
//            // Need to set the size of our grant to the correct size we need to ask rlc for, i.e. for the sdu size.
//            schedulingGrant_->setGrantedCwBytes((MAX_CODEWORDS - currentCw_), pkt->getBitLength());
//
//            pkt->setControlInfo(lteInfo);
//        }
    }

    LteMacUeRealisticD2D::handleMessage(msg);
}



void Satmac::handleSelfMessage()
{
    //EV << "----- UE MAIN LOOP -----" << endl;
    if(geoHash_NQueens)
    {

        // 每隔1ms向物理层发送需要坐标的数据包
        AskCoordinate* askCoordinate = new AskCoordinate("askCoordinate");
        UserControlInfo* uinfo = new UserControlInfo();
        uinfo->setSourceId(getMacNodeId());
        uinfo->setDestId(getMacNodeId());
        uinfo->setFrameType(ASKCOORDINATEPKT);
        askCoordinate->setControlInfo(uinfo);
        LteMacBase::send(askCoordinate,down_[OUT]);
            //delete askCoordinate;

    }


    // extract pdus from all harqrxbuffers and pass them to unmaker
    HarqRxBuffers::iterator hit = harqRxBuffers_.begin();
    HarqRxBuffers::iterator het = harqRxBuffers_.end();
    LteMacPdu *pdu = NULL;
    std::list<LteMacPdu*> pduList;

    for (; hit != het; ++hit)
    {
        pduList=hit->second->extractCorrectPdus();
        while (! pduList.empty())
        {
            pdu=pduList.front();
            pduList.pop_front();
            macPduUnmake(pdu);
        }
    }

    unsigned int purged =0;
    // purge from corrupted PDUs all Rx H-HARQ buffers
    for (hit= harqRxBuffers_.begin(); hit != het; ++hit)
    {
        purged += hit->second->purgeCorruptedPdus();
    }
    //EV << NOW << " Satmac::handleSelfMessage Purged " << purged << " PDUS" << endl;

    //EV << NOW << "Satmac::handleSelfMessage " << nodeId_ << " - HARQ process " << (unsigned int)currentHarq_ << endl;
    // updating current HARQ process for next TTI

    //unsigned char currentHarq = currentHarq_;

    // no grant available - if user has backlogged data, it will trigger scheduling request
    // no harq counter is updated since no transmission is sent.
    slotHandler();

    EV << "当前节点ID:" << nodeId_ << endl;
    EV << "节点坐标：(" << this->x << "," << this->y << ")"<<endl;
    EV << "所占用子信道：" << channel_num_ <<endl;

    SatmacSchedulingGrant* satmacGrant = dynamic_cast<SatmacSchedulingGrant*>(schedulingGrant_);

    if (satmacGrant == NULL)
        EV << NOW << " LteMacVUeMode4::handleSelfMessage " << nodeId_ << " NO configured grant" << endl;
    else
    {
        if (periodCounter_-->0 && !firstTx) // 不是该节点占用的时隙
           return;
        else if(periodCounter_ <= 0)    // grant失效，并在flushHarqBuffer()后delete grant
        {
            emit(grantBreak, 1);
            satmacGrant->setExpiration(0);
            expiredGrant_ = true;
        }
    }
//    bool requestSdu = false;
//    if (satmacGrant!=NULL) // if a grant is configured
//    {
//        bool retx = false;
//        bool availablePdu = false;
//
//        HarqTxBuffers::iterator it2;
//        LteHarqBufferTx * currHarq;
//        for(it2 = harqTxBuffers_.begin(); it2 != harqTxBuffers_.end(); it2++)
//        {
//            EV << "\t Looking for retx in acid " << (unsigned int)currentHarq_ << endl;
//            currHarq = it2->second;
//
//            // check if the current process has unit ready for retx
//            retx = currHarq->getProcess(currentHarq_)->hasReadyUnits();
//            CwList cwListRetx = currHarq->getProcess(currentHarq_)->readyUnitsIds();
//
//            if (it2->second->isSelected())
//            {
//                LteHarqProcessTx* selectedProcess = it2->second->getSelectedProcess();
//                // Ensure that a pdu is not already on the HARQ buffer awaiting sending.
//                if (selectedProcess != NULL)
//                {
//                    for (int cw=0; cw<MAX_CODEWORDS; cw++)
//                    {
//                        if (selectedProcess->getPduLength(cw) != 0)
//                        {
//                            availablePdu = true;
//                        }
//                    }
//                }
//            }
//
//            EV << "\t [process=" << (unsigned int)currentHarq_ << "] , [retx=" << ((retx)?"true":"false")
//               << "] , [n=" << cwListRetx.size() << "]" << endl;
//
//            // if a retransmission is needed
//            if(retx)
//            {
//                UnitList signal;
//                signal.first=currentHarq_;
//                signal.second = cwListRetx;
//                currHarq->markSelected(signal,schedulingGrant_->getUserTxParams()->getLayers().size());
//            }
//        }
//        // if no retx is needed, proceed with normal scheduling
//        // TODO: This may yet be changed to appear after MCS selection, issue is that if you pick max then you might get more sdus then you want
//        // Basing it on the previous mcs value is at least more realistic as to the size of the pdu you will get.
//        if(!retx && !availablePdu)
//        {
//            scheduleList_ = lcgScheduler_->schedule();
//            bool sent = macSduRequest();
//
//            if (!sent)
//            {
//                macPduMake();
//            }
//
//            requestSdu = sent;
//        }
//        // Message that triggers flushing of Tx H-ARQ buffers for all users
//        // This way, flushing is performed after the (possible) reception of new MAC PDUs
//        cMessage* flushHarqMsg = new cMessage("flushHarqMsg");
//        flushHarqMsg->setSchedulingPriority(1);        // after other messages
//        scheduleAt(NOW, flushHarqMsg);
//    }
//    //============================ DEBUG ==========================
//    HarqTxBuffers::iterator it;
//
//    EV << "\n htxbuf.size " << harqTxBuffers_.size() << endl;
//
//    int cntOuter = 0;
//    int cntInner = 0;
//    for(it = harqTxBuffers_.begin(); it != harqTxBuffers_.end(); it++)
//    {
//        LteHarqBufferTx* currHarq = it->second;
//        BufferStatus harqStatus = currHarq->getBufferStatus();
//        BufferStatus::iterator jt = harqStatus.begin(), jet= harqStatus.end();
//
//        EV_DEBUG << "\t cicloOuter " << cntOuter << " - bufferStatus.size=" << harqStatus.size() << endl;
//        for(; jt != jet; ++jt)
//        {
//            EV_DEBUG << "\t\t cicloInner " << cntInner << " - jt->size=" << jt->size()
//               << " - statusCw(0/1)=" << jt->at(0).second << "/" << jt->at(1).second << endl;
//        }
//    }
//    //======================== END DEBUG ==========================
//
//    if (!requestSdu)
//    {
//        // update current harq process id
//        currentHarq_ = (currentHarq_+1) % harqProcesses_;
//    }

    //EV << "--- END UE MAIN LOOP ---" << endl;
}

void Satmac::show_slot_occupation() {
    int i,j,free_count = 0;
    std::map<unsigned int,int> omap;
    slot_tag **fi_local_= this->collected_fi_->slot_describe;
    for(i=0 ; i < m_frame_len; i++){
        for(j = 0; j < m_channel_num; j++){
            if(fi_local_[i][j].busy== SLOT_FREE)
                free_count++;
            else {
                if (omap[fi_local_[i][j].sti]) //sti = ID
                    EV << "Node " << fi_local_[i][j].sti <<" has occupied more than one slot!";  // 需要改
                else
                    omap[fi_local_[i][j].sti] = 1;
            }
        }
    }
    //NS_LOG_DEBUG("FREE SLOT: " << free_count); // 需要改
}

void Satmac::clear_others_slot_status() {
    slot_tag **fi_local = this->collected_fi_->slot_describe;
    int count,j;
    for (count=0; count < m_frame_len; count++){
        for(j = 0; j < m_channel_num; j++)
        {
            if (fi_local[count][j].sti != nodeId_) {
                fi_local[count][j].busy = SLOT_FREE;
                fi_local[count][j].sti = 0;
                fi_local[count][j].count_2hop = 0;
                fi_local[count][j].count_3hop = 0;
                fi_local[count][j].psf = 0;
                fi_local[count][j].locker = 0;
            }
        }
    }
//    for (count=0; count < m_frame_len; count++){
//        if (fi_local[count][j].sti != nodeId_) {
//            fi_local[count][j].busy = SLOT_FREE;
//            fi_local[count][j].sti = 0;
//            fi_local[count][j].count_2hop = 0;
//            fi_local[count][j].count_3hop = 0;
//            fi_local[count][j].psf = 0;
//            fi_local[count][j].locker = 0;
//        }
//    }
}

//初始化一个fi记录
void Satmac::clear_FI(Frame_info *fi){
    //fi->frame_len;
    //fi->index;
    //fi->sti;
    fi->valid_time = 0;
    fi->remain_time = 0;
    fi->recv_slot = -1;
    if(fi->slot_describe != NULL){
        for(int i = 0; i < m_frame_len; i++)
        {
            delete[] fi->slot_describe[i];
        }
        delete[] fi->slot_describe;
    }
    fi->slot_describe = new slot_tag*[512];
    for(int i = 0; i < 512; i++)
    {
        fi->slot_describe[i] = new slot_tag[m_channel_num];
    }
}

/*
 * allocate a new fi and add insert in the head of received_fi_list;
 */
Frame_info * Satmac::get_new_FI(int slot_count){
    Frame_info *newFI= new Frame_info(slot_count, m_channel_num);
//  newFI->next_fi = this->received_fi_list_;
    Frame_info *tmp;

    if (received_fi_list_ == NULL)
        received_fi_list_ = newFI;
    else {
        for (tmp = received_fi_list_; tmp->next_fi != NULL; tmp = tmp->next_fi) {}
        tmp->next_fi = newFI;
    }
    newFI->next_fi = NULL;

    return newFI;
}

void Satmac::print_slot_status(void) {
    slot_tag **fi_local = this->collected_fi_->slot_describe;
    int i, j,count;
    int free_count_ths = 0, free_count_ehs = 0;
    for(i=0 ; i < m_frame_len; i++){
        for(j = 0; j < m_channel_num; j++)
        {
            if (fi_local[i][j].busy== SLOT_FREE)
                free_count_ths++;
            if(fi_local[i][j].busy== SLOT_FREE && fi_local[i][j].count_3hop == 0)
                free_count_ehs++;
        }
    }
    //NS_LOG_DEBUG("I'm node "<<nodeId_<<" in slot " <<slot_count_<<" FreeThs: "<<free_count_ths<<", Ehs "
            //<<free_count_ehs<<" total "<< m_frame_len<<" status: ");
    for (count=0; count < m_frame_len; count++){
        //NS_LOG_DEBUG("|| "<< fi_local[count].sti<<" ");
        for(j = 0; j < m_channel_num; j++)
        {
            switch (fi_local[count][j].busy) {
            case SLOT_FREE:
                //NS_LOG_DEBUG("(0,0) ");
                break;
            case SLOT_1HOP:
                //NS_LOG_DEBUG("(1,0) ");
                break;
            case SLOT_2HOP:
                //NS_LOG_DEBUG("(0,1) ");
                break;
            case SLOT_COLLISION:
                //NS_LOG_DEBUG("(1,1) ");
                break;
            }
        }

    }

}

bool Satmac::isNewNeighbor(int sid,int channelNum) {
    slot_tag **fi_local = this->collected_fi_->slot_describe;
    int count;
    for (count=0; count < m_frame_len; count++){
        if (fi_local[count][channelNum].sti == sid)
            return false;
    }
    return true;
}

/* This function is used to pick up a random slot of from those which is free. */
int Satmac::determine_BCH(bool strict){
    int i=0,j,chosen_slot=0;
//  int loc;
    slot_tag **fi_local_= this->collected_fi_->slot_describe;
//  int s1c[256];
    int s2c[256];
    int s0c[256];
    int s0_1c[128];
    int s0_2c[128];
//  int s2_1c[128];

//  int s1c_num = 0, s2_1c_num = 0;
    int s2c_num = 0, s0c_num = 0;
    int s0_1c_num = 0;
    int s0_2c_num = 0;
    int free_count_ths = 0, free_count_ehs = 0;

    for(i=0 ; i < m_frame_len; i++){

            if((fi_local_[i][channel_num_].busy== SLOT_FREE || (!strict && fi_local_[i][channel_num_].sti==nodeId_)) && !fi_local_[i][channel_num_].locker) {
                if (adj_ena_) {
                    s2c[s2c_num++] = i;
                    // if (i < m_frame_len/2)
                    // s2_1c[s2_1c_num++] = i;

                    if (fi_local_[i][channel_num_].count_3hop  == 0) {
                        s0c[s0c_num++] = i;
                        // s1c[s1c_num++] = i;
                        if (i < m_frame_len/2)
                            s0_1c[s0_1c_num++] = i;
                    } else if (fi_local_[i][channel_num_].count_3hop < c3hop_threshold_ ){
                        // s1c[s1c_num++] = i;
                    }

                } else {
                    s0c[s0c_num++] = i;
                }
            }

    }

    for(i=0 ; adj_frame_ena_ && i < m_frame_len; i++){
        //for(j = 0; j < m_channel_num; j++)
        //{
            if (fi_local_[i][channel_num_].busy== SLOT_FREE)
                free_count_ths++;
            if(fi_local_[i][channel_num_].busy== SLOT_FREE && fi_local_[i][channel_num_].count_3hop == 0)
                free_count_ehs++;
        //}

    }

    //Choose slot only in the fist half of frame (when adjusting slot)
    if (adj_frame_ena_&& m_frame_len > adj_frame_lower_bound_
                      &&  (((float)(m_frame_len - free_count_ehs))/m_frame_len) <= frameadj_cut_ratio_ehs_
                      && (((float)(m_frame_len - free_count_ths))/m_frame_len) <= frameadj_cut_ratio_ths_)
    {
        if (s0_1c_num != 0) {
            chosen_slot = intuniform(0, s0_1c_num-1);
            return s0_1c[chosen_slot];
        }
    }

    if (testmode_init_flag_ && choose_bch_random_switch_ == 2) {
        testmode_init_flag_ = 0;
        switch (nodeId_) {
        case 1: return 0;
        case 2: return 1;
        case 3: return 2;
        case 4: return 0;
        default: return nodeId_ -1;
        }
    }

    if (!adj_ena_) {
        if (s0c_num > 0) {
            chosen_slot = intuniform(0, s0c_num-1);
            return s0c[chosen_slot];
        } else {

//  show_slot_occupation();
//  print_slot_status();

            return -1;
        }
    } else {
        if (/*strict &&*/ s0c_num >= adj_free_threshold_) {
            if (choose_bch_random_switch_) {
                chosen_slot = intuniform(0, s0c_num-1);
            } else
                chosen_slot = 0;
            return s0c[chosen_slot];
        } else if (s2c_num != 0) {
            if (choose_bch_random_switch_)
                chosen_slot = intuniform(0, s2c_num-1);
            else
                chosen_slot = 0;
            return s2c[chosen_slot];
        } else {

//  show_slot_occupation();
//  print_slot_status();

            return -1;
        }
    }

}

pair<int,int> Satmac::determine_BCH_GeoHash(bool strict){
    int i=0,j,chosen_slot=0;
    bool temp;
//  int loc;
    slot_tag **fi_local_= this->collected_fi_->slot_describe;
//  int s1c[256];
    int s2c[256];   // 用到
    pair<int,int> s2c_[256];

    int s0c[256];   // 用到
    pair<int,int> s0c_[256];

    int s0_1c[128]; // 用到
    pair<int,int> s0_1c_[128];

    int s0_2c[128];
//  int s2_1c[128];

//  int s1c_num = 0, s2_1c_num = 0;
    int s2c_num = 0, s0c_num = 0;
    int s0_1c_num = 0;
    int s0_2c_num = 0;
    int free_count_ths = 0, free_count_ehs = 0;

    //int selected_channel_num = selectSubChannel();
    int selected_channel_num = intuniform(0,2);
    //int selected_channel_num = 2;
    for(i=0 ; i < m_frame_len; i++){
        temp = false;
        for (j = 0; j < m_channel_num; j++){
            if(fi_local_[i][j].busy == SLOT_1HOP||fi_local_[i][j].busy == SLOT_2HOP){
                temp = true;
                break;
            }
        }
        if(temp==true)
            continue;
        if((fi_local_[i][selected_channel_num].busy == SLOT_FREE || (!strict && fi_local_[i][selected_channel_num].sti==nodeId_)) && !fi_local_[i][selected_channel_num].locker) {
            if (adj_ena_) {
                // s2c[s2c_num++] = i; 修改
                s2c_[s2c_num++] = {i,selected_channel_num};

                if (fi_local_[i][selected_channel_num].count_3hop  == 0 ) {
                    // s0c[s0c_num++] = i; 修改
                    s0c_[s0c_num++] = {i,selected_channel_num};
                    // s1c[s1c_num++] = i;
                    if (i < m_frame_len/2)
                    {
                        // s0_1c[s0_1c_num++] = i; 修改
                        s0_1c_[s0_1c_num++] = {i,selected_channel_num};
                    }
                } else if (fi_local_[i][selected_channel_num].count_3hop < c3hop_threshold_ ){
                    // s1c[s1c_num++] = i;
                }

            } else {
                // s0c[s0c_num++] = i; 修改
                s0c_[s0c_num++] = {i,selected_channel_num};
            }
        }
    }

    for(i=0 ; adj_frame_ena_ && i < m_frame_len; i++){
        for(j = 0; j < m_channel_num; j++)
        {
            if (fi_local_[i][j].busy== SLOT_FREE)
                free_count_ths++;
            if(fi_local_[i][j].busy== SLOT_FREE && fi_local_[i][j].count_3hop == 0)
                free_count_ehs++;
        }

    }

    //Choose slot only in the fist half of frame (when adjusting slot)
    if (adj_frame_ena_&& m_frame_len > adj_frame_lower_bound_
                      &&  (((float)(m_frame_len - free_count_ehs))/m_frame_len) <= frameadj_cut_ratio_ehs_
                      && (((float)(m_frame_len - free_count_ths))/m_frame_len) <= frameadj_cut_ratio_ths_)
    {
        if (s0_1c_num != 0) {
            chosen_slot = intuniform(0, s0_1c_num-1);
            // return s0_1c[chosen_slot]; 修改
            return s0_1c_[chosen_slot];
        }
    }

    // 这里应该不需要
//    if (testmode_init_flag_ && choose_bch_random_switch_ == 2) {
//        testmode_init_flag_ = 0;
//        switch (nodeId_) {
//        case 1: return 0;
//        case 2: return 1;
//        case 3: return 2;
//        case 4: return 0;
//        default: return nodeId_ -1;
//        }
//    }

    if (!adj_ena_) {
        if (s0c_num > 0) {
            chosen_slot = intuniform(0, s0c_num-1);
            //return s0c[chosen_slot]; 修改
            return s0c_[chosen_slot];
        } else {

//  show_slot_occupation();
//  print_slot_status();

            return {-1,-1};
        }
    } else {
        if (/*strict &&*/ s0c_num >= adj_free_threshold_) {
            if (choose_bch_random_switch_) {
                chosen_slot = intuniform(0, s0c_num-1);
            } else
                chosen_slot = 0;
            // return s0c[chosen_slot]; 修改
                return s0c_[chosen_slot];
        } else if (s2c_num != 0) {
            if (choose_bch_random_switch_)
                chosen_slot = intuniform(0, s2c_num-1);
            else
                chosen_slot = 0;
            // return s2c[chosen_slot]; 修改
                return s2c_[chosen_slot];
        } else {

//  show_slot_occupation();
//  print_slot_status();

            return {-1,-1};
        }
    }

}

void Satmac::merge_fi(Frame_info* base, Frame_info* append, Frame_info* decision){
    int count=0,j;
    slot_tag **fi_local_ = base->slot_describe;
    slot_tag **fi_append = append->slot_describe;
    slot_tag recv_tag;
    int recv_fi_frame_len = append->frame_len;

//  printf("I'm n%d, start merge fi from n %d\n", nodeId_,append->sti);
    // status of our BCH should be updated first.
    for(j = 0; j < m_channel_num; j++)
    {
    for (count=0; count < m_frame_len; count++){
        if (count == recv_fi_frame_len)
            break;
//        for(j = 0; j < m_channel_num; j++)
//        {

            recv_tag = fi_append[count][j];

            if (fi_local_[count][j].sti == nodeId_ ) {//我自己的时隙
                // if (count != slot_num_ && count != slot_adj_candidate_) {
                // printf("I'm node %d, I recv a strange pkt..\n",nodeId_);
                // continue;
                //   }
                if (fi_local_[count][j].sti != recv_tag.sti && recv_tag.sti != 0) {//FI记录的id和我不一致
                    switch (recv_tag.busy)
                    {
                        case SLOT_1HOP:
                            if (recv_tag.psf > fi_local_[count][j].psf) {
                                fi_local_[count][j].life_time = slot_lifetime_frame_;
                                fi_local_[count][j].sti = recv_tag.sti;
                                fi_local_[count][j].count_2hop ++;
                                if(fi_local_[count][j].count_2hop>1)
                                    cout<<" ";
                                fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                                if (recv_tag.sti == append->sti) { //FI发送者是该时隙的占有者
                                    fi_local_[count][j].busy = SLOT_1HOP;
                                } else {
                                    fi_local_[count][j].busy = SLOT_2HOP;
                                }
                            } else if (recv_tag.psf == fi_local_[count][j].psf) {
                                fi_local_[count][j].busy = SLOT_COLLISION;
                            }
                            break;
                        case SLOT_2HOP:
                            fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                            break;
                        case SLOT_FREE:
                            //出现了隐藏站
                            fi_local_[count][j].busy = SLOT_COLLISION;
                            break;
                        case SLOT_COLLISION:
                            fi_local_[count][j].life_time = slot_lifetime_frame_;
                            fi_local_[count][j].sti = recv_tag.sti;
                            fi_local_[count][j].count_2hop = 1;
                            fi_local_[count][j].count_3hop = 1;
                            fi_local_[count][j].busy = SLOT_2HOP;
                            break;
                    }
                } else if (fi_local_[count][j].sti == recv_tag.sti){ //FI记录的id和我一致
                    switch (recv_tag.busy)
                    {
                        case SLOT_1HOP:
    //                      if (recv_tag.count_2hop > 1)
    //                          fi_local_[count].count_3hop += recv_tag.count_2hop;
                            break;
                        case SLOT_2HOP:
    //                      //出现了隐藏站
    //                      fi_local_[count].busy = SLOT_COLLISION;
                            break;
                        case SLOT_FREE:
                            //出现了隐藏站
                            fi_local_[count][j].busy = SLOT_COLLISION;
                            break;
                        case SLOT_COLLISION:
                            break;
                    }
                } else { //STI-slot == 0
                    if (recv_tag.busy == SLOT_FREE && count != slot_adj_candidate_&& j != channelNumber_adj) {
                        if (!isNewNeighbor(append->sti,j)) {
                            //出现了隐藏站
                            fi_local_[count][j].busy = SLOT_COLLISION;
                        }
                    } else {
                        //error state.
                    }
                }
            }
        //}

    }
    }

    //遍历每一个时隙
    for(j = 0; j < m_channel_num; j++){
        for (count=0; count < ((recv_fi_frame_len > m_frame_len)?recv_fi_frame_len:m_frame_len); count++){
            if (count == recv_fi_frame_len)
                break;

    //        if (count >= m_frame_len ) {
    //            if (fi_local_[count][j].sti != 0)
    //                printf("merge_fi: node %d Protocol ERROR!!\n", nodeId_);
    //        }

    //        for(j = 0;j < m_channel_num; j++)
    //        {
                if (fi_local_[count][j].locker == 1)
                    continue;

                //merge the recv_tag to fi_local_[slot_pos]
                recv_tag = fi_append[count][j];
                if (fi_local_[count][j].sti == nodeId_ || recv_tag.sti == nodeId_)
                    continue;
                else if (fi_local_[count][j].busy == SLOT_1HOP && fi_local_[count][j].sti != nodeId_) {//直接邻居占用
                    if (fi_local_[count][j].sti != recv_tag.sti && recv_tag.sti != 0) {
                        switch (recv_tag.busy)
                        {
                            case SLOT_1HOP:
                                if (recv_tag.sti == append->sti) { //FI发送者是该时隙的占有者
                                    if (recv_tag.psf > fi_local_[count][j].psf) {
                                        fi_local_[count][j].life_time = slot_lifetime_frame_;
                                        fi_local_[count][j].sti = recv_tag.sti;
                                        fi_local_[count][j].count_2hop ++;
                                        if(fi_local_[count][j].count_2hop>1)
                                                                            cout<<" ";
                                        fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                                        fi_local_[count][j].busy = SLOT_1HOP;
                                    } else if (recv_tag.psf == fi_local_[count][j].psf) {
                                        fi_local_[count][j].life_time = slot_lifetime_frame_;
                                        fi_local_[count][j].busy = SLOT_COLLISION;
                                    }
                                } else {
                                    fi_local_[count][j].count_2hop ++;
                                    if(fi_local_[count][j].count_2hop>1)
                                                                        cout<<" ";
                                    fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                                }
                                break;
                            case SLOT_2HOP:
                                fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                                break;
                            case SLOT_FREE:
                                break;
                            case SLOT_COLLISION:
                                fi_local_[count][j].life_time = slot_lifetime_frame_;
                                fi_local_[count][j].sti = recv_tag.sti;
                                fi_local_[count][j].count_2hop = 1;
                                fi_local_[count][j].count_3hop = 1;
                                fi_local_[count][j].busy = SLOT_2HOP;
                                break;
                        }
                    } else if (fi_local_[count][j].sti == recv_tag.sti){ //FI记录的id和我一致
                        switch (recv_tag.busy)
                        {
                            case SLOT_1HOP:
                                if (recv_tag.sti == append->sti) { //FI发送者是该时隙的占有者
                                        fi_local_[count][j].life_time = slot_lifetime_frame_;
                                    if (fi_local_[count][j].c3hop_flag == 0) {
                                        fi_local_[count][j].count_2hop ++;
                                        if(fi_local_[count][j].count_2hop>1)
                                                                            cout<<" ";
                                        fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                                        fi_local_[count][j].c3hop_flag = 1;
                                    }
                                } else {
                                    fi_local_[count][j].existed = 1;
                                    // do nothing.
                                }

                                break;
                            case SLOT_2HOP:
                            case SLOT_FREE:
                            case SLOT_COLLISION:
                                break;
                        }
                    } else { //STI-slot == 0
                        if (append->sti == fi_local_[count][j].sti) {
                            fi_local_[count][j].life_time = 0;
                            fi_local_[count][j].sti = 0;
                            fi_local_[count][j].count_2hop = 0;
                            fi_local_[count][j].count_3hop = 0;
                            fi_local_[count][j].busy = SLOT_FREE;
                            fi_local_[count][j].locker = 1;
                        }
                    }
                }else if (fi_local_[count][j].busy == SLOT_2HOP) {//两跳邻居占用
                    if (fi_local_[count][j].sti != recv_tag.sti && recv_tag.sti != 0) {
                        switch (recv_tag.busy)
                        {
                            case SLOT_1HOP:
                                fi_local_[count][j].count_2hop ++;
                                if(fi_local_[count][j].count_2hop>1)
                                                                    cout<<" ";
                                fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                                break;
                            case SLOT_2HOP:
                            case SLOT_FREE:
                                break;
                            case SLOT_COLLISION:
                                fi_local_[count][j].life_time = slot_lifetime_frame_;
                                fi_local_[count][j].sti = recv_tag.sti;
                                fi_local_[count][j].count_2hop = 1;
                                fi_local_[count][j].count_3hop = 1;
                                fi_local_[count][j].busy = SLOT_2HOP;
                                break;
                        }
                    } else if (fi_local_[count][j].sti == recv_tag.sti){ //FI记录的id和我一致
                        switch (recv_tag.busy)
                        {
                            case SLOT_1HOP:
                                if (recv_tag.sti == append->sti) { //FI发送者是该时隙的占有者
                                    fi_local_[count][j].busy = SLOT_1HOP;
                                    fi_local_[count][j].life_time = slot_lifetime_frame_;
                                    if (fi_local_[count][j].c3hop_flag == 0) {
                                        fi_local_[count][j].c3hop_flag = 1;
                                        fi_local_[count][j].count_2hop ++;
                                        if(fi_local_[count][j].count_2hop>1)
                                                                            cout<<" ";
                                        fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                                    }
                                } else {
                                    fi_local_[count][j].life_time = slot_lifetime_frame_;
                                    if (fi_local_[count][j].c3hop_flag == 0) {
                                        fi_local_[count][j].c3hop_flag = 1;
                                        fi_local_[count][j].count_2hop ++;
                                        if(fi_local_[count][j].count_2hop>1)
                                                                            cout<<" ";
                                        fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                                    }
                                }
                                break;
                            case SLOT_2HOP:
                            case SLOT_FREE:
                            case SLOT_COLLISION:
                                break;
                        }
                    } else { //STI-slot == 0
                        if (append->sti == fi_local_[count][j].sti) {
                            fi_local_[count][j].life_time = 0;
                            fi_local_[count][j].sti = 0;
                            fi_local_[count][j].count_2hop = 0;
                            fi_local_[count][j].count_3hop = 0;
                            fi_local_[count][j].busy = SLOT_FREE;
                            fi_local_[count][j].locker = 1;
                        }
                    }
                } else if (fi_local_[count][j].busy == SLOT_FREE && fi_local_[count][j].sti == 0){ //空闲时隙
                    if (fi_local_[count][j].sti != recv_tag.sti) {
                        switch (recv_tag.busy)
                        {
                            case SLOT_1HOP:
                                fi_local_[count][j].life_time = slot_lifetime_frame_;
                                fi_local_[count][j].sti = recv_tag.sti;
                                fi_local_[count][j].count_2hop = 1;
                                fi_local_[count][j].count_3hop = recv_tag.count_2hop;
                                fi_local_[count][j].c3hop_flag = 1;
                                if (recv_tag.sti == append->sti) { //FI发送者是该时隙的占有者
                                    fi_local_[count][j].busy = SLOT_1HOP;
                                } else {
                                    fi_local_[count][j].busy = SLOT_2HOP;
                                }
                                break;
                            case SLOT_2HOP:
                                fi_local_[count][j].count_3hop += recv_tag.count_2hop;
                                break;
                            case SLOT_FREE:
                                break;
                            case SLOT_COLLISION:
                                fi_local_[count][j].life_time = slot_lifetime_frame_;
                                fi_local_[count][j].sti = recv_tag.sti;
                                fi_local_[count][j].count_2hop = 1;
                                fi_local_[count][j].count_3hop = 1;
                                fi_local_[count][j].busy = SLOT_2HOP;
                                break;
                        }
                    }
                }

                if (count >= m_frame_len && fi_local_[count][j].sti != 0) {
                    //NS_LOG_DEBUG("I'm node "<<nodeId_<<" I restore frame len from "<<m_frame_len<<" to "<<recv_fi_frame_len);
                    m_frame_len = recv_fi_frame_len;
                }
            //}

        }
    }
    return;
}

bool Satmac::isSingle(int channelNum) {
    slot_tag **fi_local = this->collected_fi_->slot_describe;
    int count;
    for (count=0; count < m_frame_len; count++){
//        for(j = 0; j < m_channel_num; j++)
//        {
            if (fi_local[count][channelNum].sti != 0 && fi_local[count][channelNum].sti != nodeId_)
                return false;
//        }

    }
    return true;
}

/*
 * reduce the remain_time of each of received_fi_list_
 * if the argument time ==0 then clear the received_fi_list_;
 */
void Satmac::fade_received_fi_list(int time){   //检查FI链表中的每个FI是否失效，失效则删除
    Frame_info *current, *previous;
    current=this->received_fi_list_;
    previous=NULL;

    while(current != NULL){
        current->remain_time -= time;
        if(current->remain_time <= 0 || time == 0){
            if(previous == NULL){   //前一个节点为空时，删除当前节点
                this->received_fi_list_ = current->next_fi;
                delete current;
                current = this->received_fi_list_;
                continue;
            }
            else{                   //前一个节点不为空时，删除当前节点
                previous->next_fi= current->next_fi;
                delete current;
                current = previous->next_fi;
                continue;
            }
        }
        else{
            previous = current;
            current = current->next_fi;
        }
    }
}

void Satmac::synthesize_fi_list(){
    Frame_info * processing_fi = received_fi_list_;
    int count;
    int j;
    slot_tag **fi_local = this->collected_fi_->slot_describe;    //本地的时隙使用信息
    //slot_describe指向slot_tag类型的数组，大小=帧长，储存每个时隙的使用信息

    bool unlock_flag = 0;

    if (node_state_ != NODE_LISTEN && slot_memory_) {
        for(j = 0; j < m_channel_num; j++)
        {
            for (count=0; count < m_frame_len; count++){
        //            for(j = 0; j < m_channel_num; j++)
        //            {
                    if (fi_local[count][j].locker && fi_local[count][j].sti != 0) {
                        fi_local[count][j].locker = 0; //the locker must be locked in the last frame.
                    } else if (fi_local[count][j].locker)
                        unlock_flag = 1;

                    // 不知道(count == slot_num_ || count == slot_adj_candidate_)要不要加上channel_num_
                    if ((fi_local[count][j].sti == nodeId_ && ((count == slot_num_ && j == channel_num_) || (count == slot_adj_candidate_ && j == channelNumber_adj)))
                    || fi_local[count][j].sti == 0)
                        continue;
                    if (fi_local[count][j].life_time > 0)
                        fi_local[count][j].life_time--;

                    if (fi_local[count][j].life_time == 0) {
                        if (fi_local[count][j].busy == SLOT_2HOP) {
                            fi_local[count][j].busy = SLOT_FREE;
                            fi_local[count][j].sti = 0;
                            fi_local[count][j].count_2hop = 0;
                            fi_local[count][j].count_3hop = 0;
                            fi_local[count][j].psf = 0;
                            fi_local[count][j].c3hop_flag = 0;
                            fi_local[count][j].life_time = 0;
                            fi_local[count][j].locker = 0;
                        } else if (fi_local[count][j].busy == SLOT_1HOP && fi_local[count][j].existed == 1) {
                            fi_local[count][j].busy = SLOT_2HOP;
                            fi_local[count][j].life_time = slot_lifetime_frame_-1;
                            fi_local[count][j].locker = 0;
                        } else  {
                            fi_local[count][j].busy = SLOT_FREE;
                            fi_local[count][j].sti = 0;
                            fi_local[count][j].count_2hop = 0;
                            fi_local[count][j].count_3hop = 0;
                            fi_local[count][j].psf = 0;
                            fi_local[count][j].c3hop_flag = 0;
                            fi_local[count][j].life_time = 0;
                            fi_local[count][j].locker = 1; // lock the status for one frame.
                        }

                    } else if (fi_local[count][j].busy != SLOT_COLLISION
                            && fi_local[count][j].life_time == 0) {

                        fi_local[count][j].busy = SLOT_FREE;
                    }

                    fi_local[count][j].existed = 0;
                //}

            }
        }
    }


    while(processing_fi != NULL){
        merge_fi(this->collected_fi_, processing_fi, this->decision_fi_);
        processing_fi = processing_fi->next_fi;
    }

    if (unlock_flag) {
        for (count=0; count < m_frame_len; count++){
            for(j = 0; j < m_channel_num; j++)
            {
                if (fi_local[count][j].locker && fi_local[count][j].sti == 0) {
                    fi_local[count][j].locker = 0; //the locker must be locked in the last frame.
                }
            }
        }
    }

    //print_slot_status();

}

bool Satmac::adjust_is_needed(int slot_num,int channel_num) {
    slot_tag **fi_collection = this->collected_fi_->slot_describe;
    int i,j,free_count_ths = 0, free_count_ehs = 0;

    int s0_1c_num = 0;

    // 当频率需要调整时，需要改
    for(i=0 ; i < m_frame_len; i++){
        //for(j = 0;j < m_channel_num; j++)
        //{
            if (fi_collection[i][channel_num].busy== SLOT_FREE)
                free_count_ths++;
            if(fi_collection[i][channel_num].busy== SLOT_FREE && fi_collection[i][channel_num].count_3hop == 0) {
                free_count_ehs++;
                if (i < m_frame_len/2)
                    s0_1c_num++;
            }
        //}

    }

    if (adj_ena_ && fi_collection[slot_num][channel_num].count_3hop >= c3hop_threshold_ && free_count_ehs >= adj_free_threshold_) {
        return true;
    } else if (adj_frame_ena_ && slot_num >= m_frame_len/2
            && m_frame_len > adj_frame_lower_bound_
            && (((float)(m_frame_len - free_count_ehs))/m_frame_len) <= frameadj_cut_ratio_ehs_
            && (((float)(m_frame_len - free_count_ths))/m_frame_len) <= frameadj_cut_ratio_ths_
            && s0_1c_num != 0)
        return true;
    else
        return false;
}


void Satmac::adjFrameLen()
{
    if (!adj_frame_ena_)
        return;
    //calculate slot utilization
    int i,j;
    int free_count_ths = 0, free_count_ehs = 0;
    float utilrate_ths, utilrate_ehs;
    bool cutflag = true;
    slot_tag **fi_local_= this->collected_fi_->slot_describe;
    for(i=0 ; i < m_frame_len; i++){
        for(j = 0; j < m_channel_num; j++)
        {
            if(fi_local_[i][j].busy != SLOT_FREE) {
                if (i >= m_frame_len/2)
                    cutflag = false;
            }
            if (fi_local_[i][j].busy== SLOT_FREE)
                free_count_ths++;
            if(fi_local_[i][j].busy== SLOT_FREE && fi_local_[i][j].count_3hop == 0)
                free_count_ehs++;
        }

    }

    utilrate_ths = (float)(m_frame_len - free_count_ths)/m_frame_len;
    utilrate_ehs = (float)(m_frame_len - free_count_ehs)/m_frame_len;
    if (free_count_ths <= adj_free_threshold_)
        utilrate_ths = 1;
    if (free_count_ehs <= adj_free_threshold_)
        utilrate_ehs = 1;

    if (utilrate_ehs >= frameadj_exp_ratio_ && m_frame_len < adj_frame_upper_bound_) {
        m_frame_len *= 2;
    } else if (cutflag
            && utilrate_ths <= frameadj_cut_ratio_ths_
            && utilrate_ehs <= frameadj_cut_ratio_ehs_
            && m_frame_len > adj_frame_lower_bound_) {
        m_frame_len /= 2;
    }

    switch (m_frame_len) {
    case 16:
        adj_free_threshold_ = 3;
        break;
    case 32:
        adj_free_threshold_ = 3;
        break;
    case 64:
        adj_free_threshold_ = 4;
        break;
    case 128:
        adj_free_threshold_ = 5;
        break;
    default:
        adj_free_threshold_ = 5;
    }

}

void Satmac::generate_send_FI_packet(int channelNum){
    slot_tag **fi_local_= this->collected_fi_->slot_describe;
    std::vector<std::vector<slot_tag>> slotTag;
    for(int i = 0; i < m_frame_len; i++)
    {
        std::vector<slot_tag> v;
        for(int j = 0; j < m_channel_num; j++)
        {
            slot_tag stfs;
            stfs.busy = fi_local_[i][j].busy;
            stfs.sti = fi_local_[i][j].sti;
            stfs.count_2hop = fi_local_[i][j].count_2hop;
            stfs.psf = fi_local_[i][j].psf;
            v.push_back(stfs);
        }
        slotTag.push_back(v);
    }

    //SatmacSchedulingGrant* satmacGrant;
    SatmacSchedulingGrant* satmacGrant = new SatmacSchedulingGrant("SatmacGrant");

    satmacGrant->setNumberSubchannels(1);
    int initiailSubchannel = channelNum;
    int finalSubchannel = initiailSubchannel + satmacGrant->getNumSubchannels();
    //int num = satmacGrant->getNumSubchannels();
    RbMap grantedBlocks; //已授权的RBs
    int totalGrantedBlocks = 0;
    for (int i = initiailSubchannel;i < finalSubchannel;i++)
    {
        int initialBand = i * subchannelSize_;
        for (Band b = initialBand; b < initialBand + subchannelSize_ ; b++)
        {
            grantedBlocks[MACRO][b] = 1; //每个 band 上的 RBs 数量为1
            ++totalGrantedBlocks;
        }
    }

    satmacGrant->setPeriod(m_frame_len);
    satmacGrant->setFrameLength(m_frame_len);
    satmacGrant->setGlobleSti(nodeId_);
    satmacGrant->setChannelNumber(m_channel_num);
    satmacGrant->setSlotTag(slotTag);
    satmacGrant->setGrantedBlocks(grantedBlocks);
    satmacGrant->setTotalGrantedBlocks(totalGrantedBlocks);
    satmacGrant->setDirection(D2D_MULTI);
    satmacGrant->setCodewords(1);
    satmacGrant->setStartingSubchannel(initiailSubchannel);
    satmacGrant->setMcs(maxMCSPSSCH_);
    satmacGrant->setExpiration(m_frame_len);

    LteMod mod = _QPSK;
    if (maxMCSPSSCH_ > 9 && maxMCSPSSCH_ < 17)
    {
        mod = _16QAM;
    }
    else if (maxMCSPSSCH_ > 16 && maxMCSPSSCH_ < 29 )
    {
        mod = _64QAM;
    }
    unsigned int i = (mod == _QPSK ? 0 : (mod == _16QAM ? 9 : (mod == _64QAM ? 15 : 0)));
    const unsigned int* tbsVect = itbs2tbs(mod, SINGLE_ANTENNA_PORT0, 1, maxMCSPSSCH_ - i);
    maximumCapacity_ = tbsVect[totalGrantedBlocks-1];
    satmacGrant->setGrantedCwBytes(currentCw_, maximumCapacity_);
    satmacGrant->setGrantedCwBytes((MAX_CODEWORDS - currentCw_), maximumCapacity_);
    // Simply flips the codeword.
    currentCw_ = MAX_CODEWORDS - currentCw_;

    if(dataPktExist == false)
    {
//        if(this->node_state_==NODE_WORK_FI||this->node_state_==NODE_WORK_ADJ)
//        {

//        std::vector<std::vector<slot_tag>> a;
//        std::vector<slot_tag> b1;
//        std::vector<slot_tag> b2;
//        slot_tag stfs;
//        stfs.busy = '0';
//        stfs.sti = 0;
//        stfs.count_2hop = 0;
//        stfs.psf = 0;
//        b1.push_back(stfs);
//        b2.push_back(stfs);
//        a.push_back(b1);
//        a.push_back(b2);
//        satmacGrant->setSlotTag(a);
            satmacGrant->setUserTxParams(preconfiguredTxParams_->dup());
            SatmacSchedulingGrant* phyGrant = satmacGrant->dup();

            UserControlInfo* uinfo = new UserControlInfo();
            uinfo->setSourceId(getMacNodeId());
            uinfo->setDestId(getMacNodeId());
            uinfo->setFrameType(GRANTPKT);
            uinfo->setTxNumber(1);
            uinfo->setDirection(D2D_MULTI);
            uinfo->setUserTxParams(preconfiguredTxParams_->dup());
            uinfo->setSubchannelNumber(satmacGrant->getStartingSubchannel());
            uinfo->setSubchannelLength(satmacGrant->getNumSubchannels());
            //uinfo->setGrantStartTime(satmacGrant->getStartTime());

            phyGrant->setControlInfo(uinfo);
            sendLowerPackets(phyGrant);
            emit(macpktSent, long(1));
//            EV<<"----------------------"<<endl;
//            EV<<"----------------------"<<endl;
//            EV<<"----------------------"<<endl;
//            EV<<"slotHandler"<<endl;
//            EV<<"发送数据包！！！"<<endl;
//            EV<<"----------------------"<<endl;
//            EV<<"----------------------"<<endl;
//            EV<<"----------------------"<<endl;

//        }
        firstTx = false;
        schedulingGrant_ = satmacGrant;
        send_fi_count_++;
        periodCounter_= satmacGrant->getPeriod();
        //periodCounter_= satmacGrant->getPeriod();
        expirationCounter_= periodCounter_;
        emit(grantRequests, 1);
        return;
    }

    schedulingGrant_ = satmacGrant;
    firstTx = true;
    send_fi_count_++;
    periodCounter_= satmacGrant->getPeriod();
    //periodCounter_= satmacGrant->getPeriod();
    expirationCounter_= periodCounter_;
    emit(grantRequests, 1);
}

void Satmac::send_Fi()
{
    SatmacSchedulingGrant* satmacGrant = check_and_cast<SatmacSchedulingGrant*>(schedulingGrant_);
    satmacGrant->setUserTxParams(preconfiguredTxParams_->dup());
    SatmacSchedulingGrant* phyGrant = satmacGrant->dup();
    UserControlInfo* uinfo = new UserControlInfo();
    uinfo->setSourceId(getMacNodeId());
    uinfo->setDestId(getMacNodeId());
    uinfo->setFrameType(GRANTPKT);
    uinfo->setTxNumber(1);
    uinfo->setDirection(D2D_MULTI);
    uinfo->setUserTxParams(preconfiguredTxParams_->dup());
    uinfo->setSubchannelNumber(satmacGrant->getStartingSubchannel());
    uinfo->setSubchannelLength(satmacGrant->getNumSubchannels());
    //uinfo->setGrantStartTime(satmacGrant->getStartTime());

    phyGrant->setControlInfo(uinfo);
    sendLowerPackets(phyGrant);

    EV<<"----------------------"<<endl;
    EV<<"----------------------"<<endl;
    EV<<"----------------------"<<endl;
    EV<<"slotHandler"<<endl;
    EV<<"发送数据包！！！"<<endl;
    EV<<"----------------------"<<endl;
    EV<<"----------------------"<<endl;
    EV<<"----------------------"<<endl;
    firstTx = false;
}

//直接copy
double Satmac::get_channel_utilization()
{
    slot_tag **fi_local_= this->collected_fi_->slot_describe;
    int count = 0;
    for(int i=0 ; i < m_frame_len; i++){
        for(int j = 0;j < m_channel_num; j++)
        {
            if(fi_local_[i][j].busy != SLOT_FREE) {
                count++;
            }
        }

    }
    return ((double)count)/((double)m_frame_len);
}

//#include "ns3/vector.h"
int getdir(inet::Coord cur, inet::Coord last)
{
    double deltax = cur.x - last.x;
    double deltay = cur.y - last.y;
    if (deltax < 1.0 || deltax > -1.0) {
        if (deltay>0) return 1;
        else return 0;
    } else {
        if (deltax>0) return 1;
        else return 0;
    }
    return 1;
}

void
Satmac::slotHandler ()
{
  //NS_LOG_FUNCTION_NOARGS ();

  slot_tag **fi_collection = this->collected_fi_->slot_describe;
  // Restart timer for next slot.
  total_slot_count_ = total_slot_count_+1;
  slot_count_ = total_slot_count_ %  m_frame_len;

  //Simulator::Schedule (GetSlotTime(), &TdmaSatmac::slotHandler, this); 需要放进handleselfmessage中

  m_slotRemainTime = m_slotTime;

  slot_state_ = BEGINING;

  this->fade_received_fi_list(1);

  if (slot_count_ == slot_num_)
  {
      frame_count_++;
//      if (m_start_delay_frames > 0)
//      {
//          --m_start_delay_frames;
//          return;
//      }
      switch (node_state_)
      {
      case NODE_INIT:// the first whole slot of a newly initialized node, it begin to listen
          node_state_ = NODE_LISTEN;
          waiting_frame_count =0;
          request_fail_times = 0;
          collision_count_ = 0;
          continuous_work_fi_ = 0;
          continuous_work_fi_max_ = 0;
          adj_count_success_ = 0;
          adj_count_total_ = 0;
          last_log_time_ = NOW;
          no_avalible_count_ = 0;
          backoff_frame_num_ = 0;
          return;
      case NODE_LISTEN:


          waiting_frame_count++;

          if (backoff_frame_num_) {
//            printf("%d : %d\n",nodeId_,backoff_frame_num_);
              backoff_frame_num_--;
              return;
          }

          //根据自己的fi-local，决定自己要申请的slot，修改自己的slot_num_
          this->clear_FI(this->collected_fi_); //初始化
          fi_collection = this->collected_fi_->slot_describe;
          synthesize_fi_list();

          if(geoHash_NQueens)
          {
              pair_geohash = determine_BCH_GeoHash(0);
              slot_num_ = pair_geohash.first;
              channel_num_ = pair_geohash.second;
          }
          else
              slot_num_ = determine_BCH(0);


          if(slot_num_ < 0||channel_num_<0){
              node_state_ = NODE_LISTEN;
              slot_num_ = slot_count_;
              no_avalible_count_++;
              backoff_frame_num_ = intuniform(0, 20);
#ifdef PRINT_SLOT_STATUS
                printf("I'm node %d, in slot %d, NODE_LISTEN and I cannot choose a BCH!!\n", nodeId_, slot_count_);
#endif
              //NS_LOG_DEBUG("I'm node "<<nodeId_<<", in slot "<<slot_count_<<", NODE_LISTEN and I cannot choose a BCH!!");

              return;
          }
#ifdef PRINT_SLOT_STATUS
            printf("I'm node %d, in slot %d, NODE_LISTEN, choose: %d\n", nodeId_, slot_count_, slot_num_);
#endif
          //NS_LOG_DEBUG("I'm node "<<nodeId_<<", in slot "<<slot_count_<<", NODE_LISTEN, choose: "<<slot_num_);

          //如果正好决定的时隙就是本时隙，那么直接发送
          if(slot_num_ == slot_count_){
              fi_collection[slot_num_][channel_num_].busy = SLOT_1HOP;
              fi_collection[slot_num_][channel_num_].sti = nodeId_;
              fi_collection[slot_num_][channel_num_].count_2hop = 1;
              fi_collection[slot_num_][channel_num_].count_3hop = 1;
              fi_collection[slot_num_][channel_num_].psf = 0;
              generate_send_FI_packet(channel_num_); //必须在BCH状态设置完之后调用。
              node_state_ = NODE_REQUEST;   // 发送完一次 继续等待下一次发送
              return;
          }
          else{//否则等待发送时隙
              node_state_ = NODE_WAIT_REQUEST;  // 选择好了时隙，但还没开始发送，需要等待时隙到来
              return;
          }
          break;
      case NODE_WAIT_REQUEST:
          waiting_frame_count++;
          if (!slot_memory_) {
              this->clear_others_slot_status();
              fi_collection = this->collected_fi_->slot_describe;
          }
          synthesize_fi_list();
          if((fi_collection[slot_count_][channel_num_].sti == nodeId_ && fi_collection[slot_count_][channel_num_].busy == SLOT_1HOP)
                  || fi_collection[slot_count_][channel_num_].sti == 0) {
              fi_collection[slot_count_][channel_num_].busy = SLOT_1HOP;
              fi_collection[slot_count_][channel_num_].sti = nodeId_;
              fi_collection[slot_count_][channel_num_].count_2hop = 1;
              fi_collection[slot_count_][channel_num_].count_3hop = 1;
              fi_collection[slot_count_][channel_num_].psf = 0;
              generate_send_FI_packet(channel_num_);
              node_state_ = NODE_REQUEST;
              return;
          } else {
              if (fi_collection[slot_count_][channel_num_].sti == nodeId_) {
                  fi_collection[slot_count_][channel_num_].busy = SLOT_FREE;
                  fi_collection[slot_count_][channel_num_].sti = 0;
                  fi_collection[slot_count_][channel_num_].count_2hop = 0;
                  fi_collection[slot_count_][channel_num_].count_3hop = 0;
                  fi_collection[slot_count_][channel_num_].psf = 0;
                  fi_collection[slot_count_][channel_num_].locker = 1;
              }
              request_fail_times++;

              emit(accessCollision,1);

              if(geoHash_NQueens)
              {
                  pair_geohash = determine_BCH_GeoHash(0);
                  slot_num_ = pair_geohash.first;
                  channel_num_ = pair_geohash.second;
              }
              else
                  slot_num_ = determine_BCH(0);

              if(slot_num_ < 0 || slot_num_== slot_count_){
                  node_state_ = NODE_LISTEN;
                  no_avalible_count_ ++;
                  backoff_frame_num_ = intuniform(0, 20);
                  slot_num_ = slot_count_;
                  //NS_LOG_DEBUG("I'm node "<<nodeId_<<", in slot "<<slot_count_<<", NODE_WAIT_REQUEST and I cannot choose a BCH!!");
#ifdef PRINT_SLOT_STATUS
                    printf("I'm node %d, in slot %d, NODE_WAIT_REQUEST and I cannot choose a BCH!!\n", nodeId_, slot_count_);
#endif
                  return;
              }
#ifdef PRINT_SLOT_STATUS
                printf("I'm node %d, in slot %d, NODE_WAIT_REQUEST and current bch is unvalid, choose: %d\n", nodeId_, slot_count_, slot_num_);
#endif
              //NS_LOG_DEBUG("I'm node "<<nodeId_<<", in slot "<<slot_count_<<", NODE_WAIT_REQUEST and current bch is unvalid, choose: "<<slot_num_);
              node_state_ = NODE_WAIT_REQUEST;
              return;
          }
          break;
      case NODE_REQUEST:// or node_state_ = NODE_WORK;;
          if (!slot_memory_) {
              this->clear_others_slot_status();
              fi_collection = this->collected_fi_->slot_describe;
          }
          synthesize_fi_list();
          if((fi_collection[slot_count_][channel_num_].sti == nodeId_ && fi_collection[slot_count_][channel_num_].busy == SLOT_1HOP)
                  || fi_collection[slot_count_][channel_num_].sti == 0) {

              node_state_ = NODE_WORK_FI;

              generate_send_FI_packet(channel_num_);
          } else {
              waiting_frame_count++;
              if (fi_collection[slot_count_][channel_num_].sti == nodeId_) { // 在NODE_REQUEST状态下发现冲突，且当前id为本车辆
                  fi_collection[slot_count_][channel_num_].busy = SLOT_FREE;
                  fi_collection[slot_count_][channel_num_].sti = 0;
                  fi_collection[slot_count_][channel_num_].count_2hop = 0;
                  fi_collection[slot_count_][channel_num_].count_3hop = 0;
                  fi_collection[slot_count_][channel_num_].psf = 0;
                  fi_collection[slot_count_][channel_num_].locker = 1;
              }

              request_fail_times++;



              if(geoHash_NQueens)
              {
                  pair_geohash = determine_BCH_GeoHash(0);
                  slot_num_ = pair_geohash.first;
                  channel_num_ = pair_geohash.second;
              }
              else
                  slot_num_ = determine_BCH(0);
              if(slot_num_ < 0 || slot_num_== slot_count_||channel_num_ < 0){
                  node_state_ = NODE_LISTEN;
                  no_avalible_count_ ++;
                  backoff_frame_num_ = intuniform(0, 20);
                  slot_num_ = slot_count_;
#ifdef PRINT_SLOT_STATUS
                    printf("I'm node %d, in slot %d, NODE_REQUEST and I cannot choose a BCH!!\n", nodeId_, slot_count_);
#endif
                  //NS_LOG_DEBUG("I'm node "<<nodeId_<<", in slot "<<slot_count_<<", NODE_REQUEST and I cannot choose a BCH!!");
                  return;
              }
#ifdef PRINT_SLOT_STATUS
                printf("I'm node %d, in slot %d, NODE_REQUEST and current bch is unvalid, choose: %d\n", nodeId_, slot_count_, slot_num_);
#endif
             //NS_LOG_DEBUG("I'm node "<<nodeId_<<", in slot "<<slot_count_<<", NODE_REQUEST and current bch is unvalid, choose: "<<slot_num_);
              node_state_ = NODE_WAIT_REQUEST;
              return;
          }
          break;
      case NODE_WORK_FI:
          if (!slot_memory_) {
              this->clear_others_slot_status();
              fi_collection = this->collected_fi_->slot_describe;
          }
          synthesize_fi_list();

          if((fi_collection[slot_count_][channel_num_].sti == nodeId_ && fi_collection[slot_count_][channel_num_].busy == SLOT_1HOP)
                  || fi_collection[slot_count_][channel_num_].sti == 0)//BCH可用
          {
              continuous_work_fi_ ++;
              continuous_work_fi_max_ = (continuous_work_fi_max_ > continuous_work_fi_)?continuous_work_fi_max_:continuous_work_fi_;
              if (adjust_is_needed(slot_num_,channel_num_)) {

                  if(geoHash_NQueens)
                  {
                      pair_geohash = determine_BCH_GeoHash(1);
                      slot_adj_candidate_ = determine_BCH_GeoHash(1).first;
                      channelNumber_adj = determine_BCH_GeoHash(1).second;

                  }
                  else
                  {
                      slot_adj_candidate_ = determine_BCH(1);
                      channelNumber_adj = 0;
                  }

                  //NS_LOG_DEBUG("I'm node "<<nodeId_<<", in slot "<<slot_count_<<", NODE_WORK_FI ADJ is needed! choose: "<<slot_adj_candidate_);
                  if (slot_adj_candidate_ >= 0 && channelNumber_adj >= 0) {
                      if (adj_single_slot_ena_) {
                          node_state_ = NODE_WORK_FI;
                          adj_count_success_++;
                          slot_num_ = slot_adj_candidate_;
                          fi_collection[slot_count_][channel_num_].busy = SLOT_FREE;
                          fi_collection[slot_count_][channel_num_].sti = 0;
                          fi_collection[slot_count_][channel_num_].count_2hop = 0;
                          fi_collection[slot_count_][channel_num_].count_3hop = 0;
                          fi_collection[slot_count_][channel_num_].psf = 0;
                          fi_collection[slot_count_][channel_num_].locker = 0;

                          channel_num_ = channelNumber_adj;
                          fi_collection[slot_num_][channel_num_].busy = SLOT_1HOP;
                          fi_collection[slot_num_][channel_num_].sti = nodeId_;
                          fi_collection[slot_num_][channel_num_].count_2hop = 1;
                          fi_collection[slot_num_][channel_num_].count_3hop = 1;
                          fi_collection[slot_num_][channel_num_].psf = 0;
                      } else {  // 可能是先占用着两个时隙
                          node_state_ = NODE_WORK_ADJ;
                          adj_count_total_++;
                          fi_collection[slot_adj_candidate_][channelNumber_adj].busy = SLOT_1HOP;
                          fi_collection[slot_adj_candidate_][channelNumber_adj].sti = nodeId_;
                          fi_collection[slot_adj_candidate_][channelNumber_adj].count_2hop = 1;
                          fi_collection[slot_adj_candidate_][channelNumber_adj].count_3hop = 1;
                          fi_collection[slot_adj_candidate_][channelNumber_adj].psf = 0;
                      }
                  }
              } else
                  adjFrameLen();
              generate_send_FI_packet(channel_num_);

              //m_BCHTrace();

              if (random_bch_if_single_switch_ && isSingle(channel_num_)) {
                  if (bch_slot_lock_-- == 0) {

                      if(geoHash_NQueens)
                      {
                          pair_geohash = determine_BCH_GeoHash(0);
                          slot_num_ = pair_geohash.first;
                      }
                      else
                          slot_num_ = determine_BCH(0);
                      fi_collection[slot_count_][channel_num_].busy = SLOT_FREE;
                      fi_collection[slot_count_][channel_num_].sti = 0;
                      fi_collection[slot_count_][channel_num_].count_2hop = 0;
                      fi_collection[slot_count_][channel_num_].count_3hop = 0;
                      fi_collection[slot_count_][channel_num_].psf = 0;
                      fi_collection[slot_count_][channel_num_].locker = 1;

                      if(geoHash_NQueens)
                      {
                          channel_num_ = pair_geohash.second;
                      }

                      fi_collection[slot_num_][channel_num_].busy = SLOT_1HOP;
                      fi_collection[slot_num_][channel_num_].sti = nodeId_;
                      fi_collection[slot_num_][channel_num_].count_2hop = 1;
                      fi_collection[slot_num_][channel_num_].count_3hop = 1;
                      fi_collection[slot_num_][channel_num_].psf = 0;

                      bch_slot_lock_ = 5;
                  }
              } else
                  bch_slot_lock_ = 5;

          } else {
              continuous_work_fi_ = 0;
              if (fi_collection[slot_count_][channel_num_].sti == nodeId_) {
                  fi_collection[slot_count_][channel_num_].busy = SLOT_FREE;
                  fi_collection[slot_count_][channel_num_].sti = 0;
                  fi_collection[slot_count_][channel_num_].count_2hop = 0;
                  fi_collection[slot_count_][channel_num_].count_3hop = 0;
                  fi_collection[slot_count_][channel_num_].psf = 0;
                  fi_collection[slot_count_][channel_num_].locker = 1;
              }

              collision_count_++;

              emit(accessCollision,1);

              if(geoHash_NQueens)
              {
                  pair_geohash = determine_BCH_GeoHash(0);
                  slot_num_ = pair_geohash.first;
                  channel_num_ = pair_geohash.second;
              }
              else
                  slot_num_ = determine_BCH(0);
              if(slot_num_ < 0 || slot_num_== slot_count_){
                  node_state_ = NODE_LISTEN;
                  slot_num_ = slot_count_;
                  no_avalible_count_ ++;
                  backoff_frame_num_ = intuniform(0, 20);
                  //NS_LOG_DEBUG("I'm node "<<nodeId_<<", in slot "<<slot_count_<<", NODE_WORK_FI and I cannot choose a BCH!!");
                  return;
              }

              //NS_LOG_DEBUG("I'm node "<<nodeId_<<", in slot "<<slot_count_<<", NODE_WORK_FI and current bch is unvalid, choose: "<<slot_num_);
              node_state_ = NODE_WAIT_REQUEST;
              return;
          }
          break;
      case NODE_WORK_ADJ:   // 此时该节点处于需要调整时隙的阶段（调整未完成）
          if (!slot_memory_) {
              this->clear_others_slot_status();
              fi_collection = this->collected_fi_->slot_describe;
          }
          synthesize_fi_list();
          if((fi_collection[slot_count_][channel_num_].sti == nodeId_ && fi_collection[slot_count_][channel_num_].busy == SLOT_1HOP)
                  || fi_collection[slot_count_][channel_num_].sti == 0)//BCH依然可用
          {
              if ((fi_collection[slot_count_][channel_num_].count_3hop >= fi_collection[slot_adj_candidate_][channelNumber_adj].count_3hop)
                      && ((fi_collection[slot_adj_candidate_][channelNumber_adj].sti == nodeId_ && fi_collection[slot_adj_candidate_][channelNumber_adj].busy == SLOT_1HOP)
                              || fi_collection[slot_adj_candidate_][channelNumber_adj].sti == 0)) //ADJ时隙可用)
              {     // 调整到新的时隙中去
                  int oldbch = slot_num_;
                  int oldChannelNum = channel_num_;
                  node_state_ = NODE_WORK_FI;
                  slot_num_ = slot_adj_candidate_;
                  channel_num_ = channelNumber_adj;
                  adj_count_success_++;
                  fi_collection[oldbch][oldChannelNum].busy = SLOT_FREE;    // 释放原来所占用的时隙资源
                  fi_collection[oldbch][oldChannelNum].sti = 0;
                  fi_collection[oldbch][oldChannelNum].count_2hop = 0;
                  fi_collection[oldbch][oldChannelNum].count_3hop = 0;
                  fi_collection[oldbch][oldChannelNum].psf = 0;
                  fi_collection[oldbch][oldChannelNum].locker = 1;
              } else {  // 新的时隙不可用，还是占用原来的时隙
                  node_state_ = NODE_WORK_FI;
                  fi_collection[slot_adj_candidate_][channelNumber_adj].busy = SLOT_FREE;   // 释放新申请的时隙资源
                  fi_collection[slot_adj_candidate_][channelNumber_adj].sti = 0;
                  fi_collection[slot_adj_candidate_][channelNumber_adj].count_2hop = 0;
                  fi_collection[slot_adj_candidate_][channelNumber_adj].count_3hop = 0;
                  fi_collection[slot_adj_candidate_][channelNumber_adj].psf = 0;
                  fi_collection[slot_adj_candidate_][channelNumber_adj].locker = 1;
              }

              generate_send_FI_packet(channel_num_);
          } else { //BCH已经不可用

              if((fi_collection[slot_adj_candidate_][channelNumber_adj].sti == nodeId_ && fi_collection[slot_adj_candidate_][channelNumber_adj].busy == SLOT_1HOP)
                      || fi_collection[slot_adj_candidate_][channelNumber_adj].sti == 0) { //ADJ时隙可用
                  node_state_ = NODE_WORK_FI;
                  adj_count_success_++;
                  slot_num_ = slot_adj_candidate_;
                  channel_num_ = channelNumber_adj;
              } else { //ADJ时隙不可用
                  collision_count_++;
                  emit(accessCollision,1);
                  if (fi_collection[slot_adj_candidate_][channelNumber_adj].sti == nodeId_) {
                      fi_collection[slot_adj_candidate_][channelNumber_adj].busy = SLOT_FREE;
                      fi_collection[slot_adj_candidate_][channelNumber_adj].sti = 0;
                      fi_collection[slot_adj_candidate_][channelNumber_adj].count_2hop = 0;
                      fi_collection[slot_adj_candidate_][channelNumber_adj].count_3hop = 0;
                      fi_collection[slot_adj_candidate_][channelNumber_adj].psf = 0;
                      fi_collection[slot_adj_candidate_][channelNumber_adj].locker = 1;
                  }
                  if(geoHash_NQueens)
                  {
                      pair_geohash = determine_BCH_GeoHash(0);
                      slot_num_ = pair_geohash.first;
                      channel_num_ = pair_geohash.second;
                  }
                  else
                      slot_num_ = determine_BCH(0);
                  if(slot_num_ < 0 || slot_num_== slot_count_){
                      node_state_ = NODE_LISTEN;
                      slot_num_ = slot_count_;
                      no_avalible_count_ ++;
                      backoff_frame_num_ = intuniform(0, 20);
                      return;
                  } else {
                      node_state_ = NODE_WAIT_REQUEST;
                      return;
                  }
              }
          }

          slot_adj_candidate_ = -1; //
          break;
      }
  }
  return;
}

int Satmac::selectSubChannel()
{
//    GeoHashBits hash, fast_hash;
//
//    GeoHashRange lat_range, lon_range;
//    lat_range.max = 90.0;
//    lat_range.min = -90.0;
//    lon_range.max = 180.0;
//    lon_range.min = -180.0;
//    // 每隔1ms向物理层发送需要坐标的数据包
//    AskCoordinate* askCoordinate = new AskCoordinate("askCoordinate");
//    UserControlInfo* uinfo = new UserControlInfo();
//    uinfo->setSourceId(getMacNodeId());
//    uinfo->setDestId(getMacNodeId());
//    uinfo->setFrameType(ASKCOORDINATEPKT);
//    askCoordinate->setControlInfo(uinfo);
//    double xx,yy;
//    xx = this->x;
//    yy = this->y;
//
//    LteMacBase::send(askCoordinate,down_[OUT]);
//
//    double latitude = this->x;
//    double longitude = this->y;
//
//    int step = 7;
//    geohash_encode(lat_range, lon_range, latitude, longitude, step, &hash);

    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    EV << "当前节点需要选择子信道！！！" << endl;
    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    EV << "------------------------" << endl;
    int64_t hashBitResult;
    mbr::MbrSumo* mbrSumo = mbr::MbrSumo::GetInstance();

    double x = this->x;
    double y = this->y;

    mbrSumo->sumoCartesian2GPS(this->x, this->y, &x, &y);

    EV << "当前节点的坐标：" << "(" << this->x << "," << this->y << ")" <<endl;
    EV << "当前节点的GPS坐标：" << "(" << x << "," << y << ")" <<endl;

    hashBitResult = mbrSumo->sumoCartesian2Geohash(this->x,this->y);
    string binary_result = dec2bin(hashBitResult);
    EV << "当前节点的二进制geohash：" << binary_result <<endl;

    reverse(binary_result.begin(),binary_result.end());

    string result = binary_result.substr(0, 6);

    reverse(result.begin(),result.end());

    int num = -1;
    if(result.compare("010101") == 0 ||result.compare("111100") == 0 ||result.compare("011011") == 0 ||result.compare("110010") == 0
            ||result.compare("101111") == 0 ||result.compare("000110") == 0 ||result.compare("100001") == 0 ||result.compare("001000") == 0 )
    {
        num = 0;
    }
    if(result.compare("111111") == 0 ||result.compare("010110") == 0 ||result.compare("110001") == 0 ||result.compare("011000") == 0
            ||result.compare("000101") == 0 ||result.compare("101100") == 0 ||result.compare("001011") == 0 ||result.compare("100010") == 0 )
    {
        num = 1;
    }
    if(result.compare("110111") == 0 ||result.compare("011110") == 0 ||result.compare("111001") == 0 ||result.compare("010000") == 0
            ||result.compare("001101") == 0 ||result.compare("100100") == 0 ||result.compare("000011") == 0 ||result.compare("101010") == 0 )
    {
        num = 2;
    }
    if(result.compare("011101") == 0 ||result.compare("110100") == 0 ||result.compare("010011") == 0 ||result.compare("111010") == 0
            ||result.compare("100111") == 0 ||result.compare("001110") == 0 ||result.compare("101001") == 0 ||result.compare("000000") == 0 )
    {
        num = 3;
    }
    if(result.compare("010111") == 0 ||result.compare("110110") == 0 ||result.compare("010001") == 0 ||result.compare("111000") == 0
            ||result.compare("001111") == 0 ||result.compare("101110") == 0 ||result.compare("001001") == 0 ||result.compare("100000") == 0 )
    {
        num = 4;
    }
    if(result.compare("111101") == 0 ||result.compare("011100") == 0 ||result.compare("111011") == 0 ||result.compare("010010") == 0
            ||result.compare("100101") == 0 ||result.compare("000100") == 0 ||result.compare("100011") == 0 ||result.compare("001010") == 0 )
    {
        num = 5;
    }
    if(result.compare("011111") == 0 ||result.compare("010100") == 0 ||result.compare("011001") == 0 ||result.compare("110000") == 0
            ||result.compare("101101") == 0 ||result.compare("001100") == 0 ||result.compare("101011") == 0 ||result.compare("000010") == 0 )
    {
        num = 6;
    }
    if(result.compare("110101") == 0 ||result.compare("111110") == 0 ||result.compare("110011") == 0 ||result.compare("011010") == 0
            ||result.compare("000111") == 0 ||result.compare("100110") == 0 ||result.compare("000001") == 0 ||result.compare("101000") == 0 )
    {
        num = 7;
    }


    return num;
//    int num1 = m_channel_num % 4;
//    int num2;
//    int* arr = new int[num1];
//    //delete
//    if(result.compare("0101")||result.compare("0100")||result.compare("0001")||result.compare("0000"))
//    {
//        num2 = 0;
//        for(int i = 0; i < num1; i++)
//        {
//            arr[i] = num2;
//            num2 += 4;
//        }
//    }
//    else if(result.compare("0111")||result.compare("0110")||result.compare("0011")||result.compare("0010"))
//    {
//        num2 = 1;
//        for(int i = 0; i < num1; i++)
//        {
//            arr[i] = num2;
//            num2 += 4;
//        }
//    }
//    else if(result.compare("1101")||result.compare("1100")||result.compare("1001")||result.compare("1000"))
//    {
//        num2 = 2;
//        for(int i = 0; i < num1; i++)
//        {
//            arr[i] = num2;
//            num2 += 4;
//        }
//    }
//    else if(result.compare("1111")||result.compare("1110")||result.compare("1011")||result.compare("1010"))
//    {
//        num2 = 3;
//        for(int i = 0; i < num1; i++)
//        {
//            arr[i] = num2;
//            num2 += 4;
//        }
//    }
//    return arr;
}


void Satmac::finish()
{
    binder_->removeUeInfo(ueInfo_);

    delete preconfiguredTxParams_;
    delete ueInfo_;
}


