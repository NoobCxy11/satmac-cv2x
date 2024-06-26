//
// Generated file, do not edit! Created by nedtool 5.5 from apps/vod/VoDPacket.msg.
//

#ifndef __VODPACKET_M_H
#define __VODPACKET_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0505
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



/**
 * Class generated from <tt>apps/vod/VoDPacket.msg:13</tt> by nedtool.
 * <pre>
 * //
 * // Packet used by the video-on-demand application (~VoDUDPServer, ~VoDUDPClient)
 * //
 * packet VoDPacket
 * {
 *     int frameSeqNum;
 *     int frameLength;
 *     // temporal scalability layer
 *     int tid;
 *     // SNR scalability layer
 *     int qid;
 * }
 * </pre>
 */
class VoDPacket : public ::omnetpp::cPacket
{
  protected:
    int frameSeqNum;
    int frameLength;
    int tid;
    int qid;

  private:
    void copy(const VoDPacket& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const VoDPacket&);

  public:
    VoDPacket(const char *name=nullptr, short kind=0);
    VoDPacket(const VoDPacket& other);
    virtual ~VoDPacket();
    VoDPacket& operator=(const VoDPacket& other);
    virtual VoDPacket *dup() const override {return new VoDPacket(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual int getFrameSeqNum() const;
    virtual void setFrameSeqNum(int frameSeqNum);
    virtual int getFrameLength() const;
    virtual void setFrameLength(int frameLength);
    virtual int getTid() const;
    virtual void setTid(int tid);
    virtual int getQid() const;
    virtual void setQid(int qid);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const VoDPacket& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, VoDPacket& obj) {obj.parsimUnpack(b);}


#endif // ifndef __VODPACKET_M_H

