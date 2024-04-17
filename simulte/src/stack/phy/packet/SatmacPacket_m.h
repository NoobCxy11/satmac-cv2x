//
// Generated file, do not edit! Created by nedtool 5.5 from stack/phy/packet/SatmacPacket.msg.
//

#ifndef __SATMACPACKET_M_H
#define __SATMACPACKET_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0505
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
#include <vector>
#include "common/SatmacCommon.h"
#include "stack/phy/packet/AirFrame_m.h"
typedef std::vector<std::vector<SlotTagForSending>> SlotTag;
// }}

/**
 * Class generated from <tt>stack/phy/packet/SatmacPacket.msg:29</tt> by nedtool.
 * <pre>
 * packet SatmacPacket extends AirFrame
 * {
 *     int frameLength;
 *     int channelNumber;
 *     int globleSti;
 *     SlotTag slotTag;
 * }
 * </pre>
 */
class SatmacPacket : public ::AirFrame
{
  protected:
    int frameLength;
    int channelNumber;
    int globleSti;
    SlotTag slotTag;

  private:
    void copy(const SatmacPacket& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const SatmacPacket&);

  public:
    SatmacPacket(const char *name=nullptr, short kind=0);
    SatmacPacket(const SatmacPacket& other);
    virtual ~SatmacPacket();
    SatmacPacket& operator=(const SatmacPacket& other);
    virtual SatmacPacket *dup() const override {return new SatmacPacket(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual int getFrameLength() const;
    virtual void setFrameLength(int frameLength);
    virtual int getChannelNumber() const;
    virtual void setChannelNumber(int channelNumber);
    virtual int getGlobleSti() const;
    virtual void setGlobleSti(int globleSti);
    virtual SlotTag& getSlotTag();
    virtual const SlotTag& getSlotTag() const {return const_cast<SatmacPacket*>(this)->getSlotTag();}
    virtual void setSlotTag(const SlotTag& slotTag);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const SatmacPacket& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, SatmacPacket& obj) {obj.parsimUnpack(b);}


#endif // ifndef __SATMACPACKET_M_H

