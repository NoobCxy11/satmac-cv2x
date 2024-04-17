//
// Generated file, do not edit! Created by nedtool 5.5 from common/timer/TMultiTimerMsg.msg.
//

#ifndef __TMULTITIMERMSG_M_H
#define __TMULTITIMERMSG_M_H

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
#include "common/timer/TTimerMsg_m.h"
// }}

/**
 * Class generated from <tt>common/timer/TMultiTimerMsg.msg:17</tt> by nedtool.
 * <pre>
 * packet TMultiTimerMsg extends TTimerMsg
 * {
 *     unsigned int event;
 * }
 * </pre>
 */
class TMultiTimerMsg : public ::TTimerMsg
{
  protected:
    unsigned int event;

  private:
    void copy(const TMultiTimerMsg& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const TMultiTimerMsg&);

  public:
    TMultiTimerMsg(const char *name=nullptr, short kind=0);
    TMultiTimerMsg(const TMultiTimerMsg& other);
    virtual ~TMultiTimerMsg();
    TMultiTimerMsg& operator=(const TMultiTimerMsg& other);
    virtual TMultiTimerMsg *dup() const override {return new TMultiTimerMsg(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual unsigned int getEvent() const;
    virtual void setEvent(unsigned int event);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const TMultiTimerMsg& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, TMultiTimerMsg& obj) {obj.parsimUnpack(b);}


#endif // ifndef __TMULTITIMERMSG_M_H

