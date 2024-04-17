//
// Generated file, do not edit! Created by nedtool 5.5 from stack/phy/packet/SpsCandidateResources.msg.
//

#ifndef __SPSCANDIDATERESOURCES_M_H
#define __SPSCANDIDATERESOURCES_M_H

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
 * Class generated from <tt>stack/phy/packet/SpsCandidateResources.msg:19</tt> by nedtool.
 * <pre>
 * //
 * // TODO generated message class
 * //
 * packet SpsCandidateResources
 * {
 *     \@customize(true);
 * }
 * </pre>
 *
 * SpsCandidateResources_Base is only useful if it gets subclassed, and SpsCandidateResources is derived from it.
 * The minimum code to be written for SpsCandidateResources is the following:
 *
 * <pre>
 * class SpsCandidateResources : public SpsCandidateResources_Base
 * {
 *   private:
 *     void copy(const SpsCandidateResources& other) { ... }

 *   public:
 *     SpsCandidateResources(const char *name=nullptr, short kind=0) : SpsCandidateResources_Base(name,kind) {}
 *     SpsCandidateResources(const SpsCandidateResources& other) : SpsCandidateResources_Base(other) {copy(other);}
 *     SpsCandidateResources& operator=(const SpsCandidateResources& other) {if (this==&other) return *this; SpsCandidateResources_Base::operator=(other); copy(other); return *this;}
 *     virtual SpsCandidateResources *dup() const override {return new SpsCandidateResources(*this);}
 *     // ADD CODE HERE to redefine and implement pure virtual functions from SpsCandidateResources_Base
 * };
 * </pre>
 *
 * The following should go into a .cc (.cpp) file:
 *
 * <pre>
 * Register_Class(SpsCandidateResources)
 * </pre>
 */
class SpsCandidateResources_Base : public ::omnetpp::cPacket
{
  protected:

  private:
    void copy(const SpsCandidateResources_Base& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const SpsCandidateResources_Base&);
    // make constructors protected to avoid instantiation
    SpsCandidateResources_Base(const char *name=nullptr, short kind=0);
    SpsCandidateResources_Base(const SpsCandidateResources_Base& other);
    // make assignment operator protected to force the user override it
    SpsCandidateResources_Base& operator=(const SpsCandidateResources_Base& other);

  public:
    virtual ~SpsCandidateResources_Base();
    virtual SpsCandidateResources_Base *dup() const override {throw omnetpp::cRuntimeError("You forgot to manually add a dup() function to class SpsCandidateResources");}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
};


#endif // ifndef __SPSCANDIDATERESOURCES_M_H

