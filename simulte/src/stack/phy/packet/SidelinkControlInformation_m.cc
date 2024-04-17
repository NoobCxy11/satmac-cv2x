//
// Generated file, do not edit! Created by nedtool 5.5 from stack/phy/packet/SidelinkControlInformation.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wshadow"
#  pragma clang diagnostic ignored "-Wconversion"
#  pragma clang diagnostic ignored "-Wunused-parameter"
#  pragma clang diagnostic ignored "-Wc++98-compat"
#  pragma clang diagnostic ignored "-Wunreachable-code-break"
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wshadow"
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#  pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#  pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <iostream>
#include <sstream>
#include "SidelinkControlInformation_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp


// forward
template<typename T, typename A>
std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec);

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// operator<< for std::vector<T>
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

Register_Class(SidelinkControlInformation)

SidelinkControlInformation::SidelinkControlInformation(const char *name, short kind) : ::omnetpp::cPacket(name,kind)
{
    this->priority = 0;
    this->resourceReservationInterval = 0;
    this->frequencyResourceLocation = 0;
    this->timeGapRetrans = 0;
    this->mcs = 0;
    this->oneShotLocation = 0;
    this->retransmissionIndex = 0;
}

SidelinkControlInformation::SidelinkControlInformation(const SidelinkControlInformation& other) : ::omnetpp::cPacket(other)
{
    copy(other);
}

SidelinkControlInformation::~SidelinkControlInformation()
{
}

SidelinkControlInformation& SidelinkControlInformation::operator=(const SidelinkControlInformation& other)
{
    if (this==&other) return *this;
    ::omnetpp::cPacket::operator=(other);
    copy(other);
    return *this;
}

void SidelinkControlInformation::copy(const SidelinkControlInformation& other)
{
    this->priority = other.priority;
    this->resourceReservationInterval = other.resourceReservationInterval;
    this->frequencyResourceLocation = other.frequencyResourceLocation;
    this->timeGapRetrans = other.timeGapRetrans;
    this->mcs = other.mcs;
    this->oneShotLocation = other.oneShotLocation;
    this->retransmissionIndex = other.retransmissionIndex;
}

void SidelinkControlInformation::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cPacket::parsimPack(b);
    doParsimPacking(b,this->priority);
    doParsimPacking(b,this->resourceReservationInterval);
    doParsimPacking(b,this->frequencyResourceLocation);
    doParsimPacking(b,this->timeGapRetrans);
    doParsimPacking(b,this->mcs);
    doParsimPacking(b,this->oneShotLocation);
    doParsimPacking(b,this->retransmissionIndex);
}

void SidelinkControlInformation::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cPacket::parsimUnpack(b);
    doParsimUnpacking(b,this->priority);
    doParsimUnpacking(b,this->resourceReservationInterval);
    doParsimUnpacking(b,this->frequencyResourceLocation);
    doParsimUnpacking(b,this->timeGapRetrans);
    doParsimUnpacking(b,this->mcs);
    doParsimUnpacking(b,this->oneShotLocation);
    doParsimUnpacking(b,this->retransmissionIndex);
}

unsigned int SidelinkControlInformation::getPriority() const
{
    return this->priority;
}

void SidelinkControlInformation::setPriority(unsigned int priority)
{
    this->priority = priority;
}

unsigned int SidelinkControlInformation::getResourceReservationInterval() const
{
    return this->resourceReservationInterval;
}

void SidelinkControlInformation::setResourceReservationInterval(unsigned int resourceReservationInterval)
{
    this->resourceReservationInterval = resourceReservationInterval;
}

unsigned int SidelinkControlInformation::getFrequencyResourceLocation() const
{
    return this->frequencyResourceLocation;
}

void SidelinkControlInformation::setFrequencyResourceLocation(unsigned int frequencyResourceLocation)
{
    this->frequencyResourceLocation = frequencyResourceLocation;
}

unsigned int SidelinkControlInformation::getTimeGapRetrans() const
{
    return this->timeGapRetrans;
}

void SidelinkControlInformation::setTimeGapRetrans(unsigned int timeGapRetrans)
{
    this->timeGapRetrans = timeGapRetrans;
}

unsigned int SidelinkControlInformation::getMcs() const
{
    return this->mcs;
}

void SidelinkControlInformation::setMcs(unsigned int mcs)
{
    this->mcs = mcs;
}

unsigned int SidelinkControlInformation::getOneShotLocation() const
{
    return this->oneShotLocation;
}

void SidelinkControlInformation::setOneShotLocation(unsigned int oneShotLocation)
{
    this->oneShotLocation = oneShotLocation;
}

unsigned int SidelinkControlInformation::getRetransmissionIndex() const
{
    return this->retransmissionIndex;
}

void SidelinkControlInformation::setRetransmissionIndex(unsigned int retransmissionIndex)
{
    this->retransmissionIndex = retransmissionIndex;
}

class SidelinkControlInformationDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    SidelinkControlInformationDescriptor();
    virtual ~SidelinkControlInformationDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyname) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyname) const override;
    virtual int getFieldArraySize(void *object, int field) const override;

    virtual const char *getFieldDynamicTypeString(void *object, int field, int i) const override;
    virtual std::string getFieldValueAsString(void *object, int field, int i) const override;
    virtual bool setFieldValueAsString(void *object, int field, int i, const char *value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual void *getFieldStructValuePointer(void *object, int field, int i) const override;
};

Register_ClassDescriptor(SidelinkControlInformationDescriptor)

SidelinkControlInformationDescriptor::SidelinkControlInformationDescriptor() : omnetpp::cClassDescriptor("SidelinkControlInformation", "omnetpp::cPacket")
{
    propertynames = nullptr;
}

SidelinkControlInformationDescriptor::~SidelinkControlInformationDescriptor()
{
    delete[] propertynames;
}

bool SidelinkControlInformationDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<SidelinkControlInformation *>(obj)!=nullptr;
}

const char **SidelinkControlInformationDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *SidelinkControlInformationDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int SidelinkControlInformationDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 7+basedesc->getFieldCount() : 7;
}

unsigned int SidelinkControlInformationDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeFlags(field);
        field -= basedesc->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<7) ? fieldTypeFlags[field] : 0;
}

const char *SidelinkControlInformationDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "priority",
        "resourceReservationInterval",
        "frequencyResourceLocation",
        "timeGapRetrans",
        "mcs",
        "oneShotLocation",
        "retransmissionIndex",
    };
    return (field>=0 && field<7) ? fieldNames[field] : nullptr;
}

int SidelinkControlInformationDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='p' && strcmp(fieldName, "priority")==0) return base+0;
    if (fieldName[0]=='r' && strcmp(fieldName, "resourceReservationInterval")==0) return base+1;
    if (fieldName[0]=='f' && strcmp(fieldName, "frequencyResourceLocation")==0) return base+2;
    if (fieldName[0]=='t' && strcmp(fieldName, "timeGapRetrans")==0) return base+3;
    if (fieldName[0]=='m' && strcmp(fieldName, "mcs")==0) return base+4;
    if (fieldName[0]=='o' && strcmp(fieldName, "oneShotLocation")==0) return base+5;
    if (fieldName[0]=='r' && strcmp(fieldName, "retransmissionIndex")==0) return base+6;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *SidelinkControlInformationDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "unsigned int",
        "unsigned int",
        "unsigned int",
        "unsigned int",
        "unsigned int",
        "unsigned int",
        "unsigned int",
    };
    return (field>=0 && field<7) ? fieldTypeStrings[field] : nullptr;
}

const char **SidelinkControlInformationDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldPropertyNames(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *SidelinkControlInformationDescriptor::getFieldProperty(int field, const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldProperty(field, propertyname);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int SidelinkControlInformationDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    SidelinkControlInformation *pp = (SidelinkControlInformation *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

const char *SidelinkControlInformationDescriptor::getFieldDynamicTypeString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldDynamicTypeString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    SidelinkControlInformation *pp = (SidelinkControlInformation *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string SidelinkControlInformationDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    SidelinkControlInformation *pp = (SidelinkControlInformation *)object; (void)pp;
    switch (field) {
        case 0: return ulong2string(pp->getPriority());
        case 1: return ulong2string(pp->getResourceReservationInterval());
        case 2: return ulong2string(pp->getFrequencyResourceLocation());
        case 3: return ulong2string(pp->getTimeGapRetrans());
        case 4: return ulong2string(pp->getMcs());
        case 5: return ulong2string(pp->getOneShotLocation());
        case 6: return ulong2string(pp->getRetransmissionIndex());
        default: return "";
    }
}

bool SidelinkControlInformationDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    SidelinkControlInformation *pp = (SidelinkControlInformation *)object; (void)pp;
    switch (field) {
        case 0: pp->setPriority(string2ulong(value)); return true;
        case 1: pp->setResourceReservationInterval(string2ulong(value)); return true;
        case 2: pp->setFrequencyResourceLocation(string2ulong(value)); return true;
        case 3: pp->setTimeGapRetrans(string2ulong(value)); return true;
        case 4: pp->setMcs(string2ulong(value)); return true;
        case 5: pp->setOneShotLocation(string2ulong(value)); return true;
        case 6: pp->setRetransmissionIndex(string2ulong(value)); return true;
        default: return false;
    }
}

const char *SidelinkControlInformationDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

void *SidelinkControlInformationDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    SidelinkControlInformation *pp = (SidelinkControlInformation *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}


