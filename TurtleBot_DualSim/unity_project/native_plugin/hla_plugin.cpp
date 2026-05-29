/*
 * hla_plugin.cpp — Plugin nativo HLA para Unity (DualSim)
 *
 * Implementa la interfaz C exportada que HlaInterface.cs consume
 * vía P/Invoke. Usa el API C++ de IEEE 1516-2010 (Pitch pRTI).
 *
 * Compilar:
 *   g++ -std=c++17 -shared -fPIC -o libhla_plugin.so hla_plugin.cpp \
 *       -I/home/vicen/prti1516e/api/cpp/HLA_1516-2010 \
 *       -L/home/vicen/prti1516e/lib/gcc73_64 \
 *       -lrti1516e64 -lfedtime1516e64 \
 *       -Wl,-rpath,/home/vicen/prti1516e/lib/gcc73_64
 */

#include <RTI/NullFederateAmbassador.h>
#include <RTI/RTI1516.h>
#include <RTI/time/HLAfloat64TimeFactory.h>
#include <RTI/time/HLAfloat64Time.h>
#include <RTI/time/HLAfloat64Interval.h>

#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <algorithm>

using namespace rti1516e;

// ─── Exported struct (must match C# BoxData) ─────────────────────
#pragma pack(push, 1)
struct BoxData {
    int32_t id;
    float positionX;
    float positionY;
    float rotationY;
    float positionZ;
};
#pragma pack(pop)

// ─── Helper: string ↔ wstring ────────────────────────────────────
static std::wstring toWide(const std::string& s) {
    return std::wstring(s.begin(), s.end());
}

// ─── Helper: float32LE encode/decode ─────────────────────────────
static VariableLengthData encodeFloat(float v) {
    // float32 little-endian
    uint8_t buf[4];
    std::memcpy(buf, &v, 4);
    return VariableLengthData(buf, 4);
}

static float decodeFloat(const VariableLengthData& vld) {
    float v = 0.0f;
    if (vld.size() >= 4) {
        std::memcpy(&v, vld.data(), 4);
    }
    return v;
}

// ─── Federate Ambassador ─────────────────────────────────────────
class DualSimAmbassador : public NullFederateAmbassador {
public:
    // Object class & attributes
    ObjectClassHandle boxClassHandle;
    AttributeHandle attrPositionX;
    AttributeHandle attrPositionY;
    AttributeHandle attrRotationY;
    AttributeHandle attrPositionZ;

    // Discovered remote objects: instanceHandle → BoxData
    std::map<ObjectInstanceHandle, BoxData> remoteBoxes;
    std::mutex dataMutex;

    // Our own instance
    ObjectInstanceHandle myInstanceHandle;
    int myInstanceId = -1;
    bool instanceRegistered = false;

    // Object name reservation
    bool reservationComplete = false;
    bool reservationSucceeded = false;

    // Time management
    bool isTimeRegulating = false;
    bool isTimeConstrained = false;
    bool timeAdvanceGranted = false;
    double currentLogicalTime = 0.0;
    double lookahead = 0.05;

    // Instance counter
    int nextId = 1;

    // ─── Callbacks ─────────────────────────────────────

    void objectInstanceNameReservationSucceeded(
        const std::wstring& name) override
    {
        reservationComplete = true;
        reservationSucceeded = true;
    }

    void objectInstanceNameReservationFailed(
        const std::wstring& name) override
    {
        reservationComplete = true;
        reservationSucceeded = false;
    }

    void discoverObjectInstance(
        ObjectInstanceHandle handle,
        ObjectClassHandle classHandle,
        const std::wstring& name) override
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (remoteBoxes.find(handle) == remoteBoxes.end()) {
            BoxData b{};
            b.id = nextId++;
            remoteBoxes[handle] = b;
        }
    }

    void discoverObjectInstance(
        ObjectInstanceHandle handle,
        ObjectClassHandle classHandle,
        const std::wstring& name,
        FederateHandle producingFederate) override
    {
        discoverObjectInstance(handle, classHandle, name);
    }

    void reflectAttributeValues(
        ObjectInstanceHandle handle,
        const AttributeHandleValueMap& values,
        const VariableLengthData& tag,
        OrderType sentOrder,
        TransportationType transport,
        SupplementalReflectInfo info) override
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        auto it = remoteBoxes.find(handle);
        if (it == remoteBoxes.end()) {
            BoxData b{};
            b.id = nextId++;
            remoteBoxes[handle] = b;
            it = remoteBoxes.find(handle);
        }

        BoxData& box = it->second;
        for (auto& kv : values) {
            if (kv.first == attrPositionX) box.positionX = decodeFloat(kv.second);
            else if (kv.first == attrPositionY) box.positionY = decodeFloat(kv.second);
            else if (kv.first == attrRotationY) box.rotationY = decodeFloat(kv.second);
            else if (kv.first == attrPositionZ) box.positionZ = decodeFloat(kv.second);
        }
    }

    void reflectAttributeValues(
        ObjectInstanceHandle handle,
        const AttributeHandleValueMap& values,
        const VariableLengthData& tag,
        OrderType sentOrder,
        TransportationType transport,
        LogicalTime const& time,
        OrderType receivedOrder,
        SupplementalReflectInfo info) override
    {
        // Delegate to non-TSO version
        reflectAttributeValues(handle, values, tag, sentOrder, transport, info);
    }

    void reflectAttributeValues(
        ObjectInstanceHandle handle,
        const AttributeHandleValueMap& values,
        const VariableLengthData& tag,
        OrderType sentOrder,
        TransportationType transport,
        LogicalTime const& time,
        OrderType receivedOrder,
        MessageRetractionHandle retraction,
        SupplementalReflectInfo info) override
    {
        reflectAttributeValues(handle, values, tag, sentOrder, transport, info);
    }

    void removeObjectInstance(
        ObjectInstanceHandle handle,
        const VariableLengthData& tag,
        OrderType sentOrder,
        SupplementalRemoveInfo info) override
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        remoteBoxes.erase(handle);
    }

    void removeObjectInstance(
        ObjectInstanceHandle handle,
        const VariableLengthData& tag,
        OrderType sentOrder,
        LogicalTime const& time,
        OrderType receivedOrder,
        SupplementalRemoveInfo info) override
    {
        removeObjectInstance(handle, tag, sentOrder, info);
    }

    void removeObjectInstance(
        ObjectInstanceHandle handle,
        const VariableLengthData& tag,
        OrderType sentOrder,
        LogicalTime const& time,
        OrderType receivedOrder,
        MessageRetractionHandle retraction,
        SupplementalRemoveInfo info) override
    {
        removeObjectInstance(handle, tag, sentOrder, info);
    }

    // Time management callbacks
    void timeRegulationEnabled(const LogicalTime& time) override {
        isTimeRegulating = true;
        const auto& t = dynamic_cast<const HLAfloat64Time&>(time);
        currentLogicalTime = t.getTime();
    }

    void timeConstrainedEnabled(const LogicalTime& time) override {
        isTimeConstrained = true;
        const auto& t = dynamic_cast<const HLAfloat64Time&>(time);
        currentLogicalTime = t.getTime();
    }

    void timeAdvanceGrant(const LogicalTime& time) override {
        timeAdvanceGranted = true;
        const auto& t = dynamic_cast<const HLAfloat64Time&>(time);
        currentLogicalTime = t.getTime();
    }
};

// ─── Global state ────────────────────────────────────────────────
static std::unique_ptr<RTIambassador> g_rti;
static DualSimAmbassador g_fedAmb;
static bool g_connected = false;
static std::vector<BoxData> g_boxBuffer; // for GetBoxes return

// ─── Exported C API ──────────────────────────────────────────────
extern "C" {

#define EXPORT __attribute__((visibility("default")))

EXPORT bool Connect(const char* federationName, const char* federateName, const char* fomFilePath)
{
    try {
        RTIambassadorFactory factory;
        g_rti = factory.createRTIambassador();
        g_rti->connect(g_fedAmb, HLA_EVOKED);

        // Create federation (ignore if exists)
        std::vector<std::wstring> fomModules;
        fomModules.push_back(toWide(fomFilePath));
        try {
            g_rti->createFederationExecution(
                toWide(federationName), fomModules, L"HLAfloat64Time");
        } catch (FederationExecutionAlreadyExists&) {
            // OK
        }

        // Join
        g_rti->joinFederationExecution(
            toWide(federateName), toWide(federationName));

        // Get handles
        g_fedAmb.boxClassHandle = g_rti->getObjectClassHandle(L"HLAobjectRoot.Box");
        g_fedAmb.attrPositionX  = g_rti->getAttributeHandle(g_fedAmb.boxClassHandle, L"PositionX");
        g_fedAmb.attrPositionY  = g_rti->getAttributeHandle(g_fedAmb.boxClassHandle, L"PositionY");
        g_fedAmb.attrRotationY  = g_rti->getAttributeHandle(g_fedAmb.boxClassHandle, L"RotationY");
        g_fedAmb.attrPositionZ  = g_rti->getAttributeHandle(g_fedAmb.boxClassHandle, L"PositionZ");

        g_connected = true;
        return true;
    } catch (std::exception& e) {
        return false;
    }
}

EXPORT void Disconnect()
{
    if (!g_connected) return;
    try {
        g_rti->resignFederationExecution(NO_ACTION);
        try {
            g_rti->destroyFederationExecution(L"DualSimFed");
        } catch (...) {}
        g_rti->disconnect();
    } catch (...) {}
    g_connected = false;
}

EXPORT void PublishUnit()
{
    if (!g_connected) return;
    AttributeHandleSet attrs;
    attrs.insert(g_fedAmb.attrPositionX);
    attrs.insert(g_fedAmb.attrPositionY);
    attrs.insert(g_fedAmb.attrRotationY);
    attrs.insert(g_fedAmb.attrPositionZ);
    g_rti->publishObjectClassAttributes(g_fedAmb.boxClassHandle, attrs);
}

EXPORT void SubscribeUnit()
{
    if (!g_connected) return;
    AttributeHandleSet attrs;
    attrs.insert(g_fedAmb.attrPositionX);
    attrs.insert(g_fedAmb.attrPositionY);
    attrs.insert(g_fedAmb.attrRotationY);
    attrs.insert(g_fedAmb.attrPositionZ);
    g_rti->subscribeObjectClassAttributes(g_fedAmb.boxClassHandle, attrs);
}

EXPORT int CreateUnit()
{
    if (!g_connected) return -1;

    // Reserve name
    std::wstring name = L"UnityRobot";
    g_fedAmb.reservationComplete = false;
    g_rti->reserveObjectInstanceName(name);
    while (!g_fedAmb.reservationComplete) {
        g_rti->evokeCallback(0.1);
    }

    if (!g_fedAmb.reservationSucceeded) return -1;

    g_fedAmb.myInstanceHandle = g_rti->registerObjectInstance(
        g_fedAmb.boxClassHandle, name);
    g_fedAmb.myInstanceId = g_fedAmb.nextId++;
    g_fedAmb.instanceRegistered = true;

    return g_fedAmb.myInstanceId;
}

EXPORT void UpdateUnit(BoxData data)
{
    if (!g_connected || !g_fedAmb.instanceRegistered) return;

    AttributeHandleValueMap values;
    values[g_fedAmb.attrPositionX] = encodeFloat(data.positionX);
    values[g_fedAmb.attrPositionY] = encodeFloat(data.positionY);
    values[g_fedAmb.attrRotationY] = encodeFloat(data.rotationY);
    values[g_fedAmb.attrPositionZ] = encodeFloat(data.positionZ);

    g_rti->updateAttributeValues(g_fedAmb.myInstanceHandle, values, VariableLengthData());
}

EXPORT BoxData* GetBoxes(int* count)
{
    std::lock_guard<std::mutex> lock(g_fedAmb.dataMutex);

    g_boxBuffer.clear();
    for (auto& kv : g_fedAmb.remoteBoxes) {
        g_boxBuffer.push_back(kv.second);
    }

    *count = static_cast<int>(g_boxBuffer.size());
    return g_boxBuffer.empty() ? nullptr : g_boxBuffer.data();
}

// ─── Time Management ─────────────────────────────────────────────

EXPORT void EnableTimeManagement(int role, double lookahead)
{
    if (!g_connected) return;
    g_fedAmb.lookahead = lookahead;

    try {
        // role: 0=Sender(regulating), 1=Viewer(constrained), 2=Hybrid(both)
        if (role == 0 || role == 2) {
            HLAfloat64Interval la(lookahead);
            g_rti->enableTimeRegulation(la);
            while (!g_fedAmb.isTimeRegulating) {
                g_rti->evokeCallback(0.1);
            }
        }
        if (role == 1 || role == 2) {
            g_rti->enableTimeConstrained();
            while (!g_fedAmb.isTimeConstrained) {
                g_rti->evokeCallback(0.1);
            }
        }
    } catch (...) {}
}

EXPORT void DisableTimeManagement()
{
    if (!g_connected) return;
    try {
        if (g_fedAmb.isTimeRegulating) g_rti->disableTimeRegulation();
        if (g_fedAmb.isTimeConstrained) g_rti->disableTimeConstrained();
    } catch (...) {}
    g_fedAmb.isTimeRegulating = false;
    g_fedAmb.isTimeConstrained = false;
}

EXPORT void RequestTimeAdvance(double timeStep)
{
    if (!g_connected) return;
    g_fedAmb.timeAdvanceGranted = false;
    double newTime = g_fedAmb.currentLogicalTime + timeStep;
    try {
        HLAfloat64Time t(newTime);
        g_rti->timeAdvanceRequest(t);
    } catch (...) {}
}

EXPORT bool IsTimeAdvanceGranted()
{
    return g_fedAmb.timeAdvanceGranted;
}

EXPORT double GetCurrentLogicalTime()
{
    return g_fedAmb.currentLogicalTime;
}

struct TimeManagementState {
    bool isTimeRegulating;
    bool isTimeConstrained;
    bool timeAdvanceGranted;
    double currentLogicalTime;
    double lookahead;
};

EXPORT TimeManagementState GetTimeManagementState()
{
    TimeManagementState s;
    s.isTimeRegulating = g_fedAmb.isTimeRegulating;
    s.isTimeConstrained = g_fedAmb.isTimeConstrained;
    s.timeAdvanceGranted = g_fedAmb.timeAdvanceGranted;
    s.currentLogicalTime = g_fedAmb.currentLogicalTime;
    s.lookahead = g_fedAmb.lookahead;
    return s;
}

EXPORT void EvokeCallbacks(double seconds)
{
    if (!g_connected) return;
    try {
        g_rti->evokeCallback(seconds);
    } catch (...) {}
}

} // extern "C"
