#pragma once

#include <stdint.h>
#include <stdbool.h>

// duration constants which can be returned from start(), event() or timeout()
#define DURATION_IGNORE -2      // when returned from event() does not update the current timeout
#define DURATION_NEVER -1       // timeout() will not be called, only event()
#define DURATION_IMMEDIATELY 0  // timeout() will be called each loop

enum deviceEvent_t {
    EVENT_NONE = 0,


    EVENT_POWER_CHANGED = 1 << 1,
    EVENT_MODEL_SELECTED = 1 << 5,
    EVENT_CONNECTION_CHANGED = 1 << 6,

    // Network connectivity events
    EVENT_WIFI_CONNECTED = 1 << 7,
    EVENT_WIFI_DISCONNECTED = 1 << 8,
    EVENT_WIFI_RECONNECTING = 1 << 9,
    EVENT_BLE_CONNECTED = 1 << 10,
    EVENT_BLE_DISCONNECTED = 1 << 11,
    EVENT_NETWORK_SLOW = 1 << 12,        // Low speed network detected
    EVENT_NETWORK_FAST = 1 << 13,        // Good network speed restored
    EVENT_NETWORK_TIMEOUT = 1 << 14,     // Network timeout/unreachable

    // Configuration events
    EVENT_CONFIG_MODEL_CHANGED = 1 << 16,
    EVENT_CONFIG_MOTION_CHANGED = 1 << 20,
    EVENT_CONFIG_BUTTON_CHANGED = 1 << 21,
    EVENT_CONFIG_UID_CHANGED = 1 << 22,
    EVENT_CONFIG_SERIAL_CHANGE = 1 << 25,

    EVENT_ALL = 0xFFFFFFFF
};

/**
 * @brief Base class for all devices
 * 
 * Child devices inherit from this class and override the required methods
 */
class Device {
public:
    /**
     * @brief Constructor
     * @param subscribeEvents Bitset of events this device subscribes to
     * @param core Core ID to run device on (0 = alternate core, 1 = loop core, -1 = any)
     */
    Device(uint32_t subscribeEvents = EVENT_NONE, int8_t core = 1);
    
    virtual ~Device() = default;

    /**
     * @brief Called at the beginning of setup() so the device can configure IO pins etc.
     * @return true if initialization successful, false otherwise
     */
    virtual bool initialize() { return true; }

    /**
     * @brief called at the end of setup() and returns the number of milliseconds when
     * to call timeout() function.
     * @return Duration in milliseconds, or DURATION_NEVER, DURATION_IMMEDIATELY
     */
    virtual int start() { return DURATION_NEVER; }

    /**
     * @brief An event was fired in the main code, perform any required action that this
     * device requires and return new duration for timeout() call.
     * @return New duration, or DURATION_IGNORE to keep current timeout
     */
    virtual int event() { return DURATION_IGNORE; }

    /**
     * @brief The current timeout duration has expired so take appropriate action and return
     * a new duration, this function should not return DURATION_IGNORE.
     * @return New duration in milliseconds
     */
    virtual int timeout() { return DURATION_NEVER; }

    /**
     * @brief Get the events this device subscribes to
     */
    uint32_t getSubscribeEvents() const { return subscribeEvents; }

    /**
     * @brief Get the core this device should run on
     */
    int8_t getCore() const { return core; }

    /**
     * @brief Check if device is enabled
     */
    bool isEnabled() const { return enabled; }

    /**
     * @brief Disable device (skip all callbacks)
     */
    void setEnabled(bool en) { enabled = en; }

protected:
    uint32_t subscribeEvents;
    int8_t core;
    bool enabled = true;
};

/**
 * @brief Device Manager - manages all devices
 */
class DeviceManager {
public:
    /**
     * @brief Get singleton instance
     */
    static DeviceManager& getInstance();

    /**
     * @brief Register a list of devices to be managed
     * @param devices Array of device pointers
     * @param count Number of devices
     */
    void registerDevices(Device** devices, uint8_t count);

    /**
     * @brief Call initialize() on all registered devices
     */
    void init();

    /**
     * @brief Call start() on all registered devices
     */
    void start();

    /**
     * @brief Update all devices (called in main loop)
     * @param now Current time in milliseconds (from millis())
     */
    void update(unsigned long now);

    /**
     * @brief Trigger an event for all subscribed devices
     * @param events Event flags
     */
    void triggerEvent(uint32_t events);

    /**
     * @brief Stop all devices
     */
    void stop();

    /**
     * @brief Internal update method (public for task access)
     * @param now Current time in milliseconds
     * @return Delay until next update
     */
    int devicesUpdate(unsigned long now);

private:
    DeviceManager() = default;
    ~DeviceManager() = default;
    DeviceManager(const DeviceManager&) = delete;
    DeviceManager& operator=(const DeviceManager&) = delete;

    Device** devices = nullptr;
    uint8_t deviceCount = 0;
    uint32_t eventFired[1] = {0};  // Single core for SAMD21
    unsigned long deviceTimeout[16] = {0};  // Use unsigned long for millis()
};

// Convenience functions for backward compatibility
inline void devicesRegister(Device** devices, uint8_t count) {
    DeviceManager::getInstance().registerDevices(devices, count);
}

inline void devicesInit() {
    DeviceManager::getInstance().init();
}

inline void devicesStart() {
    DeviceManager::getInstance().start();
}

inline void devicesUpdate(unsigned long now) {
    DeviceManager::getInstance().update(now);
}

inline void devicesTriggerEvent(uint32_t events) {
    DeviceManager::getInstance().triggerEvent(events);
}

inline void devicesStop() {
    DeviceManager::getInstance().stop();
}
