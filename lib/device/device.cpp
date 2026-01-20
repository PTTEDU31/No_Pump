#include <Arduino.h>
#include "device.h"

// For SAMD21 (Arduino Nano 33 IoT) - single core architecture
#define CURRENT_CORE 0

// Device class implementation
Device::Device(uint32_t subscribeEvents, int8_t core)
    : subscribeEvents(subscribeEvents), core(core) {}

// DeviceManager implementation
DeviceManager &DeviceManager::getInstance()
{
  static DeviceManager instance;
  return instance;
}

void DeviceManager::registerDevices(Device **devs, uint8_t count)
{
  devices = devs;
  deviceCount = count;
}

void DeviceManager::init()
{
  for (size_t i = 0; i < deviceCount; i++)
  {
      bool init_success = devices[i]->initialize();
      if (!init_success)
      {
        devices[i]->setEnabled(false);
      }
  }
}


void DeviceManager::start() {
  unsigned long now = millis();

  for (size_t i = 0; i < deviceCount; i++) {
    if (devices[i]->isEnabled()) {
      deviceTimeout[i] = 0xFFFFFFFF;
      int delay = devices[i]->start();
      deviceTimeout[i] = delay == DURATION_NEVER ? 0xFFFFFFFF : now + delay;
    }
  }
}
void DeviceManager::stop()
{
}

void DeviceManager::triggerEvent(uint32_t events)
{
  eventFired[0] |= events;
}

int DeviceManager::devicesUpdate(unsigned long now)
{
  uint32_t events = eventFired[0];
  eventFired[0] = 0;
  bool handleEvents = events != 0;

  // Handle events
  if (handleEvents)
  {
    for (size_t i = 0; i < deviceCount; i++)
    {
      if (devices[i]->isEnabled() &&
          (devices[i]->getSubscribeEvents() & events) != 0)
      {
        int delay = devices[i]->event();
        if (delay != DURATION_IGNORE)
        {
          deviceTimeout[i] = delay == DURATION_NEVER ? 0xFFFFFFFF : now + delay;
        }
      }
    }
  }

  // Handle timeouts
  int smallest_delay = DURATION_NEVER;
  for (size_t i = 0; i < deviceCount; i++)
  {
    if (devices[i]->isEnabled())
    {
      long delay = deviceTimeout[i] == 0xFFFFFFFF
                       ? DURATION_NEVER
                       : (long)(deviceTimeout[i] - now);

      if ((long)now >= (long)deviceTimeout[i] && deviceTimeout[i] != 0xFFFFFFFF)
      {
        delay = devices[i]->timeout();
        deviceTimeout[i] = delay == DURATION_NEVER ? 0xFFFFFFFF : now + delay;
      }

      if (delay != DURATION_NEVER && delay >= 0)
      {
        smallest_delay =
            (smallest_delay == DURATION_NEVER)
                ? delay
                : (smallest_delay < delay ? smallest_delay : delay);
      }
    }
  }

  return smallest_delay;
}

void DeviceManager::update(unsigned long now)
{
  devicesUpdate(now);
}