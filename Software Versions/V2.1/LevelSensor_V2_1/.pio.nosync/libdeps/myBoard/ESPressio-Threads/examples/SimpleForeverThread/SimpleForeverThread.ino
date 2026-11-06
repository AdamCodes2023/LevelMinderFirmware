/*
    An extremely simple example of an ESPressio Thread loop that runs on either core of the ESP32.
*/

#define ESPRESSIO_THREAD_DEFAULT_STACK_SIZE = 1600 // This sets the default Stack Size for all Threads in the system.

#include <Arduino.h>

#include "ESPressio_IThread.hpp" // This gives us access to the `IThread` interface.
#include "ESPressio_Thread.hpp" // This gives us access to our `Thread` base class.
#include "ESPressio_ThreadManager.hpp" // This gives us access to the `ThreadManager` class.

using namespace ESPressio::Threads;

class DemoThread : public Thread {
    private:
        uint32_t _counter = 0;
    protected:
        void OnLoop() {
            Serial.printf("DemoThread: %u\n", _counter++);
            delay(1000);
        }
};

DemoThread thread;

void setup() {
    Serial.begin(115200); // Start the Serial Monitor.

    thread.SetStartOnInitialize(true); // This will start the thread as soon as it's initialized. (true is the default, but we're setting it here for clarity.)

    ThreadManager::Initialize(); // This will initialize ALL Thread instances in your code!
}

void loop() {
    // You can still use your main loop as normal, if you want to!
}