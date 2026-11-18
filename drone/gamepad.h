#pragma once
#include <Bluepad32.h>

namespace drone
{
    class Gamepad
    {
    public:
        void begin();
        void update();
        ControllerPtr get(int i) const { return controllers_[i]; }

    private:
        void onConnected(ControllerPtr ctl);
        void onDisconnected(ControllerPtr ctl);

        // BP32.setup() needs plain function pointers, so these forward to the instance
        static void onConnectedThunk(ControllerPtr ctl);
        static void onDisconnectedThunk(ControllerPtr ctl);
        static Gamepad *instance_;

        ControllerPtr controllers_[BP32_MAX_GAMEPADS] = {};
    };
}