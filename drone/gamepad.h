#pragma once
#include <Bluepad32.h>

namespace drone
{
    class Gamepad
    {
    public:
        void begin();
        void update();

    private:
        void onConnected(ControllerPtr ctl);
        void onDisconnected(ControllerPtr ctl);
        void applyToCmd(ControllerPtr ctl); // new

        static void onConnectedThunk(ControllerPtr ctl);
        static void onDisconnectedThunk(ControllerPtr ctl);
        static Gamepad *instance_;

        ControllerPtr controllers_[BP32_MAX_GAMEPADS] = {};
        bool prevA_ = false, prevB_ = false, prevY_ = false; // for press detection
    };
}