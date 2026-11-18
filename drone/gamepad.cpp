#include "gamepad.h"

namespace drone
{

    Gamepad *Gamepad::instance_ = nullptr;

    void Gamepad::begin()
    {
        instance_ = this;
        BP32.setup(&Gamepad::onConnectedThunk, &Gamepad::onDisconnectedThunk);
    }

    void Gamepad::update() { BP32.update(); }

    void Gamepad::onConnectedThunk(ControllerPtr ctl)
    {
        if (instance_)
            instance_->onConnected(ctl);
    }

    void Gamepad::onDisconnectedThunk(ControllerPtr ctl)
    {
        if (instance_)
            instance_->onDisconnected(ctl);
    }

    void Gamepad::onConnected(ControllerPtr ctl)
    {
        Serial.println("Controller connected");
        for (auto &slot : controllers_)
            if (slot == nullptr)
            {
                slot = ctl;
                return;
            }
    }

    void Gamepad::onDisconnected(ControllerPtr ctl)
    {
        Serial.println("Controller disconnected");
        for (auto &slot : controllers_)
            if (slot == ctl)
            {
                slot = nullptr;
                return;
            }
    }

} // namespace drone