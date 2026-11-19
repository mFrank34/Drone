#include "gamepad.h"
#include "shared.h" // cmd

namespace drone
{

    Gamepad *Gamepad::instance_ = nullptr;

    void Gamepad::begin()
    {
        instance_ = this;
        BP32.setup(&Gamepad::onConnectedThunk, &Gamepad::onDisconnectedThunk);
    }

    void Gamepad::update()
    {
        BP32.update();
        for (auto ctl : controllers_)
            if (ctl && ctl->isConnected() && ctl->hasData())
            {
                applyToCmd(ctl);
                break; // first pad only
            }
    }

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
        cmd.thr = 0; // drop throttle and stop on a real disconnect
        cmd.stop = true;
        for (auto &slot : controllers_)
            if (slot == ctl)
            {
                slot = nullptr;
                return;
            }
    }

    void Gamepad::applyToCmd(ControllerPtr ctl)
    {
        // throttle on the right trigger (0..1023)
        cmd.thr = constrain(ctl->throttle() / 1023.0f, 0.0f, 1.0f);

        // buttons act on the press, not while held
        const bool a = ctl->a(), b = ctl->b(), y = ctl->y();
        if (a && !prevA_)
            cmd.arm = true;
        if (b && !prevB_)
        {
            cmd.stop = true;
            cmd.thr = 0;
        }
        if (y && !prevY_)
            cmd.stab = !cmd.stab;
        prevA_ = a;
        prevB_ = b;
        prevY_ = y;
    }

} // namespace drone