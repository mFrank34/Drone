#include "commands.h"
#include "shared.h"

void Commands::poll()
{
    while (Serial.available())
    {
        const char c = Serial.read();
        if (c != '\n' && c != '\r')
        {
            if (n_ < sizeof(buf_) - 1)
                buf_[n_++] = c;
            continue;
        }
        if (n_ == 0)
            continue;
        buf_[n_] = 0;
        n_ = 0;
        handleLine(buf_);
    }
}

void Commands::handleLine(const char *line)
{
    switch (line[0])
    {
    case 'a':
        cmd.arm = true;
        break;
    case 's':
        cmd.stop = true;
        cmd.thr = 0;
        break;
    case 'b':
        cmd.stab = !cmd.stab;
        Serial.printf("stab %d\n", (int)cmd.stab);
        break;
    case 'm':
        cmd.sel = constrain(atoi(line + 1), 0, 4);
        break;
    case 't':
        cmd.thr = constrain(atoi(line + 1), 0, 100) / 100.0f;
        break;
    default:
        Serial.println("?");
        break;
    }
}