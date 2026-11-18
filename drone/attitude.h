// attitude.h

struct Attitude
{
    float roll, pitch, gxf, gyf;
};

struct Correction
{
    float roll, pitch;
};

class Attitude

{
private:
    float iRoll_ = 0, iPitch_ = 0;

public:
    Correction update(const Attitude &attitude, float dt);
    void reset();
};
