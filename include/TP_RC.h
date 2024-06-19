
#include "CRSFforArduino.hpp"

enum switch3{
    ONE,
    TWO,
    THREE
};

struct RC_Command{
    
    uint16_t throttle, yaw, roll, pitch;
    bool arm, r_toggle;
    enum switch3 sw1, sw2, sw3;
};


void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcData);

class TP_RC{
    private:

    CRSFforArduino *crsf = nullptr;

    public:
    RC_Command rc_command;

    TP_RC(){
        
        // Initialise CRSF for Arduino.
        crsf = new CRSFforArduino(&Serial1, 0, 1);
    }

    void init(){
        if (crsf->begin()){
            crsf->setRcChannelsCallback(onReceiveRcChannels);
        }
        else    {
            crsf->end();
            delete crsf;
            crsf = nullptr;
        }
    }
    void set_data(){
        rc_command.roll   = crsf->rcToUs(crsf->getChannel(1));
        rc_command.pitch  = crsf->rcToUs(crsf->getChannel(2));
        rc_command.throttle = crsf->rcToUs(crsf->getChannel(3));
        rc_command.yaw    = crsf->rcToUs(crsf->getChannel(4));
        uint16_t temp = crsf->rcToUs(crsf->getChannel(5));
        if (temp>1500)
            rc_command.arm = true;
        else
            rc_command.arm = false;
        
    }

    void update(){
        /* Guard CRSF for Arduino's API with a null check. */
        if (crsf != nullptr)
        {
            /* Call CRSF for Arduino's main function here. */
            crsf->update();
        }
    }
}tp_rc;

// void TP_RC::onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (rcChannels->failsafe == false)
    {
        tp_rc.set_data();
    }
}