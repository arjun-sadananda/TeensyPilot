#pragma once

#include <Arduino.h>
#include "Wire.h"
#include "quaternion.h"
#include "matrix3.h"

class TP_TRIAD

{
private:
protected:
public:
    Matrix3f DCM;
    Vector3f c2, m;

    TP_TRIAD(){}

    void estimate_attitude(Vector3f mag, Vector3f acc){
        c2 = (acc%mag).normalized();
        m = c2%acc;
        DCM(m, c2, acc);
        //North(compass-ish)   East  gravity(Down)
    }
};