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
    Matrix3f identityDCM;
    Vector3f c2, m;

    TP_TRIAD(){}
    void init_estimator(const Vector3f a_ref, const Vector3f m_ref){
        c2 = (a_ref%m_ref).normalized();
        m = c2%a_ref;
        identityDCM(m, c2, a_ref);
        identityDCM.invert();
    }
    void estimate_attitude(Vector3f mag, Vector3f acc){
        c2 = (acc%mag).normalized();
        m = c2%acc;
        DCM(m, c2, acc);
        DCM = identityDCM * DCM;
        //North(compass-ish)   East  gravity(Down)
    }
};