
#include "TP_StateEstimate\TP_ESTIMATOR.h"

#define SERIAL_OFF 0
#define MEKF2_COMPARE_MONITOR 1
#define SERIAL_DEBUG 2
#define SERIAL_FOR_MAG_CALIB 3
#define ALL_ESTIMATORS_MONITOR 4

void MEKF2_COMPARE_print(TP_ESTIMATOR tp_estimator){
#if ESTIMATOR == MEKF2_COMPARE
    static Quaternion q;
    q = tp_estimator.tp_mekf2.get_q();
    Serial.print(q.q1);        Serial.print(",");
    Serial.print(q.q2);        Serial.print(",");
    Serial.print(q.q3);        Serial.print(",");
    Serial.print(q.q4);        Serial.print(",");

    q = tp_estimator.tp_mekf2_acc.get_q();
    Serial.print(q.q1);        Serial.print(",");
    Serial.print(q.q2);        Serial.print(",");
    Serial.print(q.q3);        Serial.print(",");
    Serial.print(q.q4);        Serial.print(",");

    q = tp_estimator.tp_mekf2_triad.get_q();
    Serial.print(q.q1);        Serial.print(",");
    Serial.print(q.q2);        Serial.print(",");
    Serial.print(q.q3);        Serial.print(",");
    Serial.print(q.q4);        Serial.print(",");


    static Vector3f a_m, m_m, a_p, m_p;

    a_m = tp_estimator.tp_mekf2.a_m;
    m_m = tp_estimator.tp_mekf2.m_m;
    a_p = tp_estimator.tp_mekf2.a_p.tofloat();
    m_p = tp_estimator.tp_mekf2.m_p.tofloat();

    Serial.print(a_m.x);        Serial.print(",");
    Serial.print(a_m.y);        Serial.print(",");
    Serial.print(a_m.z);        Serial.print(",");

    Serial.print(m_m.x);        Serial.print(",");
    Serial.print(m_m.y);        Serial.print(",");
    Serial.print(m_m.z);        Serial.print(","); 

    Serial.print(a_p.x);        Serial.print(",");
    Serial.print(a_p.y);        Serial.print(",");
    Serial.print(a_p.z);        Serial.print(",");

    Serial.print(m_p.x);        Serial.print(",");
    Serial.print(m_p.y);        Serial.print(",");
    Serial.print(m_p.z);        Serial.print(","); 

    a_m = tp_estimator.tp_mekf2_acc.a_m;
    m_m = tp_estimator.tp_mekf2_acc.m_m;
    a_p = tp_estimator.tp_mekf2_acc.a_p.tofloat();
    m_p = tp_estimator.tp_mekf2_acc.m_p.tofloat();

    Serial.print(a_m.x);        Serial.print(",");
    Serial.print(a_m.y);        Serial.print(",");
    Serial.print(a_m.z);        Serial.print(",");

    Serial.print(m_m.x);        Serial.print(",");
    Serial.print(m_m.y);        Serial.print(",");
    Serial.print(m_m.z);        Serial.print(","); 

    Serial.print(a_p.x);        Serial.print(",");
    Serial.print(a_p.y);        Serial.print(",");
    Serial.print(a_p.z);        Serial.print(",");

    Serial.print(m_p.x);        Serial.print(",");
    Serial.print(m_p.y);        Serial.print(",");
    Serial.print(m_p.z);        Serial.print(","); 

    a_m = tp_estimator.tp_mekf2_triad.a_m;
    m_m = tp_estimator.tp_mekf2_triad.m_m;
    a_p = tp_estimator.tp_mekf2_triad.a_p.tofloat();
    m_p = tp_estimator.tp_mekf2_triad.m_p.tofloat();

    Serial.print(a_m.x);        Serial.print(",");
    Serial.print(a_m.y);        Serial.print(",");
    Serial.print(a_m.z);        Serial.print(",");

    Serial.print(m_m.x);        Serial.print(",");
    Serial.print(m_m.y);        Serial.print(",");
    Serial.print(m_m.z);        Serial.print(","); 

    Serial.print(a_p.x);        Serial.print(",");
    Serial.print(a_p.y);        Serial.print(",");
    Serial.print(a_p.z);        Serial.print(",");

    Serial.print(m_p.x);        Serial.print(",");
    Serial.print(m_p.y);        Serial.print(",");
    Serial.println(m_p.z); 
#endif
}

void ALL_ESTIMATORS_print(TP_ESTIMATOR tp_estimator){
#if ESTIMATOR == ALL_ESTIMATORS
    static Quaternion q;
    q.from_rotation_matrix(tp_estimator.tp_triad.DCM);
    Serial.print(q.q1);        Serial.print(",");
    Serial.print(q.q2);        Serial.print(",");
    Serial.print(q.q3);        Serial.print(",");
    Serial.print(q.q4);        Serial.print(",");

    q = tp_estimator.tp_mekf.get_q();
    Serial.print(q.q1);        Serial.print(",");
    Serial.print(q.q2);        Serial.print(",");
    Serial.print(q.q3);        Serial.print(",");
    Serial.print(q.q4);        Serial.print(",");

    q = tp_estimator.tp_mekf2.get_q();
    Serial.print(q.q1);        Serial.print(",");
    Serial.print(q.q2);        Serial.print(",");
    Serial.print(q.q3);        Serial.print(",");
    Serial.print(q.q4);        Serial.print(",");

    q = tp_estimator.tp_mekf2_triad.get_q();
    Serial.print(q.q1);        Serial.print(",");
    Serial.print(q.q2);        Serial.print(",");
    Serial.print(q.q3);        Serial.print(",");
    Serial.println(q.q4);
#endif
}