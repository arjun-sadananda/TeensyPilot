
#include "SD.h"

#define SD_LOG_OFF 0
#define SD_LOG_ESTIMATOR 1
#define SD_LOG_ALL_ESTIMATOR 2
#define SD_LOG_DRONE_MONITOR 3


void SD_get_header(String& data, uint8_t sd_log_type){
    if (sd_log_type == SD_LOG_ALL_ESTIMATOR){
        data = "time,gx,gy,gz,ax,ay,ax,mx,my,mz,P00,P01,P02,P03,P04,P05,P10,P11,P12,P13,P14,P15,P20,P21,P22,P23,P24,P25,P30,P31,P32,P33,P34,P35,P40,P41,P42,P43,P44,P45,P50,P51,P52,P53,P54,P55,K00,,,,,,,,,K10,,,,,,,,,K20,,,,,,,,,K30,,,,,,,,,K40,,,,,,,,,K50,,,,,,,,K58";
        data.append(",P00,P01,P02,P03,P04,P05,P10,P11,P12,P13,P14,P15,P20,P21,P22,P23,P24,P25,P30,P31,P32,P33,P34,P35,P40,P41,P42,P43,P44,P45,P50,P51,P52,P53,P54,P55,K00,,,,,,,,,K10,,,,,,,,,K20,,,,,,,,,K30,,,,,,,,,K40,,,,,,,,,K50,,,,,,,,K58");
        data.append(",P00,P01,P02,P03,P04,P05,P10,P11,P12,P13,P14,P15,P20,P21,P22,P23,P24,P25,P30,P31,P32,P33,P34,P35,P40,P41,P42,P43,P44,P45,P50,P51,P52,P53,P54,P55,K00,,,,,,,,,K10,,,,,,,,,K20,,,,,,,,,K30,,,,,,,,,K40,,,,,,,,,K50,,,,,,,,K58");
    }
    else if (sd_log_type == SD_LOG_DRONE_MONITOR){
        data = "time,q0,q1,q2,q3,wx,wy,wz,thrust,qd1,qd2,qd3,qd4,f,Mx,My,Mz";
    }
}