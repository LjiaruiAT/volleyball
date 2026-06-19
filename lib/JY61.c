#include "JY61.h"

uint8_t Gyroscope_Init_count = 0;
//顺时针为负
float Yaw_offset;
JY61_Typedef JY61;

static uint8_t sum10(uint8_t *data) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; i++)
        sum += data[i];
    return sum;
}
void JY61_Receive(JY61_Typedef* Gyro, uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (data[i] == 0x55) { // 包识别
            Angle_Pack_Typedef pack = *(Angle_Pack_Typedef *)(data + i);
            switch (pack.ID) {
                case 0x51://加速度
                    if (pack.sum == sum10(data + i)) {
                        i += 9;//跳过这个数据包长度
                        Gyro->Acceleration.X = pack.X / 32768.0f * 16;
                        Gyro->Acceleration.Y = pack.Y / 32768.0f * 16;
                        Gyro->Acceleration.Z = pack.Z / 32768.0f * 16;
                    }
                    break;
                case 0x52://角速度
                    if (pack.sum == sum10(data + i)) {
                        i += 9;
                        Gyro->AngularVelocity.X = pack.X / 32768.0f * 2000;
                        Gyro->AngularVelocity.Y = pack.Y / 32768.0f * 2000;
                        Gyro->AngularVelocity.Z = pack.Z / 32768.0f * 2000;
                    }
                    break;
                case 0x53://角度
                    if (pack.sum == sum10(data + i)) {
                        i += 9;
                        Gyro->Angle.lastYaw = Gyro->Angle.Yaw;
                        Gyro->Angle.Roll = pack.X / 32768.0f * 180;
                        Gyro->Angle.Pitch = pack.Y / 32768.0f * 180;
                        Gyro->Angle.Yaw = pack.Z / 32768.0f * 180;
                        Gyro->Temp = pack.Temp / 340.0f + 36.53f;
                        float diff = Gyro->Angle.Yaw - Gyro->Angle.lastYaw;

                        if (diff > 180)
                            Gyro->Angle.rand--;
                        else if (diff < -180)
                            Gyro->Angle.rand++;

                        Gyro->Angle.Multiturn = Gyro->Angle.Yaw + Gyro->Angle.rand * 360.0f - Yaw_offset;//顺时针为负
                    }
                    break;
            }
			Gyroscope_Init_count = 1;
        }
    }
}
