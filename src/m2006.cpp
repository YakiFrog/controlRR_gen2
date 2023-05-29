#include "m2006.h"
#include <Arduino.h>
#include <CAN.h>

static uint16_t angle = 0;
static uint16_t rpm = 0;
static uint16_t torque = 0;

static uint8_t top = 0;
static uint8_t bottom = 0;
static int16_t data = 0;

// setup()で呼び出す
bool can_init(int rx, int tx, int speed) {
    CAN.setPins(rx, tx);
    if (!CAN.begin(speed)){
        return false;
    } else {
        volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
        *pREG_IER &= ~(uint8_t)0x10;
        return true;
    }
}

// M2006に送信するデータを作成
void m2006_make_data(int16_t data_in[8], uint8_t data_out1[8], uint8_t data_out2[8]) {
    // 0x200用のデータを作成（ID1-4）
    for (int i = 0; i < 4; i++) {
        data_out1[0 + (i * 2)] = (data_in[i] >> 8) & 0xFF;
        data_out1[1 + (i * 2)] = data_in[i] & 0xFF;
    }
    // 0x1FF用のデータを作成（ID5-8）
    for (int i = 0; i < 4; i++) {
        data_out2[0 + (i * 2)] = (data_in[i + 4] >> 8) & 0xFF;
        data_out2[1 + (i * 2)] = data_in[i + 4] & 0xFF;
    }
}

// M2006にデータを送信
bool m2006_send_data(uint8_t data_out1[8], uint8_t data_out2[8]) {
    bool success0x200 = false;
    bool success0x1FF = false;
    
    CAN.beginPacket(0x200); // 0x200は，M2006のCAN ID(1-4)
    CAN.write(data_out1, 8); // 8 bytes
    if (CAN.endPacket()) success0x200 = true; // 送信
    CAN.beginPacket(0x1FF); // 0x1FFは，M2006のCAN ID(5-8)
    CAN.write(data_out2, 8); // 8 bytes
    if (CAN.endPacket()) success0x1FF = true; // 送信

    return success0x200 && success0x1FF;
}

// M2006からデータを受信
void m2006_read_data(int id, int16_t mangle[8], int16_t mrpm[8], int16_t mtorque[8]) {
    if (CAN.parsePacket() && CAN.filter(id)) {
        angle = 0;
        rpm = 0;
        torque = 0;
        top = 0;
        bottom = 0;
        data = 0;
        long id = CAN.packetId();
        for (int i = 0; i < 4; i++){
            top = CAN.read();
            bottom = CAN.read();
            data = (top << 8) | bottom;
            if (i == 0) angle = (float)(data * 360 / 8192);
            if (i == 1) rpm = (float)(data / 36);
            if (i == 2) torque = (float)(data * 100 / 8192);
            if (i == 3) data = 0;
        }
        mangle[id - 0x201] = angle;
        mrpm[id - 0x201] = rpm;
        mtorque[id - 0x201] = torque;
    }
}