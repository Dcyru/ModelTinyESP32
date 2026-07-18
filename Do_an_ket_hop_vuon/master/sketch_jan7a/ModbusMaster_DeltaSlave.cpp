#include "ModbusMaster_DeltaSlave.h"

// ---------------------------------------------------------
ModbusMaster_DeltaSlave::ModbusMaster_DeltaSlave() {
    serial = nullptr;
    dePin = -1;
    timeout = 700;
    retries = 3;
    debug = false;
}

// map D-register → địa chỉ modbus
uint16_t ModbusMaster_DeltaSlave::deltaAddr(uint16_t Dn) {
    return 4096 + Dn; 
}

// ---------------------------------------------------------
void ModbusMaster_DeltaSlave::begin(HardwareSerial &ser, int rxPin, int txPin, int dePin_, uint32_t baud) {
    serial = &ser;
    dePin = dePin_;

    pinMode(dePin, OUTPUT);
    rs485_rx();

    serial->begin(baud, SERIAL_8E1, rxPin, txPin);
}

void ModbusMaster_DeltaSlave::setRetries(uint8_t r) { retries = r; }
void ModbusMaster_DeltaSlave::setTimeout(uint16_t t) { timeout = t; }
void ModbusMaster_DeltaSlave::enableDebug(bool e) { debug = e; }

void ModbusMaster_DeltaSlave::rs485_tx() { digitalWrite(dePin, HIGH); }
void ModbusMaster_DeltaSlave::rs485_rx() { digitalWrite(dePin, LOW); }

// ---------------------------------------------------------
uint16_t ModbusMaster_DeltaSlave::crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// ---------------------------------------------------------
int ModbusMaster_DeltaSlave::readFrame(uint8_t *buf, int maxLen, uint16_t timeout) {
    int pos = 0;
    unsigned long t0 = millis();

    while (true) {
        while (serial->available() && pos < maxLen) {
            buf[pos++] = serial->read();
            t0 = millis();
        }

        if (pos > 0 && millis() - t0 > 5) break;
        if (pos == 0 && millis() - t0 > timeout) break;
    }
    return pos;
}

// ---------------------------------------------------------
bool ModbusMaster_DeltaSlave::transaction(const uint8_t *tx, int txLen, uint8_t *rx, int *rxLen, const char *label) {
    for (uint8_t a = 1; a <= retries; a++) {

        while (serial->available()) serial->read();

        if (debug) {
            Serial.print("➡️ TX: ");
            for (int i = 0; i < txLen; i++) Serial.printf("%02X ", tx[i]);
            Serial.println();
        }

        rs485_tx();
        serial->write(tx, txLen);
        serial->flush();
        rs485_rx();

        delayMicroseconds(800);

        int r = readFrame(rx, RXBUF_SZ, timeout);
        *rxLen = r;

        if (debug) {
            Serial.print("⬅️ RX: ");
            if (r == 0) Serial.println("[NO DATA]");
            else for (int i = 0; i < r; i++) Serial.printf("%02X ", rx[i]);
            Serial.println();
        }

        if (r >= 5) {
            uint16_t crcCalc = crc16(rx, r - 2);
            uint16_t crcRx = rx[r - 2] | (rx[r - 1] << 8);
            if (crcCalc == crcRx) return true;
        }
    }
    return false;
}

// ---------------------------------------------------------
// WRITE SINGLE
// ---------------------------------------------------------
bool ModbusMaster_DeltaSlave::writeSingle(uint8_t slave, uint16_t Dn, uint16_t value) {
    uint16_t addr = deltaAddr(Dn);

    txBuf[0] = slave;
    txBuf[1] = 0x06;
    txBuf[2] = addr >> 8;
    txBuf[3] = addr & 0xFF;
    txBuf[4] = value >> 8;
    txBuf[5] = value & 0xFF;

    uint16_t crc = crc16(txBuf, 6);
    txBuf[6] = crc & 0xFF;
    txBuf[7] = crc >> 8;

    int rxLen;
    return transaction(txBuf, 8, rxBuf, &rxLen, "WRITE SINGLE");
}

// ---------------------------------------------------------
// READ SINGLE
// ---------------------------------------------------------
bool ModbusMaster_DeltaSlave::readSingle(uint8_t slave, uint16_t Dn, uint16_t &outValue) {
    uint16_t addr = deltaAddr(Dn);

    txBuf[0] = slave;
    txBuf[1] = 0x03;
    txBuf[2] = addr >> 8;
    txBuf[3] = addr & 0xFF;
    txBuf[4] = 0x00;
    txBuf[5] = 0x01;

    uint16_t crc = crc16(txBuf, 6);
    txBuf[6] = crc & 0xFF;
    txBuf[7] = crc >> 8;

    int rxLen;
    if (!transaction(txBuf, 8, rxBuf, &rxLen, "READ SINGLE")) return false;

    if (rxLen >= 7 && rxBuf[1] == 0x03 && rxBuf[2] == 2) {
        outValue = (rxBuf[3] << 8) | rxBuf[4];
        return true;
    }
    return false;
}

// ---------------------------------------------------------
// WRITE MULTIPLE (API CHUẨN)
// ---------------------------------------------------------
bool ModbusMaster_DeltaSlave::writeMultiple(uint8_t slave, uint16_t startDn, const uint16_t *buf, uint16_t qty) {

    uint16_t addr = deltaAddr(startDn);

    int i = 0;
    txBuf[i++] = slave;
    txBuf[i++] = 0x10;
    txBuf[i++] = addr >> 8;
    txBuf[i++] = addr & 0xFF;
    txBuf[i++] = qty >> 8;
    txBuf[i++] = qty & 0xFF;
    txBuf[i++] = qty * 2;

    for (uint16_t k = 0; k < qty; k++) {
        txBuf[i++] = buf[k] >> 8;
        txBuf[i++] = buf[k] & 0xFF;
    }

    uint16_t crc = crc16(txBuf, i);
    txBuf[i++] = crc & 0xFF;
    txBuf[i++] = crc >> 8;

    int rxLen;
    return transaction(txBuf, i, rxBuf, &rxLen, "WRITE MULTI");
}

// ---------------------------------------------------------
// READ MULTIPLE (API CHUẨN)
// ---------------------------------------------------------
bool ModbusMaster_DeltaSlave::readMultiple(uint8_t slave, uint16_t startDn, uint16_t *outBuf, uint16_t qty) {

    uint16_t addr = deltaAddr(startDn);

    txBuf[0] = slave;
    txBuf[1] = 0x03;
    txBuf[2] = addr >> 8;
    txBuf[3] = addr & 0xFF;
    txBuf[4] = qty >> 8;
    txBuf[5] = qty & 0xFF;

    uint16_t crc = crc16(txBuf, 6);
    txBuf[6] = crc & 0xFF;
    txBuf[7] = crc >> 8;

    int rxLen;
    if (!transaction(txBuf, 8, rxBuf, &rxLen, "READ MULTI")) return false;

    if (rxLen < 3 || rxBuf[1] != 0x03) return false;

    uint8_t byteCount = rxBuf[2];
    if (byteCount != qty * 2) return false;

    for (uint16_t i = 0; i < qty; i++)
        outBuf[i] = (rxBuf[3 + 2*i] << 8) | rxBuf[4 + 2*i];

    return true;
}

bool ModbusMaster_DeltaSlave::writeSingleM(uint8_t slave, uint16_t Mn, bool on)
{
    uint16_t addr = 2049 + Mn - 1;

    txBuf[0] = slave;
    txBuf[1] = 0x05;               //=========== Write Single Coil==========//
    txBuf[2] = addr >> 8;
    txBuf[3] = addr & 0xFF;

    if (on) {
        txBuf[4] = 0xFF;
        txBuf[5] = 0x00;
    } else {
        txBuf[4] = 0x00;
        txBuf[5] = 0x00;
    }

    uint16_t crc = crc16(txBuf, 6);
    txBuf[6] = crc & 0xFF;
    txBuf[7] = crc >> 8;

    int rxLen;
    return transaction(txBuf, 8, rxBuf, &rxLen, "WRITE SINGLE COIL");
}

//======Nhớ tạo mạng trước khi dùng này ======//
/*bool mData[6] = {
    true,   // M10
    false,  // M11
    true,   // M12
    true,   // M13
    false,  // M14
    true    // M15
};

writeMultiM(1, 10, mData, 6);*/
bool ModbusMaster_DeltaSlave::writeMultiM(uint8_t slave, uint16_t Mstart, const bool *data, uint16_t quantity)
{
    uint16_t addr = 2049 + Mstart;
    uint8_t byteCount = (quantity + 7) / 8;

    txBuf[0] = slave;
    txBuf[1] = 0x0F;               // Write Multiple Coils
    txBuf[2] = addr >> 8;
    txBuf[3] = addr & 0xFF;
    txBuf[4] = quantity >> 8;
    txBuf[5] = quantity & 0xFF;
    txBuf[6] = byteCount;

    // Pack bits
    for (int i = 0; i < byteCount; i++)
        txBuf[7 + i] = 0;

    for (int i = 0; i < quantity; i++)
    {
        if (data[i])
            txBuf[7 + (i / 8)] |= (1 << (i % 8));
    }

    uint16_t frameLen = 7 + byteCount;
    uint16_t crc = crc16(txBuf, frameLen);
    txBuf[frameLen]     = crc & 0xFF;
    txBuf[frameLen + 1] = crc >> 8;

    int rxLen;
    return transaction(txBuf, frameLen + 2, rxBuf, &rxLen, "WRITE MULTI COILS");
}

/*bool mState[8];

if (readMultiM(1, 0, mState, 8))
{
    for (int i = 0; i < 8; i++)
        Serial.printf("M%d = %d\n", i, mState[i]);
} */
bool ModbusMaster_DeltaSlave::readMultiM(uint8_t slave, uint16_t Mstart, bool *out, uint16_t quantity)
{
    uint16_t addr = 2049 + Mstart - 1;

    txBuf[0] = slave;
    txBuf[1] = 0x01;               // Read Coils
    txBuf[2] = addr >> 8;
    txBuf[3] = addr & 0xFF;
    txBuf[4] = quantity >> 8;
    txBuf[5] = quantity & 0xFF;

    uint16_t crc = crc16(txBuf, 6);
    txBuf[6] = crc & 0xFF;
    txBuf[7] = crc >> 8;

    int rxLen;
    if (!transaction(txBuf, 8, rxBuf, &rxLen, "READ MULTI M"))
        return false;

    uint8_t byteCount = rxBuf[2];

    for (int i = 0; i < quantity; i++)
    {
        out[i] = (rxBuf[3 + (i / 8)] >> (i % 8)) & 0x01;
    }

    return true;
}

//===========So sánh D với M ===========//
/*
[Slave][06][AddrHi][AddrLo][DataHi][DataLo][CRCLo][CRCHi] ---> Ghi giá trị Data vào D-Register có địa chỉ Addr Data_16bit
[Slave][05][AddrHi][AddrLo][FF][00][CRCLo][CRCHi] ---> GHi coils ON FF00 nghĩa là 255 0000 là 0 off chánh nhiễu tính hiệu trả về số mức giữa ví dụ 230...

*/