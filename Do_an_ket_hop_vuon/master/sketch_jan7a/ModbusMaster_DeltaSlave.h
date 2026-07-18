#ifndef MODBUSMASTER_DELTASLAVE_H
#define MODBUSMASTER_DELTASLAVE_H

#include <Arduino.h>

#define TXBUF_SZ 128
#define RXBUF_SZ 128

class ModbusMaster_DeltaSlave {
public:
    ModbusMaster_DeltaSlave();

    void begin(HardwareSerial &ser, int rxPin, int txPin, int dePin_, uint32_t baud);

    void setRetries(uint8_t r);
    void setTimeout(uint16_t t);
    void enableDebug(bool e);

    // ======= API chuẩn bạn đã dùng trong code .ino ========
    bool writeSingle(uint8_t slave, uint16_t Dn, uint16_t value);
    bool readSingle(uint8_t slave, uint16_t Dn, uint16_t &outValue);

    // KHỚP HOÀN TOÀN lời gọi trong .ino:
    // writeMultiple(slave, startDn, buf, qty)
    bool writeMultiple(uint8_t slave, uint16_t startDn, const uint16_t *buf, uint16_t qty);

    // readMultiple(slave, startDn, buf, qty)
    bool readMultiple(uint8_t slave, uint16_t startDn, uint16_t *outBuf, uint16_t qty);

    // ======= Mở rộng thêm cho Delta Slave ========// 
    // Đọc/Ghi M-Coil (Bit)
    bool writeSingleM(uint8_t slave, uint16_t Mn, bool on);
    bool readMultiM(uint8_t slave, uint16_t Mstart, bool *out, uint16_t quantity);
    bool writeMultiM(uint8_t slave, uint16_t Mstart, const bool *data, uint16_t quantity);
private:
    HardwareSerial *serial;
    int dePin;

    uint16_t timeout;
    uint8_t retries;
    bool debug;

    uint8_t txBuf[TXBUF_SZ];
    uint8_t rxBuf[RXBUF_SZ];

    void rs485_tx();
    void rs485_rx();

    uint16_t deltaAddr(uint16_t Dn);
    uint16_t crc16(const uint8_t *buf, uint16_t len);
    int readFrame(uint8_t *buf, int maxLen, uint16_t timeout);

    bool transaction(const uint8_t *tx, int txLen, uint8_t *rx, int *rxLen, const char *label);
};

#endif
