#include "co2FromSerial.h"

const uint8_t Co2FromSerial::readCo2Cmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

Co2FromSerial::Co2FromSerial(HardwareSerial serial) : serial(serial)
{
}

void Co2FromSerial::init()
{
    serial.begin(9600);  // communication for MH-Z14A
    clearSerialBuffer(); // read out garbage (i have no idea why but this is a must have for mobile battery operation)
}

void Co2FromSerial::clearSerialBuffer()
{
    uint8_t c;
    while (serial.available())
    {
        c = serial.read();
    }
}

int Co2FromSerial::getCO2()
{
    int returnnum = 0;
    uint16_t co2 = 0;
    uint8_t readBuffer[9] = {};
    int i;
    uint8_t checksum = 0;
    clearSerialBuffer();

    serial.write(readCo2Cmd, sizeof(readCo2Cmd));
    serial.setTimeout(300); //set 300msec timeout
    returnnum = serial.readBytes(readBuffer, sizeof(readBuffer));

    if ((readBuffer[0] == 0xFF) && (readBuffer[1] == 0x86))
    {
        for (i = 1; i < 8; i++)
        {
            checksum += readBuffer[i];
        }
        checksum = 0xff - checksum;
        checksum += 1;
#ifdef DEBUG
        Serial.println(checksum, HEX);
        for (i = 0; i < sizeof(readBuffer); i++)
        {
            Serial.print(readBuffer[i], HEX);
            Serial.print(" ");
        }
#endif
        if (readBuffer[8] != checksum)
        {
            Serial.println("check sum error");
            return (-1); // -1 indicates check sum error
        }
        co2 = (uint16_t)readBuffer[2] << 8;
        co2 += readBuffer[3];
        //    Serial.print("CO2 level(ppm):" );
        // Serial.println(co2);
        return co2;
    }
    else
    {
        return -1;
    }
}
