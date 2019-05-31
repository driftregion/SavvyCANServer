/*
Everything needed to communicate with the remarkable SavvyCAN.

Things that used to be interspersed with this code, but are no longer:
- LED blinking
- logging to files 
*/

#include <Arduino.h>
#include "can_common.h"
#include "SavvyCANServer.h"
#include "EEPROM.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFiUdp.h>
#include "../board.h"

#ifndef SAVVYCAN_FREERTOS_TASK_PRIORITY
#define SAVVYCAN_FREERTOS_TASK_PRIORITY 8  // Override this if your app is competing with SavvyCANServer 
#endif

typedef struct {
    uint32_t bitsPerQuarter;
    uint32_t bitsSoFar;
    uint8_t busloadPercentage;
} BUSLOAD;

BUSLOAD busLoad[2];
uint32_t busLoadTimer;

WiFiUDP wifiUDPServer;
IPAddress broadcastAddr(255,255,255,255);

struct SystemSettings SysSettings;
struct EEPROMSettings settings;



//Get the value of XOR'ing all the bytes together. This creates a reasonable checksum that can be used
//to make sure nothing too stupid has happened on the comm.
uint8_t checksumCalc(uint8_t *buffer, int length)
{
    uint8_t valu = 0;
    for (int c = 0; c < length; c++) {
        valu ^= buffer[c];
    }
    return valu;
}

SavvyCANServer::SavvyCANServer(HardwareSerial &console, WiFiServer &server) :
    console(console),
    wifiServer(server)
{
    receive_queue = xQueueCreate(SAVVYCAN, sizeof(frameobject_t));
}

void flush_buffer(void *pvParameters)
{
    SavvyCANServer *node = (SavvyCANServer *)pvParameters;
    frameobject_t frameobject;

    while (1)
    {
        node->handleWiFi();
        node->handleSerial();
        if (xQueueReceive(node->receive_queue, &frameobject, 0))
        {
            node->packFrameToBeSent(frameobject.frame, frameobject.bus);
        }
        else
            vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void SavvyCANServer::setup(void)
{
    busLoad[0].bitsSoFar = 0;
    busLoad[0].busloadPercentage = 0;
    busLoad[0].bitsPerQuarter = settings.CAN0Speed / 4;

    busLoad[1].bitsSoFar = 0;
    busLoad[1].busloadPercentage = 0;
    busLoad[1].bitsPerQuarter = settings.CAN1Speed / 4;

    busLoadTimer = millis();

    xTaskCreate(&flush_buffer, "SavvyCAN comms", 4096, this, SAVVYCAN_FREERTOS_TASK_PRIORITY, NULL);

    settings.useBinarySerialComm = true;
    SysSettings.isWifiActive = 1;
    settings.version = EEPROM_VER;
    settings.appendFile = false;
    settings.CAN0Speed = 500000;
    settings.CAN0_Enabled = true;
    settings.CAN1Speed = 500000;
    settings.CAN1FDSpeed = 2000000;
    settings.CAN1_Enabled = false;
    settings.CAN0ListenOnly = false;
    settings.CAN1ListenOnly = false;
    settings.CAN1_FDMode = false;
    sprintf((char *)settings.fileNameBase, "CANBUS");
    sprintf((char *)settings.fileNameExt, "TXT");
    settings.fileNum = 1;
    settings.fileOutputType = CRTD;
    settings.useBinarySerialComm = false;
    settings.autoStartLogging = false;
    settings.logLevel = 1; //info
    settings.wifiMode = 2;
    sprintf((char *)settings.SSID, "ESP32DUE");
    sprintf((char *)settings.WPA2Key, "supersecret");
    settings.sysType = 0; //ESP32Due as default
    settings.valid = 0; //not used right now
}

void SavvyCANServer::setPromiscuousMode(void)
{
    CAN0.watchFor();
    CAN1.watchFor();
}

void addBits(int offset, CAN_FRAME_FD &frame)
{
    if (offset < 0) return;
    if (offset > 1) return;
    busLoad[offset].bitsSoFar += 41 + (frame.length * 9);
    if (frame.extended) busLoad[offset].bitsSoFar += 18;
}

void SavvyCANServer::handleSerial(void)
{
    int serialCnt;
    uint8_t in_byte;

    //If the buffer is almost filled then send buffered data out
    if ((serialBufferLength > (WIFI_BUFF_SIZE - 40)) ) {
        if (serialBufferLength > 0) {
            if (settings.wifiMode == 0 || !SysSettings.isWifiActive)
                // Serial.write(serialBuffer, serialBufferLength);
                ;
            else
            {
                for(int i = 0; i < MAX_CLIENTS; i++)
                {
                    if (SysSettings.clientNodes[i] && SysSettings.clientNodes[i].connected())
                    {
                        SysSettings.clientNodes[i].write(serialBuffer, serialBufferLength);
                    }
                }
            }
            serialBufferLength = 0;
            lastFlushMicros = micros();
        }
    }
    serialCnt = 0;
    while ( (Serial.available() > 0) && serialCnt < 128) {
        serialCnt++;
        in_byte = Serial.read();
        SysSettings.isWifiActive = false;
        processIncomingByte(in_byte);
    }
}

void SavvyCANServer::handleWiFi(void)
{
    if (settings.wifiMode > 0)
    {
        int i;
        if (WiFi.isConnected() || settings.wifiMode == 2)
        {
            if (wifiServer.hasClient())
            {
                for(i = 0; i < MAX_CLIENTS; i++)
                {
                    if (!SysSettings.clientNodes[i] || !SysSettings.clientNodes[i].connected())
                    {
                        if (SysSettings.clientNodes[i]) SysSettings.clientNodes[i].stop();
                        SysSettings.clientNodes[i] = wifiServer.available();
                        if (!SysSettings.clientNodes[i]) Serial.println("Couldn't accept client connection!");
                        else 
                        {
                            Serial.print("New client: ");
                            Serial.print(i); Serial.print(' ');
                            Serial.println(SysSettings.clientNodes[i].remoteIP());
                        }
                    }
                }
                if (i >= MAX_CLIENTS) {
                    //no free/disconnected spot so reject
                    wifiServer.available().stop();
                }
            }

            //check clients for data
            for(i = 0; i < MAX_CLIENTS; i++){
                if (SysSettings.clientNodes[i] && SysSettings.clientNodes[i].connected())
                {
                    if(SysSettings.clientNodes[i].available())
                    {
                        //get data from the telnet client and push it to the UART
                        while(SysSettings.clientNodes[i].available()) 
                        {
                            uint8_t inByt;
                            inByt = SysSettings.clientNodes[i].read();
                            SysSettings.isWifiActive = true;
                            // Serial.write(inByt); //echo to serial - just for debugging. Don't leave this on!
                            processIncomingByte(inByt);
                        }
                    }
                }
                else {
                    if (SysSettings.clientNodes[i]) {
                        SysSettings.clientNodes[i].stop();
                    }
                }
            }                    
        }
        else 
        {
            if (settings.wifiMode == 1)
            {
                Serial.println("WiFi disconnected. Bummer!");
                SysSettings.isWifiConnected = false;
                SysSettings.isWifiActive = false;
            }
        }

        if (micros() - lastBroadcast > 1000000ul) //every second send out a broadcast ping
        {
            uint8_t buff[4] = {0x1C,0xEF,0xAC,0xED};
            lastBroadcast = micros();
            wifiUDPServer.beginPacket(broadcastAddr, 17222);
            wifiUDPServer.write(buff, 4);
            wifiUDPServer.endPacket();
        }
    }
}



void SavvyCANServer::calculate_bus_load(void)
{
    if (millis() > (busLoadTimer + 250)) {
        busLoadTimer = millis();
        busLoad[0].busloadPercentage = ((busLoad[0].busloadPercentage * 3) + (((busLoad[0].bitsSoFar * 1000) / busLoad[0].bitsPerQuarter) / 10)) / 4;
        busLoad[1].busloadPercentage = ((busLoad[1].busloadPercentage * 3) + (((busLoad[1].bitsSoFar * 1000) / busLoad[1].bitsPerQuarter) / 10)) / 4;
        //Force busload percentage to be at least 1% if any traffic exists at all. This forces the LED to light up for any traffic.
        if (busLoad[0].busloadPercentage == 0 && busLoad[0].bitsSoFar > 0) busLoad[0].busloadPercentage = 1;
        if (busLoad[1].busloadPercentage == 0 && busLoad[1].bitsSoFar > 0) busLoad[1].busloadPercentage = 1;
        busLoad[0].bitsPerQuarter = settings.CAN0Speed / 4;
        busLoad[1].bitsPerQuarter = settings.CAN1Speed / 4;
        busLoad[0].bitsSoFar = 0;
        busLoad[1].bitsSoFar = 0;
        if(busLoad[0].busloadPercentage > busLoad[1].busloadPercentage){
            //updateBusloadLED(busLoad[0].busloadPercentage);
        } else{
            //updateBusloadLED(busLoad[1].busloadPercentage);
        }
    }
}

void sendFrame(CAN_COMMON *bus, CAN_FRAME &frame)
{
    CAN_FRAME_FD fd;
    int whichBus = 0;
    if (bus == &CAN1) whichBus = 1;
    bus->sendFrame(frame);
    bus->canToFD(frame, fd);
    // sendFrameToFile(fd, whichBus); //copy sent frames to file as well.
    addBits(whichBus, fd);
}

void SavvyCANServer::sendFrameToUSB(CAN_FRAME_FD &frame, int whichBus)
{
    frameobject_t frameobject;

    frameobject.frame = frame;
    frameobject.bus = whichBus;

    if (xQueueSend(receive_queue, &frameobject, 0) != pdPASS)
    {
        // Serial.printf("Failed to send frame id %x to SavvyCAN.  The tubes are blocked\n", frame.id);
    }
}

void SavvyCANServer::packFrameToBeSent(CAN_FRAME_FD &frame, int whichBus)
{
    uint8_t buff[40];
    uint8_t writtenBytes;
    uint8_t temp;
    uint32_t now = micros();

    if (SysSettings.lawicelMode) {
        if (SysSettings.lawicellExtendedMode) {
            Serial.print(micros());
            Serial.print(" - ");
            Serial.print(frame.id, HEX);            
            if (frame.extended) Serial.print(" X ");
            else Serial.print(" S ");
            Serial.printf("BUS%d", whichBus);
            for (int d = 0; d < frame.length; d++) {
                Serial.print(" ");
                Serial.print(frame.data.uint8[d], HEX);
            }
        }else {
            if (frame.extended) {
                Serial.print("T");
                sprintf((char *)buff, "%08x", frame.id);
                Serial.print((char *)buff);
            } else {
                Serial.print("t");
                sprintf((char *)buff, "%03x", frame.id);
                Serial.print((char *)buff);
            }
            Serial.print(frame.length);
            for (int i = 0; i < frame.length; i++) {
                sprintf((char *)buff, "%02x", frame.data.uint8[i]);
                Serial.print((char *)buff);
            }
            if (SysSettings.lawicelTimestamping) {
                uint16_t timestamp = (uint16_t)millis();
                sprintf((char *)buff, "%04x", timestamp);
                Serial.print((char *)buff);
            }
        }
        Serial.write(13);
    } else {
        if (settings.useBinarySerialComm) {
            if (frame.extended) frame.id |= 1 << 31;
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 0; //0 = canbus frame sending
            serialBuffer[serialBufferLength++] = (uint8_t)(now & 0xFF);
            serialBuffer[serialBufferLength++] = (uint8_t)(now >> 8);
            serialBuffer[serialBufferLength++] = (uint8_t)(now >> 16);
            serialBuffer[serialBufferLength++] = (uint8_t)(now >> 24);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id & 0xFF);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 8);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 16);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 24);
            if (frame.length > 8 || frame.length < 0)
            {
                Serial.printf("Warning! frame length (%d) out of bounds\n", frame.length);
            }
            serialBuffer[serialBufferLength++] = frame.length + (uint8_t)(whichBus << 4);
            for (int c = 0; c < frame.length; c++) {
                serialBuffer[serialBufferLength++] = frame.data.uint8[c];
            }
            //temp = checksumCalc(buff, 11 + frame.length);
            temp = 0;
            serialBuffer[serialBufferLength++] = temp;
            //Serial.write(buff, 12 + frame.length);
        } else {
            writtenBytes = sprintf((char *)&serialBuffer[serialBufferLength], "%d - %x", micros(), frame.id);
            serialBufferLength += writtenBytes;
            if (frame.extended) sprintf((char *)&serialBuffer[serialBufferLength], " X ");
            else sprintf((char *)&serialBuffer[serialBufferLength], " S ");
            serialBufferLength += 3;
            writtenBytes = sprintf((char *)&serialBuffer[serialBufferLength], "%i %i", whichBus, frame.length);
            serialBufferLength += writtenBytes;
            for (int c = 0; c < frame.length; c++) {
                writtenBytes = sprintf((char *)&serialBuffer[serialBufferLength], " %x", frame.data.uint8[c]);
                serialBufferLength += writtenBytes;
            }
            sprintf((char *)&serialBuffer[serialBufferLength], "\r\n");
            serialBufferLength += 2;
        }
    }

}

void SavvyCANServer::processIncomingByte(uint8_t in_byte)
{
    static CAN_FRAME build_out_frame;
    static CAN_FRAME_FD build_out_FD;
    static int out_bus;
    static byte buff[20];
    static int step = 0;
    static STATE state = IDLE;
    static uint32_t build_int;
    uint32_t busSpeed = 0;
    uint32_t now = micros();

    uint8_t temp8;
    uint16_t temp16;

    switch (state) {
    case IDLE:
        if(in_byte == 0xF1){
            state = GET_COMMAND;
        }else if(in_byte == 0xE7){
            settings.useBinarySerialComm = true;
            setPromiscuousMode(); //going into binary comm will set promisc. mode too.
        } else{
            Serial.println("THere's no console to receive this byte");
        }
        break;
    case GET_COMMAND:
        switch(in_byte)
        {
        case PROTO_BUILD_CAN_FRAME:
            state = BUILD_CAN_FRAME;
            buff[0] = 0xF1;
            step = 0;
            break;
        case PROTO_TIME_SYNC:
            state = TIME_SYNC;
            step = 0;
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 1; //time sync
            serialBuffer[serialBufferLength++] = (uint8_t) (now & 0xFF);
            serialBuffer[serialBufferLength++] = (uint8_t) (now >> 8);
            serialBuffer[serialBufferLength++] = (uint8_t) (now >> 16);
            serialBuffer[serialBufferLength++] = (uint8_t) (now >> 24);
            break;
        case PROTO_DIG_INPUTS:
            //immediately return the data for digital inputs
            temp8 = 0; //getDigital(0) + (getDigital(1) << 1) + (getDigital(2) << 2) + (getDigital(3) << 3) + (getDigital(4) << 4) + (getDigital(5) << 5);
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 2; //digital inputs
            serialBuffer[serialBufferLength++] = temp8;
            temp8 = checksumCalc(buff, 2);
            serialBuffer[serialBufferLength++]  = temp8;
            state = IDLE;
            break;
        case PROTO_ANA_INPUTS:
            //immediately return data on analog inputs
            temp16 = 0;// getAnalog(0);  // Analogue input 1
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 3;
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(1);  // Analogue input 2
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(2);  // Analogue input 3
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(3);  // Analogue input 4
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(4);  // Analogue input 5
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(5);  // Analogue input 6
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(6);  // Vehicle Volts
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp8 = checksumCalc(buff, 9);
            serialBuffer[serialBufferLength++] = temp8;
            state = IDLE;
            break;
        case PROTO_SET_DIG_OUT:
            state = SET_DIG_OUTPUTS;
            buff[0] = 0xF1;
            break;
        case PROTO_SETUP_CANBUS:
            state = SETUP_CANBUS;
            step = 0;
            buff[0] = 0xF1;
            break;
        case PROTO_GET_CANBUS_PARAMS:
            //immediately return data on canbus params
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 6;
            serialBuffer[serialBufferLength++] = settings.CAN0_Enabled + ((unsigned char) settings.CAN0ListenOnly << 4);
            serialBuffer[serialBufferLength++] = settings.CAN0Speed;
            serialBuffer[serialBufferLength++] = settings.CAN0Speed >> 8;
            serialBuffer[serialBufferLength++] = settings.CAN0Speed >> 16;
            serialBuffer[serialBufferLength++] = settings.CAN0Speed >> 24;
            serialBuffer[serialBufferLength++] = settings.CAN1_Enabled + ((unsigned char) settings.CAN1ListenOnly << 4); //+ (unsigned char)settings.singleWireMode << 6;
            serialBuffer[serialBufferLength++] = settings.CAN1Speed;
            serialBuffer[serialBufferLength++] = settings.CAN1Speed >> 8;
            serialBuffer[serialBufferLength++] = settings.CAN1Speed >> 16;
            serialBuffer[serialBufferLength++] = settings.CAN1Speed >> 24;
            state = IDLE;
            break;
        case PROTO_GET_DEV_INFO:
            //immediately return device information
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 7;
            serialBuffer[serialBufferLength++] = CFG_BUILD_NUM & 0xFF;
            serialBuffer[serialBufferLength++] = (CFG_BUILD_NUM >> 8);
            serialBuffer[serialBufferLength++] = EEPROM_VER;
            serialBuffer[serialBufferLength++] = (unsigned char) 0;
            serialBuffer[serialBufferLength++] = (unsigned char) settings.autoStartLogging;
            serialBuffer[serialBufferLength++] = 0; //was single wire mode. Should be rethought for this board.
            state = IDLE;
            break;
        case PROTO_SET_SW_MODE:
            buff[0] = 0xF1;
            state = SET_SINGLEWIRE_MODE;
            step = 0;
            break;
        case PROTO_KEEPALIVE:
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 0x09;
            serialBuffer[serialBufferLength++] = 0xDE;
            serialBuffer[serialBufferLength++] = 0xAD;
            state = IDLE;
            break;
        case PROTO_SET_SYSTYPE:
            buff[0] = 0xF1;
            state = SET_SYSTYPE;
            step = 0;
            break;
        case PROTO_ECHO_CAN_FRAME:
            state = ECHO_CAN_FRAME;
            buff[0] = 0xF1;
            step = 0;
            break;
        case PROTO_GET_NUMBUSES:
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 12;
            serialBuffer[serialBufferLength++] = 2; //just CAN0 and CAN1 on this hardware
            state = IDLE;
            break;
        case PROTO_GET_EXT_BUSES:
            serialBuffer[serialBufferLength++]  = 0xF1;
            serialBuffer[serialBufferLength++]  = 13;
            for (int u = 2; u < 17; u++) serialBuffer[serialBufferLength++] = 0;
            step = 0;
            state = IDLE;
            break;
        case PROTO_SET_EXT_BUSES:
            state = SETUP_EXT_BUSES;
            step = 0;
            buff[0] = 0xF1;
            break;
        }
        break;
    case BUILD_CAN_FRAME:
        buff[1 + step] = in_byte;
        switch(step)
        {
        case 0:
            build_out_frame.id = in_byte;
            break;
        case 1:
            build_out_frame.id |= in_byte << 8;
            break;
        case 2:
            build_out_frame.id |= in_byte << 16;
            break;
        case 3:
            build_out_frame.id |= in_byte << 24;
            if(build_out_frame.id & 1 << 31)
            {
                build_out_frame.id &= 0x7FFFFFFF;
                build_out_frame.extended = true;
            } else build_out_frame.extended = false;
            break;
        case 4:
            out_bus = in_byte & 3;
            break;
        case 5:
            build_out_frame.length = in_byte & 0xF;
            if(build_out_frame.length > 8) build_out_frame.length = 8;
            break;
        default:
            if(step < build_out_frame.length + 6)
            {
                build_out_frame.data.bytes[step - 6] = in_byte;
            } 
            else
            {
                state = IDLE;
                //this would be the checksum byte. Compute and compare.
                temp8 = checksumCalc(buff, step);
                build_out_frame.rtr = 0;
                if(out_bus == 0) sendFrame(&CAN0, build_out_frame);
                if(out_bus == 1) sendFrame(&CAN1, build_out_frame);
            }
            break;
        }
        step++;
        break;
        case TIME_SYNC:
            state = IDLE;
            break;
        case GET_DIG_INPUTS:
            // nothing to do
            break;
        case GET_ANALOG_INPUTS:
            // nothing to do
            break;
        case SET_DIG_OUTPUTS: //todo: validate the XOR byte
            buff[1] = in_byte;
            Serial.println("Digial outputs not iplemented");
            state = IDLE;
            break;
        case SETUP_CANBUS: //todo: validate checksum
            switch(step)
            {
            case 0:
                build_int = in_byte;
                break;
            case 1:
                build_int |= in_byte << 8;
                break;
            case 2:
                build_int |= in_byte << 16;
                break;
            case 3:
                build_int |= in_byte << 24;
                busSpeed = build_int & 0xFFFFF;
                if(busSpeed > 1000000) busSpeed = 1000000;

                if(build_int > 0)
                {
                    if(build_int & 0x80000000ul) //signals that enabled and listen only status are also being passed
                    {
                        if(build_int & 0x40000000ul)
                        {
                            settings.CAN0_Enabled = true;
                        } else 
                        {
                            settings.CAN0_Enabled = false;
                        }
                        if(build_int & 0x20000000ul)
                        {
                            settings.CAN0ListenOnly = true;
                        } else 
                        {
                            settings.CAN0ListenOnly = false;
                        }
                    } else 
                    {
                        //if not using extended status mode then just default to enabling - this was old behavior
                        settings.CAN0_Enabled = true;
                    }
                    //CAN0.set_baudrate(build_int);
                    settings.CAN0Speed = busSpeed;
                } else { //disable first canbus
                    settings.CAN0_Enabled = false;
                }

                if (settings.CAN0_Enabled)
                {
                    CAN0.begin(settings.CAN0Speed, 255);
                    if (settings.CAN0ListenOnly) CAN0.setListenOnlyMode(true);
                    else CAN0.setListenOnlyMode(false);
                    CAN0.watchFor();
                }
                else CAN0.disable();
                break;
            case 4:
                build_int = in_byte;
                break;
            case 5:
                build_int |= in_byte << 8;
                break;
            case 6:
                build_int |= in_byte << 16;
                break;
            case 7:
                build_int |= in_byte << 24;
                busSpeed = build_int & 0xFFFFF;
                if(busSpeed > 1000000) busSpeed = 1000000;

                if(build_int > 0){
                    if(build_int & 0x80000000){ //signals that enabled and listen only status are also being passed
                        if(build_int & 0x40000000){
                            settings.CAN1_Enabled = true;
                        } else {
                            settings.CAN1_Enabled = false;
                        }
                        if(build_int & 0x20000000){
                            settings.CAN1ListenOnly = true;
                        } else {
                            settings.CAN1ListenOnly = false;
                        }
                    } else {
                        //if not using extended status mode then just default to enabling - this was old behavior
                        settings.CAN1_Enabled = true;
                    }
                    //CAN1.set_baudrate(build_int);
                    settings.CAN1Speed = busSpeed;
                } else{ //disable second canbus
                    settings.CAN1_Enabled = false;
                }

                if (settings.CAN1_Enabled)
                {
                    CAN1.begin(settings.CAN1Speed, 255);
                    delay(2);
                    if (settings.CAN1ListenOnly) CAN1.setListenOnlyMode(true);
                    else CAN1.setListenOnlyMode(false);
                    CAN1.watchFor();
                }
                else CAN1.disable();

                state = IDLE;
                //now, write out the new canbus settings to EEPROM
                EEPROM.writeBytes(0, &settings, sizeof(settings));
                EEPROM.commit();
                //setPromiscuousMode();
                break;
            }
            step++;
            break;
        case GET_CANBUS_PARAMS:
            // nothing to do
            break;
        case GET_DEVICE_INFO:
            // nothing to do
            break;
        case SET_SINGLEWIRE_MODE:
            if(in_byte == 0x10){
            } else {
            }
            EEPROM.writeBytes(0, &settings, sizeof(settings));
            EEPROM.commit();
            state = IDLE;
            break;
        case SET_SYSTYPE:
            settings.sysType = in_byte;
            EEPROM.writeBytes(0, &settings, sizeof(settings));
            EEPROM.commit();
            Serial.println("Not calling loadSettings() because EEPROM is unused");
            // loadSettings();
            state = IDLE;
            break;
        case ECHO_CAN_FRAME:
            buff[1 + step] = in_byte;
            switch(step)
            {
            case 0:
                build_out_frame.id = in_byte;
                break;
            case 1:
                build_out_frame.id |= in_byte << 8;
                break;
            case 2:
                build_out_frame.id |= in_byte << 16;
                break;
            case 3:
                build_out_frame.id |= in_byte << 24;
                if(build_out_frame.id & 1 << 31) {
                    build_out_frame.id &= 0x7FFFFFFF;
                    build_out_frame.extended = true;
                } else build_out_frame.extended = false;
                break;
            case 4:
                out_bus = in_byte & 1;
                break;
            case 5:
                build_out_frame.length = in_byte & 0xF;
                if(build_out_frame.length > 8) build_out_frame.length = 8;
                break;
            default:
                if(step < build_out_frame.length + 6) {
                    build_out_frame.data.bytes[step - 6] = in_byte;
                } else {
                    state = IDLE;
                    //this would be the checksum byte. Compute and compare.
                    temp8 = checksumCalc(buff, step);
                    //if (temp8 == in_byte)
                    //{
                    //if(isConnected) {
                    CAN0.canToFD(build_out_frame, build_out_FD);
                    sendFrameToUSB(build_out_FD, 0);
                    //}
                    //}
                }
                break;
            }
            step++;
            break;
        case SETUP_EXT_BUSES: //setup enable/listenonly/speed for SWCAN, Enable/Speed for LIN1, LIN2
            switch(step)
            {
            case 0:
                build_int = in_byte;
                break;
            case 1:
                build_int |= in_byte << 8;
                break;
            case 2:
                build_int |= in_byte << 16;
                break;
            case 3:
                build_int |= in_byte << 24;
                if(build_int > 0){
                    settings.CAN1FDSpeed = build_int;
                    if (settings.CAN1_Enabled && settings.CAN1_FDMode) 
                    {
                        CAN1.beginFD(settings.CAN1Speed, settings.CAN1FDSpeed);
                        CAN1.watchFor();
                    }
                } else {
                    //settings.SWCAN_Enabled = false;
                }
                break;
            case 4:
                build_int = in_byte;
                break;
            case 5:
                build_int |= in_byte << 8;
                break;
            case 6:
                build_int |= in_byte << 16;
                break;
            case 7:
                build_int |= in_byte << 24;
                break;
            case 8:
                build_int = in_byte;
                break;
            case 9:
                build_int |= in_byte << 8;
                break;
            case 10:
                build_int |= in_byte << 16;
                break;
            case 11:
                build_int |= in_byte << 24;
                state = IDLE;
                //now, write out the new canbus settings to EEPROM
                EEPROM.writeBytes(0, &settings, sizeof(settings));
                EEPROM.commit();
                //setPromiscuousMode();
                break;
            }
        step++;
        break;
    }
}
