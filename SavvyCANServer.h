#ifndef SAVVYCAN_H
#define SAVVYCAN_H

#include "../AveragingMethodTimer.h"
#include "../board.h"
#include "WiFi.h"
#include "can_common.h"
#include "config.h"
#include "freertos/queue.h"

#ifndef SAVVYCAN_NUM_BUSES
#error                                                                         \
    "define a number of CAN buses for SavvyCANServer.  This sets the maximum bus number acceptable by `sendFrameToUSB(...)`"
#endif

// Buffer for CAN frames when sending over wifi. This allows us to build up a
// multi-frame packet that goes over the air all at once. This is much more
// efficient than trying to send a new TCP/IP packet for each and every frame.
// It delays frames from getting to the other side a bit but that's life.

// MTU (Maximum Transmit Unit) for WiFi media is 1500 bytes, so we make the
// buffer a little smaller than that to leave room for headers.
#define WIFI_BUFF_SIZE 1300

// The upper limit on the number of bytes needed to store a CAN frame plus all of its metadata
// (CAN ID, timestamp, bus, TX/RX, ...)
#define SERIAL_BUFFER_FRAME_SIZE 40

// Number of microseconds between hard flushes of the serial buffer (if not in
// wifi mode) or the wifi buffer (if in wifi mode) This keeps the latency more
// consistent. Otherwise the buffer could partially fill and never send.
#define SER_BUFF_FLUSH_INTERVAL 50000

// The number of frames that can occupy the SavvyCANServer receive queue
#define SAVVYCANSERVER_RX_QUEUE_SIZE_FRAMEOBJECTS 50

class SavvyCANBusConfiguration {
  public:
    uint32_t Speed;
    uint32_t FDSpeed;
    boolean Enabled;
    boolean FDMode;
    boolean ListenOnly; // if true we don't allow any messing with the bus but
                        // rather just passively monitor.

    void apply_to(CAN_COMMON *bus);
};

enum frame_direction_e
{
    FRAME_DIRECTION_RX = 0,
    FRAME_DIRECTION_TX
};

/*
This should probably go in CAN_COMMON
This is used on SavvyCANServer's receive queue to keep frames mapped to a bus
and an RX/TX direction
*/
typedef struct {
    CAN_FRAME_FD        frame;
    int                 bus_number;
    frame_direction_e   direction;
} frameobject_t;

class SavvyCANServer {
  public:
    SavvyCANServer(HardwareSerial &console, WiFiServer &server);
    CAN_COMMON *can_buses[SAVVYCAN_NUM_BUSES];

    void setup(void);
    void calculate_bus_load(void);
    void processIncomingByte(uint8_t in_byte);
    HardwareSerial &console;
    WiFiServer &wifiServer;
    void setPromiscuousMode(void);
    void handleWiFi(void);
    void handleSerial(void);
    void packFrameToBeSent(CAN_FRAME_FD &frame, int whichBus);
    QueueHandle_t receive_queue;
    bool rx_queue_overrun;
    int registerBus(CAN_COMMON &bus);
    void    task_1kHz(void); 

  private:
    byte serialBuffer[WIFI_BUFF_SIZE];
    bool serial_buffer_has_room();
    int serialBufferLength = 0; // not creating a ring buffer. The buffer should
                                // be large enough to never overflow
    uint32_t lastFlushMicros = 0;
    uint32_t lastBroadcast = 0;
    uint32_t busLoadTimer;
};

#define MAX_CLIENTS 1
#define CFG_BUILD_NUM 362
#define EEPROM_VER 0x21

struct SystemSettings {
    boolean useSD; // should we attempt to use the SDCard? (No logging possible
                   // otherwise)
    boolean logToFile; // are we currently supposed to be logging to file?
    boolean SDCardInserted;
    uint8_t LED_CANTX;
    uint8_t LED_CANRX;
    uint8_t LED_LOGGING;
    boolean txToggle; // LED toggle values
    boolean rxToggle;
    boolean logToggle;
    boolean lawicelMode;
    boolean lawicellExtendedMode;
    boolean lawicelAutoPoll;
    boolean lawicelTimestamping;
    int lawicelPollCounter;
    boolean lawicelBusReception[NUM_BUSES]; // does user want to see messages
                                            // from this bus?
    int8_t numBuses; // number of buses this hardware currently supports.
    WiFiClient clientNodes[MAX_CLIENTS];
    boolean isWifiConnected;
    boolean isWifiActive;
};

enum FILEOUTPUTTYPE { NONE = 0, BINARYFILE = 1, GVRET = 2, CRTD = 3 };

struct EEPROMSettings {
    uint8_t version;

    uint32_t CAN0Speed;
    uint32_t CAN1Speed;
    uint32_t CAN1FDSpeed;
    boolean CAN0_Enabled;
    boolean CAN1_Enabled;
    boolean CAN1_FDMode;

    boolean useBinarySerialComm; // use a binary protocol on the serial link or
                                 // human readable format?
    FILEOUTPUTTYPE fileOutputType; // what format should we use for file output?

    char fileNameBase[30]; // Base filename to use
    char fileNameExt[4];   // extension to use
    uint16_t fileNum; // incrementing value to append to filename if we create a
                      // new file each time
    boolean appendFile; // start a new file every power up or append to current?
    boolean autoStartLogging; // should logging start immediately on start up?

    uint8_t logLevel; // Level of logging to output on serial line
    uint8_t sysType;

    uint16_t
        valid; // stores a validity token to make sure EEPROM is not corrupt

    boolean CAN0ListenOnly; // if true we don't allow any messing with the bus
                            // but rather just passively monitor.
    boolean CAN1ListenOnly;

    // if we're using WiFi then output to serial is disabled (it's far too slow
    // to keep up)
    uint8_t wifiMode; // 0 = don't use wifi, 1 = connect to an AP, 2 = Create an
                      // AP
    uint8_t SSID[32]; // null terminated string for the SSID
    uint8_t WPA2Key[64]; // Null terminated string for the key. Can be a
                         // passphase or the actual key
};

enum STATE {
    IDLE,
    GET_COMMAND,
    BUILD_CAN_FRAME,
    TIME_SYNC,
    GET_DIG_INPUTS,
    GET_ANALOG_INPUTS,
    SET_DIG_OUTPUTS,
    SETUP_CANBUS,
    GET_CANBUS_PARAMS,
    GET_DEVICE_INFO,
    SET_SINGLEWIRE_MODE,
    SET_SYSTYPE,
    ECHO_CAN_FRAME,
    SETUP_EXT_BUSES
};

enum GVRET_PROTOCOL {
    PROTO_BUILD_CAN_FRAME = 0,
    PROTO_TIME_SYNC = 1,
    PROTO_DIG_INPUTS = 2,
    PROTO_ANA_INPUTS = 3,
    PROTO_SET_DIG_OUT = 4,
    PROTO_SETUP_CANBUS = 5,
    PROTO_GET_CANBUS_PARAMS = 6,
    PROTO_GET_DEV_INFO = 7,
    PROTO_SET_SW_MODE = 8,
    PROTO_KEEPALIVE = 9,
    PROTO_SET_SYSTYPE = 10,
    PROTO_ECHO_CAN_FRAME = 11,
    PROTO_GET_NUMBUSES = 12,
    PROTO_GET_EXT_BUSES = 13,
    PROTO_SET_EXT_BUSES = 14
};


/*
Reports sent and received frames to a SavvyCANServer, copying them into a queue
*/
class SavvyCANListener : public CANListener
{
public:
    SavvyCANListener(CAN_COMMON &bus, QueueHandle_t &serverReceiveQueue);
    void gotFrame(CAN_FRAME& frame);
    void sentFrame(CAN_FRAME& frame);
private:
    QueueHandle_t   &serverReceiveQueue;
    CAN_COMMON      &bus;
};

#endif

