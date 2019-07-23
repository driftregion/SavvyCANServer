#include "SavvyCANServer.h"

SavvyCANListener::SavvyCANListener(CAN_COMMON &bus, QueueHandle_t &serverReceiveQueue) : serverReceiveQueue(serverReceiveQueue),
                                                                                         bus(bus) {
}

void SavvyCANListener::gotFrame(CAN_FRAME &frame) {
    frameobject_t frameobject;

    frameobject.frame = frame;
    frameobject.bus_number = bus.bus_number;
    frameobject.direction = FRAME_DIRECTION_RX;
    xQueueSendFromISR(serverReceiveQueue, &frameobject, 0);
}

void SavvyCANListener::sentFrame(CAN_FRAME &frame) {
    frameobject_t frameobject;

    frameobject.frame = frame;
    frameobject.bus_number = bus.bus_number;
    frameobject.direction = FRAME_DIRECTION_TX;
    xQueueSendFromISR(serverReceiveQueue, &frameobject, 0);
}