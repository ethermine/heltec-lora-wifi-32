/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>  
#include "SSD1306.h" 

SSD1306 display(0x3c, 4, 15);

#define BUILTIN_LED 25

osjob_t initjob;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
// 70B3D57ED000B2AD
// 70 B3 D5 7E D0 00 B2 AD
static const u1_t PROGMEM APPEUI[8]={ 0xAD , 0xB2, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
// EF08457EB0005511
// EF 08 45 7E B0 00 55 11
static const u1_t PROGMEM DEVEUI[8]={ 0x11, 0x55, 0x00, 0xB0, 0x7E, 0x45, 0x08, 0xEF };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
// 2B7E152628AED2A6ABF7158809CF4F3C
static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x26, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

#define SS 18 // ESP32 GPIO18 (Pin18) – SX1276 NSS (Pin19) SPI Chip Select Input
#define MOSI 27 // ESP32 GPIO27 (Pin27) – SX1276 MOSI (Pin18) SPI Data Input
#define MISO 19 // ESP32 GPIO19 (Pin19) – SX1276 MISO (Pin17) SPI Data Output
#define SCK 5 // ESP32 GPIO5 (Pin5) – SX1276 SCK (Pin16) SPI Clock Input
#define RST 14 // ESP32 GPIO14 (Pin14) – SX1276 NRESET (Pin7) Reset Trigger Input
#define DIO0 26 // ESP32 GPIO26 (Pin15) – SX1276 DIO0 (Pin8) used by LMIC for IRQ RX_Done & TX_Done
#define DIO1 33 // ESP32 GPIO33 (Pin13) – SX1276 DIO1 (Pin9) used by LMIC for IRQ RX_Timeout
#define DIO2 32 // ESP32 GPIO32 (Pin12) – SX1276 DIO2 (Pin10) not used by LMIC

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = SS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RST,
    .dio = {DIO0, DIO0, DIO2},
};

void onEvent (ev_t ev) {
    Serial.print(ev);
    Serial.print(" / ");
    Serial.print(os_getTime());
    Serial.print(": ");
    display.clear();
    display.drawString(0, 20, "DevEUI : EF 08 45 7E B0 00 55 11");
    display.display();
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            display.drawString(0, 0, "EV_SCAN_TIMEOUT");
            display.display();
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            display.drawString(0, 0, "EV_BEACON_FOUND");
            display.display();
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            display.drawString(0, 0, "EV_BEACON_MISSED");
            display.display();
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            display.drawString(0, 0, "EV_BEACON_TRACKED");
            display.display();
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            display.drawString(0, 0, "EV_JOINING");
            display.display();
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            display.drawString(0, 0, "EV_JOINED");
            display.display();
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            display.drawString(0, 0, "EV_RFU1");
            display.display();
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            display.drawString(0, 0, "EV_JOIN_FAILED");
            display.display();
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            display.drawString(0, 0, "EV_REJOIN_FAILED");
            display.display();
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            display.drawString(0, 0, "EV_TXCOMPLETE");
            display.display();
            if (LMIC.txrxFlags & TXRX_ACK){
              Serial.println(F("Received ack"));
            }
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            display.drawString(0, 0, "EV_LOST_TSYNC");
            display.display();
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            display.drawString(0, 0, "EV_RESET");
            display.display();
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            display.drawString(0, 0, "EV_RXCOMPLETE");
            display.display();
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            display.drawString(0, 0, "EV_LINK_DEAD");
            display.display();
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            display.drawString(0, 0, "EV_LINK_ALIVE");
            display.display();
            break;
         default:
            Serial.println(F("Unknown event"));
            display.drawString(0, 0, "Unknown event");
            display.display();
            break;
    }
}

void do_send(osjob_t* j){
    display.clear();
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        display.drawString(0, 0, "OP_TXRXPEND, not sending");
        display.display();
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
        display.drawString(0, 0, "Packet queued");
        display.display();
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));
    pinMode(16,OUTPUT);
    digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
    delay(50); 
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
    display.init();
    display.flipScreenVertically();  
    display.setFont(ArialMT_Plain_10);
    display.clear();

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    //LMIC_reset();
    
    // Start job (sending automatically starts OTAA too)
    //do_send(&sendjob);
    os_setCallback(&initjob, initfunc);
}

void loop() {
    os_runloop();
    display.drawString(0, 20, "DevEUI : EF 08 45 7E B0 00 55 11");
    display.display();
}

// initial job 
static void initfunc (osjob_t* j) { 
  // reset MAC state 
  LMIC_reset(); 
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // start joining 
  LMIC_startJoining(); 
  // init done - onEvent() callback will be invoked... 
}

