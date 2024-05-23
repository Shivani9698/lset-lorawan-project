#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 26
#define Relay 2
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  900       /* Time ESP32 will go to sleep (in seconds) */

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

static const u1_t PROGMEM APPEUI[8] = {0x65, 0x87, 0xAE, 0x67, 0x45, 0x43, 0x65, 0x87};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8] = {0x25, 0x7A, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = {0x62, 0x81, 0xE4, 0xAC, 0xE7, 0xCD, 0xC9, 0xE3, 0x51, 0x48, 0xD3, 0x39, 0xC7, 0xA3, 0x31, 0xEB};
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

uint8_t Data;
static uint8_t mydata[1] = {0x00};
static osjob_t sendjob;

const unsigned TX_INTERVAL = 15; // 15 seconds

void readTemperature() {
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    Serial.print("Temperature: ");
    Serial.println(tempC);
    mydata[0] = (uint8_t) tempC; // Convert float to uint8_t
}

const lmic_pinmap lmic_pins = {
    .nss = 15,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 17,
    .dio = {4, 33, 32},
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              for (int i = 0; i < LMIC.dataLen; i++) {
                  if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                      Data = (LMIC.frame[LMIC.dataBeg + i]);
                  }
              }
              Serial.print("Downlink Data : ");
              Serial.println(Data);
              if (Data == 1) {
                  digitalWrite(Relay, HIGH);
                  Serial.println("Relay on");
              } else {
                  digitalWrite(Relay, LOW);
                  Serial.println("Relay off");
              }
            }

            // Schedule the next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

            // After sending data and processing the response, go to sleep
            Serial.println("Going to sleep now");
            delay(100);
            esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
            esp_deep_sleep_start();
            break;
        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        readTemperature();
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
}

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup() {
    pinMode(Relay, OUTPUT);
    Serial.begin(115200);
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));

    // Print the wakeup reason for ESP32
    print_wakeup_reason();

    Serial.println(F("Starting"));

    os_init();
    LMIC_reset();
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF7,14);

    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
