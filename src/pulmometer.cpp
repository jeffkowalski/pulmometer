#include <Adafruit_LIS3MDL.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include "credentials.h"


#define SERVER "192.168.7.20:8086"  // address of influx database
static void record_to_database (int counter) {
    WiFiClient client;
    HTTPClient http;

    Serial.print ("[HTTP] begin...\n");
    http.begin (client, "http://" SERVER "/write?db=pulmometer");
    http.addHeader ("Accept", "*/*");
    http.addHeader ("Content-Type", "application/json");

    Serial.print ("[HTTP] POST...\n");
    static char postval[256];
    sprintf (postval, "breaths value=%d", counter);
    Serial.println (postval);
    int httpCode = http.POST (postval);

    // httpCode will be negative on error
    if (httpCode > 0)
        Serial.printf ("[HTTP] POST... code: %d\n", httpCode);
    else
        Serial.printf ("[HTTP] POST... failed, error: %s\n", http.errorToString (httpCode).c_str());

    http.end();
}


#define MAX_MAGNITUDE 9000
#define MIN_MAGNITUDE 7500
#define HYSTERESIS    0.3  // [0.0 .. 0.5)
#define THRESH_LOW    (long)(MIN_MAGNITUDE + HYSTERESIS * (MAX_MAGNITUDE - MIN_MAGNITUDE))
#define THRESH_HIGH   (long)(MAX_MAGNITUDE - HYSTERESIS * (MAX_MAGNITUDE - MIN_MAGNITUDE))
static bool zero_crossing (long val, bool & state) {
    if ((state && val < THRESH_LOW) || (!state && val > THRESH_HIGH)) {
        state = !state;
        return true;
    } else
        return false;
}


static Adafruit_LIS3MDL lis3mdl;


static long read_sensor_mag() {
    lis3mdl.read();  // get X Y and Z data at once
    long mag = (long)sqrt ((double)lis3mdl.x * (double)lis3mdl.x + (double)lis3mdl.y * (double)lis3mdl.y +
                           (double)lis3mdl.z * (double)lis3mdl.z);
#ifdef DEBUG
    Serial.printf ("%6ld = |(%5d, %5d, %5d)|\n", mag, lis3mdl.x, lis3mdl.y, lis3mdl.z);
#endif
    return mag;
}


static SemaphoreHandle_t xMutex;
volatile static int      counter = 0;


static void sensor_task (void * parameter) {
    bool state     = false;  // arbitrary initial value
    bool state_set = false;  // tracks "arbitraryness" of state
    while (true) {
        long mag = read_sensor_mag();
        if (zero_crossing (mag, state) &&  // modifies state, returns true if changed
            state && state_set) {
            Serial.println ("breath");
            xSemaphoreTake (xMutex, portMAX_DELAY);
            ++counter;
            xSemaphoreGive (xMutex);
        }
        state_set = true;
#ifdef DEBUG
        vTaskDelay (500 / portTICK_PERIOD_MS);
#endif
    }
}


static void recorder_task (void * parameter) {
    while (true) {
        if (WiFi.status() != WL_CONNECTED)
            ESP.restart();

        xSemaphoreTake (xMutex, portMAX_DELAY);
        int last_counter = counter;
        counter          = 0;
        xSemaphoreGive (xMutex);

        Serial.printf ("recording %d\n", last_counter);
        record_to_database (last_counter);

#define REPORTING_PERIOD (1000 * 60)  // 1 minute
        vTaskDelay (REPORTING_PERIOD / portTICK_PERIOD_MS);
    }
}


void setup() {
    Serial.begin (112500);
    delay (1000);  // Safety

    Serial.print ("Connecting WiFi");
    WiFi.begin (WIFI_SSID, WIFI_PSK);  // defined in credentials.h
    WiFi.waitForConnectResult();       // so much neater than those stupid loops and dots
    Serial.println (WiFi.localIP());

    if (!lis3mdl.begin_I2C()) {  // hardware I2C mode
        Serial.println ("Failed to find LIS3MDL chip");
        while (1) delay (10);
    }
    Serial.println ("LIS3MDL Found!");

    lis3mdl.setIntThreshold (500);
    lis3mdl.configInterrupt (false,
                             false,
                             true,   // enable z axis
                             true,   // polarity
                             false,  // don't latch
                             true);  // enabled!

    xMutex = xSemaphoreCreateMutex();
    xTaskCreate (sensor_task, "Sensor Task", 4096, NULL, 1, NULL);
    xTaskCreate (recorder_task, "Recorder Task", 4096, NULL, 1, NULL);
    Serial.println ("tasks created");
}

void loop() {}
