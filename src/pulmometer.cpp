#include "credentials.h"
#include <Adafruit_LIS3MDL.h>
#include <ElegantOTA.h>
#include <HTTPClient.h>
#include <WiFi.h>

#define SERVER        "192.168.7.207:8086"  // address of influx database
#define WIFI_HOSTNAME "pulmometer"

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

static Adafruit_LIS3MDL lis3mdl;

static int read_sensor_mag () {
    lis3mdl.read();  // get X Y and Z data at once
    int mag = (int)sqrt ((double)lis3mdl.x * (double)lis3mdl.x + (double)lis3mdl.y * (double)lis3mdl.y +
                         (double)lis3mdl.z * (double)lis3mdl.z);
#ifdef DEBUG
    Serial.printf ("%6d = |(%5d, %5d, %5d)|\n", mag, lis3mdl.x, lis3mdl.y, lis3mdl.z);
#endif
    return mag;
}

// configurable options to adjust sensitivity
static long  CALIBRATION_DURATION = 1000 * 60 * 5;  // five minutes
static float HYSTERESIS           = 0.3;            // [0.0 .. 0.5)

// set initial values, to be overwritten in calibration
static int MAX_MAGNITUDE = INT_MIN;
static int MIN_MAGNITUDE = INT_MAX;
static int THRESH_LOW    = 0;
static int THRESH_HIGH   = 0;

static void calibrate (long duration) {
    long stop = millis() + duration;
    Serial.printf ("Calibrating...\n");
    Serial.printf ("%7s %7s %7s %7s\n", "time", "curr", "min", "max");
    Serial.printf ("%7s %7s %7s %7s\n", "-------", "-------", "-------", "-------");
    long now;
    while ((now = millis()) < stop) {
        int mag = read_sensor_mag();
        if (mag < MIN_MAGNITUDE)
            MIN_MAGNITUDE = mag;
        if (mag > MAX_MAGNITUDE)
            MAX_MAGNITUDE = mag;
        Serial.printf ("\r%7ld %7d %7d %7d", stop - now, mag, MIN_MAGNITUDE, MAX_MAGNITUDE);
    }
    THRESH_LOW  = (int)(MIN_MAGNITUDE + HYSTERESIS * (MAX_MAGNITUDE - MIN_MAGNITUDE));
    THRESH_HIGH = (int)(MAX_MAGNITUDE - HYSTERESIS * (MAX_MAGNITUDE - MIN_MAGNITUDE));
    Serial.printf ("\nDone.  MIN = %d, MAX = %d\n", MIN_MAGNITUDE, MAX_MAGNITUDE);
}

static bool zero_crossing (int val, bool & state) {
    if ((state && val < THRESH_LOW) || (!state && val > THRESH_HIGH)) {
        state = !state;
        return true;
    }
    else
        return false;
}

static SemaphoreHandle_t xMutex;
volatile static int      counter = 0;

static void sensor_task (void * parameter) {
    bool state     = false;  // arbitrary initial value
    bool state_set = false;  // tracks "arbitraryness" of state
    while (true) {
        int mag = read_sensor_mag();
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

ELEGANTOTA_WEBSERVER server (80);

void setup () {
    Serial.begin (112500);
    delay (1000);  // Safety

    Serial.print ("Connecting WiFi");
    WiFi.begin (WIFI_SSID, WIFI_PSK);  // defined in credentials.h
    WiFi.waitForConnectResult();       // so much neater than those stupid loops and dots
    Serial.println (WiFi.localIP());

#if ELEGANTOTA_USE_ASYNC_WEBSERVER == 1
    #define REQUEST_ARG AsyncWebServerRequest * request
    #define DO_SEND     request->send
#else
    #define REQUEST_ARG
    #define DO_SEND server.send
#endif
    server.on ("/", [] (REQUEST_ARG) {
        String html = "";
        html += "<html>";
        html += WIFI_HOSTNAME;
        html += "<br/>MIN = " + String (MIN_MAGNITUDE) + "; MAX = " + String (MAX_MAGNITUDE) +
                "; THRESH_LOW = " + String (THRESH_LOW) + "; THRESH_HIGH = " + String (THRESH_HIGH);
        html += "<br/><a href='/reset'>Reset</a><br/><a href='/update'>Update</a></html>";
        DO_SEND (200, "text/html", html);
    });

    server.on ("/reset", [] (REQUEST_ARG) {
        DO_SEND (200, "text/html", "<html><head><meta http-equiv=\"Refresh\" content=\"0; url='/'\"/></head></html>");
        ESP.restart();
    });

    ElegantOTA.begin (&server);  // Start ElegantOTA
    server.begin();
    Serial.println ("HTTP server started");

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

    calibrate (CALIBRATION_DURATION);

    xMutex = xSemaphoreCreateMutex();
    xTaskCreate (sensor_task, "Sensor Task", 4096, NULL, 1, NULL);
    xTaskCreate (recorder_task, "Recorder Task", 4096, NULL, 1, NULL);
    Serial.println ("tasks created");
}

void loop () {
#if !(ELEGANTOTA_USE_ASYNC_WEBSERVER == 1)
    server.handleClient();
#endif
    ElegantOTA.loop();
}
