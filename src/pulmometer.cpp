#include "credentials.h"
#include <Adafruit_LIS3MDL.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <cmath>
#include <freertos/task.h>

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

static int read_sensor_magnitude () {
    lis3mdl.read();  // get X Y and Z data at once
    int magnitude = (int)sqrt ((double)lis3mdl.x * (double)lis3mdl.x + (double)lis3mdl.y * (double)lis3mdl.y +
                               (double)lis3mdl.z * (double)lis3mdl.z);
#ifdef DEBUG
    Serial.printf ("%6d = |(%5d, %5d, %5d)|\n", magnitude, lis3mdl.x, lis3mdl.y, lis3mdl.z);
#endif
    return magnitude;
}

// configurable options to adjust sensitivity
static long  CALIBRATION_DURATION = 1000 * 60 * 5;  // five minutes, in ms
static float HYSTERESIS           = 0.2;            // tuning parameter: thresholds as a factor(fraction) of std deviation

static Preferences prefs;

static class Thresholds {
  public:
    int min;
    int max;
    int check;

    Thresholds()
        : min (0)
        , max (0)
        , check (0) {}

    void set_check (void) { check = min ^ max; }

    bool is_valid (void) const { return check && check == (min ^ max); }
} thresholds;

static void calibrate () {
    long stop = millis() + CALIBRATION_DURATION;
    Serial.printf ("Calibrating...\n");

    int   min_magnitude = INT_MAX;
    int   max_magnitude = INT_MIN;
    float sum           = 0;  // To calculate mean
    float sum_sq        = 0;  // To calculate variance
    long  sample_count  = 0;

    while (millis() < stop) {
        int magnitude = read_sensor_magnitude();
        sum += magnitude;
        sum_sq += magnitude * magnitude;
        sample_count++;

        if (magnitude < min_magnitude)
            min_magnitude = magnitude;
        if (magnitude > max_magnitude)
            max_magnitude = magnitude;

        Serial.printf ("\rSamples: %ld | Curr: %d | Min: %d | Max: %d",
                       sample_count,
                       magnitude,
                       min_magnitude,
                       max_magnitude);
        vTaskDelay (500 / portTICK_PERIOD_MS);
    }

    // Calculate mean and standard deviation
    float mean     = sum / sample_count;
    float variance = (sum_sq / sample_count) - (mean * mean);
    float std_dev  = sqrt (variance);

    // Set thresholds using standard deviations from the mean
    thresholds.min = (int)(mean - HYSTERESIS * std_dev);
    thresholds.max = (int)(mean + HYSTERESIS * std_dev);

    // Ensure valid thresholds
    if (thresholds.min < min_magnitude)
        thresholds.min = min_magnitude;
    if (thresholds.max > max_magnitude)
        thresholds.max = max_magnitude;

    thresholds.set_check();

    Serial.printf ("\nCalibration complete.\n");
    Serial.printf ("Mean = %.2f, Std Dev = %.2f\n", mean, std_dev);
    Serial.printf ("Thresholds: min = %d, max = %d, check = %d\n",
                   thresholds.min,
                   thresholds.max,
                   thresholds.check);
}

static bool zero_crossing (int val, bool & state) {
    if ((state && val < thresholds.min) || (!state && val > thresholds.max)) {
        state = !state;
        return true;
    }
    else
        return false;
}

static SemaphoreHandle_t xMutex;
volatile static long     counter        = 0;
volatile static long     last_counter   = 0;
volatile static int      last_magnitude = 0;

static void sensor_task (void * parameter) {
    bool state     = false;  // arbitrary initial value
    bool state_set = false;  // tracks "arbitraryness" of state
    while (true) {
        last_magnitude = read_sensor_magnitude();
        if (zero_crossing (last_magnitude, state) &&  // NOTE: modifies state, returns true if changed
            state && state_set) {
            Serial.println ("breath");
            ++counter;
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

        long difference = counter - last_counter;
#ifdef DEBUG
        Serial.printf ("recording %d\n", difference);
#endif
        record_to_database (difference);
        last_counter += difference;

#define REPORTING_PERIOD (1000 * 60)  // 1 minute
        vTaskDelay (REPORTING_PERIOD / portTICK_PERIOD_MS);
    }
}

ELEGANTOTA_WEBSERVER server (80);

void setup () {
    Serial.begin (112500);
    delay (5 * 1000);  // Safety

    Serial.print ("Connecting WiFi");
    WiFi.begin (WIFI_SSID, WIFI_PSK);  // defined in credentials.h
    WiFi.waitForConnectResult();       // so much neater than those stupid loops and dots
    Serial.println (WiFi.localIP());

    if (!MDNS.begin (WIFI_HOSTNAME))
        Serial.println ("Error starting mDNS");
    else
        Serial.println ("mDNS responder started: http://" WIFI_HOSTNAME ".local");

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
        html += "<br/>Last magnitude = " + String (last_magnitude) + "; counter = " + String (counter);
        html += "<br/>Thresholds: min = " + String (thresholds.min) + "; max = " + String (thresholds.max) +
                "; check = " + String (thresholds.check);
        html += "<br/><a href='/reboot'>Reboot</a><br/><a href='/calibrate'>Calibrate</a><br/><a href='/update'>Update firmware</a></html>";
        DO_SEND (200, "text/html", html);
    });

    server.on ("/reboot", [] (REQUEST_ARG) {
        DO_SEND (200, "text/html", "<html><head><meta http-equiv=\"Refresh\" content=\"0; url='/'\"/></head></html>");
        ESP.restart();
    });

    server.on ("/calibrate", [] (REQUEST_ARG) {
        calibrate();
        prefs.putBytes ("thresholds", reinterpret_cast<byte *> (&thresholds), sizeof (thresholds));
        Serial.printf ("\nThresholds: min = %d, max = %d, check = %d\n", thresholds.min, thresholds.max, thresholds.check);
        DO_SEND (200, "text/html", "<html><head><meta http-equiv=\"Refresh\" content=\"0; url='/'\"/></head></html>");
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

    prefs.begin ("calibration");  // namespace
    prefs.getBytes ("thresholds", &thresholds, sizeof (thresholds));
    if (!thresholds.is_valid()) {
        calibrate();
        prefs.putBytes ("thresholds", reinterpret_cast<byte *> (&thresholds), sizeof (thresholds));
    }
    Serial.printf ("\nThresholds: min = %d, max = %d, check = %d\n", thresholds.min, thresholds.max, thresholds.check);

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
