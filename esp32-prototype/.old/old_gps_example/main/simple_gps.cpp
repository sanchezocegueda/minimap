#include <stdio.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include <TinyGPS++.h>

const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

static const char *TAG = "main";
#define TX 1
#define RX 3
#define GPSBaud 4800

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Initializing GPS");
    
    /* Port an example from the library. Check components/gps/TinyGPSPlus/examples */
    TinyGPSPlus gps;

    // while (*gpsStream)
    //     if (gps.encode(*gpsStream++))
    //         displayInfo();
}

// void displayInfo()
// {
//   Serial.print(F("Location: ")); 
//   if (gps.location.isValid())
//   {
//     Serial.print(gps.location.lat(), 6);
//     Serial.print(F(","));
//     Serial.print(gps.location.lng(), 6);
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F("  Date/Time: "));
//   if (gps.date.isValid())
//   {
//     Serial.print(gps.date.month());
//     Serial.print(F("/"));
//     Serial.print(gps.date.day());
//     Serial.print(F("/"));
//     Serial.print(gps.date.year());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F(" "));
//   if (gps.time.isValid())
//   {
//     if (gps.time.hour() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.hour());
//     Serial.print(F(":"));
//     if (gps.time.minute() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.minute());
//     Serial.print(F(":"));
//     if (gps.time.second() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.second());
//     Serial.print(F("."));
//     if (gps.time.centisecond() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.centisecond());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.println();
// }
