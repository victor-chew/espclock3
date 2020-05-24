# espclock3
Internet-enabled Analog Clock using ESP8266 and ATtiny85

### Description
This is version 3 of my [ESPClock](https://www.randseq.org/2016/10/hacking-analog-clock-to-sync-with-ntp.html) project.

The clock gets accurate time information from the Internet, makes sure the physical clock time is up-to-date. It also automatically detects your current timezone via browser geolocation and deals with daylight saving adjustments with no user intervention.

V1 uses the ESP-12E development board to drive a cheap $2 Ikea analog clock.

V2 uses the WeMOS D1 Mini and ATtiny85 to reduce power draw. That design yields a runtime of ~1 month.

V3 uses the ESP-07, ATtiny85 and PCF8563 RTC to reduce the power draw even further. This design yields at least 4 months of runtime on a set of readily-available 4 x 1.2V NiMH rechargables.

### Links:
* [Design](https://www.randseq.org/2020/05/espclock3-final-version.html)
