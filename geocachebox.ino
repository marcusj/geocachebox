#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <HMC5983.h>
#include <Wire.h>
#include <TinyGPS++.h>


/////////////////////////////////////////////////////////////////////////////
// Put your geocache coordinates in here.
// South and West are negative numbers
#define DESTINATION_LONGITUDE -1.21984
#define DESTINATION_LATITUDE  37.83859
/////////////////////////////////////////////////////////////////////////////


#define PIN_NEOPIXEL_OUT  6
#define PIN_GPS_RX        8
#define PIN_GPS_TX        9 // Not really used but SoftwareSerial wants it

#define LED_SOUTH_EAST  0
#define LED_SOUTH       1
#define LED_SOUTH_WEST  2
#define LED_WEST        3
#define LED_NORTH_WEST  4
#define LED_NORTH       5
#define LED_NORTH_EAST  6
#define LED_EAST        7
#define LED_DISTANCE    8
#define LED_GPS_STATUS  9
#define MIN_COMPASS_LED 0
#define MAX_COMPASS_LED 7

// Tradeoff sunlight readability vs. battery life and
// Arduino 5V regulator current budget
#define LED_INTENSITY   20

#define COMPASS_UPDATE_MILLISECONDS 500

#define GPS_LOST_SECONDS    30
#define GPS_STATUS_NO_LOCK    0
#define GPS_STATUS_TIME       1
#define GPS_STATUS_POSITION   2

#define EARTH_RADIUS_METRES 6371000

// Global variables :-/
Adafruit_NeoPixel strip =
  Adafruit_NeoPixel(10, PIN_NEOPIXEL_OUT, NEO_GRB + NEO_KHZ800);
HMC5983 compass;
SoftwareSerial gpsSerial(PIN_GPS_RX, PIN_GPS_TX);
TinyGPSPlus gps;
int gpsStatus = GPS_STATUS_NO_LOCK;

/////////////////////////////////////////////////////////////////////////////
void setup()
{
  strip.begin();
  compass.begin();
  Serial.begin(115200);
  gpsSerial.begin(9600);
  strip.show(); // Initialize all pixels to 'off'
}

/////////////////////////////////////////////////////////////////////////////
void loop()
{
  float bearingToDestinationDegrees = 999.0;
  while (gpsSerial.available() > 0)
  {
    if (gps.encode(gpsSerial.read()))
    {
      float bearing = calculateHeadingToDestination();
      if(bearing >= 0.0 && bearing <= 360.0)
      {
        bearingToDestinationDegrees = bearing;
      }
    }
  }

  float compassReadingDegrees = compass.read();
  if (compassReadingDegrees == -999)
  {
    Serial.println("Reading error, discarded");
  }
  else
  {
    showCompassLED(compassReadingDegrees, bearingToDestinationDegrees);
  }
}

/////////////////////////////////////////////////////////////////////////////
void updateGPSStatusLED()
{
  static long lastGPSOK = -1;
  uint32_t statusColour = strip.Color(LED_INTENSITY, 0, 0);

  switch(gpsStatus)
  {
    case GPS_STATUS_TIME:
      statusColour = strip.Color(LED_INTENSITY, LED_INTENSITY, 0);
      break;

    case GPS_STATUS_POSITION:
      statusColour = strip.Color(0, LED_INTENSITY, 0);
      break;
  }

  strip.setPixelColor(LED_GPS_STATUS, statusColour);
  strip.show();
}

/////////////////////////////////////////////////////////////////////////////
float calculateHeadingToDestination()
{
  float retVal = 999.0;
  static float lastLongitude = 0.0;
  static float lastLatitude = 0.0;

  if (gps.time.isValid() && gps.time.isUpdated())
  {
    if(gpsStatus == GPS_STATUS_NO_LOCK)
    {
      gpsStatus = GPS_STATUS_TIME;
      updateGPSStatusLED();
    }
  }
  
  if (gps.location.isValid() && gps.location.isUpdated())
  {
    float gpsLon = gps.location.lng();
    float gpsLat = gps.location.lat();

    if(gpsStatus != GPS_STATUS_POSITION)
    {
      gpsStatus = GPS_STATUS_POSITION;
      updateGPSStatusLED();
    }

    if (lastLongitude != gpsLon || lastLatitude != gpsLat)
    {
      Serial.print("Lat,lon: ");
      Serial.print(gpsLat, 6);
      Serial.print(",");
      Serial.println(gpsLon, 6);

      retVal = bearingToPoint(gpsLon, gpsLat,
                     DESTINATION_LONGITUDE, DESTINATION_LATITUDE);

      lastLongitude = gpsLon;
      lastLatitude = gpsLat;
      updateDistanceLED(gpsLon, gpsLat);
    }
  }
  
  return retVal;
}

/////////////////////////////////////////////////////////////////////////////
void updateDistanceLED(float gpsLon, float gpsLat)
{
  float distance = distanceToPoint(gpsLon, gpsLat,
                     DESTINATION_LONGITUDE, DESTINATION_LATITUDE);
  int red = 0;
  int blue = LED_INTENSITY;

  if(distance > 5000)
  {
    red = 0;
    blue = LED_INTENSITY;
  }
  else if(distance < 2000 && distance >= 500)
  {
    red = LED_INTENSITY * 0.2;
    blue = LED_INTENSITY * 0.8;
  }
  else if(distance < 500 && distance >= 100)
  {
    red = LED_INTENSITY * 0.4;
    blue = LED_INTENSITY * 0.6;
  }
  else if(distance < 100 && distance >= 10)
  {
    red = LED_INTENSITY * 0.8;
    blue = LED_INTENSITY * 0.2;
  }
  else
  {
    red = LED_INTENSITY;
    blue = 0;
  }
  Serial.print("Distance: ");
  Serial.println(distance);
  Serial.print("Red: ");
  Serial.print(red);
  Serial.print(" Blue: ");
  Serial.println(blue);
  uint32_t distanceColour = strip.Color(red, 0, blue);
  strip.setPixelColor(LED_DISTANCE, distanceColour);
  strip.show();  
}

/////////////////////////////////////////////////////////////////////////////
void showCompassLED(float boxBearing, float bearingToDestination)
{
  static long lastCompassUpdate = millis(); // Prevent flickering of compass
  static int compassLED = LED_NORTH;
  static float latchedBearingToDestination = 999.0;
  uint32_t compassColour = strip.Color(0, 0, LED_INTENSITY);
  float displayHeading = boxBearing;

  if(bearingToDestination >= 0.0 && bearingToDestination <= 360.0)
  {
    latchedBearingToDestination = bearingToDestination;
  }    

  if(latchedBearingToDestination >= 0.0 && latchedBearingToDestination <= 360.0)
  {
    displayHeading = latchedBearingToDestination - boxBearing;
    if(displayHeading < 0.0)
    {
      displayHeading = 360.0 + displayHeading;
    }

    compassColour = strip.Color(0, LED_INTENSITY, 0);
  }
  
  if ((millis() - lastCompassUpdate) >= COMPASS_UPDATE_MILLISECONDS)
  {
    if (displayHeading >= (337.5) || displayHeading < 22.5)
    {
      compassLED = LED_NORTH;
    }
    else if (displayHeading >= (22.5) && displayHeading < (67.5))
    {
      compassLED = LED_NORTH_EAST;
    }
    else if (displayHeading >= (67.5) && displayHeading < (112.5))
    {
      compassLED = LED_EAST;
    }
    else if (displayHeading >= (112.5) && displayHeading < (157.5))
    {
      compassLED = LED_SOUTH_EAST;
    }
    else if (displayHeading >= (157.5) && displayHeading < (202.5))
    {
      compassLED = LED_SOUTH;
    }
    else if (displayHeading >= (202.5) && displayHeading < (247.5))
    {
      compassLED = LED_SOUTH_WEST;
    }
    else if (displayHeading >= (247.5) && displayHeading < (292.5))
    {
      compassLED = LED_WEST;
    }
    else
    {
      compassLED = LED_NORTH_WEST;
    }

    switchOffAllCompassLEDs();

    strip.setPixelColor(compassLED, compassColour);
    strip.show();

    lastCompassUpdate = millis();
  }
}

/////////////////////////////////////////////////////////////////////////////
void switchOffAllCompassLEDs()
{
  for (int i = MIN_COMPASS_LED; i <= MAX_COMPASS_LED; i ++)
  {
    strip.setPixelColor(i, 0, 0, 0);
  }
  strip.show();
}

/////////////////////////////////////////////////////////////////////////////
float bearingToPoint(float startLon, float startLat, float endLon, float endLat)
{
  float retVal = 0.0;

  float lon1 = radians(startLon);
  float lat1 = radians(startLat);
  float lon2 = radians(endLon);
  float lat2 = radians(endLat);

  float y = sin(lon2 - lon1) * cos(lat2);
  float x = (cos(lat1) * sin(lat2)) - (sin(lat1) * cos(lat2) * cos(lon2 - lon1));

  retVal = degrees(atan2(y, x));

  if(retVal < 0.0)
  {
    retVal = retVal + 360;
  }

  return retVal;
}

/////////////////////////////////////////////////////////////////////////////
float distanceToPoint(float startLon, float startLat, float endLon, float endLat)
{
  float lon1 = radians(startLon);
  float lat1 = radians(startLat);
  float lon2 = radians(endLon);
  float lat2 = radians(endLat);

  float deltaLat = lat2 - lat1;
  float deltaLon = lon2 - lon1;
  
  float a = (sin(deltaLat/2) * sin(deltaLat/2)) +
          (cos(lat1) * cos(lat2) * sin(deltaLon/2) * sin(deltaLon/2));
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return EARTH_RADIUS_METRES * c;
}

  

