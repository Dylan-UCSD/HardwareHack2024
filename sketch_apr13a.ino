
#ifdef ARDUINO_ARCH_SAMD
#include <WiFi101.h>
#elif defined ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#elif defined ARDUINO_ARCH_ESP32
#include <WiFi.h>
#else
#error Wrong platform
#endif 
#include <math.h>
#include <WifiLocation.h>

class Latlon{
  public:
    double lat;
    double lon;
    Latlon(double lat,double lon): lat(lat), lon(lon) { };
  
};


const char* googleApiKey = "google key";

WifiLocation location(googleApiKey);
Latlon home = Latlon(0.0,0.0);
Latlon user = Latlon(0.0,0.0);
const Latlon seventh = Latlon(32.888132,-117.242271);
const Latlon LA = Latlon(34.0549,118.2426);
bool NESW[4] = {false, false, false, false};

void updateDirections(double h_lat,double h_lon,double u_lat,double u_lon){
  // change in latitude is home lat - user lat
  double change_lat = h_lat - u_lat ;
  // change in longitude is home lon - user lon
  double change_lon = h_lon - u_lon ;
  // convert lat & lon to km
  double NS_km = change_lat * 111;
  double EW_km = change_lon * 111 * cos(change_lat* M_PI/180);
  Serial.println("NS_km: "+ String(NS_km,7));
  Serial.println("EW_km: "+ String(EW_km,7));
  // reset NESW
  for(bool direction : NESW) { direction = false; }
  // update NESW with pedantic boolean logic
  if(NS_km == 0){
    NESW[0] = false; 
    NESW[2] = false;
  }
  else if(NS_km > 0){
    NESW[0] = true;
    NESW[2] = false;
  }
  else if(NS_km < 0){
    NESW[0] = false;
    NESW[2] = true;
  }
  if(EW_km == 0){
    NESW[1] = false;
    NESW[3] = false;
  }
  else if(EW_km > 0){
    NESW[1] = true;
    NESW[3] = false;
  }
  else if(EW_km < 0){
    NESW[1] = false;
    NESW[3] = true;
  }
  

}

bool ping(){
  if(WiFi.status() != WL_CONNECTED){
    WiFi.begin("UCSD-GUEST");
  }
  while (WiFi.status() != WL_CONNECTED) {
        Serial.print("Attempting to connect to UCSD GUEST ");
        // wait 5 seconds for connection:
        Serial.print("Status = ");
        Serial.println(WiFi.status());
        delay(500);
    }
    location_t loc = location.getGeoFromWiFi();
    Serial.println("Location request data");
    Serial.println(location.getSurroundingWiFiJson());
    Serial.println("Latitude: " + String(loc.lat, 7));
    Serial.println("Longitude: " + String(loc.lon, 7));
    Serial.println("Accuracy: " + String(loc.accuracy));
    user = Latlon(loc.lat, loc.lon);
    updateDirections(home.lat, home.lon, user.lat, user.lon);
    home = seventh;
    Serial.println("NESW: " + String(NESW[0]) + String(NESW[1]) + String(NESW[2]) + String(NESW[3]));
    return true;
}



void setup() {
      Serial.begin(115200);
      Serial.print("attempting to connect to wifi");
    // Connect to WPA/WPA2 network
#ifdef ARDUINO_ARCH_ESP32
    WiFi.mode(WIFI_MODE_STA);
#endif
#ifdef ARDUINO_ARCH_ESP8266
    WiFi.mode(WIFI_STA);
#endif
    WiFi.begin("UCSD-GUEST");

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print("Attempting to connect to UCSD GUEST");
        // wait 5 seconds for connection:
        Serial.print("Status = ");
        Serial.println(WiFi.status());
        delay(500);
    }
    location_t loc = location.getGeoFromWiFi();
    Serial.println("Location request data");
    Serial.println(location.getSurroundingWiFiJson());
    Serial.println("Latitude: " + String(loc.lat, 7));
    Serial.println("Longitude: " + String(loc.lon, 7));
    Serial.println("Accuracy: " + String(loc.accuracy));
    user = Latlon(loc.lat, loc.lon);
    updateDirections(home.lat, home.lon, user.lat, user.lon);
    home = seventh;
    Serial.println("NESW: " + String(NESW[0]) + String(NESW[1]) + String(NESW[2]) + String(NESW[3]));
}

void loop() {
  delay(30000);
  ping();

}
