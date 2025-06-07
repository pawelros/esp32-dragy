#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

// GPS Module pins
#define GPS_RX 16
#define GPS_TX 17
#define GPS_POWER 18  // Add power control pin if available

// UBX message definitions
#define UBX_HEADER_1 0xB5
#define UBX_HEADER_2 0x62
#define UBX_NAV_VELNED_CLASS 0x01
#define UBX_NAV_VELNED_ID 0x12
#define UBX_NAV_VELNED_LENGTH 36
#define UBX_NAV_POSLLH_CLASS 0x01
#define UBX_NAV_POSLLH_ID 0x02
#define UBX_NAV_POSLLH_LENGTH 28
#define UBX_CFG_MSG_CLASS 0x06
#define UBX_CFG_MSG_ID 0x01
#define UBX_CFG_RATE_CLASS 0x06
#define UBX_CFG_RATE_ID 0x08
#define UBX_CFG_NAV5_CLASS 0x06
#define UBX_CFG_NAV5_ID 0x24

// UBX message structure for NAV-VELNED
struct UBX_NAV_VELNED {
  uint32_t iTOW;          // GPS time of week of the navigation epoch
  int32_t velN;           // North velocity component
  int32_t velE;           // East velocity component
  int32_t velD;           // Down velocity component
  uint32_t speed;         // Speed (3-D)
  uint32_t gSpeed;        // Ground speed (2-D)
  int32_t heading;        // Heading of motion 2-D
  uint32_t sAcc;          // Speed accuracy estimate
  uint32_t cAcc;          // Course/Heading accuracy estimate
} __attribute__((packed));

// UBX message structure for NAV-POSLLH
struct UBX_NAV_POSLLH {
  uint32_t iTOW;          // GPS time of week of the navigation epoch
  int32_t lon;            // Longitude (deg * 1e-7)
  int32_t lat;            // Latitude (deg * 1e-7)
  int32_t height;         // Height above ellipsoid (mm)
  int32_t hMSL;           // Height above mean sea level (mm)
  uint32_t hAcc;          // Horizontal accuracy estimate (mm)
  uint32_t vAcc;          // Vertical accuracy estimate (mm)
} __attribute__((packed));

// WiFi credentials
const char* ap_ssid = "ESP32Dragy";
const char* ap_password = "dragy1234";
IPAddress staticIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// GPS and Web Server objects
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
WebServer server(80);

// GPS status tracking
struct GPSStatus {
  bool isConnected = false;
  bool hasFix = false;
  unsigned long lastUpdate = 0;
  int satellites = 0;
  float hdop = 0;
  float latitude = 0;
  float longitude = 0;
  float altitude = 0;
  float speed = 0;
  float speedAccuracy = 0;
  bool dateValid = false;
  bool timeValid = false;
  String date = "";
  String time = "";
} gpsStatus;

// Speed intervals structure
struct SpeedInterval {
  float startSpeed;
  float endSpeed;
  float time;
  float speedReached;
  bool completed;
};

// Drag race metrics
struct DragMetrics {
  float maxSpeed = 0;
  float time0to100 = 0;
  float time100to200 = 0;
  float quarterMileTime = 0;
  float quarterMileSpeed = 0;
  float eighthMileTime = 0;
  float eighthMileSpeed = 0;
  bool isRecording = false;
  unsigned long startTime = 0;
  float startSpeed = 0;
  float distance = 0;
  float lastSpeed = 0;
  unsigned long lastSpeedUpdate = 0;
  float acceleration = 0;
  float roadSlope = 0;
  float maxAcceleration = 0;
  float startAltitude = 0;
  float currentAltitude = 0;
  float altitudeGain = 0;
  
  // Speed intervals array (0-10, 10-20, ..., 190-200)
  SpeedInterval speedIntervals[20] = {
    {0, 10, 0, 0, false}, {10, 20, 0, 0, false}, {20, 30, 0, 0, false},
    {30, 40, 0, 0, false}, {40, 50, 0, 0, false}, {50, 60, 0, 0, false},
    {60, 70, 0, 0, false}, {70, 80, 0, 0, false}, {80, 90, 0, 0, false},
    {90, 100, 0, 0, false}, {100, 110, 0, 0, false}, {110, 120, 0, 0, false},
    {120, 130, 0, 0, false}, {130, 140, 0, 0, false}, {140, 150, 0, 0, false},
    {150, 160, 0, 0, false}, {160, 170, 0, 0, false}, {170, 180, 0, 0, false},
    {180, 190, 0, 0, false}, {190, 200, 0, 0, false}
  };
} metrics;

// Store last 5 runs
struct RunHistory {
  DragMetrics runs[5];
  int currentIndex = 0;
  int totalRuns = 0;
} history;

bool isStopped = false;

// UBX message handling
bool parseUBX() {
  static uint8_t ubxBuffer[64]; // Increased buffer size for both message types
  static uint8_t ubxIndex = 0;
  
  while (GPSSerial.available()) {
    uint8_t c = GPSSerial.read();
    
    // Look for UBX header
    if (ubxIndex == 0 && c != UBX_HEADER_1) continue;
    if (ubxIndex == 1 && c != UBX_HEADER_2) {
      ubxIndex = 0;
      continue;
    }
    
    // Store byte in buffer
    ubxBuffer[ubxIndex++] = c;
    
    // Check if we have a complete message
    if (ubxIndex >= 8) {
      uint8_t msgClass = ubxBuffer[2];
      uint8_t msgId = ubxBuffer[3];
      uint16_t msgLength = ubxBuffer[4] | (ubxBuffer[5] << 8);
      
      // Check message type
      if (msgClass == UBX_NAV_VELNED_CLASS && msgId == UBX_NAV_VELNED_ID) {
        // Wait for complete message
        if (ubxIndex >= msgLength + 8) {
          // Verify checksum
          uint8_t ck_a = 0, ck_b = 0;
          for (int i = 2; i < msgLength + 6; i++) {
            ck_a += ubxBuffer[i];
            ck_b += ck_a;
          }
          
          if (ck_a == ubxBuffer[msgLength + 6] && ck_b == ubxBuffer[msgLength + 7]) {
            // Process NAV-VELNED message
            UBX_NAV_VELNED* velned = (UBX_NAV_VELNED*)&ubxBuffer[6];
            
            // Update speed (convert from mm/s to km/h)
            gpsStatus.speed = (float)velned->gSpeed * 0.0036; // mm/s to km/h
            gpsStatus.speedAccuracy = (float)velned->sAcc * 0.0036; // mm/s to km/h
            
            // Update GPS status
            gpsStatus.isConnected = true;
            gpsStatus.lastUpdate = millis();
            gpsStatus.hasFix = true;
            
            ubxIndex = 0;
            return true;
          }
        }
      }
      else if (msgClass == UBX_NAV_POSLLH_CLASS && msgId == UBX_NAV_POSLLH_ID) {
        // Wait for complete message
        if (ubxIndex >= msgLength + 8) {
          // Verify checksum
          uint8_t ck_a = 0, ck_b = 0;
          for (int i = 2; i < msgLength + 6; i++) {
            ck_a += ubxBuffer[i];
            ck_b += ck_a;
          }
          
          if (ck_a == ubxBuffer[msgLength + 6] && ck_b == ubxBuffer[msgLength + 7]) {
            // Process NAV-POSLLH message
            UBX_NAV_POSLLH* posllh = (UBX_NAV_POSLLH*)&ubxBuffer[6];
            
            // Update position (convert from deg * 1e-7 to degrees)
            gpsStatus.latitude = (float)posllh->lat * 1e-7;
            gpsStatus.longitude = (float)posllh->lon * 1e-7;
            gpsStatus.altitude = (float)posllh->hMSL / 1000.0; // mm to meters
            
            // Update accuracy
            gpsStatus.hdop = (float)posllh->hAcc / 1000.0; // mm to meters
            
            ubxIndex = 0;
            return true;
          }
        }
      }
      
      // Reset if message is too long
      if (ubxIndex >= sizeof(ubxBuffer)) {
        ubxIndex = 0;
      }
    }
  }
  return false;
}

// Send UBX configuration message
void sendUBX(const uint8_t *msg, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    GPSSerial.write(msg[i]);
  }
  delay(100);
}

// Configure GPS for UBX NAV-VELNED and NAV-POSLLH
void configureGPS() {
  Serial.println("Starting GPS configuration...");
  
  // Initialize GPS with basic settings
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(1000);
  
  // Send factory reset command for NEO-M8N
  Serial.println("Sending factory reset command...");
  const uint8_t factoryReset[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x17, 0x31, 0xBF};
  sendUBX(factoryReset, sizeof(factoryReset));
  delay(2000);
  
  // Power cycle GPS if power pin is available
  #ifdef GPS_POWER
    Serial.println("Power cycling GPS module...");
    pinMode(GPS_POWER, OUTPUT);
    digitalWrite(GPS_POWER, LOW);
    delay(2000);  // Wait 2 seconds
    digitalWrite(GPS_POWER, HIGH);
    delay(2000);  // Wait 2 seconds after power on
  #endif
  
  // Reinitialize serial after power cycle
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(2000);
  
  // Send cold start command
  Serial.println("Sending cold start command...");
  const uint8_t coldStart[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x64};
  sendUBX(coldStart, sizeof(coldStart));
  delay(2000);
  
  // Configure navigation settings for high precision
  Serial.println("Configuring navigation settings...");
  const uint8_t nav5Config[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };
  sendUBX(nav5Config, sizeof(nav5Config));
  delay(1000);
  
  // Configure update rate to 10Hz (100ms) - maximum reliable rate for NEO-M8N
  Serial.println("Configuring update rate...");
  const uint8_t rateConfig[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12
  };
  sendUBX(rateConfig, sizeof(rateConfig));
  delay(1000);
  
  // Configure dynamic model to "Automotive" for better speed accuracy
  Serial.println("Configuring dynamic model...");
  const uint8_t dynModelConfig[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };
  sendUBX(dynModelConfig, sizeof(dynModelConfig));
  delay(1000);
  
  // Enable only NAV-VELNED and NAV-POSLLH messages at maximum rate
  Serial.println("Configuring message rates...");
  const uint8_t msgConfig[] = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x47,  // NAV-VELNED
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47   // NAV-POSLLH
  };
  sendUBX(msgConfig, sizeof(msgConfig));
  delay(1000);
  
  // Disable NMEA messages to reduce noise
  Serial.println("Disabling NMEA messages...");
  const uint8_t disableNMEA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFB, 0x13,  // GGA
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFC, 0x13,  // GLL
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFD, 0x13,  // GSA
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFE, 0x13,  // GSV
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFF, 0x13,  // RMC
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x14   // VTG
  };
  sendUBX(disableNMEA, sizeof(disableNMEA));
  delay(1000);
  
  Serial.println("GPS configuration complete");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");
  
  // Initialize GPS with basic settings
  Serial.println("Initializing GPS...");
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(1000);
  
  configureGPS();
  
  // Initialize SPIFFS
  if(!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  
  // Format SPIFFS if needed
  if(!SPIFFS.exists("/index.html")) {
    Serial.println("Formatting SPIFFS...");
    SPIFFS.format();
    Serial.println("SPIFFS formatted");
  }
  
  // List SPIFFS contents for debugging
  Serial.println("Listing SPIFFS contents:");
  File root = SPIFFS.open("/");
  if(!root) {
    Serial.println("Failed to open root directory");
    return;
  }
  
  File file = root.openNextFile();
  while(file) {
    Serial.print("  ");
    Serial.print(file.name());
    Serial.print("  ");
    Serial.println(file.size());
    file = root.openNextFile();
  }
  
  // Configure and start Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(staticIP, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_password);
  
  Serial.println("");
  Serial.println("Access Point Started");
  Serial.print("SSID: ");
  Serial.println(ap_ssid);
  Serial.print("Password: ");
  Serial.println(ap_password);
  Serial.print("IP Address: ");
  Serial.println(staticIP);
  
  // Setup web server routes
  setupWebServer();
  
  Serial.println("Drag Meter initialized!");
}

void loop() {
  // Read GPS data
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    // Debug output - print all raw data
    Serial.write(c);
    
    if (gps.encode(c)) {
      // Update GPS status
      gpsStatus.isConnected = true;
      gpsStatus.lastUpdate = millis();
      gpsStatus.hasFix = gps.location.isValid();
      gpsStatus.satellites = gps.satellites.value();
      gpsStatus.hdop = gps.hdop.hdop();
      
      // Debug output
      if (gpsStatus.hasFix) {
        Serial.println("\nGPS Fix obtained!");
        Serial.print("Satellites: ");
        Serial.println(gpsStatus.satellites);
        Serial.print("HDOP: ");
        Serial.println(gpsStatus.hdop);
      }
      
      if (gps.location.isValid()) {
        gpsStatus.latitude = gps.location.lat();
        gpsStatus.longitude = gps.location.lng();
        Serial.print("Location: ");
        Serial.print(gpsStatus.latitude, 6);
        Serial.print(", ");
        Serial.println(gpsStatus.longitude, 6);
      }
      
      if (gps.altitude.isValid()) {
        gpsStatus.altitude = gps.altitude.meters();
        Serial.print("Altitude: ");
        Serial.println(gpsStatus.altitude);
      }
      
      if (gps.speed.isValid()) {
        gpsStatus.speed = gps.speed.kmph();
        Serial.print("Speed: ");
        Serial.println(gpsStatus.speed);
      }
      
      if (gps.date.isValid()) {
        gpsStatus.dateValid = true;
        gpsStatus.date = String(gps.date.month()) + "/" + 
                        String(gps.date.day()) + "/" + 
                        String(gps.date.year());
      }
      
      if (gps.time.isValid()) {
        gpsStatus.timeValid = true;
        gpsStatus.time = String(gps.time.hour()) + ":" + 
                        String(gps.time.minute()) + ":" + 
                        String(gps.time.second());
      }
      
      updateMetrics();
    }
  }
  
  // Check if GPS connection is lost
  if (millis() - gpsStatus.lastUpdate > 2000) { // 2 seconds timeout
    gpsStatus.isConnected = false;
    gpsStatus.hasFix = false;
    Serial.println("GPS connection lost");
  }
  
  // Handle web server clients
  server.handleClient();
}

void saveCurrentRun() {
  // Only save runs that reached at least 100 km/h
  if (metrics.maxSpeed >= 100.0) {
    // Save current run to history
    history.runs[history.currentIndex] = metrics;
    history.currentIndex = (history.currentIndex + 1) % 5;
    if (history.totalRuns < 5) history.totalRuns++;
  }
  
  // Reset metrics for next run
  metrics.isRecording = false;
  metrics.maxSpeed = 0;
  metrics.time0to100 = 0;
  metrics.time100to200 = 0;
  metrics.quarterMileTime = 0;
  metrics.quarterMileSpeed = 0;
  metrics.eighthMileTime = 0;
  metrics.eighthMileSpeed = 0;
  metrics.distance = 0;
  metrics.acceleration = 0;
  metrics.maxAcceleration = 0;
  metrics.roadSlope = 0;
  metrics.startAltitude = 0;
  metrics.currentAltitude = 0;
  metrics.altitudeGain = 0;
  
  // Reset speed intervals
  for (int i = 0; i < 20; i++) {
    metrics.speedIntervals[i].time = 0;
    metrics.speedIntervals[i].speedReached = 0;
    metrics.speedIntervals[i].completed = false;
  }
}

void updateMetrics() {
  if (gps.speed.isValid()) {
    float currentSpeed = gps.speed.kmph();
    unsigned long currentTime = millis();
    
    // Update altitude data
    if (gps.altitude.isValid()) {
      metrics.currentAltitude = gps.altitude.meters();
      if (!metrics.isRecording) {
        metrics.startAltitude = metrics.currentAltitude;
      }
      metrics.altitudeGain = metrics.currentAltitude - metrics.startAltitude;
      
      // Calculate road slope based on altitude gain and distance
      if (metrics.distance > 0) {
        // Convert distance to meters and calculate slope
        float distanceMeters = metrics.distance * 1000.0;
        metrics.roadSlope = atan2(metrics.altitudeGain, distanceMeters) * 180.0 / PI;
      }
    }
    
    // Calculate acceleration (km/h/s)
    if (metrics.lastSpeedUpdate > 0) {
      float timeDiff = (currentTime - metrics.lastSpeedUpdate) / 1000.0; // Convert to seconds
      if (timeDiff > 0) {
        metrics.acceleration = (currentSpeed - metrics.lastSpeed) / timeDiff;
        
        // Update max acceleration
        if (metrics.acceleration > metrics.maxAcceleration) {
          metrics.maxAcceleration = metrics.acceleration;
        }
      }
    }
    
    // Detect if car is stopped
    if (currentSpeed < 1.0) {
      isStopped = true;
    }
    
    // Start recording when:
    // 1. Car was stopped (speed < 1 km/h)
    // 2. Significant acceleration is detected
    if (isStopped && !metrics.isRecording && metrics.acceleration > 5.0) { // 5 km/h/s threshold
      metrics.isRecording = true;
      metrics.startTime = currentTime;
      metrics.startSpeed = currentSpeed;
      metrics.distance = 0;
      metrics.startAltitude = metrics.currentAltitude;
      isStopped = false;
      
      // Reset all speed intervals
      for (int i = 0; i < 20; i++) {
        metrics.speedIntervals[i].time = 0;
        metrics.speedIntervals[i].speedReached = 0;
        metrics.speedIntervals[i].completed = false;
      }
    }
    
    if (metrics.isRecording) {
      // Update max speed
      if (currentSpeed > metrics.maxSpeed) {
        metrics.maxSpeed = currentSpeed;
      }
      
      // Update speed intervals
      for (int i = 0; i < 20; i++) {
        if (!metrics.speedIntervals[i].completed && 
            currentSpeed >= metrics.speedIntervals[i].endSpeed) {
          metrics.speedIntervals[i].time = (currentTime - metrics.startTime) / 1000.0;
          metrics.speedIntervals[i].speedReached = currentSpeed;
          metrics.speedIntervals[i].completed = true;
        }
      }
      
      // Calculate 0-100 km/h time
      if (metrics.time0to100 == 0 && currentSpeed >= 100) {
        metrics.time0to100 = (currentTime - metrics.startTime) / 1000.0;
      }
      
      // Calculate 100-200 km/h time
      if (metrics.time100to200 == 0 && currentSpeed >= 200) {
        metrics.time100to200 = (currentTime - metrics.startTime) / 1000.0 - metrics.time0to100;
      }
      
      // Calculate 1/8 mile and 1/4 mile times
      if (gps.location.isValid()) {
        metrics.distance += gps.distanceBetween(
          gps.location.lat(), gps.location.lng(),
          gps.location.lat(), gps.location.lng()
        ) / 1000.0; // Convert to kilometers
        
        // 1/8 mile (0.201 km)
        if (metrics.eighthMileTime == 0 && metrics.distance >= 0.201) {
          metrics.eighthMileTime = (currentTime - metrics.startTime) / 1000.0;
          metrics.eighthMileSpeed = currentSpeed;
        }
        
        // 1/4 mile (0.402 km)
        if (metrics.quarterMileTime == 0 && metrics.distance >= 0.402) {
          metrics.quarterMileTime = (currentTime - metrics.startTime) / 1000.0;
          metrics.quarterMileSpeed = currentSpeed;
        }
      }
      
      // Stop recording if:
      // 1. Speed drops below 1 km/h, or
      // 2. Acceleration is negative for more than 1 second
      static unsigned long decelerationStart = 0;
      if (currentSpeed < 1.0) {
        saveCurrentRun();
      } else if (metrics.acceleration < -1.0) { // Decelerating more than 1 km/h/s
        if (decelerationStart == 0) {
          decelerationStart = currentTime;
        } else if (currentTime - decelerationStart > 1000) { // 1 second of deceleration
          saveCurrentRun();
          decelerationStart = 0;
        }
      } else {
        decelerationStart = 0;
      }
    }
    
    metrics.lastSpeed = currentSpeed;
    metrics.lastSpeedUpdate = currentTime;
  }
}

void setupWebServer() {
  // Handle root path
  server.on("/", HTTP_GET, []() {
    Serial.println("Received request for root path");
    if (!SPIFFS.exists("/index.html")) {
      Serial.println("index.html not found in SPIFFS");
      server.send(404, "text/plain", "index.html not found");
      return;
    }
    
    File file = SPIFFS.open("/index.html", "r");
    if (!file) {
      Serial.println("Failed to open index.html");
      server.send(500, "text/plain", "Failed to open file");
      return;
    }
    
    Serial.println("Serving index.html");
    server.streamFile(file, "text/html");
    file.close();
  });
  
  // Serve JSON data
  server.on("/data", HTTP_GET, []() {
    JsonDocument doc;
    
    // GPS Status
    doc["gpsConnected"] = gpsStatus.isConnected;
    doc["gpsFix"] = gpsStatus.hasFix;
    doc["satellites"] = gpsStatus.satellites;
    doc["hdop"] = gpsStatus.hdop;
    doc["speed"] = gpsStatus.speed;
    
    // Location data
    doc["hasLocation"] = gpsStatus.hasFix;
    doc["latitude"] = gpsStatus.latitude;
    doc["longitude"] = gpsStatus.longitude;
    doc["altitude"] = gpsStatus.altitude;
    
    // Run metrics
    doc["isRecording"] = metrics.isRecording;
    doc["roadSlope"] = metrics.roadSlope;
    doc["altitudeGain"] = metrics.altitudeGain;
    doc["maxSpeed"] = metrics.maxSpeed;
    doc["maxAcceleration"] = metrics.maxAcceleration;
    doc["time0to100"] = metrics.time0to100;
    doc["time100to200"] = metrics.time100to200;
    doc["eighthMileTime"] = metrics.eighthMileTime;
    doc["eighthMileSpeed"] = metrics.eighthMileSpeed;
    doc["quarterMileTime"] = metrics.quarterMileTime;
    doc["quarterMileSpeed"] = metrics.quarterMileSpeed;
    
    // Speed intervals
    JsonArray intervals = doc["speedIntervals"].to<JsonArray>();
    for (int i = 0; i < 20; i++) {
      JsonObject interval = intervals.add<JsonObject>();
      interval["startSpeed"] = metrics.speedIntervals[i].startSpeed;
      interval["endSpeed"] = metrics.speedIntervals[i].endSpeed;
      interval["time"] = metrics.speedIntervals[i].time;
      interval["speedReached"] = metrics.speedIntervals[i].speedReached;
      interval["completed"] = metrics.speedIntervals[i].completed;
    }
    
    // Run history
    JsonArray historyArray = doc["runHistory"].to<JsonArray>();
    for (int i = 0; i < history.totalRuns; i++) {
      int index = (history.currentIndex - 1 - i + 5) % 5; // Get runs in reverse chronological order
      JsonObject run = historyArray.add<JsonObject>();
      run["maxSpeed"] = history.runs[index].maxSpeed;
      run["time0to100"] = history.runs[index].time0to100;
      run["time100to200"] = history.runs[index].time100to200;
      run["eighthMileTime"] = history.runs[index].eighthMileTime;
      run["eighthMileSpeed"] = history.runs[index].eighthMileSpeed;
      run["quarterMileTime"] = history.runs[index].quarterMileTime;
      run["quarterMileSpeed"] = history.runs[index].quarterMileSpeed;
      run["maxAcceleration"] = history.runs[index].maxAcceleration;
      run["roadSlope"] = history.runs[index].roadSlope;
      run["altitudeGain"] = history.runs[index].altitudeGain;
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // Start server
  server.begin();
  Serial.println("Web server started");
}
