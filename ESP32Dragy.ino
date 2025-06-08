#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

// GPS Module pins
#define GPS_RX 16
#define GPS_TX 17

// WiFi credentials
const char* ssid = "ESP32Dragy";
const char* password = "dragy1234";

// GPS Serial object
HardwareSerial GPSSerial(1);

// Web server
WebServer server(80);

// UBX message constants
#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62
#define UBX_NAV_VELNED_CLASS 0x01
#define UBX_NAV_VELNED_ID 0x12
#define UBX_NAV_SAT_CLASS 0x01
#define UBX_NAV_SAT_ID 0x35

// GPS status tracking
struct GPSStatus {
  bool isConnected = false;
  bool hasFix = false;
  unsigned long lastUpdate = 0;
  float latitude = 0;
  float longitude = 0;
  float altitude = 0;
  float velocityNorth = 0;
  float velocityEast = 0;
  float velocityDown = 0;
  uint8_t numSats = 0;
  float speed = 0;  // Calculated from velocity components
} gpsStatus;

// Run metrics
struct RunMetrics {
  bool isRecording = false;
  float roadSlope = 0;
  float altitudeGain = 0;
  float maxSpeed = 0;
  float maxAcceleration = 0;
  float time0to100 = 0;
  float time100to200 = 0;
  float eighthMileTime = 0;
  float eighthMileSpeed = 0;
  float quarterMileTime = 0;
  float quarterMileSpeed = 0;
  unsigned long startTime = 0;
  float startAltitude = 0;
  float lastSpeed = 0;
  unsigned long lastSpeedUpdate = 0;
} runMetrics;

// Speed intervals
struct SpeedInterval {
  float startSpeed;
  float endSpeed;
  float time;
  bool completed;
  float speedReached;
};

SpeedInterval speedIntervals[] = {
  {0, 100, 0, false, 0},
  {100, 200, 0, false, 0},
  {0, 200, 0, false, 0}
};

// Run history
const int MAX_RUNS = 10;
struct RunHistory {
  float time0to100;
  float time100to200;
  float eighthMileTime;
  float eighthMileSpeed;
  float quarterMileTime;
  float quarterMileSpeed;
  float maxSpeed;
  float maxAcceleration;
  float roadSlope;
  float altitudeGain;
};

RunHistory runHistory[MAX_RUNS];
int currentRunIndex = 0;

// Add a global counter for NAV-VELNED messages
unsigned long navVelnedCounter = 0;

// UBX message parsing
bool parseUBX() {
  static uint8_t ubxBuffer[256];
  static uint8_t ubxIndex = 0;
  static bool sync1Found = false;
  
  while (GPSSerial.available()) {
    uint8_t c = GPSSerial.read();
    
    if (!sync1Found) {
      if (c == UBX_SYNC_CHAR1) {
        sync1Found = true;
        ubxIndex = 0;
        ubxBuffer[ubxIndex++] = c;
      }
      continue;
    }
    
    if (ubxIndex == 1) {
      if (c == UBX_SYNC_CHAR2) {
        ubxBuffer[ubxIndex++] = c;
      } else {
        sync1Found = false;
      }
      continue;
    }
    
    if (ubxIndex < sizeof(ubxBuffer)) {
      ubxBuffer[ubxIndex++] = c;
    }
    
    // Check if we have a complete message
    if (ubxIndex >= 6) {
      uint8_t msgClass = ubxBuffer[2];
      uint8_t msgId = ubxBuffer[3];
      uint16_t length = (ubxBuffer[5] << 8) | ubxBuffer[4];
      
      if (ubxIndex >= length + 8) {  // 8 = header(2) + class(1) + id(1) + length(2) + checksum(2)
        // Verify checksum
        uint8_t ck_a = 0, ck_b = 0;
        for (int i = 2; i < ubxIndex - 2; i++) {
          ck_a += ubxBuffer[i];
          ck_b += ck_a;
        }
        
        if (ck_a == ubxBuffer[ubxIndex - 2] && ck_b == ubxBuffer[ubxIndex - 1]) {
          // Process message
          if (msgClass == UBX_NAV_VELNED_CLASS && msgId == UBX_NAV_VELNED_ID) {
            // Increment NAV-VELNED message counter
            navVelnedCounter++;
            // Parse NAV-VELNED message
            // Note: UBX uses little-endian byte order
            // According to UBX protocol, velocity components are in cm/s
            // Payload starts at ubxBuffer[6]
            int payloadOffset = 6;
            int32_t velN = (int32_t)((uint32_t)ubxBuffer[payloadOffset + 4] |
                                     ((uint32_t)ubxBuffer[payloadOffset + 5] << 8) |
                                     ((uint32_t)ubxBuffer[payloadOffset + 6] << 16) |
                                     ((uint32_t)ubxBuffer[payloadOffset + 7] << 24));
            int32_t velE = (int32_t)((uint32_t)ubxBuffer[payloadOffset + 8] |
                                     ((uint32_t)ubxBuffer[payloadOffset + 9] << 8) |
                                     ((uint32_t)ubxBuffer[payloadOffset + 10] << 16) |
                                     ((uint32_t)ubxBuffer[payloadOffset + 11] << 24));
            int32_t velD = (int32_t)((uint32_t)ubxBuffer[payloadOffset + 12] |
                                     ((uint32_t)ubxBuffer[payloadOffset + 13] << 8) |
                                     ((uint32_t)ubxBuffer[payloadOffset + 14] << 16) |
                                     ((uint32_t)ubxBuffer[payloadOffset + 15] << 24));
            uint32_t gSpeed = (uint32_t)ubxBuffer[payloadOffset + 20] |
                               ((uint32_t)ubxBuffer[payloadOffset + 21] << 8) |
                               ((uint32_t)ubxBuffer[payloadOffset + 22] << 16) |
                               ((uint32_t)ubxBuffer[payloadOffset + 23] << 24);
            // Convert from cm/s to m/s
            gpsStatus.velocityNorth = velN / 100.0;
            gpsStatus.velocityEast = velE / 100.0;
            gpsStatus.velocityDown = velD / 100.0;
            // Convert ground speed from cm/s to km/h
            gpsStatus.speed = (gSpeed / 100.0) * 3.6;
            
            gpsStatus.isConnected = true;
            gpsStatus.lastUpdate = millis();
            
            // Update run metrics
            updateRunMetrics();
          }
          else if (msgClass == UBX_NAV_SAT_CLASS && msgId == UBX_NAV_SAT_ID) {
            // Parse NAV-SAT message
            uint8_t numSats = ubxBuffer[6];
            gpsStatus.numSats = numSats;
          }
        }
        
        // Reset for next message
        sync1Found = false;
        ubxIndex = 0;
        return true;
      }
    }
  }
  return false;
}

void updateRunMetrics() {
  if (!runMetrics.isRecording) {
    if (gpsStatus.speed > 5.0) {  // Start recording when speed exceeds 5 km/h
      runMetrics.isRecording = true;
      runMetrics.startTime = millis();
      runMetrics.startAltitude = gpsStatus.altitude;
      runMetrics.lastSpeed = gpsStatus.speed;
      runMetrics.lastSpeedUpdate = millis();
    }
    return;
  }
  
  // Update max speed
  if (gpsStatus.speed > runMetrics.maxSpeed) {
    runMetrics.maxSpeed = gpsStatus.speed;
  }
  
  // Update acceleration
  unsigned long currentTime = millis();
  float timeDiff = (currentTime - runMetrics.lastSpeedUpdate) / 1000.0;  // Convert to seconds
  if (timeDiff > 0) {
    float acceleration = (gpsStatus.speed - runMetrics.lastSpeed) / timeDiff;
    if (acceleration > runMetrics.maxAcceleration) {
      runMetrics.maxAcceleration = acceleration;
    }
  }
  runMetrics.lastSpeed = gpsStatus.speed;
  runMetrics.lastSpeedUpdate = currentTime;
  
  // Update altitude gain
  float altitudeDiff = gpsStatus.altitude - runMetrics.startAltitude;
  if (altitudeDiff > runMetrics.altitudeGain) {
    runMetrics.altitudeGain = altitudeDiff;
  }
  
  // Update speed intervals
  for (int i = 0; i < sizeof(speedIntervals)/sizeof(speedIntervals[0]); i++) {
    if (!speedIntervals[i].completed) {
      if (gpsStatus.speed >= speedIntervals[i].startSpeed) {
        if (speedIntervals[i].time == 0) {
          speedIntervals[i].time = (currentTime - runMetrics.startTime) / 1000.0;
        }
        if (gpsStatus.speed >= speedIntervals[i].endSpeed) {
          speedIntervals[i].completed = true;
          speedIntervals[i].speedReached = gpsStatus.speed;
        }
      }
    }
  }
  
  // Check if run should end
  if (gpsStatus.speed < 5.0 && runMetrics.maxSpeed > 50.0) {  // End if speed drops below 5 km/h and max speed was over 50 km/h
    endRun();
  }
}

void endRun() {
  if (!runMetrics.isRecording) return;
  
  // Save run to history
  runHistory[currentRunIndex].time0to100 = speedIntervals[0].time;
  runHistory[currentRunIndex].time100to200 = speedIntervals[1].time;
  runHistory[currentRunIndex].eighthMileTime = speedIntervals[2].time;
  runHistory[currentRunIndex].eighthMileSpeed = speedIntervals[2].speedReached;
  runHistory[currentRunIndex].quarterMileTime = speedIntervals[3].time;
  runHistory[currentRunIndex].quarterMileSpeed = speedIntervals[3].speedReached;
  runHistory[currentRunIndex].maxSpeed = runMetrics.maxSpeed;
  runHistory[currentRunIndex].maxAcceleration = runMetrics.maxAcceleration;
  runHistory[currentRunIndex].roadSlope = runMetrics.roadSlope;
  runHistory[currentRunIndex].altitudeGain = runMetrics.altitudeGain;
  
  currentRunIndex = (currentRunIndex + 1) % MAX_RUNS;
  
  // Reset run metrics
  runMetrics = RunMetrics();
  for (int i = 0; i < sizeof(speedIntervals)/sizeof(speedIntervals[0]); i++) {
    speedIntervals[i] = SpeedInterval{0, 0, 0, false, 0};
  }
}

void setupWebServer() {
  // Root endpoint - serve index.html
  server.on("/", []() {
    if(SPIFFS.exists("/index.html")) {
      File file = SPIFFS.open("/index.html", "r");
      if(!file) {
        server.send(500, "text/plain", "Failed to open index.html");
        return;
      }
      server.streamFile(file, "text/html");
      file.close();
    } else {
      server.send(404, "text/plain", "index.html not found in SPIFFS");
    }
  });

  // API endpoint for GPS data
  server.on("/data", []() {
    JsonDocument doc;
    
    doc["speed"] = gpsStatus.speed;
    doc["gpsConnected"] = gpsStatus.isConnected;
    doc["gpsFix"] = gpsStatus.hasFix;
    doc["satellites"] = gpsStatus.numSats;
    doc["hasLocation"] = gpsStatus.hasFix;
    doc["latitude"] = gpsStatus.latitude;
    doc["longitude"] = gpsStatus.longitude;
    doc["altitude"] = gpsStatus.altitude;
    doc["isRecording"] = runMetrics.isRecording;
    doc["roadSlope"] = runMetrics.roadSlope;
    doc["altitudeGain"] = runMetrics.altitudeGain;
    doc["maxSpeed"] = runMetrics.maxSpeed;
    doc["maxAcceleration"] = runMetrics.maxAcceleration;
    doc["time0to100"] = speedIntervals[0].time;
    doc["time100to200"] = speedIntervals[1].time;
    doc["eighthMileTime"] = speedIntervals[2].time;
    doc["eighthMileSpeed"] = speedIntervals[2].speedReached;
    doc["quarterMileTime"] = speedIntervals[3].time;
    doc["quarterMileSpeed"] = speedIntervals[3].speedReached;
    doc["navVelnedCount"] = navVelnedCounter;
    
    JsonArray intervals = doc["speedIntervals"].to<JsonArray>();
    for (int i = 0; i < sizeof(speedIntervals)/sizeof(speedIntervals[0]); i++) {
      JsonObject interval = intervals.add<JsonObject>();
      interval["startSpeed"] = speedIntervals[i].startSpeed;
      interval["endSpeed"] = speedIntervals[i].endSpeed;
      interval["time"] = speedIntervals[i].time;
      interval["completed"] = speedIntervals[i].completed;
      interval["speedReached"] = speedIntervals[i].speedReached;
    }
    
    JsonArray history = doc["runHistory"].to<JsonArray>();
    for (int i = 0; i < MAX_RUNS; i++) {
      JsonObject run = history.add<JsonObject>();
      run["time0to100"] = runHistory[i].time0to100;
      run["time100to200"] = runHistory[i].time100to200;
      run["eighthMileTime"] = runHistory[i].eighthMileTime;
      run["eighthMileSpeed"] = runHistory[i].eighthMileSpeed;
      run["quarterMileTime"] = runHistory[i].quarterMileTime;
      run["quarterMileSpeed"] = runHistory[i].quarterMileSpeed;
      run["maxSpeed"] = runHistory[i].maxSpeed;
      run["maxAcceleration"] = runHistory[i].maxAcceleration;
      run["roadSlope"] = runHistory[i].roadSlope;
      run["altitudeGain"] = runHistory[i].altitudeGain;
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  // Debug endpoint
  server.on("/debug", []() {
    String response = "Files in SPIFFS:\n";
    File root = SPIFFS.open("/");
    if(!root){
      server.send(500, "text/plain", "Failed to open SPIFFS root directory");
      return;
    }
    
    File file = root.openNextFile();
    while(file){
      response += file.name();
      response += " - ";
      response += file.size();
      response += " bytes\n";
      file = root.openNextFile();
    }
    server.send(200, "text/plain", response);
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
  
  server.begin();
}

void setup() {
  Serial.begin(115200);
  
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    return;
  }
  
  // List SPIFFS contents
  File root = SPIFFS.open("/");
  if(!root){
    return;
  }
  
  File file = root.openNextFile();
  while(file){
    file = root.openNextFile();
  }
  
  // Initialize WiFi in AP mode
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  
  // Initialize GPS
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(2000);
  
  // Setup web server
  setupWebServer();
}

void loop() {
  // Read GPS data
  while (GPSSerial.available() > 0) {
    parseUBX();
  }
  
  // Check if GPS connection is lost
  if (millis() - gpsStatus.lastUpdate > 2000) { // 2 seconds timeout
    gpsStatus.isConnected = false;
    gpsStatus.hasFix = false;
  }
  
  // Handle web server
  server.handleClient();
}
