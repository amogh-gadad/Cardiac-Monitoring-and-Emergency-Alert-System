#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

// --------------------------------------------------------
// !!! AD8232 ECG SENSOR RESTORED !!!
// Pinout is functional: MCP3008 (GPIO 2-5), OLED (GPIO 8-9).
// Emergency sequence is AUTOMATIC (no cancellation).
// ALERT IS SENT FOR BPM < 60 OR BPM > 120, PERSISTING FOR 5 SECONDS.
// --------------------------------------------------------

// ---------- Pins (Final Functional Assignment) ----------

// ADC (Software SPI) - RETAINED ORIGINAL PINS
#define MCP_CLK_PIN     2    // GPIO 2 (Pin 4)
#define MCP_DIN_PIN     3    // GPIO 3 (Pin 5)
#define MCP_DOUT_PIN    4    // GPIO 4 (Pin 6)
#define MCP_CS_PIN      5    // GPIO 5 (Pin 7)
#define ECG_CHANNEL     0    // AD8232 ECG Sensor connects to MCP3008 CH0

// OLED Display (I2C0) - SHIFTED PINS
#define OLED_SDA_PIN    8    // GPIO 8 (Pin 11) -> I2C0 SDA
#define OLED_SCL_PIN    9    // GPIO 9 (Pin 12) -> I2C0 SCL

// GSM / GPS Modem (UART 0 / SoftwareSerial)
#define GSM_TX_PIN      0    // GPIO 0 (Pico TX to Modem RX)
#define GSM_RX_PIN      1    // GPIO 1 (Pico RX from Modem TX)
#define GSM_BAUD        115200

// GPS (SoftwareSerial RX-only)
#define GPS_RX_PIN      6    // GPS TX -> Pico GP6 
#define GPS_SOFT_TX_PIN 7    // dummy TX pin
#define GPS_BAUD        9600
SoftwareSerial gpsSS(GPS_RX_PIN, GPS_SOFT_TX_PIN);
TinyGPSPlus gps;

// Local Alert
#define BUZZER_PIN      27   // GPIO 27 (Pin 32)

// Cache last known good fix
bool    haveLastFix = false;
double lastLat = 0.0, lastLon = 0.0;

// ---------- OLED Display ----------
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
// Use Wire (I2C0) for OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- BPM/ECG Tuning ----------
#define BPM_ANOMALY_LOW   60
#define BPM_ANOMALY_HIGH  120
#define ANOMALY_WINDOW_MS 5000 // CHANGED TO 5000 MS (5 seconds)

// State tracking for anomaly
unsigned long anomalyStartTime = 0;
bool isAnomaly = false;
uint16_t currentBPM = 0;

// ---------- ECG Processing State (Restored R-Peak Detection) ----------
#define PEAK_THRESHOLD_V 0.10 // 10% threshold change for R-peak detection
#define MIN_INTERVAL_MS  200  
#define MAX_INTERVAL_MS  1000 

unsigned long lastRTime = 0; // Time of the last R-peak
int lastVoltage = 0; // Last ADC value
int currentVoltage = 0; // Current ADC value
bool isPeak = false; // Flag to indicate if we are currently on a peak

// ---------- Buzzer / GSM Tuning (Constants) ----------
#define BEEP_WINDOW_MS        30000 
#define BEEP_ON_MS            200
#define BEEP_OFF_MS           200
#define EMERGENCY1  "+918951295339"    
#define EMERGENCY2  "+917022880385"    
#define CALL_AFTER_SMS_DELAY      5000    
#define CALL_CONFIRM_SCREEN_MS    2000    

// ------------------------------------
// ---------- MCP3008 ADC Software SPI Read (Unchanged) ----------
// ------------------------------------

static int readMCP3008(uint8_t channel) {
  uint8_t command = 0b00000000;
  command |= 0b00011000; 
  command |= (channel << 2);

  digitalWrite(MCP_CS_PIN, LOW); 

  for (int i = 7; i >= 3; i--) {
    digitalWrite(MCP_CLK_PIN, LOW);
    digitalWrite(MCP_DIN_PIN, (command & (1 << i)));
    digitalWrite(MCP_CLK_PIN, HIGH);
  }

  int result = 0;
  for (int i = 9; i >= 0; i--) {
    digitalWrite(MCP_CLK_PIN, LOW);
    result <<= 1;
    digitalWrite(MCP_CLK_PIN, HIGH);
    if (digitalRead(MCP_DOUT_PIN)) {
      result |= 0x1;
    }
  }
  
  digitalWrite(MCP_CS_PIN, HIGH); 
  return result & 0x3FF; 
}

// ------------------------------------
// ---------- BPM Calculation (RESTORED for AD8232 ECG) ----------
// ------------------------------------

static uint16_t readBPM() {
  currentVoltage = readMCP3008(ECG_CHANNEL);
  
  const int peakDelta = (int)(PEAK_THRESHOLD_V * (1023.0 / 3.3));
  unsigned long now = millis();

  // 1. Peak Detection (simplified R-peak detection logic for ECG)
  if (currentVoltage > lastVoltage + peakDelta && !isPeak) {
    // Potential R-peak found
    isPeak = true;
    
    // 2. R-R Interval Calculation
    if (lastRTime > 0) {
      unsigned long interval = now - lastRTime;
      
      // Filter out noise/bad peaks
      if (interval >= MIN_INTERVAL_MS && interval <= MAX_INTERVAL_MS) {
        currentBPM = 60000 / interval; // BPM = 60s/interval (ms)
      }
    }
    lastRTime = now;
    
  } else if (currentVoltage < lastVoltage - peakDelta) {
    // The wave has dropped significantly, reset peak flag to look for next peak
    isPeak = false;
  }
  
  // Update last voltage for next comparison
  lastVoltage = currentVoltage;
  
  // Return the last valid calculated BPM
  return currentBPM;
}

// ------------------------------------
// ---------- HELPER FUNCTIONS (Unchanged Logic) ----------
// ------------------------------------

static void beepOn()  { digitalWrite(BUZZER_PIN, HIGH); }
static void beepOff() { digitalWrite(BUZZER_PIN, LOW);  }

static void oledPrintLine(uint8_t row, const char* s) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, row * 10);
  display.print("                ");
  display.setCursor(0, row * 10);
  display.print(s);
}

static void oledPrintBPM(uint16_t bpm) {
  display.clearDisplay();
  
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("BPM:");

  display.setTextSize(4);
  display.setCursor(0, 20);
  display.printf("%3d", bpm);

  display.setTextSize(2);
  display.setCursor(95, 45);
  display.print("bpm");

  display.display();
}

static void serviceGPS(unsigned long ms) {
  for (unsigned long t = 0; t < ms; t += 5) {
    while (gpsSS.available()) {
      char c = gpsSS.read();
      gps.encode(c);
    }
    delay(5);
  }
  if (gps.location.isValid()) {
    haveLastFix = true;
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
  }
}

static String googleMapsLink(double lat, double lon) {
  String s = "https://maps.google.com/?q=";
  s += String(lat, 6); s += ","; s += String(lon, 6);
  return s;
}

static void gsmQuickCmd(const char* cmd, uint16_t pause_ms) {
  Serial1.print(cmd);
  Serial1.print("\r");
  delay(pause_ms);
}

static void modemPrepareSMS() {
  gsmQuickCmd("AT", 100);
  gsmQuickCmd("AT+CMGF=1", 100);
  gsmQuickCmd("AT+CSCS=\"GSM\"", 100);
}

static void modemSendSMSTo(const char* number, const String& msg) {
  if (!number || number[0] == '\0') return;
  Serial1.print("AT+CMGS=\"");
  Serial1.print(number);
  Serial1.print("\"\r");
  delay(350);
  Serial1.print(msg);
  Serial1.write(26);
  delay(1500);
}

static void sendAnomalySMS_Blocking(uint16_t bpm) {
  serviceGPS(800);

  String msg;
  msg.reserve(200);
  msg += "BPM Anomaly Detected! BPM="; msg += String(bpm); msg += "\n";

  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    msg += "Lat: "; msg += String(lat, 6);
    msg += "  Lon: "; msg += String(lon, 6); msg += "\n";
    msg += googleMapsLink(lat, lon);
  } else if (haveLastFix) {
    msg += "Last fix\n";
    msg += "Lat: "; msg += String(lastLat, 6);
    msg += "  Lon: "; msg += String(lastLon, 6); msg += "\n";
    msg += googleMapsLink(lastLat, lastLon);
  } else {
    msg += "GPS: No Fix";
  }

  modemPrepareSMS();
  modemSendSMSTo(EMERGENCY1, msg);
  modemSendSMSTo(EMERGENCY2, msg);
}

static void showResetAndHalt() {
  beepOff();
  display.clearDisplay();
  oledPrintLine(2, "RESET REQUIRED");
  oledPrintLine(3, "Power off/on");
  display.display();
  while (true) {
    while (gpsSS.available()) gps.encode(gpsSS.read());
    delay(200);
  }
}

static void dialAfterShowingResetAndHalt() {
  // 1) Pre-display: Called emergency
  display.clearDisplay();
  oledPrintLine(0, "Calling EMER:");
  oledPrintLine(1, EMERGENCY1);
  display.display();
  serviceGPS(CALL_CONFIRM_SCREEN_MS);

  // 2) Show RESET before sending ATD
  display.clearDisplay();
  oledPrintLine(0, "RESET REQUIRED");
  oledPrintLine(1, "Power off/on");
  display.display();
  serviceGPS(150);

  // 3) Send dial command
  Serial1.print("ATD");
  Serial1.print(EMERGENCY1);
  Serial1.print(";\r");
  delay(150);

  // 4) Halt so RESET screen stays until power-cycled
  showResetAndHalt();
}

static void handleAnomalyBlocking(uint16_t anomaly_bpm) {
  // Accident banner
  display.clearDisplay();
  oledPrintLine(0, "!!! AUTO ALERT !!!");
  char l1[17]; snprintf(l1, sizeof(l1), "BPM: %3d", anomaly_bpm);
  oledPrintLine(1, l1);
  display.display();

  // 1. Initial 3s beep (no cancellation window)
  const unsigned long startBeep = millis();
  while (millis() - startBeep < 3000) {
      beepOn();
      delay(200);
      beepOff();
      delay(200);
      serviceGPS(50); // Keep GPS running during beep
  }
  beepOff();

  // 2. Send SMS
  display.clearDisplay();
  oledPrintLine(0, "Sending SMS...");
  oledPrintLine(1, l1);
  display.display();
  sendAnomalySMS_Blocking(anomaly_bpm);

  // 3. Wait 5 s before dialing
  display.clearDisplay();
  oledPrintLine(0, "SMS sent");
  oledPrintLine(1, "Dial in 5s...");
  display.display();
  serviceGPS(CALL_AFTER_SMS_DELAY);

  // 4. Dial -> Halt
  dialAfterShowingResetAndHalt();
}
// ------------------------------------


// ------------------------------------
// ---------- SETUP / LOOP ----------
// ------------------------------------

void setup() {
  // 1. MCP3008 Pin Setup (Software SPI) - RETAINED ORIGINAL PINS
  pinMode(MCP_CLK_PIN, OUTPUT);  // GPIO 2
  pinMode(MCP_DIN_PIN, OUTPUT);  // GPIO 3
  pinMode(MCP_DOUT_PIN, INPUT);  // GPIO 4
  pinMode(MCP_CS_PIN, OUTPUT);   // GPIO 5
  digitalWrite(MCP_CS_PIN, HIGH);
  digitalWrite(MCP_CLK_PIN, LOW);
  
  // 2. OLED I2C (I2C0 using Wire) - SHIFTED TO GPIO 8/9
  Wire.setSDA(OLED_SDA_PIN); // GPIO 8 (Pin 11)
  Wire.setSCL(OLED_SCL_PIN); // GPIO 9 (Pin 12)
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(50);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false)) {
    while (1) { delay(100); }
  }
  display.clearDisplay();
  display.display();

  // Splash Screen 
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  oledPrintLine(0, "AD8232 ECG");
  oledPrintLine(1, "Monitor System");
  oledPrintLine(3, "ONLINE");
  display.display();
  delay(3000); 

  // 3. Buzzer 
  pinMode(BUZZER_PIN, OUTPUT); 
  beepOff();

  // 4. GSM UART0
  Serial1.setTX(GSM_TX_PIN); // GPIO 0
  Serial1.setRX(GSM_RX_PIN); // GPIO 1
  Serial1.begin(GSM_BAUD);
  delay(50);

  // 5. GPS SoftwareSerial
  gpsSS.begin(GPS_BAUD);
  gpsSS.listen();
  serviceGPS(500);

  // Initialize BPM state
  readBPM();
  currentBPM = 0;
  anomalyStartTime = 0;
  isAnomaly = false;
}

void loop() {
  serviceGPS(50);
  
  readBPM(); // This updates currentBPM based on ECG R-peaks

  // Anomaly detection logic (uses last valid BPM)
  if (currentBPM >= 0 && (currentBPM < BPM_ANOMALY_LOW || currentBPM > BPM_ANOMALY_HIGH)) {
    if (!isAnomaly) {
      isAnomaly = true;
      anomalyStartTime = millis();
    } else {
      if (millis() - anomalyStartTime >= ANOMALY_WINDOW_MS) {
        // ANOMALY TRIGGERED (5 second threshold met)
        handleAnomalyBlocking(currentBPM);
        // Execution halts inside handleAnomalyBlocking/dialAfterShowingResetAndHalt
      }
    }
  } else {
    isAnomaly = false;
    anomalyStartTime = 0;
  }

  oledPrintBPM(currentBPM);

  // Display anomaly timer 
  if (isAnomaly) {
    char timer_l[17];
    int time_elapsed_s = (millis() - anomalyStartTime) / 1000;
    
    if (time_elapsed_s < 0) time_elapsed_s = 0; 
    
    // Determine if it's Low or High anomaly for display clarity
    const char* type = (currentBPM < BPM_ANOMALY_LOW) ? "LOW" : "HIGH";

    // Displays time elapsed against the 5-second window
    snprintf(timer_l, sizeof(timer_l), "ALERT %s: %d/%d s", type, time_elapsed_s, ANOMALY_WINDOW_MS / 1000);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 54);
    display.print(timer_l);
    display.display();
  }

  delay(10); 
}