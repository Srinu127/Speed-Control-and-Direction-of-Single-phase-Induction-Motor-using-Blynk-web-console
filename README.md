# Speed-Control-and-Direction-of-Single-phase-Induction-Motor-using-Blynk-web-console
//By using this code you can do 1) Speed Control of IM 2) Change the direction of motor 3) get real time temp, rpm, current etc
Required Hardware: 
1) A small single phase induction motor(50watt) =   1 number
2) Relay Driver circuit (NOTE:Logic HIGH to turn on Relay) = 2 Nos
3) Electronic fan regualtor (NOTE: remove a single resistace of RC oscillation and place output of MOC 3020 open collector side and connect LED pin to ESP32 side.
4) Current Transformer( you can get it from old damaged energy meter)( ratio 20/5) make a diode bridge , place a 150k resistance at AC input side and connect Current Transformer two leads in AC input side where 150k is already soldered.
5) ESP32 any board
6) Buzzer
7) IR Sensor Module to measure the motor rpm (not new thing for you)
8) A toggle switch (fix it in board)
9) 230v to 12 v conveter circuit
10) Buck converter , vary and fix the output voltage to 3.3v to get esp32 biased


// paste Blynk credentials here
#define BLYNK_TEMPLATE_ID "TMPL3E9ANMDrF"
#define BLYNK_TEMPLATE_NAME "Speed Controller for Induction motor"
#define BLYNK_AUTH_TOKEN "H8O-mjl-HVg2HMjWUKvjB-ecW_DdPCB0"

// Here i used esp32 
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

// Blynk credentials
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "MWSLAB2"; // 
char pass[] = "Bit1234@";
// Pins
#define DHTPIN 33
#define DHTTYPE DHT11
#define IR_PIN 2
#define LDR_PIN 5 // to moc3020 led positive terminal 
#define DOR_PIN 21            //Normally closed Clockwise and Normally Open CounterClock wise
#define MAIN_POWER_PIN 22
const int  CURRENT_PIN  = 34; // converted current to voltage value 
#define BUZZER_PIN 25
DHT dht(DHTPIN, DHTTYPE);
volatile int pulseCount = 0;
unsigned long lastRPMTime = 0;
float rpm = 0.0;
float currentAmp =0.0; 
int pwmValue = 0;
bool powerButtonState = false;
bool overheatDetected = false;
bool overcurrent = false;
BlynkTimer timer;

void IRAM_ATTR countPulse() {
  pulseCount++;

}

void calculateRPM() {
  unsigned long currentTime = millis();
  unsigned long timeDiff = currentTime - lastRPMTime;

  if (timeDiff >= 1000) {
    rpm = ((pulseCount/15) * (60000.0/timeDiff));
    pulseCount = 0;
    //delay(20);
    lastRPMTime = currentTime;
    
   Serial.print("rpm=  ");
   Serial.println(rpm);
   Blynk.virtualWrite(V1,rpm);

  }
}

void readTemperature() {
  float temp = dht.readTemperature();

  if (isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Blynk.virtualWrite(V0, temp);
  Serial.print("Temperature: ");
  Serial.println(temp);

  if (temp > 39.1) {
    digitalWrite(MAIN_POWER_PIN, LOW);   // Turn off main power
    digitalWrite(BUZZER_PIN, HIGH);     // Keep buzzer alert ON
    overheatDetected = true;            // Flag overheat condition
    Blynk.virtualWrite(V2, "Motor OFF - Overheat");
  } else if (temp < 38.0 && overheatDetected) {
    digitalWrite(MAIN_POWER_PIN, HIGH); // Restore power if temp is safe
    digitalWrite(BUZZER_PIN, LOW);      // Turn off buzzer
    overheatDetected = false;           // Reset overheat condition
    Blynk.virtualWrite(V2, "Motor ON");
  }

  // Optional: Add buzzer alert for intermediate range if needed
  if (temp > 38.1 && temp < 39.0) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
  } else {
    digitalWrite(BUZZER_PIN, LOW); // Keep buzzer off in healthy condition
  }
}

   void readMotorCurrent() {
   int current = (analogRead(34)) ;
   float currentAmp = current * (3.3 / 4095.0);
  Blynk.virtualWrite(V3, currentAmp);  // Send value to Blynk
  Serial.print("Motor Current: ");
  Serial.println(currentAmp);
  
  if (currentAmp > 1.50) {
    //digitalWrite(POWER, LOW);        // Turn off motor
    digitalWrite(MAIN_POWER_PIN, LOW);   // Turn off main power
    digitalWrite(BUZZER_PIN, HIGH);

    overcurrent = true;
    Blynk.virtualWrite(V2, "Motor OFF - Overcurrent");
  } else {
    overcurrent = false;
    //digitalWrite(MAIN_POWER_PIN, HIGH);
   // if (powerButtonState) {
    //  digitalWrite(MAIN_POWER_PIN, HIGH); // Restore power
   // }
    Blynk.virtualWrite(V2, "Motor ON");
  }

}

// Virtual button V4 handler
BLYNK_WRITE(V4) {
  powerButtonState = param.asInt(); // 1 or 0

  if (!overheatDetected) {
    digitalWrite(MAIN_POWER_PIN, powerButtonState ? HIGH : LOW);
  }

  Serial.print("Manual Power State: ");
  Serial.println(powerButtonState ? "ON" : "OFF");
}

 

BLYNK_WRITE(V5) {
  pwmValue = param.asInt(); // Retrieve slider value
  analogWrite(LDR_PIN,pwmValue);
}

BLYNK_WRITE(V6) {
  int value = param.asInt(); // Retrieve slider value
  value ? digitalWrite(21,HIGH) : digitalWrite(21,LOW); // CHANGE DOR OF MOTOR
}

void setup() {
  Serial.begin(115200);

  pinMode(IR_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  //pinMode(POWER_PIN, OUTPUT);
  pinMode(MAIN_POWER_PIN, OUTPUT); 
  pinMode(LDR_PIN, OUTPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(DOR_PIN, OUTPUT);
 // digitalWrite(POWER_PIN, HIGH);
  digitalWrite(MAIN_POWER_PIN, LOW); // Start OFF until user powers ON

  dht.begin();
  Blynk.begin(auth, ssid, pass);

  attachInterrupt(digitalPinToInterrupt(IR_PIN), countPulse, RISING);

  timer.setInterval(1000L, calculateRPM);
  timer.setInterval(1500L, readMotorCurrent);
  timer.setInterval(2000L, readTemperature);
  
}

void loop() {
  //float current = (analogRead(CURRENT_PIN) / 4095.0) * 3.3;
 
  Blynk.run();
  timer.run();
  
}
