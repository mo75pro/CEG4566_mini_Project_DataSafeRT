
#define BLYNK_TEMPLATE_ID "TMPL2MpyYMZmy"
#define BLYNK_TEMPLATE_NAME "DataSafeRT"
#define BLYNK_AUTH_TOKEN "eQzVA3jipob2FzVpQSz_qPHNyKnzyyT9"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "LesSoeursAbbey";
char pass[] = "ABBEY2.0";

BlynkTimer timer;


#include <Arduino.h>
#include <DHTesp.h>
#include <ESP32Servo.h>
#include <math.h>

// ---------------------------
// Définition des broches via macros
// ---------------------------

// Capteurs
#define DHT_PIN          4      // DHT22
#define IR_INTRUSION_PIN 15     // Capteur IR pour intrusion
#define MQ2_PIN          2      // Capteur MQ‑2 (fumée) sur la broche ADC 2
#define IR_FLAME_PIN     14     // Capteur IR FLAME (flamme)

// Sorties
#define BUZZER_PIN       9      // Buzzer

// Servo
#define SERVO_PIN 19           // Broche du servo moteur
Servo myServo;                 // Objet Servo
 // LED RGB (anode commune)
#define LED_COMMON_ANODE
#ifdef LED_COMMON_ANODE
  #define LED_ON  LOW
  #define LED_OFF HIGH
#else
  #define LED_ON  HIGH
  #define LED_OFF LOW
#endif
#define LED_R_PIN 23
#define LED_G_PIN 22
#define LED_B_PIN 21



// ---------------------------
// Paramètres du DHT22
// ---------------------------
#define HOLD_TIME       5000    // 5 sec de stabilité pour DHT22
#define TEMP_THRESHOLD  0.5     // Variation minimale de température
#define HUM_THRESHOLD   1.0     // Variation minimale d'humidité

DHTesp dht;

// Variables globales pour DHT22
float finalTemp = 0, finalHum = 0;
float candidateTemp = 0, candidateHum = 0;
unsigned long candidateTimestamp = 0;
SemaphoreHandle_t sensorMutex;

// ---------------------------
// Paramètres pour MQ‑2 (fumée)
// ---------------------------
#define MQ2_THRESHOLD   1500    // Seuil ADC pour la détection de fumée
#define HOLD_TIME_MQ2   5000    // 5 sec de stabilité pour MQ‑2
bool finalSmokeState = false;
bool candidateSmokeState = false;
unsigned long candidateSmokeTimestamp = 0;
// Variable pour stocker la valeur lue du capteur MQ‑2
volatile int currentMQ2Value = 0;

// ---------------------------
// Paramètres pour IR FLAME (flamme)
// ---------------------------
#define FLAME_COUNT_THRESHOLD 3
bool finalFlameState = false;
int flameLowCount = 0;

// ---------------------------
// Variable pour capteur IR intrusion
// ---------------------------
volatile bool intrusionDetected = false;

// ---------------------------
// Tâche : Lecture du DHT22 (température/humidité)
// ---------------------------
void sensorTask(void *pvParameters) {
  (void) pvParameters;
  TempAndHumidity data = dht.getTempAndHumidity();
  if (!isnan(data.temperature)) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    finalTemp = data.temperature;
    finalHum  = data.humidity;
    candidateTemp = finalTemp;
    candidateHum  = finalHum;
    candidateTimestamp = millis();
    xSemaphoreGive(sensorMutex);
    
    Serial.print("Initial: T: ");
    Serial.print(finalTemp);
    Serial.print(" °C, H: ");
    Serial.print(finalHum);
    Serial.println(" %");
  }
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);
  
  while (1) {
    data = dht.getTempAndHumidity();
    if (isnan(data.temperature) || isnan(data.humidity)) {
      Serial.println("Erreur DHT22");
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    if ( fabs(data.temperature - candidateTemp) > TEMP_THRESHOLD ||
         fabs(data.humidity - candidateHum) > HUM_THRESHOLD ) {
      candidateTemp = data.temperature;
      candidateHum  = data.humidity;
      candidateTimestamp = millis();
    }
    if ((millis() - candidateTimestamp >= HOLD_TIME) &&
        (fabs(candidateTemp - finalTemp) > TEMP_THRESHOLD ||
         fabs(candidateHum - finalHum) > HUM_THRESHOLD)) {
      finalTemp = candidateTemp;
      finalHum  = candidateHum;
      Serial.print("Mise à jour: T: ");
      Serial.print(finalTemp, 1);
      Serial.print(" °C, H: ");
      Serial.print(finalHum, 1);
      Serial.println(" %");
    }
    xSemaphoreGive(sensorMutex);
    vTaskDelayUntil(&xLastWakeTime, period);
  }
}

// ---------------------------
// Tâche : Détection d'intrusion via capteur IR
// ---------------------------
void irDetectionTask(void *pvParameters) {
  (void) pvParameters;
  pinMode(IR_INTRUSION_PIN, INPUT);
  while (1) {
    int state = digitalRead(IR_INTRUSION_PIN);
    if (state == LOW) {
      intrusionDetected = true;
      Serial.println("Danger: Intrusion détectée dans la salle des serveurs !");
    } else {
      intrusionDetected = false;
      Serial.println("Aucune intrusion détectée.");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ---------------------------
// Tâche : Lecture du capteur MQ‑2 (fumée)
// ---------------------------
void sensorMQ2Task(void *pvParameters) {
  (void) pvParameters;
  pinMode(MQ2_PIN, INPUT);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);
  
  while (1) {
    int sensorValue = analogRead(MQ2_PIN);
    currentMQ2Value = sensorValue;  // Pour affichage
    bool currentState = (sensorValue >= MQ2_THRESHOLD);
    if (currentState != candidateSmokeState) {
      candidateSmokeState = currentState;
      candidateSmokeTimestamp = millis();
    }
    if ((millis() - candidateSmokeTimestamp >= HOLD_TIME_MQ2) &&
         (candidateSmokeState != finalSmokeState)) {
      finalSmokeState = candidateSmokeState;
    }
    vTaskDelayUntil(&xLastWakeTime, period);
  }
}

// ---------------------------
// Tâche : Lecture du capteur IR FLAME (flamme)
// ---------------------------
void sensorFlameTask(void *pvParameters) {
  (void) pvParameters;
  pinMode(IR_FLAME_PIN, INPUT_PULLUP);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);
  
  while (1) {
    int flameDigital = digitalRead(IR_FLAME_PIN);
    bool currentState = (flameDigital == LOW);
    if (currentState) {
      flameLowCount++;
    } else {
      flameLowCount = 0;
    }
    if (flameLowCount >= FLAME_COUNT_THRESHOLD) {
      finalFlameState = true;
    } else {
      finalFlameState = false;
    }
    vTaskDelayUntil(&xLastWakeTime, period);
  }
}

// ---------------------------
// Tâche : Contrôle de la LED RGB et du Buzzer
// ---------------------------
// ---------------------------
// Tâche : Contrôle de la LED RGB et du Buzzer MODIFIÉ
// ---------------------------
void ledRGBTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);
  float temp;
  
  while (1) {
    bool alertActive = false;
    
    // Vérifier les alertes
    if (intrusionDetected || finalSmokeState || finalFlameState) {
      digitalWrite(LED_R_PIN, LED_ON);
      digitalWrite(LED_G_PIN, LED_OFF);
      digitalWrite(LED_B_PIN, LED_OFF);
      alertActive = true;
    } 
    else {
      xSemaphoreTake(sensorMutex, portMAX_DELAY);
      temp = finalTemp;
      xSemaphoreGive(sensorMutex);

      if (temp >= 40.0) {
        digitalWrite(LED_R_PIN, LED_ON);
        digitalWrite(LED_G_PIN, LED_OFF);
        digitalWrite(LED_B_PIN, LED_OFF);
        alertActive = true;
      } 
      else if (temp >= 30.0) {
        digitalWrite(LED_R_PIN, LED_ON);
        digitalWrite(LED_G_PIN, LED_ON);
        digitalWrite(LED_B_PIN, LED_OFF);
      } 
      else {
        digitalWrite(LED_R_PIN, LED_OFF);
        digitalWrite(LED_G_PIN, LED_ON);
        digitalWrite(LED_B_PIN, LED_OFF);
      }
    }

    // Activer le buzzer uniquement si alerte active (LED rouge)
    digitalWrite(BUZZER_PIN, alertActive ? HIGH : LOW);
    
    vTaskDelayUntil(&xLastWakeTime, period);
  }
}
// ---------------------------
// Tâche : Contrôle du servo moteur
// ---------------------------
void servoTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);
  
  while (1) {
    // Si une alerte est présente (flamme ou fumée), ouvrir (90°)
    if (finalFlameState || finalSmokeState) {
      myServo.write(90);
      Serial.println("Servo: OPEN (ALERT detected)");
    }
    // Sinon, fermer (0°)
    else {
      myServo.write(0);
      Serial.println("Servo: CLOSED (No alert)");
    }
    vTaskDelayUntil(&xLastWakeTime, period);
  }
}

// ---------------------------
// Tâche : Affichage détaillé de l'état du système et envoi d'alertes
// ---------------------------
void statusTask(void *pvParameters) {
  (void) pvParameters;
  float temp, hum;
  
  while (1) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    temp = finalTemp;
    hum  = finalHum;
    xSemaphoreGive(sensorMutex);
    
    Serial.println("----- État du système -----");
    Serial.print("Température : ");
    Serial.print(temp, 1);
    Serial.println(" °C");
    Serial.print("Humidité    : ");
    Serial.print(hum, 1);
    Serial.println(" %");
    
    Serial.print("MQ2 (fumée) value : ");
    Serial.println(currentMQ2Value);
    
    if (temp >= 30.0 && temp < 40.0) {
      Serial.print("Alerte: Température anormale dans la salle des serveurs. Valeur : ");
      Serial.print(temp, 1);
      Serial.println(" °C");
    } else if (temp >= 40.0) {
      Serial.print("Critique: Température critique dans la salle des serveurs ! Valeur : ");
      Serial.print(temp, 1);
      Serial.println(" °C");
    } else {
      Serial.println("Température dans la plage normale.");
    }

    
    if (intrusionDetected) {
      Serial.println("Danger: Une intrusion a été détectée dans la salle des serveurs !");
    } else {
      Serial.println("Aucune intrusion détectée.");
    }
    
    if (finalSmokeState) {
      Serial.println("Alerte: Fumée détectée par le capteur MQ‑2 !");
    } else {
      Serial.println("Pas de fumée détectée.");
    }
    
    if (finalFlameState) {
      Serial.println("Alerte: Flamme détectée par le capteur IR FLAME !");
    } else {
      Serial.println("Pas de flamme détectée.");
    }
    
    Serial.println("----------------------------");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Envoi des données capteurs vers Blynk
void envoyerCapteursVersBlynk(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2000);  // toutes les 2 secondes

  while (1) {
    // 🔥 Température : Message + LED avec couleur conditionnelle
    String messageTemp;

    if (finalTemp >= 40.0) {
      messageTemp = "🔥 Température critique ! (" + String(finalTemp, 1) + " °C)";
      Blynk.setProperty(V0, "color", "#FF0000");  // Rouge
      Blynk.virtualWrite(V0, 255);                // Allume LED
    }
    else if (finalTemp >= 30.0) {
      messageTemp = "⚠️ Température élevée (" + String(finalTemp, 1) + " °C)";
      Blynk.setProperty(V0, "color", "#FFA500");  // Orange
      Blynk.virtualWrite(V0, 255);                // Allume LED
    }
    else {
      messageTemp = "✅ Température normale (" + String(finalTemp, 1) + " °C)";
      Blynk.setProperty(V0, "color", "#00FF00");  // Vert
      Blynk.virtualWrite(V0, 255);                // Allume LED
    }

    // Optionnel : Afficher le message dans un autre widget (Label/Terminal)
    Blynk.virtualWrite(V11, messageTemp);

    // 💨 Fumée
    Blynk.virtualWrite(V2, finalSmokeState ? "💨 Fumée détectée" : "✅ RAS");

    // 🔥 Flamme
    Blynk.virtualWrite(V3, finalFlameState ? "🔥 Flamme détectée" : "✅ RAS");

    // 🚨 Intrusion
    Blynk.virtualWrite(V4, intrusionDetected ? "🚨 Intrusion !" : "✅ Aucun mouvement");

    // 💧 Humidité
    Blynk.virtualWrite(V1, finalHum);

    vTaskDelayUntil(&xLastWakeTime, period);
  }
}




// ---------------------------
// Setup
// ---------------------------
void setup() {
  Serial.begin(115200);
  
  sensorMutex = xSemaphoreCreateMutex();
  
  // Configuration des broches pour les capteurs
  pinMode(DHT_PIN, INPUT);
  pinMode(MQ2_PIN, INPUT);
  pinMode(IR_FLAME_PIN, INPUT_PULLUP);
  pinMode(IR_INTRUSION_PIN, INPUT);
  
  // Configuration des broches pour la LED RGB, le buzzer et le servo
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(LED_R_PIN, LED_OFF);
  digitalWrite(LED_G_PIN, LED_OFF);
  digitalWrite(LED_B_PIN, LED_OFF);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Initialisation du servo
  myServo.attach(SERVO_PIN);
  myServo.write(0);
  Serial.println("Initialisation du servo...");
  
  // Initialisation du capteur DHT22
  dht.setup(DHT_PIN, DHTesp::DHT22);
  
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("Démarrage des tâches FreeRTOS");
  
  // Création des tâches FreeRTOS
  xTaskCreate(sensorTask,      "DHT_Task",         4096, NULL, 1, NULL);
  xTaskCreate(irDetectionTask, "IR_Intrusion_Task",2048, NULL, 1, NULL);
  xTaskCreate(sensorMQ2Task,   "MQ2_Task",         2048, NULL, 1, NULL);
  xTaskCreate(sensorFlameTask, "Flame_Task",       2048, NULL, 1, NULL);
  xTaskCreate(ledRGBTask,      "LED_Buzzer_Task",  2048, NULL, 1, NULL);
  xTaskCreate(servoTask,       "Servo_Task",       2048, NULL, 1, NULL);
  xTaskCreate(statusTask,      "Status_Task",      2048, NULL, 1, NULL); 
  xTaskCreate(envoyerCapteursVersBlynk, "UI_Task", 6144, NULL, 1, NULL);



}

// ---------------------------
// Boucle principale
// ---------------------------
void loop() {
  Blynk.run();
  timer.run();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
