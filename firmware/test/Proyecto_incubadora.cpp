/*
# SPDX-FileCopyrightText: 2024 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/

/*
Proyect: incubadora-iowlabs
Author: WAC@iowlabs
Description:

  Este código implementa el software general de la incubador. Cuenta con un control de temperatura por medio de un PID
  En paralelo acciona un motor Stepper
  Ademas cuenta con una interfaz de usuario formada por un rotary encoder y una pantalla LCD y de un control por Wi-fi mpor medio de MQTT
Esta programado con un ESP32.
El control del motor stepper se implementa en un nucleo independiente. En el nucleo principal se ejecutan todas las demas tareas.




- Módulos:
	- Rotary switch: Encoder para la interfaz gráfica
	- Pantalla LCD: Pantalla I2C de 16x2 para la interfaz gráfica
	- Sensor de temperatura BMP085: Sensor de temperatura y humedad ambiente. Controlado por I2C para el lazo de control de temperatura.
	- Modúlo EasyDriver para el control del motor stepper del shaker.
	- Modúlo Puente H IBT_2: para el control bidireccional del peltier.

Actuadores:
	- Peltier
	- Ventiladores disipadores del Peltier de 12V
	- Ventilador circulador de aire para el peltier de 12V
	- Motor Stepper nema 17


Comandos MQTT

//.\mosquitto_sub.exe -h 35.223.234.244 -p 1883 -u "iowlabs" -P "!iow_woi!" -t "test/topic"
//.\mosquitto_pub.exe -h 35.223.234.244 -p 1883 -u "iowlabs" -P "!iow_woi!" -t "test/topic" -m "SPEED: 150"
//.\mosquitto_pub.exe -h 35.223.234.244 -p 1883 -u "iowlabs" -P "!iow_woi!" -t "test/topic" -m "SETPOINT: 30"

*/

//--------------------Librerias----------------------


#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <ContinuousStepper.h>
#include <SPI.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <FS.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>

//.\mosquitto_sub.exe -h 35.223.234.244 -p 1883 -u "iowlabs" -P "!iow_woi!" -t "test/topic"
//.\mosquitto_pub.exe -h 35.223.234.244 -p 1883 -u "iowlabs" -P "!iow_woi!" -t "test/topic" -m "SPEED: 150"
//.\mosquitto_pub.exe -h 35.223.234.244 -p 1883 -u "iowlabs" -P "!iow_woi!" -t "test/topic" -m "SETPOINT: 30"

TaskHandle_t Task1;
TaskHandle_t Task2;

// Pines
#define PIN_DT 18   // Pin conectado a DT (encoder)
#define PIN_CLK 19  // Pin conectado a CLK (encoder)
#define PIN_SW 4   // Pin conectado a SW (botón encoder)

#define PIN_DIR  27  // Pin de dirección
#define PIN_STEP 14  // Pin de pasos

// Pines para el L298N y el control del Peltier
#define ENA 25  // PWM para regular la potencia
#define ENB 26  // PWM para regular la potencia

#define IN1 33  // Dirección 1
#define IN2 32  // Dirección 2

#define CH_H 5
#define CH_C 4

// Constantes
#define STARTING_SETPOINT 			37.0   // Pin conectado a DT (encoder)
#define STARTING_STEP_PER_SECOND 	1600   // Pin conectado a DT (encoder)
#define STARTING_RPM 60

// Setting PWM properties
const int freq = 2000;
const int pwmChannel = 0;
const int resolution = 8;

const char* cmd;
int arg = 0;

int pwm 	= 200;
float temp 	= 0.0;
float temp_sp = STARTING_SETPOINT;
long rpm = STARTING_RPM;
float sps = rpm *(1600/60); //step per seconds
String output = "";
long timestamp = 0;

// Definir estados
enum State {WELCOME, SELECT, TEMP_SCREEN, RPM_SCREEN};
State currentState = WELCOME;

// Inicialización del RotaryEncoder
RotaryEncoder encoder(PIN_DT, PIN_CLK, RotaryEncoder::LatchMode::TWO03);

// Inicializar el ContinuousStepper
ContinuousStepper<StepperDriver> stepper;

// Inicialización del sensor de temperatura
Adafruit_BMP085 bmp;

// Inicializacion PID
double Setpoint, Input, Output;
double Kp = 10, Ki = 1, Kd = 5;  // Valores para el PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Inicializar la pantalla LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Selection
enum Selection {TEMP_SELECTION, RPM_SELECTION};
Selection currentSelection = TEMP_SELECTION;

// Credenciales WiFi y MQTT
#define mqtt_server "35.223.234.244"
#define mqtt_user "iowlabs"
#define mqtt_password "!iow_woi!"
#define ip_esp32 1883

//const char* ssid = "VTR-2879137";       
//const char* password = "mknn5qsMrhx4"; 

const char* ssid = "iownwater";       
const char* password = "temp3_NL156$"; 

bool manualSetpointChange = false;  // Indica si el setpoint fue modificado manualmente


WiFiClient espClient;
PubSubClient client(espClient);

JsonDocument wifi_command;

// Constantes
#define STARTING_SETPOINT 37   // Pin conectado a DT (encoder)
#define STARTING_STEP_PER_SECOND 1600   // Pin conectado a DT (encoder)
#define STARTING_RPM 60   // Pin conectado a DT (encoder)

// Variables
int tempSetpoint = STARTING_SETPOINT;  // Setpoint inicial 
long RPM = STARTING_RPM;  // Velocidad inicial del motor en RPM
JsonDocument press_command;
float temperature = 0;  // Almacena la temperatura medida


void Task1code(void *pvParameters); 
void Task2code(void *pvParameters); 

void setupWiFi();

void setup() {
  Serial.begin(115200);  // Inicia la comunicación serial

  setupWiFi(); // Wifi

  // Inicializar pines del rotary encoder
  pinMode(PIN_SW, INPUT_PULLUP);

  // Inicializar stepper
  stepper.begin(PIN_STEP, PIN_DIR);
  stepper.spin(STARTING_STEP_PER_SECOND); // Rota a 1600 pasos por segundo

  // Inicialización del sensor de temperatura
  if (!bmp.begin())
	{
    	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	}

  
	pinMode(ENA, OUTPUT);
	pinMode(ENB, OUTPUT);
	pinMode(IN1, OUTPUT);
 	pinMode(IN2, OUTPUT);

	//HEAT
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, HIGH);

	 // Configuración del PWM
  	ledcSetup(CH_H, 2000, 8);    // Canal 0, 20 kHz, resolución de 8 bits
 	ledcAttachPin(ENA, CH_H);  // Asociar el pin ENA al canal 0

 	// Configuración del PWM
	ledcSetup(CH_C, 2000, 8);    // Canal 0, 20 kHz, resolución de 8 bits
	ledcAttachPin(ENB, CH_C);  // Asociar el pin ENA al canal 0

  // Inicializa el PID
  Setpoint = STARTING_SETPOINT; //Setpoint
  myPID.SetMode(AUTOMATIC);  // PID en modo automático

  // Iniciar comunicación de la pantalla
  lcd.init();
  lcd.backlight();

  // mensaje inicial
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bienvenido");

  // Definir 2 Nucleos
  
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500); 
}

// ------------------------- LOOP ---------------------------------


void execute_rotate_encoder();
void execute_press_encoder();
void handle_heater();
void handle_motor();
void handle_wifi();
void execute_json_commands();

void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    execute_rotate_encoder();
    execute_press_encoder();
    handle_heater();
    handle_wifi();
    execute_json_commands();

    vTaskDelay(1);  // Cede el control por un tick del sistema
  } 
}

void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    handle_motor();
  }
}

void loop(){

}

// ------------------------- LCD ---------------------------------

void print_selection_on_lcd(){
  lcd.clear();

  //Primera Linea
  lcd.setCursor(0, 0);
  lcd.print("Seleccione");

  // Resaltar la opción seleccionada
  if (currentSelection == TEMP_SELECTION) { 
    lcd.setCursor(0, 1);
    lcd.print("> Temp");
    lcd.setCursor(9, 1);
    lcd.print("RPM ");
  } 
  else if (currentSelection == RPM_SELECTION) {
    lcd.setCursor(0, 1);
    lcd.print("Temp ");
    lcd.setCursor(9, 1);
    lcd.print("> RPM");
  }
}

void print_temp_on_lcd(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp actual: " );  
  lcd.print(temperature);  
  lcd.setCursor(0, 1);
  lcd.print("Setpoint: ");
  lcd.print(tempSetpoint);  // Mostrar el setpoint 
}

void print_rpm_on_lcd(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RPM:");
  lcd.setCursor(4, 0);   // Coloca el cursor en la primera línea, después de "RPM:"
  lcd.print("      ");   // Borra el valor anterior
  lcd.setCursor(4, 0);   // Coloca el cursor nuevamente para mostrar el nuevo valor
  lcd.print(RPM);
}

// ------------------------- ROTATE ---------------------------------

long lastPosition = 0;
void rotate_temp(long delta);
void rotate_rpm(long delta);

void execute_rotate_encoder() {
  if (currentState == WELCOME) {
    return;
  } 

  long newPosition = encoder.getPosition();
  long delta = newPosition - lastPosition;
  if (delta == 0){
    return;
  }
  lastPosition = newPosition;
  
  if (currentState == SELECT) {
    if (currentSelection == TEMP_SELECTION) {
      currentSelection = RPM_SELECTION;
    }
    else if (currentSelection == RPM_SELECTION) {
      currentSelection = TEMP_SELECTION;
    }
    print_selection_on_lcd();
  }
  
  else if (currentState == TEMP_SCREEN) {
    rotate_temp(delta);
  } 
  
  else if (currentState == RPM_SCREEN) {
    rotate_rpm(delta);
  }
}

void rotate_temp(long delta) {
  tempSetpoint += delta;  // Ajusta el setpoint de temperatura
  Setpoint = tempSetpoint;  // Sincroniza el setpoint del PID
  manualSetpointChange = true;  // Marca que el cambio fue manual

  // Actualiza el LCD
  print_temp_on_lcd();

  // Imprime en la consola
  Serial.print("Nuevo Setpoint manual: ");
  Serial.println(tempSetpoint);
}


void rotate_rpm(long delta){
  // Ajusta RPM
  RPM += delta;

  // Actualiza el LCD con el nuevo valor de RPM
  print_rpm_on_lcd();

  // Imrpime en la consola
  Serial.print("Velocidad establecida en: ");
  Serial.print(RPM);
  Serial.println(" revoluciones por minuto");
}

// ------------------------- PRESS ---------------------------------

int last_sw_state = HIGH;

void execute_press_encoder() {
  int sw_state = digitalRead(PIN_SW);
  bool was_sw_pressed = (sw_state == LOW && last_sw_state == HIGH);
  last_sw_state = sw_state;

  if (!was_sw_pressed) {
    return;  // Si no se presionó el botón, no hace nada
  }

  press_command.clear();

  // Manejo del flujo de estados
  if (currentState == WELCOME) {
    press_command["cmd"] = "selectScreen";
  } 
  else if (currentState == SELECT) {
    if (currentSelection == TEMP_SELECTION) {
      press_command["cmd"] = "TempScreen";
    } else if (currentSelection == RPM_SELECTION) {
      press_command["cmd"] = "RPMScreen";
    }
  } 
  else if (currentState == TEMP_SCREEN || currentState == RPM_SCREEN) {
    // Regresar al estado de selección cuando se presiona el botón
    press_command["cmd"] = "selectScreen";
  }

  execute_json_commands();  // Ejecutar el comando correspondiente
}



// ------------------------- MOTOR ---------------------------------

void handle_motor(){
  encoder.tick();
  stepper.loop();
}

// ------------------------- HEATER ---------------------------------

unsigned long lastTempRead = 0;  // Última vez que se leyó la temperatura
#define TEMP_INTERVAL 2000  // Intervalo de lectura de la temperatura

unsigned long lastPIDRead = 0;  // Última vez que se leyó el PID
#define PID_INTERVAL 10  // Intervalo de lectura de la temperatura

void handle_heater() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastTempRead >= TEMP_INTERVAL) {
    lastTempRead = currentMillis; 

    // Lectura de la temperatura del sensor
    temperature = bmp.readTemperature();

    // Mostrar la temperatura medida en el LCD
    if (currentState == TEMP_SCREEN) {
      print_temp_on_lcd();
    }

    // También imprime la temperatura en la consola
    Serial.print("Temperatura medida: ");
    Serial.println(temperature);
    Serial.print("Salida PID (Pin 25): ");
    Serial.println(Output);
  }

  if (currentMillis - lastPIDRead >= PID_INTERVAL) {
    lastPIDRead = currentMillis;

    Input = temperature;  // La entrada del PID es la temperatura medida
    myPID.Compute();  // Calcula el PID

    // Limitar la salida entre 100 y 255
    if (Output < 0) Output = 0;
    if (Output > 255) Output = 255;

    pwm =  (int)Output;

  	//analogWrite(ENA, 255);
	//HEAT
	if(Input < temp_sp)
	{

		//Serial.println("heating");
		ledcWrite(CH_H, pwm);
		ledcWrite(CH_C, 0);

	}
	else
	{

		//Serial.println("cooling");
		ledcWrite(CH_H, 0);
		ledcWrite(CH_C, (255 - pwm));
	}
  }
}



// ------------------------- wifi ---------------------------------

void callback(char* topic, byte* payload, unsigned int length);

// Función para conectar a WiFi
void setupWiFi() {
  // delay(10); // TODO make sure this is nessesary

  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi conectado.");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  
  client.setServer(mqtt_server, ip_esp32); // Configurar servidor MQTT
  client.setCallback(callback);  // Asignar función para manejar los mensajes MQTT
}

// Función para reconectar al servidor MQTT
void reconnect() {
  // Reintenta la conexión al servidor MQTT si está desconectado
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    
    // Intenta conectar
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("conectado");
      // Se suscribe al tópico llamado "test/topic"
      client.subscribe("test/topic");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" reintentando en 5 segundos");
      delay(5000);
    }
  }
}


// Función para manejar los mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en el tópico: ");
  Serial.println(topic);

  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Mensaje recibido:");
  Serial.println(message);

  wifi_command.clear();

  // Verifica si el mensaje sigue el formato "SPEED: valor"
  if (message.startsWith("SPEED:")) {
    String speedValue = message.substring(6);  // Extrae el valor de velocidad
    int newRPM = speedValue.toInt();  // Convierte el valor a entero

    wifi_command["cmd"] = "set_speed";
    wifi_command["args"]["value"] = newRPM;
  }
  
  // Verifica si el mensaje sigue el formato "SETPOINT: valor"
  else if (message.startsWith("SETPOINT:")) {
    String setpointValue = message.substring(9);  // Extrae el valor del setpoint
    int newSetpoint = setpointValue.toInt();  // Convierte el valor a entero

    wifi_command["cmd"] = "set_temperature";
    wifi_command["args"]["value"] = newSetpoint;
  }

  String output;
  serializeJson(wifi_command, output);
  Serial.println("Mensaje convertido a JSON:");
  Serial.println(output);  // Imprime el JSON convertido en la terminal
}


void handle_wifi() {
  if (!client.connected()) {
    reconnect();  // Reconectar si se pierde la conexión MQTT
  }

  client.loop();  // Mantiene la conexión MQTT activa
}

// ------------------------- JSON COMMANDS ---------------------------------

void execute_single_json_command(JsonDocument* current_command);

void execute_json_commands() {
  execute_single_json_command(&press_command);
  execute_single_json_command(&wifi_command);
}

void execute_single_json_command(JsonDocument* current_command) {
  if (current_command->isNull()){
    return;
  }

  const char* cmd = (*current_command)["cmd"];  // comando
  if (strcmp(cmd, "selectScreen") == 0) {
    currentState = SELECT;
    print_selection_on_lcd();
  }

  else if (strcmp(cmd, "TempScreen") == 0) {
    currentState = TEMP_SCREEN;
    print_temp_on_lcd();
  }

  else if (strcmp(cmd, "RPMScreen") == 0) {
    currentState = RPM_SCREEN;
    print_rpm_on_lcd();
  }
  
  else if (strcmp(cmd, "set_temperature") == 0) {
  // Solo aplica el comando MQTT si no hubo un cambio manual reciente
  if (!manualSetpointChange) {
    JsonObject args = (*current_command)["args"];  // Extrae los argumentos
    int newSetpoint = args["value"];  // Obtiene el nuevo setpoint

    tempSetpoint = newSetpoint;  // Actualiza el setpoint
    Setpoint = tempSetpoint;  // Asegura que el PID use el nuevo setpoint

    Serial.print("Setpoint de temperatura establecido por MQTT en ");
    Serial.print(tempSetpoint);
    Serial.println(" grados");

    // Actualiza la pantalla si estás en la pantalla de temperatura
    if (currentState == TEMP_SCREEN) {
      print_temp_on_lcd();
    }
  } else {
    // Restablece la bandera para permitir futuros comandos MQTT
    manualSetpointChange = false;
  }
}

  else if (strcmp(cmd, "set_speed") == 0) {
    currentState = SELECT;
    print_selection_on_lcd();

    JsonObject args = (*current_command)["args"];  // argumentos
    long speed = args["value"];  // Obtiene el valor de la velocidad
    if (speed <= 0) {
      Serial.println("Error: la velocidad debe ser mayor que 0.");
    } else {
      RPM = speed;  // Actualiza el valor de steps_per_second
      float steps_per_second = (1600/60) * RPM;
      stepper.spin(steps_per_second);  // Aplica la nueva velocidad al motor
      
      Serial.print("Velocidad establecida en: ");
      Serial.print(RPM);
      Serial.println(" revoluciones por minuto");
    }
  }

  current_command->clear();
}


