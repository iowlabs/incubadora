/*
# SPDX-FileCopyrightText: 2024 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/

/*
Proyect: incubadora-iowlabs
Author: WAC@iowlabs
Description:
	Este código implementa una versión sencilla del control para la incubadora.
Cuenta con un control de temperatura por medio de un PID. Eecibe comandos vía serial y actualiza el sp de temperatura y las rpm del shaker
Envía un json con el tiempo transcurrido, la temperatura actual, las rpm acutales y el sp actual.
Esta programado con un ESP32.

El control del motor stepper se implementa en un nucleo independiente. En el nucleo principal se ejecutan todas las demas tareas.

- Módulos:
	- Rotary switch: Encoder para la interfaz gráfica
	- Pantalla LCD: Pantalla I2C de 16x2 para la interfaz gráfica
	- Sensor de temperatura sht31: Sensor de temperatura y humedad ambiente. Controlado por I2C para el lazo de control de temperatura.
	- Modúlo EasyDriver para el control del motor stepper del shaker.
	- Modúlo Puente H IBT_2: para el control bidireccional del peltier.

Actuadores:
	- Peltier
	- Ventiladores disipadores del Peltier de 12V
	- Ventilador circulador de aire para el peltier de 12V
	- Motor Stepper nema 17

*/


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_BMP085.h>
#include <Adafruit_SHT31.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <ContinuousStepper.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <PID_v1.h>


// Pines para el L298N y el control del Peltier
#define ENA 25  // PWM para regular la potencia
#define ENB 26  // PWM para regular la potencia

#define IN1 33  // Dirección 1
#define IN2 32  // Dirección 2

#define PIN_DIR  27  // Pin de dirección
#define PIN_STEP 14  // Pin de pasos

#define CH_H 5
#define CH_C 4

// Constantes
#define STARTING_SETPOINT 			37.0   // Pin conectado a DT (encoder)
#define STARTING_STEP_PER_SECOND 	1600   // Pin conectado a DT (encoder)
#define STARTING_RPM 				60   // Pin conectado a DT (encoder)

//handle motor task
TaskHandle_t Task2;
// Declaración de la biblioteca para el sensor BMP085
//Adafruit_BMP085 bmp;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Inicializar el ContinuousStepper
ContinuousStepper<StepperDriver> stepper;

// Inicializar la pantalla LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Inicializacion PID
double Setpoint, Input, Output;
double Kp = 15, Ki = 2, Kd = 10;  // Valores para el PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// Setting PWM properties
const int freq = 2000;
const int pwmChannel = 0;
const int resolution = 8;

const char* cmd;
int arg = 0;

int pwm 	  = 200;
float temp 	= 0.0;
float temp_sp = STARTING_SETPOINT;
long rpm  = STARTING_RPM;
float sps = rpm *(1600/60); //step per seconds
String output   = "";
long timestamp  = 0;

String pubData(void);
void processCmd();
void handle_motor();
void Task2code(void *pvParameters);

void setup()
{
  // Inicialización del monitor serial
	Serial.begin(115200);

	Wire.begin();

	// Inicialización del sensor sht31
  if (! sht31.begin(0x44))
	{
    	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	}

	pinMode(PIN_STEP, OUTPUT);
	pinMode(PIN_DIR, OUTPUT);

	// Inicializar stepper
	stepper.begin(PIN_STEP, PIN_DIR);
	stepper.spin(0);
	//stepper.spin(int(sps)); // Rota a 1600 pasos por segundo
	//stepper.spin(200); // Rota a 1600 pasos por segundo

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

	// Iniciar comunicación de la pantalla
	lcd.init();
	lcd.backlight();

	// mensaje inicial
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("LabTecLibre");

	// Inicializa el PID
	Setpoint = STARTING_SETPOINT; //Setpoint
	myPID.SetMode(AUTOMATIC);  // PID en modo automático
  // Limitar la salida del PID entre 0 y 255
  myPID.SetOutputLimits(0, 255);

	//create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      Task2code,   /* Task function. */
      "Task2",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      &Task2,      /* Task handle to keep track of created task */
      1);          /* pin task to core 1 */

	Serial.println("ready");
}

void loop()
{
	// Lectura de la temperatura del sensor BMP085
	temp = sht31.readTemperature();
	Input = (double)temp;
	//Calculo de PID
	myPID.Compute();
	pwm =  (int)Output;

	//temp = (int(temp *100.0 + 0.5))/100;

	lcd.setCursor(0,1);
	lcd.print("Temp:");
  lcd.print(temp);

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

	//Shaker
	stepper.loop();
	timestamp = millis();
	output = pubData();
	Serial.println(output);

	if( Serial.available() > 0 )
	{
		processCmd();
	}

  	// Pequeña espera antes de la siguiente lectura
  delay(500);
}

void Task2code( void * pvParameters )
{
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;)
  {
    handle_motor();
  }
}

void handle_motor()
{
  stepper.loop();
}

String pubData()
{
	StaticJsonDocument<512> doc_tx;

	doc_tx["time"]  	= timestamp;
	doc_tx["temp"]  	= temp;
  doc_tx["temp_sp"] = temp_sp;
	doc_tx["pwm"]  		= pwm;
	doc_tx["rpm"]  		= rpm;

	String json;
	serializeJson(doc_tx, json);
	return json;
}

void processCmd()
{
	StaticJsonDocument<1024> doc_rx;
	DeserializationError error_rx;
	//check for error
	error_rx = deserializeJson(doc_rx, Serial);
	if (error_rx)
	{
		Serial.print(F("deserializeJson() failed: "));
		Serial.println(error_rx.c_str());
	}

	//parsing incoming msg
	cmd = doc_rx["cmd"];
	arg = doc_rx["arg"];

	//prossesing incoming command
  if(strcmp(cmd,"sp")==0)
  {
    temp_sp = float (arg);
		Setpoint = double(arg);
  }
  else if(strcmp(cmd,"stop")==0)
  {
    ledcWrite(ENA,0);
		stepper.stop();
		stepper.powerOff();
	}
  else if(strcmp(cmd,"start")==0)
  {
    ledcWrite(ENA,Output);
		stepper.powerOn();
		stepper.spin(sps);
  }
  else if(strcmp(cmd,"rpm")==0)
  {
    rpm =  float (arg);
		sps = (1600/60)*rpm;
		stepper.spin(sps); // speed in step per seconds
  }

}
