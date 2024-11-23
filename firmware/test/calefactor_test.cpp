/*
# SPDX-FileCopyrightText: 2024 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/

/*
Proyect: incubadora-iowlabs
Author: WAC@iowlabs
*/


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>

// Declaración de la biblioteca para el sensor BMP085
Adafruit_BMP085 bmp;

// Pines para el L298N y el control del Peltier
const int ENA = 25;  // PWM para regular la potencia
const int IN1 = 33;  // Dirección 1
const int IN2 = 32;  // Dirección 2

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;


void setup()
{
	// Inicialización del monitor serial
	Serial.begin(9600);

	Wire.begin();

	// Inicialización del sensor BMP085
	if (!bmp.begin())
	{
    	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	}

	pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
 	pinMode(IN2, OUTPUT);


	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, HIGH);

	 // Configuración del PWM
  	ledcSetup(0, 2000, 8);    // Canal 0, 20 kHz, resolución de 8 bits
 	ledcAttachPin(ENA, 0);  // Asociar el pin ENA al canal 0
  	ledcWrite(0, 200);

	Serial.println("ready");
}

void loop()
{
	// Lectura de la temperatura del sensor BMP085
	float temperature = bmp.readTemperature();

	// Imprimir la temperatura en el monitor serial
	Serial.print("Temperatura actual: ");
	Serial.print(temperature);
	Serial.println(" *C");

  	//analogWrite(ENA, 255);
  	ledcWrite(ENA, 255);
  	// Pequeña espera antes de la siguiente lectura
  	delay(500);
}
