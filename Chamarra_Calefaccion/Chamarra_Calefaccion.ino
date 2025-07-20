//Debe instalarse el conjunto de tarjetas ESP32 desde el gestor de tarjetas
//Y seleccionarse la tarjeta ESP32 Dev Module, compatible con el modulo TTGO LORA SX1278 ESP32, y compilando sin problemas
//Necesario ir al administrador de dispositivos e instalar driver de la tarjeta USB, CP2102

//Si deseas comprarel kit exacto de tarjeta, componentes y accesorios, crearé un paquete en mi pagina www.aceru.mx
//Pero puedes adquirirlos tambien por separado en electronicas, solo cuidando que sean los modelos precisos.
//O bien, si lo deseas, adaptando el codigo y conexiones a tus tarjetas.

//Actualizaré este código conforme voy subiendo mis videos de avance a TikTok e Instagram
//Inicio de proyecto liberado 20 Julio 2025 | FerAceru

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h" 
#include "images.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_INA219.h>

// Define el pin que vas a usar para el bus 1-Wire
#define ONE_WIRE_BUS 16  // Cambia el número por el pin que uses

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_INA219 ina219;


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

