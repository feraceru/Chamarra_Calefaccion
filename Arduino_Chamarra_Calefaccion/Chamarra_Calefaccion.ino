//Debe instalarse el conjunto de tarjetas ESP32 desde el gestor de tarjetas
//Y seleccionarse la tarjeta ESP32 Dev Module, compatible con el modulo TTGO LORA SX1278 ESP32, y compilando sin problemas
//Necesario ir al administrador de dispositivos e instalar driver de la tarjeta USB, CP2102


#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h" 
#include "images.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_INA219.h>
#include <PID_v1.h>
#include <EEPROM.h>

// ------------ Configuración EEPROM ------------
#define EEPROM_SIZE        16      // 8 bytes (por si agregas más datos)
#define EEPROM_ADDR_TIME   0       // dirección donde se guarda el tiempo (unsigned long, 4 bytes)
#define EEPROM_ADDR_BATT   4       // dirección para la batería consumida (float, 4 bytes)

// ------------ Variables para monitoreo ------------
unsigned long segundosEncendido = 0;
float bateriaConsumida_mAh = 0;
float capacidadBateria_mAh = 10000; // Cambia valor para powerbank o mah disponibles. Por ejemplo: Powerbank 10,000 mAh
float bateriaRestante_mAh = 10000; // Mismo valor inicial.

// ------------ Variables para control de tiempo ------------
unsigned long tiempoAnterior = 0;

// Define el pin que vas a usar para el bus 1-Wire
#define ONE_WIRE_BUS 16  // Cambia el número por el pin que uses

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_INA219 ina219;

//TRANSMISOR ALARMA
//CODIGO Y DISEÑO POR: FERACERU
// Instagram @feraceru
// Tiktok @feraceru
//

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
//#define BAND    433E6  //Cambiar banda si es para tarjetas nuevas
#define BAND    915E6 

//PINES DE LEDS INDICADORES
//PIN 13 LED
//PIN 12 LED
//const int PRG_boton = 0; //Alias para numero de pin "0" quees el botón push de la tarjeta // Tarjeta vieja
const int PRG_boton = 36; //Para tarjeta nueva 1.3v

//Address I2C y pines de conexion para Display OLED
//SSD1306 display(0x3c, 4, 15); //Para version antigua Lora Board
SSD1306 display(0x3c, 21, 22); //Para version nueva Lora Board 1.3v


int minitimer=0; //Variable de conteo para la funcion LOOP, y asi tomar acciones cada X tiempo

int buzzitimer=0;
int vibra_time=0;
double tempe_deseo=15.0;
bool calentando=0;
double tempC;
//bool estado_vibrador=0;

const int Buzz_PWM = 12;
const int mosfet = 4;
const int ledverde = 5;

/////////////////////////////////////////
// Variables PID
double setpoint;         // Temperatura objetivo en °C
double temp_actual;           // Variable para temperatura leída
double output;                // Salida del PID (0-255 para PWM)

// Ajusta estos parámetros a tu sistema:
double Kp = 10, Ki = 2, Kd = 1;   // Prueba con estos y ajusta después

PID myPID(&tempC, &output, &tempe_deseo, Kp, Ki, Kd, DIRECT);



//:::::::::::::::::::::::BATTERY CHECK
const uint8_t vbatPin = 35; //Pin para leer ADC de bateria, pin 35.




//Funcion de muestreo de logo de inicio
void logo(){
  display.clear();
  //digitalWrite(12,HIGH);
  display.drawXbm(0,0,128,64,aceru_logo); //Proyectamos logo/pantalla de inicio.
  display.display();
  delay(4000);
  //digitalWrite(12,LOW);
  }

void Pantalla_A(void){
  display.clear();                     //Limipiamos display
  display.drawXbm(0,0,128,64,chamarra_frente); //Proyectamos fondo/pantalla de trabajo

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "ACERU");

  //display.drawRect(42,20,44,15);          //Dibujamos rectangulo para dato recibido
  display.display();
}

//////////////FUNCION PARA LEER ADC de batería y revisar carga

void bateria_rev(float corrienteActual_mA) {
//Usaremos esta funcion para calcular la bateria restante con base en el consumo actual
//e imprimir pantalla con la informacion


unsigned long tiempoActual = millis();

  // Ejecuta este bloque cada segundo
  if (tiempoActual - tiempoAnterior >= 1000) {
    segundosEncendido = segundosEncendido + (tiempoActual-tiempoAnterior) /1000;
    tiempoAnterior = tiempoActual;
    

    // === Aquí leerías tu corriente actual del INA219 ===
    //float corrienteActual_mA = 1000.0; // <-- Sustituye esto por la medición real

    // Sumar el consumo de este segundo
    bateriaConsumida_mAh += (corrienteActual_mA/3600); // 1 seg = 1/3600 horas
    bateriaRestante_mAh = capacidadBateria_mAh - bateriaConsumida_mAh;
    float tiempoRestante = bateriaRestante_mAh/corrienteActual_mA;

     //Limpieza de area de texto temperatura
    display.setColor(WHITE);

    display.setFont(ArialMT_Plain_10); 
    display.drawString(0, 23, "Gastado:   " + String(bateriaConsumida_mAh) + " mA");
    display.drawString(0, 31, "Resta:      " + String(bateriaRestante_mAh) + " mA");
    display.drawString(0, 45, "Tiempo Uso:   " + String((segundosEncendido%3600)/60) + " m");
    display.drawString(0, 53, "Tiempo Rest:   " + String(tiempoRestante) + " h");
    display.display();



    // Imprime los datos en serial
    Serial.print("Tiempo: ");
    Serial.print(segundosEncendido / 3600); Serial.print("h ");
    Serial.print((segundosEncendido % 3600) / 60); Serial.print("m ");
    Serial.print(segundosEncendido % 60); Serial.print("s | ");
    Serial.print("Bateria consumida: ");
    Serial.print(bateriaConsumida_mAh, 1); Serial.print(" mAh | ");
    Serial.print("Restante: ");
    Serial.println(bateriaRestante_mAh, 1);

    Serial.println("Tiempo de uso total (m): ");
    Serial.println((segundosEncendido % 3600) / 60);
    Serial.println("Tiempo restante (h): ");
    Serial.println(tiempoRestante);

    // ---- Guarda los datos cada minuto ----
    if (segundosEncendido % 60 == 0) {
      EEPROM.put(EEPROM_ADDR_TIME, segundosEncendido);
      EEPROM.put(EEPROM_ADDR_BATT, bateriaConsumida_mAh);
      EEPROM.commit();  // IMPORTANTE en ESP32
      Serial.println("Datos guardados en EEPROM.");
    }
  }




int per_charge; //Cambiar o desarrollar codigo, necesario contabilizar las horas ya consumidas
per_charge = (bateriaRestante_mAh / capacidadBateria_mAh)*100;

//Creamos dos variables para definir rapidamente la posición del icono de la batería
int batx = 110;
int baty = 3;

    display.setColor(WHITE);
  // Si la carga se encuentra entre 0 y 19% mostramos icono de batería 0%
  if (per_charge>=0 && per_charge<20){
  display.drawFastImage(batx,baty,16,16, bateria_0);
  }
  // Si la carga se encuentra entre 20 y 39% mostramos icono de batería 20%
  if (per_charge>=20 && per_charge<40){
  display.drawFastImage(batx,baty,16,16, bateria_20);
  }

  if (per_charge>=40 && per_charge<60){
  display.drawFastImage(batx,baty,16,16, bateria_40);
  }

  if (per_charge>=60 && per_charge<80){
  display.drawFastImage(batx,baty,16,16, bateria_60);
  }

  if (per_charge>=80 && per_charge<90){
  display.drawFastImage(batx,baty,16,16, bateria_80);
  }

  if (per_charge>=90){
  display.drawFastImage(batx,baty,16,16, bateria_100);
  }
  display.display();
  
}     //Fin de función Bateria Rev




//Funcion de configuraciones iniciales
void setup() {
///////////////////SLEEP MODE
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial Inicializado... > OK");


/////////////////LECTURA EEPROM
 EEPROM.begin(EEPROM_SIZE);

  // Leer los valores guardados al iniciar
  EEPROM.get(EEPROM_ADDR_TIME, segundosEncendido);
  EEPROM.get(EEPROM_ADDR_BATT, bateriaConsumida_mAh);

    // Si el valor es inválido, no hacer nada
    if (isnan(segundosEncendido) || segundosEncendido <= 0) {
        // Aquí puedes mostrar "Lectura inválida" o saltar el cálculo
        segundosEncendido=0;
    }

      if (isnan(bateriaConsumida_mAh) || bateriaConsumida_mAh <= 0) {
        // Aquí puedes mostrar "Lectura inválida" o saltar el cálculo
        bateriaConsumida_mAh=0;
    }

  bateriaRestante_mAh = capacidadBateria_mAh - bateriaConsumida_mAh;

  Serial.println("Valores restaurados de EEPROM:");
  Serial.print("Tiempo encendido (seg): "); Serial.println(segundosEncendido);
  Serial.print("Bateria consumida (mAh): "); Serial.println(bateriaConsumida_mAh);
  Serial.print("Bateria restante (mAh): "); Serial.println(bateriaRestante_mAh);

  tiempoAnterior = millis();
////////////////////

display.init();                     //Iniciamos display OLED

 if (!ina219.begin()) {
    Serial.println("No se encontró el sensor INA219. Verifica conexiones.");
    while (1) { delay(10); }
  }
  Serial.println("INA219 iniciado correctamente.");




////////////////////////////////

  pinMode(vbatPin, INPUT);  //Definimos el pin vbatPin (Pin 35) como entrada
  pinMode(PRG_boton, INPUT_PULLUP); //Activamos la PULLUP del boton PRG_boton de la tarjeta, el integrado.
  
  pinMode(mosfet, OUTPUT);
  pinMode(16, INPUT_PULLUP); //Activamos la PULLUP Sensor DS18B20 OneWire

  pinMode(ledverde,OUTPUT);  //Pin 5 como salida | LED VERDE DE FABRICA
  pinMode(2,OUTPUT);  //Pin 2 como salida | LED ROJO LORA TTGO BOARD
  pinMode(0,INPUT);   //Boton Push de TTGO LORA BOARD
  pinMode(37,INPUT);  //PIN GPIO Digital sin funciones adicionales | Para evaluarl sensor capacitivo

  //pinMode(16,OUTPUT); //RST OLED
  //pinMode(12,OUTPUT); //PRUEBA PWM BUZZER
  digitalWrite(ledverde,HIGH); //APAGAMOS LED VERDE DE FABRICA

  // Configurar el pin como salida
  pinMode(Buzz_PWM, OUTPUT);

  //Inicializo sensor de temperatura
  sensors.begin();

  Serial.print("Sensores encontrados: ");
  Serial.println(sensors.getDeviceCount());
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0);
  tempe_deseo = (int)tempC;



////////////////////////////////////////////
 // Configura los límites del PID (PWM)
  myPID.SetOutputLimits(0, 255);  // 0-255 para analogWrite
  myPID.SetMode(AUTOMATIC);       // Modo automático
//////////////////////////////////////

  // Configurar el canal PWM - usar el canal 0 en este caso
  ledcSetup(0, 2500, 8);  // Canal 0, frecuencia de 5000 Hz, resolución de 8 bits
  ledcAttachPin(Buzz_PWM, 0);   // Asociar el canal PWM con el pin específico
//////////////////////////////

// Configura canal 1, frecuencia 5000Hz, 8 bits de resolución
  ledcAttachPin(mosfet,1);    // pin, canal
  ledcSetup(1, 5000, 8);           // canal 1, freq, resolución

  /*
  digitalWrite(16, LOW);    // Pin GPIO16 en bajo o cero => Reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // Activamos GPIO16 para utilizar OLED
  */
 
  
  //Iniciamos configuracion SPI
  SPI.begin(SCK,MISO,MOSI,SS);

   //Sonido de inicialiacion en Buzzer | Beep de primer paso superado: LORA INICIADO..
  ledcWrite(0, 0);
  delay(33);
  ledcWrite(0,255);
 
  display.init();                     //Iniciamos display OLED
  //display.flipScreenVertically();     //Configuramos posición vertical
  display.setFont(ArialMT_Plain_10);  //Elegimos fuente y tamaño para texto
  
  logo();                             //Proyectamos pantallas de inicio, mediante función logo();
  Pantalla_A();

   //Sonido de inicialiacion en Buzzer |SETUP FINALIZADO| 2 beeps
  ledcWrite(0, 0);
  delay(33);
  ledcWrite(0,255);
  delay(33);
  ledcWrite(0, 0);
  delay(33);
  ledcWrite(0,255);

}




int presion_larga=0;

//FUNCION PRINCIPAL | BUCLE INFINITO | LOOP
void loop() {
Serial.flush();
delay(1);       //Retardo de 1ms para aumentar los contadores a cada paso por loop | Estimado de 1ms cada vez.
minitimer++;


//Si ha transcurrido 1 segundo aproximadamente revisamos sensores y demas...
if (minitimer % 1000==0){
//lecturaINA219();


  //Leemos temperatura 
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0);
  Serial.print("Temperatura: ");
  Serial.print(tempC);
  Serial.println(" °C");

//Funcion para desplegar termometro de tempC Sensor
//Ejemplo, Posicion de Inixio X -  Inicio Y - Largo de Barra    - Grosor Barra  - Temp    - Limite Superior Temp   - Limite Inferior Temp
//              65              40              63              3             temp        15            50
  termometro(65,   15,   63,   3,    tempC,    15,   50);


//FUNCION DE CALENTAMIENTO
//ACTIVADOR DE PWM MEDIANTE PID
  myPID.Compute();    // Calcula el PID
  ledcWrite(1, (int)output);   // canal , valor PWM

  // Imprime para depuración
  Serial.print("Temp: "); Serial.print(tempC);
  Serial.print(" | PWM: "); Serial.println(output);
 // minitimer=0; //Reiniciamos minitimer

//Si la temperatura (y la salida PID PWM es mayor que cero)
//Significa que se está calentando la chamarra => Activamos animacion o señal de encendido para display.
if (output>0) anima_calienta();
}




////////////////////////////////////////
///////////////////////SECCION LOOP : PRESION DE BOTON : FUNCION
////////////////////////////////////
if (!digitalRead(PRG_boton))  {     //Si el botón de la tarjeta es presionado
  delay(50);                        //Esperamos 50ms para antirebotes
  while (!digitalRead(PRG_boton)){  //Y esperamos a que el botón deje de presionarse
  delay(1);
  presion_larga++;                //Aumentamos el contador de "tiempo" para detectar presion larga
  if (presion_larga >= 2000) lecturaINA219();  //Si hay presion larga leemos datos de corriente para mostrar a usuario.
  }  

if (presion_larga<2000){    //Si la presion no fue suficientemente larga, entonces fue una pulsacion normal
    tempe_deseo++;          //Solo aumentamos la temperatura deseada

   }
 
  //Leemos temperatura 
  Serial.print("Temperatura Deseada: ");
  Serial.print(tempe_deseo);
  Serial.println(" °C");
  termometro(65,40,63,3,tempe_deseo,15,50);

if (tempe_deseo > tempC){   //Activamos Buzzer indicando que se esta aumentando la temperatura deseada y se calentará.
    avisobuzzer();
  }

  }

  presion_larga=0;  //Terminamos revision y reiniciamos contador de presion larga

if (minitimer>=15000) {
  lecturaINA219();
  termometro(65,40,63,3,tempe_deseo,15,50);  
  minitimer=0;
}
}



void anima_calienta(){
    display.setColor(WHITE);
    display.fillCircle(27, 30, 2);
    display.fillCircle(39, 30, 2);
    display.display();
    delay(150);
    display.setColor(BLACK);
    display.fillCircle(27, 30, 2);
    display.fillCircle(39, 30, 2);
    display.display();
    
}

///////////////////////////
void avisobuzzer(){
     //Sonido de inicialiacion en Buzzer |SETUP FINALIZADO| 2 beeps
  ledcWrite(0, 0);
  delay(33);
  ledcWrite(0,255);
  delay(33);
  ledcWrite(0, 0);
  delay(33);
  ledcWrite(0,255);
}
/////////////////////////




//////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////



void lecturaINA219(){
/////////////////////LECTURA INA219
  float corriente_mA = ina219.getCurrent_mA(); // Corriente en miliamperes
  float voltaje = ina219.getBusVoltage_V();    // Voltaje en volts
  float potencia = ina219.getPower_mW();       // Potencia en miliwatts

  Serial.println("::::::::MEDICION ELECTRICA CALEFACCION::::::::");
  Serial.print("Corriente: "); Serial.print(corriente_mA); Serial.println(" mA");
  Serial.print("Voltaje: "); Serial.print(voltaje); Serial.println(" V");
  Serial.print("Potencia: "); Serial.print(potencia); Serial.println(" mW");
  Serial.println("-----------------------------------------------");

    //Limpieza de area de texto temperatura
    display.clear();
    display.setColor(BLACK);
    display.fillRect(0, 15, 60, 30); // Xinicial, Yinicial, Largo, Grosor.
    display.setColor(WHITE);
    display.setFont(ArialMT_Plain_10); 
    display.drawString(0, 0, String(voltaje) + " V");
    display.drawString(45, 0, String(corriente_mA) + " mA");
    display.drawString(0, 10, String(potencia) + " mW");
    display.display();

    bateria_rev (corriente_mA);

    delay(2500);
    display.clear();
    Pantalla_A();


}


////////////////////
///////////////////////////

//Funcion que me ayuda a visualizar un "termometro" (lectura de °C) en la posicion que necesito
//Generando automaticamente la barra de medicion dinamica (solo funcion para visualizar en display)
//Ejemplo, Posicion de Inixio X -  Inicio Y - Largo de Barra    - Grosor Barra  - Temp    - Limite Superior Temp   - Limite Inferior Temp
//              65              40              63              3             temp        15            50
void termometro (int xi_term, int yi_term, int tamano_bar, int grosor_term, float temp, int liminf, int limsup){

    char string[15]; 
    itoa(temp,string,10);
    sprintf(string,"%4.2f °C",temp);
  //Datos de posicion inicial, grosor y largo de barra esperado.
    float bar_val; // valor para barra px

  //   if (temp > 50) tempe_deseo = liminf;
  //   if (tempe_deseo<liminf) tempe_deseo = liminf;

    //Limpieza de area de texto temperatura
    display.setColor(BLACK);
    display.fillRect(xi_term, yi_term, tamano_bar, 18); // Xinicial, Yinicial, Largo, Grosor.
    
    //Reescritura de datos temperatura XX °C
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);  //Elegimos fuente y tamaño para text
    display.setColor(WHITE);
    display.drawString(xi_term, yi_term, string);
    display.display();

    //Barra de carga de acuerdo a temperatura, y espcio definido
    //Limpieza de barra termometro
    display.setColor(BLACK);
    display.fillRect( xi_term, yi_term+19, tamano_bar ,grosor_term); 

    bar_val = (temp-liminf) / (limsup - liminf);
    bar_val = bar_val * tamano_bar;

    Serial.print("VALOR DE H:");
    Serial.println(bar_val);

    //Ciclo animacion Termometro Barra
    for (int h = 0;  h < bar_val; h++ ){
    delay (10);
    display.setColor(WHITE);
    display.fillRect(xi_term, yi_term+19, h, grosor_term);
    display.display();
    }

}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

