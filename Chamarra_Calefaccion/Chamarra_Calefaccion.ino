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

const int Buzz_PWM = 12;
const int mosfet = 4;
const int ledverde = 5;

//:::::::::::::::::::::::BATTERY CHECK
const uint8_t vbatPin = 35; //Pin para leer ADC de bateria, pin 35.




//Funcion de muestreo de logo de inicio
void logo(){
  display.clear();
  //digitalWrite(12,HIGH);
  display.drawXbm(0,0,128,64,jofearlogo); //Proyectamos logo/pantalla de inicio.
  display.display();
  delay(3000);
  //digitalWrite(12,LOW);
  }

void Pantalla_A(void){
  display.clear();                     //Limipiamos display
  //display.drawXbm(0,0,128,64,SISIBG); //Proyectamos fondo/pantalla de trabajo

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "ACERU");

  //display.drawRect(42,20,44,15);          //Dibujamos rectangulo para dato recibido
  display.display();
}

//////////////FUNCION PARA LEER ADC de batería y revisar carga

void bateria_rev() {

  char string[25]; //Cadena que utilizamos para convertir y mostrar los valores de voltaje o porcentaje de bateria
  float Vbat = (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;    //Almacenamos la lectura del ADC en variable flotante Vbat
  //float Vbat = (float)(analogRead(vbatPin)) / 4095*3.3; 

  /*
  El ADC en este caso es de 12-bit de resolucion, por lo que alcanza un conteo de hasta 4095 (desde cero)
  Para convertirlo a un valor entero de voltaje leído, primero calculamos que porcentaje o parte del total (4095) es leído.
  Ejemplo, si leemos un valor del ADC de 1000, entonces 1000/4095 nos dirá que tenemos un 0.24 o 24% de la lectura maxima posible.
  Pero estamos utilizando un divisor de voltaje (mitad) por lo que el valor debemos multiplicarlo por 2.
  En seguida, se multiplica por 3.3V ya que es el voltaje de alimentación de ESP32 con el que está trabajando
  Y por ultimo se multiplica por 1.1V que es el voltaje de referencia al que trabaja el ADC.
  Así podremos obtener una lectura de voltaje correcta.
  */

 //Mostramos en la consola serial el valor leido en voltaje.
  //Serial.println();
 // Serial.println();
  //Serial.print("Voltaje Vbat = "); Serial.print(Vbat); Serial.println(" Volts");

    // Encontré ésta pagina para generar códigos de algunas fuentes y símbolos http://oleddisplay.squix.ch/
    // Ejemplo simbolos climaticos con fuente Meteocons
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
   
   //Rutina para mostrar en OLED los datos de voltaje. La desactivo por el momento, pero podría necesitarla despues para visualizar
   //datos técnicos en una segunda Slide en la pantalla.
  
  /*
    display.drawString(0, 8, "Voltaje: ");
    itoa(Vbat,string,10);
    sprintf(string,"%4.2f volts",Vbat);
    display.setColor(BLACK);
    display.fillRect(35,8,35,10);
    display.setColor(WHITE);
    display.drawString(40, 8, string);
    */

    bool carga=0;   // Variable para conocer si el dispositivo se encuentra cargando o usando batería
                    //carga 0 = bateria    || carga 1 = cargando USB

  
    float per_charge = ((Vbat-3)*100)/1.2; //Restamos 3 del valor leido en volts, ya que en realidad el rango efectivo de batería sería de 3V a 4.2V
                                          //Luego calculamos que porcentaje de energía tenemos respecto ese rango (4.2-3 = 1.2V) para tener una medicion util
                                          //Multiplicamos por 100 para tener el valor en porcentaje.
    itoa(per_charge,string,10);           //Convertimos el valor auna cadena de caracteres String mediante base 10.

        
    if (per_charge <=100){
        carga=false;                              //Si recibimos voltaje menor o igual a 100% solo está el voltaje de la batería 4.2V o menos => Batería
        sprintf(string,"%2.0f%%",per_charge);     //"Imprimimos" el valor en la cadena string, como valor flotante de dos enteros.
        display.setColor(BLACK);
        display.fillRect(84,0,52,10);
        display.setColor(WHITE);
        display.drawString(102, 0, string);    //Antes de imprimir el valor actual de batería.
        //Serial.print("Batería: "); Serial.println(string);
    }
    
    
    else if (per_charge > 100){      //Si el porcentaje de carga es superior a 100% significa que recibimos mucho mas voltaje (5V) => Cargando USB
       carga=true;              //Activamos variable carga como verdadero.
       per_charge=100;          //Limitamos el valor a 100% para no mostrar valores superiores en porcentaje.

                        
       display.setColor(BLACK);
        display.fillRect(84,0,52,10);
        display.setColor(WHITE);
        display.drawString(102, 0, "USB");    // //Si el porcentaje de carga es mayor que 100% corregimos el valor de la cadena para mostrar "USB".
        Serial.println("Batería: Cargando USB");
    }


  //Creamos dos variables para definir rapidamente la posición del icono de la batería
int batx = 84;
int baty = 3;

//Primer opcion de ciclo => Si no hay carga USB, entonces mostramos nivel de carga en icono de batería
  if (!carga){
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
}


// Segunda opcion de ciclo de icono de bateria => Animación de batería cargandose
// Se muestra solo si el estado de "carga" es verdadero.

if (carga){
  display.drawFastImage(batx,baty,16,16, bateria_20); 
  display.display();
}

/*
if (carga){
//Animación carga de bateria
  display.drawFastImage(batx,baty,16,16, bateria_0);
  display.display();

  display.drawFastImage(batx,baty,16,16, bateria_20); 
  delay(100);
  display.display();

  display.drawFastImage(batx,baty,16,16, bateria_40);
  delay(100);
  display.display();

  display.drawFastImage(batx,baty,16,16, bateria_60);
  delay(100);
  display.display();

  display.drawFastImage(batx,baty,16,16, bateria_80);
  delay(100);
  display.display();

  display.drawFastImage(batx,baty,16,16, bateria_100);
  delay(100);
  display.display();
    }
    */
}     //Fin de función Bateria Rev

////////////////////////////////////////////////////////////////
///////////////CODIGOS PARA PROBAR SLEEP MODES Y WAKEUPS

//VARIABLES PARA TIMER WAKE UP SLEEP MODE
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}



//Funcion de configuraciones iniciales
void setup() {
///////////////////SLEEP MODE
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial Inicializado... > OK");


 if (!ina219.begin()) {
    Serial.println("No se encontró el sensor INA219. Verifica conexiones.");
    while (1) { delay(10); }
  }
  Serial.println("INA219 iniciado correctamente.");


  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  Configuramos el SLEEP para funcionar cada "TIME TO SLEEP" segundos, en caso de que ocupemos sleep
  por TIMER. Pero en este programa la prioridad es WAKEUP por mensaje LORA recibido.
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

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

  // Configurar el canal PWM - usar el canal 0 en este caso
  ledcSetup(0, 2500, 8);  // Canal 0, frecuencia de 5000 Hz, resolución de 8 bits
  // Asociar el canal PWM con el pin específico
  ledcAttachPin(Buzz_PWM, 0);
//////////////////////////////

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


int buzzitimer=0;
int vibra_time=0;
float tempe_deseo=15.0;
bool calentando=0;
float tempC;
//bool estado_vibrador=0;



//FUNCION PRINCIPAL | BUCLE INFINITO | LOOP
void loop() {
//Serial.flush();
delay(1);       //Retardo de 1ms para aumentar los contadores a cada paso por loop | Estimado de 1ms cada vez.
minitimer++;

if (minitimer>=1000){


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
    display.setColor(BLACK);
    display.fillRect(0, 15, 60, 30); // Xinicial, Yinicial, Largo, Grosor.
    display.setColor(WHITE);
    display.setFont(ArialMT_Plain_10); 
    display.drawString(0, 15, String(corriente_mA) + " mA");
    display.drawString(0, 25, String(voltaje) + " V");
    display.drawString(0, 35, String(potencia) + " mW");
    display.display();


  //Leemos temperatura 
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0);
  Serial.print("Temperatura: ");
  Serial.print(tempC);
  Serial.println(" °C");

//Ejemplo, Posicion de Inixio X -  Inicio Y - Largo de Barra    - Grosor Barra  - Temp    - Limite Superior Temp   - Limite Inferior Temp
//              65              40              63              3             temp        15            50
  termometro(65,   15,   63,   3,    tempC,    15,   50);

  minitimer=0;
}



if (!digitalRead(PRG_boton))  {     //Si el botón de la tarjeta es presionado
  delay(50);                        //Esperamos 50ms
  while (!digitalRead(PRG_boton));  //Y esperamos a que el botón deje de presionarse

  tempe_deseo++;
  if (tempe_deseo > tempC){
    //display.drawString(20, 30, "+++");
    display.display();
    digitalWrite(ledverde,LOW);
    digitalWrite(mosfet,HIGH);
    avisobuzzer();
  }


    //Leemos temperatura 
  Serial.print("Temperatura Deseada: ");
  Serial.print(tempe_deseo);
  Serial.println(" °C");
  termometro(65,40,63,3,tempe_deseo,15,50);
}


if (minitimer==500)bateria_rev(); //Revisamos batería         
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

