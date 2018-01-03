// ---------------------------------------------------------------------------
// Creado por Juan Benavent Andres. 1/8/15.
// Copyright 2011 - Under creative commons license:
// ---------------------------------------------------------------------------
#include <Wire.h>
#include <EEPROM.h>
#include <I2CIO.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

/**
     Prototipos de funciones
**/
void PintaTexto (String texto);
void ConfigValor (byte parametro);
void DisplayModo (byte muestraModo);
void RecalculaAMP (unsigned int);
void RecalculaBAT (void);
void RecalculaFREC1 (void);
void RecalculaFREC2 (void);
void RecalculaTEMP (void);
void SetRegulador (boolean modoReg);
void GuardaValores (void);
void RecuperaValores (void);
void RegulaCorriente (void);
void Calibra (void);
void ConfiguraPrescalers(void);

#define TEMP_CAL_OFFSET   334    // Offset de calibracion de temperatura

#define BOTON_NP          0
#define BOTON_MODO        4
#define BOTON_IZQ         5
#define BOTON_DER         6
#define BOTON_OK          7

#define AMP_PIN           A3       // Pin para la lectura de corriente
#define TEMP_PIN          A2       // Pin para la lectura de temperatura
#define BAT_PIN           A0       // Pin para la lectura de bateria

#define PWM1_PIN          9       // Salida de PWM1
#define PWM2_PIN          3       // Salida de PWM2

#define LED_NANO          13      // LED en placa NANO

// Define los diferentes modos en los que se encuentra el programa
#define VER_VALORES     1         // Modo inicial en el que se muestra la corriente actual medida
#define VER_CONFIG      2
#define VER_FRECS       3
#define CONFIG_MODO     4
#define SEL_MODO        5
#define CONFIG          8
#define SEL_AMP         9
#define SEL_FREC1       10
#define SEL_FREC2       11
#define SEL_CALIBRA     12
#define CALIBRA_OK      13
#define CONFIG_AMP      14
#define CONFIG_FREC1    15
#define CONFIG_FREC2    16
#define SEL_UMBRAL      17
#define CONFIG_UMBRAL   18

#define SET             1              // Boton MODO pulsado
#define DER             2              // Boton DER pulsado
#define IZQ             3              // Boton IZQ pulsado
#define OK              4              // Boton OK pulsado
#define FASTUP          5              // Boton DER y SET pulsado. Incremento rapido
#define FASTDOWN        6              // Boton IZQ y SET pulsado. Decremento rapido

#define REG_OFF         1              // Modo de regulacion parado
#define REG_AUTO        2              // Modo de regulacion automatico
#define REG_MAN         3              // Modo de regulacion manual
                           

LiquidCrystal_I2C lcd(0x20, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);      // lcd_Addr, En, Rw, Rs, d4, d5, d6, d7, backlighPin, pol
byte boton = BOTON_NP;               // Guarda el boton pulsado
byte botonPrevio;                    // Guarda el ultimo boton pulsado
byte modo;                           // Modo/pantalla actual
unsigned int corrienteSel;           // Valor de la corriente seleccionada * 100
unsigned int corrienteSelAnt = 0;    // Valor de la corriente seleccionada anterior * 100. si es distinta a la actual es porque ha habido un cambio
unsigned int corrienteSens;          // Valor de la corriente medida * 100
unsigned int corrienteAnt = 0;       // Valor de la corriente medida el minuto anterior * 100
unsigned int tensionUmbral;          // Valor de la tension a la cual se activara la regulacion * 100
unsigned int limImax;                // limite maximo de corriente
unsigned long sumatorioAMP = 0;      // Valor acumulado de la corriente en voltios del ADC. Sirve para calcular la media de la corriente en 100 ciclos

unsigned int prescaler1;
unsigned int prescaler2;
byte pwmSel;                         // Valor seleccionado de PWM
byte resolucionMA;                   // Valor de corriente equivalente a un bit del pwm. Multiplicada por 100 para trabajar con mA enteros
unsigned long frec1, frec2;          // Frecuencia Principal y secundaria
double volts;                        // valor de los voltios de la bateria * 100

String strValorAMP;                  // String para visualizar el valor de corriente
String strValorFREC1, strValorFREC2; // Strings para visualizar el valor de frecuencia base y secundaria
String strValorTEMP;                 // String para visualizar el valor de la temperatura
String strValorBAT;                  // String para visualizar el valor de la tension
String strValorUMBRAL;               // String para visualizar el valor umbral de la tension de arranque
String strValorPWM;                  // String para visualizar el valor del ratio de PWM
String strToPrint;                   // String que recoje el texto a imprimir en ese momento

unsigned int i, j, pulsacion, loops=1; // iteradores
String lineaA, lineaB;               // Se utilizan para recordar la linea escrita actual y saber si hay que repintarla
byte modoRegulador;                  // Estado actual OFF / MAN/ AUTO
byte modoReguladorPrevio;            // Sirve para indicar si se ha cambiado el modo del regulador
boolean motorOn;                     // Indica si el motor esta en marcha por tension de bateria
//boolean estadoLedNano = LOW;       // Marca el estado del Led de la placa Nano 
boolean calibrando = false;          // Sirve para que, mientras se calibra, PWM2_PIN se mantenga al maximo

void setup ()
{
  pinMode (BOTON_MODO, INPUT_PULLUP);      // inicializa el boton de Modo como entrada
  pinMode (BOTON_DER, INPUT_PULLUP);       // inicializa el boton de Incremento como entrada
  pinMode (BOTON_IZQ, INPUT_PULLUP);       // inicializa el boton de Decremento como entrada
  pinMode (BOTON_OK, INPUT_PULLUP);        // inicializa el boton OK como entrada
  pinMode (AMP_PIN, INPUT);                // inicializa como entrada analogica
  pinMode (TEMP_PIN, INPUT);               // inicializa como entrada analogica
  pinMode (PWM1_PIN, OUTPUT);
  pinMode (PWM2_PIN, OUTPUT);
  pinMode (LED_NANO, OUTPUT);
   
 //  Serial.begin ( 57600 );
//   analogReference ( INTERNAL );
   
  lcd.begin (16, 2);
  
  RecuperaValores();            // Obtiene valores de frecuencia y modo de la EEPROM
   
 // if (OCR2A == 0x00)          // Si no se ha inicializado nunca y los valores de la EEPROM estan fuera de rango. Inicializamos valores
  if (modoRegulador == 0xff)
  {
    corrienteSel = 500;               // Por defecto 5 A
    tensionUmbral = 300;//1350;       // Por defecto Tension a la cual se activara la regulacion     
    OCR2A = 250;                      // Se corresponde a una frecuencia de 8000Hz
    prescaler1 = 128;//256            // Por defecto prescaler para frec1
    prescaler2 = 8;                   // Por defecto prescaler para frec2 (8000Hz)
    pwmSel = 50;                      // Por defecto 50 %     
    limImax = 2550;                   // Por defecto limite maximo de corriente 25,5 A
    modoRegulador = REG_MAN;//REG_OFF;    // Por defecto, regulador apagado
  }
  
  TCCR1A |= _BV(WGM10) | _BV(COM1A1);  // Fast PWM Mode. Como WGM12 = 1, el tope maximo es 0xFF (modo 8 bits). set non-inverting mode for OC1A
  TCCR2B |= _BV(WGM12);                // Tope maximo es 0xFF (modo 8 bits)
  
  TCCR2A |= _BV(WGM21) | _BV(WGM20) | _BV(COM2B1);   // Fast PWM Mode. Set non-inverting mode for OC2B
  TCCR2B |= _BV(WGM22);                // Tope maximo es OCR2A    
 
  modoReguladorPrevio = 0xff;          // Fijo un modo previo de regulacion inexistente para que en loop se detecte como un cambio
  modo = VER_VALORES;                  // Modo al iniciar en el que se muestra la corriente   
  loops = 0;

  RecalculaFREC1 ();  
  RecalculaFREC2 ();
  strValorPWM = String (pwmSel) + "%";
  RecalculaTEMP ();
  RecalculaAMP (1);
  
  OCR1A = (pwmSel * 0xff) / 100;
  OCR2B = OCR2A / 2;
}

void loop ()
{
  botonPrevio = boton;                      // Guardo el estado anterior del boton para ver si ha cambiado
      
  RecalculaBAT ();  
  
  if (volts > tensionUmbral)                        // Comprueba que el motor sigue en marcha
  {  
    motorOn = true;
    if (modoReguladorPrevio != modoRegulador)       // Si se ha cambiado el modo del regulador, actualiza parametros
    {
      SetRegulador(modoRegulador);
      modoReguladorPrevio = modoRegulador;
    }
  }
  else
  {
    motorOn = false;
    SetRegulador(REG_OFF);
    modoReguladorPrevio = 0xff;            // Lo pongo por defecto para que cuando haya tension suficiente actue como reset
  }
    
  if (!digitalRead(BOTON_MODO))                         // Garantiza que el boton ha cambiado respecto al previo y, ademas que la actual pulsacion el '1'
    boton = SET;
  else if (!digitalRead(BOTON_DER))
    boton = DER;
  else if (!digitalRead(BOTON_IZQ))
    boton = IZQ;
  else if (!digitalRead(BOTON_OK))
    boton = OK;
  else
    boton = BOTON_NP; 

  if ((boton != BOTON_NP) && (botonPrevio == BOTON_NP))
  {  
    /*                     */
    /*   MODOS INICIALES   */
    /*                     */  
    switch (modo)
    {
      case VER_VALORES:  switch (boton)
                         {
                           case IZQ:   if(modoRegulador != REG_OFF)                  // Si el regulador esta OFF, no muestra esta pantalla
                                         modo = VER_CONFIG;
                                       break;
                           case SET:   modo = CONFIG; break;
                           case DER:   if(modoRegulador != REG_OFF)                  // Si el regulador esta OFF, no muestra esta pantalla
                                         modo = VER_FRECS;
                                       break;
                         }
                         break;
                         
      case VER_FRECS:    switch (boton)
                         {
                           case IZQ:   modo = VER_VALORES; break;
                           case SET:   modo = CONFIG; break;
                           case DER:   modo = VER_CONFIG; break;
                         }
                         break;
  
      case VER_CONFIG:   switch (boton)
                         {
                           case IZQ:   modo = VER_FRECS; break;
                           case SET:   modo = CONFIG; break;
                           case DER:   modo = VER_VALORES; break;
                         }
                         break;
      /*                                               */
      /*   MODOS SELECCION DE PARAMETRO A CONFIGURAR   */
      /*                                               */
      case CONFIG:      switch (boton)
                        {
                          case OK:   if (modoRegulador == REG_OFF)
                                       modo = SEL_MODO;
                                     else
                                       modo = SEL_AMP;
                                     break;                    
                          case SET:  modo = VER_VALORES; break;
                        }
                        break;    
      
      case SEL_AMP:     switch (boton)
                        {
                          case IZQ:  modo = SEL_UMBRAL; break;
                          case SET:  modo = VER_VALORES; break;
                          case OK:   modo = CONFIG_AMP; break; 
                          case DER:  modo = SEL_FREC1; break; 
                        }
                        break;  
                        
      case SEL_FREC1:   switch (boton)
                        {
                          case IZQ: modo = SEL_AMP; break;
                          case SET: modo = VER_VALORES; break; 
                          case OK:  modo = CONFIG_FREC1; break;  
                          case DER: modo = SEL_FREC2; break;      
                        }
                        break;
      
      case SEL_FREC2:   switch (boton)
                        {
                          case IZQ: modo = SEL_FREC1; break; 
                          case SET: modo = VER_VALORES; break;
                          case OK:  modo = CONFIG_FREC2; break;
                          case DER: modo = SEL_MODO; break;        
                        }
                        break;                    
      
      case SEL_MODO:    switch (boton)
                        {
                          case IZQ:  if (modoRegulador == REG_OFF)
                                       modo = SEL_UMBRAL;
                                     else
                                       modo = SEL_FREC2;
                                     break;
                          case SET:  if (modoRegulador == REG_OFF)
                                       modoRegulador = REG_MAN;
                                     else
                                       modoRegulador = REG_OFF;
                                     modo = CONFIG_MODO;
                                     break;
                          case OK:   if (modoRegulador == REG_AUTO)
                                       modoRegulador = REG_MAN;
                                     else
                                       modoRegulador = REG_AUTO;
                                     modo = CONFIG_MODO;
                                     break;
                          case DER:  modo = SEL_CALIBRA; break;
                        }
                        break;      
                        
      case SEL_CALIBRA: switch (boton)
                        {
                          case IZQ: modo = SEL_MODO; break;
                          case SET: modo = VER_VALORES; break;
                          case OK:  modo = CALIBRA_OK; break;
                          case DER: modo = SEL_UMBRAL; break;
                        }
                        break;
      
      case SEL_UMBRAL:  switch (boton)
                        {
                          case IZQ: modo = SEL_CALIBRA; break;
                          case SET: modo = VER_VALORES; break;
                          case OK:  modo = CONFIG_UMBRAL; break;
                          case DER: if (modoRegulador == REG_OFF)
                                      modo = SEL_MODO;
                                    else
                                      modo = SEL_AMP;
                                    break;               
                        }
                        break;
                        
      case CALIBRA_OK:  if (boton == OK)        
                          Calibra(); 
                        break; 
      default: break;
    }
  }
  
  sumatorioAMP += analogRead (AMP_PIN);                    // Mido la corriente en cada ciclo y cada 100 luego calculo la media
    
  /** 
      Recalcula valores y los muestra
  **/    
  if(loops++ == 10)
  {
//    digitalWrite(LED_NANO, estadoLedNano = ~estadoLedNano);          // Invierte salida del Led L de la placa Nano
       
    RecalculaTEMP ();
    RecalculaAMP (loops);                                             // Recalcula la corriente a mostrar (media de los ultimos 100 ciclos)  
    
    //if (modoRegulador == REG_AUTO)
      //RegulaCorriente();                                              // Regula corriente cada cierto tiempo
        
    loops = 0; 
  }
    
  DisplayModo (modo);
    
  delay (100);
} // fin de loop ()

/**
    DisplayModo
**/
void DisplayModo(byte muestraModo)
{
  byte blankLinea;      // Caracteres " " necesarios entre valores
  byte i;
  String punto;
  int decimales;
  
  switch (muestraModo)
  {
      case VER_VALORES: if (motorOn)
                        {
                          if (modoRegulador != REG_OFF)
                          {
                            strToPrint = strValorAMP;                         // Valor de corriente    
        
                            blankLinea = 16 - (strValorAMP.length() + strValorPWM.length());
                            
                            for (i = 0; i != blankLinea; i++)
                              strToPrint += " ";                           // Añade los ceros necesarios en funcion de la longitud del valor      
                               
                            strToPrint += strValorPWM;                        // Valor de temperatura y de tension seguidos
                          }
                          else
                            strToPrint = "----------------";
                               
                          strToPrint += strValorBAT;                        // Valor de temperatura y de tension seguidos 
                          
                          blankLinea = 16 - (strValorBAT.length() + strValorTEMP.length());
                          
                          for (i = 0; i != blankLinea; i++)
                            strToPrint += " ";                           // Añade los ceros necesarios en funcion de la longitud del valor      
                               
                          strToPrint += strValorTEMP;
                        }
                        else
                          strToPrint = "Espera Arranque " + strValorBAT + "     ";
    
                        PintaTexto (strToPrint);                        
                        break;    
    
    case VER_FRECS:     strToPrint = strValorFREC1 + strValorFREC2;                         // Valores de frecuencia
                        PintaTexto (strToPrint);
                        break;
                        
    case VER_CONFIG:    decimales = corrienteSel % 100;
    
                        if(decimales < 10)
                          punto = ".0";
                        else
                          punto = ".";
    
                        if (modoRegulador == REG_MAN)
                          strToPrint = "R:" + strValorPWM;
                        else
                          strToPrint = "Is:" + String (corrienteSel / 100) +  punto + String (decimales/10) + "A";
                                
                        blankLinea = 9 - strToPrint.length();
                        
                        for (i = 0; i != blankLinea; i++)
                          strToPrint += " ";
                          
                        switch (modoRegulador)
                        {
                          case REG_OFF: strToPrint += "    OFF"; break;
                          case REG_MAN: strToPrint += "    MAN"; break;
                          case REG_AUTO:strToPrint += "   AUTO"; break;
                        }
                          
                        decimales = limImax % 100;
    
                        if(decimales < 10)
                          punto = ".0";
                        else
                          punto = ".";                          
 
                        strToPrint += "Im:" + String (limImax / 100) +  punto + String (decimales/10) + "A";                        
                        blankLinea = 25 - strToPrint.length();
                        
                        for (i = 0; i != blankLinea; i++)
                          strToPrint += " ";
                          
                        strToPrint += "U:" + String (tensionUmbral / 100) +  "." + String ((tensionUmbral % 100)/10) + "V"; 
           
                        PintaTexto (strToPrint);
                        break;
 
    case CONFIG:        PintaTexto ("   Configurar   ""   [NO]  <SI>   "); break;
    case SEL_AMP:       PintaTexto (" Set Corriente  ""<| [NO]  <SI> |>"); break;
    case CONFIG_AMP:    
    case CONFIG_FREC1:  
    case CONFIG_FREC2:  
    case CONFIG_UMBRAL: lcd.clear();
                        ConfigValor (muestraModo); 
                        break;    
    case SEL_MODO:      switch (modoRegulador)
                        {
                          case REG_OFF:  PintaTexto ("Regulacion   OFF""<| [MAN]<AUTO>|>"); break;
                          case REG_MAN:  PintaTexto ("Regulacion   MAN""<| [OFF]<AUTO>|>"); break;
                          case REG_AUTO: PintaTexto ("Regulacion  AUTO""<| [OFF] <MAN>|>"); break;
                        }
                        break;
                        
    case CONFIG_MODO:   if (modoReguladorPrevio != modoRegulador)      // Si ha cambiado el modo de regulacion
                        {
                          switch (modoRegulador)
                          {
                            case REG_OFF:  modoReguladorPrevio = REG_OFF;
                                           PintaTexto ("Regulacion      "" #  INACTIVA  # ");
                                           break;
                            case REG_MAN:  modoReguladorPrevio = REG_MAN;
                                           PintaTexto ("Regulacion      "" #   MANUAL   # ");
                                           break;
                            case REG_AUTO: modoReguladorPrevio = REG_AUTO;
                                           PintaTexto ("Regulacion      "" # AUTOMATICA # ");
                                           break;                          
                          }
                          SetRegulador(modoRegulador);                // Se ha cambiado el modo del regulador
                          
                          delay(1000);
                          modo = VER_VALORES;
                        }
                        break;
    
    case SEL_FREC1:     PintaTexto (" Set Frec Base  ""<| [NO]  <SI> |>"); break;
    case SEL_FREC2:     PintaTexto ("  Set Frec Sec  ""<| [NO]  <SI> |>"); break;
    case SEL_CALIBRA:   PintaTexto ("    Calibrar    ""<| [NO]  <SI> |>"); break;
    case CALIBRA_OK:    PintaTexto ("    Confirma    ""   [NO]  <SI>   "); break;
    case SEL_UMBRAL:    PintaTexto ("Ajustar  'START'""<| [NO]  <SI> |>"); break;
  }
}

/**
  ConfigValor
**/
void ConfigValor (byte Parametro)
{
  int pulsacion = 0;
  byte blankLinea;
  
  delay (500);  
  botonPrevio = OK;
  
  do
  {
    boton = BOTON_NP;    
    pulsacion++;
  
    if (!digitalRead(BOTON_DER))
    {
      if (!digitalRead(BOTON_MODO))
        boton = FASTUP;
      else
        boton = DER;
    }
    else if (!digitalRead(BOTON_IZQ))
    {
      if (!digitalRead(BOTON_MODO))
        boton = FASTDOWN;
      else
        boton = IZQ;
    }
    else
      pulsacion = 0;                          // En cuanto dejo de pulsar resetea contador
      
    if (!digitalRead(BOTON_OK))                // Si se pulsa el boton OK cuando no estaba pulsado
    {
      if (botonPrevio == BOTON_NP)
      boton = OK;
    }
    else
      botonPrevio = BOTON_NP;    
    
    switch (Parametro)
    {
      case CONFIG_AMP:    if (modoRegulador == REG_AUTO)
                          {
                            if (boton != OK)
                            {
                                if ((pulsacion < 7) && (corrienteSel > 0) && (boton == IZQ))
                                  corrienteSel -= 1;
                                 
                                if ((pulsacion > 6) && (corrienteSel > 9) && (boton == IZQ))
                                  corrienteSel -= 10;
                                 
                                if ((corrienteSel > 99) && (boton == FASTDOWN))
                                  corrienteSel -= 100;
                                  
                                if (((pulsacion > 6) && (corrienteSel < 10) && (boton == IZQ)) || ((corrienteSel < 100) && (boton == FASTDOWN)))
                                  corrienteSel = 0;
      
                                if ((pulsacion < 7) && (byte (corrienteSel) <= (limImax - 1)) && (boton == DER))
                                  corrienteSel += 1;
                                   
                                if ((pulsacion > 6) && (byte (corrienteSel) <= (limImax - 10)) && (boton == DER))
                                  corrienteSel += 10;
      
                                if ((byte (corrienteSel) <= (limImax - 100))  &&  (boton == FASTUP))
                                  corrienteSel += 100;
      
                                if (((pulsacion > 6) && (byte (corrienteSel) > (limImax - 10)) && (boton == DER)) || ((byte (corrienteSel) > (limImax - 100)) && (boton == FASTUP)))
                                  corrienteSel = limImax;                            
      
                                if(corrienteSel > limImax)  // NO DEBERIA SER NECESARIO
                                  corrienteSel = limImax;
                            }    
 
                            RecalculaAMP (1);                            // Da igual el parametro que le pase porque va a crear el string con la corrienteSel  
                          
                            strToPrint = "Iselect:";
   
                            blankLinea = 8 - strValorAMP.length();
                            
                            for (i = 0; i != blankLinea; i++)
                              strToPrint += " ";
                            
                            strToPrint += strValorAMP;         
                            PintaTexto (strToPrint);
                            lcd.setCursor (0,1);
                            lcd.print ("<- [++]  <OK> +>");
                          }
                          else if (modoRegulador == REG_MAN)
                          {
                            if (boton != OK)
                            {
                                if ((pwmSel > 0) && (boton == IZQ))
                                  pwmSel -= 1;
      
                                if ((pwmSel < 100) && (boton == DER))
                                  pwmSel += 1;
                                  
                                OCR1A = (pwmSel * 0xff) / 100;                                  
                            }
                            
                            strValorPWM = String (pwmSel) + "%";
                            
                            strToPrint = "Regulacion:";   
                            blankLinea = 5 - strValorPWM.length();
                            
                            for (i = 0; i != blankLinea; i++)
                              strToPrint += " ";
                            
                            strToPrint +=  strValorPWM;
                            
                            PintaTexto (strToPrint);
                            lcd.setCursor (0,1);
                            lcd.print ("<-       <OK> +>");
                          }
                          break;
                                                  
      case CONFIG_FREC1:  if (boton != OK)
                          {
                            switch (prescaler1)
                            {
                              case 1024: if (boton == DER)
                                           prescaler1 = 256;
                                         break;
                              case 256:  if (boton == DER)                              prescaler1 = 128;
                                         else if (boton == IZQ)                         prescaler1 = 1024;
                                         break;
                              case 128:  if(boton == DER)                               prescaler1 = 64;
                                         else if (boton == IZQ)                         prescaler1 = 256;
                                         break;                                              
                              case 64:   if (boton == DER)                              prescaler1 = 32;
                                         else if (boton == IZQ)                         prescaler1 = 128;
                                         break;
                              case 32:   if (boton == DER)                              prescaler1 = 8;
                                         else if (boton == IZQ)                         prescaler1 = 64;
                                         break;
                              case 8:    if (boton == DER)                              prescaler1 = 1;
                                         else if (boton == IZQ)                         prescaler1 = 32;
                                         break;
                              case 1:    if (boton == IZQ)                              prescaler1 = 8;
                                         break;
                            }
                            ConfiguraPrescalers();
                            RecalculaFREC1();                         
                          }
                          PintaTexto (strValorFREC1);
                          lcd.setCursor (0,1);
                          lcd.print ("<-       <OK> +>"); 
                          break;
                          
      case CONFIG_FREC2:  if (boton != OK)
                          {
                            switch (prescaler2)
                            {
                                case 256:  if (boton == FASTUP)        {  OCR2A = 254;  prescaler2 = 128;  }
                                           else if (boton == FASTDOWN)    OCR2A = 255;
                                           else if (boton == DER)
                                           {
                                              if (--OCR2A < 128)       {  OCR2A = 254;  prescaler2 = 128;  }
                                           }                                           
                                           else if ((boton == IZQ) && (OCR2A < 255))   OCR2A++;
                                           break;
                                case 128:  if (boton == FASTUP)        {  OCR2A = 254;  prescaler2 = 64;  }
                                           else if (boton == FASTDOWN) {  OCR2A = 128;  prescaler2 = 256; }
                                           else if (boton == DER)
                                           {
                                             if (--OCR2A < 128)        {  OCR2A = 254;  prescaler2 = 64;  }
                                           }                                                                                       
                                           else if (boton == IZQ)
                                           {
                                             if (++OCR2A > 254)        {  OCR2A = 128;  prescaler2 = 256; }
                                           }               
		                           break;                                             
                                case 64:   if (boton == FASTUP)        {  OCR2A = 254;  prescaler2 = 32;  }
                                           else if (boton == FASTDOWN) {  OCR2A = 128;  prescaler2 = 128; }
                                           else if (boton == DER)
                                           {
                                             if (--OCR2A < 128)        {  OCR2A = 254;  prescaler2 = 32;  }
                                           }                                           
                                           else if (boton == IZQ)
                                           {
                                             if (++OCR2A > 254)        {  OCR2A = 128;  prescaler2 = 128; }
                                           }               
		                           break;
                                 case 32:  if (boton == FASTUP)        {  OCR2A = 254;  prescaler2 = 8;  }
                                           else if (boton == FASTDOWN) {  OCR2A = 128;  prescaler2 = 64; }
                                           else if (boton == DER)
                                           {
                                             if (--OCR2A  < 64)        {  OCR2A = 254;  prescaler2 = 8;  }
                                           }                                          
                                           else if (boton == IZQ)
                                           {
                                             if (++OCR2A > 254)        {  OCR2A = 128;  prescaler2 = 64; }
                                           }
                                           break;                                            
                                 case 8:   if (boton == FASTUP)        {  OCR2A = 254;  prescaler2 = 1;  }
                                           else if (boton == FASTDOWN) {  OCR2A = 64;   prescaler2 = 32; }
                                           else if (boton == DER)
                                           {
                                             if (--OCR2A  < 32)        {  OCR2A = 254;  prescaler2 = 1;  }
                                           }                                           
                                           else if (boton == IZQ)
                                           {
                                             if (++OCR2A > 254)        {  OCR2A = 64;   prescaler2 = 32; }
                                           }
                                           break;
                                 case 1:   if (boton == FASTUP)           OCR2A = 1;
                                           if (boton == FASTDOWN)      {  OCR2A = 32;   prescaler2 = 8;  }
                                           else if ((boton == DER) && (OCR2A > 1))  OCR2A--;
                                           else if (boton == IZQ)
                                           {
                                             if (++OCR2A > 254)        {  OCR2A = 32;   prescaler2 = 8;  }
                                           }
                                           break;
                            }
                            OCR2B = OCR2A / 2;                  // Para la frecuencia secundaria duty siempre 50%
                            ConfiguraPrescalers();
                            RecalculaFREC2 ();                         
                          }
                          PintaTexto (strValorFREC2); 
                          lcd.setCursor (0,1);
                          lcd.print ("<- [++]  <OK> +>"); 
                          break;    
                          
      case CONFIG_UMBRAL: if (boton != OK)
                          {
                              if ((pulsacion < 7) && (tensionUmbral >= 1) && (boton == IZQ))
                                tensionUmbral -= 1;
                               
                              if ((pulsacion >= 7) && (tensionUmbral >= 10) && (boton == IZQ))
                                tensionUmbral -= 10;
                                
                              if ((pulsacion >= 7) && (tensionUmbral < 10) && (boton == IZQ))
                                tensionUmbral = 0;
    
                              if ((pulsacion < 7) && (byte (tensionUmbral) <= (limImax - 1)) && (boton == DER))
                                tensionUmbral += 1;
                                 
                              if ((pulsacion >= 7) && (byte (tensionUmbral) <= (limImax - 10)) && (boton == DER))
                                tensionUmbral += 10;
    
                              if ((pulsacion >= 7) && (byte (tensionUmbral) > (limImax - 10)) && (boton == DER))
                                tensionUmbral = limImax;                            
                          }
                 
                          strValorUMBRAL =  "Arranca a " + String (tensionUmbral / 100) + "." + String (tensionUmbral % 100) + "V<-       <OK> +>";
                         
                          PintaTexto (strValorUMBRAL);                         
                          break;
    }
    
    delay (200);
  }
  while(boton != OK);
  
  modo = VER_VALORES;
  GuardaValores();
}

/**
    RecalculaTEMP
**/
void RecalculaTEMP(void)
{
  double temp;
  unsigned int tempADC;                            // Valor de la medida analogica de temperatura de 0 - 0x7ff
  
  tempADC = analogRead (TEMP_PIN);
  
  temp = log(((10240000/tempADC) - 10000));
  temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp ))* temp );
  temp = temp - 273.15;
   
  if(temp > 99)
    strValorTEMP = "Max!";
  else
    strValorTEMP = String (int(temp)) + "oC"; 
}

/**
    RecalculaBAT
**/
void RecalculaBAT(void)
{
  unsigned int batADC;  // Valor de la medida analogica de tension
  String punto;
  unsigned int decimales;
  
  batADC = analogRead (BAT_PIN);

  volts = (batADC * (1500/1024.0)) + 43;                    // Multiplica por 3 por el divisor de 10K + 10K + 10K. Le sumo offset medido en la practica
  
  if(volts < 0)
    volts = 0;

  decimales = int(volts) % 100;
  
  if(decimales < 10)
    punto = ".0";
  else
    punto = ".";

  strValorBAT = String (int(volts) / 100) +  punto + String (decimales) + "V";
}

/**
    RecalculaAMP
**/
void RecalculaAMP(unsigned int iteraciones)
{
  unsigned long regA, regB;
  unsigned int voltsADC, verCorriente;
  String punto;
  unsigned int decimales;

  if (modo == CONFIG_AMP)                  // Si estamos en modo configurar corriente, tengo que mostrar la corriente seleccionada
    verCorriente = corrienteSel;                
  else                                     // Si no, tengo que mostrar la corriente medida            
  {
    voltsADC = sumatorioAMP/iteraciones;

    sumatorioAMP = 0;

    // El valor de 0A corresponde con una medida voltsADC = 510
    if (voltsADC > 518)                          // Si la medida es positiva le resto el offset
      voltsADC -= 518;
    else                                         // Si no, la fijo a 0 ya que no puede ser negativa
      voltsADC = 0;
  
    // La resolucion es de 39mA/bit
    corrienteSens = ((voltsADC * 39) / 10);      // Solo divido por 10 por que correntSens es I*10 y los mV los he multiplicado * 1000.
    
    verCorriente = corrienteSens;               // Si estamos en modo ver corriente, tengo que ver la corriente medida
  }

  decimales = verCorriente % 100;
  
  if(decimales < 10)
    punto = ".0";
  else
    punto = ".";
    
  strValorAMP = String (verCorriente / 100) +  punto + String (decimales) + "A";
   
  if(((modoReguladorPrevio != modoRegulador) && (modoRegulador == REG_MAN)) || (corrienteSelAnt != corrienteSel))        // Si el regulador esta activado o ha variado la seleccion de corriente, reajustamos la s  
  {
    if(!calibrando)
      analogWrite(PWM2_PIN, 255-(corrienteSel/10));          // Si estoy calibrando dejo PWM2_PIN al maximo

    corrienteSelAnt = corrienteSel;
  }
}

/**
    RecalculaFREC1. Salida PWM por pin OC1A
**/
void RecalculaFREC1(void)
{
  frec1 = 62500 / prescaler1;                    // Calculo la frecuencia correspondiente para mostrarla en el display

  strValorFREC1 = "F1: ";

  String StrFREC1 = String (frec1);
  
  i = 10 - StrFREC1.length();
 
  for (j=0; j != i; j++)
    strValorFREC1 += " ";                        // Añade los ceros necesarios en funcion de la longitud del valor      

  strValorFREC1 += StrFREC1 + "Hz";   
}

/**
    RecalculaFREC2. Salida PWM por pin OC2B
**/
void RecalculaFREC2(void)
{  
  frec2 = (16000000 / prescaler2) / OCR2A;        // Calculo la frecuencia correspondiente para mostrarla en el display
  
  strValorFREC2 = "F2: ";
  String StrFREC2 = String (frec2);
  
  i = 10 - StrFREC2.length();
  
  for (j=0; j != i; j++)
    strValorFREC2 += " ";                        // Añade los ceros necesarios en funcion de la longitud del valor    
      
  strValorFREC2 += StrFREC2 + "Hz";
}

/**
  PintaTexto (String texto)
**/
void PintaTexto (String texto)
{
  String linA, linB;
  
  linA = texto;
  linB = texto.substring (16);
    
  int numCarTexto = texto.length();      // Obtiene el numero de caracteres del string
 
  if (linA != lineaA)                    // Si lo que va a escribir es diferente de lo que hay escrito
  {
    lcd.setCursor (0, 0);
    lcd.print (texto);                  // Representa la primera linea
    lineaA = linA;                      // Memoriza el texto actual
  }
  
  if ((numCarTexto > 16) && (linB != lineaB))                  // Hay linea B y lo que va a escribir es diferente de lo que hay escrito
  {
     lcd.setCursor ( 0, 1 );
     lcd.print (texto.substring (16));  // Representa los caracteres que pasan de 16 en la linea de abajo
     lineaB = linB;                        // Memoriza el texto actual
   }
}


/**
  SetRegulador(byte modoReg)
**/
void SetRegulador (byte modoReg)
{
  switch (modoReg)
  {    
    case REG_OFF:   TCCR1B &= 0xf8;              // Pongo CS12, CS11 y CS10 a 0
                    TCCR2B &= 0xf8;              // Pongo CS22, CS21 y CS20 a 0
                    pinMode (PWM1_PIN, INPUT);
                    pinMode (PWM2_PIN, INPUT);               
                    break;
                    
    case REG_AUTO:  ConfiguraPrescalers();
                    RegulaCorriente();
                    pinMode (PWM1_PIN, OUTPUT);
                    pinMode (PWM2_PIN, OUTPUT);
                    break;
                   
    case REG_MAN:   ConfiguraPrescalers();                   
                    OCR1A = (pwmSel * 0xff) / 100;
                    pinMode (PWM1_PIN, OUTPUT);
                    pinMode (PWM2_PIN, OUTPUT);
                    break;
  }
}

void RegulaCorriente(void)
{
  //NOta: tener en cuenta que es 16bits
  OCR1A = (((corrienteSel + corrienteSens)/2) * 100) / resolucionMA;    // Cada cierto tiempo se reajusta el valor de OCR1A para ajustar la salida
}
        
void ConfiguraPrescalers(void)
{
  byte cs2cs1cs0;
  switch (prescaler1)
  {
    case 256: cs2cs1cs0 = 6; break;
    case 128: cs2cs1cs0 = 5; break;
    case 64:  cs2cs1cs0 = 4; break;
    case 32:  cs2cs1cs0 = 3; break;
    case 8:   cs2cs1cs0 = 2; break;
    case 1:   cs2cs1cs0 = 1; break;
  }
  TCCR1B = (TCCR1B & 0xf8) | cs2cs1cs0;     //Actualizo el valor del prescaler en el registro para que tenga efecto sobre la salida PWM
  
  switch (prescaler2)
  {
    case 1024: cs2cs1cs0 = 7; break;
    case 256:  cs2cs1cs0 = 6; break;
    case 128:  cs2cs1cs0 = 5; break;
    case 64:   cs2cs1cs0 = 4; break;
    case 32:   cs2cs1cs0 = 3; break;
    case 8:    cs2cs1cs0 = 2; break;
    case 1:    cs2cs1cs0 = 1; break;
 }
 TCCR2B = (TCCR2B & 0xf8) | cs2cs1cs0;   //Actualizo el valor del prescaler en el registro para que tenga efecto sobre la salida PWM
}

void Calibra(void)
{
  unsigned int cont;
  byte contSecs = 0;
  byte minutos, segundos;
  unsigned int decimales;
  String punto;

  PintaTexto ("   Iniciando    ""   Calibracion  ");  delay(1000);
  PintaTexto ("  Espera a que  ""    finalice..  ");  delay(1000);
  PintaTexto ("  Puede llevar  ""varios minutos..");  delay(1000);
  PintaTexto (" Pulsa    [OK]  "" para cancelar..");  delay(1000);
  
  calibrando = true;                            // Para que la corriente se mantenga al maximo mientras se calibra
  analogWrite(PWM2_PIN, 0);                     // Pongo el pwm al 100%
  
  PintaTexto ( "   Calibrando   ""                ");
  
  sumatorioAMP = 0;
  
  for (cont = 0; cont < 1800; cont ++)          // 30 minutos como maximo
  {
    minutos = byte (cont / 60);
    segundos= byte ( cont - (minutos * 60));
    
    lcd.setCursor ( 0, 1 );
    
    if(minutos < 10)
      lcd.print ("0");
    
    lcd.print (String (minutos) + ":");
 
    if(segundos < 10)
      lcd.print ("0");
    
    lcd.print (String (segundos));
        
    sumatorioAMP += analogRead (AMP_PIN);
    
    if((cont & 0x1f) == 0x10)                     // cada 17 vueltas (segundos) recalcula la media de corriente y la muestra
    {
        RecalculaAMP(17);

        if( corrienteSens < 999)
          lcd.print ("    ");
        else
          lcd.print ("   ");
    
        decimales = corrienteSens % 100;
      
        if(decimales < 10)
          punto = ".0";
        else
          punto = ".";
    
        lcd.print ("I:" + String (corrienteSens / 100) +  punto + String (decimales) +  "A");
    }
    
    if (contSecs > 60)
    {
     // RecalculaAMP(60);
      
      if ((corrienteSens - corrienteAnt) < 20)  // Si la corriente actual medida solo ha subido 0,2A desde la ultima medida lo doy por estable
      {                                         // Si se pulsa el boton de modo, tambien
        if(corrienteSens > 0xff)                // La compara para ver que sea menor de 25,5A
          limImax = 0xff;                       // Si ha llegado al maximo, fija el valor
        else
          limImax = corrienteSens;              // Limita el fondo de escala

        if(corrienteSel > limImax)
          corrienteSel = limImax;        
        
        resolucionMA = byte ((corrienteSens * 100) / 255);  // Si, ej: I = 15 A (150), esta sera la corriente maxima, asi que la resolucion del pwm sera 150*100/255 = 60mA por bit
        
        cont = 1800;      // para salir del bucle for
      } 
     
      corrienteAnt = corrienteSens;             // La guardo para la proxima comparacion
      contSecs = 0;                             // Reseteo el contador de segundos
    } // fin de comparacion cada minuto
    
    if (!digitalRead(BOTON_MODO))              // Si se pulsa el boton de modo, tambien fijo Imax
    {                                         
      corrienteAnt = corrienteSens;           // La guardo para la proxima comparacion
      
      if(corrienteSens > 0xff)                // La compara para ver que sea menor de 25,5A
        limImax = 0xff;                       // Si ha llegado al maximo, fija el valor
      else
        limImax = corrienteSens;              // Limita el fondo de escala

        if(corrienteSel > limImax)
          corrienteSel = limImax;
      
      resolucionMA = byte ((corrienteSens * 100) / 255);  // Si, ej: I = 15 A (150), esta sera la corriente maxima, asi que la resolucion del pwm sera 150*100/255 = 60mA por bit
     
      cont = 1800;      // para salir del bucle for
    } // Fin de salida por boton
            
    delay(1000);
    contSecs++;
  }  // bucle for
  
  calibrando = false;
  modo = VER_VALORES;
  DisplayModo (modo);
}

/**
    GuardaValores: Guarda variables en la EEPROM
**/
void GuardaValores(void)
{
  
  EEPROM.write(0, byte(corrienteSel & 0xff)); 
  EEPROM.write(1, byte((corrienteSel >> 8) & 0xff));
  
  EEPROM.write(2, byte(tensionUmbral & 0xff));  
  EEPROM.write(3, byte((tensionUmbral >> 8) & 0xff)); 

  EEPROM.write(4, byte(OCR2A & 0xff));   
  
  EEPROM.write(5, byte(prescaler1 & 0xff));  
  
  EEPROM.write(6, byte(prescaler2 & 0xff));  
  EEPROM.write(7, byte((prescaler2 >> 8) & 0xff)); 
  
  EEPROM.write(8, byte(limImax & 0xff));  
  EEPROM.write(9, byte((limImax >> 8) & 0xff));   
  
  EEPROM.write(10, byte(pwmSel & 0xff));
  EEPROM.write(11, byte(modoRegulador));
}

/**
    RecuperaValores: Recupera variables dela EEPROM
**/
void RecuperaValores(void)
{
  corrienteSel = EEPROM.read(1) << 8;
  corrienteSel = (corrienteSel << 8) + EEPROM.read(0);
  
  tensionUmbral = EEPROM.read(3);
  tensionUmbral = (tensionUmbral << 8) + EEPROM.read(2);
  
  OCR2A = EEPROM.read(4);   
  
  prescaler1 = EEPROM.read(5); 

  prescaler2 = EEPROM.read(7);
  prescaler2 = (prescaler2 << 8) + EEPROM.read(6);
  
  limImax = EEPROM.read(9);
  limImax = (limImax << 8) + EEPROM.read(8);
  
  pwmSel = EEPROM.read(10);
  modoRegulador = EEPROM.read(11);
}
