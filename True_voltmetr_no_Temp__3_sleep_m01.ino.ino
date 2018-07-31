/* Видеообзоры и уроки работы с ARDUINO на YouTube-канале IOMOIO: https://www.youtube.com/channel/UCmNXABaTjX_iKH28TTJpiqA
 *  
 
 * Copyright 2015 Yuriy Tim http://tim4dev.com
 * На основе оригинального кода (c) Scott Daniels http://provideyourown.com
 *
 * В ATmega есть внутренний источник опорного напряжения 1.1В (другое название internal bandgap reference voltage), которое не зависит от Vcc.
 *
 * Даташит на ATMEGA 328
 * http://www.atmel.com/devices/atmega328p.aspx?tab=documents
 *   "bandgap reference voltage" :
 *   min = 1.0
 *   typical = 1.1
 *   max = 1.2
 *   
 *   В предыдущей версии отлажен вольтметр.
 *   Отлажен вывод на экран температуры и 4 напряжений.
 *   Необходимо отладить логику работы переключателей.
 *   
 */
// Вольтметр до 50В
// для отладки через консоль, обеспечивает вывод на монитор последовательного порта

const uint8_t debug = 4;              // Режим отладки, 0-отключено, >0-выключено

#include <avr/sleep.h>
#include <OneWire.h>                  // Подключаем библиотеку для взаимодействия с устройствами, работающими на шине и по протоколу 1-Wire
#include <OLED_I2C.h>                 // Подключаем библиотеку OLED_I2C для работы со шрифтами и графикой
OLED  myOLED(A0, A1, 8);              // Ds,bn gjhn A3 Определяем пины I2C интерфейса: UNO и NANO -- SDA - пин A2, SCL - пин A3; MEGA -- SDA - пин 20, SCL - пин 21//
extern uint8_t RusFont[];                           // Подключаем русский шрифт
extern uint8_t SmallFont[];                         // Подключаем латинский шрифтом
extern uint8_t TinyFont[];
extern uint8_t MediumNumbers[];
extern uint8_t BigNumbers[];          // Подключаем шрифты цифровые
extern uint8_t MegaNumbers[]; 

//Определение пинов для выходных управляющих сигналов
#define ONE_WIRE_BUS  2               // Указываем пин подключения data-вывода датчика температуры
#define term_power    4               // Указываем пин подключения питания датчика температуры
#define GENERATOR_pin 5               // Указываем пин подключения запуска генератора Солнечн. панели (1 выключает генератор)
#define p_channel_pin 6               // Указываем пин подключения открывания (1 открывает ключ)
OneWire oneWire(ONE_WIRE_BUS);        // Сообщаем библиотеке об устройстве, работающем по протоколу 1-Wire
// Определяем пины аналоговых входов в виде масива
/* 
#define V0 A2                            // 16 вывод напряжение V0 на солнечной панели
#define V1 A6                            // 20!!! Возможна неисправность 15 и 17 вывод напряжение V1 на батарейке
#define V4 A4                            // 18 вывод напряжение V4 после блокинга от панели (до p- ключа)
#define V5 A5                            // 19 вывод напряжение V5 после USB ключа (до разделяющего диода)
*/

const uint8_t arr=4;                  // определяем размерность массива для измеряемого напряжения
uint8_t Varr[arr]={16,20,18,19};      // определение выводов 14(А0)-дисплей-15(А1), 16(A2) 18(А4), 19(А5), 20(А6), 21(А7)
                                      //Для измерения напряжений
float VF[arr];                        // Массив переменных для напряжений

uint8_t i;                          // переменные для счетчиков
uint8_t j;
int m = 0;
uint8_t blink_m = 0;
const uint8_t COUNT = 16;             // определяем счетчик для измерения напряжения


// Делитель напряжения
const float r1 = 3318.5;           // 3,3K резистор, идущий к земле
const float r2 = 3300;           // 3,3K
const float typVbg = 1.09999;      // 1.0 -- 1.2 величина опорного напряжения, подбирается индивидуально.
const float k = r2 / (r1 + r2); // коэффициент, используется при вычислениях
const float Dev = (r1 + r2)/(1023.0*r2);  // определение опорного напряжения


float Vcc =         0.0;         // Измеренное напряжение питания при самотестировании
float VccDev =      0.0;         // Промежуточные переменные
//float VccDevCOUNT = 0.0;
float MaxVoltage =  0.0;         // Максимальное измеряемое напряжение (через делитель, без перегрузки портов)
uint16_t ResultADC =  0;
long previousMillis = 0;        // храним время последнего переключения светодиода 
long interval = 10000;           // интервал между включение/выключением светодиода (1 секунда)


/*
float V0_F = 0;                    // 14 вывод напряжение V0 на солнечной панели
float V1_F = 0;                    // 15 вывод напряжение V1 на батарейке
float V4_F = 0;                    // 18 вывод напряжение V4 после блокинга от панели (до p- ключа)
float V5_F = 0;                    // 19 вывод напряжение V5 после USB ключа (до разделяющего диода)

Примечание:
Цифровык пины: 0-7 (PD0-7) и 8-13 (PB0-5)
DDRB = DDRB | B10000000;    // устанавливаем 13 пин как OUTPUT, не изменяя остальные
PORTB = PORTB | B10000000;  // устанавливаем состояние pin13 как HIGH за 4 такта, не изменяя остальные
PORTB |= 1<<7;              // аналогично устанавливаем состояние pin13 как HIGH за 4 такта, не изменяя остальные
PORTD |= 1<<4;              // аналогично устанавливаем состояние pin4 как HIGH за 4 такта, не изменяя остальные
PORTB =  B00000000;         // устанавливаем состояние pin13 и остальных, как LOW
PORTD &=  ~(1<<4);          // устанавливаем состояние pin4 и остальных, как LOW
PORTB &= B01111111;         // устанавливаем состояние только pin13 как LOW
PORTB = PORTB & B01111111;  // аналогично устанавливаем состояние только pin13 как LOW

Пример установки битов регистра ADMUX в 0 и 1:
ADMUX &= _BV(MUX2) | _BV(MUX1) | _BV(MUX0) 
ADMUX &= ~_BV(MUX2) & ~_BV(MUX1) & ~_BV(MUX0)ADMUX|= _BV(MUX2) | _BV(MUX1) | _BV(MUX0) 
ADMUX &= ~_BV(MUX2) & ~_BV(MUX1) & ~_BV(MUX0)

Bit Math Tutorial by CosineKitty
http://playground.arduino.cc/Code/BitMath
Port Registers
https://www.arduino.cc/en/Reference/PortManipulation
Используем бесконечный цикл для ускорения функции LOOP
while (1)
 {
 }
Каждый из 14 цифровых выводов Pro, используя функции pinMode(), digitalWrite(), и digitalRead(), 
может настраиваться как вход или выход. Выводы работают при напряжении 3,3 В. 
Каждый вывод имеет нагрузочный резистор (стандартно отключен) 20-50 кОм и может пропускать до 40 мА. 
Некоторые выводы имеют особые функции:

Последовательная шина: 

0 (RX) и 1 (TX). 
Выводы используются для получения (RX) и передачи (TX) данных TTL. 
Данные выводы имеют соединение с выводами TX-0 и RX-1 блока из шести выводов.

Внешнее прерывание: 
2 и 3. Данные выводы могут быть сконфигурированы на вызов прерывания либо на младшем значении, 
либо на переднем или заднем фронте, или при изменении значения. 
Подробная информация находится в описании функции attachInterrupt().
ШИМ: 

3, 5, 6, 9, 10, и 11. Любой из выводов обеспечивает ШИМ с разрешением 8 бит при помощи функции analogWrite().
SPI: 10 (SS), 11 (MOSI), 

12 (MISO), 13 (SCK). Посредством данных выводов осуществляется связь SPI, которая, хотя и поддерживается аппаратной частью, не включена в язык Arduino.
LED: 13. Встроенный светодиод, подключенный к цифровому выводу 13. Если значение на выводе имеет высокий потенциал, то светодиод горит. 
На платформе Pro Mini установлены 6 аналоговых входов, каждый разрешением 10 бит (т.е. может принимать 1024 различных значения). Четыре из них расположены на краю платформы, а другие два (входы 4 и 5) ближе к центру. Измерение происходит относительно земли до значения VCC.  Некоторые выводы имеют дополнительные функции:

I2C: A4 (SDA) и A5 (SCL). Посредством выводов осуществляется связь I2C (TWI), для создания которой используется библиотека Wire.

Аналоговые порты, по-умолчанию, определенны на ввод сигнала и в отличие
от цифровых портов их не требуется конфигурировать
с помощью вызова функции pinMode. 
Аналоговые входы (analog pins) могут быть использованы как цифровые вход/выходы (digital pins). 
Обращение к ним идет по номерам от 14 (для аналогового входа 0) до 19 (для аналогового входа 5).
A0 - 14
A1 - 15
A2 - 16
A3 - 17
A4 - 18
A5 - 19
A7 - 21

*/
/* =====================================================
 *  ****************************************************
 * =====================================================
 */
void setup()
{
  myOLED.begin();                     // Инициализируем библиотеку OLED_I2C 
  if (debug > 1) {
  Serial.begin(9600);   
  Serial.println("---+++++");         //Метка начало
  }

  ADCinit_m();                        //Инициализация 
  Vcc = readVcc();
}
/*
 * =====================================================
 */
void loop()
{
  // определение опорного напряжения
  analogReference(DEFAULT);  // DEFAULT INTERNAL использовать Vcc как AREF, т.е. в данном случае 1023 - это Vcc
  
  
// Периодически проверяем питающее напряжение AVcc (не забываем его стабилизировать)? период времени interval
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis > interval) {
        // сохраняем время последнего переключения
        previousMillis = currentMillis; 
        Vcc = readVcc();
      }

//
//1,1 в от внутреннего источника, в документации он проходит как bandgap reference (некоторые из них 2,56  В, например ATMega 2560). 
//Выбор осуществляется функцией analogReference() с параметром INTERNAL: 
//analogReference(INTERNAL) ;
//внешний источник опорного наптяжения, на ардуинке подписан  AREF. Выбор: 
//analogReference(EXTERNAL);
//Vcc — источник питания самого контроллера. Выбор: 
//analogReference(DEFAULT).

// ==== Измерение напряжений, всех четырех и усредняем каждое на число COUNT  
  VccDev=Vcc*Dev;
//  VccDevCOUNT = VccDev/COUNT;
  MaxVoltage = Vcc / k;

    if (debug > 1) {
      Serial.print("Vcc = ");
      Serial.println(Vcc,4);
      Serial.print("Max V. = ");
      Serial.println( MaxVoltage );
      Serial.println("---");
   }

  if (debug > 1) {
    Serial.print("VccDev = ");
    Serial.println(VccDev,4);
    Serial.print("--   Dev = ");
    Serial.println(Dev,6);
    delay(10);
    Serial.print("--");
    delay(10);
//    Serial.print("VccDevCOUNT=");
//    delay(10);
//    Serial.println(VccDevCOUNT,4);
//    delay(10);
    Serial.print("--   COUNT = ");
    delay(10);
    Serial.println(COUNT);
    Serial.print("--   k = ");
    Serial.println(k,4);
  }
// Считываем массив напряжений VF[i]
  for (j=0; j < arr; j++) {
        if (debug > 3) {
        Serial.print("== Varr[j]=");
        Serial.print(Varr[j]);
        Serial.print("== arr=");
        Serial.println(arr);
         }   
        analogNoiseReducedRead(COUNT, Varr[j]); // В переменную ResultADCResultADC из подпрограммы получаем значение напряжения
        VF[j]=ResultADC*VccDev;
     
  }                                    // конец цикла i       

//==== отладочная распечатка
    if (debug > 1) {
  for (i=0; i < arr; i++) {
    Serial.print(" V(");
    Serial.print(i);
    Serial.print(")=");
    Serial.print(VF[i],4);
    
    }                           // конец цикла вывожа строки напряжений
        Serial.println("  =");    Serial.println("  =");
   
  }                             // конец отладочной распечатки

    myOLED.setFont(MediumNumbers);
    if (blink_m < 1){blink_m = 1; myOLED.print(String(blink_m), 5, 0);}
    else{blink_m = 0; myOLED.print(String(blink_m), 5, 0);}
    
//==== Вывод на дисплей  напряжений
    myOLED.setFont(MediumNumbers);
    for (i=0; i<arr; i++){
    if(i<2) myOLED.print(String(VF[i],2), (1+i*60), 15);
    if(i>=2) myOLED.print(String(VF[i],2), (1+(i-2)*60), 45);
    }                 // конец цикла
    myOLED.update();                                   // Обновляем информацию на дисплее
}

/*Функции
 **************************************************** 
 * ===================================================
 */
 


void ADCinit_m() {

//========Разрешения прерываний АЦП микроконтроллера======
//   ADCSRA = ADCSRA|(1<<ADEN)|(1<<ADATE); //Установил разрешение на работу АЦП,подключил автозапуск.
//   ADCSRA = ADCSRA|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0); //установка предделителя для задания частоты АЦП в пределах 100-200кГц, 1/64 FCPU.
//   ADCSRB = ADCSRB|(1<<ACME); //Установил режим работы непрерывного преобразования.
//   ADMUX = ADMUX|(1<<MUX1)|(1<<MUX0);//указал, какой у меня вход задействован в качестве АЦП.
//   ADMUX = ADMUX&(~(1<<REFS0));// установка источника опорного напряжения = VCC
//   ADMUX = ADMUX|(1<<ADLAR); // Результат преобразования сдвинут вправо. Результат можно читать из ADCH при этом в ADCL будут храниться в 7 и 8 бите младшие разряды рпеобразования, ими в нашем случае можно пренебречь.
//   DIDR0 =  DIDR0|(1<<ADC3D); // отключение цифрового буфера от канала ADC3.
//   ADCSRB = (ADCSRB&0b11111000)|(0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0); //Установка источника сигнала для выполнения преобразования...000 - непрерывное преобразование.
//   ADCSRA = ADCSRA|(1<<ADIE); //Разрешение прерывания от АДЦ
//   ADCSRA = ADCSRA|(1<<ADSC);//Запуск первого преобразования!!!
   //=======КОНЕЦ Разрешения прерываний АЦП микроконтроллера=
//http://www.fi-com.ru/mcu/avr_adc.htm

//Код, найденный на форуме Roboternetz.de по ссылке http://www.roboternetz.de/phpBB2/zeigebeitrag.php?t=8497&highlight=adc+atmega16:
//hier ein beispiel aus dem forum, von mir zusammengewurfelt. 

//

//#define ADCinit           ADCSRA|=_BV(ADEN)         // Writing this bit to one enables the ADC
//#define ADCdisable        ADCSRA &=~_BV(ADEN)       // By writing it to zero, the ADC is turned off.
                  //Turning the ADC off while a conversion is in progress, will terminate this conversion. 
//#define ADCstart          ADCSRA|=_BV(ADSC)         // ADC Start Conversion
//                  In Single Conversion mode, write this bit to one to start each conversion. 
//                  In Free Running mode, write this bit to one to start the first conversion. 
//#define ADCfree           ADCSRA|=_BV(ADATE)        // ADC Auto Trigger Enable 
//#define ADCvintern        ADMUX|=_BV(REFS0)     // interne Spannungsversorgung 
//#define ADCinterrupt_on   ADCSRA|=_BV(ADIE)                // ADC Interrupt Enable 
//#define ADCprescaler_2    ADCSRA |=_BV(ADPS0)              // gewunschter Teilungsfaktor/Prescaler 
//#define ADCprescaler_4    ADCSRA|=_BV(ADPS1) 
//#define ADCprescaler_8    ADCSRA=_BV(ADPS1) | _BV(ADPS0) 
//#define ADCprescaler_16   ADCSRA|=_BV(ADPS2) 
//#define ADCprescaler_32   ADCSRA=_BV(ADPS2) | _BV(ADPS0) 
//#define ADCprescaler_64   ADCSRA=_BV(ADPS2) | _BV(ADPS1) 
//#define ADCprescaler_128  ADCSRA=_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) 
//#define ADCprescaler_reset ADCSRA &= ~_BV(ADPS2) & ~_BV(ADPS1) & ~_BV(ADPS0)        
//#define ADCchannel_1                                      //gewunschter Kannal z.B bei ATmega32 PINA0 - PINA7 
//#define ADCchannel_2      ADMUX|= _BV(MUX0)                // bei nicht freilaufen muss ADCchannel_x vor 
//#define ADCchannel_3      ADMUX|= _BV(MUX1)                // ADCstart kommen dann kann man mit getadc() der 
//#define ADCchannel_4      ADMUX =_ BV(MUX1) | _BV(MUX0)    // Adcwert des gewahlten Kannals auslesen 
//#define ADCchannel_5      ADMUX|= _BV(MUX2) 
//#define ADCchannel_6      ADMUX = _BV(MUX2) | _BV(MUX0) 
//#define ADCchannel_7      ADMUX = _BV(MUX2) | _BV(MUX1) 
//#define ADCchannel_8      ADMUX = _BV(MUX2) | _BV(MUX1) | _BV(MUX0) 
//#define ADCchannel_reset  ADMUX = ~_BV(MUX2) & ~_BV(MUX1) & ~_BV(MUX0) 
//==============
  ADCSRA = ADCSRA|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0); //установка предделителя для задания частоты АЦП в пределах 100-200кГц, 1/64 FCPU.
//  ADCSRA |=_BV(ADEN);                  // enable the ADC
//  ADCSRA =_BV(ADPS1); // | _BV(ADPS0);    //Prescaler Select = 64, установка коэффициента деления
//  ADCSRA &= ~_BV(ADATE);               // ADC Auto Trigger disable
//  ADMUX = ~_BV(MUX2) & ~_BV(MUX1) & ~_BV(MUX0); // ADC channels reset
//  ADCSRA|=_BV(ADSC);         // ADC Start Conversion
//  ADCSRB = ~_BV(ADTS2) & ~_BV(ADTS1) & ~_BV(ADTS0);
  
} // End of ADCinit_m()


 //===== Измеряем напряжение питания ===========
 // *******************************************

float readVcc()
{
/*
Данная подпрограмма считавает внутренний источник образцового напряжения (1,1 В), используя в качестве опорного (максимального) 
источник питания Vcc и зная образцовое напряжение обратным счетом вычисляет вычисляет Vcc.
*/

  float result = 0.0;
  long res[8] = {0,0,0,0,0,0,0,0};
  long tmp = 0;
  uint8_t i;

  // Усреднение 8 раз.
  for (i = 0; i < 8; i++) {

//        ADCSRA = ADCSRA|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0); //установка предделителя для задания частоты АЦП в пределах 100-200кГц, 1/64 FCPU.
        ADCSRA |= _BV( ADIE );             //Set ADC interrupt
        set_sleep_mode(SLEEP_MODE_ADC);    //Set sleep mode

      // Read 1.1V reference against AVcc
      // set the reference to Vcc and the measurement to the internal 1.1V reference
        ADMUX |= _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);//Регистр ADMUX (регистр управления мультиплексором) 
        ADMUX &= ~_BV(REFS1) & ~_BV(MUX0);   // REFS1=1, REFS0=0 -AVCC with external capacitor at AREF pin
                                            // MUX3=MUX2=MUX1=1, MUX0=0 to measure 1.1V (VBG)
       
      // works on an Arduino 168 or 328
      
        delay(10);           // Wait for Vref to settle
        ADCSRA |= _BV(ADSC); // Запуск преобразования (в режиме однократного преобразования)
                             //0 – преобразование завершено
                             //1 – начать преобразование
        sleep_enable();                    //Enable sleep (разрешает sleep)
  do
  {                                  
   // Loop until reading is completed
   // The following line of code is only important on the second pass.  For the first pass it has no effect.
   // Ensure interrupts are enabled before sleeping
    sei();                           //Enable interrupts
   // Sleep (MUST be called immediately after sei)
    sleep_mode();                    //Go to sleep
   // Checking the conversion status has to be done with interrupts disabled to avoid a race condition
   // Disable interrupts so the while below is performed without interruption
    cli();                           //Disable interrupts.
  } while (bit_is_set(ADCSRA,ADSC)); // ждем, пока не будет сброшен ADSC
  
//  while(((ADCSRA & (1 << ADSC) ) != 0)); 
  //Loop if the interrupt that woke the cpu was something other than the ADC finishing the reading
  
  
  // while (bit_is_set(ADCSRA,ADSC)); // ждем, пока не будет сброшен ADSC
    sleep_disable();                   //Disable sleep
    ADCSRA &= ~ _BV( ADIE );           //Clear ADC interupt
    sei();                             //Enable interrupts
//    
    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    tmp = (high<<8) | low;
    res[i] = tmp;

      if (debug > 2) {
    delay(100);
    Serial.print("low = ");
    Serial.print(low);    
    Serial.print("---high = ");
    Serial.print(high,4);
    Serial.print("---res[i] = ");
    Serial.println(res[i],4);
      }
  }
  for (i = 0; i < 8; i++) {
      result = result + float(res[i]);
   }
  
  result = (typVbg * 1023.0) *8.0 / result;
  return result;
}   // конец измерения readVcc ====

 //==================================== Измеряем напряжение в спящем режиме =============
int analogNoiseReducedRead(uint8_t COUNTp, uint8_t pinNumber)
{
  long resul_tmp = 0;
  ResultADC = 0;
    if (debug > 3) {
    delay(10);
    Serial.println("=Enter in analogNoiseReducedRead");
    delay(10);
    }
    
   // Усреднение COUNT раз.
  for (i = 0; i < COUNTp; i++) {   
   ADCSRA |= _BV( ADIE );             //Set ADC interrupt
   set_sleep_mode(SLEEP_MODE_ADC);    //Set sleep mode

  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX |= _BV(REFS0);     //Регистр ADMUX (регистр управления мультиплексором)     
  ADMUX &= ~_BV(REFS1);   // Выставляем в качестве опорного Vcc (т.е. Это максимальное напряжение), биты (регистра ADMUX) REFS0=1, REFS1=0
// works on an Arduino 168 or 328

// определение выводов 14(А0)-дисплей-15(А1), 16(A2) 18(А4), 19(А5), 20(А6), 21(А7)
  switch (pinNumber) {
    case 14:
      // A0 (14) 0000
//    ADMUX |= 
      ADMUX &= ~_BV(MUX3) & ~_BV(MUX2) & ~_BV(MUX1) & ~_BV(MUX0);
      break;
    case 15:
      // A1  (15) 0001
      ADMUX |= _BV(MUX0);
      ADMUX &= ~_BV(MUX3) & ~_BV(MUX2) & ~_BV(MUX1); // & ~_BV(MUX0);
      break;
    case 16:
      // A2  (16) 0010
      ADMUX |= _BV(MUX1);
      ADMUX &= ~_BV(MUX3) & ~_BV(MUX2) & ~_BV(MUX0); // & ~_BV(MUX0);
      break;
    case 17:
      // A3  (17) 0011
      ADMUX |= _BV(MUX1) |_BV(MUX0);
      ADMUX &= ~_BV(MUX3) & ~_BV(MUX2); // & ~_BV(MUX0); // & ~_BV(MUX0);
      break;
    case 18:
      // A4  (18) 0100
      ADMUX |= _BV(MUX2); // |_BV(MUX0);
      ADMUX &= ~_BV(MUX3) & ~_BV(MUX1) & ~_BV(MUX0); // & ~_BV(MUX0);
      break;
    case 19:
      // A5  (19) 0101
      ADMUX |= _BV(MUX2) |_BV(MUX0);
      ADMUX &= ~_BV(MUX3) & ~_BV(MUX1); // & ~_BV(MUX0); // & ~_BV(MUX0);
      break;
    case 20:
      // A6  (20) 0110
      ADMUX |= _BV(MUX2) |_BV(MUX1);
      ADMUX &= ~_BV(MUX3) & ~_BV(MUX0); // & ~_BV(MUX0); // & ~_BV(MUX0);
      break;
    case 21:
      // A7  (21) 0111
//    ADMUX |= |_BV(MUX2) |_BV(MUX1) |_BV(MUX0);
      ADMUX &= ~_BV(MUX3); // & ~_BV(MUX0); // & ~_BV(MUX0);
      break;   
    
    }     // конец switch

  
//  ADCSRA |= _BV( ADIE );             //Set ADC interrupt
//  set_sleep_mode(SLEEP_MODE_ADC);    //Set sleep mode
//  reading = analogRead(pinNumber);   //Start reading
//  sleep_enable();                    //Enable sleep (разрешает sleep)
    delay(10);           // Wait for Vref to settle

    if (debug > 3) {
    delay(10);
    Serial.print("=Enter to cycle COUNT=");
    Serial.print(COUNTp);
    Serial.print(", pinNumber=");
    Serial.print(pinNumber);
    Serial.print(", i=");
    Serial.println(i);
    delay(10);
    }
    
    ADCSRA |= _BV(ADSC); // Запуск преобразования (в режиме однократного преобразования)
                         //0 – преобразование завершено
                         //1 – начать преобразование
    sleep_enable();                    //Enable sleep (разрешает sleep)
  
  do
  {                                  //Loop until reading is completed
   // The following line of code is only important on the second pass.  For the first pass it has no effect.
   // Ensure interrupts are enabled before sleeping
    sei();                           //Enable interrupts
   // Sleep (MUST be called immediately after sei)
    sleep_mode();                    //Go to sleep
   // Checking the conversion status has to be done with interrupts disabled to avoid a race condition
   // Disable interrupts so the while below is performed without interruption
    cli();                           //Disable interrupts . ??????
  } while(((ADCSRA & (1 << ADSC) ) != 0)); //Loop if the interrupt that woke the cpu was something other than the ADC finishing the reading
//  sleep_disable();                   //Disable sleep
  ADCSRA &= ~ _BV( ADIE );           //Clear ADC interupt
  sei();                             //Enable interrupts

  // ResultADC получаем в обработчике прерывание

    resul_tmp = resul_tmp + ResultADC;
  
  } // конец цикла усреднения COUNT раз

      ResultADC = resul_tmp / COUNTp;

    if (debug > 3) {  
    delay(10);
    Serial.print("=ResultADC in the end PP= ");
    Serial.println(ResultADC);
    delay(10);
    }
   
//  return(ResultADC);
}     // Конец analogNoiseReducedRead()

 //===== Заглушка по прерыванию от АЦП =============
ISR(ADC_vect) 
{
  cli();                           //Disable interrupts . ??????
  sleep_disable();                   //Disable sleep

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  ResultADC =(high<<8) | low;
    if (debug > 3) {
    delay(10);
    Serial.print("ResultADC in ISR(ADC_vect)= ");
    Serial.println(ResultADC);
    delay(10);
    }

  sei();                             //Enable interrupts
//  
}
