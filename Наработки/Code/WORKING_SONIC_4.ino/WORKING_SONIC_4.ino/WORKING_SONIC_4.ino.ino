// Начальная настройка УЗ датчика(ов)
#include <iarduino_HC_SR04_int.h>  
#define TRIG_FRONT 18
#define ECHO_FRONT 19
#define TRIG_RIGHT 2
#define ECHO_RIGHT 3
#define TRIG_LEFT 20
#define ECHO_LEFT 21
iarduino_HC_SR04_int hcsr1(TRIG_FRONT, ECHO_FRONT);
iarduino_HC_SR04_int hcsr2(TRIG_RIGHT, ECHO_RIGHT);
iarduino_HC_SR04_int hcsr3(TRIG_LEFT, ECHO_LEFT);
float f_dist = 0;
float r_dist = 0;
float l_dist = 0;

// Начальная настройка Т датчика(ов)
const int MAX_RESULTS = 10; // Размер Массива
volatile int results [MAX_RESULTS];
volatile int resultNumber;
byte bt[2];

// Начальная настройка мотора
#define STBY 41 // 
#define PWMA 46 // 
#define PWMB 45

#define AIN1 44 // 
#define AIN2 47 // 
#define BIN2 4 // 
#define BIN1 5 // 
int zeroSpeed = 0;
int minSpeed = 180;
int normSpeed = 200;
int maxSpeed = 215;

// Начальная настройка ПИД регулятора
#include "GyverPID.h"
int period = 100;
GyverPID pid(0.5, 0, 0);

// Начальная настройка - дополнительно
bool GoAhead = true;



void setup() {
  // put your setup code here, to run once:  
  Serial.begin(115200);
  motorSetup();
  delay(5000);
  pid.setpoint = 10;
  pid.setDt(period); 
  VKL_ADC(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(period);
  f_dist = hcsr1.distance();
  r_dist = hcsr2.distance();
  l_dist = hcsr3.distance();

  pid.input = l_dist;
  pid.getResult();

  if(r_dist < 18)
  {
    if(f_dist < 40)
    {
      DO_MOTOR_GO_AHEAD(zeroSpeed);
    }
    else
    {
      if(f_dist < 70)
      {
        DO_MOTOR_GO_AHEAD(minSpeed);
      }
      else
      {
        DO_MOTOR_GO_AHEAD(normSpeed);
      }
    }
  }
  else
  {
    DO_MOTOR_GO_AHEAD(zeroSpeed);
  }

  /*Serial.print("THE FRONT DIST IS : ");
  Serial.println(f_dist);
  Serial.print("THE RIGHT DIST IS : ");
  Serial.println(r_dist);
  Serial.println("=============================================");*/

  

  //analogWrite(..., pid.output);

  Serial.print(pid.input); Serial.print(' ');
  Serial.print(pid.output); Serial.print(' ');
  Serial.print(pid.integral); Serial.print(' ');
  Serial.println(pid.setpoint);
}



void motorSetup()
{
  pinMode(PWMA, OUTPUT);  
  pinMode(PWMB, OUTPUT);  
  
  digitalWrite(AIN1, LOW); // крутим моторы в одну сторону
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);  
  
  pinMode(STBY, OUTPUT);  

  digitalWrite(STBY, HIGH); 
}

void DO_MOTOR_GO_AHEAD(int mSpeed)
{
  digitalWrite(AIN1, LOW); // крутим моторы в одну сторону
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, mSpeed);
  if(mSpeed == 0)
  {
    analogWrite(PWMB, mSpeed);
  }
  else
  {
    analogWrite(PWMB, mSpeed + pid.output);
  }
}
void DO_MOTOR_GO_BACK(int mSpeed)
{
  digitalWrite(AIN1, HIGH); // крутим моторы в одну сторону
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, mSpeed);
  analogWrite(PWMA, mSpeed);
}

int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) 
{
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

void VKL_ADC() // Включение АЦП прерываний
{
    ADMUX |= 1<<REFS0 | 0<<REFS1; // ПОДКЛЮЧАЕМ К AREF V
    ADMUX |= 0<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0; // измеряем на ADC7
    ADCSRA |= (1 << ADPS2);                     //Биту ADPS2 присваиваем единицу - коэффициент деления 16
    ADCSRA &= ~ ((1 << ADPS1) | (1 << ADPS0));  //Битам ADPS1 и ADPS0 присваиваем нули
    ADCSRB |= 0<<ADTS2 | 0<<ADTS1 | 0<<ADTS0;  // включаем АЦ каналы MUX, режим скользящей выборки
    sei(); // устанавливаем флаг прерывания
    ADCSRA |= (1 << ADATE); // Включаем автоматическое преобразование
    ADCSRA |= (1 << ADIE);  // Разрешаем прерывания по завершении преобразования
    ADCSRA |= (1 << ADEN);  // Включаем АЦП
    ADCSRA |= (1 << ADSC);  // Запускаем преобразование
    resultNumber=0; // ОБНУЛЕНИЕ ДЛЯ МАССИВА
}

ISR(ADC_vect)
{
  bt[1] = ADCL; // сохраняем младший байт результата АЦП
  bt[2] = ADCH;   // сохраняем старший байт АЦП


  results[resultNumber++] = bt[2]*256+bt[1]; // ЧИТАЕМ ЗНАЧЕНИЕ И ЗАПИСЫВАЕМ В МАССИВ
  if(resultNumber == MAX_RESULTS) // ЕСЛИ МАССИВ ЗАПОЛНЕН, ТО ВЫКЛЮЧАЕМ АЦП, И ЖДЕМ ПОКА LOOP ЕГО СНОВА НЕ ВКЛЮЧИТ
  {
    ADCSRA = 0; // ВЫКЛЮЧЕНИЕ АЦП ПРЕРЫВАНИЙ
  }
}


