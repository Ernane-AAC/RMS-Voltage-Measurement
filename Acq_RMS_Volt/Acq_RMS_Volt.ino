// UNVERSIDADE FEDERAL DE UBERLÂNDIA - Faculdade de Engenharia Elétrica
// Subject: DSP-EPS - Prof. Ernane A. A. Coelho - http://lattes.cnpq.br/9183492978798433
// CODE for Voltage acquisition (channel 0) and rms value calculation
//  * run on Arduino Lab System (Arduino UNO - Atmega328p)
//  * Sample rate ~=6kHz (5999.25 Hz)  ==> 100 samples per period (fundamental at 60Hz)
//  * Rms calculation processed by the ADC ISR at 6kHz
//  * Each iteration of the ISR: (moving average)
//            -> ISR is invoked when a new sample is ready
//            -> sample is converted to Volts: V=(digital word)*gain + offset
//            -> hundredth sample in the past is subtracted from the sum of the samples (circular buffer)
//            -> the square of the new voltage sample is inserted into the circular buffer
//            -> new squared sample is added to the summation of samples
//            -> calculating the mean squared
//            -> calculation of the new rms value (sqrt(average(V^2))
// Updated rms value is displayed on LCD by main code (rate 2Hz)

#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> //LCD I2C Library
#define LCD_col 16 // defines the number of columns of the used display
#define LCD_lin  4 // defines the number of rows of the used display
#define LDCi2c_Address  0x27 // set the i2c address of the display
// -> Note that SCL/SDA are multiplexed with PC5(ADC5)/PC4(ADC4), respectively, do not use ADC5 and ADC4 channels.

// ============ Config LCD ====================================
LiquidCrystal_I2C lcd(LDCi2c_Address,LCD_col,LCD_lin);

float samples2[100];  //circular buffer of 100 squared samples (400 bytes)
int index;            //index of last entry in circular buffer
float sum;            //sum of samples
int sample;           //digital word - acquisition result
float sample_V;       //sample in Volts
float gain, offset;   //gain and offset result of thr channel calibration with sensor and signal conditioning
float average2;       //sample mean squared
float Vrms;           //rms voltage
int interrupt_counter; // ADC interrupt_counter
char quantity1[10], quantity2[15]; //variables used in formatting the RMS value for writing on the LCD

// ================= SETUP ============================================================
void setup(){
  PORTB = 0x00; //reset PORTB output data
  DDRB |= 0x21; //0010 0001 -> set PB5 (Onboad LED) and PB0 to output
  //------- inicializations ---------
  gain   = 0.84436; // gain of the voltage signal conditioning system
  offset = -432.76; // Offset of the voltage signal conditioning system
  for(index=0;index<100;index++)
     {
      samples2[index]=0; //reset circular buffer
     }
  sum=0;
  index=0; 
  interrupt_counter=0;
    
  lcd.init();
  lcd.backlight();
  Config_ADC();
  Timer1_Init();
  print_home();
  // start acquisitions
  ADCSRA |= 0x08;  //set bit-3 ADIE - AD Interrupt Enable 
  //--------------------------------------------------
  //TIFR1 – Timer/Counter1 Interrupt Flag Register
  //Bit         7  6    5   4  3    2     1     0
  //0x16 (0x36) –  –  ICF1  –  –  OCF1B OCF1A TOV1 
  //Read/Write  R  R   R/W  R  R   R/W   R/W   R/W
  //Init. Val.  0  0    0   0  0    0     0     0
  //-------------------------------------------------- 
  TIFR1 |= 0x01; // Write "1" -> Reset Bit-0 TOV1 (Timer1 Overflow Flag) for triggering the first acquisition  
                 // Timer overflow generates SOC trigger for ADC on rising edge

}
// ================= LOOP ============================================================
void loop(){
  if(interrupt_counter==0)   //refresh display every 0.5s
     {
      dtostrf(Vrms, 5, 1,quantity1);
      sprintf(quantity2,"%6s V", quantity1); 
      lcd.setCursor(22,1);  
      lcd.print(quantity2);
     }
  asm("NOP");
}

//====================================================================================
void print_home(void){
  lcd.clear();
  lcd.setCursor(0,0);           
  //         0123456789012345 
  lcd.print(" RMS Voltmeter  ");
  lcd.setCursor(0,1); 
  lcd.print("6kHz - 100s/per ");
  lcd.setCursor(16,0);
  lcd.print("Channel A0      ");
  lcd.setCursor(16,1);
  //         0123456789012345 
  lcd.print("Vrms:           ");
}

//====================================================================================
// ========== ADC interrupt service  ===============
ISR(ADC_vect)
{
  PINB |=0x01;    //toggle PB0 - used to measure duration of ISR (real time check) 
  //--------------------------------------------------
  //TIFR1 – Timer/Counter1 Interrupt Flag Register
  //Bit         7  6    5   4  3    2     1     0
  //0x16 (0x36) –  –  ICF1  –  –  OCF1B OCF1A TOV1 
  //Read/Write  R  R   R/W  R  R   R/W   R/W   R/W
  //Init. Val.  0  0    0   0  0    0     0     0
  //--------------------------------------------------     
  TIFR1 |= 0x01; // Write "1" -> Reset Bit-0 TOV1 (Timer1 Overflow Flag) for triggering the first acquisition  
                 // Timer overflow generates SOC trigger for ADC on rising edge
                 // As there is no timer interrupt, there is no automatic reset of the flag
                 // Therefore, it is necessary to reset via software to trigger a new acquisition.
 
  sample=ADC;                          //reads new digital sample
  sample_V=gain*(float)sample+offset;  //convert to Volts
  sum = sum - samples2[index];         //removes last squared sample from summation
  samples2[index]=sample_V*sample_V;   //insert new sample squared at position of last sample in buffer
  sum = sum + samples2[index];         //adds the new squared sample to the summation
  index =  index + 1;                  //advance pointer to next iteration
  if(index >= 100) index = 0;          //reset pointer if reached last position in memory (circular buffer)

  average2 = sum*0.01;                 //averages 100 samples
  Vrms = sqrt(average2);               //calculate Vrms
 
  interrupt_counter++;
  if(interrupt_counter>=3000)
     {
       interrupt_counter=0;
       PINB |=0x20;            //blink onboad LED - 1Hz
     }
                 
  PINB |=0x01;   //toggle PB0 - used to measure duration of ISR on oscilloscope connected at pin PB0(real time check)            
}


//====================================================================================
void Config_ADC(void)
{
 //----------------------------------------------------
 //ADMUX – ADC Multiplexer Selection Register
 //Bit         7     6     5    4    3    2    1    0
 //(0x7C)    REFS1 REFS0 ADLAR  –  MUX3 MUX2 MUX1 MUX0 
 //Read/Write R/W   R/W   R/W   R   R/W  R/W  R/W  R/W
 //Init. Val.  0     0     0    0    0    0    0    0
 //This code   0     1     0    0    0    0    0    0   -> Vref=AVCC*; result right adjusted; Mux-> Channel A0   
 //-----------------------------------------------------   * AVCC on Arduino-UNO board is connected to 5V
 //                                                     -> for reference stability, the use of an external power supply (7 to 12V)
                                                         // with a 5V level defined by the internal regulator is better than the power via USB                                             
 ADMUX=0x40;                                             // consider this in calibration process

 //----------------------------------------------------
 //ADCSRA – ADC Control and Status Register A
 //Bit         7    6    5     4    3    2     1     0
 //(0x7A)    ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0 
 //Read/Write R/W  R/W  R/W   R/W  R/W  R/W   R/W   R/W
 //Init. Val.  0    0    0     0    0    0     0     0
 //This code   1    0    1     0    0    1     1     1  -> enable ADC; Auto trigger; disable interrupt*; ADC_Clk=125kHz
 //---------------------------------------------------- -> ADC_Clk affects the precision. Atmega328P datasheet recommends ADC_Clk<200kHz
 //                                                     -> In auto-trigger -> sample time: 2 cycles, total conversion time 13 cycles (13/125kHz=104us)
 //----------------------------------------------------    * it will be enabled at the time of data acquisition
 ADCSRA=0xA7;                                             

 //----------------------------------------------------
 //ADCSRB – ADC Control and Status Register B
 //Bit        7    6   5  4  3    2     1     0
 //(0x7B)     –  ACME  –  –  –  ADTS2 ADTS1 ADTS0      -> Analog Comparator Multiplexer Enable=0; not used in this application
 //Read/Write R   R/W  R  R  R   R/W   R/W   R/W
 //Init. Val. 0    0   0  0  0    0     0     0
 //This code  0    0   0  0  0    1     1     0        -> Trigger-> Timer/Counter1 overflow
 //----------------------------------------------------
  ADCSRB=0x06;

 //----------------------------------------------------
 //DIDR0 – Digital Input Disable Register 0
 //Bit        7  6    5     4     3     2     1     0
 //(0x7E)     –  –  ADC5D ADC4D ADC3D ADC2D ADC1D ADC0D 
 //Read/Write R  R   R/W   R/W   R/W   R/W   R/W   R/W
 //Init. Val. 0  0    0     0     0     0     0     0
 //This Code  0  0    0     0     0     0     0     1 -> disable digital input PC0 -> channel A0 
//----------------------------------------------------
 DIDR0=0x01;
}

//====================================================================================
void Timer1_Init(void)
{              
 //--------------------------------------------------------------- 
 //TCCR1A - Timer/Counter1 Control Register A
 //Bit        7      6      5      4     3     2     1      0
 //(0x80) COM1A1 COM1A0 COM1B1 COM1B0    –     –  WGM11  WGM10 
 //Init. Val. 0      0      0      0     0     0     0      0
 //This code  0      0      0      0     0     0     1      1   -> Normal port operation, OC1A/OC1B disconnected.
 //------------------------------------------------------------ -> Waveform generation Mode=15 Fast PWM  -> WGM=15 ->  WGM11:WGM10=11 
 TCCR1A=0x03;     
 //---------------------------------------------------------------           
 //TCCR1B – Timer/Counter1 Control Register B
 //Bit       7     6     5     4     3     2     1     0
 //(0x81) ICNC1 ICES1    –  WGM13 WGM12  CS12  CS11  CS10 
 //Init. Val. 0      0   0    0     0     0     0      0
 //This code  0      0   0    1     1     0     0      1   -> Waveform generation Mode=15 WGM=15 ->  WGM13:WGM12=11; clk_timer=clk_cpu/1=16MHz 
 //---------------------------------------------------------------  
 TCCR1B=0x19;
                           
 //->Timer cycle= Timer_CLK/(2667 steps)=16MHz/2667=5999.25 Hz (sample rate is not exactly 6kHz!)
 //-> 2667 steps in counting UP -> TOP=2666 (0x0A6A)
 OCR1A=0x0A6A;
         
 //---------------------------------------------------------------
 //TIMSK1 – Timer/Counter1 Interrupt Mask Register
 //Bit        7    6    5    4    3      2      1     0
 //(0x6F)     –    – ICIE1   –    –  OCIE1B OCIE1A  TOIE1 
 //Read/Write R    R   R/W   R    R     R/W    R/W   R/W
 //Init. Val. 0    0    0    0    0      0      0     0      -> timer 1 interrupt control
 //--------------------------------------------------------------- 
 TIMSK1=0x00;  //no timer 1 interrupt is used -> only the trigger for the ADC
} 

//====================================================================================
