/*
  Parts:
    Arduino Pro Mini 3.3V 8MHz (RAW input: 3.4V-12V)
    Digilent PmodTC1 (MAX31855 and K-type thermocouple)
    SSR-2528ZD3 solid state relay
    OLED display 128x64
    LD-BZEN-1205 buzzer
    Button (2x)
    100Ω resistor (1x)
    100kΩ resistor (2x)

  Arduino
    millis    2.4s slower in 300s
*/


#include <SPI.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

extern "C"{
  #include "fonts.h"
}


#define OLED_ADDRESS                  0x3C                            // 0x78 - write to 0x3C
#define BLUETOOTH                                                     // uncomment to enable Bluetooth logging

/* PID Settings */
#define PID_KP                        0.50                            // PID controller's Proportional term
#define PID_KI                        0.05                            // PID controller's Integral term
#define PID_KD                        0.00                            // PID controller's Derivative term


float thermcpl_temp                   = 0.0;                          // thermocouple temperature data
float coldj_temp                      = 0.0;                          // cold-junction temperature data
float deviation_temp                  = 0.0;                          // deviation from profile and actual temperature
unsigned long reflow_start            = 0;

volatile uint8_t pressButton1         = 0;
volatile uint8_t pressButton2         = 0;
volatile uint8_t ButtonDebounce1      = 0;
volatile uint8_t ButtonDebounce2      = 0;
volatile uint8_t chngAllowed          = 1;

volatile uint8_t timer1_match         = 0;
volatile uint8_t clkCounter           = 0;
volatile uint32_t timer2_overflow1    = 0;
volatile uint32_t timer2_overflow2    = 0;
uint32_t buzzerFreqCount              = 0;
uint32_t buzzerDurCount               = 0;

uint8_t solderProfile                 = 0;                            // Pb, Pb-Free, 10%, 20%..
uint8_t solderStatePrev               = 0;
volatile uint8_t solderState          = 0;                            // OFF, WARM-UP, PREHEAT, SOAK, RAMP-UP, REFLOW, COOLING
volatile uint8_t dutyCycle            = 0;
volatile uint8_t dutyCycleAvg         = 0;
volatile uint8_t controllerActive     = 0;
volatile uint8_t PIDcontrol           = 0;
volatile uint16_t processTime         = 0;
volatile uint16_t processTimeOffset   = 0;
volatile uint8_t update_flags         = 0;                            // Bit 0: update Temperature, Bit 1: update Time, Bit 2: Bluetooth, Bit 3: Temperature Deviation
volatile uint8_t state_flags          = 0b00000001;                   // Bit 0: in OFF, Bit6: in COOLING

uint16_t p_warmup                     = 50;                           // [°C] limit temperature for PID to take over
uint8_t p_warmup_dc                   = 10;                           // [%] duty cycle
uint16_t p_preheat_temp               = 150;                          // [°C] preheat ending temperature
uint16_t p_preheat_time               = 60;                           // [s] preheat ending time
uint8_t p_preheat_dc                  = 10;                           // [%] duty cycle
uint16_t p_soak_temp                  = 165;                          // [°C] soak ending temperature
uint16_t p_soak_time                  = 170;                          // [s] soak ending time
uint8_t p_soak_dc                     = 6;                            // [%] duty cycle
uint16_t p_rampup_temp                = 230;                          // [°C] rampup ending temperature
uint16_t p_rampup_time                = 215;                          // [s] rampup ending time
uint8_t p_rampup_dc                   = 10;                           // [%] duty cycle
uint16_t p_reflow_temp                = 230;                          // [°C] reflow ending temperature
uint16_t p_reflow_time                = 235;                          // [s] reflow ending time
uint8_t p_reflow_dc                   = 3;                            // [%] duty cycle
uint16_t p_cooling_temp               = 50;                           // [°C] cooling ending temperature
uint16_t p_cooling_time               = 280;                          // [s] cooling ending time
uint8_t p_cooling_dc                  = 0;                            // [%] duty cycle

uint16_t profile_PID_Pb[]             = {50, 150, 60, 180, 170, 230, 215, 230, 235, 50, 280};     // °C, °C, s, °C, s, °C, s, °C, s, °C, s
uint16_t profile_PID_PbF[]            = {50, 150, 60, 210, 175, 250, 225, 250, 240, 50, 290};     // °C, °C, s, °C, s, °C, s, °C, s, °C, s
uint16_t profile_Tmp_Pb[]             = {50, 10, 150, 10, 180, 6, 225, 10, 225, 3, 50, 0, 15};    // °C, %, °C, %, °C, %, °C, %, °C, %, °C, %, s
uint16_t profile_Tmp_PbF[]            = {50, 10, 150, 10, 210, 7, 245, 10, 245, 4, 50, 0, 15};    // °C, %, °C, %, °C, %, °C, %, °C, %, °C, %, s
uint16_t profile_Tim_Pb[]             = {50, 10, 70, 10, 100, 5, 60, 10, 15, 3, 50, 0};           // °C, %, s, %, s, %, s, %, s, %, °C, %
uint16_t profile_Tim_PbF[]            = {50, 10, 80, 10, 150, 6, 60, 10, 15, 3, 50, 0};           // °C, %, s, %, s, %, s, %, s, %, °C, %

float Kp                              = PID_KP;
float Ki                              = PID_KI;
float Kd                              = PID_KD;
volatile float _error                 = 0.0;
volatile float _errI                  = 0.0;
volatile float _errD                  = 0.0;
volatile float _inptLast              = 0.0;
volatile float _output                = 0.0;


// Setup function ---------------------------------------------------------------------------------
void setup()
{
  pinMode(10, OUTPUT);                                                // CS pin
  digitalWrite(10, HIGH);
  SPI.begin();                                                        // Arduino <- MAX31855

  I2C_init();                                                         // Arduino <-> OLED

#ifdef BLUETOOTH

  Serial.begin(115200);

#endif // BLUETOOTH

  OLED_init();
  OLED_clear();
  OLED_initial_screen();                                              // show initial screen
  OLED_update_profile(solderProfile);
  OLED_update_state(solderState);

  DDRD &= 0b11110011;                                                 // D2 and D3 set as INPUT (Buttons)
  
  DDRD |= 0b10000000;                                                 // D7 set as OUTPUT - SSR (solid state relay)
  PORTD &= 0b01111111;                                                // D7 set LOW

  pinMode(8, OUTPUT);                                                 // D8 LED
  digitalWrite(8, LOW);                                               // D8 set LOW
  
  _delay_ms(200);                                                     // The PmodTC1 requires at least 200 ms to power up.

  Timer1_setup();                                                     // start a background timer
}


// Loop function ----------------------------------------------------------------------------------
void loop()
{
  /* Button 1 Press */
  if(pressButton1 >= 4)                                               // 4 * 25ms = 100ms
  {
    if(chngAllowed)
    {
      Buzzer_alarm(2000, 100);
      
      solderProfile = (solderProfile + 1) % 16;

      if(solderProfile == 0)
      {
        p_warmup        = profile_PID_Pb[0];
        p_preheat_temp  = profile_PID_Pb[1];
        p_preheat_time  = profile_PID_Pb[2];
        p_soak_temp     = profile_PID_Pb[3];
        p_soak_time     = profile_PID_Pb[4];
        p_rampup_temp   = profile_PID_Pb[5];
        p_rampup_time   = profile_PID_Pb[6];
        p_reflow_temp   = profile_PID_Pb[7];
        p_reflow_time   = profile_PID_Pb[8];
        p_cooling_temp  = profile_PID_Pb[9];
        p_cooling_time  = profile_PID_Pb[10];
      }
      else if(solderProfile == 1)
      {
        p_warmup        = profile_PID_PbF[0];
        p_preheat_temp  = profile_PID_PbF[1];
        p_preheat_time  = profile_PID_PbF[2];
        p_soak_temp     = profile_PID_PbF[3];
        p_soak_time     = profile_PID_PbF[4];
        p_rampup_temp   = profile_PID_PbF[5];
        p_rampup_time   = profile_PID_PbF[6];
        p_reflow_temp   = profile_PID_PbF[7];
        p_reflow_time   = profile_PID_PbF[8];
        p_cooling_temp  = profile_PID_PbF[9];
        p_cooling_time  = profile_PID_PbF[10];
      }
      else if(solderProfile == 2)
      {
        p_warmup        = profile_Tmp_Pb[0];
        p_warmup_dc     = profile_Tmp_Pb[1];
        p_preheat_temp  = profile_Tmp_Pb[2];
        p_preheat_dc    = profile_Tmp_Pb[3];
        p_soak_temp     = profile_Tmp_Pb[4];
        p_soak_dc       = profile_Tmp_Pb[5];
        p_rampup_temp   = profile_Tmp_Pb[6];
        p_rampup_dc     = profile_Tmp_Pb[7];
        p_reflow_temp   = profile_Tmp_Pb[8];
        p_reflow_dc     = profile_Tmp_Pb[9];
        p_cooling_temp  = profile_Tmp_Pb[10];
        p_cooling_dc    = profile_Tmp_Pb[11];
        p_reflow_time   = profile_Tmp_Pb[12];
      }
      else if(solderProfile == 3)
      {
        p_warmup        = profile_Tmp_PbF[0];
        p_warmup_dc     = profile_Tmp_PbF[1];
        p_preheat_temp  = profile_Tmp_PbF[2];
        p_preheat_dc    = profile_Tmp_PbF[3];
        p_soak_temp     = profile_Tmp_PbF[4];
        p_soak_dc       = profile_Tmp_PbF[5];
        p_rampup_temp   = profile_Tmp_PbF[6];
        p_rampup_dc     = profile_Tmp_PbF[7];
        p_reflow_temp   = profile_Tmp_PbF[8];
        p_reflow_dc     = profile_Tmp_PbF[9];
        p_cooling_temp  = profile_Tmp_PbF[10];
        p_cooling_dc    = profile_Tmp_PbF[11];
        p_reflow_time   = profile_Tmp_PbF[12];
      }
      else if(solderProfile == 4)
      {
        p_warmup        = profile_Tim_Pb[0];
        p_warmup_dc     = profile_Tim_Pb[1];
        p_preheat_time  = profile_Tim_Pb[2];
        p_preheat_dc    = profile_Tim_Pb[3];
        p_soak_time     = profile_Tim_Pb[4];
        p_soak_dc       = profile_Tim_Pb[5];
        p_rampup_time   = profile_Tim_Pb[6];
        p_rampup_dc     = profile_Tim_Pb[7];
        p_reflow_time   = profile_Tim_Pb[8];
        p_reflow_dc     = profile_Tim_Pb[9];
        p_cooling_temp  = profile_Tim_Pb[10];
        p_cooling_dc    = profile_Tim_Pb[11];
      }
      else if(solderProfile == 5)
      {
        p_warmup        = profile_Tim_PbF[0];
        p_warmup_dc     = profile_Tim_PbF[1];
        p_preheat_time  = profile_Tim_PbF[2];
        p_preheat_dc    = profile_Tim_PbF[3];
        p_soak_time     = profile_Tim_PbF[4];
        p_soak_dc       = profile_Tim_PbF[5];
        p_rampup_time   = profile_Tim_PbF[6];
        p_rampup_dc     = profile_Tim_PbF[7];
        p_reflow_time   = profile_Tim_PbF[8];
        p_reflow_dc     = profile_Tim_PbF[9];
        p_cooling_temp  = profile_Tim_PbF[10];
        p_cooling_dc    = profile_Tim_PbF[11];
      }
      
      OLED_update_profile(solderProfile);
    }

    pressButton1 = 0;
    ButtonDebounce1 = 6;                                              // 6 * 25ms = 150ms
  }

  /* Button 2 Press */
  if(pressButton2 >= 4)                                               // 4 * 25ms = 100ms
  { 
    if(solderState == 0)                                              // from OFF state to WARM-UP
    {
      if(thermcpl_temp < (float)p_warmup)                             // only enable Controller if internal temperature is below limit
      {
        solderState = 1;
        chngAllowed = 0;
  
        Controller_enable();
      }
      else
      {
        Buzzer_alarm(1600, 100);
      }
    }
    else if(solderState >= 1)                                         // from any active state to OFF
    {
      solderState = 0;
      chngAllowed = 1;

      Controller_disable();

      OLED_draw_string_8x16((uint8_t*)"__:__", 72, 3, 5);             // update time
      OLED_draw_string_8x16((uint8_t*)"   0", 8, 3, 4);               // update deviation
    }

    pressButton2 = 0;
    ButtonDebounce2 = 6;                                              // 6 * 25ms = 150ms
  }

  /* Update OLED */
  if(update_flags & 0b00000001)                                       // update Temperature
  {
    update_flags &= 0b11111110;
    OLED_update_temperature();
  }

  if(update_flags & 0b00001000)                                       // update Temperature Deviation
  {
    update_flags &= 0b11110111;
    OLED_update_temp_dev();
  }

  if(update_flags & 0b00000010)                                       // update Time
  {
    update_flags &= 0b11111101;
    OLED_update_time();
  }

#ifdef BLUETOOTH

  if(update_flags & 0b00000100)                                       // send data over Bluetooth
  {
    update_flags &= 0b11111011;

    Serial.print(millis() - reflow_start);
    Serial.print(",");
    Serial.print(thermcpl_temp, 2);
    Serial.print(",");
    Serial.print(coldj_temp, 2);
    Serial.print(",");
    Serial.print(deviation_temp, 2);
    Serial.print(",");
    Serial.print((uint16_t)dutyCycleAvg * 10 / 4);
    Serial.print(",");
    Serial.print(_error, 2);
    Serial.print(",");
    Serial.print(_errI, 2);
    Serial.print(",");
    Serial.print(_errD, 2);
    Serial.print(",");
    Serial.println(_output, 2);
    
    dutyCycleAvg = 0;
  }

#endif // BLUETOOTH

  if(solderState != solderStatePrev)
  {
    solderStatePrev = solderState;

    if(solderState == 1)
    {
      Buzzer_alarm(1800, 100);
      _delay_ms(130);
      Buzzer_alarm(2000, 100);
    }
    else if(solderState == 2 || solderState == 3 || solderState == 4 || solderState == 5)
    {
      Buzzer_alarm(2000, 100);
    }
    else if(solderState == 6)
    {
      Buzzer_alarm(2400, 100);
      _delay_ms(200);
      Buzzer_alarm(2400, 100);
    }
    else if(solderState == 0)
    {
      Buzzer_alarm(2000, 100);
      _delay_ms(130);
      Buzzer_alarm(1800, 100);
    }
    
    OLED_update_state(solderState);
  }
}


// MAX31855 functions -----------------------------------------------------------------------------
/*
  Digilent PmodTC1 cold-junction thermocouple-to-digital converter module.
  Built around MAX31855 IC with a K-type thermocouple.

  Thermocouple (K-type)
    -73°C to 482°C
  PmodTC1 module
    -270°C to 1800°C

  3.3V operating voltage.

  Pinout
    1   CS    active low chip select
    2   N/A   N/A
    3   MISO  serial data out
    4   SCLK  serial clock
    5   GND   power supply ground
    6   VCC   power supply (3.3V)

  SPI clock frequency 5MHz max.
  
  Each temperature conversion takes up 70ms (typical), 100 ms (max) to complete.
  The PmodTC1 requires at least 200 ms to power up.

  Default Arduino ProMini (8MHz) SPI clock is 2MHz.
    One temperature transfer takes: 1 / 2000000 * 32 = 16us.
*/


/*
  error_status
    Bit 3   Fault
    Bit 2   Short to VCC
    Bit 1   Short to GND
    Bit 0   Open Circuit
*/
uint8_t MAX31855_temperature_read(void)
{
  uint32_t data = 0;
  int16_t therm = 0;
  int16_t coldj = 0;
  uint8_t error_status = 0;

  digitalWrite(10, LOW);
  data = (uint32_t)SPI.transfer(0x00) << 24;
  data |= (uint32_t)SPI.transfer(0x00) << 16;
  data |= (uint32_t)SPI.transfer(0x00) << 8;
  data |= (uint32_t)SPI.transfer(0x00) << 0;
  digitalWrite(10, HIGH);

  // 14-bit value (must add bits 15 and 14 in negative numbers)
  if(data & 0x80000000) therm = (int16_t)((data & 0xFFFC0000) >> 18) | 0xC000;
  else therm = (int16_t)((data & 0xFFFC0000) >> 18);

  // 12-bit value (must add bits 15, 14, 13 and 12 in negative numbers)
  if(data & 0x8000) coldj = (int16_t)((data & 0xFFF0) >> 4) | 0xF000;
  else coldj = (int16_t)((data & 0xFFF0) >> 4);
  
  thermcpl_temp = (float)therm / 4.0;
  coldj_temp = (float)coldj / 16.0;

  error_status = ((data & 0x10000) >> 13) | (data & 0x03);

  return error_status;
}


// Buzzer functions -------------------------------------------------------------------------------
/*
  LD-BZEN-1205 buzzer.
    operating voltage     3-8V
    rated current         40mA
    sound output (10cm)   85dB
    coil resistance       47Ω
    resonant frequency    2400Hz

  Timer0 used by Arduino's core (delay(), millis(), micros()) - pins D5 and D6.
  Timer1 16-bit used by Servo library - pins D9 and D10.
  Timer2 8-bit used by Tone library - pins D3 and D11.
*/


/*
  Buzzer located on Arduino pin 9 (PB1).
  Timer2 is used to fire an interrupt which oversees the proper waveform generation and duration.
  Maximum 1s, minimum 1ms buzz.
*/
void Buzzer_alarm(uint32_t freq, uint32_t duration)
{
  if(freq < 1) freq = 1;
  buzzerFreqCount = 15625 / freq;
  if(buzzerFreqCount < 1) buzzerFreqCount = 1;

  if(duration < 1) duration = 1;
  buzzerDurCount = 31250 / (1000 / duration);
  if(buzzerDurCount < 1) buzzerDurCount = 1;
  
  timer2_overflow1 = 0;
  timer2_overflow2 = 0;
  
  DDRB |= 0b00000010;                                                 // configure PB1 as output
  PORTB &= 0b11111101;                                                // set PB1 low

  TCCR2B = (1 << CS20);                                               // CLK=8MHz no prescaling
  TCNT2 = 0;                                                          // initialize counter
  TIMSK2 |= (1 << TOIE2);                                             // enable overflow interrupt
  sei();                                                              // enable global interrupts
}


/*
  Timer2 overflow interrupt service routine is called when TCNT2 overflows.
*/
ISR(TIMER2_OVF_vect)
{
  timer2_overflow1++;                                                 // every: 1 / 8000000 * 256 = 32us (31.250kHz)
  timer2_overflow2++;

  if(timer2_overflow1 >= buzzerFreqCount)
  {
    PORTB ^= 0b00000010;                                              // toggle PB1
    timer2_overflow1 = 0;
  }

  if(timer2_overflow2 >= buzzerDurCount)
  {
    PORTB &= 0b11111101;                                              // set PB1 low
    TIMSK2 &= ~(1 << TOIE2);                                          // disable overflow interrupt
    TCCR2B = 0;                                                       // stop Timer2
  }
}


// CONTROLLER functions ---------------------------------------------------------------------------
/*
  Timer0 used by Arduino's core (delay(), millis(), micros()) - pins D5 and D6.
  Timer1 16-bit used by Servo library - pins D9 and D10.
  Timer2 8-bit used by Tone library - pins D3 and D11.
*/


/*
  Timer1 in CTC Mode running in the background providing 1s time pulse. When active, managing the PID controller.
*/
void Timer1_setup(void)
{
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);                  // 8MHz/64 prescling, CLK=125kHz, CTC mode
  TCCR1A = 0;                                                         // CTC mode (OCR1A top)
  OCR1A = 3125;                                                       // output compare match every 25ms  
  TCNT1 = 0;                                                          // initialize counter
  TIMSK1 |= (1 << OCIE1A);                                            // enable output compare A match interrupt
  sei();
}


/*
  
*/
void Controller_enable(void)
{
  PIDcontrol = 0;
  _errI = 0.0;                                                        // reset PID's integral
  _inptLast = 0.0;                                                    // reset PID's last input

  deviation_temp = 0;
  update_flags |= 0b00001000;                                         // update Temperature Deviation
  state_flags = 0b00000001;                                           // in OFF state
  
  TCNT1 = 0;                                                          // reset counter
  reflow_start = millis();
  clkCounter = 39;
  timer1_match = 9;
  processTime = 65535;
  processTimeOffset = 0;
  controllerActive = 1;
  TCNT1 = 3124;                                                       // force first Interrupt immediately
}


/*

*/
void Controller_disable(void)
{
  controllerActive = 0;
  
  PORTD &= 0b01111111;                                                // D7 set LOW (SSR)
  digitalWrite(8, LOW);                                               // D8 set LOW
}


/*
  Timer/Counter1 Compare Match A interrupt service routine is called when TCNT1 matches OCR1A.
*/
ISR(TIMER1_COMPA_vect)
{
  TCNT1 = 0;                                                          // reset counter
  clkCounter = (clkCounter + 1) % 40;                                 // every: 25ms until 1s
  timer1_match = (timer1_match + 1) % 10;                             // every: 25ms until 250ms, values from 0 to 9

  /* Button press */
  uint8_t buttons = PIND;

  if(ButtonDebounce1)
  {
    ButtonDebounce1--;
  }
  else
  {
    if(buttons & 0b00000100) pressButton1++;
    else pressButton1 = 0;
  }

  if(ButtonDebounce2)
  {
    ButtonDebounce2--;
  }
  else
  {
    if(buttons & 0b00001000) pressButton2++;
    else pressButton2 = 0;
  }

  /* 1s Time Pulse */
  if(clkCounter == 0)
  {
    if(controllerActive == 1)
    {
      update_flags |= 0b00000010;                                     // update Time
    }
    
    if(controllerActive == 0)
    {
      MAX31855_temperature_read();
      update_flags |= 0b00000001;                                     // update Temperature
    }

    update_flags |= 0b00000100;                                       // send data over Bluetooth
  }

  /* Temperature Measurement and PID evaluation */
  if(controllerActive == 1 && timer1_match == 0)
  {
    processTime = processTime + 1;                                    // keeps track of number of 250ms time segments
    
    MAX31855_temperature_read();
    update_flags |= 0b00000001;                                       // update Temperature

    if(solderProfile >= 6)                                            // Duty Cycle based control
    {
      dutyCycle = solderProfile - 5;
      dutyCycleAvg += dutyCycle;
    }
    else if(solderProfile == 2 || solderProfile == 3)                 // Temperature based control (no time dependence - almost)
    {
      if(state_flags == 0b00000001 || state_flags == 0b00000010)
      {
        if(thermcpl_temp >= (float)p_warmup)
        {
          solderState = 2;
          state_flags = 0b00000100;
        }
        else
        {
          solderState = 1;
          state_flags = 0b00000010;
        }
      }
      else if(state_flags == 0b00000100)
      {
        if(thermcpl_temp >= (float)p_preheat_temp)
        {
          solderState = 3;
          state_flags = 0b00001000;
        }
        else
        {
          solderState = 2;
          state_flags = 0b00000100;
        }
      }
      else if(state_flags == 0b00001000)
      {
        if(thermcpl_temp >= (float)p_soak_temp)
        {
          solderState = 4;
          state_flags = 0b00010000;
        }
        else
        {
          solderState = 3;
          state_flags = 0b00001000;
        }
      }
      else if(state_flags == 0b00010000)
      {
        if(thermcpl_temp >= (float)p_rampup_temp)
        {
          solderState = 5;
          state_flags = 0b00100000;
        }
        else
        {
          solderState = 4;
          state_flags = 0b00010000;
        }
        
        processTimeOffset = processTime;
      }
      else if(state_flags == 0b00100000)
      {
        if(((float)(processTime - processTimeOffset) / 4.0) >= (float)p_reflow_time)
        {
          solderState = 6;
          state_flags = 0b01000000;
        }
        else
        {
          solderState = 5;
          state_flags = 0b00100000;
        }
      }
      else if(state_flags == 0b01000000)
      {
        if(thermcpl_temp < (float)p_warmup)
        {
          solderState = 0;
          state_flags = 0b00000001;
        }
        else
        {
          solderState = 6;
          state_flags = 0b01000000;
        }
      }
      
      if(solderState == 1) dutyCycle = p_warmup_dc;                   // in WARM-UP
      else if(solderState == 2) dutyCycle = p_preheat_dc;             // in PREHEAT
      else if(solderState == 3) dutyCycle = p_soak_dc;                // in SOAK
      else if(solderState == 4) dutyCycle = p_rampup_dc;              // in RAMP-UP
      else if(solderState == 5) dutyCycle = p_reflow_dc;              // in REFLOW
      else if(solderState == 6) dutyCycle = p_cooling_dc;             // in COOLING
      else if(solderState == 0)
      {
        dutyCycle = 0;                                                // in OFF
        controllerActive = 0;
        chngAllowed = 1;
        
        PORTD &= 0b01111111;                                          // D7 set LOW (SSR)
        digitalWrite(8, LOW);                                         // D8 set LOW
      }
      
      dutyCycleAvg += dutyCycle;
    }
    else if(solderProfile == 4 || solderProfile == 5)                 // Time based control (no temperature dependence - almost)
    {
      if(state_flags == 0b00000001 || state_flags == 0b00000010)
      {
        if(thermcpl_temp >= (float)p_warmup)
        {
          solderState = 2;
          state_flags = 0b00000100;
        }
        else
        {
          solderState = 1;
          state_flags = 0b00000010;
        }

        processTimeOffset = processTime;
      }
      else if(state_flags == 0b00000100)
      {
        if(((float)(processTime - processTimeOffset) / 4.0) >= (float)p_preheat_time)
        {
          solderState = 3;
          state_flags = 0b00001000;
        }
        else
        {
          solderState = 2;
          state_flags = 0b00000100;
        }
      }
      else if(state_flags == 0b00001000)
      {
        if(((float)(processTime - processTimeOffset) / 4.0) >= ((float)p_soak_time + (float)p_preheat_time))
        {
          solderState = 4;
          state_flags = 0b00010000;
        }
        else
        {
          solderState = 3;
          state_flags = 0b00001000;
        }
      }
      else if(state_flags == 0b00010000)
      {
        if(((float)(processTime - processTimeOffset) / 4.0) >= ((float)p_rampup_time + (float)p_soak_time + (float)p_preheat_time))
        {
          solderState = 5;
          state_flags = 0b00100000;
        }
        else
        {
          solderState = 4;
          state_flags = 0b00010000;
        }
      }
      else if(state_flags == 0b00100000)
      {
        if(((float)(processTime - processTimeOffset) / 4.0) >= ((float)p_reflow_time + (float)p_rampup_time + (float)p_soak_time + (float)p_preheat_time))
        {
          solderState = 6;
          state_flags = 0b01000000;
        }
        else
        {
          solderState = 5;
          state_flags = 0b00100000;
        }
      }
      else if(state_flags == 0b01000000)
      {
        if(thermcpl_temp < (float)p_warmup)
        {
          solderState = 0;
          state_flags = 0b00000001;
        }
        else
        {
          solderState = 6;
          state_flags = 0b01000000;
        }
      }
      
      if(solderState == 1) dutyCycle = p_warmup_dc;                   // in WARM-UP
      else if(solderState == 2) dutyCycle = p_preheat_dc;             // in PREHEAT
      else if(solderState == 3) dutyCycle = p_soak_dc;                // in SOAK
      else if(solderState == 4) dutyCycle = p_rampup_dc;              // in RAMP-UP
      else if(solderState == 5) dutyCycle = p_reflow_dc;              // in REFLOW
      else if(solderState == 6) dutyCycle = p_cooling_dc;             // in COOLING
      else if(solderState == 0)
      {
        dutyCycle = 0;                                                // in OFF
        controllerActive = 0;
        chngAllowed = 1;
        
        PORTD &= 0b01111111;                                          // D7 set LOW (SSR)
        digitalWrite(8, LOW);                                         // D8 set LOW
      }
      
      dutyCycleAvg += dutyCycle;
    }
    else                                                              // Temperature Profile PID control
    {
      float _procTime = (float)(processTime - processTimeOffset) / 4.0;
      float _setpoint = 0.0;
      
      if(thermcpl_temp >= (float)p_warmup) PIDcontrol = 1;
      
      if(!PIDcontrol)                                                 // in WARM-UP
      {
        solderState = 1;
        processTimeOffset = processTime;
        _setpoint = (float)p_warmup;
      }
      else if(_procTime < (float)p_preheat_time)                      // in PREHEAT
      {
        solderState = 2;

        _setpoint = _procTime / (float)p_preheat_time * ((float)p_preheat_temp - (float)p_warmup) + (float)p_warmup;
      }
      else if(_procTime < (float)p_soak_time)                         // in SOAK
      {
        solderState = 3;

        _setpoint = (_procTime - (float)p_preheat_time) / ((float)p_soak_time - (float)p_preheat_time) * ((float)p_soak_temp - (float)p_preheat_temp) + (float)p_preheat_temp;
      }
      else if(_procTime < (float)p_rampup_time)                       // in RAMP-UP
      {
        solderState = 4;

        _setpoint = (_procTime - (float)p_soak_time) / ((float)p_rampup_time - (float)p_soak_time) * ((float)p_rampup_temp - (float)p_soak_temp) + (float)p_soak_temp;
      }
      else if(_procTime < (float)p_reflow_time)                       // in REFLOW
      {
        solderState = 5;

        _setpoint = (_procTime - (float)p_rampup_time) / ((float)p_reflow_time - (float)p_rampup_time) * ((float)p_reflow_temp - (float)p_rampup_temp) + (float)p_rampup_temp;
      }
      else if(_procTime < (float)p_cooling_time)                      // in COOLING
      {
        solderState = 6;

        _setpoint = (_procTime - (float)p_reflow_time) / ((float)p_cooling_time - (float)p_reflow_time) * ((float)p_cooling_temp - (float)p_reflow_temp) + (float)p_reflow_temp;
      }
      else                                                            // FINISHED - in OFF
      {
        solderState = 0;
        controllerActive = 0;
        chngAllowed = 1;
        
        PORTD &= 0b01111111;                                          // D7 set LOW (SSR)
        digitalWrite(8, LOW);                                         // D8 set LOW
        
        _setpoint = (float)p_cooling_temp;
      }

      if(solderState == 1)                                            // WARM-UP
      {
        dutyCycle = 10;
        dutyCycleAvg += dutyCycle;
      }
      else if(solderState == 6)                                       // COOLING
      {
        dutyCycle = 0;
        dutyCycleAvg += dutyCycle;
      }
      else                                                            // PID control
      {
        _error = _setpoint - thermcpl_temp;                           // proportional part
        _errI += _error * Ki;                                         // integral part
        _errD = thermcpl_temp - _inptLast;                            // derivative part

        if(_errI > 10.0) _errI = 10.0;                                // limits in case of integral windup
        else if(_errI < 0.0) _errI = 0.0;
        
        _output = Kp * _error + _errI - Kd * _errD;                   // PID output
        
        if(_output > 10.0) _output = 10.0;                            // limits in case of integral windup
        else if(_output < 0.0) _output = 0.0;
        
        _inptLast = thermcpl_temp;                                    // remember value

        dutyCycle = (uint8_t)roundf(_output);
        dutyCycleAvg += dutyCycle;
      }
      
      deviation_temp = thermcpl_temp - _setpoint;
      update_flags |= 0b00001000;                                     // update Temperature Deviation
    }
  }
  
  /* Duty Cycle */
  if(controllerActive == 1)
  {
    if(timer1_match < dutyCycle)                                      // heating elements on
    {
      PORTD |= 0b10000000;                                            // D7 set HIGH (SSR)
      digitalWrite(8, HIGH);                                          // D8 set HIGH
    }
    else                                                              // heating elements off
    {
      PORTD &= 0b01111111;                                            // D7 set LOW (SSR)
      digitalWrite(8, LOW);                                           // D8 set LOW
    }
  }
}


// I2C functions ----------------------------------------------------------------------------------
/*
  I2C interface between Arduino and OLED display. 
*/


/*
  
*/
void I2C_init(void)
{
  TWSR = 0x00;                                                        // set the prescaler value to 1
  TWBR = 0x48;                                                        // set the division factor for 100kHz clock signal (0x48 -> 16000000/(16+2*72*1)=100000)
  TWCR = (1<<TWEN);                                                   // I2C enable
}


/*

*/
void I2C_start(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);                             // TWINT - clearing the 'job finished' flag, TWSTA - if the bus is clear become Master, TWEN - I2C enable
  while ((TWCR & (1<<TWINT)) == 0);                                   // waiting for the 'job finished' flag
}


/*

*/
void I2C_stop(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);                             // TWSTO - generate a Stop condition
}


/*

*/
void I2C_write_byte(uint8_t u8data)
{
  TWDR = u8data;                                                      // fill the Data Register
  TWCR = (1<<TWINT)|(1<<TWEN);                                        // TWINT - clearing the 'job finished' flag, TWEN - I2C enable
  while ((TWCR & (1<<TWINT)) == 0);                                   // waiting for the 'job finished' flag
}


// OLED functions ---------------------------------------------------------------------------------
/*
  OLED display 128x64 pixels.
*/


/*
  keywords:
    SEG (segment) = COL (column) = byte of data (bits represent 8 rows within the column)
    COM = row
    Page = 8 rows of pixels of 128 columns
    Display = 8 pages
*/
void OLED_init(void)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0xAE);                                               // DISPLAY_OFF
  I2C_write_byte(0xA8);                                               // SET_MUX_RATIO
  I2C_write_byte(0x3F);
  I2C_write_byte(0xD3);                                               // SET_DISPLAY_OFFSET
  I2C_write_byte(0x00);
  I2C_write_byte(0x40);                                               // SET_DISPLAY_START_LINE
  I2C_write_byte(0xA1);                                               // SET_SEGMENT_REMAP
  I2C_write_byte(0xC8);                                               // SET_COM_SCAN_MODE
  I2C_write_byte(0xDA);                                               // SET_COM_PIN_MAP
  I2C_write_byte(0x12);
  I2C_write_byte(0x81);                                               // SET_CONTRAST
  I2C_write_byte(0x7F);
  I2C_write_byte(0xA4);                                               // DISPLAY_RAM
  I2C_write_byte(0xA6);                                               // DISPLAY NORMAL
  I2C_write_byte(0xD5);                                               // SET_DISPLAY_CLK_DIV
  I2C_write_byte(0x80);
  I2C_write_byte(0x8D);                                               // SET_CHARGE_PUMP
  I2C_write_byte(0x14);
  I2C_write_byte(0xD9);                                               // SET_PRECHARGE
  I2C_write_byte(0x22);
  I2C_write_byte(0xDB);                                               // SET_VCOMH_DESELECT
  I2C_write_byte(0x30);
  I2C_write_byte(0x20);                                               // SET_MEMORY_ADDR_MODE
  I2C_write_byte(0x00);
  I2C_write_byte(0xAF);                                               // DISPLAY_ON
  I2C_stop();
}


/*

*/
void OLED_draw_string_5x7(uint8_t * string, uint8_t column, uint8_t page, uint16_t stringsize)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *string;
    uint16_t x = (uint16_t)c * 5;
    
    for(uint8_t y = 0; y < 5; y++)
    {
      I2C_write_byte(pgm_read_byte(&font5x7[x]));
      x++;
    }
    
    string++;
  }
  
  I2C_stop();
}


/*

*/
void OLED_draw_string_6x8(uint8_t * string, uint8_t column, uint8_t page, uint16_t stringsize)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *string - 0x20;
    
    for(uint8_t y = 0; y < 6; y++)
    {
      I2C_write_byte(pgm_read_byte(&font6x8[c][y]));
    }
    
    string++;
  }
  
  I2C_stop();
}


/*

*/
void OLED_draw_string_8x16(uint8_t * string, uint8_t column, uint8_t page, uint16_t stringsize)
{
  uint8_t * stringf = string;
  
  // First Row
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *stringf - 0x20;
    uint16_t x = (uint16_t)c * 16;
    
    for(uint8_t y = 0; y < 8; y++)
    {
      I2C_write_byte(pgm_read_byte(&font8x16[x]));
      x++;
    }
    
    stringf++;
  }
  
  I2C_stop();

  // Second Row
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page+1);
  I2C_write_byte(0x07);
  I2C_stop();

  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *string - 0x20;
    uint16_t x = (uint16_t)c * 16 + 8;
    
    for(uint8_t y = 0; y < 8; y++)
    {
      I2C_write_byte(pgm_read_byte(&font8x16[x]));
      x++;
    }
    
    string++;
  }
  
  I2C_stop();
}


/*

*/
void OLED_clear(void)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(0x00);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(0x00);
  I2C_write_byte(0x07);
  I2C_stop();

  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint16_t i = 0; i < 1024; i++)
  {
    I2C_write_byte(0x00);
  }
  
  I2C_stop();
}


/*
  Draws initial screen arrangement.
*/
void OLED_initial_screen(void)
{
  OLED_draw_string_8x16((uint8_t*)"   0", 8, 0, 4);
  OLED_draw_string_8x16((uint8_t*)"^C", 40, 0, 2);
  OLED_draw_string_8x16((uint8_t*)"   0", 64, 0, 4);
  OLED_draw_string_8x16((uint8_t*)"^C", 96, 0, 2);

  OLED_draw_string_8x16((uint8_t*)"   0", 8, 3, 4);
  OLED_draw_string_8x16((uint8_t*)"^C", 40, 3, 2);
  OLED_draw_string_8x16((uint8_t*)"__:__", 72, 3, 5);
  
  OLED_draw_string_6x8((uint8_t*)"profile:", 2, 6, 8);
  OLED_draw_string_6x8((uint8_t*)"       ", 2, 7, 7);
  
  OLED_draw_string_8x16((uint8_t*)"       ", 64, 6, 7);
}


/*
  Draws soldering profile selection.
*/
void OLED_update_profile(uint8_t profile)
{
  switch(profile)
  {
    case 0:
      OLED_draw_string_6x8((uint8_t*)"PID Pb ", 2, 7, 7);
      break;
    case 1:
      OLED_draw_string_6x8((uint8_t*)"PID PbF", 2, 7, 7);
      break;
    case 2:
      OLED_draw_string_6x8((uint8_t*)"Tmp Pb ", 2, 7, 7);
      break;
    case 3:
      OLED_draw_string_6x8((uint8_t*)"Tmp PbF", 2, 7, 7);
      break;
    case 4:
      OLED_draw_string_6x8((uint8_t*)"Tim Pb ", 2, 7, 7);
      break;
    case 5:
      OLED_draw_string_6x8((uint8_t*)"Tim PbF", 2, 7, 7);
      break;
    case 6:
      OLED_draw_string_6x8((uint8_t*)"  10%  ", 2, 7, 7);
      break;
    case 7:
      OLED_draw_string_6x8((uint8_t*)"  20%  ", 2, 7, 7);
      break;
    case 8:
      OLED_draw_string_6x8((uint8_t*)"  30%  ", 2, 7, 7);
      break;
    case 9:
      OLED_draw_string_6x8((uint8_t*)"  40%  ", 2, 7, 7);
      break;
    case 10:
      OLED_draw_string_6x8((uint8_t*)"  50%  ", 2, 7, 7);
      break;
    case 11:
      OLED_draw_string_6x8((uint8_t*)"  60%  ", 2, 7, 7);
      break;
    case 12:
      OLED_draw_string_6x8((uint8_t*)"  70%  ", 2, 7, 7);
      break;
    case 13:
      OLED_draw_string_6x8((uint8_t*)"  80%  ", 2, 7, 7);
      break;
    case 14:
      OLED_draw_string_6x8((uint8_t*)"  90%  ", 2, 7, 7);
      break;
    case 15:
      OLED_draw_string_6x8((uint8_t*)" 100%  ", 2, 7, 7);
      break;
  }
}


/*
  Draws current thermocouple and cold-junction temperatures.
*/
void OLED_update_temperature(void)
{
  uint16_t _thermcpl, _coldjnct;
  uint8_t _thermcplA[4], _coldjnctA[4];

  if(thermcpl_temp < 0.0) _thermcpl = 65535 - (uint16_t)thermcpl_temp & 0x7FFF;
  else _thermcpl = (uint16_t)thermcpl_temp;

  if(coldj_temp < 0.0) _coldjnct = 65535 - (uint16_t)coldj_temp & 0x7FFF;
  else _coldjnct = (uint16_t)coldj_temp;

  if(_thermcpl >= 100) _thermcplA[1] = _thermcpl / 100 % 10 + '0';
  else _thermcplA[1] = ' ';
  if(_thermcpl >= 10) _thermcplA[2] = _thermcpl / 10 % 10 + '0';
  else _thermcplA[2] = ' ';
  _thermcplA[3] = _thermcpl % 10 + '0';

  if(_coldjnct >= 100) _coldjnctA[1] = _coldjnct / 100 % 10 + '0';
  else _coldjnctA[1] = ' ';
  if(_coldjnct >= 10) _coldjnctA[2] = _coldjnct / 10 % 10 + '0';
  else _coldjnctA[2] = ' ';
  _coldjnctA[3] = _coldjnct % 10 + '0';

  if(thermcpl_temp < 0.0)
  {
    if(_thermcpl >= 100) _thermcplA[0] = '-';
    else if(_thermcpl >= 10) {_thermcplA[0] = ' '; _thermcplA[1] = '-';}
    else {_thermcplA[0] = ' '; _thermcplA[1] = ' '; _thermcplA[2] = '-';}
  }
  else
  {
    if(_thermcpl >= 100) _thermcplA[0] = ' ';
    else if(_thermcpl >= 10) {_thermcplA[0] = ' '; _thermcplA[1] = ' ';}
    else {_thermcplA[0] = ' '; _thermcplA[1] = ' '; _thermcplA[2] = ' ';}
  }

  if(coldj_temp < 0.0)
  {
    if(_coldjnct >= 100) _coldjnctA[0] = '-';
    else if(_coldjnct >= 10) {_coldjnctA[0] = ' '; _coldjnctA[1] = '-';}
    else {_coldjnctA[0] = ' '; _coldjnctA[1] = ' '; _coldjnctA[2] = '-';}
  }
  else
  {
    if(_coldjnct >= 100) _coldjnctA[0] = ' ';
    else if(_coldjnct >= 10) {_coldjnctA[0] = ' '; _coldjnctA[1] = ' ';}
    else {_coldjnctA[0] = ' '; _coldjnctA[1] = ' '; _coldjnctA[2] = ' ';}
  }

  OLED_draw_string_8x16(_thermcplA, 8, 0, 4);
  OLED_draw_string_8x16(_coldjnctA, 64, 0, 4);
}


/*
  Draws current state of soldering process.
*/
void OLED_update_state(uint8_t state)
{
  switch(state)
  {
    case 0:
      OLED_draw_string_8x16((uint8_t*)"  OFF  ", 64, 6, 7);
      break;
    case 1:
      OLED_draw_string_8x16((uint8_t*)"WARM-UP", 64, 6, 7);
      break;
    case 2:
      OLED_draw_string_8x16((uint8_t*)"PREHEAT", 64, 6, 7);
      break;
    case 3:
      OLED_draw_string_8x16((uint8_t*)" SOAK  ", 64, 6, 7);
      break;
    case 4:
      OLED_draw_string_8x16((uint8_t*)"RAMP-UP", 64, 6, 7);
      break;
    case 5:
      OLED_draw_string_8x16((uint8_t*)"REFLOW ", 64, 6, 7);
      break;
    case 6:
      OLED_draw_string_8x16((uint8_t*)"COOLING", 64, 6, 7);
      break;
  }
}


/*
  Draws current reflow time.
*/
void OLED_update_time(void)
{
  uint8_t _time[5];

  unsigned long _elapsed = (millis() - reflow_start);
  uint16_t _seconds = _elapsed / 1000;

  uint8_t _min = (uint8_t)(_seconds / 60);
  uint8_t _sec = (uint8_t)(_seconds % 60);

  _time[0] = _min / 10 % 10 + '0';
  _time[1] = _min % 10 + '0';
  _time[2] = ':';
  _time[3] = _sec / 10 % 10 + '0';
  _time[4] = _sec % 10 + '0';
  
  OLED_draw_string_8x16(_time, 72, 3, 5);
}


/*
  Draws deviation between profile and actual temperature.
*/
void OLED_update_temp_dev(void)
{
  uint16_t _dev;
  uint8_t _devA[4];
  float _devTemp = deviation_temp;

  if(_devTemp >= -0.5 && _devTemp < 0.5)
  {
    _devA[0] = ' ';
    _devA[1] = ' ';
    _devA[2] = ' ';
    _devA[3] = '0';
  }
  else if(_devTemp > -1.0 && _devTemp < -0.5)
  {
    _devA[0] = ' ';
    _devA[1] = ' ';
    _devA[2] = '-';
    _devA[3] = '1';
  }
  else if(_devTemp >= 0.5 && _devTemp < 1.0)
  {
    _devA[0] = ' ';
    _devA[1] = ' ';
    _devA[2] = ' ';
    _devA[3] = '1';
  }
  else
  {
    if(_devTemp < 0.0) _dev = 65535 - (uint16_t)_devTemp & 0x7FFF;
    else _dev = (uint16_t)_devTemp;
  
    if(_dev >= 100) _devA[1] = _dev / 100 % 10 + '0';
    else _devA[1] = ' ';
    if(_dev >= 10) _devA[2] = _dev / 10 % 10 + '0';
    else _devA[2] = ' ';
    _devA[3] = _dev % 10 + '0';
    
    if(_devTemp < 0.0)
    {
      if(_dev >= 100) _devA[0] = '-';
      else if(_dev >= 10) {_devA[0] = ' '; _devA[1] = '-';}
      else {_devA[0] = ' '; _devA[1] = ' '; _devA[2] = '-';}
    }
    else
    {
      if(_dev >= 100) _devA[0] = ' ';
      else if(_dev >= 10) {_devA[0] = ' '; _devA[1] = ' ';}
      else {_devA[0] = ' '; _devA[1] = ' '; _devA[2] = ' ';}
    }
  }
  
  OLED_draw_string_8x16(_devA, 8, 3, 4);
}








