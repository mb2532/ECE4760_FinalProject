/*
 * File:        Hexapod Robot
 * Authors:      Joseph Primmer, Kenny Mao, 
 *               Megan Baukus, Bruce Land et. al.
 * Target PIC:  PIC32MX250F128D
 */
#include "config_1_2_3.h"
// need for rand function
#include <stdlib.h>

//Macros to map pwm functionality onto specific pins
#define map_pwmB(index, bit, point) \
        if(point[conf_i][index] == -1) mPORTBClearBits(bit); \
        else if(count_val < point[conf_i][index]) mPORTBSetBits(bit); \
        else if(count_val >= point[conf_i][index]) mPORTBClearBits(bit);
#define map_pwmC(index, bit, point) \
        if(point[conf_i][index] == -1) mPORTCClearBits(bit); \
        if(count_val < point[conf_i][index]) mPORTCSetBits(bit); \
        else if(count_val >= point[conf_i][index]) mPORTCClearBits(bit);

#define NOP asm("nop");
// wait 5 is 125 nS
#define wait125  NOP;NOP;NOP;NOP;NOP;
// wait10 is 250 nS
#define wait250  NOP;NOP;NOP;NOP;NOP; NOP;NOP;NOP;NOP;NOP;
// zero bit on time
#define wait0on wait250; NOP;//NOP; //NOP;NOP;
// one bit on time
#define wait1on wait250; wait250; wait125 //NOP;//NOP;NOP;
// one bit off time
#define wait1off wait250; wait125;  //NOP; //NOP;NOP;NOP;
// zero bit off time
#define wait0off wait250; wait125; NOP;NOP;NOP;NOP;NOP;
// bit_test macro
// returns NON-ZERO if TRUE
#define bit_test(v,bit_num) ((v)&(1<<(bit_num)))

// === select pixel colors ===================================
// number of pixels
#define NeoNum 64

//These are PWM parameters used within the ISR
const unsigned int count_high = 800;
volatile unsigned int count_val = 0;
volatile unsigned int pwm_value = 0;
volatile int conf_i;

//These giant array chunks represent leg movement patterns for the robot
//pwm values go from 40 to 80, any other values and the servos might go
// out of range (try at own risk), 40 is full counter clockwise, 80 is 
// full clockwise. PWM value can also be -1, this makes the servo 
// go slack and conserves power (wattage)
                  //B2  B3  C0  C1  C2  B4  C3  C4  C5  B5  B10 B11
char neutral[12] = {67, 80, 56, 64, 55, 65, 52, 57, 48, 56, 62, 40}; 
char walk1[12] =   { 0,  0,  0,  0,  0,-40,  0,  0,  0,  0, 40,  0};
char walk2[12] =   { 0,  0,  0,  0,-11,-40,  0,  0,  0,  0, 40, 10};
char walk3[12] =   { 0,  0,  0,  0,-11,  0,  0,  0,  0,  0,  0, 10};
char walk4[12] =   {-40, 0,  0,  0,-11,  0, 40,  0,  0,  0,  0, 10};
char walk5[12] =   {-40,-11, 0,  0,-11,  0, 40, 10,  0,  0,  0, 10};
char walk6[12] =   {  0,-11, 0,  0,-11,  0,  0, 10,  0,  0,  0, 10};
char walk7[12] =   {  0,-11, 0,-40,-11,  0,  0, 10, 40,  0,  0, 10};
char walk8[12] =   {  0,-11,-11,-40,-11, 0,  0, 10, 40, 10,  0, 10};
char walk9[12] =   {  0,-11,-11,  0,-11, 0,  0, 10,  0, 10,  0, 10};
char walk10[12] =  {  0,-11,-11,  0,-11, 0,  0, 10,  0, 10,  0, 10};

                    //B2  B3  C0  C1  C2  B4  C3  C4  C5  B5  B10 B11
char right1[12] =   {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,-40,  0};
char right2[12] =   {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,-40,-10};
char right3[12] =   {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,-10};
char right4[12] =   {  0,  0,  0,  0,  0,  0,-40,  0,  0,  0,  0,-10};
char right5[12] =   {  0,  0,  0,  0,  0,  0,-40,-10,  0,  0,  0,-10};
char right6[12] =   {  0,  0,  0,  0,  0,  0,  0,-10,  0,  0,  0,-10};
char right7[12] =   {  0,  0,  0,  0,  0,  0,  0,-10,-40,  0,  0,-10};
char right8[12] =   {  0,  0,  0,  0,  0,  0,  0,-10,-40,-10,  0,-10};
char right9[12] =   {  0,  0,  0,  0,  0,  0,  0,-10,  0,-10,  0,-10};
char right10[12] =  {  0,  0,  0,  0,  0,  0,  0,-10,  0,-10,  0,-10};

                   //B2  B3  C0  C1  C2  B4| C3  C4  C5  B5  B10 B11
char left1[12] =   { 0,  0,  0,  0,  0, 40,  0,  0,  0,  0,  0,  0};
char left2[12] =   { 0,  0,  0,  0, 11, 40,  0,  0,  0,  0,  0,  0};
char left3[12] =   { 0,  0,  0,  0, 11,  0,  0,  0,  0,  0,  0,  0};
char left4[12] =   { 40, 0,  0,  0, 11,  0,  0,  0,  0,  0,  0,  0};
char left5[12] =   { 40, 11, 0,  0, 11,  0,  0,  0,  0,  0,  0,  0};
char left6[12] =   {  0, 11, 0,  0, 11,  0,  0,  0,  0,  0,  0,  0};
char left7[12] =   {  0, 11, 0, 40, 11,  0,  0,  0,  0,  0,  0,  0};
char left8[12] =   {  0, 11, 11, 40, 11, 0,  0,  0,  0,  0,  0,  0};
char left9[12] =   {  0, 11, 11,  0, 11, 0,  0,  0,  0,  0,  0,  0};
char left10[12] =  {  0, 11, 11,  0, 11, 0,  0,  0,  0,  0,  0,  0};

char conf_len = 11;

//Robot neopixel face configurations
char hface[] = {9,10,18,17, 13,14,22,21, 50,51,52,53, 41,46};
int hface_len = 14; 
char nface[] = {9,10,18,17, 13,14,22,21, 50-8,51-8,52-8,53-8, 41};
int nface_len = 13; 
char sface[] = {9,18, 14,21, 50-8,51-8,52-8,53-8, 57-8,62-8};
int sface_len = 10; 

//Arrays representing leg movement patterns
volatile char* config_forward[] = {neutral, walk1, walk2, walk3, walk4, 
                            walk5, walk6, walk7, walk8, walk9, walk10};
volatile char* config_left[] = {neutral, left1, left2, left3, left4, 
                            left5, left6, left7, left8, left9, left10};
volatile char* config_right[] = {neutral, right1, right2, right3, right4,
                       right5, right6, right7, right8, right9, right10};
//Handle for the current leg movement pattern
volatile char** config_now = config_forward;

//Ultrasonic Sensor timing
volatile unsigned int distance = 0; 
//Mood of the robot
volatile char mood; //0-20: 20,17--> happy  16,6 --> neutral 5,0 --> angeR

unsigned char NeoGreen [NeoNum]; //Set green neopixels
unsigned char NeoBlue [NeoNum];  //Set blue neopixels
unsigned char NeoRed [NeoNum];   //Set red neopixels

//Neopixel functions: by Bruce Land
// === output one bit ======================================
void NeoBit (char Bit){
   if (Bit == 0){ 
    //wait32;// Bit '0'   
    mPORTCSetBits(BIT_7);wait0on;mPORTCClearBits(BIT_7);wait0off 
  } else {
    //wait24;// Bit '1'
    mPORTCSetBits(BIT_7);wait1on;mPORTCClearBits(BIT_7);wait1off; 
  }     
}

// === draw pixel colors ===================================
void NeoDraw (void){
   unsigned char NeoPixel;
   signed char BitCount;
   for (NeoPixel = 0; NeoPixel < NeoNum; NeoPixel++)
   {    
      for (BitCount = 7; BitCount >= 0; BitCount--)      
         NeoBit(bit_test(NeoGreen[NeoPixel], BitCount));      
      for (BitCount = 7; BitCount >= 0; BitCount--)           
         NeoBit(bit_test(NeoRed[NeoPixel], BitCount));            
      for (BitCount = 7; BitCount >= 0; BitCount--)      
         NeoBit(bit_test(NeoBlue[NeoPixel], BitCount));      
   }
   mPORTCClearBits(BIT_7);
}
//===========================================================

// Interrupt service routine to output pwm signals to servoes
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    count_val++;
    if(count_val == count_high) count_val = 0;

    map_pwmB(0 , BIT_2, config_now);
    map_pwmB(1 , BIT_3, config_now);
    map_pwmC(2 , BIT_0, config_now);
    map_pwmC(3 , BIT_1, config_now);
    map_pwmC(4 , BIT_2, config_now);
    map_pwmB(5 , BIT_4, config_now);
    
    map_pwmC(6 , BIT_3, config_now);
    map_pwmC(7 , BIT_4, config_now);
    map_pwmC(8 , BIT_5, config_now);
    map_pwmB(9 , BIT_5, config_now);
    map_pwmB(10 , BIT_10, config_now);
    map_pwmB(11 , BIT_11, config_now);
    
    mT2ClearIntFlag();
}

// Calculate distance using ultrasonic sensors
void calc_distance(void){
    int i;
    mPORTASetBits(BIT_0);
    for(i = 0; i < 50; i ++){asm("nop");}
    mPORTAClearBits(BIT_0);  
    while(!mPORTAReadBits(BIT_1));
    WriteTimer1(0);
    while(mPORTAReadBits(BIT_1) && ReadTimer1() < 600);
    distance = ReadTimer1();
}


void main(void) {

  //Start button on robot 
  mPORTCSetPinsDigitalIn(BIT_9);
  //Debug LED on the board
  mPORTCSetPinsDigitalOut(BIT_8);
  
  //Each of these hooks up to the pwm of a servo
  mPORTBSetPinsDigitalOut(BIT_2);  //B2
  mPORTBSetPinsDigitalOut(BIT_3);  //B3
  mPORTCSetPinsDigitalOut(BIT_0);  //C0
  mPORTCSetPinsDigitalOut(BIT_1);  //C1
  mPORTCSetPinsDigitalOut(BIT_2);  //C2
  mPORTBSetPinsDigitalOut(BIT_4);  //B4
  mPORTBSetPinsDigitalOut(BIT_10); //B10
  mPORTBSetPinsDigitalOut(BIT_11); //B11
  mPORTBSetPinsDigitalOut(BIT_5);  //B5
  mPORTCSetPinsDigitalOut(BIT_3);  //C3
  mPORTCSetPinsDigitalOut(BIT_4);  //C4
  mPORTCSetPinsDigitalOut(BIT_5);  //C5
  
  //The trigger pin for the HC SR04 US Sensor
  mPORTASetPinsDigitalOut(BIT_0);
  //The echo pin for the HC SR04 US Sensor
  mPORTASetPinsDigitalIn(BIT_1);
  //C7 is the signal pin for the neopixels 
  mPORTCSetPinsDigitalOut(BIT_7);  
  
  //These for-loops add the "neutral" offset to each of the
  //elements of the leg pattern. ->Allows easy callibration
  int i;
  for(i = 1; i < conf_len; i++){
      int j;
      for(j = 0; j < 12; j++){
        config_forward[i][j] += config_forward[0][j];
      }
  }
  for(i = 1; i < conf_len; i++){
      int j;
      for(j = 0; j < 12; j++){
        config_left[i][j] += config_left[0][j];
      }
  }
  for(i = 1; i < conf_len; i++){
      int j;
      for(j = 0; j < 12; j++){
        config_right[i][j] += config_right[0][j];
      }
  }
  
  //Timer to time the ultrasonic sensor
  OpenTimer1(T1_ON | T1_PS_1_256, 0xffff);

  //Timer to trigger the pwm ISR 
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 1000);
  INTEnableSystemMultiVectoredInt();
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); //clear the interrupt flag

  while(mPORTCReadBits(BIT_9)); //Wait for start

  while (1){
    config_now = config_forward;
    int obstacle = 1;
    int i;
    for(i = 0; i < 2; i++){
      calc_distance(); //check twice, improves error
      if(distance > 500) obstacle = 0;
    }
    if(obstacle){
        mood-=8;
        if(mood < 0) mood = 0;
    }else{
        mood++;
        if(mood > 20) mood = 20;
    }

    //re-write neopixel arrays so that the correct face displays
    char gr, bl, rd;
    if(mood  < 6){
        gr = 0; bl = 0; rd = 130;
    }else if (mood >= 6 && mood < 17){
        gr = 0; bl = 130; rd = 0; 
    }else{
        gr = 130; bl = 0; rd = 0;
    }
    int f;
    for(f = 0; f < hface_len; f++){
        NeoGreen[hface[f]] = gr;
    }
    for(f = 0; f < nface_len; f++){
        NeoBlue[nface[f]] = bl;
    }
    for(f = 0; f < sface_len; f++){
        NeoRed[sface[f]] = rd;
    }
    
    //Lockout the pwm ISR for the time sensitive neopixels
    INTDisableInterrupts();
    NeoDraw(); //redraw neopixel face
    INTEnableInterrupts();

    if(obstacle){
        mPORTCSetBits(BIT_8);
        if(rand() % 2 == 1){ //randomly choose a way to turn
            config_now = config_right;
        }else{
            config_now = config_left;
        }
        //Cycle through the current leg movement pattern 10 times
        int k;
        for(k = 0; k < 10; k ++){
        int j;
          for(j = 0; j < conf_len; j++){
            int i;
            for(i = 0; i < 100000; i ++){asm("nop");}
              conf_i = j;
          }
        }
    }else{
        mPORTCClearBits(BIT_8);
        config_now = config_forward;
        //Cycle thorugh the "forward" pattern once
        int j;
          for(j = 0; j < conf_len; j++){
            int i;
            for(i = 0; i < 100000; i ++){asm("nop");}
              conf_i = j;
        }  
    } 
  }
} // main
// === end  ======================================================
