//    Block Compiler is a Block Diagram Continuous System Simulator
//    for the Teensy processors LC 3.5 and 3.6
//    Copyright (C) 2021  V.A. Kanto
//    Modified for Raspberry Pi Pico 12-26-2022 Simple-Circuit
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.


//baud = 460800 with one stop bit, 8-data bits and no parity

#include "RP2040_PWM.h" //used for header files, pwm directly programmed

#define pwm_pin4    4 
#define pwm_pin6    6 
#define pwm_pin9    9 
#define pwm_pin10   10 

//****Block Diagram Definitions
#define NOP 0 
#define INT 1
#define ADU 2
#define SUB 3
#define MUL 4
#define PRT 5
#define END 6
#define DAC 7
#define SVF 8
#define EUL 9
#define DIV 10
#define SWG 11
#define LPF 12
#define LIM 13
#define SMT 14
#define OSC 15
#define ZIN 16
#define SQT 17
#define DIF 18
#define SUM 19
#define DLA 20
#define PWM 21
#define MOT 22
#define ENC 23
#define RAL 24
#define PRF 25
#define ABS 26
#define BRZ 27
#define BNZ 28
#define BRM 29
#define BRP 30
#define OUT 31
#define INP 32
#define RST 33
#define MUX 34
#define ANG 35
#define ORG 36
#define XOR 37
#define RLY 38
#define NOC 39
#define NCC 40

#define BLK_MAX 41
#define PMAX 50
#define VMAX 100
#define TIMING_PIN 22

//****End Block Definitions


//****System Variables

#define ADC_GAIN 0.004969

volatile long analog_avg;     //average of 1024 samples of adc data
volatile int lut[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; //encoder look up table
volatile int old_state = 0;                                   //past encoder state
volatile int enc_pin1 = 2;   //encoder pin-1 input
volatile int enc_pin2 = 3;   //encoder pin-2 input
volatile long enc_count = 0;  //cumulative endcoder count +/-2147483648
volatile int c;               //received serial character
volatile int n;               //counts pins from command command
int c_count = 0;            //command buffer counter
char cmd_buf[10];           //serial port input buffer to hold command string
volatile int cmd_val = 0;   //integer value from command string
double frequency = 146484; //10-bits 150mhz/1024 for DACs
double frequency2 = 2289; //for motor
      
//****Start Block Variables
volatile char line_buf[80];
volatile int line_count = 0;
volatile char name_array[VMAX+1][3];     //variable name storage array
volatile float val_array[VMAX+1][3];     //variable value array
volatile int prog_array[PMAX+1][6];      //program and variable token array 
volatile int num_var[BLK_MAX];
volatile int prog_count = 0;
volatile int prog_save = 0;
volatile int prog_edit = 0;
volatile int edit_flag = -1;
volatile int var_count = 4;

volatile double tb = 0.0;
volatile float dT = 0.01;  
volatile float tmax = 1.0;  
volatile unsigned long time_us;
volatile unsigned long dt_us;
volatile int count = 0;

volatile float adc0_os = 10.0; 
volatile float adc1_os = 10.0; 

//**** End block variables

void setup(){
   pinMode(25, OUTPUT); //built in LED
}
void loop(){

}

 
//setup1 initiallizes the i/o pin directions, the serial interface and the ADC
//block runs on core 1  
void setup1() {
 
gpio_set_function(pwm_pin4,GPIO_FUNC_PWM);
gpio_set_function(pwm_pin6,GPIO_FUNC_PWM);
gpio_set_function(pwm_pin9,GPIO_FUNC_PWM);
gpio_set_function(pwm_pin10,GPIO_FUNC_PWM);

// Used to find channels and slice numbers for PWM
//delay(10000);
//Serial.println( pwm_gpio_to_slice_num(pwm_pin4));
//Serial.println( pwm_gpio_to_channel(pwm_pin4));
//Serial.println( pwm_gpio_to_slice_num(pwm_pin6));
//Serial.println( pwm_gpio_to_channel(pwm_pin6));
//Serial.println( pwm_gpio_to_slice_num(pwm_pin9));
//Serial.println( pwm_gpio_to_channel(pwm_pin9));
//Serial.println( pwm_gpio_to_slice_num(pwm_pin10));
//Serial.println( pwm_gpio_to_channel(pwm_pin10));

pwm_set_wrap(2,1023);
pwm_set_chan_level(2,0,512);
pwm_set_enabled(2,true);

pwm_set_wrap(3,1023);
pwm_set_chan_level(3,0,512);
pwm_set_enabled(3,true);

pwm_set_wrap(4,65535);
pwm_set_chan_level(4,1,0);
pwm_set_enabled(4,true);

pwm_set_wrap(5,65535);
pwm_set_chan_level(5,0,0);
pwm_set_enabled(5,true);
      
      pinMode(0, INPUT_PULLUP); //GP digital input
      pinMode(1, INPUT_PULLUP); //GP digital input
      pinMode(2, INPUT_PULLUP); //GP digital input / encoder
      pinMode(3, INPUT_PULLUP); //GP digital input / encoder
//      pinMode(4, OUTPUT); //pwm high freq out
      pinMode(5, OUTPUT); //gp output 
//      pinMode(6, OUTPUT); //pwm high freq out
      pinMode(7, OUTPUT); //gp out
      pinMode(8, OUTPUT); //gp output
//      pinMode(9, OUTPUT); //gp output : motor pwm/pwm out
//      pinMode(10, OUTPUT); //gp output : motor pwm/pwm out
      pinMode(11, OUTPUT); //gp output
      pinMode(12, OUTPUT); //gp output
      pinMode(13, OUTPUT); //gp output
      pinMode(14, OUTPUT); //gp output
      pinMode(15, INPUT_PULLUP); //GP digital input
      pinMode(16, INPUT_PULLUP); //GP digital input
      pinMode(17, INPUT_PULLUP); //GP digital input
      pinMode(18, INPUT_PULLUP); //GP digital input
      pinMode(19, INPUT_PULLUP); //GP digital input
      pinMode(20, INPUT_PULLUP); //GP digital input          
      pinMode(21, OUTPUT); //gp output
      pinMode(22, OUTPUT); //timing out test point for adc/dac interrupt loop
      pinMode(26, INPUT); //analog input
      pinMode(27, INPUT); //analog input
      pinMode(28, INPUT); //analog input
      pinMode(25, OUTPUT); //built in LED     
      enc_pin1 = 2;          //assign the encoder input pins
      enc_pin2 = 3;
      attachInterrupt(digitalPinToInterrupt( enc_pin1),ip0,CHANGE);  //attach the encoder pins to an interrupt
      attachInterrupt(digitalPinToInterrupt( enc_pin2),ip0,CHANGE);       
      
    Serial.begin(460800);        
}

void loop1() { 
  //Send operation to Block program
  Block();   
}

//interrupt service routine for encoder count update
//Look up table method from:
//Oleg Mazurov
//"Interrupt Service Routine for AVR micros"
//Circuits@Home

void ip0(){
  old_state = ((old_state << 2) | (digitalRead(enc_pin1) << 1) | digitalRead(enc_pin2)) & 15;
  enc_count = enc_count + lut[old_state]; 
}


//****** Block Diagram Compiler Main Loop

void Block() {
 volatile char c;
 volatile int flg;
 int sst = 0;
 int ist = 0;
 
 reset_all(); //initiallize fixed variables
  
//This section scans the serial input string for a command string 
 while(true){  
  c = 0; 
  line_count = 0;    
  Serial.print(prog_count);
  Serial.print(": ");  
       
  while (c != 13) {
    while (Serial.available() == 0); 
    c = Serial.read();     
    if (c != 10) {
      Serial.write(c);
      if (c==8){             //Back Space
        line_count--;
        if (line_count < 0) line_count = 0;
      } else {
       line_buf[line_count++] = c;
       if (line_count > 79) line_count = 79;
      }            
    }
  }  
    Serial.write(10);   //LF
    c_count = 0;        //reset to command start
    cmd_buf[0] = 0;
    cmd_buf[1] = 0;
    cmd_buf[2] = 0;
    line_count = 0;


  while(true){ 
    c = read_line();            
    if (c_count < 3){
      cmd_buf[c_count] = c;  //put the character in the command array    
      c_count++;             //command names 3 char max 
    }
    if ((c == 13) || (c == ' ') || (c == 9)){ //white space terminates command
      if (c_count < 3) cmd_buf[c_count] = 0;
      c = 0;
      fndBcmd(); 
      break;
    }  
  }
  Serial.println();
 }
}

char read_line(void){
  char c;
  c = line_buf[line_count++];
  if (line_count > 79) {
    line_count = 79;
    return(13);
  }
  return(c);
}


char skip_ws(void){
volatile char c = ' ';
     while ((c == ' ') || (c == 9)){   //skip white space
       c = read_line(); 
     } 
     return(c);   
}

int setname(void) {
volatile char c;
  int i;

    if (var_count>=VMAX) return(-1); 
    c_count = 0; 
    cmd_buf[0] = 0;
    cmd_buf[1] = 0;
    cmd_buf[2] = 0;
    c = skip_ws();    
    while(true){         
     if (c_count < 3) {
      cmd_buf[c_count] = c;  //put the character in the command array
     }
     if ((c == 13) || (c == 10) || (c == ' ') || (c == 9)){ //white space terminates command
       cmd_buf[c_count] = 0;
       cmd_buf[9] = c;
       i = 0;
       while (i < var_count){
        if ((cmd_buf[0] == name_array[i][0]) && (cmd_buf[1] == name_array[i][1]) && (cmd_buf[2] == name_array[i][2])){
          return(i);
        }
        i++; 
       }
       name_array[var_count][0] = cmd_buf[0];
       name_array[var_count][1] = cmd_buf[1];
       name_array[var_count][2] = cmd_buf[2]; 
       var_count++;      
       return(var_count-1);    
    }
    c = read_line(); 
    c_count++;   //command names 3 char max               
  }
}

int setfloat(int indx) {
 float f;
 char c;
 String s;

  c = 0;
  while ((c != 13) && (c != 10)){ 
    c = read_line();                      
    s += c;
  }
  f = s.toFloat();
  val_array[indx][0] = f;
  val_array[indx][1] = f;
  val_array[indx][2] = f; 
  return(0); 
}

void printblock(int b){
   if (b == SUM) Serial.print("sum ");
   if (b == INT) Serial.print("int ");
   if (b == ADU) Serial.print("adc ");
   if (b == DAC) Serial.print("dac ");
   if (b == SUB) Serial.print("sub ");
   if (b == MUL) Serial.print("mul ");
   if (b == PRT) Serial.print("prt ");
   if (b == END) Serial.print("end ");
   if (b == SVF) Serial.print("svf ");
   if (b == EUL) Serial.print("eul ");
   if (b == DIV) Serial.print("div ");
   if (b == SWG) Serial.print("swg ");
   if (b == LPF) Serial.print("lpf ");
   if (b == LIM) Serial.print("lim ");
   if (b == SMT) Serial.print("smt ");
   if (b == OSC) Serial.print("osc ");
   if (b == ZIN) Serial.print("zin ");
   if (b == SQT) Serial.print("sqt ");
   if (b == DIF) Serial.print("dif ");
   if (b == NOP) Serial.print("nop ");
   if (b == MOT) Serial.print("mot ");
   if (b == NOP) Serial.print("    ");
   if (b == PWM) Serial.print("pwm ");
   if (b == ENC) Serial.print("enc ");
   if (b == RAL) Serial.print("ral "); 
   if (b == PRF) Serial.print("prf ");    
   if (b == ABS) Serial.print("abs ");    
   if (b == BRZ) Serial.print("brz ");    
   if (b == BNZ) Serial.print("bnz ");    
   if (b == BRP) Serial.print("brp ");    
   if (b == BRM) Serial.print("brm ");    
   if (b == OUT) Serial.print("out ");    
   if (b == INP) Serial.print("inp ");    
   if (b == RST) Serial.print("rst ");    
   if (b == MUX) Serial.print("mux "); 
   if (b == DLA) Serial.print("dla ");    
   if (b == ANG) Serial.print("and ");    
   if (b == ORG) Serial.print("ore "); 
   if (b == XOR) Serial.print("xor "); 
   if (b == RLY) Serial.print("rly "); 
   if (b == NOC) Serial.print("noc "); 
   if (b == NCC) Serial.print("ncc ");    
}    
    

int getnames(int n) {
volatile int j;
volatile int i;

   prog_array[prog_count][1] = 0;
   prog_array[prog_count][2] = 0;
   prog_array[prog_count][3] = 0;
   prog_array[prog_count][4] = 0;
   prog_array[prog_count][5] = 0;
   
   for (j=1;j<=n;j++){ 
    if ((i = setname()) < 0) { 
      Serial.println(" ***** Variable Name Error ***** "); 
     return(-1);
    } else {
    prog_array[prog_count][j] = i;
    if (cmd_buf[9] == 13) break;
    }
   }
   prog_count++;
   if (prog_count>=PMAX){
    Serial.print(" ***** To Many Program Blocks ***** ");
    return(-1);
   } 
   return(0);  
}

void reset_all(void){
   int i;
   
  //Clear Variable and Program space
   for (i=0;i<VMAX;i++){
    name_array[i][0] = 0;
    name_array[i][1] = 0;
    name_array[i][2] = 0;
    val_array[i][0] = 0.0;
    val_array[i][1] = 0.0;
    val_array[i][2] = 0.0;
   } 
    name_array[1][0] = 'd';
    name_array[1][1] = 't';
    name_array[1][2] = 0;
    val_array[1][0] = 0.01;
    val_array[1][1] = 0.01;
    val_array[1][2] = 0.01;

    name_array[2][0] = 't';
    name_array[2][1] = 0;
    name_array[2][2] = 0;

    name_array[3][0] = 'm';
    name_array[3][1] = 'a';
    name_array[3][2] = 'x';
    val_array[3][0] = 1.0;
    val_array[3][1] = 1.0;
    val_array[3][2] = 1.0;
 
    name_array[4][0] = 'd';
    name_array[4][1] = 'e';
    name_array[4][2] = 'c';
    val_array[4][0] = 1.0;
    val_array[4][1] = 1.0;
    val_array[4][2] = 1.0;

    name_array[5][0] = 'f';
    name_array[5][1] = 'm';
    name_array[5][2] = 't';
    val_array[5][0] = 6;
    val_array[5][1] = 6;
    val_array[5][2] = 6;
    
    name_array[6][0] = 'a';
    name_array[6][1] = 'v';
    name_array[6][2] = 'g';
    val_array[6][0] = 1;
    val_array[6][1] = 1;
    val_array[6][2] = 1;
    
   
   for (i=0;i<PMAX;i++){
    prog_array[i][0] = 0;
    prog_array[i][1] = 0;
    prog_array[i][2] = 0;
    prog_array[i][3] = 0;
    prog_array[i][4] = 0;
   }    

   prog_count = 0;
   var_count = 7;  
//   var_count = 6;
}

 


//routine to search input string for block command
//additional 3-character commands with integer input can be added
//for more functions
int fndBcmd(void) {
  int i;
  int j;
  char ch;

  //Clear Variable and Program space
  if ((cmd_buf[0] == 'h') && (cmd_buf[1] == 'l') && (cmd_buf[2] == 'p')) {
   Serial.println("var t : time in seconds");
   Serial.println("var dt : step time in seconds");
   Serial.println("var max : max run real time, if negative run at full speed");
   Serial.println("var dec : print decimation, e.g. 1 = every loop, 6 = every 6th loop");
   Serial.println("var fmt : print format = number of decimals after point");
   Serial.println("var avg : number of adc sample to average per reading");
   Serial.println("");
   Serial.println("abs in out : out = |in|");
   Serial.println("adc channel out  : read adc channel 0,1 at +-10V, 2 at 0 to 3.3V");
   Serial.println("and in1 in2 out : out = 1 if in1>0 and in2>0 else out = 0");
   Serial.println("bnz in offset :if in!=0 then PC=PC+offset else PC=PC+1");
   Serial.println("brm in offset :if in<0 then PC=PC+offset else PC=PC+1");
   Serial.println("brp in offset :if in>0 then PC=PC+offset else PC=PC+1");
   Serial.println("brz in offset :if in==0 then PC=PC+offset else PC=PC+1");
   Serial.println("dac value channel : send dac value -10 to +10V to channel 0,1");
   Serial.println("dif in filter out  :differentiator with 0 <= LP filter <= 1.0");
   Serial.println("div a b x  :x = a/b");
   Serial.println("dla in delay out :when in > 0, out --> 1 after delay sec");
   Serial.println("end :end of block program");
   Serial.println("eul in gain limit out :1st order integrate lim=0 none, lim>0 +-, lim<0 0+");
   Serial.println("inp pin value :I/0 value = dig_in(pin) [0,1], 0<=pin<=28");
   Serial.println("int in gain limit out :2nd order integrate lim=0 none, lim>0 +-, lim<0 0+");
   Serial.println("lim in +lim -lim out  :limit signal to +- value");
   Serial.println("mot in :pwm motor H-bridge Dout-9 Dout-10  -1.0 <= in <= 1.0"); 
   Serial.println("mul a b x  :x = a*b");
   Serial.println("mux in1 in2 ctr out : if ctr<=0 out = in1 else out = in2");
   Serial.println("ncc in ctr out : out = 1 if in>0 and ctr<=0 else out = 0");
   Serial.println("noc in ctr out : out = 1 if in>0 and ctr>0 else out = 0");
   Serial.println("ore in1 in2 out : out = 1 if in1>0 or in2>0 else out = 0");
   Serial.println("osc w offset magnitude out  :sine wave oscillator locked on t");
   Serial.println("out value pin :I/0 dig_out(pin)= value [0<=0,1>0] 4<=pin<=14");
   Serial.println("prf in :fast print");
   Serial.println("prt in1 in2 in3 in4 in5  :print up to 5 inputs");
   Serial.println("pwm in channel  :pulse width modulation ch=4,6,9,10 0 <= in <= 1.0");
   Serial.println("ral in rate out  :limit slew rate to volts/sec");
   Serial.println("rly nc no coil out : out = 1 if (nc>0 and coil<=0) or (no>0 and coil>0)");
   Serial.println("rst val var :reset variable var to value val");
   Serial.println("smt in level out  :schmidt trigger +-level, initial out is not zero");
   Serial.println("sqr in out :out = sign(in)*square root (|in|)");
   Serial.println("sub a b x  :x = a-b");
   Serial.println("sum a b x  :x = a+b");
   Serial.println("svf in g1 g2 BPout LPout  :state variable filter");
   Serial.println("swg width out  :square wave generator width=1/2 duty, initial out > 0 ");
   Serial.println("xor in1 in2 out : out = in1 xor in2 where logic 0 is <=0, logic 1 is > 0");
   Serial.println("zin in out  :delay one dt time step");
   Serial.println("");
   Serial.println("run : command to run program");
   Serial.println("<esc> or '!' : command charater to stop program");
   Serial.println("clr : command to clear program and variables");
   Serial.println("set var val : command to assign a variable a value");
   Serial.println("lst : command to list program");
   Serial.println("var : command to list variable values");
   Serial.println("lin nnn: command to set editor to line # nnn");
   Serial.println("srt : command to sort program");
   Serial.println("zro : command to zero encoder count");
   Serial.println("hlp : command to list blocks, commands and built-in variables");
   
   return(0); 
  } 
  
  //Clear Variable and Program space
  if ((cmd_buf[0] == 'c') && (cmd_buf[1] == 'l') && (cmd_buf[2] == 'r')) {
   reset_all();
   return(0); 
  } 
   
  //Sort Program 
  if ((cmd_buf[0] == 's') && (cmd_buf[1] == 'r') && (cmd_buf[2] == 't')) { 
    sort_blocks();
    return(0);
  }
  //Define new Variable
  if ((cmd_buf[0] == 's') && (cmd_buf[1] == 'e') && (cmd_buf[2] == 't')) { 
   if ((i = setname()) < 0) { 
    Serial.println(" ***** Variable Name Error ***** "); 
    return(-1);
   } else {
    if (setfloat(i) < 0) {
      Serial.println(" ***** Variable Value Error ***** "); 
      return(-1);
    }
   }
    return(0);
  }

  //Set Code Line Pointer
  if ((cmd_buf[0] == 'l') && (cmd_buf[1] == 'i') && (cmd_buf[2] == 'n')) {
     if (setfloat(VMAX) < 0) {
      Serial.println(" ***** Line Value Error ***** ");
      return(-1);
    }
    if (val_array[VMAX][0] >= PMAX){
      Serial.println(" ***** Line Value Error ***** ");
      return(-1);
    } else {
    prog_count = round(val_array[VMAX][0]);
    printblock(prog_array[prog_count][0]);
    for (j=1;j<=5;j++){
     ch = name_array[(prog_array[prog_count][j])][0];
     if (ch==0) ch = ' '; 
     Serial.write(ch);
     ch = name_array[(prog_array[prog_count][j])][1];
     if (ch==0) ch = ' '; 
     Serial.write(ch);
     ch = name_array[(prog_array[prog_count][j])][2];
     if (ch==0) ch = ' '; 
     Serial.write(ch);
     Serial.print(" ");   
    }
    Serial.println(); 
      }
    return(1);
  }

  //List Variables
  if ((cmd_buf[0] == 'v') && (cmd_buf[1] == 'a') && (cmd_buf[2] == 'r')) {
   Serial.println(); 
   for (i=1; i<var_count;i++){
    Serial.print("set ");
    ch = name_array[i][0];
    if (ch==0) ch =' ';
    Serial.write(ch);
    ch = name_array[i][1];
    if (ch==0) ch =' ';
    Serial.write(ch);
    ch = name_array[i][2];
    if (ch==0) ch =' ';
    Serial.write(ch);
    Serial.print(" ");
    Serial.println(val_array[i][1],6); 
   }  
   return(0);
  }

  //List Block Program
  if ((cmd_buf[0] == 'l') && (cmd_buf[1] == 's') && (cmd_buf[2] == 't')) {
   Serial.println(); 
   for (i=0; i<prog_count;i++){
    printblock(prog_array[i][0]);
    for (j=1;j<=5;j++){
     Serial.write(name_array[(prog_array[i][j])][0]);
     Serial.write(name_array[(prog_array[i][j])][1]);
     Serial.write(name_array[(prog_array[i][j])][2]); 
     Serial.print(" ");   
    }
    Serial.println(); 
   }  
   return(0);
  }

  //No Op
  if ((cmd_buf[0] == 'n') && (cmd_buf[1] == 'o') && (cmd_buf[2] == 'p')) { 
   prog_array[prog_count][0] = NOP;
   num_var[NOP] = 0;
   return(getnames(0));
  }
  
  //Branch if zero
  if ((cmd_buf[0] == 'b') && (cmd_buf[1] == 'r') && (cmd_buf[2] == 'z')) { 
   prog_array[prog_count][0] = BRZ;
   num_var[BRZ] = 2;
   return(getnames(2));
  }

  //Branch if not zero
  if ((cmd_buf[0] == 'b') && (cmd_buf[1] == 'n') && (cmd_buf[2] == 'z')) { 
   prog_array[prog_count][0] = BNZ;
   num_var[BNZ] = 2;
   return(getnames(2));
  }
  //Branch if positive
  if ((cmd_buf[0] == 'b') && (cmd_buf[1] == 'r') && (cmd_buf[2] == 'p')) { 
   prog_array[prog_count][0] = BRP;
   num_var[BRP] = 2;
   return(getnames(2));
  }
  //Branch if minus
  if ((cmd_buf[0] == 'b') && (cmd_buf[1] == 'r') && (cmd_buf[2] == 'm')) { 
   prog_array[prog_count][0] = BRM;
   num_var[BRM] = 2;
   return(getnames(2));
  }

  //Digital out
  if ((cmd_buf[0] == 'o') && (cmd_buf[1] == 'u') && (cmd_buf[2] == 't')) { 
   prog_array[prog_count][0] = OUT;
   num_var[OUT] = 2;
   return(getnames(2));
  }

  //Digital input
  if ((cmd_buf[0] == 'i') && (cmd_buf[1] == 'n') && (cmd_buf[2] == 'p')) { 
   prog_array[prog_count][0] = INP;
   num_var[INP] = 2;
   return(getnames(2));
  }

  //Delayed Trigger
  if ((cmd_buf[0] == 'd') && (cmd_buf[1] == 'l') && (cmd_buf[2] == 'a')) { 
   prog_array[prog_count][0] = DLA;
   num_var[DLA] = 3;
   return(getnames(3));
  }

    //And Gate
  if ((cmd_buf[0] == 'a') && (cmd_buf[1] == 'n') && (cmd_buf[2] == 'd')) { 
   prog_array[prog_count][0] = ANG;
   num_var[ANG] = 3;
   return(getnames(3));
  }
  
  //Or Gate
  if ((cmd_buf[0] == 'o') && (cmd_buf[1] == 'r') && (cmd_buf[2] == 'e')) { 
   prog_array[prog_count][0] = ORG;
   num_var[ORG] = 3;
   return(getnames(3));
  }

  //Xor Gate
  if ((cmd_buf[0] == 'x') && (cmd_buf[1] == 'o') && (cmd_buf[2] == 'r')) { 
   prog_array[prog_count][0] = XOR;
   num_var[XOR] = 3;
   return(getnames(3));
  }

  //Relay
  if ((cmd_buf[0] == 'r') && (cmd_buf[1] == 'l') && (cmd_buf[2] == 'y')) { 
   prog_array[prog_count][0] = RLY;
   num_var[RLY] = 4;
   return(getnames(4));
  }

  //Normall open contact
  if ((cmd_buf[0] == 'n') && (cmd_buf[1] == 'o') && (cmd_buf[2] == 'c')) { 
   prog_array[prog_count][0] = NOC;
   num_var[NOC] = 3;
   return(getnames(3));
  }

  //Normall closed contact
  if ((cmd_buf[0] == 'n') && (cmd_buf[1] == 'c') && (cmd_buf[2] == 'c')) { 
   prog_array[prog_count][0] = NCC;
   num_var[NCC] = 3;
   return(getnames(3));
  }

  //multiplexer
  if ((cmd_buf[0] == 'm') && (cmd_buf[1] == 'u') && (cmd_buf[2] == 'x')) { 
   prog_array[prog_count][0] = MUX;
   num_var[MUX] = 4;
   return(getnames(4));
  }

  //Reset Variable
  if ((cmd_buf[0] == 'r') && (cmd_buf[1] == 's') && (cmd_buf[2] == 't')) { 
   prog_array[prog_count][0] = RST;
   num_var[RST] = 2;
   return(getnames(2));
  }
    
  //DAC out
  if ((cmd_buf[0] == 'd') && (cmd_buf[1] == 'a') && (cmd_buf[2] == 'c')) { 
   prog_array[prog_count][0] = DAC;
   num_var[DAC] = 2;
   return(getnames(2));
  }

  //Sine Oscillator
  if ((cmd_buf[0] == 'o') && (cmd_buf[1] == 's') && (cmd_buf[2] == 'c')) { 
   prog_array[prog_count][0] = OSC;
   num_var[OSC] = 4;
   return(getnames(4));
  }

  //Pulse Width Modulator out
  if ((cmd_buf[0] == 'p') && (cmd_buf[1] == 'w') && (cmd_buf[2] == 'm')) { 
   prog_array[prog_count][0] = PWM;
   num_var[PWM] = 2;
   return(getnames(2));
  }

  //Square Root
  if ((cmd_buf[0] == 's') && (cmd_buf[1] == 'q') && (cmd_buf[2] == 't')) { 
   prog_array[prog_count][0] = SQT;
   num_var[SQT] = 2;
   return(getnames(2));
  }

  //Read Encoder Count
  if ((cmd_buf[0] == 'e') && (cmd_buf[1] == 'n') && (cmd_buf[2] == 'c')) { 
   prog_array[prog_count][0] = ENC;
   num_var[ENC] = 2;
   return(getnames(2));
  }

  //Clear Encoder Count
  if ((cmd_buf[0] == 'z') && (cmd_buf[1] == 'r') && (cmd_buf[2] == 'o')) { 
   enc_count = 0;
   return(0);
  }

  //Cal ADC0 and ADC1 offsets
  if ((cmd_buf[0] == 'c') && (cmd_buf[1] == 'a') && (cmd_buf[2] == 'l')) { 
   analog_avg = 0;
   for (int i =0; i < 256; i++) analog_avg = analogRead(26) + analog_avg; 
   adc0_os = analog_avg*ADC_GAIN/256.0;
   analog_avg = 0;
   for (int i =0; i < 256; i++) analog_avg = analogRead(27) + analog_avg; 
   adc1_os = analog_avg*ADC_GAIN/256.0; 
   return(0);
  }

  //Motor H-bridge PWM
  if ((cmd_buf[0] == 'm') && (cmd_buf[1] == 'o') && (cmd_buf[2] == 't')) { 
   prog_array[prog_count][0] = MOT;
   num_var[MOT] = 1;
   return(getnames(1));
  }

  //Sum two numbers
  if ((cmd_buf[0] == 's') && (cmd_buf[1] == 'u') && (cmd_buf[2] == 'm')) { 
   prog_array[prog_count][0] = SUM;
   num_var[SUM] = 3;
   return(getnames(3));
  }

  //Integrate 2nd order with limiter
  if ((cmd_buf[0] == 'i') && (cmd_buf[1] == 'n') && (cmd_buf[2] == 't')) { 
   prog_array[prog_count][0] = INT;
   num_var[INT] = 4;
   return(getnames(4));
  }

  //Integrate 1st order with limiter
  if ((cmd_buf[0] == 'e') && (cmd_buf[1] == 'u') && (cmd_buf[2] == 'l')) { 
   prog_array[prog_count][0] = EUL;
   num_var[EUL] = 4;
   return(getnames(4));
  }

  //State Variable Filter
  if ((cmd_buf[0] == 's') && (cmd_buf[1] == 'v') && (cmd_buf[2] == 'f')) { 
   prog_array[prog_count][0] = SVF;
   num_var[SVF] = 5;
   return(getnames(5));
  }

  //Subtract two numbers
  if ((cmd_buf[0] == 's') && (cmd_buf[1] == 'u') && (cmd_buf[2] == 'b')) { 
   prog_array[prog_count][0] = SUB;
   num_var[SUB] = 3;
   return(getnames(3));
  }

  //Divide two numbers
  if ((cmd_buf[0] == 'd') && (cmd_buf[1] == 'i') && (cmd_buf[2] == 'v')) { 
   prog_array[prog_count][0] = DIV;
   num_var[DIV] = 3;
   return(getnames(3));
  }

  //Multiply two numbers
  if ((cmd_buf[0] == 'm') && (cmd_buf[1] == 'u') && (cmd_buf[2] == 'l')) { 
   prog_array[prog_count][0] = MUL;
   num_var[MUL] = 3;
   return(getnames(3));
  }
  
  //Z^-1 delay
  if ((cmd_buf[0] == 'z') && (cmd_buf[1] == 'i') && (cmd_buf[2] == 'n')) { 
   prog_array[prog_count][0] = ZIN;
   num_var[ZIN] = 2;
   return(getnames(2));
  }
  
  //Absolute value
  if ((cmd_buf[0] == 'a') && (cmd_buf[1] == 'b') && (cmd_buf[2] == 's')) { 
   prog_array[prog_count][0] = ABS;
   num_var[ABS] = 2;
   return(getnames(2));
  }
  
  //Limiter
  if ((cmd_buf[0] == 'l') && (cmd_buf[1] == 'i') && (cmd_buf[2] == 'm')) { 
   prog_array[prog_count][0] = LIM;
   num_var[LIM] = 4;
   return(getnames(4));
  }

  //Schmidt Trigger
  if ((cmd_buf[0] == 's') && (cmd_buf[1] == 'm') && (cmd_buf[2] == 't')) { 
   prog_array[prog_count][0] = SMT;
   num_var[SMT] = 3;
   return(getnames(3));
  }

  //ADC in
  if ((cmd_buf[0] == 'a') && (cmd_buf[1] == 'd') && (cmd_buf[2] == 'c')) { 
   prog_array[prog_count][0] = ADU;
   num_var[ADU] = 2;
   return(getnames(2));
  }
  
  //Differentiator
  if ((cmd_buf[0] == 'd') && (cmd_buf[1] == 'i') && (cmd_buf[2] == 'f')) { 
   prog_array[prog_count][0] = DIF;
   num_var[DIF] = 3;
   return(getnames(3));
  }
  
  //Rate Limiter
  if ((cmd_buf[0] == 'r') && (cmd_buf[1] == 'a') && (cmd_buf[2] == 'l')) { 
   prog_array[prog_count][0] = RAL;
   num_var[RAL] = 3;
   return(getnames(3));
  } 
  //Square Wave
  if ((cmd_buf[0] == 's') && (cmd_buf[1] == 'w') && (cmd_buf[2] == 'g')) { 
   prog_array[prog_count][0] = SWG;
    num_var[SWG] = 2;
   return(getnames(2));
  }

  //Print up to 5 numbers, 1ms delay per variable
  if ((cmd_buf[0] == 'p') && (cmd_buf[1] == 'r') && (cmd_buf[2] == 't')) { 
   prog_array[prog_count][0] = PRT;
   num_var[PRT] = 5;
   return(getnames(5));
  }

  //Fast Print one variable with no delay
  if ((cmd_buf[0] == 'p') && (cmd_buf[1] == 'r') && (cmd_buf[2] == 'f')) { 
   prog_array[prog_count][0] = PRF;
   num_var[PRF] = 1;
   return(getnames(1));
  }

  //End of program loop back until t = tmax
  if ((cmd_buf[0] == 'e') && (cmd_buf[1] == 'n') && (cmd_buf[2] == 'd')) { 
   prog_array[prog_count][0] = END;
   prog_array[prog_count][1] = 0;
   prog_array[prog_count][2] = 0;
   prog_array[prog_count][3] = 0;
   prog_array[prog_count][4] = 0;
   prog_array[prog_count][5] = 0;   
   prog_count++;
   if (prog_count>=PMAX){
    Serial.print(" ***** To Many Program Blocks ***** ");
    return(-1);
   } 
   return(0);
  }

  if ((cmd_buf[0] == 'r') && (cmd_buf[1] == 'u') && (cmd_buf[2] == 'n')) { 
   dT = val_array[1][0];
   tb = val_array[2][0];
   tmax = val_array[3][0];
   dt_us = (int)(1000000*dT);
   time_us = micros() + dt_us;
   i = 0;
   while(true){
    digitalWrite(TIMING_PIN,1);
    if (Serial.available() != 0){ 
      c = Serial.read(); 
      if (c==27) break;  
      if (c=='!') break; 
    }
    j = prog_array[i][0];
    switch (j) {
     case NOP:
      i++;
      break;
     case SWG:
      square(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case SMT:
      schmidt(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case LIM:
      limiter(prog_array[i][1],prog_array[i][2],prog_array[i][3],prog_array[i][4]);
      i++;
      break;
     case ZIN:
      invz(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case EUL:
      euler(prog_array[i][1],prog_array[i][2],prog_array[i][3],prog_array[i][4]);
      i++;
      break;
     case MUL:
      mult(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case DIV:
      divide(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case SUB:
      subtr(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case ADU:
      adcB(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case DAC:
      dacB(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case SUM:
      sum(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case INT:
      integrate(prog_array[i][1],prog_array[i][2],prog_array[i][3],prog_array[i][4]);
      i++;
      break;
     case PRT:
      text(prog_array[i][1],prog_array[i][2],prog_array[i][3],prog_array[i][4],prog_array[i][5]);
      i++;
      break;
     case PRF:
      textfast(prog_array[i][1]);
      i++;
      break;
     case SVF:
      filter(prog_array[i][1],prog_array[i][2],prog_array[i][3],prog_array[i][4],prog_array[i][5]);
      i++;
      break;
     case OSC:
      sine_osc(prog_array[i][1],prog_array[i][2],prog_array[i][3],prog_array[i][4]);
      i++;
      break;
     case DIF:
      differentiate(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case PWM:
      pwmout(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case RAL:
      rate_lim(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case SQT:
      sqroot(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case ABS:
      absval(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case ENC:
      readenc(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case MOT:
      mot(prog_array[i][1]);
      i++;
      break;
     case BRZ:
      if (val_array[prog_array[i][1]][1] == 0.0) i = i + round(val_array[prog_array[i][2]][1]);
      else i++;
      if (i < 0) i = 0;
      break;
     case BNZ:
      if (val_array[prog_array[i][1]][1] != 0.0) i = i + round(val_array[prog_array[i][2]][1]);
      else i++;
      if (i < 0) i = 0;
      break;
     case BRM:
      if (val_array[prog_array[i][1]][1] < 0.0) i = i + round(val_array[prog_array[i][2]][1]);
      else i++;
      if (i < 0) i = 0;
      break;
     case BRP:
      if (val_array[prog_array[i][1]][1] >= 0.0) i = i + round(val_array[prog_array[i][2]][1]);
      else i++;
      if (i < 0) i = 0;
      break;
     case OUT:
      dout(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case INP:
      dinp(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case MUX:
      amux(prog_array[i][1],prog_array[i][2],prog_array[i][3],prog_array[i][4]);
      i++;
      break;
     case RLY:
      relay(prog_array[i][1],prog_array[i][2],prog_array[i][3],prog_array[i][4]);
      i++;
      break; 
     case RST:
      resetvar(prog_array[i][1],prog_array[i][2]);
      i++;
      break;
     case DLA:
      delaytrig(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case NOC:
      noswitch(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case NCC:
      ncswitch(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case ANG:
      andgate(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;      
     case ORG:
      orgate(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break;
     case XOR:
      xorgate(prog_array[i][1],prog_array[i][2],prog_array[i][3]);
      i++;
      break; 
     case END:
      digitalWrite(TIMING_PIN,0);
      update_all();
      i = 0;
      if (tb >= abs(tmax)) {
        Serial.println(" ***** T = MAX ***** "); 
         pwm_set_chan_level(4,1,0);
         pwm_set_chan_level(5,0,0);
        return(0);
      }
      break;
     default:
      // if nothing else matches, do the default
      // default is optional
     break;
    }
    if (i > prog_count) return(-1);
   } 
  }
   pwm_set_chan_level(4,1,0);
   pwm_set_chan_level(5,0,0);
 Serial.println(" ***** ??? ***** "); 
 return(0); 
}

void textfast(int a1){
  float f;
  
  if (val_array[4][1] <= val_array[4][0]){
   val_array[4][0] = 0;    
   if (a1 != 0){
    f = (val_array[a1][1]);
    Serial.println(f,(int)val_array[5][1]);
   }
  } 
  val_array[4][0]++;
} 

void text(int a1, int a2, int a3, int a4, int a5){
  float f;
  int fmt;
  if (val_array[4][1] <= val_array[4][0]){
   val_array[4][0] = 0;
   fmt = (int)val_array[5][1];   
  if (a1 != 0){
    f = (val_array[a1][1]);
    Serial.print(f,fmt);
    Serial.print(" ");
    delay(1);
  }
  if (a2 != 0){
   f = (val_array[a2][1]);
   Serial.print(f,fmt);
   Serial.print(" ");
    delay(1);
  }
  if (a3 != 0){
   f = (val_array[a3][1]);
   Serial.print(f,fmt);
   Serial.print(" ");
    delay(1);
  }
  if (a4 != 0){
   f = (val_array[a4][1]);
   Serial.print(f,fmt);
   Serial.print(" ");  
    delay(1);
  }
  if (a5 != 0){
   f = (val_array[a5][1]);
   Serial.print(f,fmt);
    delay(1);
  }
  
  Serial.println();
  }
  val_array[4][0]++;
  //Serial.send_now();
}

void assign(int a1, float f1){
  val_array[a1][0] = f1;
  val_array[a1][1] = f1; 
  val_array[a1][2] = f1; 
}

void dout(int a1, int a2){
 int p = (int)val_array[a2][1];
 if ((p <= 14) && (p >= 4)) digitalWrite(p,val_array[a1][1] > 0.0); 
 if ((p == 21) || (p == 25)) digitalWrite(p,val_array[a1][1] > 0.0); 
}

void dinp(int a1, int a2){
 int p = (int)val_array[a1][1];
 if ((p >= 0) && (p <= 23))  val_array[a2][1] = digitalRead(p); 
}

void delaytrig(int a1, int a2, int s1){
  
 if ((val_array[s1][2] >= val_array[a2][1]) && (val_array[a1][1] > 0.0)) val_array[s1][1] = 1.0;
 else val_array[s1][1] = 0.0;

 if (val_array[a1][1] <= 0.0){
  val_array[s1][2] = 0.0;
  val_array[s1][1] = 0.0;    
 } else val_array[s1][2] = val_array[s1][2] + dT;
 
}

void andgate(int a1, int a2, int s1){  
 if ((val_array[a1][1] > 0.0) && (val_array[a2][1] > 0.0)) val_array[s1][1] = 1.0;
 else val_array[s1][1] = 0.0;
}

void orgate(int a1, int a2, int s1){  
 if ((val_array[a1][1] > 0.0) || (val_array[a2][1] > 0.0)) val_array[s1][1] = 1.0;
 else val_array[s1][1] = 0.0;
}

void xorgate(int a1, int a2, int s1){ 
 int bit1 = 0;
 int bit2 = 0; 
 if (val_array[a1][1] > 0.0) bit1 = 1;
 if (val_array[a2][1] > 0.0) bit2 = 1;
 val_array[s1][1] = (float)(bit1 ^ bit2);
}

void resetvar(int a1, int a2){
 val_array[a2][0]=val_array[a1][1]; 
 val_array[a2][1]=val_array[a1][1]; 
 val_array[a2][2]=val_array[a1][1]; 

 if (a2 == 1) {
  dT = val_array[1][0];
  dt_us = (int)(1000000*dT);
 }

 if (a2 == 2) tb = val_array[2][0];
 if (a2 == 3) tmax = val_array[3][0];
  
//   time_us = micros() + dt_us;

}

void amux(int a1, int a2, int c1, int s1){
 if (val_array[c1][1] <= 0.0) val_array[s1][1]=val_array[a1][1]; else val_array[s1][1]=val_array[a2][1]; 
}

void relay(int a1, int a2, int c1, int s1){
 if (val_array[c1][1] > 0.0){
  if ((val_array[s1][0] > 0.0) || (val_array[a1][1] > 0.0)) val_array[s1][0] = 1.0;
 } else {
  if ((val_array[s1][0] > 0.0) || (val_array[a2][1] > 0.0)) val_array[s1][0] = 1.0;
 }
}

void noswitch(int a1, int c1, int s1){
 if (val_array[c1][1] > 0.0){
  if ((val_array[s1][0] > 0.0) || (val_array[a1][1] > 0.0)) val_array[s1][0] = 1.0;
 }  
}

void ncswitch(int a1, int c1, int s1){
 if (val_array[c1][1] <= 0.0){
  if ((val_array[s1][0] > 0.0) || (val_array[a1][1] > 0.0)) val_array[s1][0] = 1.0;
 }  
}

void sum(int a1, int a2, int s1){
 val_array[s1][1]=val_array[a1][1]+val_array[a2][1]; 
}

void subtr(int a1, int a2, int s1){
 val_array[s1][1]=val_array[a1][1]-val_array[a2][1]; 
}

void mult(int a1, int a2, int s1){
 val_array[s1][1]=val_array[a1][1]*val_array[a2][1]; 
}

void divide(int a1, int a2, int s1){
 val_array[s1][1]=val_array[a1][1]/val_array[a2][1]; 
}

void integrate(int a1, int g_1, int l1, int s1){
 float tmp = val_array[a1][1]*dT*val_array[g_1][1];
 float limH = val_array[l1][1];
 float limL = -limH;
 if (limH <0.0) {
  limL = 0.0;
  limH = -limH;
 }
 val_array[s1][0]=(tmp+val_array[s1][2])*0.5+val_array[s1][1];
 val_array[s1][2] = tmp; 
 if (limH != 0.0){
  if (val_array[s1][0] > limH) val_array[s1][0] = limH;
  if (val_array[s1][0] < limL) val_array[s1][0] = limL;
 } 
}

void euler(int a1, int g_1, int l1, int s1){
 float limH = val_array[l1][1];
 float limL = -limH;
 if (limH <0.0) {
  limL = 0.0;
  limH = -limH;
 }
 val_array[s1][0]= val_array[s1][0]+val_array[a1][1]*val_array[g_1][1]*dT;
 if (limH != 0.0){
  if (val_array[s1][0] > limH) val_array[s1][0] = limH;
  if (val_array[s1][0] < limL) val_array[s1][0] = limL;
 } 
}

void differentiate(int a1, int f1, int d1){
 float f =  val_array[f1][1]; 
 val_array[d1][1]= val_array[d1][1]*f+((1-f)*(val_array[a1][1]-val_array[d1][2])/dT);
 val_array[d1][2]= val_array[a1][1];
}

void filter(int a1, int g_1, int g_2, int s1, int c1){
 val_array[s1][0]= val_array[s1][0] + (val_array[a1][1]-val_array[s1][0]-val_array[c1][0])*val_array[g_1][1]*dT;
 val_array[c1][0]= val_array[c1][0] + val_array[s1][0]*val_array[g_2][1]*dT;
}

void invz(int a1, int z1){
  val_array[z1][0]=val_array[a1][1];
}

void absval(int a1, int z1){
  val_array[z1][1]=abs(val_array[a1][1]);
}
void sqroot(int a1, int z1){
  float v = val_array[a1][1];
  if (v > 0.0) val_array[z1][1]=sqrt(v); else val_array[z1][1]=-sqrt(-v);
}

void limiter(int a1, int p1, int m1, int s1){
 val_array[s1][1] = val_array[a1][1];
 if (val_array[a1][1] > val_array[p1][1]) val_array[s1][1] = val_array[p1][1];
 if (val_array[a1][1] < val_array[m1][1]) val_array[s1][1] = val_array[m1][1]; 
}

void schmidt(int a1, int t1, int s1){
 if (val_array[a1][1] > val_array[t1][1]) val_array[s1][1]= -1.0;
 if (val_array[a1][1] < -val_array[t1][1]) val_array[s1][1]= 1.0;
}

void rate_lim(int a1, int r1, int s1){
 volatile float rd = (val_array[a1][1] - val_array[s1][1]);
 volatile float rL = val_array[r1][1]*dT;
 volatile float rp = rL;
 volatile float rm = -rL;
 if ((rd <= rp)&&(rd>=rm)) val_array[s1][1] = val_array[a1][1]; else {
   if (rd < 0.0) rL = -rL;
   val_array[s1][1] = val_array[s1][1] + rL;
 }
}

void adcB(int a1, int s1){         //adc(channel: 0-5, data out: 0.0V to 5.0V);
  int chan = int(val_array[a1][1]);
  if (chan < 0) chan = 0;
  if (chan > 2) chan = 2;
  int ch = chan+26;
  int avgs = (int)val_array[6][1];
  analog_avg = 0;
  for (int i =0; i < avgs; i++) analog_avg = analogRead(ch) + analog_avg; 
  if (chan == 0) val_array[s1][1] = (analog_avg*ADC_GAIN/float(avgs)) - adc0_os; 
  if (chan == 1) val_array[s1][1] = (analog_avg*ADC_GAIN/float(avgs)) - adc1_os; 
  if (chan == 2) val_array[s1][1] = analog_avg*0.0008057/float(avgs);
}

void dacB(int vin, int os){
   int ch; 
   int chan = int(val_array[os][1]);
   ch = int((val_array[vin][1]*51.2) + 512);    //cal here gain 51.2
   if (ch < 0) ch = 0;
   if (ch > 1023) ch = 1023;
  if (chan == 0){  
     pwm_set_chan_level(2,0, ch );
  }
  if (chan == 1){   
     pwm_set_chan_level(3,0, ch );
  }
}

void pwmout(int vin, int a1){
  int chan = int(val_array[a1][1]);
  int mc2 = int(val_array[vin][1]*65535.0);
  if (mc2>65535) mc2 = 65535;
  if (mc2<0) mc2 = 0;    
  if (chan == 4){  
     pwm_set_chan_level(2,0, mc2>>6 );
  }
  if (chan == 6){   
     pwm_set_chan_level(3,0, mc2>>6 );
  }
  if (chan == 9){   
     pwm_set_chan_level(4,1, mc2 );
  }
  if (chan == 10){   
     pwm_set_chan_level(5,0, mc2 );
  }
}

void readenc(int g, int n){
 if  (val_array[g][1] == 0) enc_count = 0;
 val_array[n][1] = val_array[g][1] * enc_count;
}

void mot(int vin ){
  float mc;
   mc = int(val_array[vin][1]*65535.0);
   if (mc<0){
    mc = -mc;
    if (mc>65535) mc = 65535;
     pwm_set_chan_level(4,1, 0 );
     pwm_set_chan_level(5,0, mc );
   } else {
    if (mc>65535) mc = 65535;
     pwm_set_chan_level(5,0, 0 );
     pwm_set_chan_level(4,1, mc );
   }
}

void square(int t1, int s1){
  val_array[t1][2] = val_array[t1][2] - dT;  
  if (val_array[t1][2] <= 0){
    val_array[s1][1] = -val_array[s1][1];
    val_array[t1][2] = val_array[t1][1];
  }
}

void sine_osc(int w1, int os, int mag, int s1){ 
 val_array[s1][1]= val_array[mag][1]*sin(val_array[w1][1]*tb)+val_array[os][1];
}

void update_all() {
  int i,j,k;
  for (i = 0; i<prog_count; i++){
    j = prog_array[i][0];
    if ((j==INT) || (j==EUL)|| (j==RLY)){
      k = prog_array[i][4];
      val_array[k][1] = val_array[k][0];
    }
    if ((j==NOC)||(j==NCC)) {
      k = prog_array[i][3];
      val_array[k][1] = val_array[k][0];
    }    
    if (j==SVF){
      k = prog_array[i][4];
      val_array[k][1] = val_array[k][0];
      k = prog_array[i][5];
      val_array[k][1] = val_array[k][0];     
    }    
    if (j==ZIN) { 
     k = prog_array[i][2];
     val_array[k][1] = val_array[k][0];       
    }  
  }
  
  for (i = 0; i<prog_count; i++){
    j = prog_array[i][0];
    if (j==RLY){
      k = prog_array[i][4];
      val_array[k][0] = 0.0;
    }  
    if ((j==NOC)||(j==NCC)){
      k = prog_array[i][3];
      val_array[k][0] = 0.0;
    } 
  }

  
  tb = tb + dT;
  
  val_array[2][0] = tb;
  val_array[2][1] = tb;
  if (tmax > 0.0){ 
    while (micros() < time_us);
    time_us = time_us+dt_us;
  } 
}

int sort_blocks(void) {
   int i,j,k, n;
   int x0,x1,x2,x3,x4,x5;
   int last_line = -1;
   //look for "END" in program, error if not found
   for (i = 0; i < prog_count; i++){ 
    if (prog_array[i][0] == END) last_line = i;
   }
   if (last_line < 0){
    Serial.print("***** END Missing *****");
    return(-1);
   }
 
   n = last_line-1;
   if (n <=0 ) return(0);

   //Can't sort if branch statements used
   for (i = 0; i < n; i++){
     k = prog_array[i][0];
     if ((k==BRZ)||(k==BNZ)||(k==BRP)||(k==BRM)){
      Serial.println("***** BRANCH Used, Can't Sort *****");
      return(-1);
     }
   }  


   //Move ZIN, EUL, SVF, INT to bottom of program  
   for (j = 0; j < n; j++) {
    for (i = 0; i < n; i++){
     k = prog_array[i][0];
     if (k==ZIN) bubble_down(i,n);
     if (k==EUL) bubble_down(i,n);
     if (k==SVF) bubble_down(i,n);
     if (k==INT) bubble_down(i,n);
     if (k==RLY) bubble_down(i,n);  
    }
   }
   
   //find input variable and see if an output variable matches
   //then move the input block below the output block
   for (j=n; j>=0; j--){
    k = prog_array[j][0];
    x0 = num_var[k];  //get output index
    if  ((k!=DAC)&&(k!=PRT)&&(k!=PRF)&&(k!=INT)&&(k!=ZIN)&&(k!=EUL)&&(k!=SVF)&&(k!=MOT)&&(k!=PWM)){
      for (i=(j-1);i>=0;i--) {
        x1 = 1; //search for first variable
        if ((prog_array[j][x0] == prog_array[i][1])) bubble_down(i,j);
      }
    }
   } 
   for (j=n; j>=0; j--){
    k = prog_array[j][0];
    x0 = num_var[k];  //get output index
    if  ((k!=DAC)&&(k!=PRT)&&(k!=PRF)&&(k!=INT)&&(k!=ZIN)&&(k!=EUL)&&(k!=SVF)&&(k!=MOT)&&(k!=PWM)){  
     for (i=(j-1);i>=0;i--) {
       k = prog_array[i][0];
       x1 = 2; //search for second variable
       if  ((k==SUM)||(k==SUB)||(k==MUL)||(k==MUL)||(k==DIV)){
        if ((prog_array[j][x0] == prog_array[i][2])) bubble_down(i,j);
       } 
     }     
    }
  }  
 return(0); 
}

void bubble_down(int st, int fin){
   int j;
   int x0,x1,x2,x3,x4,x5;
   if (st<fin){
      for (j = st; j < fin; j++){
       x0 = prog_array[j+1][0]; 
       x1 = prog_array[j+1][1]; 
       x2 = prog_array[j+1][2]; 
       x3 = prog_array[j+1][3]; 
       x4 = prog_array[j+1][4]; 
       x5 = prog_array[j+1][5]; 
       prog_array[j+1][0] = prog_array[j][0]; 
       prog_array[j+1][1] = prog_array[j][1]; 
       prog_array[j+1][2] = prog_array[j][2]; 
       prog_array[j+1][3] = prog_array[j][3]; 
       prog_array[j+1][4] = prog_array[j][4]; 
       prog_array[j+1][5] = prog_array[j][5]; 
       prog_array[j][0] = x0; 
       prog_array[j][1] = x1; 
       prog_array[j][2] = x2; 
       prog_array[j][3] = x3; 
       prog_array[j][4] = x4; 
       prog_array[j][5] = x5;       
      } 
   }   
}
