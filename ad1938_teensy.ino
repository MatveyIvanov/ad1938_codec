#include <Audio.h>
#include <Wire.h>
#include <SPI.h>

#include"control_ad1938.h"

/*
AD1938 pins  Teensy gpio pins
Chip select - 7
MOSI        - 11
MISO         -12
SCK          -14 
RESET        -17

DAC/ADCBLK     -9
DAC/ADCLRCLK   -23

DSDATA1 TX   -22
ASDARA1 RX   -13
*/

/*
The daisy chain connection of two ad1938 and Teensy are as follows
AD1938 (slave)->AD1938(master)->Teensy(slave)
*/

int clatch_slave =10;//ad1938 slave spi latch
int clatch =10;       //ad1938 master spi latch
int cout=12;         //spi miso
int cin =11;         //spi  mosi
int cclk =13;        //spi clock
int reset_pin_slave =17; //ad1938 slave reset pin
int reset_pin =17;        //ad1938 master reset pin

AudioControlAD1938       ad1938master;
AudioControlAD1938       ad1938slave;

//only for slave ++
AudioInputI2Sslave    i2s_in;
AudioOutputI2Sslave   i2s_out;
//only for slave --

//AudioSynthWaveformSine   sine1;

//only for ad138 master  and teensy slave++
//AudioInputTDMslave   i2s_in;
//AudioOutputTDMslave  i2s_out;
//only for master --

/*ADC0 to DAC0*/
AudioConnection          pc0(i2s_in, 0, i2s_out, 0);  // ADC1 L -> DAC1 L
AudioConnection          pc2(i2s_in, 2, i2s_out, 2);  // ADC1 R -> DAC1 R 

/*ADC1 to DAC1*/
//AudioConnection          pc4(i2s_in, 4, i2s_out, 4);  // ADC2 L -> DAC2 L
//AudioConnection          pc6(i2s_in, 6, i2s_out, 6);  // ADC2 R -> DAC2 R 

/*ADC2 to DAC2*/
//AudioConnection          pc8(i2s_in, 8, i2s_out, 12);    // ADC3 L -> DAC3 L
//AudioConnection          pc10(i2s_in,10, i2s_out, 14);  // ADC3 R -> DAC3 R 

/*ADC3 to DAC3*/
//AudioConnection          pc12(i2s_in, 12, i2s_out, 8); // ADC4 L -> DAC4 L
//AudioConnection          pc14(i2s_in, 14, i2s_out, 10); // ADC4 R -> DAC4 R 
//TMD16
/*sine to DAC4*/
//AudioConnection          pc16(i2s_in, 0, i2s_out, 16); // ADC1 L -> DAC5 L 
//AudioConnection          pc18(i2s_in, 2, i2s_out, 18); // ADC1 R -> DAC5 R 
/*sine to DAC5*/
//AudioConnection          pc20(i2s_in, 4, i2s_out, 20); // ADC2 L -> DAC6 L
//AudioConnection          pc22(i2s_in, 6, i2s_out, 22); // ADC2 R -> DAC6 R 

/*sine to DAC6*/
//AudioConnection          pc24(i2s_in, 0, i2s_out, 24); // sine -> DAC7 L
//AudioConnection          pc26(i2s_in , 2 , i2s_out, 26); // sine -> DAC7 R 
/*sine to DAC 7*/
//AudioConnection          pc28(sine1, 0, i2s_out, 28); // sine -> DAC8 L
//AudioConnection          pc30(sine1, 0, i2s_out, 30); // sine -> DAC8 R 



void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  //pinMode(21, OUTPUT);
  //pinMode(23, OUTPUT);
  //pinMode(8, INPUT);
  //pinMode(7, OUTPUT);
  //pinMode(20, OUTPUT);

 //pinMode(18, INPUT);
 //pinMode(16, INPUT);


  pinMode(14, INPUT);

 // put your setup code here, to run once:
 delay(2000);
  //sine1.frequency(440);
  //sine1.amplitude(0.4);
 #if 0
  /*configure AD1938 (slave) wit */
  ad1938master.spiInit( clatch, reset_pin, cout, cin, cclk);
  delay(200);
  ad1938master.config(FS_48000,BITS_24,I2S_TDM_8CH,AD1938_I2S_MASTER);
  ad1938master.config(FS_48000,BITS_16,I2S_TDM_16CH,AD1938_I2S_MASTER);
  ad1938master.volume(1);
  ad1938master.enable();
  delay(2000);
 //ad1938master.readAllreg();

 #endif

 
 #if 1
  ad1938slave.spiInit(clatch_slave, reset_pin_slave, cout, cin, cclk);
  delay(200);
  //ad1938slave.config(FS_48000,BITS_24,I2S_TDM_8CH,AD1938_I2S_SLAVE);
  ad1938slave.config(FS_48000,BITS_16,I2S_STEREO_2CH,AD1938_I2S_SLAVE);
  ad1938slave.volume(1);
  ad1938slave.enable();
  delay(2000);
  //ad1938slave.dacMute(false);
  //ad1938slave.adcMute(false);
  ad1938slave.readAllreg();

#endif 

#if 0
  /*configure AD1938 (slave) wit */
  ad1938master.spiInit( clatch, reset_pin, cout, cin, cclk);
  delay(2000);
  ad1938master.readAllreg();
  delay(200);
  //ad1938master.config(FS_48000,BITS_24,I2S_TDM_8CH,AD1938_I2S_MASTER);
  ad1938master.config(FS_48000,BITS_16,I2S_STEREO_2CH,AD1938_I2S_MASTER);
  ad1938master.volume(1);
  ad1938master.enable();
  //ad1938master.dacMute(false);
  //ad1938master.adcMute(false);
  delay(2000);
  ad1938master.readAllreg();

 #endif
  AudioMemory(256); 
  AudioInterrupts();
  
 AudioProcessorUsageMaxReset();
 AudioMemoryUsageMaxReset();
}

int write = LOW;
int read;
int rx;
int tx;

void loop() {
  Serial.println("________________I2S_________________");
  Serial.print("BCLK: ");
  Serial.println(digitalRead(21));
  Serial.print("LRCLK: ");
  Serial.println(digitalRead(20));
  Serial.print("MCLK: ");
  Serial.println(digitalRead(23));
  Serial.println("________________I2S_________________");
  Serial.println("");
   read = analogRead(14) / 4;
   Serial.print("ANALOG INPUT: ");
    Serial.println(read);
  /*
    write = (write == LOW) ? HIGH: LOW;
    digitalWrite(15, write);
    Serial.print("\n\n");
    Serial.print("analog write LN is: ");
    Serial.println(write);
    digitalWrite(14, write);
    Serial.print("analog write LP is: ");
    Serial.println(write);
  */
    rx = analogRead(8);
    Serial.print("Reciever (A): ");
    Serial.println(rx / 4);
    rx = digitalRead(8);
    Serial.print("Reciever: ");
    Serial.println(rx);
  
    digitalWrite(7, rx);
    tx = digitalRead(7);
    Serial.print("Transmitter: ");
    Serial.println(tx);
    
    read = analogRead(16) / 4;
    Serial.print("analog R read is: ");
    Serial.println(read);
    read = analogRead(18) / 4;
    Serial.print("analog L read is: ");
    Serial.println(read);
    Serial.print("\n");
    delay(1000); 
  // put your main code here, to run repeatedly:
  /*
  Serial.print("\n");
  Serial.print("CPU Peak CPU=");
  Serial.print(AudioProcessorUsage());
  Serial.print(",");
  Serial.print(AudioProcessorUsageMax());
  Serial.print("    ");
  Serial.print("Memory: ");
  Serial.print(AudioMemoryUsage());
  Serial.print(",");
  Serial.print(AudioMemoryUsageMax());
  Serial.print(",");
  Serial.print(i2s_out.processorUsageMax());
  delay(1000);
  */
}
