#include <wiringPi.h>
#include <mcp3004.h>
#include <stdio.h>
#include <stdlib.h>
#include <mcp4802.h>
#include <wiringPiSPI.h>
#include "mcp4802.h"

#define MCP3008_PIN_BASE 100
#define MCP3008_SPI_CHAN 0
#define MCP3008_SPI_CHANB 1
#define MCP3008_SPI_CLK_RATE 100000
#define MCP4802_PIN_BASE 200
#define MCP4802_SPI_CHAN 1
#define DELAYTIME 1
#define DAC_RESOLUTION 4095 //12 bit mcp4822: 4096 - 1
#define ADC_RESOLUTION 1023 //10 bit-1


int main(void)
{
    mcp3004Setup(MCP3008_PIN_BASE, MCP3008_SPI_CHAN);
    mcp4822Setup(MCP4802_PIN_BASE, MCP4802_SPI_CHAN);

    FILE *fp;
    int i=0;
    float r=56;


    fp = fopen("result.txt", "w+");

    for (int i=0;i<DAC_RESOLUTION;i++) {

        analogWrite(MCP4802_PIN_BASE, i);

        int x = analogRead(MCP3008_PIN_BASE + MCP3008_SPI_CHAN);
	      int z = analogRead(MCP3008_PIN_BASE + MCP3008_SPI_CHANB);

       float u1 = (3.3/ADC_RESOLUTION)*x;
	     float u2 = (3.3/ADC_RESOLUTION)*z;
  	   float current = ((u1-u2)/r)*1000;

       printf("U1: %f V - U2: %f V - DAC: %i - I: %f mA\n", u1, u2, i, current);
       fprintf(fp,"%f\t%f\n",current,u2);

       delay(DELAYTIME); //For

    }
    analogWrite((MCP4802_PIN_BASE + MCP3008_SPI_CHAN), 0);
    fclose(fp);

}


static void mcp4822AnalogWrite (struct wiringPiNodeStruct *node, int pin, int value)
{
  unsigned char spiData [2] ;
  unsigned char chanBits, dataBits ;
  int chan = pin - node->pinBase ;

  if (chan == 0)
    chanBits = 0x30 ;
  else
    chanBits = 0xB0 ;

  chanBits |= ((value >> 8) & 0x0F) ; //4822: 8 Chan Bits
  dataBits  = ((value << 4) & 0xF0) ;

  spiData [0] = chanBits ;
  spiData [1] = dataBits ;

  wiringPiSPIDataRW (node->fd, spiData, 2) ;
}


int mcp4822Setup (const int pinBase, int spiChannel)
{
  struct wiringPiNodeStruct *node ;

  if (wiringPiSPISetup (spiChannel, 1000000) < 0)
    return FALSE ;

  node = wiringPiNewNode (pinBase, 2) ;

  node->fd          = spiChannel ;
  node->analogWrite = myAnalogWrite ;

  return TRUE ;
}
