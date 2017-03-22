/*
ADITI: Affordable Diagnostics Enabled Thermal Incubator.
ADITI is a wearable body suit with thermal heating for premature birth children in developing countries. It monitors the Blood Oxygenation, HR, Body Temp, Respiration Rate
For SpO2, HR the MAX30100 sensor was used, For respiration sensing a custom Piezoresistive sensor was used. LM35 for Body Temperature.
It uses a 25V N-MOSFET to drive a LiTex heating textile at 9V using PWM (PID heating to be added)
Connections: 
1)Connect SDA, SCL of MAX30100 to D0,D1 respectively and Vin,Gnd to 3.3V and Gnd.
2)Connect the analog output of LM35 to A1 pin and power and Gnd to 5V and Gnd.
3)Connect one terminal of eeonyx to 5V and the other to A0, Connect a 6.8kohm resistor and 10uF filter cap between A1 and Gnd
4)Connect one terminal of the LiTex heating pad to the Positive terminal of the XL6009 Boost converter and the negative of heating pad to the drain of the IRL3713 NMOSFET 
  and connect source of MOSFET to the negative terminal of boost convertor. Connect gate to PWM pin D3.
5)Connect the ADS1292 ECG sheild  
  
DISCLAIMER **While I aim to make this more safe, accurate it this is experimental software (and pre-beta) at that.
It is not suitable for any particular purpose. No life-critical devices should be based on this software.**
Work by Vignesh Ravichandran (hello@rvignesh.xyz)
*/
#include "FIR.h"
#include "ads1292r.h"
//#include <SPI.h>
#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleCurieBLE.h>
#include <CurieBLE.h>

#include "MAX30100_PulseOximeter.h"
#define REPORTING_PERIOD_MS     2000  //Change depending on usage
PulseOximeter pox; //MAX30100 init
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "";
BLEPeripheral  blePeripheral;




uint32_t tsLastReport = 0;

#define FILTERTAPS 5
#define	CES_CMDIF_PKT_START_1		0x0A
#define	CES_CMDIF_PKT_START_2		0xFA
#define	CES_CMDIF_TYPE_DATA		0x02
#define	CES_CMDIF_PKT_STOP		0x0B

ads1292r ADS1292; //init ads1292 ECG AFE
FIR fir; //FIR filter

volatile char DataPacketHeader[5];
volatile char DataPacketFooter[2];
volatile int datalen = 135;
unsigned long timet;

volatile byte SPI_RX_Buff[150] ;
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile int Responsebyte = false;
volatile unsigned int pckt =0 , buff=0,t=0 , l1=0,l2=0;
volatile unsigned long int EEG_Ch1_Data[150],EEG_Ch2_Data[150];
volatile unsigned char datac[150];
unsigned long ueegtemp = 0,Pkt_Counter=0;
signed long seegtemp=0;
 volatile int i;
 float feegtemp ,output;
int peakValue = 0;
int threshold = 63;   //set your own value based on your sensors
long peaktime=0;
long timediff=0;
long lastpeak=0;
long currenttime;

void setup() {

          blePeripheral.setLocalName("ADITI");
        blePeripheral.setDeviceName("ADITI");
        blePeripheral.setAppearance(384); 
        Blynk.begin(auth, blePeripheral);
        blePeripheral.begin();

        // initalize the  data ready and chip select pins:
        pinMode(ADS1292_DRDY_PIN, INPUT);  //6
        pinMode(ADS1292_CS_PIN, OUTPUT);    //7
        pinMode(ADS1292_START_PIN, OUTPUT);  //5
        pinMode(ADS1292_PWDN_PIN, OUTPUT);  //4
        pinMode(3, OUTPUT);
//        pinMode(4, OUTPUT);
//        digitalWrite(4, LOW);

        pox.begin();
        //initalize ADS1292 slave
        ADS1292.ads1292_Init();
       //ADS1292.ads1292_Reset();

        DataPacketHeader[0] = CES_CMDIF_PKT_START_1;
        DataPacketHeader[1] = CES_CMDIF_PKT_START_2;
        DataPacketHeader[2] = (datalen);
        DataPacketHeader[3] = (datalen >> 8);
        DataPacketHeader[4] = CES_CMDIF_TYPE_DATA;

        DataPacketFooter[0] = 0x00;
        DataPacketFooter[1] = CES_CMDIF_PKT_STOP;

	// declare variables for coefficients
	// these should be calculated by hand, or using a tool
	// in case a phase linear filter is required, the coefficients are symmetric
	// for time optimization it seems best to enter symmetric values like below
	float coef[FILTERTAPS] = { 0.021, 0.096, 0.146, 0.096, 0.021};
	fir.setCoefficients(coef);

	  //declare gain coefficient to scale the output back to normal
	float gain = 1; // set to 1 and input unity to see what this needs to be
	fir.setGain(gain);
}
 BLYNK_WRITE(V4)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  // You can also use:
  // String i = param.asStr();
  // double d = param.asDouble();
    analogWrite(3, pinValue);
}

void loop() {
  Blynk.run();
 currenttime=millis();
 int sensorValue = analogRead(A0);  //Read eeonyx sensor value
  if (sensorValue > peakValue) {
    peakValue = sensorValue;
    peaktime=millis();
  }
  if (sensorValue <peakValue) {
    
    if (sensorValue < threshold) {
      // you have a peak value:
      timediff=peaktime-lastpeak;
      lastpeak= peaktime;
      // reset the peak value:
      peakValue = 0;

    }
  }
  int respirations = 60000/timediff; //Respiration rate calculation
  pox.update();
      if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Blynk.virtualWrite(V0,pox.getHeartRate()); //Heart Rate
        Blynk.virtualWrite(V1,pox.getSpO2()); //spo2
        Blynk.virtualWrite(V2,((5.0 * analogRead(A1) * 100.0) / 1024)); // LM35 temperature formula 
        Blynk.virtualWrite(V3,respirations);  //Respiration Rate

        
        //Serial.print(pox.getHeartRate());
       // Serial.print(" ");
       // Serial.println(pox.getSpO2());
        tsLastReport = millis();
    }
     ReadAdc();
     Serial.println(output); //ECG output (See ECG in serial plotter)
     blePeripheral.poll();

}

float Filter(float fInput) {
    static float fLast = 0.0f;
    int a = -1;
    float fResult = fInput+a*fLast;
    fLast = fInput;
    return 0.5*fResult;
    }

void ReadAdc()
{
 if((digitalRead(ADS1292_DRDY_PIN)) == LOW)
  {
    SPI_RX_Buff_Ptr = ADS1292.ads1292_Read_Data();
    Responsebyte = true;

  }

  if(Responsebyte == true)
  {
     for(i = 0; i < 9; i++)
     {
       SPI_RX_Buff[SPI_RX_Buff_Count++] = *(SPI_RX_Buff_Ptr + i);
     }
     Responsebyte = false;
  }

  if(SPI_RX_Buff_Count >= 9)
  {


    	pckt = 0;	l1=0;		l2=0;
	for(i=3;i<9;i+=9)
	{

		EEG_Ch2_Data[l2++]= (unsigned char)SPI_RX_Buff[i+3];
		EEG_Ch2_Data[l2++]= (unsigned char)SPI_RX_Buff[i+4];
		EEG_Ch2_Data[l2++]= (unsigned char)SPI_RX_Buff[i+5];

	}

		for(t=0; t< 1 ; t++)
		{
			buff = 0;
			Pkt_Counter++; //if(Pkt_Counter > 0) Pkt_Counter = 0x00;

			datac[buff++] = 0xA0;  // sync0
			datac[buff++] = 36;   //sync1

			datac[buff++] = (unsigned char)(Pkt_Counter >> 24);
			datac[buff++] = (unsigned char)(Pkt_Counter >> 16);
			datac[buff++] = (unsigned char)(Pkt_Counter >> 8);
			datac[buff++] = (unsigned char)(Pkt_Counter );

                        ueegtemp = (unsigned long) ((EEG_Ch2_Data[pckt]<<16)|(EEG_Ch2_Data[pckt+1]<<8)|EEG_Ch2_Data[pckt+2]);


            		ueegtemp = (unsigned long) (ueegtemp<<8);
			seegtemp = (signed long) (ueegtemp);
			seegtemp = (signed long) (seegtemp>>8);
                        //feegtemp = (float)((seegtemp)* 2.23517E-05 );
                        feegtemp = (float)((seegtemp*50));
                      
                        output = fir.process(feegtemp);
                   //    Serial.println(output);   
                   //     Serial.print("  ");  
                    //    Serial.println(feegtemp);                                         
                        //output = Filter(output);
                        //seegtemp = (signed long)((output)/ 2.23517E05 );
                        seegtemp = (signed long)((output));
                        //seegtemp = abs(seegtemp);

			pckt+= 3;

			datac[buff++] = (unsigned char) (seegtemp);
			datac[buff++] = (unsigned char) (seegtemp>>8);   //>>8
			datac[buff++] = ( unsigned char) (seegtemp >> 16); //>>16
			datac[buff++] = (unsigned char) (seegtemp >>24);

                    //    Serial.println(seegtemp);

			for(i=0;i<28;i++)  // fill channel 1 buff
			{
	        		datac[buff++] = 0x00 ;
			}

				datac[buff++] = 0xc0;//(uint8_t)seegtemp ;


			for(i=0; i<39; i++) // transmit the data
			{

                    //     Serial.write(datac[i]);

			}

		}

		SPI_RX_Buff_Count = 0;


	}


}
