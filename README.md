# ADITI_Blynk101Hackster
ADITI: Affordable Diagnostics Enabled Thermal Incubator. Hackter Arduino 101 challenge

ADITI is a wearable body suit with thermal heating for premature birth children in developing countries. It monitors the Blood Oxygenation, HR, Body Temp, Respiration Rate
For SpO2, HR the MAX30100 sensor was used, For respiration sensing a custom Piezoresistive sensor was used. LM35 for Body Temperature.
It uses a 25V N-MOSFET to drive a LiTex heating textile at 9V using PWM (PID heating to be added)
Connections: 


<n>1)Connect SDA, SCL of MAX30100 to D0,D1 respectively and Vin,Gnd to 3.3V and Gnd.</n>

2)Connect the analog output of LM35 to A1 pin and power and Gnd to 5V and Gnd.

3)Connect one terminal of eeonyx to 5V and the other to A0, Connect a 6.8kohm resistor and 10uF filter cap between A1 and Gnd

4)Connect one terminal of the LiTex heating pad to the Positive terminal of the XL6009 Boost converter and the negative of heating pad to the drain of the IRL3713 NMOSFET 
  and connect source of MOSFET to the negative terminal of boost convertor. Connect gate to PWM pin D3.
  
5)Connect the ADS1292 ECG sheild  
  
Thanks to Oxullo interscans for awesome MAX30100 library!
