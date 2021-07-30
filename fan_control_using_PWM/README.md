# Description 
The voltage across potentiomenter is sensed by analog pin and equivalent PWM signal is generated for Motor control. The ADC data is transferred to PC using UART communication through buffer.

# PIN Connection 
|Pin | Function |
| ------ | ------ |
|PA2 |UART2_Tx|
|PA3 |UART2_Rx|
|PA4 |To Potentiometer for PWM|
|PA0 |PWM signal generation|
|PC13| Status LED |