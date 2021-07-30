# Description 
Program is written in such a way that a callback fuction is triggered in every 20 sec interval. Unlike HAL_DELAY() function this allows microcontroller to process continously without holding. The timer status is sent to PC using UART communication through FTDI module. 

# PIN Connection 

|Pin | Function |
| ------ | ------ | 
|PC13 | Status LED for Multi Threading demo |
|PA2|UART_Tx|
|PA3|UART_Rx|