# DroneRF_Project

 Final Design Document: https://drive.google.com/file/d/1hLyYco8pAcUMKnUi00cZ6kdRbGmCKY5S/view?usp=sharing
 
Sub Ghz radio using TI CC1312R Launchpad.
- **DroneTx**: RF TX; stages JPEG from UART.
- **GroundStation**: RF RX; reassembles and saves JPEG.

## Build
Open each folder as a CCS project; build with SimpleLink SDK + FreeRTOS.
