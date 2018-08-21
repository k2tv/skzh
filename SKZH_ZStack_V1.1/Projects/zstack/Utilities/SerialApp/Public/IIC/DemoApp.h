/**************************************************************************************************
  Filename:       DemoApp.h
 
  Description:    Header file for the Sensor Demo application. This application
                  has two device types; Collectors and Sensors. 

                  A collector device functions as Routers and may be configured
                  in gateway mode to accept binding requests from Sensors. The
                  gateway node also forward received sensor reports to its
                  serial port. 

                  The sensors will periodically report sensor data to the 
                  gateway in the system. 


**************************************************************************************************/


#ifndef SIMPLE_APP_H
#define SIMPLE_APP_H

/******************************************************************************
 * CONSTANTS
 */

#define MY_PROFILE_ID                     0x0F20
#define MY_ENDPOINT_ID                    0x02

#define MY_DEVICE_ID                      0x02  //add by wu 2010.9.20

// Define devices
#define DEV_ID_SENSOR                     1
#define DEV_ID_COLLECTOR                  2

#define DEVICE_VERSION_SENSOR             1
#define DEVICE_VERSION_COLLECTOR          1

// Define the Command ID's used in this application
#define SENSOR_REPORT_CMD_ID              2
#define DUMMY_REPORT_CMD_ID               3

#define RX_BUF_LEN                        128

/******************************************************************************
 * PUBLIC FUNCTIONS
 */

void initUart(halUARTCBack_t pf,uint8 UARTBR);
void uartRxCB( uint8 port, uint8 event );

#endif // SIMPLE_APP_H






