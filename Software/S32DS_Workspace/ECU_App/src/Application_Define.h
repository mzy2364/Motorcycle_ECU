/*
 * Application_Define.h
 *
 *  Created on: 2023年3月4日
 *      Author: mzy2364
 */

#ifndef APPLICATION_DEFINE_H_
#define APPLICATION_DEFINE_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"

//定义曲轴上面的齿数
#define NUMBER_OF_TEETH 	24
//四冲程一个循环的齿数
#define NUMBER_OF_TEETH_4C NUMBER_OF_TEETH * 2
//每个齿对应的度数
#define DEGREES_PER_TOOTH (360 / NUMBER_OF_TEETH)


  //Indicate what tooth represents TDC based on detection method above.
  //Use zero as reference for missing tooth.  Tooth one define as first detected
  //edge after the missing tooth gap.  Tooth count goes from 0 to NUMBER_OF_TEETH-1
  #define TDC_TOOTH 9
    //This results in an angle relative to TDC to be used in the application.
    #define TDC_ANGLE (TDC_TOOTH * DEGREES_PER_TOOTH)
    
  //Specify the orientation between cylinders 1 and 2 if it is a two cylinder engine.
  #ifdef TWO_CYLINDER
  //Standard orientation is 180 degrees for two stroke.
  //Standard orientation is 360 degrees for four stroke.
  #define CYLINDER_OFFSET_ANGLE 180
  #endif

  //Define how many spark events per cycle are performed.
  //Currently only 1 event supported
  #define SPARK_EVENTS_NUM 1

  //Define the angle when fuel injection starts in degrees.  This is relative to the 
  //missing tooth.  Application uses fixed start time for fuel injection 
  //and makes the pulse time from this angle.  
  //Typical application will use TDC.    
  #define FUEL1_REF_ANGLE  TDC_ANGLE
    //This results in an angle for when the fuel is injector for the 
    //second cylinder if it is defined.
    #ifdef TWO_CYLINDER
      //For a two stroke engine, this is just the offset.
      #ifdef TWO_STROKES
        #define FUEL2_REF_ANGLE  (TDC_ANGLE + CYLINDER_OFFSET_ANGLE)
      #endif
      //For a four stroke engine, we have to compensate as we need 
      //a reference angle to start the engine in two stroke mode.  
      //A separate reference angle is created later for when the application
      //is synchronized to the four stroke process.  
      #ifdef FOUR_STROKES
        #define FUEL2_REF_ANGLE  (FUEL1_REF_ANGLE + CYLINDER_OFFSET_ANGLE - 180)
        //This definition only works for the 360 degree offset case.  
      #endif      
    #endif
    
    
    //This also results in a secondary reference angle if four stroke 
    //application is defined.
    #ifdef FOUR_STROKES
      //Create the reference angle for four stroke operation.
      #define FUEL1_REF_ANGLE_4C  FUEL1_REF_ANGLE
      #ifdef TWO_CYLINDER
        //Create the reference angle for four stroke operation.
        #define FUEL2_REF_ANGLE_4C  FUEL1_REF_ANGLE + CYLINDER_OFFSET_ANGLE
      #endif
    #endif


//Set the engine running parameters
//Engine running parameters
//Minimum engine speed that defines the engine as running.
//This is tooth period counts based on 1.6us timer tic.
#define Stall_Speed 300 // 500 RPM

//Maximum engine speed that defines overrunning.
//This is tooth period counts based on 1.6us timer tic.
#define Over_Speed 8000   // 8000 RPM

//Engine speed required to recover from an over speed condition
//This is tooth period counts based on 1.6us timer tic.
#define Over_Speed_Recovery 416 //7500 RPM
   
   
/*Crankshaft Synchronization Parameters*****************************************/

  //Set the minimum RPM for engine rotation
  #define RPM_MIN 200
    //This results in a maximum tooth period that is valid.
    //Application uses this value in terms of timer tics.
                                
  //Set the maximum RPM for engine rotation
  #define RPM_MAX 9000
    //This results in a minimum tooth period that is valid.
    //Application uses this value in terms of timer tics.
      
  //Specify how much the tooth period can vary from tooth to tooth.
  //This is the tooth tolerance and is based on manufacturing of the 
  //teeth on the flywheel and the pulsation of the engine.  
  #define POS_PERIOD_PERCENTAGE 50
  #define NEG_PERIOD_PERCENTAGE 50
    
  //Specify the length of time before synchronization is lost when a 
  //tooth is not detected, based on the last tooth period.
  #define CRANKSHAFT_TIMEOUT_PERCENTAGE 80  
  //A default timeout is set for conditions prior to getting the first 
  //tooth period time.  This is set for the minimum RPM defined.    

  //Specify what gap ratio is for a missing tooth compared to a normal tooth.
  //This is typically 2 for a single missing tooth.
  //This results in minimum and maximum values of the gap ratio based on
  //tooth tolerance specified above.
  #define GAP_RATIO 2  
  

/*Fuel and Spark table lookup parameters************************************/
  
  //Fuel Table Parameters
  //Specify the size of the data used in the fuel table.
  //Default is for 8 bit data unless you define 16 bit data using 
  //the define below.  
  #define Fuel_Pulse_Data_16_bit
  //Specify how many load points are in the fuel map.
  #define FUEL_LOAD_POINTS 12
  //Specify how many RPM points there are in the fuel map.
  #define FUEL_RPM_POINTS 31
  //Make sure you keep the table size resonable.  You only have so much 
  //memory and the larger it is, the longer the look up time.  
  //Values for the load and RPM points must be placed in Application Map.c.
  //Actual fuel table data is placed in Application Map.c as an array.
  
  
  //Spark Table Parameters
  //Specify the size of the data used in the spark timing table.
  //Default is for 8 bit data unless you define 16 bit data using 
  //the define below.  
  #define Spark_Data_16_bit
  //Specify how many load points are in the spark map.
  #define SPARK_LOAD_POINTS 4
  //Specify how many RPM points are in the spark map.  
  #define SPARK_RPM_POINTS 31
  //Make sure you keep the table size resonable.  You only have so much 
  //memory and the larger it is, the longer the look up time.  
  //Values for the load and RPM points must be placed in Application Map.c.
  //Actual spark table data is placed in Application Map.c as an array.

  //Dwell Table Parameters
  //Specify the size of the data used in the dwell time table.
  //Default is for 8 bit data unless you define 16 bit data using 
  //the define below.  
  #define Dwell_Data_16_bit
  //Specify how many RPM points are in the spark map.  
  #define DWELL_RPM_POINTS 5
  //Make sure you keep the table size resonable.  You only have so much 
  //memory and the larger it is, the longer the look up time.  
  //Values for the RPM points must be placed in Application Map.c.
  //Actual dwell table data is placed in Application Map.c as an array.




/*Analog Signal Definitions********************************************************/
   
   //Analog signals collection rate
  //in milliseconds. Can be 1 - 255 ms
  #define ANALOG_SIGNALS_COLLECTION_RATE 1
   
  //  
  #define DATA_MANAGEMENT_TASK  1

  //Choose the data type used for collecting analog signal data.  If using 10 or 12 bit,
  //the data will be stored as a 16 bit value and use more twice the memory of 8 bit data.  
  //Select only one type.
  //#define Analog_Data_8
  //#define Analog_Data_10
  #define Analog_Data_12
      
  //Manifold Absolute Pressure(MAP)
  //This is a special analog signal as you can measure it in a time based
  //method or in a tooth based method.  Tooth based is used for synchronizing
  //a single cylinder 4 stroke application.  May applications may also measure   
  //MAP in this manor as it can reduce the variabilitiy of the measurement.    
  //Define the signal for the system to enable functionality.
  #define MAP
  //Define data collection method for MAP measurements.  Choose one.
  //#define MAP_TIME_BASED
  #define MAP_TOOTH_BASED
  //Define for MAP filter algorithm selection.  Only average is available.
  //Leave undefined for using raw data only. 
  //#define MAP_AVERAGE_FILTER
  //If the data is being collected through a time based method, specify the rate in ms.
  //Data collection periodic rate can be from 1 - 255ms.
  #ifdef MAP_TIME_BASED
  #define MAP_DATA_COLLECTION_RATE 1
  #endif
  //If the data is being collected through a tooth based method, specify the tooth range.
  //Data collection tooth start and end must be specified.  Teeth are counted from 
  //0 to NUMBER_OF_TEETH-1.  
  #ifdef MAP_TOOTH_BASED
    #define MAP_TOOTH_START 11
    #define MAP_TOOTH_END 18
  #endif
  
  // MAP data buffer size
  //Must be same as number of teeth data is collected if tooth based. 
  #ifdef MAP_TOOTH_BASED
    #define MAP_BUFFER_SIZE MAP_TOOTH_END - MAP_TOOTH_START + 1
  #else 
    //Else define size of buffer here for time based.
    #define MAP_BUFFER_SIZE 4
  #endif
  
  //Define the decrease in MAP data from tooth to tooth for valid signature on intake cycle.
  //Recommended greater than 1.  Use actual value based on sensor and engine used.  
  #ifdef MAP_TOOTH_BASED
    #define MAP_SIGNATURE_DROP_MIN 30
  #endif  

  //Mass Air Flow(MAF)
  //Define the signal for the system to enable functionality.
  //#define MAF
  //Define for MAF filter algorithm selection.  Only average is available.
  //Leave undefined for using raw data only. 
  //#define MAF_AVERAGE_FILTER
  //Data collection periodic rate can be from 1 - 255ms.
  //#define MAF_DATA_COLLECTION_RATE 16
  // MAF data buffer size 
  //#define MAF_BUFFER_SIZE 16

  //Oxygen Sensor(O2)
  //Define the signal for the system to enable functionality.
  //#define O2
  //Define for O2 filter algorithm selection.  Only average is available.
  //Leave undefined for using raw data only. 
  //#define O2_AVERAGE_FILTER
  //Data collection periodic rate can be from 1 - 255ms.
  //#define O2_DATA_COLLECTION_RATE 16
  // O2 data buffer size 
  //#define O2_BUFFER_SIZE 16

  //Throttle Position Sensor(TPS)
  //Define the signal for the system to enable functionality.
  #define TPS
  //Define for TPS filter algorithm selection.  Only average is available.
  //Leave undefined for using raw data only. 
  #define TPS_FILTER
  //Data collection periodic rate can be from 1 - 255ms.
  #define TPS_DATA_COLLECTION_RATE 6
  // TPS data buffer size 
  #define TPS_BUFFER_SIZE 16
  //TPS data filter type
  #define TPS_AVERAGE_FILTER

  //Engine Temperature(ETEMP)
  //Define the signal for the system to enable functionality.
  #define ETEMP
  //Define for ETEMP filter algorithm selection.  Only average is available.
  //Leave undefined for using raw data only. 
  #define ETEMP_AVERAGE_FILTER
  //Data collection periodic rate can be from 1 - 255ms.
  #define ETEMP_DATA_COLLECTION_RATE 1
  // ETEMP data buffer size 
  #define ETEMP_BUFFER_SIZE 16
  
  //Air Temperature(ATEMP)
  //Define the signal for the system to enable functionality.
  #define ATEMP
  //Define for ATEMP filter algorithm selection.  Only average is available.
  //Leave undefined for using raw data only. 
  #define ATEMP_AVERAGE_FILTER
  //Data collection periodic rate can be from 1 - 255ms.
  #define ATEMP_DATA_COLLECTION_RATE 16
  // ATEMP data buffer size 
  #define ATEMP_BUFFER_SIZE 32

  //Barametric Air Pressure(BAP)
  //Define the signal for the system to enable functionality.
  #define BAP
  //Define for BAP filter algorithm selection.  Only average is available.
  //Leave undefined for using raw data only. 
  #define BAP_AVERAGE_FILTER
  //Data collection periodic rate can be from 1 - 255ms.
  #define BAP_DATA_COLLECTION_RATE 16
  // BAP data buffer size 
  #define BAP_BUFFER_SIZE 16

  //Battery Voltage(VBAT)
  //Define the signal for the system to enable functionality.
  #define VBAT
  //Define for VBAT filter algorithm selection.  Only average is available.
  //Leave undefined for using raw data only. 
  #define VBAT_AVERAGE_FILTER
  //Data collection periodic rate can be from 1 - 255ms.
  #define VBAT_DATA_COLLECTION_RATE 16
  // VBAT data buffer size
  #define VBAT_BUFFER_SIZE 16
  
  //Select if transient detection (TIP) is to be used
//#define TIP_Detection


/*Digital Signal Definitions****************************************************/

    //Digital signals collection rate
  //in milliseconds. Can be 1 - 255 ms
  #define DIGITAL_SIGNALS_COLLECTION_RATE 1
  
  //Oil Pressure Switch(OPS)
  //Define the signal for the system to enable functionality.
  //#define OPS
  //No filter algorithm to specifiy.  Filtering performed by filling the buffer.
  //When the buffer is full of the same value, it changes to that value.  
  //Data collection periodic rate can be from 1 - 255ms.
  //#define OPS_DATA_COLLECTION_RATE 100
  //OPS data buffer size
  //#define OPS_BUFFER_SIZE 3

  //Ignition Switch(IGNSW)
  //Define the signal for the system to enable functionality.
  //#define IGNSW
  //No filter algorithm to specifiy.  Filtering performed by filling the buffer.
  //When the buffer is full of the same value, it changes to that value.  
  //Data collection periodic rate can be from 1 - 255ms.
  //#define IGNSW_DATA_COLLECTION_RATE 100
  //IGNSW data buffer size
  //#define IGNSW_BUFFER_SIZE 3

  //Kickstand Switch(KICKSW)
  //Define the signal for the system to enable functionality.
  //#define KICKSW
  //No filter algorithm to specifiy.  Filtering performed by filling the buffer.
  //When the buffer is full of the same value, it changes to that value.  
  //Data collection periodic rate can be from 1 - 255ms.
  //#define KICKSW_DATA_COLLECTION_RATE 100
  //KICKSW data buffer size
  //#define KICKSW_BUFFER_SIZE 3
  // KICKSW active level
  //#define KICKSW_ACTIVE_HIGH

  //Clutch Switch(CLTCHSW)
  //Define the signal for the system to enable functionality.
  //#define CLTCHSW
  //No filter algorithm to specifiy.  Filtering performed by filling the buffer.
  //When the buffer is full of the same value, it changes to that value.  
  //Data collection periodic rate can be from 1 - 255ms.
  //#define CLTCHSW_DATA_COLLECTION_RATE 100
  //CLTCHSW data buffer size
  //#define CLTCHSW_BUFFER_SIZE 3
  // CLTCHSW active level
  //#define CLTCHSW_ACTIVE_HIGH
  //#define CLTCHSW_ACTIVE_LOW

  //Neutral Gear(NGEARSW)
  //Define the signal for the system to enable functionality.
  //#define NGEARSW
  //No filter algorithm to specifiy.  Filtering performed by filling the buffer.
  //When the buffer is full of the same value, it changes to that value.  
  //Data collection periodic rate can be from 1 - 255ms.
  //#define NGEARSW_DATA_COLLECTION_RATE 100
  //NGEARSW data buffer size
  //#define NGEARSW_BUFFER_SIZE 3
  // NGEARSW active level
  //#define NGEARSW_ACTIVE_HIGH
  //#define NGEARSW_ACTIVE_LOW

  //Engine Stop Switch(ENGSTOP_SW)
  //Define the signal for the system to enable functionality.
  #define ENGSTOPSW
  //No filter algorithm to specifiy.  Filtering performed by filling the buffer.
  //When the buffer is full of the same value, it changes to that value.  
  //Data collection periodic rate can be from 1 - 255ms.
  #define ENGSTOPSW_DATA_COLLECTION_RATE 100
  //ENGSTOPSW data buffer size
  #define ENGSTOPSW_BUFFER_SIZE 3
  // ENGSTOPSW active level
  #define ENGSTOPSW_ACTIVE_HIGH


/*Control Outputs****************************************************************/

  //Idle Speed Motor Control(ISM)
  //Define the output functionality.
  //#define ISM
  //Specify the number of steps the motor can move.
  //The possible range from 0-65535.  
  //For Yamaha C3 500 is maximum number of steps
  //#define ISM_STEPS  500
  //For Suitcase demo it is 100 steps
  //#define ISM_STEPS  100  
  //The motion of the idle speed motor is defined as follows, no selection here.  
    #define CW 0
    #define CCW 1
  //Map direction of rotation to the opening or closing of the air passage.
  //Choose Set 1
  //#define ISM_OPEN CW
  //#define ISM_CLOSE CCW
  //or choose Set 2
  //#define ISM_OPEN CCW
  //#define ISM_CLOSE CW

  //Define which module ouput pins are connected to the idle speed
  //stepper motor.  Use H1INA, H1INB, H2INA, and H2INB only.  
  
  #if HARDWARE == EMULATOR
  #define SIN_P H1INA
  #define SIN_N H1INB
  #define COS_P H2INA
  #define COS_N H2INB
  #endif
  
  //#if HARDWARE == REFERENCE
  //#define SIN_P 
  //#define SIN_N 
  //#define COS_P 
  //#define COS_N 
  //#endif
  
  //Define how long each position is held for between steps.
  //This is based on a fundamental 100us size.  
  //Programmable from 100us to 20ms.  Example- value of 10 is 1ms.
  #define ISM_STEP_TIME 10
  
  //Power Supply Control(PSC)
  //Define the output functionality.  
  #define PSC
  //Two functions go with the PSC.  Enable and disable. 
 
  //Malfunction Indicator Lamp(MIL)
  //Define the output functionality.  
  #define MIL
  //Two functions go with the MIL.  Enable and disable.  

  //Oxygen Sensor Heater(O2H)
  //Define the output functionality.  
  #define O2H
  //Two functions go with the O2H.  Enable and disable.  

  //Relay 1(R1)
  //Define the output functionality.  
  #define R1
  //Two functions go with the R1.  Enable and disable.  

  //Relay 2(R2)
  //Define the output functionality.  
  #define R2
  //Two functions go with the R2.  Enable and disable.  

  //Relay 3(R3)
  //Define the output functionality.  
  //#define R3
  //Two functions go with the R3.  Enable and disable.  


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DEFINE_H_ */
