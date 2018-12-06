// In case we need a second serial port for debugging
#define SOFT_SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed
#ifdef SOFT_SERIAL_DEBUGGING
// Library to use serial debugging with a second board
#include <SoftwareSerial.h>
SoftwareSerial mySerial(0, 1); // RX, TX
SoftwareSerial pxSerial(0, 1);  // RX, TX
#endif

#include "mavlink.h"
//LOOK INTO IF WE WANT TO INCLUDE THIS FOR OR PROJECT
//----------------------------------------------------
//#include "common/mavlink_msg_request_data_stream.h"


// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

//SEEMS TO BE LED CONTROLLING VARABLES BUT HAVEN'T SINGLED OUT WHAT WE SPECIFICALLY NEED
//--------------------------------------------------------------------------------------
// Lights flashing adjustment
unsigned long previousMillis = 0;     // will store last time LED was updated
unsigned long next_interval = 0;      // next interval
const long tiempo_on = 20;
const long tiempo_off = 80;
const long tiempo_descanso = 880;
int test_led_tipo = 4;
//end------------------------------------------------------------------------------------
/*
//sonic pathing--------------------------------------------------------------------------
int sonicControl(){
  //legend for int out
  //0=front/nothing
  //1=left
  //-1=right
  //4=back
  int m1,m2,m3;
  bool front=true;
  bool left=true;
  bool right=true;
  
  m1=sensor1.read();
  if(m1>MAXD){m1=0;}
  if((m1<SPOOKD)&&(m1!=0)){front = false;}  //If less then spook then don't go forward
  
  m2=sensor2.read();
  if(m2>MAXD){m2=0;}
  if((m2<SPOOKD)&&(m2!=0)){left = false;}   //Same
  
  m3=sensor3.read();
  if(m3>MAXD){m3=0;}
  if((m3<SPOOKD)&&(m3!=0)){right = false;}  //Same

  //drifting/object appearance conditions
  if((m2<SSPOOKD)&&(m2!=0)&&(right == true)){ return -1; }  //for drifting left/object appearing on left, go right
  if((m3<SSPOOKD)&&(m3!=0)&&(left  == true)){ return 1; }   //for drifting right/object appearing on right, go left

  //if passes drifting/object apperance take these conditionals
  if (front == true){return 0;} //go front if available
  if ((front == false)&&(left == true)){return 1;}  //go left of front not availible
  if ((front == false)&&(left == false)&&(right == true)){return -1;} //go right
  if ((front == false)&&(left==false)&&(right==false)){return 4;} //go backwards
  else return 4;  //all else fails, go backwards until out of mess.
}
//end----------------------------------------------------------------------------
*/

void setup() {
  // MAVLink interface start
  Serial.begin(57600);  //Mavlink is specific on baud rate so for the most part just leave this alone.

#ifdef SOFT_SERIAL_DEBUGGING
  // [DEB] Soft serial port start
  Serial.begin(57600);
  Serial.println("MAVLink starting.");
  mySerial.begin(57600);
#endif
}

void loop() {
  //Again seems to be more LED stuff need to narrow down if we need this
  //--------------------------------------------------------------------
  // Lights management
  // Light pulses: 2 quick flashes per second. 100 ms each cycle
  unsigned long currentMillis = millis();
  int i = 0;

  // Normal mode, lights on.
  if (currentMillis - previousMillis >= next_interval) {
    // Keep time last mode changed
    previousMillis = currentMillis;
  }
  //end------------------------------------------------------------------

  //MAVLink-----------------------------------------------------------------------------
  /* The default UART header for your MCU */
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Guardamos la última vez que se cambió el modo
    previousMillisMAVLink = currentMillisMAVLink;

#ifdef SOFT_SERIAL_DEBUGGING
    pxSerial.write(buf, len);
    mySerial.println("Ardu HB");
#else
    Serial.write(buf, len);
#endif

    Mav_Request_Data();
    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs) {
      // Request streams from Pixhawk
#ifdef SOFT_SERIAL_DEBUGGING
      mySerial.println("Streams requested!");
#endif
      Mav_Request_Data();
      num_hbs_pasados = 0;
    }

  }

  // Check reception buffer
  comm_receive();
}
//end-------------------------------------------------------------------------
//THIS IS A PORTION WE CARE ABOUT.^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//--------------------------------------
//Can add more commands to this section!
//--------------------------------------
void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // STREAMS that can be requested
  /*
     Definitions are in common.h: enum MAV_DATA_STREAM

     MAV_DATA_STREAM_ALL=0, // Enable all data streams
     MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
     MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
     MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
     MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
     MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
     MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
     MAV_DATA_STREAM_ENUM_END=13,
     MAV_CMD

     Data in PixHawk available in:
      - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
      - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
  */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02, 0x05};


  for (int i = 0; i < maxStreams; i++) {
    /*
       mavlink_msg_request_data_stream_pack(system_id, component_id,
          &msg,
          target_system, target_component,
          MAV_DATA_STREAM_POSITION, 10000000, 1);

       mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id,
          mavlink_message_t* msg,
          uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
          uint16_t req_message_rate, uint8_t start_stop)

    */
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
#ifdef SOFT_SERIAL_DEBUGGING
    pxSerial.write(buf, len);
#else
    Serial.write(buf, len);
#endif
  }

  // Request: PARAM_REQUEST_LIST. Only for full log recording
  /*
     Primitive: mavlink_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                   uint8_t target_system, uint8_t target_component)
  */
  
    // Configure
    uint8_t system_id=2;
    uint8_t component_id=200;
    // mavlink_message_t* msg;
    uint8_t target_system=1;
    uint8_t target_component=0;

    // Pack
    mavlink_msg_param_request_list_pack(system_id, component_id, &msg,
      target_system, target_component);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send
    #ifdef SOFT_SERIAL_DEBUGGING
      pxSerial.write(buf,len);
    #else
      Serial.write(buf, len);
    #endif
  
}
//end------------------------------------------------------------------------------


void comm_receive() {

  mavlink_message_t msg;
  mavlink_status_t status;

  // Echo for manual debugging
  Serial.println("---Start---");

#ifdef SOFT_SERIAL_DEBUGGING
  while (pxSerial.available() > 0) {
    uint8_t c = pxSerial.read();
#else
  while (Serial.available() > 0) {
    uint8_t c = Serial.read();
#endif

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
#ifdef SOFT_SERIAL_DEBUGGING
            //mySerial.println("PX HB");
#endif
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
                  mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
            */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.print("PX SYS STATUS: ");
            mySerial.print("[Bat (V): ");
            mySerial.print(sys_status.voltage_battery);
            mySerial.print("], [Bat (A): ");
            mySerial.print(sys_status.current_battery);
            mySerial.print("], [Comms loss (%): ");
            mySerial.print(sys_status.drop_rate_comm);
            mySerial.println("]");
#endif
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
                  mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
            */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX PARAM_VALUE");
            mySerial.println(param_value.param_value);
            mySerial.println(param_value.param_count);
            mySerial.println(param_value.param_index);
            mySerial.println(param_value.param_id);
            mySerial.println(param_value.param_type);
            mySerial.println("------ Fin -------");
#endif
          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
                  static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
            */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX RAW IMU");
            mySerial.println(raw_imu.xacc);
#endif
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
           /* Message decoding: PRIMITIVE
                  mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
            */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX ATTITUDE");
            mySerial.println(attitude.roll);
            //if (attitude.roll > 1) leds_modo = 0;
            //else if (attitude.roll < -1) leds_modo = 2;
            //else leds_modo = 1;
#endif
          }
          break;


        default:
#ifdef SOFT_SERIAL_DEBUGGING
          mySerial.print("--- Otros: ");
          mySerial.print("[ID: ");
          mySerial.print(msg.msgid);
          mySerial.print("], [seq: ");
          mySerial.print(msg.seq);
          mySerial.println("]");
#endif
          break;
      }
    }
  }
}
