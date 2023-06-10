/*
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

CONFIDENTIAL AND PROPRIETARY
The Information contained in this document shall remain the sole exclusive
property of s.m.s smart microwave sensors GmbH and shall not be disclosed by the
recipient to third parties without prior consent of s.m.s smart microwave
sensors GmbH in writing.
*/

/*Includes*/
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <signal.h>

#include <fcntl.h>
#include <termios.h>

/*Defines*/
#define VERBOSE_OUTPUT 0
#define SMS_SRC_IP "192.168.11.11"
#define SMS_DEST_IP "192.168.11.17"
#define SMS_DEFAULT_PORT 55555

#define READ_BUFFER_SIZE 2048
#define CRC16_LENGTH 2

#define INSTRUCTIONS 46
#define TARGET_LIST 66
#define OBJECT_LIST 67
#define COM_DYNAMICS 81
#define OCC_GRID_OUTPUT 84

#define NOF_INSTRUCTIONS 10 //Max number of Instructions
#define NOF_TARGETS 110     //Max number of Targets
#define NOF_OBJECTS 50      //Max number of Objects
#define NOF_OCC_CELLS 110   //Max number of Occupied Cells

/*SMS transport protocol flags definition*/
#define SMS_PROTOCOL_FLAG_MSG_COUNTER 1
#define SMS_PROTOCOL_FLAG_SOURCE_CLIENT_ID 8
#define SMS_PROTOCOL_FLAG_DATA_IDENTIFIER 32
#define SMS_PROTOCOL_FLAG_SEGMENTATION 64

#define clear() printf("\033[H\033[J") // Clear the Console

/*Prototypes*/
bool initUDPConnection(void);
bool openUDPSocket(void);
bool bindUDPServerAddressToSocket(void);

void monitorSMSEthernet(void);
void parse_inputSMSTransportProtocol(uint8_t *u8_input_buffer, uint32_t u32_len);
void parse_inputSMSTransportPayload(uint8_t *u8_input_buffer, int i32_len);
void parse_inputSMSInstructions(uint8_t *u8_input_buffer, int i32_len);
void parse_SMSInstruction(uint8_t *u8_input_buffer, int index);
void parse_inputSMSTargets(uint8_t *u8_input_buffer, int i32_len);
void parse_SMSTarget(uint8_t *u8_input_buffer, int index);
void parse_inputSMSObjects(uint8_t *u8_input_buffer, int i32_len, int16_t i16_portMajVer);
void parse_SMSObject_V2(uint8_t *u8_input_buffer, int index);
void parse_SMSObject_V3(uint8_t *u8_input_buffer, int index);
void parse_inputSMSComDynamicsData(uint8_t *u8_input_buffer, int index);
void parse_inputSMSOccGridOutput(uint8_t *u8_input_buffer, int i32_len);
void parse_SMSOccCell(uint8_t *u8_input_buffer, int index);

void printStartInformation(void);
void printSMSTransportHeader(void);
void printSMSPortHeader(void);
void printSMSInstructionHeader(void);
void printSMSInstruction(int index);
void printSMSTargetHeader(void);
void printSMSTarget(int index);
void printSMSObjectHeader(void);
void printSMSObject_V2(int index);
void printSMSObject_V3(int index);
void printSMSComDynamicsData(void);
void printSMSOccGridOutputHeader(void);
void printSMSOccCell(int index);

/*Structures*/
typedef struct
{
  int i32_socket; // Socket descriptor server
  int i32_client; // Socket descriptor UMRR
  struct sockaddr_in s_server;
  struct sockaddr_in s_client;
} SMS_ETHERNET_CONNECTION_T;

typedef struct
{
  uint8_t u8_startPattern;
  uint8_t u8_protocolVersion;
  uint8_t u8_headerLength;
  uint16_t u16_payloadLength;
  uint8_t u8_AppProtocolType;
  uint32_t u32_flags;
  uint16_t u16_msgCounter;
  uint32_t u32_srcClientID;
  uint16_t u16_dataIdentifier;
  uint16_t u16_segmentation;
  uint16_t u16_headerCRC16;

  uint8_t au8_Header[READ_BUFFER_SIZE];
  uint8_t au8_Payload[READ_BUFFER_SIZE];
  uint16_t u16_indexHeader;
  uint16_t u16_indexPayload;
} SMS_TRANSPORT_DATA_FRAME_T;

typedef struct
{
  uint32_t u32_portIdentifier;
  int16_t i16_portMajVer;
  int16_t i16_portMinVer;
  uint64_t u64_portTimeStamp;
  uint32_t u32_portSize;
  uint8_t u8_portEndianess;
  uint8_t u8_portIndex;
  uint8_t u8_portHeaderMajVer;
  uint8_t u8_portHeaderMinVer;
} SMS_PORT_HEADER_T;

typedef struct
{
  uint8_t u8_nofInstructions;
} SMS_INSTRUCTION_HEADER_T;

typedef struct
{
  uint8_t u8_request;
  uint8_t u8_response;
  uint16_t u16_section;
  uint16_t u16_id;
  uint8_t u8_dataType;
  uint8_t u8_dimCount;
  uint16_t u16_dimElement_0;
  uint16_t u16_dimElement_1;
  uint32_t u32_signature;
  uint64_t u64_value;
} SMS_INSTRUCTION_T;

typedef struct
{
  float f_cycleTime;
  uint16_t u16_nOfTargets;
  uint16_t u16_reserved;
} SMS_TARGET_HEADER_T;

typedef struct
{
  float f_range;
  float f_speed;
  float f_azAngle;
  float f_elAngle;
  float f_rcs;
  float f_power;
  float f_noise;
} SMS_TARGET_T;

typedef struct
{
  float f_cycleTime;
  uint16_t u16_nOfObjects;
  uint16_t u16_reserved;
  uint64_t u64_timestampOfMeasurement;
} SMS_OBJECT_HEADER_T;

typedef struct
{
  float f_xPos;
  float f_yPos;
  float f_zPos;
  float f_speedAbs;
  float f_heading;
  float f_length;
  uint32_t u32_reserved1;
  float f_quality;
  float f_accel;
  int16_t i16_trkChannel;
  uint16_t u16_idleCycles;
  uint8_t u8_reserved1;
  uint8_t u8_reserved2;
  uint8_t u8_reserved3;	
  uint8_t u8_statusFlags;
} SMS_OBJECT_T;

typedef struct
{
  uint32_t u32_updateStatus;
  uint8_t u8_dynSrc;
  float f_egoSpeed;
  float f_yawRate;
  float f_egoSpeedQuality;
  float f_yawRateQuality;
} SMS_DYNAMICS_DATA_T;

typedef struct
{
  float f_cycleTime;
  uint16_t u16_nOfOccCells;
  uint16_t u16_reserved;
} SMS_OCC_GRID_OUTPUT_HEADER_T;

typedef struct
{
  float f_xPos;
  float f_yPos;
  float f_zPos;
  float f_reserved;
} SMS_OCC_CELL_T;

enum state_t
{
  START_PATTERN,
  PROTOCOL_VERSION,
  HEADER_LENGTH,
  PAYLOAD_LENGTH_H,
  PAYLOAD_LENGTH_L,
  APPPROTOCOL_TYPE,
  FLAG_1,
  FLAG_2,
  FLAG_3,
  FLAG_4,
  MSG_COUNTER_1,
  MSG_COUNTER_2,
  SRC_CLIENT_ID_1,
  SRC_CLIENT_ID_2,
  SRC_CLIENT_ID_3,
  SRC_CLIENT_ID_4,
  DATA_IDENTIFIER_1,
  DATA_IDENTIFIER_2,
  SEGMENTATION_1,
  SEGMENTATION_2,
  HEADER_CRC16_H,
  HEADER_CRC16_L,
  PAYLOAD_DATA,
  PORT_IDENTIFIER,
  PORT_MAJOR_VER,
  PORT_MINOR_VER,
  PORT_TIME_STAMP,
  PORT_SIZE,
  PORT_ENDIANESS,
  PORT_INDEX,
  PORT_HEADER_MAJOR_VER,
  PORT_HEADER_MINOR_VER
};

enum instr_req_state_t
{
  INSTR_REQ_UNKNOWN,
  INSTR_REQ_SET_PARAM,
  INSTR_REQ_GET_PARAM,
  INSTR_REQ_GET_STATUS,
  INSTR_REQ_EXEC_CMD
};

enum instr_res_state_t
{
  INSTR_RES_UNKNOWN,
  INSTR_RES_SUCCESS,
  INSTR_RES_ERR,
  INSTR_RES_INV_REQ_NR,
  INSTR_RES_INV_SEC,
  INSTR_RES_INV_ID,
  INSTR_RES_PARAM_READ_ONLY,
  INSTR_RES_VAL_MIN_BOUNDS,
  INSTR_RES_VAL_MAX_BOUNDS,
  INSTR_RES_INV_VAL,
  INSTR_RES_INV_TYPE,
  INSTR_RES_INV_DIM,
  INSTR_RES_INV_ELEM,
  INSTR_RES_INV_SIG,
  INSTR_RES_INV_CMD,
  INSTR_RES_INV_CMD_ACC_LVL,
  INSTR_RES_CMD_NOT_EXEC
};

enum instr_data_type_t
{
  INSTR_DATA_TYPE_UNKNOWN,
  INSTR_DATA_TYPE_I8,
  INSTR_DATA_TYPE_U8,
  INSTR_DATA_TYPE_I16,
  INSTR_DATA_TYPE_U16,
  INSTR_DATA_TYPE_I32,
  INSTR_DATA_TYPE_U32,
  INSTR_DATA_TYPE_F32
};

/*Global Variables*/
volatile sig_atomic_t stop = 0;
uint8_t au8_readBuffer[READ_BUFFER_SIZE];
SMS_ETHERNET_CONNECTION_T s_socket; // Socket information

SMS_TRANSPORT_DATA_FRAME_T s_dataFrame;
SMS_PORT_HEADER_T s_portHeader;
SMS_INSTRUCTION_HEADER_T s_instructionHeader;
SMS_INSTRUCTION_T as_instruction[NOF_INSTRUCTIONS];
SMS_TARGET_HEADER_T s_targetHeader;
SMS_TARGET_T as_target[NOF_TARGETS];
SMS_OBJECT_HEADER_T s_objectHeader;
SMS_OBJECT_T as_object[NOF_OBJECTS];
SMS_DYNAMICS_DATA_T s_comDynamicsData;
SMS_OCC_GRID_OUTPUT_HEADER_T s_occGridOutHeader;
SMS_OCC_CELL_T as_occCell[NOF_OCC_CELLS];
