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

#include "sms_types.h"
#include "checksum.h"
#include "decode_ethernet.h"
// #include <opencv2/opencv.hpp>
#include "stdio.h"

FILE *fpt;

/*Static variables*/
static bool crc_tabccitt_init = false;
static uint16_t crc_tabccitt[256];

/*Static functions definition*/
static void init_crcccitt_tab(void);
static uint16_t crc16_ccitt_generic(const unsigned char *input_str, size_t num_bytes, uint16_t start_value);

/*Static functions declaration*/
static void init_crcccitt_tab(void)
{
   uint16_t crc;
   uint16_t c;

   for (uint16_t i = 0; i < 256; ++i)
   {
      crc = 0;
      c = i << 8;

      for (uint16_t j = 0; j < 8; ++j)
      {
         if ((crc ^ c) & 0x8000)
            crc = (crc << 1) ^ CRC_POLY_CCITT;
         else
            crc = crc << 1;

         c = c << 1;
      }

      crc_tabccitt[i] = crc;
   }

   crc_tabccitt_init = true;

} /* init_crcccitt_tab */

static uint16_t crc16_ccitt_generic(const unsigned char *input_str, size_t num_bytes, uint16_t start_value)
{
   const unsigned char *ptr = input_str;
   uint16_t crc = start_value;

   if (!crc_tabccitt_init)
      init_crcccitt_tab();

   if (ptr != NULL)
   {
      uint16_t short_c;
      uint16_t tmp;

      for (size_t a = 0; a < num_bytes; a++)
      {
         short_c = 0x00ff & (unsigned short)*ptr;
         tmp = (crc >> 8) ^ short_c;
         crc = (crc << 8) ^ crc_tabccitt[tmp];
         ptr++;
      }
   }

   return crc;

} /* crc16_ccitt_generic */

void initRadarData(){
   if (initUDPConnection())
   {
      printStartInformation();
   }

   fpt = fopen("rader_data.csv", "w+");

}



int getRadarData()
{
   // clear();

   // if connection is successfull, monitor the data
   // if (initUDPConnection())
   // {
   //    printStartInformation();
   //    monitorSMSEthernet();
   // }
   monitorSMSEthernet();

   // printSMSObject_V3(25);

   int random_number = rand() % 100+1;
   return random_number;
   // return EXIT_SUCCESS;
}

bool initUDPConnection()
{
   return (openUDPSocket() && bindUDPServerAddressToSocket());
}

bool openUDPSocket()
{
   s_socket.i32_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
   if (s_socket.i32_socket < 0)
   {
      perror("... could not initialize the Socket");
      return false;
   }

   printf("... Socket initialization successful\n");
   return true;
}

bool bindUDPServerAddressToSocket(void)
{
   //bind the address to the socket
   printf("... binding Socket to Address\n");
   memset(&s_socket.s_server, 0, sizeof(s_socket.s_server));

   s_socket.s_server.sin_family = AF_INET;                     // IPv4
   s_socket.s_server.sin_addr.s_addr = inet_addr(SMS_DEST_IP); // 192.168.11.17
   s_socket.s_server.sin_port = htons(SMS_DEFAULT_PORT);       // 55555

   if (bind(s_socket.i32_socket, (struct sockaddr *)&s_socket.s_server, sizeof(s_socket.s_server)) < 0)
   {
      perror("... binding Socket to Address unsuccessful");
      return false;
   }
   printf("... binding Socket to Address successful\n");

   // Copy server socket to the client socket
   s_socket.i32_client = s_socket.i32_socket;
   return true;
}

void monitorSMSEthernet(void)
{
   // Initialize the buffer
   memset(&au8_readBuffer, 0, sizeof(au8_readBuffer));
   int i32_recv_bytes;

   // while (!stop)
   // {
      //Recieve the incoming data packets
      i32_recv_bytes = recv(s_socket.i32_client, &au8_readBuffer, READ_BUFFER_SIZE, 0);
      if (i32_recv_bytes < 0)
      {
         printf("... no SMS data received \n");
         // continue;
      }
      parse_inputSMSTransportProtocol(/*&*/ au8_readBuffer, i32_recv_bytes);
   // }
}

void parse_inputSMSTransportProtocol(uint8_t *pu8_input_buffer, uint32_t u32_len)
{
   static enum state_t s_state_Header = START_PATTERN;
   uint8_t u8_input = 0;

   for (uint16_t u16_i = 0; u16_i < u32_len; ++u16_i)
   {
      u8_input = pu8_input_buffer[u16_i];

      switch (s_state_Header)
      {
      case START_PATTERN:
         // Looking for the Startpattern 0x7E
         if (u8_input == 0x7E)
         {
            // reset or initialze the data frame structure at the start
            memset(&s_dataFrame, 0, sizeof(s_dataFrame));

            s_dataFrame.u8_startPattern = u8_input;
            s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

            s_state_Header = PROTOCOL_VERSION;
         }
         break;
      case PROTOCOL_VERSION:
         // Protocol Version is 1
         if (u8_input == 0x1)
         {
            s_dataFrame.u8_protocolVersion = u8_input;
            s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
            s_state_Header = HEADER_LENGTH;
         }
         else
         {
            s_state_Header = START_PATTERN;
         }
         break;
      case HEADER_LENGTH:
         // store the Headerlength
         s_dataFrame.u8_headerLength = u8_input;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = PAYLOAD_LENGTH_H;
         break;
      case PAYLOAD_LENGTH_H:
         // store payload length high byte
         s_dataFrame.u16_payloadLength |= u8_input << 8;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = PAYLOAD_LENGTH_L;
         break;
      case PAYLOAD_LENGTH_L:
         // store payload length low byte
         s_dataFrame.u16_payloadLength |= u8_input;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = APPPROTOCOL_TYPE;
         break;
      case APPPROTOCOL_TYPE:
         // App Protocol type 8 = PORT Application Protocol
         if (u8_input == 8)
         {
            s_dataFrame.u8_AppProtocolType = u8_input;
            s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
            s_state_Header = FLAG_1;
         }
         else
         {
            s_state_Header = START_PATTERN;
         }
         break;
      case FLAG_1:
         // store Flag
         s_dataFrame.u32_flags |= u8_input << 24;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = FLAG_2;
         break;
      case FLAG_2:
         // store Flag
         s_dataFrame.u32_flags |= u8_input << 16;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = FLAG_3;
         break;
      case FLAG_3:
         // store Flag
         s_dataFrame.u32_flags |= u8_input << 8;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = FLAG_4;
         break;
      case FLAG_4:
         // store Flag
         s_dataFrame.u32_flags |= u8_input;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

         if (((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_MSG_COUNTER) == 1) &&
             (s_dataFrame.u8_headerLength > 12))
         {
            s_state_Header = MSG_COUNTER_1; 
         }
         else
         {
            s_state_Header = HEADER_CRC16_H;
         }
         break;
      case MSG_COUNTER_1:
         s_dataFrame.u16_msgCounter |= u8_input << 8;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = MSG_COUNTER_2; 
         break;
      case MSG_COUNTER_2:
         s_dataFrame.u16_msgCounter |= u8_input;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

         if (/*((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_SOURCE_CLIENT_ID) == 1) &&*/
             (s_dataFrame.u8_headerLength > 14))
         {
            s_state_Header = SRC_CLIENT_ID_1; 
         }
         else
         {
            s_state_Header = HEADER_CRC16_H; 
         }
         break;
      case SRC_CLIENT_ID_1:
         s_dataFrame.u32_srcClientID |= u8_input << 24;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = SRC_CLIENT_ID_2;
         break;
      case SRC_CLIENT_ID_2:
         s_dataFrame.u32_srcClientID |= u8_input << 16;

         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         if (s_dataFrame.u8_headerLength > 16)
         {
            s_state_Header = SRC_CLIENT_ID_3; 
         }
         else
         {
            s_state_Header = HEADER_CRC16_H; 
         }
         break;
      case SRC_CLIENT_ID_3:
         s_dataFrame.u32_srcClientID |= u8_input << 8;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = SRC_CLIENT_ID_4;
         break;
      case SRC_CLIENT_ID_4:
         s_dataFrame.u32_srcClientID |= u8_input;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

         if (((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_DATA_IDENTIFIER) == 1) &&
             (s_dataFrame.u8_headerLength > 18))
         {
            s_state_Header = DATA_IDENTIFIER_1; 
         }
         else
         {
            s_state_Header = HEADER_CRC16_H; 
         }
         break;
      case DATA_IDENTIFIER_1:
         // stores data idetifier
         s_dataFrame.u16_dataIdentifier |= u8_input << 8;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = DATA_IDENTIFIER_2;
         break;
      case DATA_IDENTIFIER_2:
         // stores data idetifier
         s_dataFrame.u16_dataIdentifier |= u8_input;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

         if (((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_SEGMENTATION) == 1) &&
             (s_dataFrame.u8_headerLength > 20))
         {
            s_state_Header = SEGMENTATION_1; 
         }
         else
         {
            s_state_Header = HEADER_CRC16_H; 
         }
         break;
      case SEGMENTATION_1:
         // stores segmentation
         s_dataFrame.u16_segmentation |= u8_input << 8;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = SEGMENTATION_2;
         break;
      case SEGMENTATION_2:
         // stores segmentation
         s_dataFrame.u16_segmentation |= u8_input;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = HEADER_CRC16_H;
         break;
      case HEADER_CRC16_H:
         // store Header CRC16 High Byte
         s_dataFrame.u16_headerCRC16 |= u8_input << 8;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         s_state_Header = HEADER_CRC16_L;
         break;
      case HEADER_CRC16_L:
         // store Header CRC16 Low Byte
         s_dataFrame.u16_headerCRC16 |= u8_input;
         s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
         // verify the Header crc
         uint16_t u16_crc16 = crc16_ccitt_generic(&s_dataFrame.au8_Header[0], s_dataFrame.u8_headerLength - CRC16_LENGTH, CRC_START_CCITT_FFFF);

         if (u16_crc16 == s_dataFrame.u16_headerCRC16)
         {
            s_state_Header = PAYLOAD_DATA;
         }
         else
         {
            printf("\nHeader CRC16 Error\n");
            s_state_Header = START_PATTERN;
         }
         break;
      case PAYLOAD_DATA:
         // store Payload
         if (s_dataFrame.u16_indexPayload < s_dataFrame.u16_payloadLength) // read all data into the s_dataFrame.au8_Payload Buffer
         {
            s_dataFrame.au8_Payload[s_dataFrame.u16_indexPayload] = u8_input;
            s_dataFrame.u16_indexPayload++;
            if (s_dataFrame.u16_indexPayload >= s_dataFrame.u16_payloadLength)
            {
				#if VERBOSE_OUTPUT
					printSMSTransportHeader();
				#endif
               parse_inputSMSTransportPayload(/*&*/ s_dataFrame.au8_Payload, s_dataFrame.u16_payloadLength);
               s_state_Header = START_PATTERN;
            }
         }
         break;
      default:; //do nothing
         break;
      } // switch(s_state_Header)
      s_dataFrame.u16_indexHeader++;
   } // for
}

void parse_inputSMSTransportPayload(uint8_t *u8_input_buffer, int i32_len)
{
   int portHeaderLen = sizeof(s_portHeader); // 24 bytes

   if (portHeaderLen <= i32_len)
   {
      // Decode port header
      memset(&s_portHeader, 0, sizeof(s_portHeader));

      s_portHeader.u32_portIdentifier |= u8_input_buffer[0] << 24;
      s_portHeader.u32_portIdentifier |= u8_input_buffer[1] << 16;
      s_portHeader.u32_portIdentifier |= u8_input_buffer[2] << 8;
      s_portHeader.u32_portIdentifier |= u8_input_buffer[3];

      s_portHeader.i16_portMajVer = (int16_t)((u8_input_buffer[4] << 8) | u8_input_buffer[5]);
      s_portHeader.i16_portMinVer = (int16_t)((u8_input_buffer[6] << 8) | u8_input_buffer[7]);

      s_portHeader.u64_portTimeStamp |= (uint64_t)u8_input_buffer[8] << 56;
      s_portHeader.u64_portTimeStamp |= (uint64_t)u8_input_buffer[9] << 48;
      s_portHeader.u64_portTimeStamp |= (uint64_t)u8_input_buffer[10] << 40;
      s_portHeader.u64_portTimeStamp |= (uint64_t)u8_input_buffer[11] << 32;
      s_portHeader.u64_portTimeStamp |= u8_input_buffer[12] << 24;
      s_portHeader.u64_portTimeStamp |= u8_input_buffer[13] << 16;
      s_portHeader.u64_portTimeStamp |= u8_input_buffer[14] << 8;
      s_portHeader.u64_portTimeStamp |= u8_input_buffer[15];

      s_portHeader.u32_portSize |= u8_input_buffer[16] << 24;
      s_portHeader.u32_portSize |= u8_input_buffer[17] << 16;
      s_portHeader.u32_portSize |= u8_input_buffer[18] << 8;
      s_portHeader.u32_portSize |= u8_input_buffer[19];

      s_portHeader.u8_portEndianess = u8_input_buffer[20];

      s_portHeader.u8_portIndex = u8_input_buffer[21];

      s_portHeader.u8_portHeaderMajVer = u8_input_buffer[22];

      s_portHeader.u8_portHeaderMinVer = u8_input_buffer[23];

      // Print sms port header information
	  #if VERBOSE_OUTPUT
		printSMSPortHeader();
	  #endif

      // Decode port body
      uint8_t u8_portBody[READ_BUFFER_SIZE];
      memcpy(&u8_portBody, u8_input_buffer + portHeaderLen, i32_len - portHeaderLen);

      switch (s_portHeader.u32_portIdentifier)
      {
      case INSTRUCTIONS:
         // Decode instructions
         if (sizeof(s_instructionHeader) <= (i32_len - portHeaderLen))
         {
            parse_inputSMSInstructions(/*&*/ u8_portBody, i32_len - portHeaderLen);
         }
         else
         {
            printf(" Insufficient payload to decode instruction data \n");
         }
         break;
      case TARGET_LIST:
         // Decode targets
         if (sizeof(s_targetHeader) <= (i32_len - portHeaderLen))
         {
            parse_inputSMSTargets(/*&*/ u8_portBody, i32_len - portHeaderLen);
         }
         else
         {
            printf(" Insufficient payload to decode target data \n");
         }
         break;
      case OBJECT_LIST:
         // Decode objects
         if (sizeof(s_objectHeader) <= (i32_len - portHeaderLen))
         {
            parse_inputSMSObjects(/*&*/ u8_portBody, i32_len - portHeaderLen, s_portHeader.i16_portMajVer);
         }
         else
         {
            printf(" Insufficient payload to decode object data \n");
         }
         break;
      case COM_DYNAMICS:
         // Decode com dynamics data
         if (sizeof(s_comDynamicsData) <= (i32_len - portHeaderLen))
         {
            parse_inputSMSComDynamicsData(/*&*/ u8_portBody, i32_len - portHeaderLen);
         }
         else
         {
            printf(" Insufficient payload to decode com dynamics data \n");
         }
         break;
      case OCC_GRID_OUTPUT:
         // Decode occupancy grid output list
         if (sizeof(s_occGridOutHeader) <= (i32_len - portHeaderLen))
         {
            parse_inputSMSOccGridOutput(/*&*/ u8_portBody, i32_len - portHeaderLen);
         }
         else
         {
            printf(" Insufficient payload to decode occupancy grid output list \n");
         }
         break;
      default:
         printf("Unknown port identifier\n");
         break;
      }
   }
   else
   {
      printf(" Insufficient payload to decode port data \n");
   }
}

void parse_inputSMSInstructions(uint8_t *u8_input_buffer, int i32_len)
{
   int instrHeaderLen = sizeof(s_instructionHeader) + 7; // 1 + (7 reserved bytes)

   if (instrHeaderLen <= i32_len)
   {
      // Decode instruction header
      memset(&s_instructionHeader, 0, instrHeaderLen);

      s_instructionHeader.u8_nofInstructions = u8_input_buffer[0];

      // Print instruction header information
      printSMSInstructionHeader();

      // Decode instructions
      uint8_t u8_instrBody[READ_BUFFER_SIZE];
      memcpy(&u8_instrBody, u8_input_buffer + instrHeaderLen, i32_len - instrHeaderLen);

      int instrBodyLen = s_instructionHeader.u8_nofInstructions * sizeof(SMS_INSTRUCTION_T);

      if (instrBodyLen <= (i32_len - instrHeaderLen))
      {
         for (int i = 0; i < s_instructionHeader.u8_nofInstructions; ++i)
         {
            parse_SMSInstruction(/*&*/ u8_instrBody, i);
            // Print instruction
            printSMSInstruction(i);
         }
      }
      else
      {
         printf(" Insufficient payload to decode instructions \n");
      }
   }
   else
   {
      printf(" Insufficient payload to decode instruction data \n");
   }
}

void parse_SMSInstruction(uint8_t *u8_input_buffer, int index)
{
   // Decode instruction data
   memset(&as_instruction[index], 0, sizeof(SMS_INSTRUCTION_T));

   as_instruction[index].u8_request = u8_input_buffer[0];
   as_instruction[index].u8_response = u8_input_buffer[1];

   as_instruction[index].u16_section |= u8_input_buffer[2] << 8;
   as_instruction[index].u16_section |= u8_input_buffer[3];

   as_instruction[index].u16_id |= u8_input_buffer[4] << 8;
   as_instruction[index].u16_id |= u8_input_buffer[5];

   as_instruction[index].u8_dataType = u8_input_buffer[6];
   as_instruction[index].u8_dimCount = u8_input_buffer[7];

   as_instruction[index].u16_dimElement_0 |= u8_input_buffer[8] << 8;
   as_instruction[index].u16_dimElement_0 |= u8_input_buffer[9];

   as_instruction[index].u16_dimElement_1 |= u8_input_buffer[10] << 8;
   as_instruction[index].u16_dimElement_1 |= u8_input_buffer[11];

   as_instruction[index].u32_signature |= u8_input_buffer[12] << 24;
   as_instruction[index].u32_signature |= u8_input_buffer[13] << 16;
   as_instruction[index].u32_signature |= u8_input_buffer[14] << 8;
   as_instruction[index].u32_signature |= u8_input_buffer[15];

   as_instruction[index].u64_value |= (uint64_t)u8_input_buffer[16] << 56;
   as_instruction[index].u64_value |= (uint64_t)u8_input_buffer[17] << 48;
   as_instruction[index].u64_value |= (uint64_t)u8_input_buffer[18] << 40;
   as_instruction[index].u64_value |= (uint64_t)u8_input_buffer[19] << 32;
   as_instruction[index].u64_value |= u8_input_buffer[20] << 24;
   as_instruction[index].u64_value |= u8_input_buffer[21] << 16;
   as_instruction[index].u64_value |= u8_input_buffer[22] << 8;
   as_instruction[index].u64_value |= u8_input_buffer[23];
}

void parse_inputSMSTargets(uint8_t *u8_input_buffer, int i32_len)
{
   // currently not implemented, because it is not needed for the MSE use case
}


void parse_inputSMSObjects(uint8_t *u8_input_buffer, int i32_len, int16_t i16_portMajVer)
{
   int objHeaderLen = sizeof(s_objectHeader);
   union s_float_convert
	{
		float fvalue;
		u_int8_t uvalue[4];
	} s_float_convert;   

   if (objHeaderLen <= i32_len)
   {
		// Decode object header
		memset(&s_objectHeader, 0, objHeaderLen);

		s_float_convert.uvalue[0] = u8_input_buffer[3];
		s_float_convert.uvalue[1] = u8_input_buffer[2];
		s_float_convert.uvalue[2] = u8_input_buffer[1];
		s_float_convert.uvalue[3] = u8_input_buffer[0];
		s_objectHeader.f_cycleTime = s_float_convert.fvalue;      
	  		
		s_objectHeader.u16_nOfObjects = (u8_input_buffer[4] << 8) | u8_input_buffer[5];

		// Print object header information
		printSMSObjectHeader();

		// Decode objects
		uint8_t u8_objBody[READ_BUFFER_SIZE];
		memcpy(&u8_objBody, u8_input_buffer + objHeaderLen, i32_len - objHeaderLen);

		int objBodyLen = 0;

		if (i16_portMajVer > 2)
		{
		 // zPos is included in the Object from the port header major version v3
		 objBodyLen = s_objectHeader.u16_nOfObjects * sizeof(SMS_OBJECT_T);
		 if (objBodyLen <= (i32_len - objHeaderLen))
		 {
			for (int i = 0; i < s_objectHeader.u16_nOfObjects; ++i)
			{
			   parse_SMSObject_V3(&(u8_objBody[i * sizeof(SMS_OBJECT_T)]), i);
			   // Print object information
			   // printSMSObject_V3(i);

            int objectId = i + 1; 
            // getObjectId(i+1);
            
            // x_coord = (as_object[i].f_xPos);
            // int y_coord = (as_object[i].f_yPos);
            // int z_coord = (as_object[i].f_zPos);
            // int heading = (as_object[i].f_heading);
			}
		 }
		 else
		 {
			printf(" Insufficient payload to decode objects \n");
		 }
		}
		else
		{
		 // zPos is not included in the Object prior to the port header major version v3
		 objBodyLen = s_objectHeader.u16_nOfObjects * (sizeof(SMS_OBJECT_T) - 4);
		 if (objBodyLen <= (i32_len - objHeaderLen))
		 {
			for (int i = 0; i < s_objectHeader.u16_nOfObjects; ++i)
			{
			   parse_SMSObject_V2(/*&*/ u8_objBody, i);
			   // Print object information
			   printSMSObject_V2(i);
			}
		 }
		 else
		 {
			printf(" Insufficient payload to decode objects \n");
		 }
		}
		}
		else
		{
		printf(" Insufficient payload to decode objects data \n");
		}
}

void parse_SMSObject_V2(uint8_t *u8_input_buffer, int index)
{
	// Decode object data
	memset(&as_object[index], 0, sizeof(SMS_OBJECT_T));

	printf("\nError: V2 currently not supported");
}

void parse_SMSObject_V3(uint8_t *u8_input_buffer, int index)
{
   union s_float_convert
	{
		float fvalue;
		u_int8_t uvalue[4];
	} s_float_convert;
   
   // Decode object data
   memset(&as_object[index], 0, sizeof(SMS_OBJECT_T));

   
   s_float_convert.uvalue[0] = u8_input_buffer[3];
   s_float_convert.uvalue[1] = u8_input_buffer[2];
   s_float_convert.uvalue[2] = u8_input_buffer[1];
   s_float_convert.uvalue[3] = u8_input_buffer[0];
   as_object[index].f_xPos = s_float_convert.fvalue;

   s_float_convert.uvalue[0] = u8_input_buffer[7]; 
   s_float_convert.uvalue[1] = u8_input_buffer[6]; 
   s_float_convert.uvalue[2] = u8_input_buffer[5]; 
   s_float_convert.uvalue[3] = u8_input_buffer[4]; 
   as_object[index].f_yPos = s_float_convert.fvalue;

   s_float_convert.uvalue[0] = u8_input_buffer[11]; 
   s_float_convert.uvalue[1] = u8_input_buffer[10]; 
   s_float_convert.uvalue[2] = u8_input_buffer[9]; 
   s_float_convert.uvalue[3] = u8_input_buffer[8];    
   as_object[index].f_zPos = s_float_convert.fvalue; //((u8_input_buffer[8] << 24) | (u8_input_buffer[9] << 16) | (u8_input_buffer[10] << 8) | u8_input_buffer[11]);
   
   s_float_convert.uvalue[0] = u8_input_buffer[15]; 
   s_float_convert.uvalue[1] = u8_input_buffer[14]; 
   s_float_convert.uvalue[2] = u8_input_buffer[13]; 
   s_float_convert.uvalue[3] = u8_input_buffer[12];   
   as_object[index].f_speedAbs = s_float_convert.fvalue; //((u8_input_buffer[12] << 24) | (u8_input_buffer[13] << 16) | (u8_input_buffer[14] << 8) | u8_input_buffer[15]);
   
   s_float_convert.uvalue[0] = u8_input_buffer[19]; 
   s_float_convert.uvalue[1] = u8_input_buffer[18]; 
   s_float_convert.uvalue[2] = u8_input_buffer[17]; 
   s_float_convert.uvalue[3] = u8_input_buffer[16];   
   as_object[index].f_heading =  s_float_convert.fvalue; //((u8_input_buffer[16] << 24) | (u8_input_buffer[17] << 16) | (u8_input_buffer[18] << 8) | u8_input_buffer[19]);
   
   s_float_convert.uvalue[0] = u8_input_buffer[23]; 
   s_float_convert.uvalue[1] = u8_input_buffer[22]; 
   s_float_convert.uvalue[2] = u8_input_buffer[21]; 
   s_float_convert.uvalue[3] = u8_input_buffer[20];   
   as_object[index].f_length =  s_float_convert.fvalue; //((u8_input_buffer[20] << 24) | (u8_input_buffer[21] << 16) | (u8_input_buffer[22] << 8) | u8_input_buffer[23]);
   
   s_float_convert.uvalue[0] = u8_input_buffer[31]; 
   s_float_convert.uvalue[1] = u8_input_buffer[30]; 
   s_float_convert.uvalue[2] = u8_input_buffer[29]; 
   s_float_convert.uvalue[3] = u8_input_buffer[28];   
   as_object[index].f_quality =  s_float_convert.fvalue; //((u8_input_buffer[28] << 24) | (u8_input_buffer[29] << 16) | (u8_input_buffer[30] << 8) | u8_input_buffer[31]);
   
   s_float_convert.uvalue[0] = u8_input_buffer[35]; 
   s_float_convert.uvalue[1] = u8_input_buffer[34]; 
   s_float_convert.uvalue[2] = u8_input_buffer[33]; 
   s_float_convert.uvalue[3] = u8_input_buffer[32];    
   as_object[index].f_accel =  s_float_convert.fvalue; //((u8_input_buffer[32] << 24) | (u8_input_buffer[33] << 16) | (u8_input_buffer[34] << 8) | u8_input_buffer[35]);
   
   as_object[index].i16_trkChannel = (int16_t)((u8_input_buffer[36] << 8) | u8_input_buffer[37]);
   as_object[index].u16_idleCycles = (u8_input_buffer[38] << 8) | u8_input_buffer[39];
   as_object[index].u8_statusFlags = u8_input_buffer[43] >> 1; // >> 1 is required but not in sync with current documentation
}

void parse_inputSMSComDynamicsData(uint8_t *u8_input_buffer, int i32_len)
{
   int comDynamicsDataLen = sizeof(s_comDynamicsData);

   if (comDynamicsDataLen <= i32_len)
   {
      
		union s_float_convert
		{
		   float fvalue;
		   u_int8_t uvalue[4];
		} s_float_convert;	  

		// Decode com dynamics data
		memset(&s_comDynamicsData, 0, comDynamicsDataLen);

		s_comDynamicsData.u32_updateStatus = (uint32_t)((u8_input_buffer[0] << 24) | (u8_input_buffer[1] << 16) | (u8_input_buffer[2] << 8) | u8_input_buffer[3]);
		s_comDynamicsData.u8_dynSrc = u8_input_buffer[4];

		s_float_convert.uvalue[0] = u8_input_buffer[8]; 
		s_float_convert.uvalue[1] = u8_input_buffer[7]; 
		s_float_convert.uvalue[2] = u8_input_buffer[6]; 
		s_float_convert.uvalue[3] = u8_input_buffer[5]; 	  
		s_comDynamicsData.f_egoSpeed = s_float_convert.fvalue; 

		s_float_convert.uvalue[0] = u8_input_buffer[15]; 
		s_float_convert.uvalue[1] = u8_input_buffer[14]; 
		s_float_convert.uvalue[2] = u8_input_buffer[13]; 
		s_float_convert.uvalue[3] = u8_input_buffer[12]; 		
		s_comDynamicsData.f_yawRate = s_float_convert.fvalue; 
		
		s_float_convert.uvalue[0] = u8_input_buffer[19]; 
		s_float_convert.uvalue[1] = u8_input_buffer[18]; 
		s_float_convert.uvalue[2] = u8_input_buffer[17]; 
		s_float_convert.uvalue[3] = u8_input_buffer[16]; 		
		s_comDynamicsData.f_egoSpeedQuality = s_float_convert.fvalue; 
		
		s_float_convert.uvalue[0] = u8_input_buffer[23]; 
		s_float_convert.uvalue[1] = u8_input_buffer[22]; 
		s_float_convert.uvalue[2] = u8_input_buffer[21]; 
		s_float_convert.uvalue[3] = u8_input_buffer[20]; 		
		s_comDynamicsData.f_yawRateQuality = s_float_convert.fvalue; 

		// Print com dynamics information
		printSMSComDynamicsData();
   }
   else
   {
      printf(" Insufficient payload to decode com dynamics data \n");
   }
}

void parse_inputSMSOccGridOutput(uint8_t *u8_input_buffer, int i32_len)
{
   // not implemented & not needed
}

void parse_SMSOccCell(uint8_t *u8_input_buffer, int index)
{
   // not implemented & not needed
}

void printStartInformation()
{
   printf("************************************\n");
   printf("*            smartmicro            *\n");
   printf("* Example Code - Customer Ethernet *\n");
   printf("************************************\n");
   printf("\n");
}

void printSMSTransportHeader()
{
   printf("************************************\n");
   printf("*       SMS Transport Header       *\n");
   printf("************************************\n");
   printf("Start Pattern: \t%u\n", s_dataFrame.u8_startPattern);
   printf("Protocol Version: \t%u\n", s_dataFrame.u8_protocolVersion);
   printf("Header Length [Bytes]: \t%u\n", s_dataFrame.u8_headerLength);
   printf("Payload Length [Bytes]: \t%u\n", s_dataFrame.u16_payloadLength);

   if (s_dataFrame.u8_AppProtocolType == 8)
   {
      printf("Application Protocol Type: \t%u - PORT \n", s_dataFrame.u8_AppProtocolType);
   }
   else
   {
      printf("Application Protocol Type: \t%u - Unknown \n", s_dataFrame.u8_AppProtocolType);
   }

   printf("Flags: \t%x\n", s_dataFrame.u32_flags);

   if ((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_MSG_COUNTER) == 1)
   {
      printf("Message Counter: \t%u\n", s_dataFrame.u16_msgCounter);
   }
   if ((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_SOURCE_CLIENT_ID) == 1)
   {
      printf("Source Client ID: \t%x\n", s_dataFrame.u32_srcClientID);
   }
   if ((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_DATA_IDENTIFIER) == 1)
   {
      printf("Data Identifier: \t%u\n", s_dataFrame.u16_dataIdentifier);
   }
   if ((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_SEGMENTATION) == 1)
   {
      printf("Segmentation: \t%u\n", s_dataFrame.u16_segmentation);
   }

   printf("HeaderCRC: \t%x\n", s_dataFrame.u16_headerCRC16);
   printf("\n");
}

void printSMSPortHeader()
{
   printf("************************************\n");
   printf("*         SMS Port Header          *\n");
   printf("************************************\n");
   printf("Port Identifier: \t%u\n", s_portHeader.u32_portIdentifier);
   printf("Major Version: \t%d\n", s_portHeader.i16_portMajVer);
   printf("Minor Version: \t%d\n", s_portHeader.i16_portMinVer);
   printf("Timestamp [us]: \t%llu\n", s_portHeader.u64_portTimeStamp);
   printf("Size [bytes]: \t%u\n", s_portHeader.u32_portSize);
   printf("Endianess: \t%u\n", s_portHeader.u8_portEndianess);
   printf("Port Index: \t%u\n", s_portHeader.u8_portIndex);
   printf("Header Major Version: \t%u\n", s_portHeader.u8_portHeaderMajVer);
   printf("Header Minor Version: \t%u\n", s_portHeader.u8_portHeaderMinVer);
   printf("\n");
}

void printSMSInstructionHeader()
{
   printf("************************************\n");
   printf("*      SMS Instruction Header      *\n");
   printf("************************************\n");
   printf("NrOfInstructions: \t%u\n", s_instructionHeader.u8_nofInstructions);
   printf("\n");
}

void printSMSInstruction(int index)
{
   printf("************************************\n");
   printf("*        Instruction - %d          *\n", index + 1);
   printf("************************************\n");

   switch (as_instruction[index].u8_request)
   {
   case INSTR_REQ_SET_PARAM:
      printf("Request: Set/Modify a Parameter\n");
      break;
   case INSTR_REQ_GET_PARAM:
      printf("Request: Get/Read a Parameter\n");
      break;
   case INSTR_REQ_GET_STATUS:
      printf("Request: Get/Read Status\n");
      break;
   case INSTR_REQ_EXEC_CMD:
      printf("Request: Execute a command\n");
      break;
   default:
      printf("Request: Unknown\n");
      break;
   }

   switch (as_instruction[index].u8_response)
   {
   case INSTR_RES_SUCCESS:
      printf("Response: Success\n");
      break;
   case INSTR_RES_ERR:
      printf("Response: General Error\n");
      break;
   case INSTR_RES_INV_REQ_NR:
      printf("Response: Invalid request number\n");
      break;
   case INSTR_RES_INV_SEC:
      printf("Response: Invalid section\n");
      break;
   case INSTR_RES_INV_ID:
      printf("Response: Invalid ID\n");
      break;
   case INSTR_RES_PARAM_READ_ONLY:
      printf("Response: Parameter is read only (write protected)\n");
      break;
   case INSTR_RES_VAL_MIN_BOUNDS:
      printf("Response: Value out of minimal bounds\n");
      break;
   case INSTR_RES_VAL_MAX_BOUNDS:
      printf("Response: Value out of maxmimum bounds\n");
      break;
   case INSTR_RES_INV_VAL:
      printf("Response: Value is not a number (related to float access)\n");
      break;
   case INSTR_RES_INV_TYPE:
      printf("Response: Instruction Type is not valid\n");
      break;
   case INSTR_RES_INV_DIM:
      printf("Response: Instruction Dimension is not valid\n");
      break;
   case INSTR_RES_INV_ELEM:
      printf("Response: Instruction Element is not valid\n");
      break;
   case INSTR_RES_INV_SIG:
      printf("Response: Signature of the Instruction is not valid\n");
      break;
   case INSTR_RES_INV_CMD:
      printf("Response: Command is not valid\n");
      break;
   case INSTR_RES_INV_CMD_ACC_LVL:
      printf("Response: Command access level is not valid\n");
      break;
   case INSTR_RES_CMD_NOT_EXEC:
      printf("Response: Command is not executable\n");
      break;
   default:
      printf("Response: Unknown\n");
      break;
   }

   printf("Section: \t%u\n", as_instruction[index].u16_section);
   printf("ID: \t%u\n", as_instruction[index].u16_id);

   switch (as_instruction[index].u8_dataType)
   {
   case INSTR_DATA_TYPE_I8:
      printf("DataType: i8\n");
      break;
   case INSTR_DATA_TYPE_U8:
      printf("DataType: u8\n");
      break;
   case INSTR_DATA_TYPE_I16:
      printf("DataType: i16\n");
      break;
   case INSTR_DATA_TYPE_U16:
      printf("DataType: u16\n");
      break;
   case INSTR_DATA_TYPE_I32:
      printf("DataType: i32\n");
      break;
   case INSTR_DATA_TYPE_U32:
      printf("DataType: u32\n");
      break;
   case INSTR_DATA_TYPE_F32:
      printf("DataType: f32\n");
      break;
   default:
      printf("DataType: Unknown\n");
      break;
   }

   printf("DimCount: \t%u\n", as_instruction[index].u8_dimCount);
   printf("DimElement0: \t%u\n", as_instruction[index].u16_dimElement_0);
   printf("DimElement1: \t%u\n", as_instruction[index].u16_dimElement_1);
   printf("Signature: \t%x\n", as_instruction[index].u32_signature);
   printf("Value: \t%llx\n", as_instruction[index].u64_value);
   printf("\n");
}

void printSMSTargetHeader()
{
   printf("************************************\n");
   printf("*        SMS Target Header         *\n");
   printf("************************************\n");
   printf("CycleTime [sec]: \t%f\n", s_targetHeader.f_cycleTime);
   printf("NrOfTargets: \t%u\n", s_targetHeader.u16_nOfTargets);
   if (s_targetHeader.u16_reserved > 0)
   {
      printf("Reserved Info: \t%u\n", s_targetHeader.u16_reserved);
   }
   printf("\n");
}

void printSMSTarget(int index)
{
   printf("************************************\n");
   printf("*          Target - %d             *\n", index + 1);
   printf("************************************\n");
   printf("Range [m]: \t%f\n", as_target[index].f_range);
   printf("Radial Speed [m/s]: \t%f\n", as_target[index].f_speed);
   printf("AzAngle [rad]: \t%f\n", as_target[index].f_azAngle);
   printf("ElAngle [rad]: \t%f\n", as_target[index].f_elAngle);
   printf("RCS [m^2]: \t%f\n", as_target[index].f_rcs);
   printf("Power [dB]: \t%f\n", as_target[index].f_power);
   printf("Noise [dB]: \t%f\n", as_target[index].f_noise);
   printf("\n");
}

void printSMSObjectHeader()
{
   printf("************************************\n");
   printf("*        SMS Object Header         *\n");
   printf("************************************\n");
   printf("CycleTime [sec]: \t%f\n", s_objectHeader.f_cycleTime);
   printf("NrOfObjects: \t%u\n", s_objectHeader.u16_nOfObjects);
   printf("\n");
}

void printSMSObject_V2(int index)
{
   printf("**************SMSv2**********************\n");
   printf("*          Object - %d             *\n", index + 1);
   printf("************************************\n");
   printf("X Pos [m]: \t%f\n", as_object[index].f_xPos);
   printf("Y Pos [m]: \t%f\n", as_object[index].f_yPos);
   printf("Absolute Speed [m/s]: \t%f\n", as_object[index].f_speedAbs);
   printf("Heading [rad]: \t%f\n", as_object[index].f_heading);
   printf("Length [m]: \t%f\n", as_object[index].f_length);
   printf("Quality [m]: \t%f\n", as_object[index].f_quality);
   printf("Acceleration [m/(s^2)]: \t%f\n", as_object[index].f_accel);
   printf("Tracker Channel: \t%d\n", as_object[index].i16_trkChannel);
   printf("Idle Cycle: \t%u\n", as_object[index].u16_idleCycles);

   if (as_object[index].u8_statusFlags == 1)
   {
      printf("Status: \t%u - Guard Rail\n", as_object[index].u8_statusFlags);
   }
   else
   {
      printf("Status: \t%u - Reserved\n", as_object[index].u8_statusFlags);
   }

   printf("\n");
}

void printSMSObject_V3(int index)
{
   printf("*************SMSv3***********************\n");
   printf("*          Object - %d             *\n", index + 1);
   printf("************************************\n");
   printf("X Pos [m]: \t%f\n", as_object[index].f_xPos);
   printf("Y Pos [m]: \t%f\n", as_object[index].f_yPos);
   printf("Z Pos [m]: \t%f\n", as_object[index].f_zPos);
   printf("Absolute Object Speed [m/s]: \t%f\n", as_object[index].f_speedAbs);
   printf("Heading [rad]: \t%f\n", as_object[index].f_heading);
   printf("Length [m]: \t%f\n", as_object[index].f_length);
   printf("Quality [m]: \t%f\n", as_object[index].f_quality);
   printf("Acceleration [m/(s^2)]: \t%f\n", as_object[index].f_accel);
   printf("Tracker Channel: \t%d\n", as_object[index].i16_trkChannel);
   printf("Idle Cycle: \t%u\n", as_object[index].u16_idleCycles);

   if (as_object[index].u8_statusFlags == 1)
   {
      printf("Status: \t%u - Guard Rail\n", as_object[index].u8_statusFlags);
   }
   else
   {
      printf("Status: \t%u - Reserved\n", as_object[index].u8_statusFlags);
   }

   // radarDatas_.push_back(index + 1);
   // radarDatas_.push_back(as_object[index].f_xPos);
   // radarDatas_.push_back(as_object[index].f_yPos);
   // radarDatas_.push_back(as_object[index].f_zPos);


   // rData.objectId = index + 1;
   // rData.x_coord = as_object[index].f_xPos;
   // rData.y_coord = as_object[index].f_yPos;
   // rData.z_coord = as_object[index].f_zPos;

   printf("\n");
}

char *printSMSObject_V3_1(int index)
{
   printf("*************SMSv3***********************\n");
   printf("*          Object - %d             *\n", index + 1);
   printf("************************************\n");
   printf("X Pos [m]: \t%f\n", as_object[index].f_xPos);
   printf("Y Pos [m]: \t%f\n", as_object[index].f_yPos);
   printf("Z Pos [m]: \t%f\n", as_object[index].f_zPos);
   printf("Absolute Object Speed [m/s]: \t%f\n", as_object[index].f_speedAbs);
   printf("Heading [rad]: \t%f\n", as_object[index].f_heading);
   printf("Length [m]: \t%f\n", as_object[index].f_length);
   printf("Quality [m]: \t%f\n", as_object[index].f_quality);
   printf("Acceleration [m/(s^2)]: \t%f\n", as_object[index].f_accel);
   printf("Tracker Channel: \t%d\n", as_object[index].i16_trkChannel);
   printf("Idle Cycle: \t%u\n", as_object[index].u16_idleCycles);

   // if (as_object[index].u8_statusFlags == 1)
   // {
   //    printf("Status: \t%u - Guard Rail\n", as_object[index].u8_statusFlags);
   // }
   // else
   // {
   //    printf("Status: \t%u - Reserved\n", as_object[index].u8_statusFlags);
   // }
   // arr[6] = { (index + 1), (as_object[index].f_xPos), (as_object[index].f_yPos), (as_object[index].f_zPos), (as_object[index].f_speedAbs), as_object[index].f_heading};
   // radarDatas_.push_back(index + 1);
   // radarDatas_.push_back(as_object[index].f_xPos);
   // radarDatas_.push_back(as_object[index].f_yPos);
   // radarDatas_.push_back(as_object[index].f_zPos);


   // rData.objectId = index + 1;
   // rData.x_coord = as_object[index].f_xPos;
   // rData.y_coord = as_object[index].f_yPos;
   // rData.z_coord = as_object[index].f_zPos;
   float x_coord = as_object[index].f_xPos;
   float y_coord = as_object[index].f_yPos;
   float z_coord = as_object[index].f_zPos;
   int f_speedAbs_out = as_object[index].f_speedAbs;
   float out_heading = as_object[index].f_heading;
   float f_length_out = as_object[index].f_length;
   
   int lengthx = snprintf( NULL, 0, "%f", (x_coord) );
   int lengthy = snprintf( NULL, 0, "%f", (y_coord) );
   int lengthz = snprintf( NULL, 0, "%f", (z_coord) );
   int out_f_speedAbs = snprintf( NULL, 0, "%f", (f_speedAbs_out) );
   int length_out_heading = snprintf( NULL, 0, "%f", (out_heading) );
   int length_f_length_out = snprintf( NULL, 0, "%f", (f_length_out) );
   // int out_f_accel = snprintf( NULL, 0, "%f", (f_accel_out) );
   
   printf("%f, %f, %f, %f\n", x_coord, y_coord, z_coord, f_length_out); /* prints "blablablaH" */


   int length = lengthx + lengthy + lengthz + out_f_speedAbs + length_out_heading + length_f_length_out;
   char* str = malloc( length + 2 );
   snprintf( str, length + 2, "%f, %f, %f, %f, %f, %f", (x_coord), y_coord, z_coord, f_speedAbs_out, out_heading, f_length_out);

   // char c = ':';

   // size_t len = strlen(str);

   // /* one for extra char, one for trailing zero */
   // char *str2 = malloc(len + 1 + 1);

   // strcpy(str2, str);
   // str2[len] = c;
   // str2[len + 1] = '\0';
   fprintf(fpt,"%d, %f,  %f, %f, %f, %f, %f\n", (index + 1) ,(x_coord), y_coord, z_coord, f_speedAbs_out, out_heading, f_length_out);

   // printf("%f, %f, %f, %f\n", x_coord, y_coord, z_coord, f_speedAbs_out); /* prints "blablablaH" */

   // int length2 = snprintf( NULL, 0, "%d", (as_object[index].f_xPos) );
   // char* str3 = malloc( length2 + 1 );
   // snprintf( str3, length2 + 1, "%d", (as_object[index].f_xPos) );

   // size_t len3 = strlen(str3);

   // char *str4 = malloc(len3 + 1 + 1);

   // strcpy(str4, str3);
   // str4[len3] = str3;
   // str4[len3 + 1] = '\0';

   return str;
   free(str);

   // arr[0] = '1';

   // printf("\n");
   // return array;
   
}

void printSMSComDynamicsData()
{
   printf("************************************\n");
   printf("*        SMS Com Dynamics Data     *\n");
   printf("************************************\n");
   switch (s_comDynamicsData.u32_updateStatus)
   {
   case 1:
      printf("Update Status: Speed Update\n");
      break;
   case 2:
      printf("Update Status: Speed Quality Update\n");
      break;
   case 4:
      printf("Update Status: Yaw Rate Update\n");
      break;
   case 8:
      printf("Update Status: Yaw Rate Quality Update\n");
      break;
   default:
      printf("Update Status: Unknown\n");
      break;
   }
   printf("Dynamics Source: \t%u\n", s_comDynamicsData.u8_dynSrc);
   printf("Ego Speed [m/s]: \t%f\n", s_comDynamicsData.f_egoSpeed);
   printf("Yaw Rate [rad/s]: \t%f\n", s_comDynamicsData.f_yawRate);
   printf("Ego Speed Quality: \t%f\n", s_comDynamicsData.f_egoSpeedQuality);
   printf("Yaw Rate Quality: \t%f\n", s_comDynamicsData.f_yawRateQuality);
   printf("\n");
}

void printSMSOccGridOutputHeader()
{
   printf("******************************************\n");
   printf("*    SMS Occupancy Grid Output Header    *\n");
   printf("******************************************\n");
   printf("CycleTime [sec]: \t%f\n", s_occGridOutHeader.f_cycleTime);
   printf("NrOfOccCells: \t%u\n", s_occGridOutHeader.u16_nOfOccCells);
   printf("\n");
}

void printSMSOccCell(int index)
{
   printf("************************************\n");
   printf("*          Occ Cell - %d           *\n", index + 1);
   printf("************************************\n");
   printf("X Pos [m]: \t%f\n", as_occCell[index].f_xPos);
   printf("Y Pos [m]: \t%f\n", as_occCell[index].f_yPos);
   printf("Z Pos [m]: \t%f\n", as_occCell[index].f_zPos);
   if (as_occCell[index].f_reserved > 0.0)
   {
      printf("Reserved Info: \t%u\n", as_occCell[index].f_reserved);
   }
   printf("\n");
}
