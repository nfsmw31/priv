#include "DSP2834x_Device.h"

#include "Ser_Comm.def"
#include "Err_Hndl.def"
#include "FPGA.def"
#include "Design.def"
#include "Record.def"
#include "CommFdbk.def"
#include "i2c.def"
#include "ExFbVar.def"

#include "Ser_Comm.var"
#include "Display.var"
#include "Drive.var"
#include "FltCntrl.var"
#include "Init.var"
#include "Extrn_Asm.var"
#include "Record.var"
#include "ExFbVar.var"


#include "Prototypes.pro"


//**********************************************************
// Function Name: InitSerCommunication
// Description:
//   Initialize Serial communication variables and peripherials
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void InitSerCommunication(void)
{
   int i;
// Initialize the input and output buffer pointers to point to the
// beginning of the buffers.
   p_u8_In_Buf_In_Ptr   = u8_Input_Buffer;
   p_u8_In_Buf_Out_Ptr  = u8_Input_Buffer;
   p_u8_Out_Buf_In_Ptr  = u8_Output_Buffer;
   p_u8_Out_Buf_Out_Ptr = u8_Output_Buffer;

// Initialize pointers to the end of the input and output buffers
   p_u8_In_Buf_End  = u8_Input_Buffer + COMMS_BUFFER_SIZE - 1;
   p_u8_Out_Buf_End = u8_Output_Buffer + COMMS_BUFFER_SIZE - 1;

// Set the input and output buffer free space to the buffer size
   u8_Input_Buffer_Free_Space = COMMS_BUFFER_SIZE;
   u8_Output_Buffer_Free_Space = COMMS_BUFFER_SIZE;

// Determine the end point of the Message Buffer
   p_u8_Message_Buffer_End_Ptr = u8_Message_Buffer + COMMS_BUFFER_SIZE - 1;

// Initialize the Message Buffer
   for(i=0;i<COMMS_BUFFER_SIZE;i++)
   {
      u8_Message_Buffer[i] = '\0';
   }
   p_u8_Message_Buffer_Ptr = u8_Message_Buffer;

   InitSci();
}


void InitSerCommunication2(void)
{
   int drive = 0;
   long long s64_Modbus_Brate;
   int Modbus_Brate_Idx = BGVAR(u16_P3_01_BRT) & 0x0F; // 4 low Bits are Modbus Brate

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //Udi Oct 15,2013: Init UART for Modbus
   if (u16_Product == SHNDR_HW)
   {
       switch(Modbus_Brate_Idx)
       {
           case 0 : s64_Modbus_Brate = 4800; break;
           case 1 : s64_Modbus_Brate = 9600; break;
           case 2 : s64_Modbus_Brate = 19200; break;
           case 3 : s64_Modbus_Brate = 38400; break;
           case 4 : s64_Modbus_Brate = 57600; break;
           case 5 : s64_Modbus_Brate = 115200; break;
           default: s64_Modbus_Brate = 115200; break;


       }
        SalSerialBaudRateCommand(s64_Modbus_Brate, 0);
        InitSci();
        //Set Extension time before Disabling the Transmit Enable bit on RS485: 32 = 1 milli sec
        // Set time to ensure few bits can be send after fifo is empty.
        // Last bit + 2 stop bits + 2 spare time: total 5 bits.
        // One bit is send in 32000/s64_Modbus_Brate ticks.
        u16_RS485_Tx_Enable_Ext_Time = (unsigned int) ((5L*32000)/(long) s64_Modbus_Brate) + 1;

   }
   if (u8_FPGA_ok)
   {
      //read rotary knob
     Rotary_Switch = ReadRotarySwitch();
     Prev_Rotary_Switch = Rotary_Switch;

      // Initialize Communication options
      BGVAR(u8_Comms_Options) = 0xFF;
      drive = 1;
      BGVAR(u8_Comms_Options) = 0xFF;
      drive = 0;

      if (BGVAR(u16_Message)  == 0)
      {
         BGVAR(u8_Comms_Options) &= ~PROMPT_MASK;
         BGVAR(u8_Comms_Options) &= ~ERR_MSG_MASK;
      }

      if (BGVAR(u16_Echo)  == 0)
      {
         BGVAR(u8_Comms_Options) &= ~ECHO_CONTROL_MASK;
      }

      drive = 1;
      if (BGVAR(u16_Message)  == 0)
      {
         BGVAR(u8_Comms_Options) &= ~PROMPT_MASK;
         BGVAR(u8_Comms_Options) &= ~ERR_MSG_MASK;
      }

      if (BGVAR(u16_Echo)  == 0)
      {
         BGVAR(u8_Comms_Options) &= ~ECHO_CONTROL_MASK;
      }
      drive = 0;

      //set drive addressed parameters
      u8_Drive_Id = ReadDriveId();

      u8_Axis_Id  = AXIS1_SELECTED; //TBD - allow values 0-1 when multi-axis support is added

      // init - not globally addressed
      u8_Comms_State &= ~GLOBAL_ADDRESSED_MASK;

      //init - if addr is 0: drive addressed (not globally)
      //       else        : drive not addresses (and not globally addressed)
      if(u8_Drive_Id == 0)
      {
       u8_Comms_State |= ADDRESSED_MASK;
      }
      else
      {
       u8_Comms_State &= ~ADDRESSED_MASK;
      }
   }
   else// NO FPGA - enable serial communication regrdless of drive address
   {
      BGVAR(u8_Comms_Options) = (PROMPT_MASK | ECHO_CONTROL_MASK | ERR_MSG_MASK);
      drive = 1;
      BGVAR(u8_Comms_Options) = (PROMPT_MASK | ECHO_CONTROL_MASK | ERR_MSG_MASK);
      drive = 0;

      // make the drive addressed but not globaly addressed
      u8_Comms_State = ADDRESSED_MASK;
      u8_Comms_State &= ~GLOBAL_ADDRESSED_MASK;
      u8_Drive_Id = 0;
      u8_Axis_Id  = 0;
   }
}

unsigned int ReadRotarySwitch(void)
{
   unsigned int u16_rotary_from_fpga = 0;

   if ((u16_Product == DDHD) && (!IS_DDHD_EC2_DRIVE))
   {
      return((((int)(GpioDataRegs.GPBDAT.all >> 26)) & 0x000f) ^ 0x000f);
   }
   else if (u16_Product == SHNDR_HW)
   {
      return(0);
   }
   else if ( u8_Is_Retro_Display )//if rotary switch exist
   {
      u16_rotary_from_fpga = *((int*)FPGA_SWITCH_REG_ADD);

      // Test if rotary was read correctly otherwise set address to 0 (so communication can be established) and raise a fault
      if (((u16_rotary_from_fpga & 0x000F) > 9) || (((u16_rotary_from_fpga & 0x00F0)>>4) > 9))
      {
         u16_Bad_Rotary_Switch = 1;
         u16_rotary_from_fpga = 0;
      }
      s8_Drive_Address = u16_rotary_from_fpga;
      return(u16_rotary_from_fpga);
   }
   
   else return s8_Drive_Address;
}


unsigned char ReadDriveId(void)
{
   unsigned int u16_rotary_switch;
   unsigned char u8_drive_id;

   if (u16_Product == DDHD)
   {
      u16_rotary_switch = ReadRotarySwitch();

      if (u16_Axis_Num == 0)
      {
         switch (u16_rotary_switch)
         {
            case 0:  u8_drive_id = 0x2; break;
            case 1:  u8_drive_id = 0x4; break;
            case 2:  u8_drive_id = 0x6; break;
            case 3:  u8_drive_id = 0x8; break;
            case 4:  u8_drive_id = 0x10; break; //it means ID = 10
            case 5:  u8_drive_id = 0x5; break;
            case 6:  u8_drive_id = 0x6; break;
            case 7:  u8_drive_id = 0x4; break;
            case 8:  u8_drive_id = 0x5; break;
            case 9:  u8_drive_id = 0x6; break;
            case 10: u8_drive_id = 0x3; break;
            case 11: u8_drive_id = 0x4; break;
            default: u8_drive_id = 0x2; break;
         }
      }
      else
      {
         switch (u16_rotary_switch)
         {
            case 0:  u8_drive_id = 0x1; break;
            case 1:  u8_drive_id = 0x3; break;
            case 2:  u8_drive_id = 0x5; break;
            case 3:  u8_drive_id = 0x7; break;
            case 4:  u8_drive_id = 0x9; break;
            case 5:  u8_drive_id = 0x1; break;
            case 6:  u8_drive_id = 0x2; break;
            case 7:  u8_drive_id = 0x1; break;
            case 8:  u8_drive_id = 0x2; break;
            case 9:  u8_drive_id = 0x3; break;
            case 10: u8_drive_id = 0x1; break;
            case 11: u8_drive_id = 0x2; break;
            default: u8_drive_id = 0x1; break;
         }
      }
      return(u8_drive_id);
   }
   else if (u16_Product == SHNDR_HW)
   {
      return((unsigned char)0);    // A P param should define drive id
   }
   else
   {
      u16_rotary_switch = ReadRotarySwitch();
      return(u16_rotary_switch);
   }
}


//**********************************************************
// Function Name: IncrementPtr
// Description:
//   Increment cyclic buffer pointer
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
/*void inline IncrementPtr(char** ptr,char* buff_start)
{
   if (*ptr == p_u8_In_Buf_End)
    {
      *ptr = buff_start;
      }
      else
      *ptr = *ptr + 1;
}
*/


//**********************************************************
// Function Name: InitMsgBuffer
// Description:
//   Increment Initialize the message buffer and his pointer
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void InitMsgBuffer(void)
{
//    Initialize the message buffer
   p_u8_Message_Buffer_Ptr = u8_Message_Buffer;
    *p_u8_Message_Buffer_Ptr = '\0';
}


//**********************************************************
// Function Name: WriteOutputBuffer
// Description:
//   Writes character to output buffer
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void WriteOutputBuffer(char data_byte)
{
   char  u8_MsbDigit=0, u8_LsbDigit=0;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u8_CR_Flag=0;

   if (u8_Comms_State & GLOBAL_ADDRESSED_MASK)return;
   if (!(u8_Comms_State & ADDRESSED_MASK)) return;



// Check for free space at output buffer
   if (!u8_Output_Buffer_Free_Space) return;
   //////////////////////////////
   if (data_byte==CR && u8_Prev_Data_Byte==0 )
      u8_CR_Flag=0;
   else
      u8_CR_Flag=1;

   //////////////////////////////////////
   //if the prev Data Byte is Null enable ,u8_CR_Flag
   //and checksum was required ,u8_ChkSm[0]
   //and the state isn't PROMPT,u8_Prompt_Indicator
   //and not in echo ,u8_Echo_Indicator
   if ((BGVAR(u8_ChkSm)) && (u8_Prompt_Indicator == 0) && (u8_Echo_Indicator == 0) && (u8_CR_Flag))
   {
      /////////////////////////////////////////////////////////////
      // As long te output Data Byte isn't CR or LF,add to checksum
      /////////////////////////////////////////////////////////////
      if (data_byte!=CR && data_byte!=LF )
         u8_Output_Chk_Sum_Str+=data_byte;
      //////////////////////////////////////////////////////////
     //In case the prev Data Byte was'nt the command propmot >
     //convert the checksum to ASCII
      /////////////////////////////////////////////////////////////
      if (data_byte==CR && u8_Prev_Data_Byte!='>')
      {
         u8_Output_Chk_Sum_Str&=0x00ff;
         u8_MsbDigit=( u8_Output_Chk_Sum_Str & 0x00f0)>>4;
         u8_LsbDigit=( u8_Output_Chk_Sum_Str & 0x000f);
       if ( u8_MsbDigit>=0 &&  u8_MsbDigit<=9)
           u8_MsbDigit+='0';
       if ( u8_MsbDigit>=0x0a &&  u8_MsbDigit<=0x0f)
           u8_MsbDigit+=-0x0a+'A';
       if ( u8_LsbDigit>=0 &&  u8_LsbDigit<=9)
           u8_LsbDigit+='0';
       if ( u8_LsbDigit>=0x0a &&  u8_LsbDigit<=0x0f)
           u8_LsbDigit+=-0x0a+'A';
       PrintChar('<');
       PrintChar( u8_MsbDigit);
       PrintChar( u8_LsbDigit);
       PrintChar('>');
       u8_Prev_Data_Byte=0;
         u8_Output_Chk_Sum_Str=0;
     }
   }
   //////////////////////////////////////
   // Storing the prev printed Data Byte
   //////////////////////////////////////
    u8_Prev_Data_Byte=data_byte;
   //////////////////////////////

//  Copy data to the output buffer
   *p_u8_Out_Buf_In_Ptr = data_byte;


//  Increment output buffer inp pointer
   IncrementPtr(p_u8_Out_Buf_In_Ptr,u8_Output_Buffer,p_u8_Out_Buf_End);

//  Decrement free space counter
    u8_Output_Buffer_Free_Space--;
}

//**********************************************************
// Function Name: WriteOutputBuffer16
// Description:
//   Writes word to output buffer
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void WriteOutputBuffer16(unsigned int data_word)
{
   extern unsigned int u16_Packet_Check_Sum;

   // Check for free space at output buffer
   if (u8_Output_Buffer_Free_Space<2) return;

//  Copy data to the output buffer
   *p_u8_Out_Buf_In_Ptr = data_word;
//  Increment output buffer inp pointer
   IncrementPtr(p_u8_Out_Buf_In_Ptr,u8_Output_Buffer,p_u8_Out_Buf_End);
//  Copy data to the output buffer
   *p_u8_Out_Buf_In_Ptr = data_word>>8;
//  Increment output buffer inp pointer
   IncrementPtr(p_u8_Out_Buf_In_Ptr,u8_Output_Buffer,p_u8_Out_Buf_End);

//  Decrement free space counter
    u8_Output_Buffer_Free_Space-=2;

   u16_Packet_Check_Sum += data_word;//update checksum
}

//**********************************************************
// Function Name: WriteCheckSum
// Description:
//   Writes checksum to output buffer
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void WriteCheckSum()
{
   extern unsigned int u16_Packet_Check_Sum;
// Check for free space at output buffer
   if (u8_Output_Buffer_Free_Space<2) return;

//  Copy data to the output buffer
   *p_u8_Out_Buf_In_Ptr =  0x10000 - u16_Packet_Check_Sum; //LSB checksum
//  Increment output buffer inp pointer
   IncrementPtr(p_u8_Out_Buf_In_Ptr,u8_Output_Buffer,p_u8_Out_Buf_End);
//  Copy data to the output buffer
   *p_u8_Out_Buf_In_Ptr = (0x10000 - u16_Packet_Check_Sum)>>8; //MSB checksum
//  Increment output buffer inp pointer
   IncrementPtr(p_u8_Out_Buf_In_Ptr,u8_Output_Buffer,p_u8_Out_Buf_End);

//  Decrement free space counter
    u8_Output_Buffer_Free_Space-=2;

}

//**********************************************************
// Function Name: ReadInputBuffer
// Description:
//   Reads character from input buffer
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
char ReadInputBuffer(void)
{
   char data_byte;         /* Character read from the buffer */

// Check if the input buffer is empty
   if (u8_Input_Buffer_Free_Space == COMMS_BUFFER_SIZE) return 0;

   data_byte = *p_u8_In_Buf_Out_Ptr;

// Increment input buffer output ptr
   IncrementPtr(p_u8_In_Buf_Out_Ptr,u8_Input_Buffer,p_u8_In_Buf_End);

// Increment free space counter
   u8_Input_Buffer_Free_Space++;

   return data_byte;
}

//**********************************************************
// Function Name: SciInputProcessor
// Description:
//   Read character from SCI-A and put it to input buffer
//   This function called from real time
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(SciInputProcessor, "ramfunc_2");
void SciInputProcessor(void)
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(BGVAR(s16_Serial_Type) != SERIAL_COMM_TYPE) //Udi
   return;

   // if FOE is active (parans are sent via FOE), ignore incoming bytes from SCI input.
   if (BGVAR(u16_Foe_ParamsFile_State) != FOE_PARAMS_FILE_DWLD_IDLE)
   {
      return;
   }
   
   // AZ debug =S=
   // Normally =S= should not listen to SCI-B, but here it is done mainly for debug, assuming
   // no received data in both SCI-C and SCI-B.
   // To remove the debug, put the code below in a comment.

   while ( (ScibRegs.SCIFFRX.bit.RXFFST > 0) && (u8_Input_Buffer_Free_Space > 0))
   {
      //  Read data
      *p_u8_In_Buf_In_Ptr = ScibRegs.SCIRXBUF.bit.RXDT;

      //  Increment input buffer input ptr
      IncrementPtr(p_u8_In_Buf_In_Ptr,u8_Input_Buffer,p_u8_In_Buf_End);

      //  Decrement free space counter
      u8_Input_Buffer_Free_Space--;
   }
}

//**********************************************************
// Function Name: SciOutputProcessor
// Description:
//   Read character from output buffer and put it to SCI-A
//   This function called from real time
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(SciOutputProcessor, "ramfunc_2");
void SciOutputProcessor(void)
{
   //int drive = 0;
   //// AXIS_OFF;

   if(BGVAR(s16_Serial_Type) != SERIAL_COMM_TYPE)
       return;

      while ((ScibRegs.SCIFFTX.bit.TXFFST < 0x0F) && (!EMPTY_OUTPUT_BUFFER))
      {
         //  Write data from output buffer to SCI-B
         ScibRegs.SCITXBUF = *p_u8_Out_Buf_Out_Ptr;

         //  Increment output buffer cyclic pointer
          IncrementPtr(p_u8_Out_Buf_Out_Ptr,u8_Output_Buffer,p_u8_Out_Buf_End);

         //  Increment free space counter
          u8_Output_Buffer_Free_Space++;
      }
   }

//**********************************************************
// Function Name: CommsPreProcessor
// Description:
//       This function implements the communications handler pre-processor.
//
//
// Author: Dimitry //Gil
// Algorithm:
// Return Value:
//   0: Pre processor still busy
//   1: A complete message has been received
//   2: A communication error was detected
//   3: A CheckSum error was detected
// Revisions:
//**********************************************************
char CommsPreProcessor(void)
{
   char data_byte;         /* Character read from input buffer */
   char result = 0;        /* Return value */
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   do
    {
      if (EMPTY_INPUT_BUFFER) return 0;

      // Read data from input buffer
      data_byte = ReadInputBuffer();

       // Check the value of the character read from the input buffer
       switch(data_byte)
       {
             case ESC:   /* Escape character */
                // Clear the message buffer
               u8_Message_Buffer[0] = '\0';
               p_u8_Message_Buffer_Ptr = u8_Message_Buffer;
               break;

             case CR:    /* Carriage Return character */
            // Set the return value to indicate that a complete command
            //has been received
               result = 0x01;
         ///////////////////////////////////////////////////////////
         // In case there's a need for checksum utility
         ///////////////////////////////////////////////////////////

         if (*(p_u8_Message_Buffer_Ptr-1)=='>' && *(p_u8_Message_Buffer_Ptr-4)!='<' )
         {
            result=0x03;
               WriteOutputBuffer(CR);
              WriteOutputBuffer(LF);
            return result;
         }
         if (*(p_u8_Message_Buffer_Ptr-1)=='>' )
         {
            u8_Read_Orig_Chk_Sum=0;
            u8_Res_Chk_Sum=0;
               if (*(p_u8_Message_Buffer_Ptr-2)>='0' && *(p_u8_Message_Buffer_Ptr-2)<='9')
               u8_Read_Orig_Chk_Sum=*(p_u8_Message_Buffer_Ptr-2)-'0';
              if (*(p_u8_Message_Buffer_Ptr-2)>='A' && *(p_u8_Message_Buffer_Ptr-2)<='F')
               u8_Read_Orig_Chk_Sum=*(p_u8_Message_Buffer_Ptr-2)-'A'+10;
            if (*(p_u8_Message_Buffer_Ptr-3)>='0' && *(p_u8_Message_Buffer_Ptr-3)<='9')
               u8_Read_Orig_Chk_Sum+=(*(p_u8_Message_Buffer_Ptr-3)-'0')<<4;
               if (*(p_u8_Message_Buffer_Ptr-3)>='A' && *(p_u8_Message_Buffer_Ptr-3)<='F')
                  u8_Read_Orig_Chk_Sum+=((*(p_u8_Message_Buffer_Ptr-3)-'A'+10)<<4);

               *(p_u8_Message_Buffer_Ptr-4)='\0';
              *(p_u8_Message_Buffer_Ptr-3)='\0';
              *(p_u8_Message_Buffer_Ptr-2)='\0';
              *(p_u8_Message_Buffer_Ptr-1)='\0';
              *(p_u8_Message_Buffer_Ptr)='\0';

              p_u8_Message_Buffer_Ptr = u8_Message_Buffer;
            u8_Res_Chk_Sum=CheckSum (p_u8_Message_Buffer_Ptr,0);
               if ( u8_Res_Chk_Sum== u8_Read_Orig_Chk_Sum)
            {
                 result=0x01;
              WriteOutputBuffer(CR);
                 WriteOutputBuffer(LF);
              }
              else
            {
                 result=0x03;
              WriteOutputBuffer(CR);
                 WriteOutputBuffer(LF);
            }
         }
         else
         {
               *p_u8_Message_Buffer_Ptr = '\0';
               p_u8_Message_Buffer_Ptr = u8_Message_Buffer;
            WriteOutputBuffer(CR);
               WriteOutputBuffer(LF);
         }

            break;


           case BS: /* Back space */

            // Decrement the message buffer pointer, if the buffer is not empty
               if (p_u8_Message_Buffer_Ptr != u8_Message_Buffer)
               {
                     p_u8_Message_Buffer_Ptr --;

                   // Echo the BS character
                   WriteOutputBuffer(data_byte);
                }
                break;

           default:
            // Check if the message buffer is full
               if ((p_u8_Message_Buffer_Ptr < p_u8_Message_Buffer_End_Ptr) && (data_byte != LF))
               {
               // Write the character to the message buffer
                  *p_u8_Message_Buffer_Ptr = data_byte;

               // Increment the pointer to the message buffer
                  p_u8_Message_Buffer_Ptr ++;
               }
               // If echo enabled copy character to output buffer
             u8_Echo_Indicator = 1;
            if (BGVAR(u16_Echo) == 1)
               WriteOutputBuffer(data_byte);
               break;
       }
    }
    while ( data_byte != CR );
   u8_Echo_Indicator = 0;


   return result;
}

//**********************************************************
// Function Name: ParseAddressCmd
// Description:
//       This function parses the address command. If the command is valid,
//      it will also set the address state of the drive.
//
//
// Author: Dimitry
// Algorithm:
// Return Value:
//   0: Syntax error
//   1: Address command successfully parsed
// Revisions:
//**********************************************************
char ParseAddressCmd(void)
{
   int s16_addr = 0xFF;

   // Check the third and forth character in the message
   // we allow "\x" and "\xy"
   if ((u8_Message_Buffer[2] != NULL)  && (u8_Message_Buffer[3] != NULL))  return 0;

   // Check the second character in the message
   switch( u8_Message_Buffer[1] )
   {
      case ASTERISK:
         // Global address command
         u8_Comms_State |= ADDRESSED_MASK;
         u8_Comms_State |= GLOBAL_ADDRESSED_MASK;
         break;

      case BACKSLASH:
        //un-address the addressed drive
       u8_Comms_State &= ~ADDRESSED_MASK;
         break;

      default:
       if(u8_Message_Buffer[2] == NULL) //address is one digit "\x"
       {
             if ((u8_Message_Buffer[1]>='0') && (u8_Message_Buffer[1]<='9'))
            {
               s16_addr = u8_Message_Buffer[1] - '0';
               InitSerCommunication();
          }
        }

       else if(u8_Message_Buffer[3] == NULL)//address is two digit "\xy"
       {
          if ((u8_Message_Buffer[1]>='0') && (u8_Message_Buffer[1]<='9') && (u8_Message_Buffer[2]>='0') && (u8_Message_Buffer[2]<='9'))
            {
            s16_addr = ((u8_Message_Buffer[1] - '0') << 4);
            s16_addr += (u8_Message_Buffer[2] - '0');
          }
       }

       if (u8_Drive_Id == s16_addr )
       {
            u8_Comms_State |= ADDRESSED_MASK;
            u8_Comms_State &= ~GLOBAL_ADDRESSED_MASK;
       }
         else
        {
         // Unknown address
         u8_Comms_State &= ~ADDRESSED_MASK;
         u8_Comms_State &= ~GLOBAL_ADDRESSED_MASK;
       }
           break;
   }
   return 1;
}

void OmmitSubString(char startCh, char endCh)
{
   char *msg_ptr_start = u8_Message_Buffer;
   char *msg_ptr_end;

   while ((*msg_ptr_start != startCh) && (*msg_ptr_start != '\0')) msg_ptr_start++;

   if (*msg_ptr_start == startCh)
   {
      if (*(msg_ptr_start-1) == ' ') msg_ptr_start--;

      msg_ptr_end = msg_ptr_start;
      while ((*msg_ptr_end != endCh) && (*msg_ptr_end != '\0'))
      {
         msg_ptr_end++;
      }

      if (*msg_ptr_end == endCh) // Units were found - ommit the [...]
      {
         do
         {
            msg_ptr_end++;
            *msg_ptr_start = *msg_ptr_end;
         }
         while (*msg_ptr_end != '\0');
      }
   }
}

// This function clears the [units] from the recieved string
void OmmitUnits()
{
   OmmitSubString('[', ']');
}

void OmmitChecksum()
{
   OmmitSubString('<', '>');
}

//**********************************************************
// Function Name: CommsProcessor
// Description:
//       This functions is the communications processor manager.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void CommsProcessor(int drive)
{
   int return_value;
   int drive_backup = drive;

   drive = 0;
   if(BGVAR(s16_Serial_Type) != SERIAL_COMM_TYPE)
   {
      //ModbusProcessor(0); //Udi Test July 27 2014
      return;
   }
   drive = drive_backup;

   // Select the communications handler state
   switch( s16_Comms_Processor_State )
   {
      case PRE_PROCESSOR:
         // Execute parser pre-processor

         return_value = CommsPreProcessor();

         // Check the return value
         switch(return_value)
         {
            case 0:
            // A complete message has not yet been received. Do nothing
            // in the meantime.
            break;

            case 1:
            // A complete message has been received: check whether it may be the
            // addressing command.
               if ( u8_Message_Buffer[0] == BACKSLASH )
               {
               // Parse the addressing command
               if ( !ParseAddressCmd() )
               {
                        // syntax error was found in the addressing command
                        ProcessError(u8_Axis_Id,SYNTAX_ERROR);

                     // Set comms processor state to the Prompt Handler state
                        s16_Comms_Processor_State = PROMPT_HANDLER;
                  }
                  else
                  {
                     // Address command successfully parsed: Change the comms processor state
                           s16_Comms_Processor_State = CHANGE_ADDRESS;
                  }
               }
               else
               {
               // Ommit the units from the recieved string to allow sending the response of DUMP
               OmmitUnits();
               // Check that the drive is addressed.
               if (DRIVE_ADDRESSED)
               {
                   s16_Comms_Processor_State = PARSING;
           }

               }
            break;

         case 2:
            // A comms error was detected: process the error
            ProcessError(u8_Axis_Id,COMMS_ERROR);

            // Set comms processor state to the Prompt Handler state
               s16_Comms_Processor_State = PROMPT_HANDLER;

            // Initialize the message buffer
               InitMsgBuffer();
         break;

         case 3:
            // A CheckSum error was detected: process the error
                  ProcessError(u8_Axis_Id,INVALID_CHECK_SUM);
            // Set comms processor state to the Prompt Handler state
               s16_Comms_Processor_State = PROMPT_HANDLER;

            // Initialize the message buffer
               InitMsgBuffer();
         break;

         }
      break;

      case CHANGE_ADDRESS:
         // Check if the drive is addressed, but not Globally addressed
         if ( DRIVE_ADDRESSED && !DRIVE_GLOBALY_ADDRESED )
               s16_Comms_Processor_State = PROMPT_HANDLER;
         else
            s16_Comms_Processor_State = PRE_PROCESSOR;
      break;

      case PARSING:
         // Execute the Parser
         return_value = ParserManager(drive);

         // Examine the return value
         switch(return_value)
         {
            case 0:
               // Parser not yet finished: carry on
            break;

            case 1:
               // Parser successfully completed: Initialize the message buffer
               InitMsgBuffer();

               // Set the communications processor state to Execution. Execution will
               // begin the next time the background is executed.
                 s16_Comms_Processor_State = EXECUTION;
            break;

            case 2:
               // Invalid mnemonic: Process the error
                 ProcessError(u8_Axis_Id,UNKNOWN_COMMAND);

               // Set comms processor state to the Prompt Handler state
                 s16_Comms_Processor_State = PROMPT_HANDLER;

               // Initialize the message buffer
                 InitMsgBuffer();
            break;

            case 3:
               // Syntax error: Process the error
               ProcessError(u8_Axis_Id,SYNTAX_ERROR);

               // Set comms processor state to the Prompt Handler state
                 s16_Comms_Processor_State = PROMPT_HANDLER;

               // Initialize the message buffer
                 InitMsgBuffer();
            break;

            case 4:
               // Invalid parameter: Process the error
                 ProcessError(u8_Axis_Id,VALUE_OUT_OF_RANGE);

               // Set comms processor state to the Prompt Handler state
                 s16_Comms_Processor_State = PROMPT_HANDLER;

               // Initialize the message buffer
               InitMsgBuffer();
            break;

             case 5:
               // Set comms processor state to the Prompt Handler state
                 s16_Comms_Processor_State = PROMPT_HANDLER;

               // Initialize the message buffer
                 InitMsgBuffer();
                 break;

             case 6:
               // Valid mnemonic but not programmable: Process the error
                 ProcessError(u8_Axis_Id,NOT_PROGRAMMABLE);

               // Set comms processor state to the Prompt Handler state
                 s16_Comms_Processor_State = PROMPT_HANDLER;

               // Initialize the message buffer
                 InitMsgBuffer();
              break;
         }
      break;

      case EXECUTION:
         if (((!DRIVE_GLOBALY_ADDRESED)     &&            /* drive adressed but not globally */
              (DRIVE_ADDRESSED)               )
              || 
              (BGVAR(u16_Foe_ParamsFile_State) == FOE_PARAMS_FILE_DWLD_WAIT_PARSE_FINISH))
         {
            // Call the execution handler.
            return_value = ExecutionHandler();

            switch( return_value )
            {
               case SAL_NOT_FINISHED:
                  // Execution not yet finished
               break;

               case SAL_SUCCESS:
                  // Execution successful: Set comms processor state to the Prompt Handler state
                  s16_Comms_Processor_State = PROMPT_HANDLER;
               break;

               default:
                     // Execution not successful. The 'return_value' gives the error
                     SetMostRecentError(u8_Axis_Id, return_value );

                  // Set comms processor state to the Prompt Handler state
                  s16_Comms_Processor_State = PROMPT_HANDLER;
               break;
            }
         }
         else if ((DRIVE_ADDRESSED) || (BGVAR(u16_Script_Str_Cntr)>0))  //if both drives selected
         { /* drive adressed globally */
            s16_Comms_Processor_State = EXECUTION_AXIS1;
         }
         else
            s16_Comms_Processor_State = PRE_PROCESSOR; //Udi Test March 25, 2014

      break;

      case EXECUTION_AXIS1: // This state executed only in case that both drives selected
//         u8_Drive_Id = AXIS1_SELECTED;//!!!!!!!!!!!!!!!!!!!!!!!!!!!
            return_value = ExecutionHandler();

            // if function finished execute the same function on axis2
             //if (return_value) s16_Comms_Processor_State = EXECUTION_AXIS2;
          if (return_value) s16_Comms_Processor_State = PRE_PROCESSOR;
          break;

      case EXECUTION_AXIS2:
      //      u8_Drive_Id = AXIS2_SELECTED;
            return_value = ExecutionHandler();

            // if function finished reset state mashine
             if (return_value) s16_Comms_Processor_State = PRE_PROCESSOR;
      break;

       case PROMPT_HANDLER:
         // In this state the Asynchronous Error message and the Prompt are returned
         // First call the prompt handler, and check the return value
          u8_Prompt_Indicator=1;
           return_value = PromptHandler();

         // Check the return value
           switch(return_value)
           {
            case 0:
               // Not yet finished
            break;

            default:
               // Finished: return comms processor to the Pre-processor state
               s16_Comms_Processor_State = PRE_PROCESSOR;
            u8_Prompt_Indicator=0;
            break;
         }
      break;
   }
}


//**********************************************************
// Function Name: PromptHandler
// Description:
//       This function is called to write the prompt to the output buffer.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int PromptHandler(void)
{
   int return_value=0;

   if (EMPTY_OUTPUT_BUFFER)
   {
      return_value = 1;
      if (DRIVE_ADDRESSED)
      {
         // First send the asynchronous error message (if enabled)
         AsyncErrorMessage();

         // First check if prompt is enabled AND drive is not globally addressed
         if ( (BGVAR_COMM(u8_Comms_Options) & PROMPT_MASK) &&  (!DRIVE_GLOBALY_ADDRESED) )
         {

          if (u8_Drive_Id != 0)
         {
            // Write first prompt character to output buffer
            if(u8_Drive_Id < 10)
            {
                  WriteOutputBuffer(u8_Drive_Id +'0');
            }
            else
            {
                  WriteOutputBuffer((u8_Drive_Id >> 4) + '0');
                  WriteOutputBuffer((u8_Drive_Id & 0x0F) + '0');
            }
         }
         else
         {
               WriteOutputBuffer(DASH);
         }

            // Write second prompt character to output buffer: this is always the DASH
            WriteOutputBuffer(DASH);

            // Write third prompt character to output buffer: this is always the >
            WriteOutputBuffer(ARROW_RIGHT);
       }
      }
   }

    return return_value;
}

//**********************************************************
// Function Name: PromptHandler
// Description:
//       This function executes a number of actions necessary to process
//         the occurrence of an error in message processing.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void ProcessError(int drive,int error_code)
{
   SetMostRecentError(drive,error_code);
}

//**********************************************************
// Function Name: PrintString
// Description:
//       Prints string to u8_Output_Buffer , if len=0 accepted ,will print to '\0'
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void PrintString(char* str,int len)
{
   int i=0;

   // check len range
   if ( (len<0) || (len>COMMS_BUFFER_SIZE) || (str==0)) return;

    if (len>0)
    {
      for (i=0;i<len;i++)
         WriteOutputBuffer(str[i]);
    }

    if (len==0)
    {
      while (str[i] != '\0')
         WriteOutputBuffer(str[i++]);
    }

}

//**********************************************************
// Function Name: PrintStringCrLf
// Description:
//       Prints string to u8_Output_Buffer , if len=0 accepted ,will print to '\0'
//       Print CrLf
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void PrintStringCrLf(char* str,int len)
{
   PrintString(str,len);
   PrintCrLf();
}

//**********************************************************
// Function Name: UnsignedIntegerToAscii
// Description:
//       Converts unsigned integer to string
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
char* UnsignedIntegerToAscii(unsigned long uint_val)
{
   unsigned int modulo;
   unsigned long result;
   char buff[10];
   int j,i=0;

   result = uint_val;
   if (result == 0)
   {
     u8_Response[0] = '0';
     u8_Response[1] = '\0';
   }
   else
   {
      while (result > 0)
      {
         // Take the modulo-10 of the integer value
         modulo = result % 10;

         // Convert this to ASCII and add it to the buffer
         buff[i++] = (modulo + 0x30);

         // Divide integer value by 10
         result /= 10;
      };
      i--;

         for (j=0;j<=i;j++)
            u8_Response[j] = buff[i-j];

         u8_Response[i+1] = '\0';
   }

   return u8_Response;
}

char* UnsignedLongLongToAscii(unsigned long long uint_val)
{
   unsigned int modulo;
   unsigned long long result;
   char buff[21];
   int j,i=0;

   result = uint_val;
   if (result == 0)
   {
      u8_Response[0] = '0';
      u8_Response[1] = '.';
      u8_Response[2] = '0';
      u8_Response[3] = '0';
      u8_Response[4] = '0';
      u8_Response[5] = '\0';
   }
   else
   {
      while (result > 0)
      {
         // Take the modulo-10 of the integer value
         modulo = result % 10;

         // Convert this to ASCII and add it to the buffer
         buff[i++] = (modulo + 0x30);

         // Divide integer value by 10
         result /= 10;
      };

      // Handle decimal part
      if (i<=3) // If less than 1
      {
         for (j=i;j<3;j++)
            buff[i++] = '0';
         buff[i++] = '.';
         buff[i] = '0';
      }
      else // Insert decimal point
      {
         for (j=i;j>3;j--)
            buff[j] = buff[j-1];
         buff[3] = '.';
      }

         for (j=0;j<=i;j++)
            u8_Response[j] = buff[i-j];

         u8_Response[i+1] = '\0';
   }
   return u8_Response;
}

char* SignedLongLongToAscii(long long int_val)
{
   int i = 0;

    if ( int_val >= 0 )
    {
      return UnsignedLongLongToAscii(int_val);
    }
   else
   {
      UnsignedLongLongToAscii(-int_val);

      // Add the '-' at the begining
      while (u8_Response[i++] != '\0');

      while (i>0)
      {
         u8_Response[i] = u8_Response[i-1];
         i--;
      }
      u8_Response[0] = '-';
   }
   return u8_Response;
}

//**********************************************************
// Function Name: SignedIntegerToAscii
// Description:
//       Converts signed integer to string
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
char* SignedIntegerToAscii(long int_val)
{
   unsigned int modulo;
   unsigned long result;
   char buff[10];
   int j,i=0;

   result = (unsigned long)int_val;


   if ( int_val >= 0 )
   {
      return UnsignedIntegerToAscii(result);
   }
   else
   {
         result = -int_val;
         while (result > 0)
         {
         // Take the modulo-10 of the integer value
            modulo = result % 10;

         // Convert this to ASCII and add it to the buffer
         buff[i++] = (modulo + 0x30);

         // Divide integer value by 10
            result /= 10;
         };
         i--;

         u8_Response[0]='-';

        for (j=0;j<=i;j++)
            u8_Response[j+1] = buff[i-j];

         u8_Response[j+1] = '\0';
   }

   return u8_Response;

}

//**********************************************************
// Function Name: UnsignedIntegerToAscii
// Description:
//       Converts unsigned integer to string
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
char* UnsignedInt64ToAscii(unsigned long long u64_val)
{
   unsigned int modulo;
   unsigned long long result;
   char buff[20];
   int j,i=0;

   result = u64_val;
   if (result == 0)
   {
     u8_Response[0] = '0';
     u8_Response[1] = '\0';
   }
   else
   {
      while (result > 0)
      {
         // Take the modulo-10 of the integer value
         modulo = result % 10;

         // Convert this to ASCII and add it to the buffer
         buff[i++] = (modulo + 0x30);

         // Divide integer value by 10
         result /= 10;
      };
      i--;

         for (j=0;j<=i;j++)
            u8_Response[j] = buff[i-j];

         u8_Response[i+1] = '\0';
   }

   return u8_Response;
}

//**********************************************************
// Function Name: CANUnsignedIntegerToAscii
// Description:
//       Converts unsigned integer to string
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int CANUnsignedInt64ToAscii(unsigned long long u64_val,char *InStr)
{
   unsigned int modulo;
   unsigned long long result;
   char buff[20];
   int j,i=0;
   int counter=0;

   result = u64_val;
   if (result == 0)
   {
     counter=1;
     *InStr = '0';
     *(InStr+1) = '\0';
   }
   else
   {
      while (result > 0)
      {
         // Take the modulo-10 of the integer value
         modulo = result % 10;

         // Convert this to ASCII and add it to the buffer
         buff[i++] = (modulo + 0x30);

         // Divide integer value by 10
         result /= 10;
      };
      i--;


         for (j=0;j<=i;j++)
           *(InStr+j) = buff[i-j];

         *(InStr+i+1) = '\0';
       counter=i+1;
   }
   return counter;
}

//**********************************************************
// Function Name: SignedInt64ToAscii
// Description:
//       Converts signed 64bit integer to string
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
char* SignedInt64ToAscii(long long s64_val)
{
   unsigned int modulo;
   unsigned long long result;
   char buff[20];
   int j,i=0;

   result = (unsigned long long)s64_val;


   if ( s64_val >= 0 )
   {
      return UnsignedInt64ToAscii(result);
   }
   else
   {
         result = -s64_val;
         while (result > 0)
         {
         // Take the modulo-10 of the integer value
            modulo = result % 10;

         // Convert this to ASCII and add it to the buffer
         buff[i++] = (modulo + 0x30);

         // Divide integer value by 10
            result /= 10;
         };
         i--;

         u8_Response[0]='-';

        for (j=0;j<=i;j++)
            u8_Response[j+1] = buff[i-j];

         u8_Response[j+1] = '\0';
   }

   return u8_Response;
}

char* DecimalPoint32ToAscii(long s32_val)
{
   unsigned char u8_sgn = 0;
   unsigned char u8_buf[15];
   char s8_cntr = 0;
   unsigned char u8_index;

   if (s32_val < 0)
   {
      u8_sgn = 1;
      s32_val = -s32_val;
   }

   if (s32_val == 0)
   {
      u8_Response[0] = u8_Response[2] = u8_Response[3] = u8_Response[4] = '0';
      u8_Response[1] = '.';
      u8_Response[5] = '\0';
      return (u8_Response);
   }

   s8_cntr = 0;
   while (s32_val)
   {
      u8_buf[s8_cntr++] = ((s32_val % 10) + '0');
      s32_val = s32_val / 10;
      if (s8_cntr == 3)
      {
         u8_buf[s8_cntr++] = '.';
         if (!s32_val) u8_buf[s8_cntr++] = '0';
      }
   }

   while (s8_cntr < 3)
   {
      u8_buf[s8_cntr++] = '0';
      if (s8_cntr == 3)
      {
         u8_buf[s8_cntr++] = '.';
         u8_buf[s8_cntr++] = '0';
      }
   }

   if (u8_sgn) u8_buf[s8_cntr++] = '-';

   s8_cntr --;
   for (u8_index = 0;s8_cntr >= 0;u8_index++)
   {
      u8_Response[u8_index] = u8_buf[s8_cntr];
      s8_cntr--;
   }
   u8_Response[u8_index] = '\0';
   return (u8_Response);
}


//**********************************************************
// Function Name: PrintUnsignedInteger
// Description:
//       Prints unsigned integer to u8_Output_Buffer
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void PrintUnsignedInteger(unsigned int num)
{
   unsigned long ulong_num;
   ulong_num = 0L;
   ulong_num = num;
   PrintString(UnsignedIntegerToAscii(ulong_num),0);
}

void PrintUnsignedDecimal(unsigned int num)
{
   unsigned long ulong_num;
   if (num < 100)
   {
      PrintChar('0');
      if (num < 10)
      {
         PrintChar('0');
      }
   }
   ulong_num = num;

   PrintString(UnsignedIntegerToAscii(ulong_num),0);
}

void PrintSignedLong(long num)
{
   PrintString(SignedIntegerToAscii((long)num),0);
}

void PrintSignedLongLong(long long num)
{
   PrintString(SignedLongLongToAscii((long long)num),0);
}

void PrintUnsignedLong(unsigned long num)
{
   PrintString(UnsignedIntegerToAscii(num),0);
}

void PrintUnsignedLongLong(unsigned long long num)
{
   PrintString(UnsignedLongLongToAscii(num),0);
}

void PrintSignedInt16(int s16_int)
{
   PrintString(SignedInt64ToAscii((long long)s16_int),0);
}

void PrintSignedInt32(long s32_int)
{
   PrintString(SignedInt64ToAscii((long long)s32_int),0);
}

void PrintSignedInt64(long long s64_int)
{
   PrintString(SignedInt64ToAscii(s64_int),0);
}

void PrintUnsignedInt16(unsigned int u16_int)
{
   unsigned long long u64_temp = (unsigned long long)u16_int;
   u64_temp &= 0x000000000000FFFFLL; //from some reason compiler extends sign bits even for "unsigned" type casting , therefore clear it
   PrintString(UnsignedInt64ToAscii(u64_temp),0);
}

void PrintUnsignedInt32(unsigned long u32_int)
{
   unsigned long long u64_temp = (unsigned long long)u32_int;
   u64_temp &= 0x00000000FFFFFFFFLL; //from some reason compiler extends sign bits even for "unsigned" type casting , therefore clear it

   PrintString(UnsignedInt64ToAscii(u64_temp),0);
}

void PrintUnsignedInt64(unsigned long long u64_int)
{
   PrintString(UnsignedInt64ToAscii(u64_int),0);
}


//**********************************************************
// Function Name: PrintSignedInteger
// Description:
//       Prints unsigned integer to u8_Output_Buffer
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void PrintSignedInteger(int num)
{
   long long_num;
   long_num = 0L;
   long_num = num;

   PrintString(SignedIntegerToAscii(long_num),0);
}


void PrintDecInAsciiHex(unsigned long long val,int len)
{
   PrintString(DecToAsciiHex(val,len),0);
}

char * DecToAsciiHex(unsigned long long dec_val,int len)
{
   unsigned int modulo;
   unsigned long long result;
   char buff[20];
   int j,i=0;

   if ((len < 0) || (len > 16)) return 0;

   result = dec_val;

   u8_Response[0] = 'H';

   while ((result > 0) && (i<len))
   {
      // Take the modulo-16 of the integer value
      modulo = result % 16LL;

      // Convert this to ASCII and add it to the buffer
      if (modulo < 10)
      {
         buff[i++] = (modulo + 0x30);
      }
      else
      {
            buff[i++] = (modulo + 0x37);
        }

         // Divide integer value by 16
        result /= 16LL;
   }

    // Pad with zeros
    while (i<len)
      buff[i++] = '0';

   i--;

   for (j=0;j<=i;j++)
      u8_Response[j+1] = buff[i-j];

   u8_Response[j+1] = '\0';

   return u8_Response;
}

void PrintHexChar(unsigned int hex)
{
   if (hex < 0x0A) PrintChar(hex + '0');
   else PrintChar(hex - 10 + 'A');
}

void PrintChar(char c)
{
   WriteOutputBuffer(c);
}

void PrintCrLf(void)
{
   WriteOutputBuffer(CR);
   WriteOutputBuffer(LF);
}


int SalQueryCommOption(int drive,int opt)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u8_Comms_Options) & opt) return 1;
   else return 0;
}



char CheckSum (char *InStr,int len)
{
    int i=0;
    char Res=0;
    while (i<len || *(InStr+i)!=0)
       Res+=*(InStr+i++);
    Res&=0x00ff;
    return Res;
}


//**********************************************************
// Function Name: SalSerialBaudRateCommand
// Description:
//          This function is called in response to the BAUDRATE command.
//
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
int SalSerialBaudRateCommand(long long lparam, int drive)
{
   unsigned long u32_param;

   drive += 0;     // defeat compiler remark

//  //For =S= Do not set baud rate in Factory restore. New setting will become active only after power cycle the drive.
//    if(((BGVAR(u16_Lexium_Bg_Action_Applied) & LEX_DO_FACTORY_RESTORE) == LEX_DO_FACTORY_RESTORE) &&
//      ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)))
//    return SAL_SUCCESS;
   u32_param = (unsigned long)lparam;

   // wait for the output buffer to be empty
   if (!EMPTY_OUTPUT_BUFFER) return SAL_NOT_FINISHED;
   if (u32_param == 4800L)
   {
      ScibRegs.SCIHBAUD    = 0x0002;
      ScibRegs.SCILBAUD    = 0x008A;
   }
   else if (u32_param == 9600L)
   {
      ScibRegs.SCIHBAUD    = 0x0001;
      ScibRegs.SCILBAUD    = 0x0045;
   }
   else if (u32_param == 19200L)
   {
      ScibRegs.SCIHBAUD    = 0x0000;
      ScibRegs.SCILBAUD    = 0x00A2;
   }
   else if (u32_param == 38400L)
   {
      ScibRegs.SCIHBAUD    = 0x0000;
      ScibRegs.SCILBAUD    = 0x0050;
   }
   else if (u32_param == 57600L)
   {
      ScibRegs.SCIHBAUD    = 0x0000;
      ScibRegs.SCILBAUD    = 0x0035;
   }
   else if (u32_param == 115200L)
   {
      ScibRegs.SCIHBAUD    = 0x0000;
      ScibRegs.SCILBAUD    = 0x001A;
   }
   else
   {
      return (VALUE_OUT_OF_RANGE);
   }

   drive = 0;
   BGVAR(u32_Serial_Baud_Rate) = u32_param;
   drive = 1;
   BGVAR(u32_Serial_Baud_Rate) = u32_param;

   if (u16_Product == SHNDR_HW)
   {
      ScicRegs.SCIHBAUD = ScibRegs.SCIHBAUD;
      ScicRegs.SCILBAUD = ScibRegs.SCILBAUD;
   }

   return SAL_SUCCESS;
}

