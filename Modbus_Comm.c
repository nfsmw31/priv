//###########################################################################
//
// FILE: BG/Modbus_Comm.c
//
// TITLE:  Clone of Ser_Comm.c to handle Modbus communication
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 13 Jun 2002 | D.R. | Creation
// 0.02|  09 Apr 2013| Udi  | Based on Ser_comm.c  - with functionality acording to Modbus spec.
//##################################################################

#include "DSP2834x_Device.h"
#include "Modbus_Crc.h"
#include <string.h>

#include "design.def"
#include "Drive_Table.def"
#include "Err_Hndl.def"
#include "FPGA.def"
#include "Flash.def"
#include "i2c.def"
#include "Modbus_Comm.def"
#include "MultiAxis.def"
#include "PtpGenerator.def"
#include "Record.def"
#include "Ser_Comm.def"

#include "Drive_Table.var"
#include "Display.var"
#include "Drive.var"
#include "Extrn_Asm.var"
#include "Exe_Hndl.var"
#include "FltCntrl.var"
#include "init.var"
#include "Modbus_Comm.var"
#include "Ser_Comm.var"
#include "Motor.var"
#include "Record.var"
#include "Velocity.var"
#include "FlashHandle.var"
#include "PtpGenerator.var"
#include "Prototypes.pro"

//Do not forget to update EDS and P10-26 in the excel (min max default) with Number of elements!
int PParam_In_MonAccess[] =
{1001, 1002, 8, 46, 400, 401, 402, 403, 404, 1010, 1015, 537, 538, 425, 1031, 928, 930, 934, 557, 558,
      9, 10, 11, 12, 13}; //July 16 2015, add mapping param - p0-09 -p0-013 CQ1248

//Do not forget to update EDS and P10-28 in the excel (min max default) with Number of elements!
int PParam_In_Dat1Access[] =
{1100, 1101, 1102, 1103, 1104, 1105, 1106, 1107, 1108, 1109, 1110, 1111, 1114, 1115, 1116, 1127,
      1139, 1140, 1141, 1142, 537, 1149, 1150, 1153, 1154, 1155, 1177};
// 1112, 1113, 1117, 1118,  1151, 1152, 1156, 1164, 1165, 1167, 1168, 1169, //Removed until implemented.





/// Simple Byte Manipulation

//Extract High (or low) 8 bit of 16 bit
char L8(int16 i)
{
   return (i & 0x00FF);
}
char H8(int i)
{
   return ((i>>8) & 0x00FF);
}


//Combine 2 bytes to ceate int16
int16  bytes2Int16(int high,int low )
{
   return ((high << 8) + low );
}

//Combine 4 bytes to create int32
// b0 is most significant
int32  bytes2Int32(int  b0, int  b1, int  b2, int  b3)
{
   return (((int32)b0 << 24) + ((int32)b1 << 16) + ((int32)b2 << 8) + (b3));
}



//Combine 8 bytes to create int64
// b0 is most significant
long long  bytes2Int64(int  buff[])
{
   long long response = 0;
   int i;
   for(i=0; i < 8; i ++)
   {
      response += (long long)buff[i] << (8*(8-i));
   }
   return response;
}




//extract byte Num Idx  from Uint32
int byteFrom32 ( Uint32 u, int idx)
{
   return (((u>>(8*(3-idx))) & 0x000000FF));
}


// extract byte Num Idx from long long
int byteFrom64 ( long long  l, int idx)
{
   return (((l>>(8*(7-idx))) & 0x000000FF));
}


void Append_1_BytesTxBuff(int value)
{
   MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = L8(value);
}

void Append_2_BytesTxBuff(int value)
{

   MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = H8(value);
   MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = L8(value);
}


void Append_4_BytesTxBuff(long value)
{
   int i;
   for(i = 0; i < 4; i++)
   {
      MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = byteFrom32(value,i);  //ith byte
   }
}

void Append_8_BytesTxBuff(long long value)
{
   int i;
   for(i = 0; i < 8; i++)
   {
      MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = byteFrom64(value,i);  //ith byte
   }
}


//**********************************************************
// Function Name: SalReadMonaccessBlock
// Description:
// Stub for pparam 10-27.and 10-29. Never called.
// Actual work done  in reading parm list as defined in:  PParam_In_MonAccess[]
// Placing the results in Modbus-Transmit buffer.
// Author: Udi
//**********************************************************
int SalReadMonaccessBlock(long long *data)
{
   *data += 0 ;
   return SAL_SUCCESS;
}



//**********************************************************
// Function Name: SalWriteWordOrder
// Description:
// Set the word order of parameter values for read and write PParam.
// Note: Word Order is ignored after Ping command, until  logout.
// Author: Udi
//**********************************************************
int SalWriteWordOrder(long long data, int drive)
{
   REFERENCE_TO_DRIVE;

   BGVAR(s16_MB_Word_Order) = (int)  data ;
   if(u16_Login != 3)
      s16_MB_Word_Order_In_Use = BGVAR(s16_MB_Word_Order);
   return SAL_SUCCESS;
}



//**********************************************************
// Function Name: SalReadLogin
// Description:
//   This function called in response to read P?? MAND.LOGIN
//   to get the the status of login
//
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int SalReadLogin(long long *data,int drive)
{

   REFERENCE_TO_DRIVE;
   *data = u16_Login;
   return SAL_SUCCESS;
}



//**********************************************************
// Function Name: SalWriteLogin
// Description:
//   This function called in response to write LOGIN
//
//
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteLogin(long long data,int drive)
{
   long s32_temp;

   switch (data)
   {
      case 0: //Logout:  remove Forced IO, restore word order, release access right

         AX0_u16_P3_06_SDI = 0; // Remove FORCE from Inputs

         // Remove FORCE from outputs: same as writing 400 to P2-08
         // To Pass CiA (Can Open certification test) only do this if already login
         if (u16_Login != 0)
         {
         BGVAR(u16_Special_Factory_Settings) = 400;
         BGVAR(u16_Force_Output_Enable) = 0;
         }
         // restore word Order
         s16_MB_Word_Order_In_Use  = BGVAR(s16_MB_Word_Order);

         // Release access-rights if given to the commissioning-tool / Modbus
         s32_temp = 0;
         ExecutePParam(drive, 1032, OP_TYPE_WRITE, &s32_temp, LEX_CH_MODBUS_RS485);

         //Reset node-guard detection time P10-03 to 0 on every logout
         BGVAR(u16_MB_Nguard_Login) = 0;

         u16_Login = 0;
         return SAL_SUCCESS;

      case 3: //Login as DTM (Comisionning tool)
         u16_Login = 3;
         return SAL_SUCCESS;

      default:
         return VALUE_IS_NOT_ALLOWED;

   }

}



//**********************************************************
// Function Name: SalReadConfigLock
// Description:
//   This function called in response to read P10-06
//   to get the the status of Config lock
//
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int SalReadConfigLock(long long *data,int drive)
{

   drive +=0;  // to avoid remark
   *data = u16_Config_Lock;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWriteConfigLock
// Description:
//   This function called in response to write P10-06
//   to set the the status of Config lock
//       lock options: 0: Idle; 1: Read access; 2: Write access;
//                     3: Exit (same as 0) ; 4: Save and exit
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
long int s32_Save_Time_Capture = 0;
int SalWriteConfigLock(long long data,int drive)
{

   drive +=0;  // to avoid remark


   //Check transition : (As requested by Schnider, not needed by the code).
   // Allowed:    0 => 1 or 2 ;  1 or 2 => 0, or 3, or 4
   // No Allowed: 1 => 2, 2 => 1.
   // Note: 0, 3 and 4 ends with lock = 0;
   if(((u16_Config_Lock == CONFIG_LOCK_READ) && (data == CONFIG_LOCK_WRITE)) ||
         ((u16_Config_Lock == CONFIG_LOCK_WRITE) && (data == CONFIG_LOCK_READ) ))
      return VALUE_IS_NOT_ALLOWED;

   switch (data)
   {
      case CONFIG_LOCK_EXIT:
      case CONFIG_UNLOCK:
         //In upload or download session: abort it.
         MB_ST.s16_Umas_State = UMAS_STATE_READY;

         u16_Config_Lock =  CONFIG_UNLOCK;
         return SAL_SUCCESS;

      case CONFIG_LOCK_READ:
         u16_Config_Lock = CONFIG_LOCK_READ;
         return SAL_SUCCESS;


      case CONFIG_LOCK_WRITE:
         MB_ST.s16_Last_Error_Code = SAL_SUCCESS ;
         MB_ST.s16_Addr_Last_Error = 0;

         u16_Config_Lock = CONFIG_LOCK_WRITE;
         s32_Save_Time_Capture = 0;
         return SAL_SUCCESS;




      case CONFIG_LOCK_SAVE_EXIT: //Save only if no error
         u16_Config_Lock = CONFIG_UNLOCK;
         //Save all parameters for this session ?
         if(MB_ST.s16_Addr_Last_Error != 0)
            return VALUE_IS_NOT_ALLOWED;
         //SalTriggerNvSaveViaPParam(1,0);
         //Test Apr 20, 2015: If in box programming, do not send ack until save is completed.
        // if (u16_Power_Board_Off == 1) //Drive run on 5v from MultiLoader, power and FPGA are off
        //  {
         //   if(s32_Save_Time_Capture == 0)
         //   {
         //      s32_Save_Time_Capture = Cntr_1mS; // Set base for save opration
         //      return SAL_NOT_FINISHED;
         //   }
         //   else
         //   {
         //      if(!PassedTimeMS(2000L,s32_Save_Time_Capture))
         //         return SAL_NOT_FINISHED;
         //   }

            // if(  ((BGVAR(u16_Lexium_Bg_Action) & LEX_DO_NV_SAVE) != 0) ||
            //      ((BGVAR(u16_Lexium_Bg_Action_Applied) & LEX_DO_NV_SAVE) !=  0) )
            //    return SAL_NOT_FINISHED;

            //PassedTimeMS(MB_ST.s32_ModbusFrameMaxDelay,s32_Save_Time_Capture))

         // }
         s32_Save_Time_Capture = 0;
         return SAL_SUCCESS;


      default:
         return VALUE_IS_NOT_ALLOWED;

   }


   //return VALUE_IS_NOT_ALLOWED;

}


//**********************************************************
// Function Name: SalReadInvalidParam
// Description:
//   This function called in response to P10-10
//   to return Modbus address of parameter with Write or config  error
//
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int SalReadInvalidParam(long long *data,int drive)
{

   drive +=0;  // to avoid remark
   *data = MB_ST.s16_Addr_Last_Error;
   return SAL_SUCCESS;
}




//**********************************************************
// Function Name: InitModbusSerCommunication
// Description:
//   Initialize Serial communication variables and peripherials
//   UART initialization is in "InitSci" and "InitSerCommunication2"
//
// Author: Dimitry
// Algorithm:
// Revisions:
// 09 Apr 2013| Udi  | Addapt to Modbus
//**********************************************************
void InitModbusSerCommunication()
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //Default to Modbus if Schnider (LXM28 hardware)
   if (u16_Product == SHNDR_HW)
   {
      BGVAR(s16_Serial_Type) = MODBUS_COMM_TYPE;

      //Set echo =0 as 485 communication is half-duplex
      SalEchoCommand(0LL,0);
   }

   // start clean
   memset(&MB_ST, 0, sizeof(MB_ST));


   //Actual initialization values

   BGVAR(u16_Num_Monaccess)  = sizeof(PParam_In_MonAccess);
   BGVAR(u16_Num_Dat1access) = sizeof(PParam_In_Dat1Access);



   if((MB_ST.s32_ModbusFrameMaxDelay < 20 ) || (MB_ST.s32_ModbusFrameMaxDelay > 60))
      MB_ST.s32_ModbusFrameMaxDelay = 40;

   MB_ST.s16_Modbus_DriveAddr = BGVAR(u16_P3_00_ADR_Current);
   s16_MB_Word_Order_In_Use  = BGVAR(s16_MB_Word_Order);

   resetStatus(TRUE, TRUE);


}



//**********************************************************
// Function Name: ModbusTimeOutResponse
// Description:
//     Called when modbus time out accured - to display a meesage and optionaly stop the drive.
// Author: Udi
//**********************************************************
#pragma CODE_SECTION(ModbusTimeOutResponse, "ramfunc_2");
void ModbusTimeOutResponse()
{
   int drive = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If Comunication error during login ignore P3-03 setting
   //Confirmed in email from Juergen Fiess Feb 2, 2015: P3-03 does not effect if in "login"
   if((u16_Login == 3 ) || (u16_P3_03_FLT == 1)) //Set fault and Stop by Dec
   {
      // set bit 0 to signal fault
      u16_Modbus_NodeGuard_Flt |= MODBUS_NODEGUARD_FLT_INDC_MASK;

   }

   // If Comunication error during login - force logout - in next BG
   if(u16_Login == 3 )
   {
      u16_Force_Logout = 1;
      return;
   }

   // If Comunication error during NOT login: and P3-03 say only  warning
   if(u16_P3_03_FLT == 0) //Warning
   {
      u16_Modbus_NodeGuard_Flt |= MODBUS_NODEGUARD_WRN_INDC_MASK;
   }
}

//**********************************************************
// Function Name: SciModbusInputProcessor
// Description:
//   Modbus copy of serial commuication.
//   Read character from SCI and put it to recive buffer
//   This function called from real time
// Author: Dimitry
// Algorithm:
// Revisions:
// 09 Apr 2013| Udi  | Addapt to Modbus
//                    frame ditected by  silent period on the line. (3.5 char)
//**********************************************************

#pragma CODE_SECTION(SciModbusInputProcessor, "ramfunc_2");
void SciModbusInputProcessor(void)
{
   static long s32_Char_Time_Capture;
   int drive = 0;
   unsigned int  u16_active_nguard = 0;


   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(BGVAR(s16_Serial_Type) != MODBUS_COMM_TYPE)
      return;

   //If Node guard is active, and if time expired - Stop the drive or display message.
   // Two Differnt Node guard exists: one when CT is login , the other for PLC
   if (u16_Login == 3)
      u16_active_nguard = BGVAR(u16_MB_Nguard_Login);
   else
      u16_active_nguard = BGVAR(u16_P3_04_CWD);

   if( (u16_active_nguard > 0) && (MB_ST.s32_Frame_Time_Capture > 0) &&
         PassedTimeMS((long ) u16_active_nguard, MB_ST.s32_Frame_Time_Capture))
      ModbusTimeOutResponse();
   else // clear warning
      u16_Modbus_NodeGuard_Flt &= ~MODBUS_NODEGUARD_WRN_INDC_MASK;

   //No input allowed when processing
   if(MB_ST.ModbusStatus != MB_WAIT_COMMAND)
      return;

   // If more then "FrameDelay" after last char: call this a full frame , let the parser test it
   if((s32_Char_Time_Capture > 0)&&
         PassedTimeMS(MB_ST.s32_ModbusFrameMaxDelay,s32_Char_Time_Capture))
   {
      MB_ST.ModbusStatus = MB_PARSE_COMMAND;
      MB_ST.s16_Transmit_Counter  = 0;
      MB_ST.s16_Transmit_Idx      = 0;
      s32_Char_Time_Capture = 0;
      // MB_ST.s32_Frame_Time_Capture = 0;
   }

   // AZ debug =S=
   // Normally =S= should not listen to SCI-B, but here it is done mainly for debug, assuming
   // no received data in both SCI-C and SCI-B.
   // To remove the debug, put the code below in a comment.

   while ( (ScibRegs.SCIFFRX.bit.RXFFST > 0) && (MB_ST.s16_Recive_Counter < MAX_RX-1))
   {
      //  Read data
      MB_ST.s16_Recive_Buff[(MB_ST.s16_Recive_Counter++)] = (unsigned char)ScibRegs.SCIRXBUF.bit.RXDT;

      //Restart timer with each new char
      s32_Char_Time_Capture = Cntr_1mS;
   }
}


//**********************************************************
// Function Name: SalSetModbusResponseDelayTime
// Description:
//   Calculate min time between reciving last char in a message and sending the response
//
// Author: Udi
// Algorithm:
//  Min time is max of
//      Should be (3.5 X "char time") as defind by baude rate (p3-01)
//                But I  use 1 ms constant, Because sending is in the 1 ms interupt
//      ms as defined by p3-07 (in 0.5 ms)
// Revisions:
//**********************************************************

// int SalSetModbusResponseDelayTime(int drive)
// {
//        drive += 0;
//
//        if(BGVAR(u16_P3_07_CDT) < 2)
//            MB_ST.s16_Modbus_Response_DelayTime = 1;
//        else
//            MB_ST.s16_Modbus_Response_DelayTime = BGVAR(u16_P3_07_CDT+1) / 2;
//
//
//        return SAL_SUCCESS;
// }
//

//**********************************************************
// Function Name: SciModbusOutputProcessor
// Description:
//   Read character from transmit buffer and put it to SCI (UART)
//   This function called from real time
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
// 09 Apr 2013| Udi  | Addapt to Modbus
//**********************************************************
#pragma CODE_SECTION(SciModbusOutputProcessor, "ramfunc_2");
void SciModbusOutputProcessor(void)
{

   int drive = 0;
   long s32_msg_time;
   //  static int dbg_counter = 0;
   int s16_min_delay = 2;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(BGVAR(s16_Serial_Type) != MODBUS_COMM_TYPE)
      return;

   //Output only when complete answer is ready for send
   if(MB_ST.ModbusStatus != MB_SEND_ANSWER)
      return;

   if(MB_ST.s16_Modbus_Command_Addr == 0)
   { //Master broadcust message - no response allowed
      MB_ST.s16_Transmit_Idx =  MB_ST.s16_Transmit_Counter; //Fake as if all are send;
      return;
   }




   //Wait by p3-07 Response delay time. P3-07 is in 0.5 ms, so div by 2
   // But at least 2 ms. - wait 2 counts to make sure.

   if(BGVAR(u16_P3_07_CDT) > 4 )
      s16_min_delay = (BGVAR(u16_P3_07_CDT)  + 1) >> 1;
   if(!PassedTimeMS(s16_min_delay, MB_ST.s32_Frame_Time_Capture))

   {
      //  dbg_counter ++;
      return;
   }


   //Keep max message time, for debug
   if(MB_ST.s32_Frame_Time_Capture > 0)
   {
      s32_msg_time = Cntr_1mS - MB_ST.s32_Frame_Time_Capture ;
      if(Cntr_1mS < MB_ST.s32_Frame_Time_Capture)
         s32_msg_time = -s32_msg_time; // Not sure this is right, but it will be next month ..

      if(s32_Max_Msg_Time < s32_msg_time )
         s32_Max_Msg_Time = s32_msg_time;
   }




   //CanOpen Messages are send over CAN, not over serial.
   if(MB_ST.u16_Modbus_Over_Can == 1)
      return;

      while ((ScibRegs.SCIFFTX.bit.TXFFST < 0x0F) &&
            ( MB_ST.s16_Transmit_Counter > MB_ST.s16_Transmit_Idx))
      {
         //  Write data from transmit buffer to UART
         ScibRegs.SCITXBUF = MB_ST.s16_Transmit_Buff[ MB_ST.s16_Transmit_Idx++];
      }
   }

//**********************************************************
// Function Name: ModbusProcessor
// Description:
//       Main State machine for Modbus communication
// Author:  Udi
// Algorithm:
//   Called from the background:
//   1. State machine: Wait(and read), Parse, loop {Do, Apend} , and send.
//      Each state will move to the next after comlition, with the following excption:
//         a. "Add answer" may return to "Do Command" if more parameters to read/write
//         b. on failure - Send Negative response and return to wait command
//  2.Handle flags from the 1ms interrupt:
//         a. Logout if Modbus node guard detected communication lost.
//
// Revisions:
// 09 Apr 2013 First verion
//**********************************************************
void ModbusProcessor(int drive)
{
   drive += 0;  // to avoid remark

   if(BGVAR(s16_Serial_Type) != MODBUS_COMM_TYPE)
      return;
   // Perform requests made by 1 ms or real time routines
   // 1. Logout after communication lost
   if(u16_Login == 3 && u16_Force_Logout == 1 )
   {
      u16_Force_Logout = 0;
      SalWriteLogin(0,0);
      return;
   }

   switch(MB_ST.ModbusStatus)
   {
      case MB_WAIT_COMMAND:     break;                        // No-op - implemented in read function in 1ms interupt
      case MB_PARSE_COMMAND:    ModbusParser();      break;
      case MB_DO_ONE_COMMAND:   ModbusDoCmd();       break;
      // case MB_ADD_ONE_ANSWER:   ModbusAddReadResponse(); break; //May return to DO_ONE_COMMAND to loop throu n requested  parameters
      case MB_SEND_ANSWER:      ModbusSendAnswer();  break;
   }

}





//**********************************************************
// Function Name: ModbusDoCmd
// Description:
//       Execute one modbus command - for single parameter
//       Than Change the global state-machine to MB_APPEND_ANSWER, OR Close command
// Author:  Udi
//
// Revisions:
// 23 Apr 2013 First verion
//**********************************************************
void ModbusDoCmd()
{

   // Do the request operation
   switch (MB_ST.s16_Modbus_Request)
   {
      case MB_CMD_READ:          ModbusRead();     break;
      case MB_CMD_WRITE:         ModbusWrite();    break;
      case MB_CMD_DIAGNOSTIC:    ModbusDiag();     break;
      case MB_CMD_ENCAPSULATION: ModbusIdentify(); break;
      case MB_CMD_DATA_TRANSFER: ModbusDataTransfer();   break;

      default:                // Command Not Known/Used - Prepare Error Answer
         SendException(MB_ILLEGAL_FUNCTION);
         break;
   }

}





//**********************************************************
// Function Name: ModbusSendAnswer
// Description:
//       Wait for command to be send from transmit buffer to the UART (or over can)
//       Than Change the global state-machine to WAIT_Command
// Author:  Udi
//
// Revisions:
// 23 Apr 2013 First verion
//**********************************************************
void ModbusSendAnswer()
{
   int transmitDone = FALSE;

   //Wait until response is send out from the UART / (or CanOpen)
   // Then reset status.

   if (MB_ST.u16_Modbus_Over_Can == 1)
   {
      if ( MB_ST.s16_Transmit_Counter == MB_ST.s16_Transmit_Idx)
         resetStatus(FALSE, TRUE);
      return;

   }

   //Check correct UART by product
   if (u16_Product == SHNDR_HW)
   {
      if(ScicRegs.SCIFFTX.bit.TXFFST == 0)
         transmitDone = TRUE;
   }
   else
   {
      if(ScibRegs.SCIFFTX.bit.TXFFST == 0)
         transmitDone = TRUE;
   }

   if ( MB_ST.s16_Transmit_Counter == MB_ST.s16_Transmit_Idx &&
         transmitDone == TRUE )
   {
      resetStatus(FALSE, TRUE);
   }
}



//**********************************************************
// Function Name: resetStatus
// Description:
//       Initialize the MB_ST (Modbus Status) Stacture to wait to new command
//       But keep modbus counters , and if UMAS - keep session ID end more.
// Author:  Udi
// Algorithm:
// Revisions:
// 09 Apr 2013 First verion
//**********************************************************

void resetStatus(int Reset_UMAS, int Reset_Transmit)
{
   // For UMAS requests, some of the fields should stay across several Modbus requests.
   // Other fields are initialized at each requests.
   //if(MB_ST.s16_Modbus_Request != MB_CMD_DATA_TRANSFER )
   if(Reset_UMAS == TRUE)
   {
      MB_ST.s16_Umas_State = UMAS_STATE_READY;
      MB_ST.s16_Umas_Session = 0;
      MB_ST.s16_Umas_Packet_Size = 0;
      MB_ST.s16_Umas_Packet_Idx = 0;
      MB_ST.s16_partial_line_count = 0;
      MB_ST.s16_Recived_Data_Offset = 0;
      MB_ST.s16_HeaderSize  = 0;
      MB_ST.s16_Umas_Line_Size = 0;
   }

   MB_ST.s16_Umas_Request = 0;

   MB_ST.s16_Recive_Counter    = 0;
   if (Reset_Transmit == TRUE)
   {
      MB_ST.s16_Transmit_Counter  = 0;
      MB_ST.s16_Transmit_Idx      = 0;
  }

   MB_ST.s16_Modbus_Request    = 0;
   MB_ST.s16_Modbus_Param       = 0;
   MB_ST.s16_Modbus_Param_Count = 0;
   MB_ST.s16_Modbus_Param_Idx   = 0;
   MB_ST.s16_Modbus_First_Idx = 0;
   MB_ST.s16_Modbus_First_Write_Value = 0;
   MB_ST.s16_Modbus_Keep_Original_Request    = 0;
   MB_ST.s16_Modbus_Keep_Param    = 0;
   MB_ST.s16_Modbus_Keep_Param_Count    = 0;
   MB_ST.s16_Modbus_Sub_Function    = 0;
   MB_ST.s16_Modbus_Data    = 0;
   MB_ST.u16_Modbus_Over_Can = 0;

   MB_ST.ModbusStatus = MB_WAIT_COMMAND;
}

//**********************************************************
// Function Name: ModbusParser
// Description:
//       Modbus communication Parse
// Author:  Udi
// Algorithm: Check address and checksum
//            Change the state-machine to MB_DO_ONE_COMMAND, OR ignore and return to wait
// Revisions:
// 09 Apr 2013 First verion
//**********************************************************
void ModbusParser()
{
   int address, crc;
   int bytes2_3;
   int bytes4_5;
   int bytes6_7;
   int bytes8_9;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //If not enough chars - ignore this message (because can not check CRC, function code ID etc)
   if (MB_ST.s16_Recive_Counter < MIN_MODBUS_FRAME_SIZE )
   {
      resetStatus(FALSE, TRUE);
      MB_ST.u16_Modbus_Err_Cnt ++;
      MB_ST.u16_Modbus_No_Rsep_Cnt ++;
      return;
   }

   address                     = MB_ST.s16_Recive_Buff[0];
   MB_ST.s16_Modbus_Request    = MB_ST.s16_Recive_Buff[1];
   bytes2_3                    = bytes2Int16(MB_ST.s16_Recive_Buff[2],MB_ST.s16_Recive_Buff[3] );
   bytes4_5                    = bytes2Int16(MB_ST.s16_Recive_Buff[4],MB_ST.s16_Recive_Buff[5] );

   // Check the checksum (CRC)
   // Udi Feb 10, 2014 : No CRC in Modbus Over Can.
   if (!MB_ST.u16_Modbus_Over_Can)
   {
      crc = CRC16(MB_ST.s16_Recive_Buff, MB_ST.s16_Recive_Counter-2);
      if( (MB_ST.s16_Recive_Buff[(MB_ST.s16_Recive_Counter-1)]!= H8(crc)) ||
            (MB_ST.s16_Recive_Buff[(MB_ST.s16_Recive_Counter-2)]!= L8(crc))     )
      {
         resetStatus(FALSE, TRUE);
         MB_ST.u16_Modbus_Err_Cnt ++;
         MB_ST.u16_Modbus_No_Rsep_Cnt ++;
         return; //Ignore silently
      }
   }
   MB_ST.s32_Frame_Time_Capture = Cntr_1mS; // Set base for last good modbus frame. for Node guard and Response delay time

   // Also count  mesages to different address in this counter.
   // If not needed - move (~10) lines lower - after address is tested.
   MB_ST.u16_Modbus_Msg_Cnt ++;

   //If not my address and not broadcast (248, 0 ?) - ignore entire message
   // allow address 0 (Zero) as broadcast for write and read-write only.
   if (address != MB_ST.s16_Modbus_DriveAddr && address != 248  &&
         !((address == 0) &&
               (MB_ST.s16_Modbus_Request ==  MB_CMD_WRITE ||
                     MB_ST.s16_Modbus_Request ==  MB_CMD_READ_WRITE)))

   {
      resetStatus(FALSE, TRUE);
      MB_ST.u16_Modbus_No_Rsep_Cnt ++;
      return;//Ignore silently
   }

   MB_ST.s16_Modbus_Command_Addr = address;

   if (address == MB_ST.s16_Modbus_DriveAddr)
   {
      MB_ST.u16_Modbus_Slave_Cnt ++;
   }


   //Cheat to escape to Serial mode
   // New cheat (201)allways OK. Old cheat (200) ok ONLY in fw ver 01.14.x
   if(MB_ST.s16_Modbus_Request ==  MB_CMD_CHEAT_TO_SERIAL )
   {       //Force Serial mode
      resetStatus(TRUE, TRUE);
      MB_ST.s32_Frame_Time_Capture = 0;
      // Reset *Serial* comms processor state to the Prompt Handler state
      s16_Comms_Processor_State = PROMPT_HANDLER;
      BGVAR(s16_Serial_Type) = SERIAL_COMM_TYPE ;
   }

   switch (MB_ST.s16_Modbus_Request)
   {
      case MB_CMD_ENCAPSULATION:  //Identification
         MB_ST.s16_Modbus_Param_Idx = MB_ST.s16_Recive_Buff[4]; //Zero or first object to send
         MB_ST.s16_Modbus_First_Idx = MB_ST.s16_Modbus_Param_Idx;
         break;

      case MB_CMD_DATA_TRANSFER:  // UMAS - Upload /Download files
         MB_ST.s16_Umas_Request =  MB_ST.s16_Recive_Buff[4];
         if(MB_ST.s16_Umas_Request == UMAS_BEGIN_UPLOAD)//Begin Upload: set packet size
            MB_ST.s16_Umas_Packet_Size = bytes2Int16(MB_ST.s16_Recive_Buff[11],MB_ST.s16_Recive_Buff[12] );
         break;

      case MB_CMD_DIAGNOSTIC:
         // bytes 2&3 are sub function, bytes 4&5 are data
         MB_ST.s16_Modbus_Sub_Function =   bytes2_3;
         MB_ST.s16_Modbus_Data         =   bytes4_5;
         break;


      case MB_CMD_READ:
         //If Monitoring services - read of p10-27, or p10-29
         if(bytes2_3 == Monitor1_MB_ADDR || bytes2_3 == Monitor2_MB_ADDR )
         {
            MB_ST.s16_Modbus_Keep_Param = bytes2_3;
            SetNextMonitoringParam();
         }
         else
         {
            MB_ST.s16_Modbus_Param = bytes2_3;
         }
         MB_ST.s16_Modbus_Param_Count =bytes4_5;// GetParamCount(bytes4_5)  ;
         MB_ST.s16_Modbus_Param_Idx  = 0;

         break;
      case MB_CMD_WRITE:
         MB_ST.s16_Modbus_Param = bytes2_3;
         MB_ST.s16_Modbus_Param_Count =bytes4_5;// GetParamCount(bytes4_5)  ;
         MB_ST.s16_Modbus_Param_Idx  = 0;
         MB_ST.s16_Modbus_First_Write_Value = 7; //First byte of data to write
         break ;

      case MB_CMD_READ_WRITE:
         // For read-write command, we first do the write, than read. So keep read param and param count

         MB_ST.s16_Modbus_Keep_Param = bytes2_3;
         MB_ST.s16_Modbus_Keep_Param_Count =bytes4_5;// GetParamCount(bytes4_5);

         bytes6_7 = bytes2Int16(MB_ST.s16_Recive_Buff[6],MB_ST.s16_Recive_Buff[7] );
         bytes8_9 = bytes2Int16(MB_ST.s16_Recive_Buff[8],MB_ST.s16_Recive_Buff[9] );

         MB_ST.s16_Modbus_Param = bytes6_7;
         MB_ST.s16_Modbus_First_Write_Value = 11;  //First byte of data to write
         MB_ST.s16_Modbus_Param_Count = bytes8_9;// GetParamCount(bytes8_9)  ;
         MB_ST.s16_Modbus_Param_Idx  = 0;
         // Now pretend this was a write , so we do the write first
         MB_ST.s16_Modbus_Request =  MB_CMD_WRITE;
         MB_ST.s16_Modbus_Keep_Original_Request =  MB_CMD_READ_WRITE;
         break;

      default:
         break;
   }
   

   MB_ST.s16_Transmit_Buff[0] = address; //Drive address on return frame is addr from request, not necessarily actual drive address. (eMail from Juergen fiess Oct 15, 2013)
   //Special care for Ping command -
   if( (MB_ST.s16_Modbus_Request == MB_CMD_READ )&&
         ((MB_ST.s16_Modbus_Param == 0) || (MB_ST.s16_Modbus_Param == 1)))
   {   
      // Fix IPR#1384 No communication indicator available for Modbus:
      // toggle the last bit acording to the MODBUS traffic

      // indicate a new incoming message
      BGVAR(u16_ModBus_Ind) |= MODBUS_IND_MSG_INDICATION;
      
      // Ping is read from  parameter 1, length 1, should return value 0x008
      //      MB_ST.s16_Transmit_Buff[0] = address; //Drive address on return frame is addr from request, not necessarily actual drive address. (eMail from Juergen fiess Oct 15, 2013)
      // Udi Feb 12 2015: Wating decission if this is in Ping or in Login.
      //Force "Correct" byte order after ping
      s16_MB_Word_Order_In_Use  = 0;

      MB_ST.s16_Transmit_Buff[1] = MB_ST.s16_Modbus_Request;
      MB_ST.s16_Transmit_Buff[2] = 0x2;
      MB_ST.s16_Transmit_Buff[3] = 0x80;
      MB_ST.s16_Transmit_Buff[4] = 0x00;
      MB_ST.s16_Transmit_Counter = 5;
      ModbusCloseAnswer();
      return;
   }



   //Init answer also check if request is valid
   if(ModbusInitAnswer()> 0)
   {
      // Fix IPR#1384 No communication indicator available for Modbus:
      // toggle the last bit acording to the MODBUS traffic

      // indicate a new incoming message
      BGVAR(u16_ModBus_Ind) |= MODBUS_IND_MSG_INDICATION;
   
      MB_ST.ModbusStatus = MB_DO_ONE_COMMAND;
   }   
}




//**********************************************************
// Function Name: PParamToModbus
// Description:
//       Translate P-Param Modbus address
// Author:  Udi
// Algorithm:
//**********************************************************
int PParamToModbus(int p_param)
{
   int x,y,  modbus_addr ;

   x = (int)( (p_param) / 100 );
   y = (p_param - (100*x)  ) ;
   modbus_addr =(x*256 + y*2 + 256);

   return modbus_addr;
}




//**********************************************************
// Function Name: ModbusToPParam
// Description:
//       Translate Modbus Address to P-Param idx
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************
int ModbusToPParam(int s16_Modbus_Addr)
{
   int x,y,p_param;

   x = (int)( (s16_Modbus_Addr - 0x100) / 256 );
   y = ( s16_Modbus_Addr - 0x100 - 256*x ) / 2;
   p_param =(100*x + y);

   return p_param;
}


//**********************************************************
// Function Name: resetModbusCom
// Description:
//       reset Modbus Communication port
// Author:  Udi
// Algorithm:
//
// Revisions:
// 09 June 2013 First verion
//**********************************************************
int resetModbusCom()
{
   // Reset SCI with SW_REset
   // Defined by Spec, not sure if doing anything - because we can not reach here if there is an SCI error .
    ScicRegs.SCICTL1.bit.SWRESET = 0;
    ScicRegs.SCICTL1.bit.SWRESET = 1;

    // Per email from Juergan Aug 5 2015: Ignore data field, allways clear all
    resetModbusCounter(1); // Clear error counters

   //if (MB_ST.s16_Modbus_Data ==0x0000)
   //   resetModbusCounter(0); // Do not clear error counters
   //else if (MB_ST.s16_Modbus_Data ==0xFF00)
   //   resetModbusCounter(1); // Clear error counters
   //else
   //   return FALSE;

   return TRUE;
}



//**********************************************************
// Function Name: resetModbusCounter
// Description:
//       reset Modbus Communication Counters
// Author:  Udi
// Algorithm:
//
// Revisions:
// 09 June 2013 First verion
// 15 July 2015: Option to keep or clear "event log" (Error countnters) for Interoperability tests.
// Note - Only used with s16_Clear_Errors==1.
//**********************************************************
int resetModbusCounter(int s16_Clear_Errors)
{

   MB_ST.u16_Modbus_Msg_Cnt = 0;
   MB_ST.u16_Modbus_Slave_Cnt = 0;
   if (s16_Clear_Errors == 1) // 15 July 2015: Option to keep or clear "event log" (Error countnters)
   {
      MB_ST.u16_Modbus_Err_Cnt = 0;
      MB_ST.u16_Modbus_Exp_Cnt = 0;

      MB_ST.u16_Modbus_No_Rsep_Cnt = 0;

       //Udi Apr 24 2014 as requested by Gulam
      MB_ST.s16_Last_Error_Code = SAL_SUCCESS;
   }
   return TRUE;
}

//**********************************************************
// Function Name: ModbusDiag
// Description:
//       Do Diagnostic command - Communication command and counter report
// Author:  Udi
// Algorithm:
//
// Revisions:
// 09 June 2013 First verion
//**********************************************************
int ModbusDiag()
{
   int16 returnData = 0;

   // For Modbus Interoperability Test: Data field shuld be zero, with 2 exceptions:
   //  i. For sub-function 0 (Query data) it can be Any value.
   //  ii.For sub-function 01 (Restart) it can be 0000 or FF00.

    if (((MB_ST.s16_Modbus_Sub_Function == 1) && (MB_ST.s16_Modbus_Data != 0xFF00) && (MB_ST.s16_Modbus_Data != 0x0000)) ||
         ((MB_ST.s16_Modbus_Sub_Function >1 ) && (MB_ST.s16_Modbus_Data != 0x0000)))
   {
      SendException(MB_ILLEGAL_DATA_VALUE);
      return FALSE;
   }



   switch (MB_ST.s16_Modbus_Sub_Function)
   {
      case 0: // Echo data recived
         returnData = MB_ST.s16_Modbus_Data;
         break;
      case 1: // Reset Communincation by data field
         if (resetModbusCom() != 1)
         {
            SendException(MB_ILLEGAL_DATA_VALUE );
            return FALSE;
         }
         returnData = MB_ST.s16_Modbus_Data;
         break;
      case 2: // Send last error (Add 0x1000 to make all errors unique(SAL error, Faults and Warning )
         if(MB_ST.s16_Last_Error_Code != SAL_SUCCESS)
            returnData = 0x1000 + MB_ST.s16_Last_Error_Code;
         else
            returnData = 0;
         break;

      case 10:  //Reset counter
         resetModbusCounter(1);
         break;
      case 11: //return Bus Message Count
         returnData = MB_ST.u16_Modbus_Msg_Cnt;
         break;

      case 12: //return All communication Error Count
         returnData = MB_ST.u16_Modbus_Err_Cnt;
         break;

      case 13: //Returns all exception error messages
         returnData = MB_ST.u16_Modbus_Exp_Cnt;
         break;

      case 14: //return All  slave messages
         returnData = MB_ST.u16_Modbus_Slave_Cnt;
         break;

      case 15: //return All slave no resp. messages
         returnData = MB_ST.u16_Modbus_No_Rsep_Cnt;
         break;


      default:
         SendException(MB_ILLEGAL_FUNCTION);
         return FALSE;
   }

   MB_ST.s16_Transmit_Buff[4] = H8(returnData);
   MB_ST.s16_Transmit_Buff[5] = L8(returnData);
   MB_ST.s16_Transmit_Counter  = 6;

   ModbusCloseAnswer();
   return TRUE;
}




//**********************************************************
// Function Name: ModbusWrite
// Description:
//       Do Write command - Write value of one parameter
//       from recived buffer to the CDHD drive
// Author:  Udi
// Algorithm:
//
// Revisions:
// 14 Apr 2013 First verion
//**********************************************************
int ModbusWrite()
{
   int    Return_Value = 0;
   long   s32_Value;
   int    Data_Idx;
   int drive = 0;

   int p_param = ModbusToPParam(MB_ST.s16_Modbus_Param);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   Data_Idx = MB_ST.s16_Modbus_First_Write_Value +
         (MB_ST.s16_Modbus_Param_Idx *4);

   //Word Order
   if(s16_MB_Word_Order_In_Use == 0)
      s32_Value  = bytes2Int32(MB_ST.s16_Recive_Buff[Data_Idx]  ,MB_ST.s16_Recive_Buff[Data_Idx+1],
            MB_ST.s16_Recive_Buff[Data_Idx+2],MB_ST.s16_Recive_Buff[Data_Idx+3] );
   else // Word Order == 1, reverse words Order.
      s32_Value  = bytes2Int32(MB_ST.s16_Recive_Buff[Data_Idx+2]  ,MB_ST.s16_Recive_Buff[Data_Idx+3],
            MB_ST.s16_Recive_Buff[Data_Idx],MB_ST.s16_Recive_Buff[Data_Idx+1] );


   //   s32_Value      = bytes2Int(&(MB_ST.s16_Recive_Buff[Data_Idx]),4);


   Return_Value = ExecutePParam(0, p_param, OP_TYPE_WRITE, &s32_Value, LEX_CH_MODBUS_RS485);
   if(Return_Value == SAL_NOT_FINISHED)
      return SAL_NOT_FINISHED;


   if (Return_Value != SAL_SUCCESS)
   {   // Todo - Change if we need to check all values before first write.
      MB_ST.s16_Last_Error_Code = Return_Value;
      MB_ST.s16_Addr_Last_Error = MB_ST.s16_Modbus_Param;
      if(Return_Value == UNKNOWN_COMMAND )
         SendException(MB_ILLEGAL_DATA_ADDRESS);
      else if (Return_Value ==  NOT_PROGRAMMABLE)
         SendException(MB_ILLEGAL_DATA_VALUE);
      else
         SendException(MB_SLAVE_DEVICE_FAILURE);

      return FALSE;
   }
   else
   {
      ModbusNextParamOrClose();
   }
   return TRUE;

}


//**********************************************************
// Function Name: ModbusRead
// Description:
//       Do read command - read value of one parameter from the CDHD drive
// Author:  Udi
// Algorithm:
//
// Revisions:
// 29 Apr 2013 First verion
//**********************************************************
int ModbusRead()
{
   int Return_Value = 0;
   long   s32_Value = 0;
   int p_param;
   long Low_Word,High_Word;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_param = ModbusToPParam(MB_ST.s16_Modbus_Param);

   MB_ST.u32_Last_Answer  = 0xFF;            // Temp fail indication


   Return_Value = ExecutePParam(0, p_param,  OP_TYPE_READ, &s32_Value, LEX_CH_MODBUS_RS485);
   if(Return_Value == SAL_NOT_FINISHED)
      return SAL_NOT_FINISHED;

   if (Return_Value != SAL_SUCCESS)
   {
      MB_ST.s16_Last_Error_Code = Return_Value;
      MB_ST.s16_Addr_Last_Error = MB_ST.s16_Modbus_Param;

      if(Return_Value == UNKNOWN_COMMAND )
         SendException(MB_ILLEGAL_DATA_ADDRESS);
      else
         SendException(MB_ILLEGAL_DATA_VALUE);

      return FALSE;
   }

   //Word Order
   if(s16_MB_Word_Order_In_Use == 1)//Reverse words Order.
   {
      Low_Word =  (s32_Value << 16) & 0xFFFF0000L;
      High_Word = (s32_Value >> 16) & 0x0000FFFFL;
      s32_Value = Low_Word | High_Word ;
   }


   //Keep response as 32 bit

   MB_ST.u32_Last_Answer = (long) s32_Value;
   ModbusAddReadResponse();
   // MB_ST.ModbusStatus = MB_ADD_ONE_ANSWER;
   return TRUE;


}



//**********************************************************
// Function Name: UmasGetFileID
// Description:
//       Identify file to upload
// Author:  Udi
// Algorithm:
//
// Revisions:
//**********************************************************

int UmasGetFileID()
{
   int type, subType,ver, subVer;

   //Get Type info
   type    = MB_ST.s16_Recive_Buff[7];
   subType = MB_ST.s16_Recive_Buff[8];
   ver     = MB_ST.s16_Recive_Buff[9];
   subVer  = MB_ST.s16_Recive_Buff[10];

   ver += 0; //Omit compiler warning.
   MB_ST.s16_Umas_File_ID = 0 ; //Unknown
   switch (type)
   {
      case 100:
         switch (subType)
         {

            case 1:   return FileID_Parameters1;     // User Parameters1 P0-00 to P5-99
            case 2:   return FileID_Parameters2;     // User Parameters1 p8-00 to P10-99
            case 6:   return FileID_Motor ;         // Motor Parameters
            case 20:  return FileID_Motion1;        // Motion sequence  0-16
            case 21:  return  FileID_Motion2;       // Motion sequence  17-32
            case 100: return FileID_Scope;          // Recirded data

            default : return  0 ; //Unknown File
         }

         //no break

            case 255:
               switch (subVer) //!for some reason, 255 are sub-ver, not sub-type !
               {
                  case 255: return FileID_Compatibility; //Device Compatibility
                  case 254 :return FileID_Files_List; //Configuration Files List
                  default : return  0 ; //Unknown File
               }

            //no break

                  default : return  0 ; //Unknown File
   }

}


//**********************************************************
// Function Name: UmasNegative
// Description:
//       return Modbus nessage with negative UMAS response code:
//       currently only  fail code avilable.0x9191
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************

int UmasNegative(int ums_err)
{
   UmasInitAnswer(FALSE);
   MB_ST.s16_Transmit_Buff[5] = H8(ums_err);
   MB_ST.s16_Transmit_Buff[6] = L8(ums_err);
   MB_ST.s16_Transmit_Counter = 7;
   ModbusCloseAnswer();
   return SAL_SUCCESS;
}



//**********************************************************
// Function Name: UmasGetFileSize
// Description:
//       Return file size , per file type
// Author:  Udi
// Algorithm:
//
// Revisions:
//**********************************************************
int UmasGetFileSize()
{

   switch (MB_ST.s16_Umas_File_ID)
   {

      case FileID_Compatibility:
         return Compatibility_File_Size; //Size of competability file is fixed.
      case FileID_Files_List:
         return Files_List_File_Size;

      case FileID_Parameters1:
      case FileID_Parameters2:
         return Param_File_Header_Size + (GetParamFileCount(MB_ST.s16_Umas_File_ID) * Param_Size_In_File);

      case FileID_Motor:
         return  Motor_File_Size;

      case FileID_Motion1: return Path_File_Header_Size + (Path_Line_Size * Paths_In_Group1);
      case FileID_Motion2: return Path_File_Header_Size + (Path_Line_Size * Paths_In_Group2);

      case FileID_Scope:
         //   return MB_ST.s16_HeaderSize + (s16_Recorded_Variables_Number * s16_Recording_Length * 4 );
         return Umas_Scope_Header_Size + (s16_Recorded_Variables_Number * s16_Recording_Length * 4 );

      default:
         return 0;

   }
}



//**********************************************************
// Function Name: BeginUpDnLoad
// Description:
//       Initiate seesion acording to requested file type
// Author:  Udi
// Algorithm:
//
// Revisions:
//**********************************************************


int BeginUpDnLoad(int Upload)
{
   long fileSize;
   int drive = 0;

   MB_ST.s16_Umas_Packet_Idx = 0;
   MB_ST.s16_Umas_Line_Idx_In_File = 0;
   s16_UMAS_sessionID ++;
   //Get file ID
   MB_ST.s16_Umas_File_ID = UmasGetFileID();
   if(MB_ST.s16_Umas_File_ID == 0)
      return UmasNegative(UMAS_ERR_ACCESS_VIOLATION);



   //Set bytes 0-4
   UmasInitAnswer(TRUE);

   //Set session ID
   Append_2_BytesTxBuff(s16_UMAS_sessionID);
   if(Upload) //Get file size by File ID
   {
      //Check specific for user parameters
      // 5 Jan 2014: Config required for read
      // 2 Feb 2014: Config required for User parameter and Motion path ONLY.

      if(((MB_ST.s16_Umas_File_ID == FileID_Parameters1) || (MB_ST.s16_Umas_File_ID == FileID_Parameters2) ||
            (MB_ST.s16_Umas_File_ID == FileID_Motion1 )   ||
            (MB_ST.s16_Umas_File_ID == FileID_Motion2 ))&&
            (u16_Config_Lock != CONFIG_LOCK_READ))
         return UmasNegative(UMAS_ERR_ACCESS_VIOLATION);


      fileSize = UmasGetFileSize();
      Append_4_BytesTxBuff (fileSize);
      MB_ST.s16_Umas_State = UMAS_STATE_UPLOAD;

   }
   else //Download
   {
      // 2 Feb 2014: Config required for i. User parameter and Motion path ONLY.

      if(((MB_ST.s16_Umas_File_ID == FileID_Parameters1) || (MB_ST.s16_Umas_File_ID == FileID_Parameters1) ||
            (MB_ST.s16_Umas_File_ID == FileID_Motion1 )   ||
            (MB_ST.s16_Umas_File_ID == FileID_Motion2 ))&&
            (u16_Config_Lock != CONFIG_LOCK_WRITE))
         return UmasNegative(UMAS_ERR_ACCESS_VIOLATION);

      if(Enabled(drive))
         return UmasNegative(UMAS_ERR_ACCESS_VIOLATION);

      MB_ST.s16_Umas_State = UMAS_STATE_DOWNLOAD;

      //Reset last error info
      MB_ST.s16_Last_Error_Code = SAL_SUCCESS ;
      MB_ST.s16_Addr_Last_Error = 0;
      MB_ST.s16_partial_line_count = 0;
   }


   ModbusCloseAnswer();

   return SAL_SUCCESS;


}


//**********************************************************
// Function Name: EndUpDnLoad
// Description:
//       Terminate upload packet session
// Author:  Udi
// Algorithm:
//
// Revisions:
//**********************************************************
int EndUpDnLoad(int Upload)
{

   //General upload and download protocol

   if(Upload && MB_ST.s16_Umas_State != UMAS_STATE_UPLOAD)
      return UmasNegative(UMAS_ERR_BAD_SEQ);

   if(!Upload && MB_ST.s16_Umas_State != UMAS_STATE_DOWNLOAD)
      return UmasNegative(UMAS_ERR_BAD_SEQ);


   UmasInitAnswer(TRUE);
   ModbusCloseAnswer();
   MB_ST.s16_Umas_Packet_Idx = 0;

   MB_ST.s16_Umas_Line_Idx_In_File = 0;
   MB_ST.s16_Umas_State = UMAS_STATE_READY;

   return SAL_SUCCESS;
}


// **********************************************************
// Function Name: SaveParamFromBuffer
// Description:
//   Save (Execute P Param) One Parameter recived via UMAS file
//   Param stracure: int16 MB_ADDR; Int16 Type; Int32 Value
// Author:  Udi
// Algorithm:
// **********************************************************
int SaveParamFromBuffer(int16 *data_buffer)
{
   int  Return_Value = 1;
   int  Modbus_Param;
   long s32_Value;
   int  P_Param ;

   Modbus_Param = bytes2Int16(data_buffer[0], data_buffer[1]);
   P_Param = ModbusToPParam(Modbus_Param);
   if((P_Param < 0) || P_Param > 2000 ) //Safty - should never happend
   {
      u16_Modbus_Config_Transfer_Flt = 1;
      MB_ST.s16_Last_Error_Code = Return_Value;
      MB_ST.s16_Addr_Last_Error = -1;
      return  Return_Value;
   }

   s32_Value = bytes2Int32(data_buffer[4], data_buffer[5], data_buffer[6], data_buffer[7]);

   if((u16_Config_Lock == CONFIG_LOCK_WRITE) &&
      ((P_Param >= 210) && (P_Param <= 217)))
   {   //Special case for P2-10 to P2-17 (Inmode) : remove func if need to.
      Return_Value = SetInmodeFromConfigFile(0, P_Param, s32_Value);
   }
   else
   {// not INPUT assign
   Return_Value = ExecutePParam(0, P_Param, OP_TYPE_WRITE, &s32_Value, LEX_CH_MODBUS_RS485);
   }

   if(Return_Value == SAL_NOT_FINISHED)
      return SAL_NOT_FINISHED; //This parameter not finished yet - to be continued in next BG loop

   if (Return_Value != SAL_SUCCESS)
   {   // Save first error only
      if (MB_ST.s16_Last_Error_Code == SAL_SUCCESS)
      {
         u16_Modbus_Config_Transfer_Flt = 1;
         MB_ST.s16_Last_Error_Code = Return_Value;
         MB_ST.s16_Addr_Last_Error = P_Param; //MB_ST.s16_Modbus_Param;
      }
   }

   return  Return_Value;
}



// **********************************************************
// Function Name: DownLoadPathLineFromBuff
// Description:
//   Process one motion path (with 4 parameters) from temp buffer
// Author:  Udi
// Algorithm:
// **********************************************************
int DownLoadPathLineFromBuff(int data_buff[])
{
   int idx;
   int Return_Value;
   for(idx = 0; idx < Params_In_Path; idx ++)
   {
      Return_Value = SaveParamFromBuffer(&data_buff[2 + (idx * Param_Size_In_File)]);
   }
   return Return_Value;
}





//**********************************************************
// Function Name: HandlePartialLines()
// Description:
//       Each packet contains line, but a line may be broken between 2 packets.
//       This function keep partial line from end of packet,
//       and append it to the partial line at the begining on folowing packet.
//       Line is 8 bytes: "ParamID-Type-Value"
//       Note: this version is fo user param only !!
// Author:  Udi
// Algorithm:
//
//*********************************************************
int HandlePartialLines()
{
   int s16_line_size;
   int s16_moved_byte = 0;
   int s16_bytes_to_keep;
   int s16_first_byte_to_move;
   int s16_data_header_size = 0;
   int s16_return_value;
   int s16_loop_limit;

   // Set line and header size for file type
   MB_ST.s16_HeaderSize = DownLoad_Packet_Header_Size;
   if((MB_ST.s16_Umas_File_ID == FileID_Parameters1) || (MB_ST.s16_Umas_File_ID == FileID_Parameters2))
   {
      s16_line_size = Param_Size_In_File;
      if(MB_ST.s16_Umas_Packet_Idx == 1)
      {
         MB_ST.s16_HeaderSize +=  Param_File_Header_Size ;
         s16_data_header_size = Param_File_Header_Size ;
      }
   }
   else if((MB_ST.s16_Umas_File_ID == FileID_Motion1) || (MB_ST.s16_Umas_File_ID == FileID_Motion2))
   {
      s16_line_size = Path_Line_Size;
      if(MB_ST.s16_Umas_Packet_Idx == 1)
      {
         MB_ST.s16_HeaderSize +=  Path_File_Header_Size ;
         s16_data_header_size = Path_File_Header_Size ;
      }
   }

   MB_ST.s16_Recived_Data_Offset = MB_ST.s16_HeaderSize;



   //If Partial line exist from end of previous packet, add data drom  from beginning this packet  and process it.
   if(MB_ST.s16_partial_line_count > 0)
   {
      for(;MB_ST.s16_partial_line_count < s16_line_size; MB_ST.s16_partial_line_count++ )
      {
         MB_ST.partial_line_data[MB_ST.s16_partial_line_count] = MB_ST.s16_Recive_Buff[MB_ST.s16_Recived_Data_Offset++];
         s16_moved_byte ++;
      }
      //Process partial line - by file type
      if((MB_ST.s16_Umas_File_ID == FileID_Parameters1) || (MB_ST.s16_Umas_File_ID == FileID_Parameters2))
      {
         s16_return_value = SaveParamFromBuffer(MB_ST.partial_line_data);
         s16_loop_limit = 5;
         while (s16_return_value == SAL_NOT_FINISHED)
         {
            s16_return_value = SaveParamFromBuffer(MB_ST.partial_line_data);
            //Safe guard to insure out of loop in less then 5 times.
            if(s16_loop_limit-- <= 0)
            {
               u16_Modbus_Config_Transfer_Flt = 1;
               MB_ST.s16_Last_Error_Code = -1;
               MB_ST.s16_Addr_Last_Error = -1;
               return  -1;
            }

         }
      }
      else if((MB_ST.s16_Umas_File_ID == FileID_Motion1) || (MB_ST.s16_Umas_File_ID == FileID_Motion2))
         DownLoadPathLineFromBuff(MB_ST.partial_line_data);

      //clean
      MB_ST.s16_partial_line_count = 0;
   }

   //Check if partial line exists at end of file and if so : keep it for next packet.
   s16_bytes_to_keep = (MB_ST.s16_Umas_Packet_Size - s16_data_header_size -s16_moved_byte ) % s16_line_size;
   if(s16_bytes_to_keep > 0)
   {

      s16_first_byte_to_move = MB_ST.s16_Recive_Counter - 2 - s16_bytes_to_keep; //Last 2 bytes are checksum

      for(s16_moved_byte = 0; s16_moved_byte < s16_bytes_to_keep; s16_moved_byte++ )
         MB_ST.partial_line_data[s16_moved_byte] = MB_ST.s16_Recive_Buff[s16_first_byte_to_move + s16_moved_byte];

      MB_ST.s16_partial_line_count = s16_bytes_to_keep;
   }
   return SAL_SUCCESS;
}



//**********************************************************
// Function Name: DownLoadPacket()
// Description:
//       Process one packet of any  file  via UMAS
// Author:  Udi
// Algorithm:
//
//*********************************************************
int DownLoadPacket()
{
   static int Download_State = 0;
   int requestedPacketIdx = -1;
   int status;

   if(MB_ST.s16_Umas_State != UMAS_STATE_DOWNLOAD)
      return UmasNegative(UMAS_ERR_BAD_SEQ);


   // static int temp_idx = 0;
   switch (Download_State)
   {
      case 0: // Check and Init reciving of packet

         requestedPacketIdx = bytes2Int16(MB_ST.s16_Recive_Buff[7],MB_ST.s16_Recive_Buff[8]);
         MB_ST.s16_Umas_Packet_Size = bytes2Int16(MB_ST.s16_Recive_Buff[9],MB_ST.s16_Recive_Buff[10]);

         if(requestedPacketIdx != MB_ST.s16_Umas_Packet_Idx + 1 )
         {
            return UmasNegative(UMAS_ERR_BAD_SEQ);
            // UmasInitAnswer(FALSE);
            // ModbusCloseAnswer();
            // return SAL_SUCCESS;
         }
         MB_ST.s16_Umas_Packet_Idx++;

         // For Compatibility File, download is check, need to be done once for all lines.
         if(MB_ST.s16_Umas_File_ID ==  FileID_Compatibility)
            return DownloadCompatibilityFile();

         status = HandlePartialLines();
         if(status !=  SAL_SUCCESS)
            return UmasNegative(UMAS_ERR_BAD_SEQ);
         Download_State ++;
         break;
      case 1: //Process next line in packet;
         status = DownLoadFileLine();
         //Check if done;
         if(status ==  SAL_SUCCESS)
            Download_State ++;
         break;
      case 2: // Terminate
         UmasInitAnswer(TRUE);
         Append_1_BytesTxBuff(0);//<Wait> byte in the spec for positive response
         ModbusCloseAnswer();
         Download_State = 0;
         return SAL_SUCCESS;

   }

   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: UploadPacket()
// Description:
//       Prepare and send next paket of any  file  via UMAS
// Author:  Udi
// Algorithm:
//
//**********************************************************
int UploadPacket()
{
   static int Upload_State = 0;
   static int Header_Size = 9;
   int Data_Size;
   int Return_Value;
   int requestedPacketIdx;

   if(MB_ST.s16_Umas_State != UMAS_STATE_UPLOAD)
      return UmasNegative(UMAS_ERR_BAD_SEQ);


   switch (Upload_State)
   {
      case 0: // Init packet
         requestedPacketIdx =         bytes2Int16(MB_ST.s16_Recive_Buff[7],MB_ST.s16_Recive_Buff[8]);


         if(requestedPacketIdx != MB_ST.s16_Umas_Packet_Idx + 1 )
         {
            UmasInitAnswer(FALSE);
            ModbusCloseAnswer();
            return SAL_SUCCESS;
         }
         MB_ST.s16_Umas_Packet_Idx++;
         UmasInitAnswer(TRUE);
         // Update the  "end of file" (bytes 5-6) and "Data Length" (7-8) fields in the packet
         Append_2_BytesTxBuff(0);     //"end of file" -> different from 0 is End of file , this may be overideen after data is written
         Append_2_BytesTxBuff(1);     //"Data Length" (7-8), will be overideen after data is written

         if(MB_ST.s16_Umas_Packet_Idx == 1 )
         { //If first packet of this file - add file header
            Header_Size = MB_ST.s16_Transmit_Counter;
            UploadFileHeader();
         }
         Upload_State++;


         break;

      case 1: // Append one line of data until packet is full
         Return_Value = UploadFileLine();
         if (Return_Value ==  SAL_SUCCESS)
            Upload_State++;
         return Return_Value;

      case 2: // Close UMAS frame and transmit
         if(MB_ST.s16_Umas_Packet_Idx == -1 )  // If Last data packet
            MB_ST.s16_Transmit_Buff[6] = L8(-1);// Mark as last packet

         Data_Size = MB_ST.s16_Transmit_Counter - Header_Size;
         MB_ST.s16_Transmit_Buff[7]= H8(Data_Size); // Data length
         MB_ST.s16_Transmit_Buff[8]= L8(Data_Size);

         ModbusCloseAnswer();
         Upload_State = 0;
         return SAL_SUCCESS;
   }

   return SAL_SUCCESS;
}








//**********************************************************
// Function Name: ModbusDataTransfer
// Description:
//       Prepare recorded data to be send via modbus UMAS Upload request
// Author:  Udi
// Algorithm:
//
// Revisions:
// 17 June 2013 First verion
//**********************************************************
int ModbusDataTransfer()
{
   switch (MB_ST.s16_Umas_Request )
   {
      case UMAS_BEGIN_UPLOAD: return BeginUpDnLoad(TRUE);
      case UMAS_UPLOAD_PACKET:return UploadPacket();
      case UMAS_END_UPLOAD:   return EndUpDnLoad(TRUE);


      case UMAS_BEGIN_DOWNLOAD:return BeginUpDnLoad(FALSE);
      case UMAS_DOWNLOAD_PACKET:return DownLoadPacket();
      case UMAS_END_DOWNLOAD:  return EndUpDnLoad(FALSE);

      default :
         return UmasNegative(UMAS_ERR_BAD_SEQ);
         // UmasInitAnswer(FALSE);
         //ModbusCloseAnswer();
         //return FALSE;
   }

}



//**********************************************************
// Function Name: GetFwDate
// Description:
//       Place holder - in case internal string is not what they want to see.
//       Return string with version date as defined in Schneider spec.
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************
char* GetFwDate(void)
{
   return p_s8_Version_Date;
}


//**********************************************************
// Function Name: GetFwVer
// Description:
//       Return string with version number as defined in Schneider spec.
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************
char* GetFwVer(void)
{
   return p_s8_Shndr_Software_Version;
}


//**********************************************************
// Function Name: GetAppName
// Description:
//       Return string with Appliation Name as defined in Schneider spec.
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************
char* GetAppName(void)
{
   int drive = 0;

   static char appName[17];
   int i;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for(i=0;i<4;i++)
   {
      appName[i*4]     = (BGVAR(u32_User_App_Name)[i] >> 24 )  & 0x00FF;
      appName[i*4 + 1] = (BGVAR(u32_User_App_Name)[i] >> 16 )  & 0x00FF;
      appName[i*4 + 2] = (BGVAR(u32_User_App_Name)[i] >> 8 )  & 0x00FF;
      appName[i*4 + 3] =  BGVAR(u32_User_App_Name)[i] & 0x00FF;

   }
   return appName;
}

//**********************************************************
// Function Name: GetModuleConfig
// Description:
//       Return string with Module Config as defined in Schneider spec.
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************
char* GetModuleConfig(void)
{

   return s8_Module_Config;
}


//**********************************************************
// Function Name: GetMotorName
// Description:
//       Return string with Motor Name as defined in Schneider spec.
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************
char* GetMotorName(void)
{
   return ((char *)(BGVAR(u8_MotorName)));
}


//**********************************************************
// Function Name: GetMotorNumber
// Description:
//       Return string with Motor (Serial?) Number as defined in Schneider spec.
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************
char* GetMotorNumber(void)
{
   UnsignedIntegerToAscii(BGVAR(u32_MotorFileName)); //result in u8_Response
   return &u8_Response[0];
}


//**********************************************************
// Function Name: GetSerialNum
// Description:
//       Return string with Drive Serial number as defined in Schneider spec.
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************
char* GetSerialNum(void)
{

   int i;
   for(i=0; i < 12; i++)
   {
      s8_LXM_Serial_Number[i+4] = s8_Product_Serial_Number[i];
   }

   return s8_LXM_Serial_Number;

}



//**********************************************************
// Function Name: GetProductCode
// Description:
//       Return string with Product code field as defined in Schneider spec.
// Author:  Udi
// Algorithm: See SetProductCode below.
//**********************************************************
char* GetProductCode(void)
{

   return s8_Product_Code;
}


//**********************************************************
// Function Name: GetProductCode
// Description:
//       Return string with Product Name
// Author:  Udi
// Algorithm: "Try and Error" to get Multiloader to work.
//**********************************************************
char* GetProductName(void)
{
   if (IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK))
      return "Lexium 28       ";
   else
      return "Lexium 26       ";
}


//**********************************************************
// Function Name: SetProductCode
// Description:
//       Set "Product_Code"  as defined in Schneider spec.
// Author:  Udi
// Algorithm:
//  Bai-Yang - With CANOpen hardware : Code:  LXM28AUppM3X
//  QuingSong - without CANOpen hardware : Code: LXM26DUppM3X
//  where pp indicate Power in Watt.
//start with default :
//  char s8_Product_Code[] I (= "LXM28AU01M3X    ");
//
//  Samples from SE Spec:
//   50 Watt -> LXM28AUA5M3X
//  750 Watt -> LXM28AU07M3X
// 1500 Watt -> LXM26DU15M3X (No CAN)
//**********************************************************
void SetProductCode(void)
{
   int drive = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //Change LXM28 (with CAN)  to be LXM26D(No Can), and  update Drive Interface <D/A/E>byte 6 (idx starting at 1)
   if (u16_Flash_Type == 1)
   {
      s8_Product_Code[3] = '2';
      s8_Product_Code[4] = '8';
      s8_Product_Code[5] = 'E';

      p_s8_Shndr_Software_Version[5] = '3';
   }
   else if (IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK))
   {
      s8_Product_Code[3] = '2';
      s8_Product_Code[4] = '8';
      s8_Product_Code[5] = 'A';
   }
   else
   {

      s8_Product_Code[3] = '2';//'E';
      s8_Product_Code[4] = '6';//'3';
      s8_Product_Code[5] = 'D';//'D';

      p_s8_Shndr_Software_Version[5] = '2';
   }

   //update Drive_Power_Watt.
   UnsignedIntegerToAscii(BGVAR(u16_Drive_Power_Watt)); //result in u8_Response

   if(BGVAR(u16_Drive_Power_Watt) < 100)
   {
      s8_Product_Code[7] = 'A';
      s8_Product_Code[8] = u8_Response[0];
   }
   else if(BGVAR(u16_Drive_Power_Watt) < 1000)
   {
      s8_Product_Code[7] = '0';
      s8_Product_Code[8] = u8_Response[0];
   }
   else
   {
      s8_Product_Code[7] = u8_Response[0];
      s8_Product_Code[8] = u8_Response[1];
   }

   return ;
}


//**********************************************************
// Function Name: ModbusIdentify
// Description:
//       Prepare drive identification data to be send via modbus
// Author:  Udi
// Algorithm:
//
// Revisions:
// 17 June 2013 First verion
// 24 Dec 2014 To Pass interoperability tests: Support readcode = 4 (Read single object)
//**********************************************************
int ModbusIdentify()
{
   int s16_read_code = 0;

   //Check sub function - only Identify is implemented.
   if(MB_ST.s16_Recive_Buff[2] != MB_ID_MEI_14 )
   {
      SendException(MB_ILLEGAL_DATA_VALUE);
      return -1;
   }


    //Check read code
   s16_read_code = MB_ST.s16_Recive_Buff[3];
   if((MB_ST.s16_Recive_Counter < 4) ||
         (s16_read_code < MB_ID_BASIC || s16_read_code > MB_ID_SINGLE))
   {
      SendException(MB_ILLEGAL_DATA_VALUE);
      return -1;
   }

   //Check first object ID exists (For interoperability tests)
   if(MB_ST.s16_Modbus_First_Idx > MB_IDX_Moduleconfig)
   {
   // Udi 22 Nov 2015: Change to return avilable info, if index is too high. 
   //                  Per IPR 462, report IPR462_CriticallityB_20151021.docx
    
       MB_ST.s16_Modbus_First_Idx = 0;
       MB_ST.s16_Modbus_Param_Idx = 0;
      //SendException(MB_ILLEGAL_DATA_ADDRESS);
      //return -1;
   }


   // Check if enough data for specific "read Code"
   if (((s16_read_code == MB_ID_BASIC)    && (MB_ST.s16_Modbus_Param_Idx >= 3)) ||
       ((s16_read_code == MB_ID_REGULAR)  && (MB_ST.s16_Modbus_Param_Idx >= 8)) ||
       ((s16_read_code == MB_ID_EXTENDED) && (MB_ST.s16_Modbus_Param_Idx >= 13)) )
   {
      ModbusCloseAnswer();
      return TRUE;
   }


   // Add requested data to the transmit buffer
   switch (MB_ST.s16_Modbus_Param_Idx)
   {
      //Add basic info
      case MB_IDX_VendorName:    AddText (ID_VendorName, "Schneider Electric  ", 20); break;
      case MB_IDX_ProductCode:   AddText (ID_ProductCode, GetProductCode(), 16); break;
      case MB_IDX_MajorMinorRev: AddText (ID_MajorMinorRev, GetFwVer(), 14); break;

      //Add Regular info
      case MB_IDX_ProductName:   AddText (ID_ProductName, GetProductName(), 16);break;
      case MB_IDX_ModelName:     AddText (ID_ModelName, GetProductCode(), 16);break;
      case MB_IDX_AppName:       AddText (ID_AppName , GetAppName(), 16);break;
      case MB_IDX_TRI_Class:     AddText (ID_TRI_Class, "A01-SLAVE-PL1(O)-B20-EHT100", 27);break;
      case MB_IDX_TRComServices: ModbusAddBinaryResponse(ID_TRComServices, 0xB, 2);break; //Send binary 0xB, as requested in the spec

      //Add Extended info
      case MB_IDX_SWDate:        AddText(ID_SWDate ,     GetFwDate(),    8);break;
      case MB_IDX_SerialNum:     AddText(ID_SerialNum ,  GetSerialNum(),    16);break;
      case MB_IDX_MotorNumber:   AddText(ID_MotorNumber ,GetMotorNumber(),   8);break;
      case MB_IDX_MotorName:     AddText(ID_MotorName ,  GetMotorName(),    20);break;
      case MB_IDX_Moduleconfig:  AddText(ID_Moduleconfig,GetModuleConfig(), 18);break;

   }


   MB_ST.s16_Transmit_Buff[7] = MB_ST.s16_Modbus_Param_Idx + 1 - MB_ST.s16_Modbus_First_Idx; // Number of objects included: 3,8, or 13 (idx is zero based)



   if(s16_read_code == MB_ID_SINGLE)     //To Pass interoperability tests check for readcode = 4
   {
      MB_ST.s16_Transmit_Buff[7] =  1; // Number of objects included: Only one
      ModbusCloseAnswer();
      return TRUE;
   }

   MB_ST.s16_Modbus_Param_Idx ++;

   return TRUE;
}


//**********************************************************
// Function Name: GetParamCount
// Description:
//       Return requested count div by 2, as each modbus ID return 2 P Param
// Author:  Udi
// Algorithm:
// Revisions:
// 17 June 2013 First verion
//**********************************************************
// int GetParamCount( int origValue)
// {
//    if(origValue > 1 && ((origValue & 0x1) == 0))
//        return  (origValue / 2);
//    else
//        return -1; // will be reported as error.
// }


//**********************************************************
// Function Name: SetNextMonitoringParam
// Description:
// Author:  Udi
// Algorithm: Set Next param from hard coded list as current param
//**********************************************************

void SetNextMonitoringParam()
{
   if(MB_ST.s16_Modbus_Keep_Param == Monitor1_MB_ADDR )
      MB_ST.s16_Modbus_Param = PParamToModbus (PParam_In_MonAccess[MB_ST.s16_Modbus_Param_Idx]);
   else if (MB_ST.s16_Modbus_Keep_Param == Monitor2_MB_ADDR )
      MB_ST.s16_Modbus_Param = PParamToModbus (PParam_In_Dat1Access[MB_ST.s16_Modbus_Param_Idx]);

}


//**********************************************************
// Function Name: ModbusNextParamOrClose
// Description:
// Author:  Udi
// Algorithm:
//  If additional  parameters need processing then advance current parm Idx.
//  else, if this is a write part of read-write command - switch to be read command
//  else - work is completed, close the response.
// Revisions:
// 09 Apr 2013 First verion
//**********************************************************

void ModbusNextParamOrClose()
{
   //If Read of multiply parameters can be "Monitoring" or successive modbus address.
   if((++(MB_ST.s16_Modbus_Param_Idx))< MB_ST.s16_Modbus_Param_Count)
   {
      //If this read is in monitoring services - take next Modbus adress from hardcoded list
      //Else - advance next param.
      if(MB_ST.s16_Modbus_Keep_Param == Monitor1_MB_ADDR ||
            MB_ST.s16_Modbus_Keep_Param == Monitor2_MB_ADDR )
         SetNextMonitoringParam();
      else
         MB_ST.s16_Modbus_Param += 2;// In standard

      MB_ST.ModbusStatus = MB_DO_ONE_COMMAND; //Check if this is needed, or is this the value already ?
   }
   // If write part of read-write was completed: switch to the read part.
   else if ((MB_ST.s16_Modbus_Keep_Original_Request == MB_CMD_READ_WRITE) &&
         (MB_ST.s16_Modbus_Request == MB_CMD_WRITE))
   {
      MB_ST.s16_Modbus_Request = MB_CMD_READ;
      MB_ST.s16_Modbus_Param  = MB_ST.s16_Modbus_Keep_Param;
      MB_ST.s16_Modbus_Param_Count  = MB_ST.s16_Modbus_Keep_Param_Count ;
      MB_ST.s16_Modbus_Param_Idx  = 0 ;

      //Response for ReadWrite is like response to read:
      MB_ST.s16_Transmit_Buff[2] = 0;                          //Actual data length - will be overiden
      MB_ST.s16_Transmit_Counter = 3;                          //Number bytes ready in the response

      MB_ST.ModbusStatus = MB_DO_ONE_COMMAND; //Check if this is needed, or is this the value already ?
   }

   else

   { // All done - close this response.
      ModbusCloseAnswer();
   }
}


//**********************************************************
// Function Name: ModbusAddReadResponse
// Description:
//       Add response from "LastAnswer" into the transmit buffer.
// Author:  Udi
// Algorithm:
//            Bytes 0 and 1 - Address and  Request ID are ready.
//            add value
// Revisions:
// 09 Apr 2013 First verion
//**********************************************************

void ModbusAddReadResponse()
{

   // P-Param info is 32 bit - 4 byte answer
   Append_4_BytesTxBuff (MB_ST.u32_Last_Answer);
   MB_ST.s16_Transmit_Buff[2] += 4; //num of bytes in actual data

   //Check If more parameters need processing
   ModbusNextParamOrClose();
}


//**********************************************************
// Function Name: AddText
// Description:
//       Add text response to the transmit buffer.
//       As part of the Identification message.
// Author:  Udi
// Algorithm:
//            Add obj ID, data length, and data to the transmit buffer,
//            keep Transmit_Counter up-to-date.
//            Checksum will be added later
//               Input String must be null terminated
// Revisions:
// 09 Apr 2013 First verion
//**********************************************************

void AddText(int objID, char textResponse[], int MaxLength )
{
   int i, strLen;

   strLen = strlen(textResponse);
   if(strLen > MaxLength)
      strLen= MaxLength;

   MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = objID;
   MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = MaxLength;

   //copy data
   for(i = 0; i < strLen; i++)
   {
      MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = textResponse[i];
   }

   //Fill with Spaces.
   for(; i < MaxLength; i++)
   {
      MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = ' ';
   }

}



//**********************************************************
// Function Name: ModbusAddBinaryResponse
// Description:
//       Add 'count' bytes from given value into the transmit buffer.
//       As part of the Identification message
// Author:  Udi
// Algorithm:
//  Count can be 1-2, if count ==1 only the low byte is send.
// Revisions:
// 20 Jun 2013 First verion
//**********************************************************
void ModbusAddBinaryResponse(int objID, int value, int count)
{
   if (count > 2)
      count = 2;

   MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = objID;
   MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = count;

   if(count == 2)
      MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = H8(value);

   MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Counter++] = L8(value);

}



//**********************************************************
// Function Name:
// Description:
// Author:  Udi
// Algorithm:
// Revisions:
//**********************************************************
void UmasAny()
{

}


//**********************************************************
// Function Name: UmasInitAnswer
// Description:
//       Prepare Positive or Negative UMAS Modbus Reply on the transmit buffer
//       set bytes 0-4.
// Author:  Udi
// Algorithm:

// Revisions:
// 23 June 2013 First version
//**********************************************************
void UmasInitAnswer(int positive)
{
   //   MB_ST.s16_Transmit_Buff[0] = MB_ST.s16_Modbus_DriveAddr; Address is set by address on the command
   MB_ST.s16_Transmit_Buff[1] = MB_CMD_DATA_TRANSFER;
   MB_ST.s16_Transmit_Buff[2] = 0; // reservation number, High byte
   MB_ST.s16_Transmit_Buff[3] = 0; // reservation number, Low byte
   if(positive)
      MB_ST.s16_Transmit_Buff[4] = 0xFE; //OK
   else
      MB_ST.s16_Transmit_Buff[4] = 0xFD; //Negative



   MB_ST.s16_Transmit_Counter = 5;//leave empy space for end of file and data length
}


//**********************************************************
// Function Name: SendException
// Description:
//       Prepare Negative Modbus Reply on the transmit buffer
// Author:  Udi
// Algorithm:
//            Indicate as error (Request ID + 0x80 ?)
//            add Exception code  and close to send.
//**********************************************************
void SendException(int ExeptionCode)

{
   MB_ST.s16_Transmit_Buff[1] += 0x80;
   MB_ST.s16_Transmit_Buff[2] = ExeptionCode;
   MB_ST.s16_Transmit_Counter = 3;

   MB_ST.u16_Modbus_Exp_Cnt ++;

   ModbusCloseAnswer();

}




//**********************************************************
// Function Name: ValidRequest
// Description:
//       Validate input data
// Author:  Udi
// Algorithm:
//     Call validate by request types
//Return TUE/FALSE
//**********************************************************
int ValidRequest()
{
   int minIdentCount = 6;


   if(MB_ST.u16_Modbus_Over_Can)
   { // No crc on CAN, so message is shorter
      minIdentCount -= 2;
   }

   if((MB_ST.s16_Modbus_Request == MB_CMD_ENCAPSULATION ) &&
         (MB_ST.s16_Recive_Counter <= minIdentCount))
   {
      SendException(MB_ILLEGAL_DATA_VALUE );//MB_ILLEGAL_DATA_VALUE
      return FALSE;
   }



   if((MB_ST.s16_Modbus_Request == MB_CMD_READ) ||
         (MB_ST.s16_Modbus_Request == MB_CMD_WRITE) ||
         (MB_ST.s16_Modbus_Request == MB_CMD_READ_WRITE ))
      return ValidRW(); // Read or Write requests

   if((MB_ST.s16_Modbus_Request   == MB_CMD_DATA_TRANSFER) )
      return ValidUMAS(); // Data transfer (UMAS) requests


   return TRUE;
}



//**********************************************************
// Function Name: ValidUMAS
// Description:
//       Validate input data for UMAS request
// Author:  Udi
// Algorithm:
//     Force packet size to what we have
//Return TUE/FALSE
//**********************************************************
int ValidUMAS()
{
   // 32 bytes for header or other overhead ?
   if( (MB_ST.s16_Umas_Packet_Size > MAX_RX - 32 ))
      MB_ST.s16_Umas_Packet_Size = MAX_RX -  32;


   return TRUE;
}





//**********************************************************
// Function Name: ValidRW
// Description:
//       Validate input data for read , write and read-write requests.
// Author:  Udi
// Algorithm:
//     If wrong address :      report  0x02 = Illegal Data Address
//     If wrong size (count):  report 0x03 = Illegal Data Value
//     Update in Mail from Juergen Fiess Oct 31 2013:
//         Odd address return Error 2 (MB_ILLEGAL_DATA_ADDRESS)
//         Odd Counter return error 3 (MB_ILLEGAL_DATA_VALUE)
//     Update for BY..456: Not enough char return 03 MB_ILLEGAL_DATA_VALUE
//     Verify each address in multiply Read/Write only is valid address (CQ 462 - Interoperability tests)
//Return TUE/FALSE
//**********************************************************

int ValidRW()
{
   int s16_NumAvilableBytes;
   int s16_NumBytesToWrite;
   int s16_MinReadCount = 8;
   int s16_MinWriteCount = 13;
   int idx = 2;
   unsigned int u16_p_param = 0;
   int s16_Cmd_index;
   unsigned int u16_param_index;

   if(MB_ST.u16_Modbus_Over_Can)
   { // No crc on CAN, so message is shorter
      s16_MinReadCount -= 2;
      s16_MinWriteCount -= 2;
   }

   if(((MB_ST.s16_Modbus_Request == MB_CMD_READ) && (MB_ST.s16_Recive_Counter < s16_MinReadCount )) ||
         ((MB_ST.s16_Modbus_Request == MB_CMD_WRITE)&& (MB_ST.s16_Recive_Counter < s16_MinWriteCount)))
   {
      SendException(MB_ILLEGAL_DATA_VALUE );
      return FALSE;
   }

   if((MB_ST.s16_Modbus_Param_Count  > 0x7B)  || (MB_ST.s16_Modbus_Param_Count  <= 1) ||
         ((MB_ST.s16_Modbus_Param + MB_ST.s16_Modbus_Param_Count)  > 0xFFFF) ||
         (MB_ST.s16_Modbus_Param_Count % 2 != 0) )

   {
      SendException(MB_ILLEGAL_DATA_VALUE );//MB_ILLEGAL_DATA_VALUE
      return FALSE;
   }


   if ((MB_ST.s16_Modbus_Param_Count < 1) ||
         (MB_ST.s16_Modbus_Param_Count > MB_MAX_PARAM_READ_WRITE) ||
         (MB_ST.s16_Modbus_Param % 2 != 0 ))
   {
      SendException(MB_ILLEGAL_DATA_ADDRESS);
      return FALSE;
   }


   // Verify each address in multiply Read/Write only is valid address (CQ 462 - Interoperability tests)
   if ((MB_ST.s16_Modbus_Param_Count >= 2) &&
         (MB_ST.s16_Modbus_Keep_Param != Monitor1_MB_ADDR ) &&  (MB_ST.s16_Modbus_Keep_Param != Monitor2_MB_ADDR)) //10.27 and 10.29 are special cases, where read-multiply is allowed.
   {

      u16_p_param = (unsigned int) ModbusToPParam(MB_ST.s16_Modbus_Param);
      for (idx = 0; idx < MB_ST.s16_Modbus_Param_Count; idx += 2 )
      {
         s16_Cmd_index = SearchPParamIndex(u16_p_param, &u16_param_index);
         if (s16_Cmd_index == 0) // If cmd not found
         {
            SendException(MB_ILLEGAL_DATA_ADDRESS );//MB_ILLEGAL_DATA_VALUE
            return FALSE;
         }
         u16_p_param += 1; //Each PParam is 2 modbus address
      }
   }



   // For Write: Verify byte Count match Number of Parameters  (CQ 462 - Interoperability tests)
   if ((MB_ST.s16_Modbus_Request == MB_CMD_WRITE) && (MB_ST.s16_Modbus_Keep_Original_Request  != MB_CMD_READ_WRITE))
   {
      s16_NumBytesToWrite  =  MB_ST.s16_Recive_Buff[6];
      if (s16_NumBytesToWrite < (MB_ST.s16_Modbus_Param_Count*2) )
      {
           SendException(MB_ILLEGAL_DATA_VALUE);
           return FALSE;
      }
   }





   //Repeat same validation for the "keep-parameter" part of read-write.

   if( MB_ST.s16_Modbus_Keep_Original_Request  == MB_CMD_READ_WRITE )
   {
      if ( MB_ST.s16_Modbus_Keep_Param_Count  > 0x7B  || MB_ST.s16_Modbus_Keep_Param_Count  <= 1 ||
            (MB_ST.s16_Modbus_Keep_Param + MB_ST.s16_Modbus_Keep_Param_Count)  > 0xFFFF ||
            MB_ST.s16_Modbus_Keep_Param_Count % 2  != 0 )
      {
         SendException(MB_ILLEGAL_DATA_VALUE );
         return FALSE;
      }



      if ( MB_ST.s16_Modbus_Keep_Param_Count < 1 ||
            MB_ST.s16_Modbus_Keep_Param % 2 != 0  ||
            MB_ST.s16_Modbus_Keep_Param_Count > MB_MAX_PARAM_READ_WRITE)
      {
         SendException(MB_ILLEGAL_DATA_ADDRESS);
         return FALSE;
      }



      // Verify each address in multiply Read/Write only is valid address (CQ 462 - Interoperability tests)
          if (MB_ST.s16_Modbus_Keep_Param_Count >= 2)
          {

             u16_p_param = (unsigned int) ModbusToPParam(MB_ST.s16_Modbus_Keep_Param);
             for (idx = 0; idx < MB_ST.s16_Modbus_Keep_Param_Count; idx += 2 )
             {

                s16_Cmd_index = SearchPParamIndex(u16_p_param, &u16_param_index);
                if (s16_Cmd_index == 0) // If cmd not found
                {
                   SendException(MB_ILLEGAL_DATA_ADDRESS );//MB_ILLEGAL_DATA_VALUE
                   return FALSE;
                }
                u16_p_param += 1; //Each PParam is 2 modbus address
             }
          }

          // Verify byte Count to write match Number of Parameters  (CQ 462 - Interoperability tests)
          s16_NumBytesToWrite  =  MB_ST.s16_Recive_Buff[10];
          if (s16_NumBytesToWrite < (MB_ST.s16_Modbus_Keep_Param_Count*2) )
          {
               SendException(MB_ILLEGAL_DATA_VALUE);
               return FALSE;
          }




      MB_ST.s16_Modbus_Keep_Param_Count /= 2;

      //Check if enough data in input buffer for write  (if FC =  read-write)
      s16_NumAvilableBytes = MB_ST.s16_Modbus_Param_Count * 2;
      if( (s16_NumAvilableBytes + 13) > MB_ST.s16_Recive_Counter)
      {
         SendException(MB_ILLEGAL_DATA_VALUE );
         return FALSE;
      }
   }

   if( MB_ST.s16_Modbus_Keep_Original_Request  == MB_CMD_WRITE )
   {
      //Check if enough data in input buffer for write (if FC =  write)
      s16_NumAvilableBytes = MB_ST.s16_Modbus_Param_Count * 2;
      if( (s16_NumAvilableBytes + 8) > MB_ST.s16_Recive_Counter)
      {
         SendException(MB_ILLEGAL_DATA_VALUE );
         return FALSE;
      }


   }





   MB_ST.s16_Modbus_Param_Count /=  2;
   return TRUE;
}



//**********************************************************
// Function Name: ModbusInitAnswer
// Description:
//       Prepare inital fields on the transmit buffer
// Author:  Udi
// Algorithm:
//            Set Address, Request ID
// Revisions:
// June 2013 First verion
//**********************************************************
int ModbusInitAnswer()
{
   int word_count;

   //Basic info in the answer
   // MB_ST.s16_Transmit_Buff[0] = MB_ST.s16_Modbus_DriveAddr; //Address
   if(MB_ST.s16_Modbus_Keep_Original_Request > 0)
      MB_ST.s16_Transmit_Buff[1] = MB_ST.s16_Modbus_Keep_Original_Request;
   else
      MB_ST.s16_Transmit_Buff[1] = MB_ST.s16_Modbus_Request;  //Request ID

   MB_ST.s16_Transmit_Counter = 2;


   //Validate input data
   //For Read and Write: Check if number of param is valid
   //  validate even number.

   if(!ValidRequest())
      return -1;


   word_count = MB_ST.s16_Modbus_Param_Count*2;

   switch
   (MB_ST.s16_Modbus_Request)
   {


      case MB_CMD_READ:
      case MB_CMD_READ_WRITE:             // For now answer for read and ReadWrite have identical format.
         MB_ST.s16_Transmit_Buff[2] = 0;                          //Actual data length - will be overiden
         MB_ST.s16_Transmit_Counter = 3;                          //Number bytes ready in the response
         break;
      case MB_CMD_WRITE:
         MB_ST.s16_Transmit_Buff[2] = H8(MB_ST.s16_Modbus_Param); //May need overide if error
         MB_ST.s16_Transmit_Buff[3] = L8(MB_ST.s16_Modbus_Param);
         MB_ST.s16_Transmit_Buff[4] = H8(word_count);             //May need overide if error
         MB_ST.s16_Transmit_Buff[5] = L8(word_count);
         MB_ST.s16_Transmit_Counter = 6;                          //Number bytes ready in the response
         break;
      case MB_CMD_DIAGNOSTIC:
         MB_ST.s16_Transmit_Buff[2] = H8(MB_ST.s16_Modbus_Sub_Function);
         MB_ST.s16_Transmit_Buff[3] = L8(MB_ST.s16_Modbus_Sub_Function);
         MB_ST.s16_Transmit_Counter = 4;
         break;
      case MB_CMD_ENCAPSULATION: //Identification
         MB_ST.s16_Transmit_Buff[2] = MB_ST.s16_Recive_Buff[2]; // MB_ID_MEI_14
         MB_ST.s16_Transmit_Buff[3] = MB_ST.s16_Recive_Buff[3]; // 1,2,or 3
         MB_ST.s16_Transmit_Buff[4] = 0x83; // Conformity level: extended identification(stream access and individual access)
         MB_ST.s16_Transmit_Buff[5] = 0; // No More follow
         MB_ST.s16_Transmit_Buff[6] = 0; // No Next object ID
         MB_ST.s16_Transmit_Buff[7] = 3; // Number of objects included: 3,8, or 13
         MB_ST.s16_Transmit_Counter = 8;
         break;
      case MB_CMD_DATA_TRANSFER:
         break;
      default:
         SendException(MB_ILLEGAL_FUNCTION);
         return -1;              // Command Not Known/Used
         // break;
   }



   return 1;

}


//**********************************************************
// Function Name: ModbusCloseAnswer
// Description:
//       Close Modbus Reply on the transmit buffer
// Author:  Udi
// Algorithm:
//            Add crc at end of response
// Revisions:
// 23 Apr 2013 First verion
//**********************************************************

void ModbusCloseAnswer()
{
   int16 crc;


   // Udi Feb 10, 2014 : No CRC in Modbus Over Can.
   if (MB_ST.u16_Modbus_Over_Can == 1)
   {
      // resetStatus(FALSE, FALSE);
      MB_ST.ModbusStatus = MB_SEND_ANSWER;
      return;
   }

   //Add CRC
   crc = CRC16(MB_ST.s16_Transmit_Buff, MB_ST.s16_Transmit_Counter);


   MB_ST.s16_Transmit_Buff[(MB_ST.s16_Transmit_Counter++)]= L8(crc);  //Chars already swapped
   MB_ST.s16_Transmit_Buff[(MB_ST.s16_Transmit_Counter++)]= H8(crc);


   MB_ST.ModbusStatus = MB_SEND_ANSWER;

}

//**********************************************************
// Function Name: setModbusSampleTime
// Description:
// Set the internal s32_Gap_Value in "clock-ticks", by sampling time in micro-sec
//  email from Juergen, Aug 6 2013 : If we have a 32-bit for the "sampling time" we can go like this:
//       31   =  31.25 \B5s
//       62   =  62.5 \B5s
//       125  = 125 \B5s
//       250  = 250 \B5s
//       500  = 500 \B5s
//       1000 =   1 ms
//       2000 =   2 ms
//Return True or false
// Author:  Udi
// Revisions:
//**********************************************************
int setModbusSampleTime(long Gap_Value_micro_sec)
{
   if(Gap_Value_micro_sec < 31 || Gap_Value_micro_sec > 31250000 ) //1 to 1,000,000 tick.
      return FALSE;

   if(Gap_Value_micro_sec < 100)
   {
      switch (Gap_Value_micro_sec)
      {
         case 31: s32_Gap_Value = 1;  break;
         case 62: s32_Gap_Value = 2;  break;
         case 93: s32_Gap_Value = 3;  break;// Not in the spec above.
         default:
            return FALSE;
      }
      return TRUE;
   }

   if(((Gap_Value_micro_sec * 100) % 3125) != 0)
      return FALSE;

   s32_Gap_Value = (Gap_Value_micro_sec * 100) /  3125;
   return TRUE;
}


//**********************************************************
// Function Name: GetModbusSampleTime
// Description: see setModbusSampleTime above
// Author:  Udi
// Revisions:
//**********************************************************
long GetModbusSampleTime(long Gap_Value)
{
   return (long) ((31.25) * Gap_Value);
}






//**********************************************************
// Function Name: CRC16
// Description:
//       Calculate CRC16 on byte per byte base.returns low byte first
// Author:  Udi Copied from modbus spec:
// "MODBUS over serial line specification and implementation guide V1.02"
// Algorithm: work on 8 bit bytes, todo- change to the int16 version.
// Revisions:
// 22 Apr 2013 First verion
//**********************************************************
int16 CRC16 (int16 s16_Msg_Buff[],int16 s16_Msg_Len )
{
   int s16_outCRCHi = 0xFF ; /* high byte of CRC initialized  */
   int s16_outCRCLo = 0xFF ; /* low byte of CRC initialized  */
   int16 s16_Tbl_Index ; /* will index into CRC lookup table  */
   int i = 0;
   while ((s16_Msg_Len--) > 0) /* pass through message buffer  */
   {
      s16_Tbl_Index = (int16 )(s16_outCRCLo ^ s16_Msg_Buff[i++]) ;   /* calculate the CRC   */
      s16_outCRCLo = (char) (s16_outCRCHi ^ s16_CRC_Tbl_Hi[s16_Tbl_Index]);
      s16_outCRCHi = s16_CRC_Tbl_Lo[s16_Tbl_Index] ;
   }
   return (int16)(s16_outCRCHi << 8 | s16_outCRCLo);
}


//**********************************************************
// Function Name: UploadFileLine
// Description:
//       Append one line to transmit buffer, acording to file type
// Author: Udi
//**********************************************************
int UploadFileLine(void)
{

   //If packet is full - return this packet
   if ( MB_ST.s16_Umas_Packet_Size - MB_ST.s16_Transmit_Counter < MB_ST.s16_Umas_Line_Size)
   {
      return SAL_SUCCESS; //send this packet
   }



   switch (MB_ST.s16_Umas_File_ID)
   {

      case FileID_Compatibility:  //return SAL_SUCCESS; //Entire file was loaded as Header
      case FileID_Files_List:
         MB_ST.s16_Umas_Packet_Idx = -1; //Mark as last pakadge   Udi Jan 24
         return SAL_SUCCESS;//Entire file was loaded as Header

      case FileID_Parameters1:
      case FileID_Parameters2:  return UploadUserParamLine();

      case FileID_Motor:        return UploadMotorLine();

      case FileID_Motion1:
      case FileID_Motion2:      return UploadPathLine();

      case FileID_Scope:          return UploadScopeLine();

      default:
         return SAL_SUCCESS; //Entire file was loaded as Header;;

   }
}





//**********************************************************
// Function Name: DownLoadFileLine
// Description:
//     Process one line from recived buffer, acording to file type
// Author: Udi
//**********************************************************
int DownLoadFileLine(void)
{
   switch (MB_ST.s16_Umas_File_ID)
   {

      case FileID_Compatibility:  return SAL_SUCCESS; // Place holder.DownloadCompatibilityFile();
      case FileID_Files_List:     return SAL_SUCCESS;

      case FileID_Parameters1:
      case FileID_Parameters2:
         return DownLoadParamLine();

      case FileID_Motor:
         return DownLoadMotorLine();

         //case FileID_CAN_Parameters:
      case FileID_Motion1:
      case FileID_Motion2:
         return DownLoadPathLine();

      case FileID_Scope:
         return SAL_SUCCESS;

      default:
         return SAL_SUCCESS;

   }
}


//**********************************************************
// Function Name: UploadFileHeader
// Description:
//       Set header of any file  on transmit buffer:
//       Also set line size to be used later.
// Author: Udi
// File format:
//================
//
//**********************************************************
int UploadFileHeader(void)
{
   //long long data;
   MB_ST.s16_Umas_Line_Size = 1;

   switch (MB_ST.s16_Umas_File_ID)
   {

      case FileID_Compatibility: return UploadCompatibilityFile();
      case FileID_Files_List:      return UploadFileList();

      case FileID_Parameters1:
      case FileID_Parameters2:
         Append_2_BytesTxBuff(GetParamFileCount(MB_ST.s16_Umas_File_ID));   //Number of parameters
         //SalReadSchneiderSoftwareVersion(&data, 0);
         //Append_2_BytesTxBuff((int)data);   // Get Firmware version as int
         MB_ST.s16_Umas_Line_Size = 3 * Param_Line_Size;
         return TRUE;
      case FileID_Motor:
         Append_2_BytesTxBuff(256); //Data size
         Append_2_BytesTxBuff(1); //MTP version
         return TRUE;

      case FileID_Motion1:
         Append_2_BytesTxBuff(Paths_In_Group1);
         Append_2_BytesTxBuff(4); // 4 parameter per path
         MB_ST.s16_Umas_Line_Size = 4 * Param_Line_Size;
         return TRUE;
      case FileID_Motion2:
         Append_2_BytesTxBuff(Paths_In_Group2);
         Append_2_BytesTxBuff(4); // 4 parameter per path
         MB_ST.s16_Umas_Line_Size = 4 * Param_Line_Size;
         return TRUE;

      case FileID_Scope:
         MB_ST.s16_Umas_Line_Size = s16_Recorded_Variables_Number * 4;
         UploadScopeHeader();
         return TRUE;

      default:
         return FALSE;

   }

}




/// Code below related to sending Recorded data via modbus
///Udi July 28 2013 - For this version the drive sends recorded data in user-units
// **********************************************************
// Function Name: UploadScopeHeader
// Description:
//       Set header of recorded data on transmit buffer:
//       num points, time interval, recorded parameters IDS
// Author: Udi
// File format:
//================
//size, Offset, Field
// 2, 0-1, Version
// 1, 2, Number of channels
// 1, 3, Number of points per channel
// 2, 4-5, Channel 1 parameter Logical Address
// 2*6, .., Channel n parameter Logical Address
// , X = 14-15?,
// , , Trigger characteristics
// 2, X+0, Trigger parameter Logical Address
// 2???, X+2, Sampling time
// 2, X+4, Trigger Value 1
// 1, X+6 - x+10, Trigger operation
// 1???, , Delay
// 1, , Trigger position in the data table
// 1???, , Scope mode
//
// Revisions:
// **********************************************************

int UploadScopeHeader(void)
{
   int i;
   long temp;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   MB_ST.s16_HeaderSize = Umas_Scope_Header_Size;// + (s16_Recorded_Variables_Number * 4); //4 bytes (32 bit) for chanel
   if ( MB_ST.s16_Umas_Packet_Size - MB_ST.s16_Transmit_Counter > MB_ST.s16_HeaderSize)
   {

      Append_4_BytesTxBuff(1);                        //version
      Append_2_BytesTxBuff(s16_Recorded_Variables_Number);//Num chanels
      Append_2_BytesTxBuff(s16_Recording_Length);     //Number of points

      // for(i=0; i < s16_Recorded_Variables_Number; i++)
      for(i=0; i < 4; i++) // Chanel Address kept as PParam - translate to MB address
      {
         Append_4_BytesTxBuff((unsigned long) PParamToModbus(s32_Schneider_Rec_Channel[i]));       //Add Modbus address
      }

      Append_4_BytesTxBuff(PParamToModbus(BGVAR(u32_P10_24_LOADRTRI)));  //Add  Trigger MB addres
      temp = GetModbusSampleTime(s32_Gap_Value);
      Append_4_BytesTxBuff(temp);  //Sampling time

      Append_4_BytesTxBuff(BGVAR(s32_P10_19_TRIGVAL1));     //Trigger Value
      Append_2_BytesTxBuff(BGVAR(u16_P10_17_TRIGOP)); //Trigger operation (Direction) ?? s16_Rec_Trig_Direction
      Append_2_BytesTxBuff(BGVAR(u16_P10_18_DELAYTRIG));               //Delay ?? same as Pre point below
      Append_2_BytesTxBuff(BGVAR(u16_P10_18_DELAYTRIG));             //Pre points Same as delay above
      Append_2_BytesTxBuff(0);             //Scope Mode

      Append_4_BytesTxBuff(12);             //???? "PTH Power Elapsed time" ???

      return TRUE; //send this packet
   }

   return TRUE;
}



// **********************************************************
// Function Name: UploadScopeLine
// Description:
//   Based on PrintBinaryDataPacket
//   Translate recoded data to user units, and send them via modbus umas
// Author: D.R./ Udi
// Algorithm:
//  The recorded data is send on multiply packets,
//  This procedure add one line to the current packet.
// Revisions:
// **********************************************************
int UploadScopeLine(void)
{

   static  int MB_Record_Ptr = -1;
   int i, ch_ptr;
   unsigned int u8_var_flags;
   unsigned int u16_index;
   unsigned long u32_value;
   unsigned long long u64_value;


   if(MB_ST.s16_Umas_Line_Idx_In_File == 0)
      MB_Record_Ptr = u16_Record_Start; //Start file

   //Add one line to the packet.
   ch_ptr = 0;
   for (i = 0; i < s16_Recorded_Variables_Number; i++)
   {
      u32_value = 0;
      //Get value of any var type into u64 variable.
      GetNextValue(MB_Record_Ptr,&ch_ptr,&u16_index,&u8_var_flags, i, &u64_value);

      if( s32_Schneider_Rec_Channel[i] < 1000)
      {
         //Translate from internal to user units
         ConvertValue(0, u16_index,  &u64_value);
         u32_value = (unsigned long) u64_value;
      }
      // Special case for p11-parameter
      // P11-xx are shadow parameter where unit translation is hard coded , not in the excel
      // P10-xx are not recordable
      else if( s32_Schneider_Rec_Channel[i] >= 1100 && s32_Schneider_Rec_Channel[i] < 1200)
      {
         u32_value =  ConvertDriveStatusValue(0, u64_value, s32_Schneider_Rec_Channel[i] - 1100,1 );

      }

      Append_4_BytesTxBuff(u32_value);

   }//end for

   MB_Record_Ptr++;      //increment cyclic buffer
   if (MB_Record_Ptr == u16_Ram_Rec_Length_Avaliable)   MB_Record_Ptr = 0;


   MB_ST.s16_Umas_Line_Idx_In_File ++;

   //If no more data -  send this packet, and mark as last in file
   if (MB_Record_Ptr == u16_Record_End)
   {
      MB_Record_Ptr = -1;
      MB_ST.s16_Umas_Packet_Idx = -1;

      MB_ST.s16_Umas_Line_Idx_In_File = 0;
      u16_MB_Read_Scope_Done = 1;
      return SAL_SUCCESS; //send this packet as last packet.
   }


   return SAL_NOT_FINISHED; //not finished yet - to be continued in next BG loop
}



// **********************************************************
// Function Name: SaveParamFromRecivedBuf
// Description:
//   Save (Execute P Param) One Parameter recived via UMAS file
//   Param stracure: int16 MB_ADDR; Int16 Type; Int32 Value
// Author:  Udi
// Algorithm:
// **********************************************************
int SaveParamFromRecivedBuf(int Data_Offset)
{
   //Check if all params in packet are saved.
   if ( Data_Offset + Param_Size_In_File >  MB_ST.s16_Recive_Counter)
   {
      return SAL_SUCCESS; //Send response packet
   }

   SaveParamFromBuffer(&MB_ST.s16_Recive_Buff[Data_Offset]);
   return PACKET_NOT_FINISHED; //Packet not finished yet - to be continued in next BG loop
}


// **********************************************************
// Function Name: DownLoadParamLine
// Description:
//   Process one Param line - part of download user param packet
// Author:  Udi
// Algorithm:
// **********************************************************
int DownLoadParamLine(void)
{

   int  Return_Value;

   Return_Value = SaveParamFromRecivedBuf(MB_ST.s16_Recived_Data_Offset);
   if(Return_Value == SAL_NOT_FINISHED)
      return Return_Value;

   MB_ST.s16_Recived_Data_Offset +=  Param_Size_In_File;
   MB_ST.s16_Modbus_Param_Idx ++;

   return Return_Value;
}





// **********************************************************
// Function Name: DownLoadMotorLine
// Description:
//   Process one line of motor parameters
// Author:  Udi
// Algorithm:
// **********************************************************
int DownLoadMotorLine(void)
{

   //    int  Data_Offset;
   //    int  Header_Size = DownLoad_Packet_Header_Size; // Umas Header size
   //    int  Return_Value;
   //    int idx;
   //
   //    if(MB_ST.s16_Umas_Packet_Idx == 1)
   //        Header_Size += Path_File_Header_Size; // Add Path  file header size if first packet of file
   //
   //    Data_Offset = Header_Size +
   //                  (MB_ST.s16_Modbus_Param_Idx * Path_Line_Size);//?? + 2;
   //
   //    for(idx = 0; idx < Params_In_Path; idx ++)
   //    {
   //        Return_Value = SaveParamFromBuffer(Data_Offset + (idx * Param_Size_In_File));
   //    }
   //    MB_ST.s16_Modbus_Param_Idx ++;

   //   return Return_Value;
   return SAL_SUCCESS;
}







// **********************************************************
// Function Name: DownLoadPathLine
// Description:
//   Process one motion path (with 4 parameters)
// Author:  Udi
// Algorithm:
// **********************************************************
int DownLoadPathLine(void)
{
   int  Return_Value;
   int idx;

   MB_ST.s16_Recived_Data_Offset += 2;
   for(idx = 0; idx < Params_In_Path; idx ++)
   {
      Return_Value = SaveParamFromRecivedBuf(MB_ST.s16_Recived_Data_Offset);
      MB_ST.s16_Recived_Data_Offset +=  Param_Size_In_File;
   }

   MB_ST.s16_Modbus_Param_Idx ++;

   return Return_Value;
}

// **********************************************************
// Function Name: UploadFileIDs
// Description:
//         Send hard coded list of avilable files
//         Creats header and full list in one call, same BG cycle.
// Author:  Udi
// Algorithm:
// Revisions:
// **********************************************************
int UploadFileIDs(int V, int W, int X, int Y)
{
   Append_1_BytesTxBuff(V);
   Append_1_BytesTxBuff(W);
   Append_1_BytesTxBuff(X);
   Append_1_BytesTxBuff(Y);
   return SAL_SUCCESS;
}

// **********************************************************
// Function Name: UploadFileList
// Description:
//         Send hard coded list of avilable files
//         Creats header and full list in one call, same BG cycle.
// Author:  Udi
// Algorithm:
// Revisions:
// **********************************************************
int UploadFileList()
{


   Append_2_BytesTxBuff(Num_Files); //number of files

   //   UploadFileIDs(255,255,255,255); //Compatibility
   //   UploadFileIDs(255,255,255,254); //Files_List

   UploadFileIDs(100,  1,0,0); //User Parameters
   UploadFileIDs(100,  2,0,0); //User Parameters
   // UploadFileIDs(100,  6,0,0); //Motor Parameters
   UploadFileIDs(100, 20,0,0); //Motion1
   UploadFileIDs(100, 21,0,0); //Motion2

   // UploadFileIDs(100,100,0,0); //Scope
   // UploadFileIDs(100,114,0,0); //Ident_File

   return SAL_SUCCESS; //send this packet
}



// **********************************************************
// Function Name: UploadCompatibilityFile
// Description:
//         Send Compatibility data (hw and fw version(
//         Creats header and packet in one call, same BG cycle.
// Author:  Udi
// Algorithm:
// Revisions:
// **********************************************************
int UploadCompatibilityFile()
{
   //long long s64_data;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //Config file version
   Append_1_BytesTxBuff(1); //File version High
   Append_1_BytesTxBuff(1); //File version Low

   //Program number (firmware number ) P9-00
   //SalReadSchneiderProgramNumber(&s64_data,0);
   //Append_4_BytesTxBuff((long) s64_data);

   //Version Number MAND.VERSION P00-00
   //SalReadSchneiderSoftwareVersion(&s64_data,0);
   //Append_2_BytesTxBuff((int) s64_data);

   //Min Version Number MAND.MINVERS
   // Send the same version as min version for parameter competability ?
   //Append_2_BytesTxBuff((int) s64_data);

   //Drive power Watt
   Append_2_BytesTxBuff(BGVAR(u16_Drive_Power_Watt));

   //Apr 27, 2015: Add Motor reference
   Append_4_BytesTxBuff (BGVAR(u32_MotorFileName));

   return SAL_SUCCESS; //send this packet
}





// **********************************************************
// Function Name: CheckCompatibilityFile
// Description:
//         Check Compatibility file
//         Return error if versions do not match
// Author:  Udi
// Revisions:
//           Apr 27, 2015: Add Motor reference
// **********************************************************
int CheckCompatibilityFile()
{
   //long long  s64_Drive_Data;
   long s32_File_Data;
   int s16_File_Data;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //Check Config file version

   if((int)(MB_ST.s16_Recive_Buff[11]) != UMAS_COMPATIBILITY_FILE_VER)
      return FALSE;

   //Ignore minor file version
   //   if((int)(MB_ST.s16_Recive_Buff[12]) != 1)    return FALSE;



   //Check Program number (firmware number ) P9-00
   //SalReadSchneiderProgramNumber(&s64_Drive_Data,0);
   s32_File_Data =  bytes2Int32(MB_ST.s16_Recive_Buff[13],MB_ST.s16_Recive_Buff[14],
         MB_ST.s16_Recive_Buff[15],MB_ST.s16_Recive_Buff[16]);
   //if( s32_File_Data != (long) s64_Drive_Data)
     // return FALSE;

   //Check Version Number MAND.VERSION P00-00
   //SalReadSchneiderSoftwareVersion(&s64_Drive_Data,0);
   s16_File_Data =  bytes2Int16(MB_ST.s16_Recive_Buff[17],MB_ST.s16_Recive_Buff[18]);

   //if( s16_File_Data > (int) s64_Drive_Data)
      //return FALSE;

   //Check Min Version Number MAND.MINVERS
   // Use Hard coded number 109 (version 1.9.xx is lowest version allowed in this fw).
   //Udi Nov 27 - Check: change to 0x116 (version 1.16.xx is lowest version allowed in this fw).
   //              And version format is Hex ???
   s16_File_Data =  bytes2Int16(MB_ST.s16_Recive_Buff[19],MB_ST.s16_Recive_Buff[20]);
   if( s16_File_Data < 0x115) //Was 109
      return FALSE;

   //Check Drive power Watt
   s16_File_Data =  bytes2Int16(MB_ST.s16_Recive_Buff[21],MB_ST.s16_Recive_Buff[22]);
   if( s16_File_Data != (int) BGVAR(u16_Drive_Power_Watt))
      return FALSE;

   //Apr 27, 2015: Add Motor reference. But also support older Compatibility files without this field.
   //              New file is identified by its length : More then 26 char.
   if(MB_ST.s16_Recive_Counter > Umas_Compatibility_File_Old_Buffer_Size)
   {
      //only check if motor is connected (i.e. u32_MotorFileName != 0)
      s32_File_Data =  bytes2Int32(MB_ST.s16_Recive_Buff[23],MB_ST.s16_Recive_Buff[24],MB_ST.s16_Recive_Buff[25],MB_ST.s16_Recive_Buff[26]);
      if( (BGVAR(u32_MotorFileName) != 0 ) && ( s32_File_Data != (long) BGVAR(u32_MotorFileName)))
         return FALSE;
   }

   return TRUE; //Compatibility check is OK
}



// **********************************************************
// Function Name: DownloadCompatibilityFile
// Description:
//         Check Compatibility file
//         Return error if versions do not match
// Author:  Udi
// Algorithm:
// Revisions:
// **********************************************************
int DownloadCompatibilityFile()
{

   if(CheckCompatibilityFile())
   {
      UmasInitAnswer(TRUE) ;
      Append_1_BytesTxBuff(0);//<Wait> byte in the spec for positive response
      ModbusCloseAnswer();
   }
   else
      UmasNegative(UMAS_ERR_COMPATIBILITY);

   return SAL_SUCCESS;
}


// **********************************************************
// Function Name: AddMTP_Field
// Description:
// Algorithm:
//  Copy one field from MTP data in memory to the transmit buffer.
// Revisions:
// **********************************************************
int AddMTP_Field(int segment, int offset, int size)
{
   int u16_Value_Buff[22];
   int i,s16_ret_val;

   s16_ret_val = GetDataFromMTP(segment, offset,(long *)u16_Value_Buff , size*8);
   if (s16_ret_val != SAL_SUCCESS)
      return -1; //Todo: get some error code;

   for(i=0; i<= size/2; i++)
      Append_2_BytesTxBuff(u16_Value_Buff[2*i]);

   return SAL_SUCCESS;
}

// **********************************************************
// Function Name: UploadMotorLine
// Description:
// Algorithm:
//  MTP data is stored in memory, so the entire packet is send as one line
// Revisions:
// **********************************************************
int UploadMotorLine()
{
   // Code to return the entire file
   //See Code below to return specific fields from it
   unsigned int i, start, num_longs; // In Long

   //Force packe size to be multiply of 4, as we copy "long" into it.
   if((MB_ST.s16_Umas_Packet_Size % 4) != 0)
      MB_ST.s16_Umas_Packet_Size -= (MB_ST.s16_Umas_Packet_Size % 4);

   start = (MB_ST.s16_Umas_Packet_Idx - 1) *  MB_ST.s16_Umas_Packet_Size;
   start /= 4;
   num_longs = MB_ST.s16_Umas_Packet_Size / 4;

   //Last pakage will be smaller
   if((start +  num_longs) > (MTP_IMAGE_SIZE))
      num_longs = MTP_IMAGE_SIZE  - start;

   // Copy from MTP-Image into Modbus transmit buffer
   for(i=0; i<= num_longs; i++)
      Append_4_BytesTxBuff(u32_MTP_Image[i]);


   if((start +  num_longs) >= (MTP_IMAGE_SIZE))
      MB_ST.s16_Umas_Packet_Idx = -1; //Mark as last pakadge

   return SAL_SUCCESS;
}


// Code
//Code to Return specific field by list in file Motor_Data_File_mtp_V02.xlsx
//    Total Bytes:         88
//See Code above to return specific fields from it

//Todo - check for failure
//    unsigned long u32_Result = 0;
//            //Segment,Offset,Sise (bytes)
//    AddMTP_Field(2,6,2); //uiEncodertype
//    AddMTP_Field(5,16,2); //bBrake
//    AddMTP_Field(5,24,2); //uiBrakeCouplingTime
//    AddMTP_Field(5,26,2); //uiBrakeDisconnectionTime
//    AddMTP_Field(3,20,2); //uiContStallTorque
//
//    AddMTP_Field(3,14,2); //uiContStallCurrent
//    AddMTP_Field(3,10,2); //uiPeakCurrent
//    AddMTP_Field(3,12,2); //uiNomCurrent
//    AddMTP_Field(5,6,2); //uiMaxSpeed
//    AddMTP_Field(3,6,2); //uiNomSpeed
//
//    AddMTP_Field(5,8,4); //ulMotorInertia
//    AddMTP_Field(3,52,4); //ulEMF_Constant
//    AddMTP_Field(3,8,2); //uiNomPhaseVoltage
//    AddMTP_Field(3,22,2); //uiPeakTorque
//
//    //Todo: Calculate it !!
//    Append_4_BytesTxBuff(u32_Result);
//
//    AddMTP_Field(4,8,2); //uiMaxMotorTemperature
//    AddMTP_Field(4,6,2); //uiThermalConstant
//    AddMTP_Field(1,92,4); //dwMotorfileName
//    AddMTP_Field(1,12,20); //tMotorname
//    AddMTP_Field(3,4,2); //uiPolePair
//
//    AddMTP_Field(3,32,2); //uiPhaseResistance
//    AddMTP_Field(3,34,2); //uiQuadraturePhaseInductance
//    AddMTP_Field(3,36,2); //uiDirectPhaseInductance
//    // 24.      not available          ??
//    AddMTP_Field(1,32,20); //tMotorSerialNumber
//
//    return SAL_SUCCESS;

//}


// **********************************************************
// Function Name: UploadPathLine
// Description:
//   Add one motion path (with all 4 parameters) to the current packet to upload
// Related P -Params are:
//            Path     PATHCTRL    ACC_DEC     SPD_DLY
//  Path 1  : P6-02        P6-03       P7-02      P7-03
//  Path 2 : P6-04         P6-05       P7-04      P7-05
//  File 1 contains 17 paths: 0-16, ; File 2 contains 16 paths: 17-32
// Author:  Udi
// Algorithm:
// Revisions:
// **********************************************************
int UploadPathLine()
{
   int Path_Index;
   int Base_Param6;
   int Base_Param7;
   int Paths_In_Group;
   int Idx_offset;

   if (MB_ST.s16_Umas_File_ID == FileID_Motion1)
   {
      Paths_In_Group = Paths_In_Group1;
      Base_Param6 = 600 ;
      Idx_offset = 0;
   }
   else
   {
      Paths_In_Group = Paths_In_Group2;
      Base_Param6 = 600 + (Paths_In_Group1 * 2);
      Idx_offset = Paths_In_Group1;
   }
   Base_Param7 = Base_Param6 + 100;
   Path_Index = MB_ST.s16_Umas_Line_Idx_In_File;


   //Add one line with index and 4 param to the packet.

   //Each param includes: Modbus addr, value-type, value
   Append_2_BytesTxBuff(Path_Index + Idx_offset);
   AppendParam(Base_Param6 + (Path_Index * 2));   //e.g. p6-00,2, ...
   AppendParam(Base_Param6 + (Path_Index * 2)+1); //e.g. p6-01,3, ...
   AppendParam(Base_Param7 + (Path_Index * 2));   //e.g. p7-00,2, ...
   AppendParam(Base_Param7 + (Path_Index * 2)+1); //e.g. p7-01,3, ...


   MB_ST.s16_Umas_Line_Idx_In_File ++;
   //If File is done -   Mark as end of file send this packet

   if (MB_ST.s16_Umas_Line_Idx_In_File >= Paths_In_Group)
   {

      MB_ST.s16_Umas_Line_Idx_In_File = 0;
      MB_ST.s16_Umas_Packet_Idx = -1; //Mark as last pakadge- Udi Jan 28
      return SAL_SUCCESS; //send this packet as last packet.
   }


   return SAL_NOT_FINISHED; //not finished yet - to be continued in next BG loop
}

//**********************************************************
// Function Name: GetParamFileCount
// Description:
//         Returns number of Parameters in the selected file
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int GetParamFileCount()
{
   int i,j;
   int s16_param_count = 0;

   for (i=0; i<COMMANDS_TABLE_SIZE; i++)
   {
      if(Commands_Table[i].u16_dump_group < 2 )
         continue ; // Do not count if not in dump;

      if(Commands_Table[i].u16_p_param_number != 0xFFFF )
      {
         if(((MB_ST.s16_Umas_File_ID == FileID_Parameters1) && (Commands_Table[i].u16_p_param_number < 600)) ||
               ((MB_ST.s16_Umas_File_ID == FileID_Parameters2) && (Commands_Table[i].u16_p_param_number >= 800)))
            s16_param_count++;  //Count line with dump-group snd P_Param in range for file ID
         continue;
      }
      //Handle lines with list of P-Param:
      if(Commands_Table[i].u16_p_param_list_index != 0)
      {
         for(j = 0; j < SUB_INDEX_SIZE; j++)
         { // Count all pparm in list for command table entry with few P Param
            if( P_Param_List[Commands_Table[i].u16_p_param_list_index-1][j] == 0xFFFF)
               break;
            if(((MB_ST.s16_Umas_File_ID == FileID_Parameters1) && (P_Param_List[Commands_Table[i].u16_p_param_list_index-1][j] < 600)) ||
                  ((MB_ST.s16_Umas_File_ID == FileID_Parameters2) && (P_Param_List[Commands_Table[i].u16_p_param_list_index-1][j] >= 800)))
               s16_param_count++;  //Count only if  P_Param in range for file ID

         }
      }

   }
   return s16_param_count;
}


//**********************************************************
// Function Name: GetParamValue
// Description:
//             Get Value of specific P param
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int GetParamValue(int pParam, long *s32_Value)
{
   int  Return_Value = ExecutePParam(0, pParam,  OP_TYPE_READ, s32_Value, LEX_CH_MODBUS_RS485);
   if (Return_Value == SAL_NOT_FINISHED)
      return SAL_NOT_FINISHED;

   if (Return_Value != SAL_SUCCESS)
   {
      MB_ST.s16_Last_Error_Code = Return_Value;
      MB_ST.s16_Addr_Last_Error = PParamToModbus(pParam);

      return -1;
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: AppendParamByCmdIdx
// Description:
//             Add Modbus address, type and  value per each Command-idx
//             One command_index may inlude several PPAram.
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int AppendParamByCmdIdx(int Cmd_Idx)
{
   static int PParam_Sub_Idx = 0;
   static int Last_Cmd_Idx = 0;
   int PParam , PParam_Idx;

   PParam = Commands_Table[Cmd_Idx].u16_p_param_number;
   if (PParam != 0xFFFF && PParam != 0 )
   {
      //  Last_Cmd_Idx = Cmd_Idx;
      return (AppendParam(PParam));
   }

   //Command in this idx have list of PPAram : add next sub idx to transmit buffer.
   PParam_Idx = Commands_Table[Cmd_Idx].u16_p_param_list_index;
   if(PParam_Idx == 0xFFFF || PParam_Idx == 0)
      return SAL_SUCCESS;


   if(Last_Cmd_Idx == Cmd_Idx)
      PParam_Sub_Idx ++;
   else
   {
      PParam_Sub_Idx = 0;
      Last_Cmd_Idx = Cmd_Idx;
   }

   PParam = P_Param_List[PParam_Idx - 1][PParam_Sub_Idx];
   if(PParam == 0xFFFF)
   {
      PParam_Sub_Idx = 0;
      return SAL_SUCCESS;
   }

   //Check if this PParam is in this file.
   if ((PParam < s16_First_In_File) || (PParam > s16_Last_In_File))
      return SAL_NOT_FINISHED;



   AppendParam(PParam);
   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: AppendParam
// Description:
//             Add Modbus address, type and  value per each param
//             used for user-param and Motion Path files.
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int AppendParam(int pParam)
{

   int  modbusAddr, type, Return_Value; //
   long paramValue;

   if(pParam < 1 || pParam > 20000) //Safe-guard, should be check in the main loop
      return -1;

   modbusAddr = PParamToModbus(pParam);

   type = 2; // find real type.
   Return_Value = GetParamValue(pParam, &paramValue);
   if (Return_Value == SAL_NOT_FINISHED)
      return SAL_NOT_FINISHED;


   Append_2_BytesTxBuff(modbusAddr);
   Append_2_BytesTxBuff(type);
   Append_4_BytesTxBuff(paramValue);

   return SAL_SUCCESS;

}

//**********************************************************
// Function Name: UploadUserParamLine
// Description:
//   Preapre backup file on Modbus Transmit buffer, for Upload user file via UMAS
//   Copy of DumpCommand adapted for Modbus and P Param.
//
//  Send commands with P Param regardless of their dump priority,
//  where PParam P0-00 to P5-99 in file 1; P8-00 to P10-99 in file 2
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int UploadUserParamLine()
{
   static int mb_dump_state = UPLOAD_INIT;  //search param, Add param
   static int mb_dump_idx = 1;
   int PParam;
   unsigned int PParam_Idx;
   int return_status;

   switch (mb_dump_state)
   {
      case UPLOAD_INIT:
         mb_dump_idx = 1;
         if(MB_ST.s16_Umas_File_ID == FileID_Parameters1)
         {//File 1 with PParam P0-00 to P5-99
            s16_First_In_File = 0;
            s16_Last_In_File =  599;
         }
         else
         {//File 2 with PParam P8-00 to P10-99
            s16_First_In_File = 800;
            s16_Last_In_File =  1099;
         }
         mb_dump_state = UPLOAD_SEARCH_PARAM;
         //no break
      case UPLOAD_SEARCH_PARAM: // Search cmd with  P Param

         //Find next line in Command Table with PPAram to be included in this file.
         for(;mb_dump_idx < COMMANDS_TABLE_SIZE; mb_dump_idx++)
         {
            if (Commands_Table[mb_dump_idx].u16_dump_group < 2 )
               continue;
            //Check if PParam is in the list of multipl PParam per line
            PParam_Idx = Commands_Table[mb_dump_idx].u16_p_param_list_index;
            if((PParam_Idx > 0)  && (PParam_Idx <= SUB_INDEX_SIZE) )
               PParam =  P_Param_List[PParam_Idx - 1][0];
            else
               PParam =  Commands_Table[mb_dump_idx].u16_p_param_number;

            //Check if this PParam is in this file.
            if ((PParam < s16_First_In_File) || (PParam > s16_Last_In_File))
               continue;
            break;
         }

         if (mb_dump_idx >= COMMANDS_TABLE_SIZE)
         { // End of dump
            MB_ST.s16_Umas_Packet_Idx = -1; //Mark as last pakadge
            mb_dump_state = UPLOAD_INIT;
            return SAL_SUCCESS;
         }
         mb_dump_state = UPLOAD_ADD;
         //no break

      case UPLOAD_ADD:
         return_status = AppendParamByCmdIdx(mb_dump_idx);
         MB_ST.s16_Umas_Line_Idx_In_File++;
         if(return_status == SAL_NOT_FINISHED)
            return SAL_NOT_FINISHED;

         mb_dump_idx++;
         mb_dump_state = UPLOAD_SEARCH_PARAM;
         return SAL_NOT_FINISHED;

      default:
         return SAL_SUCCESS; // Should never happend
   }
}
