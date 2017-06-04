//###########################################################################
//
// FILE: Hiface.c
//
// TITLE:   Contains Hiperface routines
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 30 MAY 2004 | D.R. | Creation
//
//##################################################################

#include "DSP2834x_Device.h"

#include "Design.def"
#include "Endat.def"
#include "Err_Hndl.def"
#include "FltCntrl.def"
#include "FPGA.def"
#include "Hiface.def"
#include "ModCntrl.def"
#include "MultiAxis.def"
#include "Ser_Comm.def"

#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Hiface.var"
#include "Motor.var"
#include "Ser_Comm.var"
#include "Drive.var"
#include "MotorSetup.var"

#include "Prototypes.pro"


void ScicSendByte(unsigned int u8_byte)
{
   if (u16_Product != SHNDR_HW)
   {
      *(int*)FPGA_MFB_ENCODER_RX_TX_REG_ADD |= 0x0002;    //en transmit mode , dis receive

      ScicRegs.SCITXBUF = u8_byte;
      while (ScicRegs.SCICTL2.bit.TXEMPTY == 0);

      *(int*)FPGA_MFB_ENCODER_RX_TX_REG_ADD &= ~0x0002;    //dis transmit mode, en receive
   }
}


void ScicClrRxBuffer(void)
{
   if (u16_Product != SHNDR_HW)
   {
      ScicRegs.SCIFFRX.bit.RXFIFORESET  = 0;  // reset FIFO
      ScicRegs.SCIFFRX.bit.RXFIFORESET  = 1;  // reset FIFO
   }
}


void ScicGetBuffer()
{
   unsigned int u16_i = 0;

   if (u16_Product != SHNDR_HW)
   {
      while ( (ScicRegs.SCIFFRX.bit.RXFFST > 0) && (u16_i < HIFACE_BUFFER_SIZE))
      {
         u8_Scic_Rx_Data[u16_i] = ScicRegs.SCIRXBUF.bit.RXDT;      //  Read data
         u16_i++;
      }
   }
}


//**********************************************************
// Function Name: ReadHifaceAbsPos
// Description:
//          This function is called to read HIFACE HW position
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int ReadHifaceAbsPos(int drive)
{
   int ret_val = 0, index;
   static int state = 0;
   long abs_pos;

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (u16_Product == SHNDR_HW) //Udi Dec 26 2013: Scic Used by Modbus
       return SAL_SUCCESS;

   switch (state)
   {
      case 0:
         state++;

      case 1:
         ScicClrRxBuffer();
         ScicSendByte(0x40);  // Send the encoder default address 0x40
         ScicSendByte(0x42);  // Send the Read absolute position command
         ScicSendByte(0x02);  // Send the checksum value (EXOR)

         state++;
         s32_Hiface_Abs_Pos_Timer = Cntr_1mS;
         VAR(AX0_s16_Sine_Enc_Bits) |= SW_SINE_READ_POS_ENA_MASK;
      break;

      case 2:
         if ( (PassedTimeMS(250L, s32_Hiface_Abs_Pos_Timer)) || (u16_Hiface_Rx_Buffer_Len == 7) )
         {
            if (u16_Hiface_Rx_Buffer_Len < 7)            // If not enough data recieved
            {
               ret_val = COMMUNICATION_ERROR;
               BGVAR(s16_Hiface_Error) |= (HIPERFACE_TIMEOUT_ERROR | 0x1000);
               state = 0;
               break;
            }
            ScicGetBuffer();                //copy FIFO to ram

            s16_Hiface_Rx_Chksum_Read = u8_Scic_Rx_Data[6]; // Read the checksum value
            s16_Hiface_Rx_Data_Exor = 0;
            for (index = 0; index < 6; index++)
            {
               s16_Hiface_Rx_Data_Exor ^= u8_Scic_Rx_Data[index];
            }

            if (s16_Hiface_Rx_Chksum_Read != s16_Hiface_Rx_Data_Exor) // Check the checksum value,
            {                   // if different from expected value then signal an error and abort
               ret_val = COMMUNICATION_ERROR;
               BGVAR(s16_Hiface_Error) |= (HIPERFACE_CHECKSUM_ERROR | 0x1000);
               state = 0;
               break;
            }

            if (ScicRegs.SCIRXST.bit.RXERROR == 1)
            {
               ret_val = COMMUNICATION_ERROR;
               BGVAR(s16_Hiface_Error) |= (HIPERFACE_PARITY_ERROR | 0x1000);
               state = 0;
               break;
            }

            abs_pos   = 0;
            abs_pos |= u8_Scic_Rx_Data[2];
            abs_pos <<= 8;
            abs_pos |= u8_Scic_Rx_Data[3];
            abs_pos <<= 8;
            abs_pos |= u8_Scic_Rx_Data[4];
            abs_pos <<= 8;
            abs_pos |= u8_Scic_Rx_Data[5];

            LVAR(AX0_s32_Sw_Sine_Abs_Pos_32) = abs_pos;
            ret_val = SAL_SUCCESS;
            state = 0;
         }
   }

   return ret_val;
}


//**********************************************************
// Function Name: ReadHifaceEncType
// Description:
//          This function is called to read HIFACE Encoder Type
//          and other data
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int ReadHifaceEncType(long *u32_enc_res, int drive)
{
   int ret_val = 0, i, index;
   static int state = 0;
   long Type_Label;
   // AXIS_OFF;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch (state)
   {
      case 0:
         state++;

      case 1:
         ScicClrRxBuffer();
         ScicSendByte(0x40);  // Send the encoder default address 0x40
         ScicSendByte(0x52);  // Send the Read Type Label command
         ScicSendByte(0x12);  // Send the checksum value (EXOR)

         state++;
         s32_Hiface_Abs_Pos_Timer = Cntr_1mS;
      break;

      case 2:
         if ( (PassedTimeMS(250L, s32_Hiface_Abs_Pos_Timer)) || (u16_Hiface_Rx_Buffer_Len == 7) )
         {
            if (u16_Hiface_Rx_Buffer_Len < 7)            // If not enough data recieved
            {
               ret_val = COMMUNICATION_ERROR;
               BGVAR(s16_Hiface_Error) |= (HIPERFACE_TIMEOUT_ERROR | 0x2000);
               state = 0;
               break;
            }
            ScicGetBuffer();                //copy FIFO to ram

            s16_Hiface_Rx_Chksum_Read = u8_Scic_Rx_Data[6]; // Read the checksum value
            s16_Hiface_Rx_Data_Exor = 0;
            for (i = 0; i < 6; i++)
               s16_Hiface_Rx_Data_Exor ^= u8_Scic_Rx_Data[i];

            if (s16_Hiface_Rx_Chksum_Read != s16_Hiface_Rx_Data_Exor) // Check the checksum value,
            {                   // if different from expected value then signal an error and abort
               ret_val = COMMUNICATION_ERROR;
               BGVAR(s16_Hiface_Error) |= (HIPERFACE_CHECKSUM_ERROR | 0x2000);
               state = 0;
               break;
            }

            if (ScicRegs.SCIRXST.bit.RXERROR == 1)
            {
               ret_val = COMMUNICATION_ERROR;
               BGVAR(s16_Hiface_Error) |= (HIPERFACE_PARITY_ERROR | 0x2000);
               state = 0;
               break;
            }
            state++;
         }
      break;

      case 3:
         Type_Label   = 0;
         Type_Label |= u8_Scic_Rx_Data[2];
         Type_Label <<= 8;
         Type_Label |= u8_Scic_Rx_Data[3];
         Type_Label <<= 8;
         Type_Label |= u8_Scic_Rx_Data[4];
         Type_Label <<= 8;
         Type_Label |= u8_Scic_Rx_Data[5];

         BGVAR(s32_Hiface_Type_Label) = Type_Label;
         Type_Label = (BGVAR(s32_Hiface_Type_Label) >> 16) & 0xFF;

         switch (Type_Label)
         { // Update as required...
            case 0x42: case 0x47: // Encoder Families SEK and SEL...
               index = 16;
            break;
            case 0x32: case 0x37: // Encoder Families SKS and SKM...
               index = 128;
            break;
            case 0x02: case 0x07: // Encoder Families SHS, SCS, and SCM...
               index = 512;
            break;
            case 0x22: case 0x27: // Encoder Families SRS, SCK, SFS60, SRM, SCL, and SFM60...
               index = 1024;
            break;
            case 0x82: case 0x90: case 0x91: case 0x92: case 0x93: case 0x94: // Encoder Families
               index = 0; // LinCoder and DME4000/5000...
            break;
            case 0xFF: // Encoder Families SEK90, SEK160, SEK260, TTK70, and XKS
               index = 1; // Additional information needs to be read from the Encoder...
            break;
            default: // Unknown Encoder Type
               index = -1;
            break;
         }

         if ( (Type_Label == 0x02) || (Type_Label == 0x22) || (Type_Label == 0x32) || (Type_Label == 0x42) )
         {
            BGVAR(u16_Stegmann_Single_Turn) = 1;
         }
         else
         {
            BGVAR(u16_Stegmann_Single_Turn) = 0;
            VAR(AX0_u16_Abs_Fdbk_Device) |= 0x01;
            BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = HIPERFACE_ENCDR_MAX_NUM_TURNS;
         }

         if (index == -1)
         {
            BGVAR(u64_Sys_Warnings) &= ~HIFACE_MENCRES_MISMATCH_WRN;
            BGVAR(u16_Sys_Remarks) &= ~HIFACE_TYPE_NOT_SUPPORTED;
            BGVAR(u16_Sys_Remarks) |= HIFACE_UNKNOWN_DEVICE_WRN;
         }
         else if (index == 0)
         {
            BGVAR(u64_Sys_Warnings) &= ~HIFACE_MENCRES_MISMATCH_WRN;
            BGVAR(u16_Sys_Remarks) &= ~HIFACE_UNKNOWN_DEVICE_WRN;
            BGVAR(u16_Sys_Remarks) |= HIFACE_TYPE_NOT_SUPPORTED;
         }
         else
         { // index == 1 is used to signal additional Data Reads, therefore no Warning issued
            BGVAR(u16_Sys_Remarks) &= ~(HIFACE_UNKNOWN_DEVICE_WRN | HIFACE_TYPE_NOT_SUPPORTED);
            if ( (index != 1) && (index == BGVAR(u32_User_Motor_Enc_Res)) )
               BGVAR(u64_Sys_Warnings) &= ~HIFACE_MENCRES_MISMATCH_WRN;
            else
               BGVAR(u64_Sys_Warnings) |= HIFACE_MENCRES_MISMATCH_WRN;
         }
         if (index > 0) *u32_enc_res = (long)index;
         ret_val = SAL_SUCCESS;
         state = 0;

         // This updates the operational homing offset with the user non-volatile variable
         // according to the feedback multi turn absolute
         UpdateRefOffsetValue(drive);
      break;
   }

   return ret_val;
}


// Read the Hiperface data without printing it, so it can be used by the SAL function and internaly
int HifaceCmdGenericSilent(int *s16_number_of_rcvd_bytes)
{
   unsigned int i = 0;

   if (u16_Product == SHNDR_HW) //Udi Dec 26 2013: Scic Used by Modbus
       return NOT_SUPPORTED_ON_HW;

   ScicClrRxBuffer();   // Clear Serial Comm. Interface C Receive Buffer

   ScicSendByte(0x40);  // Send the encoder default address 0x40
   s16_Hiface_Rx_Data_Exor = 0x40;  // Initialize Checksum value

   for (i = 0; i < s16_Number_Of_Parameters; i++)   // Send Command and associated Parameters,
   {                                       // and update Command Cheecksum for each Parameter.
      ScicSendByte((int)s64_Execution_Parameter[i] & 0xFF);
      s16_Hiface_Rx_Data_Exor ^= ((int)s64_Execution_Parameter[i] & 0xFF);
   }

   ScicSendByte(s16_Hiface_Rx_Data_Exor & 0xFF); // Send Command Checksum.
   i = 0;
   s32_Hiface_Abs_Pos_Timer = Cntr_1mS;

   while ((!PassedTimeMS(50L, s32_Hiface_Abs_Pos_Timer)) || (u16_Hiface_Rx_Buffer_Len > 0) )
   {
      if (u16_Hiface_Rx_Buffer_Len > 0)                           // If Data still coming in
      {
         u8_Scic_Rx_Data[i++] = ScicRegs.SCIRXBUF.bit.RXDT;       // Read data, advance Index
         s32_Hiface_Abs_Pos_Timer = Cntr_1mS;                     // Restart the "waiting" time
         if (i >= HIFACE_BUFFER_SIZE) // Too much data recieved
            return HF_TOO_MUCH_DATA;
      }
   }
   *s16_number_of_rcvd_bytes = i;

   for (;i < HIFACE_BUFFER_SIZE; i++) // Clean the unused part of the Buffer
      u8_Scic_Rx_Data[i] = 0;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalHifaceCmdGeneric
// Description:
//   This function called in response to HIFACEREAD command
// Instructions:
//   a.  Encoder Address 0x40 is assumed and hard-coded.
//   b.  Enter all Parameters of Hiperface Command in order; the Function calculates Checksum.
//   c.  If printing is problematic, observe array of u8_Scic_Rx_Data[32] for response bytes
//       (including Address, Command, specifics, and Checksum).
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalHifaceCmdGeneric(void)
{
   int s16_ret_val, i, s16_number_of_rcvd_bytes;

   s16_ret_val = HifaceCmdGenericSilent(&s16_number_of_rcvd_bytes);

   if (s16_ret_val == SAL_SUCCESS)
   {
      for (i = 0; i < s16_number_of_rcvd_bytes; i++)
      {
         PrintDecInAsciiHex((unsigned long long)u8_Scic_Rx_Data[i], 2);
         PrintChar(' ');
         if (((i % 8) == 7) && (i < (s16_number_of_rcvd_bytes - 1))) PrintCrLf();
      }
      PrintCrLf();
   }

   return s16_ret_val;
}


//**********************************************************
// Function Name: ReadHifaceEncData
// Description:
//   This function called to determine Encoder Resolution when Type-Label is 0xFF
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int ReadHifaceEncData(long *u32_res_data, int drive)
{
   int ret_val = 0, index;
   static int state = 0;
   unsigned long Periods_Rev;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (u16_Product == SHNDR_HW) //Udi Dec 26 2013: Scic Used by Modbus
       return SAL_SUCCESS;


   switch (state)
   {
      case 0:
         state++;

      case 1:
         ScicClrRxBuffer();
         ScicSendByte(0x40);  // Send the encoder default address 0x40
         ScicSendByte(0x4A);  // Send the Read Data Field command
         ScicSendByte(0xFF);  // Send the Data Field designator (Encoder Description Field)
         ScicSendByte(0x00);  // Send the Data Field 1st Byte address (Byte 0 for Encoder Data)
         ScicSendByte(0x0A);  // Send the Byte Count (10, to read Enc. Resolution only)
         ScicSendByte(0x55);  // Send the Field Access Code (default)
         ScicSendByte(0xAA);  // Send the checksum value (EXOR)

         state++;
         s32_Hiface_Abs_Pos_Timer = Cntr_1mS;
      break;

      case 2:
         if ( (PassedTimeMS(250L, s32_Hiface_Abs_Pos_Timer)) || (u16_Hiface_Rx_Buffer_Len >= 16) )
         {
            if (u16_Hiface_Rx_Buffer_Len < 16)            // If not enough data recieved
            {
               ret_val = COMMUNICATION_ERROR;
               BGVAR(s16_Hiface_Error) |= (HIPERFACE_TIMEOUT_ERROR | 0x4000);
               state = 0;
               break;
            }
            ScicGetBuffer();                //copy FIFO to ram

            s16_Hiface_Rx_Chksum_Read = u8_Scic_Rx_Data[15]; // Read the checksum value
            s16_Hiface_Rx_Data_Exor = 0;
            for (index = 0; index < 15; index++)
               s16_Hiface_Rx_Data_Exor ^= u8_Scic_Rx_Data[index];

            if (s16_Hiface_Rx_Chksum_Read != s16_Hiface_Rx_Data_Exor) // Calculate checksum value,
            {                   // if different from expected value signal an error and abort
               ret_val = COMMUNICATION_ERROR;
               BGVAR(s16_Hiface_Error) |= (HIPERFACE_CHECKSUM_ERROR | 0x4000);
               state = 0;
               break;
            }

            if (ScicRegs.SCIRXST.bit.RXERROR == 1)
            {
               ret_val = COMMUNICATION_ERROR;
               BGVAR(s16_Hiface_Error) |= (HIPERFACE_PARITY_ERROR | 0x4000);
               state = 0;
               break;
            }
            if ((u8_Scic_Rx_Data[6] & 0x000F) != 0)
            { // Linear Device, not supported.
               *u32_res_data = 0;
               BGVAR(u64_Sys_Warnings) &= ~(HIFACE_MENCRES_MISMATCH_WRN);
               BGVAR(u16_Sys_Remarks) &= ~(HIFACE_UNKNOWN_DEVICE_WRN);
               BGVAR(u16_Sys_Remarks) |= HIFACE_TYPE_NOT_SUPPORTED;
            }
            else
            { // Determine MENCRES from Encoder Data
               Periods_Rev   = 0;
               Periods_Rev |= u8_Scic_Rx_Data[7];
               Periods_Rev <<= 8;
               Periods_Rev |= u8_Scic_Rx_Data[8];
               Periods_Rev <<= 8;
               Periods_Rev |= u8_Scic_Rx_Data[9];
               Periods_Rev <<= 8;
               Periods_Rev |= u8_Scic_Rx_Data[10];
               *u32_res_data = Periods_Rev;
               BGVAR(u16_Sys_Remarks) &= ~(HIFACE_UNKNOWN_DEVICE_WRN | HIFACE_TYPE_NOT_SUPPORTED);
               if (Periods_Rev != BGVAR(u32_User_Motor_Enc_Res))
                  BGVAR(u64_Sys_Warnings) |= HIFACE_MENCRES_MISMATCH_WRN;
               else
                  BGVAR(u64_Sys_Warnings) &= ~HIFACE_MENCRES_MISMATCH_WRN;
            }
            ret_val = SAL_SUCCESS;
            state = 0;
            break;
         }
   }

   return ret_val;
}


//**********************************************************
// Function Name: HifaceHandler
// Description:
//    HIPERFACE handler, run each background
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void HifaceHandler(int drive)
{
   // AXIS_OFF;
   int  s16_temp, ret_val, pos_mismatch;
   long long s64_temp;

   if (!FEEDBACK_STEGMANN)
      return;

   switch (BGVAR(u16_Hiface_Init_State))
   {
      case HIFACE_IDLE_STATE:
         if ( (BGVAR(u16_MTP_Mode) != 2) && (BGVAR(u16_MTP_Mode) != 3) ) // Load MPHASE and PFBoffset from
            ret_val = LoadParamsFromHiperface(drive); // Hiperface's EEPROM only if MTPMODE is not 2 or 3.
         else // avoid Fault Indication if not reading Data when MTPMODE is 2 or 3.
            ret_val = SAL_SUCCESS;
         if ((SAL_SUCCESS != ret_val) && (SAL_NOT_FINISHED != ret_val))
         {
            BGVAR(u32_Hiface_Init_Error) = ((0x1000000 * ((long)BGVAR(u16_Hiface_Init_State))) | ((0x100 * ((long)ret_val))));
            BGVAR(u16_Hiface_Init_State) = HIFACE_INIT_ERROR_STATE;
         }
         else if (SAL_SUCCESS == ret_val)
         {
            VAR(AX0_s16_Sin_Cos_Negate) = 0x0000; // Reset the "Attempts" Counter to allow two
                           // passes, Initializing with and without correction if required.
            BGVAR(u32_Hiface_Init_Error) = 0;
         }
      break;

      case HIFACE_REVERSAL_RESET_STATE:
         ResetEncState(drive);    //reset encoder state machine

         s32_Hiface_Timer = Cntr_1mS;
         BGVAR(u16_Hiface_Init_State) = HIFACE_WAIT_REVERSAL_STATE;
      break;

      case HIFACE_WAIT_REVERSAL_STATE:  //  Wait for reset applied
         if (PassedTimeMS(500L, s32_Hiface_Timer))
         {
            BGVAR(u16_Hiface_Init_State) = HIFACE_START_INIT;
         }
      break;

      case HIFACE_START_INIT:
         VAR(AX0_s16_Sine_Enc_Bits) &= ~SW_SINE_READ_POS_ENA_MASK;

         InitSciC();    // init uart

         *(int*)FPGA_MFB_ENCODER_SEL_REG_ADD = 0x04; // Hiperface

         BGVAR(s16_Hiface_Error) = 0;

         BGVAR(u16_Hiface_Init_State) = HIFACE_SEND_ENC_RESET;    //go to next state
      break;

      case HIFACE_SEND_ENC_RESET:
         ScicSendByte(0x40);  // Send the encoder default address 0x40
         ScicSendByte(0x53);  // Send the reset command
         ScicSendByte(0x13);  // Send the checksum value (EXOR)

         s32_Hiface_Timer = Cntr_1mS;

         BGVAR(u16_Hiface_Init_State) = HIFACE_WAIT_RESET_1_STATE;
      break;

      case HIFACE_WAIT_RESET_1_STATE:  //  Wait for reset applied
         if (PassedTimeMS(200L, s32_Hiface_Timer))
         {
            BGVAR(u32_Hiface_Resolution) = -1;
            BGVAR(u16_Hiface_Init_State) = HIFACE_READ_TYPE_LABEL_STATE;
         }
      break;

      case HIFACE_READ_TYPE_LABEL_STATE: // Read Type Label.
         ret_val = ReadHifaceEncType(&BGVAR(u32_Hiface_Resolution), drive);
         if (ret_val == SAL_SUCCESS)
         {
            BGVAR(u16_Hiface_Init_State) = HIFACE_WAIT_2_STATE;
            s32_Hiface_Timer = Cntr_1mS;
         }
         else if (ret_val != SAL_NOT_FINISHED)
         {
             BGVAR(u32_Hiface_Init_Error) = ((0x1000000 * ((long)BGVAR(u16_Hiface_Init_State))) | ((0x100 * ((long)ret_val))));

            BGVAR(u16_Hiface_Init_State) = HIFACE_INIT_ERROR_STATE;
         }
      break;

      case HIFACE_WAIT_2_STATE:  //  Wait again...
         if (PassedTimeMS(200L, s32_Hiface_Timer))
         {
            if (BGVAR(u32_Hiface_Resolution) > 1)
               BGVAR(u16_Hiface_Init_State) = HIFACE_READ_ABS_POS_STATE;
            else
               BGVAR(u16_Hiface_Init_State) = HIFACE_READ_ENC_DATA_STATE;
            // Left Shift of Digital Data = 16 - 5 (Sine-Signal Interpolation Level) + 2
            // (leaving Quadrature Bit at bottom of AH after shift) = 13
            VAR(AX0_u16_Sine_Match_Shift) = 13;
         }
      break;

      case HIFACE_READ_ENC_DATA_STATE: // Read Enc.
         ret_val = ReadHifaceEncData(&BGVAR(u32_Hiface_Resolution), drive);
         if (ret_val == SAL_SUCCESS)
         {
            BGVAR(u16_Hiface_Init_State) = HIFACE_WAIT_3_STATE;
            s32_Hiface_Timer = Cntr_1mS;
         }
         else if (ret_val != SAL_NOT_FINISHED)
         {
             BGVAR(u32_Hiface_Init_Error) = ((0x1000000 * ((long)BGVAR(u16_Hiface_Init_State))) | ((0x100 * ((long)ret_val))));
             BGVAR(u16_Hiface_Init_State) = HIFACE_INIT_ERROR_STATE;
         }
      break;

      case HIFACE_WAIT_3_STATE:  //  Wait again...
         if (PassedTimeMS(200L, s32_Hiface_Timer))
            BGVAR(u16_Hiface_Init_State) = HIFACE_READ_ABS_POS_STATE;
      break;

      case HIFACE_READ_ABS_POS_STATE:  //  Send the absolute position read command
         ret_val = ReadHifaceAbsPos(drive);
         if (ret_val == SAL_SUCCESS)
         {  // Initialize offsets between analog and digital vaules
            if ((VAR(AX0_s16_Sin_Cos_Negate) & 0x0002) == 2) // If Second attempt
               LVAR(AX0_s32_Sw_Sine_Abs_Pos_32) += 32; // correct reading...
            VAR(AX0_s16_Sine_Enc_Bits) |= SW_SINE_READ_POS_DONE_MASK;

            BGVAR(u16_Hiface_Init_State) = HIFACE_INIT_PFB_AND_COMM_STATE;
         }
         else if (ret_val != SAL_NOT_FINISHED)
         {
             BGVAR(u32_Hiface_Init_Error) = ((0x1000000 * ((long)BGVAR(u16_Hiface_Init_State))) | ((0x100 * ((long)ret_val))));
             BGVAR(u16_Hiface_Init_State) = HIFACE_INIT_ERROR_STATE;
         }
      break;

      case HIFACE_INIT_PFB_AND_COMM_STATE:
         // Wait till the RT completes its init
         if ((VAR(AX0_s16_Sine_Enc_Bits) & SW_SINE_READ_POS_DONE_MASK) == 0)
         {
            // Combined absolute value is stored at AX0_u32_Comm_Abs_Pos_32_Lo/Hi
            do {
               s16_temp = Cntr_3125;
               s64_temp = LLVAR(AX0_u32_Comm_Abs_Pos_32_Lo);
            } while (s16_temp != Cntr_3125);
            // To match Abs_Fdbk_Offset to the value in s64_temp, convert from 32.32 (Revolutions
            // in upper 32 bits) to Encoder Counts which is MENCRES times 2^16 interpolation.  Thus
            // multiply by u32_User_Motor_Enc_Res * 2^16 and then shift 32 to the right; cancel
            // powers of 2 to obtain multiplication by MENCRES and Shift Right by 16.
            s64_temp = s64_temp + MultS64ByFixU32ToS64(BGVAR(s64_Abs_Fdbk_Offset), BGVAR(u32_User_Motor_Enc_Res), 16);
            LVAR(AX0_s32_Pos_Fdbk_Hi) = (long)(s64_temp / (long long)LVAR(AX0_s32_Counts_Per_Rev));

            // Handle Sign extension (Hiperface has a fixed 12 bit multiturn value)
            //if (LVAR(AX0_s32_Pos_Fdbk_Hi) > 0x7FF) LVAR(AX0_s32_Pos_Fdbk_Hi) -= 0x0FFF;

            LVAR(AX0_u32_Fdbk_Accu) = (unsigned long)(s64_temp - ((long long)LVAR(AX0_s32_Pos_Fdbk_Hi) * (long long)LVAR(AX0_s32_Counts_Per_Rev)));
            LVAR(AX0_u32_Pos_Fdbk_Lo) = (unsigned long)((unsigned long long)LVAR(AX0_u32_Fdbk_Accu) << 32LL) / (long long)LVAR(AX0_s32_Counts_Per_Rev);
            VAR(AX0_s16_Abs_Fdbk_Ofst_Adj) =
            -(int)(((MultS64ByFixU32ToS64(BGVAR(s64_Abs_Fdbk_Offset), BGVAR(u32_User_Motor_Enc_Res), 16) %
            (long long)LVAR(AX0_s32_Counts_Per_Rev)) << 16) / (long long)LVAR(AX0_s32_Counts_Per_Rev));

            // Signal RT it can start updating PFB
            VAR(AX0_s16_Skip_Flags) &= ~SW_SINE_UPDATING_PFB_MASK;

            BGVAR(s64_SysNotOk) &= ~SIN_COMM_FLT_MASK;
            }

         do {
            s16_temp = Cntr_3125;
            s64_temp = LLVAR(AX0_u32_Comm_Abs_Pos_32_Lo);
         } while (s16_temp != Cntr_3125);

         pos_mismatch = (int)((s64_temp >> (11 - VAR(AX0_s16_Sw_Sine_Shr)))
                        - LVAR(AX0_s32_Sw_Sine_Abs_Pos_32));
         if ((VAR(AX0_s16_Sin_Cos_Negate) & 0x0002) == 2) // If Second attempt
            pos_mismatch = (int)((s64_temp >> (11 - VAR(AX0_s16_Sw_Sine_Shr)))
                           - LVAR(AX0_s32_Sw_Sine_Abs_Pos_32)) + 32;
         // determine the difference between the Absolute Position Read and the result of matching
         // Initial Read with ATAN calculations.
         if (pos_mismatch < 0) // a Negative Mismatch will be corrected to Positive to
         { // maintain unumbiguity
            if ((VAR(AX0_s16_Sin_Cos_Negate) & 0x0002) == 2) // If Second attempt, declare Fault
            {
                BGVAR(u32_Hiface_Init_Error) = ((0x1000000 * ((long)BGVAR(u16_Hiface_Init_State))) | ((0x100 * ((long)ret_val))));
                BGVAR(u16_Hiface_Init_State) = HIFACE_INIT_ERROR_STATE;
            }
            else
            {
               VAR(AX0_s16_Sin_Cos_Negate) |= 0x0002; // indicate second pass, and restart
               BGVAR(u16_Hiface_Init_State) = HIFACE_REVERSAL_RESET_STATE; // Initialization
            }
         }
         else // Enter MTP handling only AFTER the Mismatch was resolved...
         {
            if (((VAR(AX0_s16_Sine_Enc_Bits) & SW_SINE_READ_POS_DONE_MASK) == 0) && (BGVAR(u16_Hiface_Init_State) == HIFACE_INIT_PFB_AND_COMM_STATE)) // This is to check if reversal has been handeled already
         {
               if (BGVAR(u16_MTP_Mode)) // Init from MTP if needed
               {
                  BGVAR(s16_DisableInputs) |= MTP_READ_DIS_MASK;
                  BGVAR(u16_Init_From_MTP) = 1; // Initiate MTP init if needed
                  BGVAR(s16_DisableInputs) &= ~HIFACE_DIS_MASK; // Hiperface Init. Mask here, not at Param. Load
               }

               BGVAR(u16_Load_Params_From_Hiperface_State) = 1;
               BGVAR(u16_Hiface_Init_State) = HIFACE_IDLE_STATE;
            }
         }
      break;

      case HIFACE_INIT_ERROR_STATE:
         VAR(AX0_s16_Sin_Cos_Negate) = 0x0000; // Reset the "Attempts" Counter to allow two
                           // passes, Initializing with and without Correction if required.
      break;
   }
}


// Read a 32bit value from the specified datafield
int HifaceReadAddr(unsigned int u16_datafield, unsigned int u16_address, long* s32_response)
{
   int s16_ret_val, s16_num_of_rcvd_bytes;
   long s32_temp_value;

   STORE_EXECUTION_PARAMS_0_15

   // Construct read message
   s64_Execution_Parameter[0] = 0x004A;
   s64_Execution_Parameter[1] = ((long long)u16_datafield) & 0x00FF;
   s64_Execution_Parameter[2] = ((long long)u16_address) & 0x00FF;
   s64_Execution_Parameter[3] = 0x0004;
   s64_Execution_Parameter[4] = 0x0055;
   s16_Number_Of_Parameters = 5;

   s16_ret_val = HifaceCmdGenericSilent(&s16_num_of_rcvd_bytes);

   if (s16_ret_val == SAL_SUCCESS)   // extract the data from the hiperface response
   {
      if (s16_num_of_rcvd_bytes != 10)
      {
         s16_ret_val = FDBKTYPE_COMM_ERR;
      }
      else
      {
         s32_temp_value = u8_Scic_Rx_Data[8] & 0x00FF;
         s32_temp_value <<= 8;
         s32_temp_value |= u8_Scic_Rx_Data[7] & 0x00FF;
         s32_temp_value <<= 8;
         s32_temp_value |= u8_Scic_Rx_Data[6] & 0x00FF;
         s32_temp_value <<= 8;
         s32_temp_value |= u8_Scic_Rx_Data[5] & 0x00FF;

         *s32_response = s32_temp_value;
      }
   }

   RESTORE_EXECUTION_PARAMS_0_15

   return s16_ret_val;
}


//**********************************************************
// Function Name: SalHifaceWrite
// Description:
//   This function called in response to HIFACEWRITE command
//   The Function writes four consecutive Bytes with the 1st Byte being the LSB in the 32-Bit value.
//
// Author: A. H.
// Algorithm:
// Revisions:
//***********************************************************
int SalHifaceWrite(void)
{
   unsigned int u16_datafield = (unsigned int)s64_Execution_Parameter[0];
   unsigned int u16_address = (unsigned int)s64_Execution_Parameter[1];
   unsigned long u32_value = (unsigned long)s64_Execution_Parameter[2];
   int s16_ret_val = SAL_NOT_FINISHED, s16_num_of_rcvd_bytes;

   STORE_EXECUTION_PARAMS_0_15

   // Construct write message
   s64_Execution_Parameter[0] = 0x004B;
   s64_Execution_Parameter[1] = ((long long)u16_datafield) & 0x00FF;
   s64_Execution_Parameter[2] = ((long long)u16_address) & 0x00FF;
   s64_Execution_Parameter[3] = 0x0004LL; // Four Data Bytes
   s64_Execution_Parameter[4] = 0x0055LL; // Access Code
   s64_Execution_Parameter[5] = (long long)(u32_value & 0x00FFL);
   s64_Execution_Parameter[6] = (long long)((u32_value >> 8) & 0x00FFL);
   s64_Execution_Parameter[7] = (long long)((u32_value >> 16) & 0x00FFL);
   s64_Execution_Parameter[8] = (long long)((u32_value >> 24) & 0x00FFL);
   s16_Number_Of_Parameters = 9;

   s16_ret_val = HifaceCmdGenericSilent(&s16_num_of_rcvd_bytes);
   // Analyze the hiperface response:
   if (s16_ret_val == SAL_SUCCESS) // Generic Command completed successfully...
   {
      if (s16_num_of_rcvd_bytes != 6)
         s16_ret_val = FDBKTYPE_COMM_ERR; // If not the expected response to Store-Data Command...
   }

   RESTORE_EXECUTION_PARAMS_0_15

   return s16_ret_val;
}


// Read the specified datafield size
int HifaceGetFieldSize(unsigned int u16_datafield, unsigned int* u16_response)
{
   int s16_ret_val, s16_num_of_rcvd_bytes;
   unsigned int u16_temp_value;

   STORE_EXECUTION_PARAMS_0_1

   // Construct read message
   s64_Execution_Parameter[0] = 0x004C;
   s64_Execution_Parameter[1] = ((long long)u16_datafield) & 0x00FF;
   s16_Number_Of_Parameters = 2;

   s16_ret_val = HifaceCmdGenericSilent(&s16_num_of_rcvd_bytes);

   if (s16_ret_val == SAL_SUCCESS)   // Extract the size data from the hiperface response
   {
      if (s16_num_of_rcvd_bytes != 5)
      {
         s16_ret_val = FDBKTYPE_COMM_ERR;
      }
      else
      {
         u16_temp_value = u8_Scic_Rx_Data[3];
         u16_temp_value = ((u16_temp_value & 0x07) + 1) * 16; // size in bytes = (3LSBs + 1) * 16
         *u16_response = u16_temp_value;
      }
   }

   RESTORE_EXECUTION_PARAMS_0_1

   return s16_ret_val;
}


// Open a specified datafield with given size
int HifaceOpenDataField(unsigned int u16_datafield, unsigned int u16_size, unsigned int u16_Write_Access)
{
   int s16_ret_val, s16_num_of_rcvd_bytes;

   u16_size = (u16_size >> 4) - 1;
   STORE_EXECUTION_PARAMS_0_15

   // Construct read message
   s64_Execution_Parameter[0] = 0x004D;
   s64_Execution_Parameter[1] = ((long long)u16_datafield) & 0x00FF;
   // Set Status Word:  Enable (Bit 7), Code-Enable (Bit 3), Write-Enable (Access-Right) and Size
   s64_Execution_Parameter[2] = ((long long)(0x0088 | (u16_Write_Access << 6) | u16_size)) & 0x00FF;
   s64_Execution_Parameter[3] = 0x0055;
   s16_Number_Of_Parameters = 4;

   s16_ret_val = HifaceCmdGenericSilent(&s16_num_of_rcvd_bytes);

   RESTORE_EXECUTION_PARAMS_0_15

   return s16_ret_val;
}


//**********************************************************
// Function Name: HSaveLoadCallHIFACEREAD
// Description:call the HIFACEREAD Read/Write Sal Function
//
// The returned data from hiperface is located at u8_Scic_Rx_Data[].
// The first 2 bytes are also saved to BGVAR(u16_Hiperface_Read_Returned_Value)
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int HSaveLoadCallHIFACEREAD(unsigned int u16_command, unsigned int u16_field, unsigned int u16_address, unsigned int u16_size, unsigned int u16_data)
{
   int s16_ret_val, s16_num_of_rcvd_bytes;
   unsigned int u16_temp_value = 0;

   STORE_EXECUTION_PARAMS_0_15

   // Construct read message
   s64_Execution_Parameter[0] = ((long long)u16_command) & 0x00FF;
   s64_Execution_Parameter[1] = ((long long)u16_field) & 0x00FF;
   s64_Execution_Parameter[2] = ((long long)u16_address) & 0x00FF;
   s64_Execution_Parameter[3] = ((long long)u16_size) & 0x00FF; // Num of bytes to Raed/Write
   s64_Execution_Parameter[4] = 0x0055;
   if (0x4A == u16_command) // Read Data Command
   {
      s16_Number_Of_Parameters = 5;
   }
   else if (0x4B == u16_command) // Store Data Command
   {
      s64_Execution_Parameter[5] = ((long long)u16_data) & 0x00FF;
      s64_Execution_Parameter[6] = ((((long long)u16_data) & 0xFF00) >> 8);
      s16_Number_Of_Parameters = 7;
   }
   else
      return VALUE_IS_NOT_ALLOWED;

   s16_ret_val = HifaceCmdGenericSilent(&s16_num_of_rcvd_bytes);

   if (s16_ret_val == SAL_SUCCESS)   // extract the data from the hiperface response
   {
      if (0x0050 == (u8_Scic_Rx_Data[1] & 0x00FF)) // This returned value symbolizes an error in Hiface
      {
         if (0x4B == u16_command) s16_ret_val = SAVE_TO_FLASH_FAILED;
         else                   s16_ret_val = READ_FROM_FLASH_FAILED;
      }
      else if (0x4A == u16_command) // Read Data Command - save returned data
      {
         if (s16_num_of_rcvd_bytes != (6 + u16_size))
            s16_ret_val = FDBKTYPE_COMM_ERR;
         else
         {
            u16_temp_value |= u8_Scic_Rx_Data[6] & 0x00FF;
            u16_temp_value <<= 8;
            u16_temp_value |= u8_Scic_Rx_Data[5] & 0x00FF;

            BGVAR(u16_Hiperface_Read_Returned_Value) = u16_temp_value;
         }
      }
      else                          // Store Data Command
      {
         if (s16_num_of_rcvd_bytes != 6)
            s16_ret_val = FDBKTYPE_COMM_ERR;
      }
   }
   RESTORE_EXECUTION_PARAMS_0_15

   return s16_ret_val;
}


//**********************************************************
// Function Name: GetHiperfaceParamsField
// Description:
//          This function Looks for the designated field for params Read/Write in Hiperface.
//          Sequently each field is scanned.
//             - If a known STAMP value appears, field is found.
//             - If an Error msg returns, this field is non-opened, HSAVE will open it for params use if needed.
//             - If a new, opened, unused field is found - field will be used for params.
//
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int GetHiperfaceParamsField (void)
{
   unsigned int ret_val = 1, u16_two_scans_max;
   static int u16_field = 0;

   BGVAR(u16_Hiperface_Closed_Field_Found) = 0;
   ScicClrRxBuffer();//Clear Rx Buffer
   //for (index = 0; index<13; index++) u8_Scic_Rx_Data[index] = 0;// clear Rx Buffer

   for (u16_two_scans_max = 1; !(IS_STAMP_VALUE_WRITTEN     || // STAMP Value Scanned
                               IS_HIPERFACE_FIELD_CLEAR     || // New Opened Field Found
                               (0 == (u16_two_scans_max%3)) || // Scanned twice in a row already
                               (SAL_SUCCESS != ret_val)       ); // Another unexpected issue occured
                                       u16_field++, u16_two_scans_max++)
   {  // Scan the first 16 bytes in a field
      ret_val = HSaveLoadCallHIFACEREAD(HIFACE_READ_DATA_COMMAND, u16_field, 0, 16, 0);
   }

   BGVAR(u16_Hiperface_Params_Field) = u16_field - 1;

   /**   What Caused The Exist Of The FOR Loop ??? **/

   // Option#1: Arrived an opened new field --> let it be the params' field, store STAMP value of DISREGARD_PARAMETERS inside.
   if (IS_HIPERFACE_FIELD_CLEAR)
   {
      ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_STAMP_OFFSET,2, HIFACE_STAMP_DISREGARD_PARAMETERS);
      u16_field = 0;
   }
   // Option#2: scan returned READ_FROM_FLASH_FAILED (reached non-opened field) --> raise Flag and return the value (HSAVE will open that field if desired).
   else if (READ_FROM_FLASH_FAILED == ret_val)
   {
      ret_val = SAL_SUCCESS;
      BGVAR(u16_Hiperface_Closed_Field_Found) = 1;
      u16_field = 0;
   }
   // Option#3: A Known STAMP Value was scanned               --> nothing to be done, field is found.
   // Option#4: scan returned diffrent value than SAL_SUCCESS --> nothing to be done, return the value;
   else if ((SAL_SUCCESS != ret_val) || IS_STAMP_VALUE_WRITTEN)
   {
      u16_field = 0;
   }
   // Option#5: Two scans were made already --> return SAL_NOT_FINISHED to avoid Watch-Dog, continue scaning next BG cycle.
   else if (0 == u16_two_scans_max%3)
   {
      ret_val = SAL_NOT_FINISHED;
   }

   if (100 < u16_field)
   {
      ret_val = FLASH_INVALID;
      u16_field = 0;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SalHSaveToHifaceFlash
// Description:
//          This function is called in response to the HSAVE command when feedback assigned as Hiperface.
//          -- Params writing to Hiperface's EEPROM in 0xb field.
//          number of params:
//          0 - write parameters (MPHASE, PFB_offset, checksum) to Hiperface's EEPROM, and mark the 'stamp' with READY_TO_LOAD value
//          1 - mark the 'stamp' with DONT_LOAD value
//
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int SalHSaveToHifaceFlash (int drive)
{
   int ret_val = 0;
   unsigned int checksum = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   switch (BGVAR(u16_HSave_Hiface_State))
   {
      case 0: // first run, check legit param
         if (SW_SINE_FDBK != BGVAR(u16_FdbkType))
            return NOT_SUPPORTED_ON_FEEDBACK;
         if ((s16_Number_Of_Parameters == 1) && (s64_Execution_Parameter[0] != 1LL))
            return VALUE_IS_NOT_ALLOWED;
         BGVAR(u16_HSave_Hiface_Num_Of_Params) = s16_Number_Of_Parameters;
         BGVAR(u16_HSave_Hiface_State)++;
      break;

      case 1://if a the field found is a non-opened one - open it
         if (1 == BGVAR(u16_Hiperface_Closed_Field_Found))
         {
            ret_val = HifaceOpenDataField(BGVAR(u16_Hiperface_Params_Field), 16, 1);
            if (SAL_SUCCESS != ret_val)
            {
               BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State)) | (0x100000000 * (long long)ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
               BGVAR(u16_HSave_Hiface_State) = 0;
               return ret_val;
            }
            else
            {
               BGVAR(u16_Hiperface_Closed_Field_Found) = 0;
               BGVAR(s32_Hiface_time_stamp) = Cntr_1mS;
               BGVAR(u16_HSave_Hiface_State)++;
            }
         }
         else
         {
            BGVAR(u16_HSave_Hiface_State) = 3;
         }
      break;

      case 2://wait 200ms
         if (PassedTimeMS(200L, s32_Hiface_time_stamp))
            BGVAR(u16_HSave_Hiface_State)++;
      break;

      case 3://STAMP write
         if (1 == BGVAR(u16_HSave_Hiface_Num_Of_Params)) // reset STAMP in Hiface so parameters will not be loaded to drive on Load()
         {
            ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field),  HIFACE_STAMP_OFFSET, 2, HIFACE_STAMP_DISREGARD_PARAMETERS);
            if (SAL_SUCCESS != ret_val)
               BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State)) | (0x100000000 * (long long)ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            else
               BGVAR(u64_HSave_Hiface_Error) = 0;
            BGVAR(u16_HSave_Hiface_State) = 0;
            return ret_val;
         }
         else //No param - save all parameters to Hiperface
         {
             //write into Hiperface's STAMP a Value of 'starting-to-copy-parameters'
            ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_STAMP_OFFSET, 2, HIFACE_STAMP_COPYING_PARAMETERS);
            if (SAL_NOT_FINISHED == ret_val) return ret_val;
            if (SAL_SUCCESS != ret_val)
            {
               BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State)) | (0x100000000 * (long long)ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
               BGVAR(u16_HSave_Hiface_State) = 0;
               return ret_val;
            }
            else
               BGVAR(u16_HSave_Hiface_State)++;
         }
      break;

      case 4://MPHASE write
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_MPHASE_OFFSET, 2, (unsigned int)VAR(AX0_s16_Electrical_Phase_Offset));
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State)) | (0x100000000 * (long long)ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_HSave_Hiface_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_Hiface_State)++;
         break;

       case 5://PFBoffset write #1/4 (MSB bits --> 48-63)
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_FIRST_PFB_OFFSET, 2, ((unsigned int)((LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) >> 16) & 0xffff)));
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State)) | (0x100000000 * (long long)ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_HSave_Hiface_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_Hiface_State)++;
      break;

      case 6://PFBoffset write #2/4 (bits --> 32-47)
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_SECOND_PFB_OFFSET, 2, ((unsigned int)(LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) & 0xffff)));
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State)) | (0x100000000 * (long long)ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_HSave_Hiface_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_Hiface_State)++;
      break;

      case 7://PFBoffset write #3/4 (bits --> 16-31)
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_THIRD_PFB_OFFSET, 2, ((unsigned int)((LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) >> 16) & 0xffff)));
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State))|(0x100000000*(long long)ret_val)|(0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_HSave_Hiface_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_Hiface_State)++;
      break;

      case 8://PFBoffset write #4/4 (LSB bits --> 0-15)
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field),  HIFACE_FOURTH_PFB_OFFSET,2, ((unsigned int)(LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) & 0xffff)));
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State))|(0x100000000*(long long)ret_val)|(0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_HSave_Hiface_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_Hiface_State)++;
      break;

      case 9://Checksum Write
         checksum = (unsigned int)(HIFACE_STAMP_PARAMS_READY_FOR_LOAD)
                  + (unsigned int)VAR(AX0_s16_Electrical_Phase_Offset)
                  + ((unsigned int)((LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) >> 16) & 0xffff))
                  + ((unsigned int)(LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) & 0xffff))
                  + ((unsigned int)((LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) >> 16) & 0xffff))
                  + ((unsigned int)(LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) & 0xffff));
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_CHECKSUM_OFFSET,2, checksum);
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State)) | (0x100000000*(long long)ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_HSave_Hiface_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_Hiface_State)++;
      break;

      case 10:// STAMP Write - to state parameters writing is done
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_STORE_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_STAMP_OFFSET, 2, HIFACE_STAMP_PARAMS_READY_FOR_LOAD);
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Hiface_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_Hiface_State)) | (0x100000000*(long long)ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_HSave_Hiface_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u64_HSave_Hiface_Error) = 0;
            BGVAR(u16_HSave_Hiface_State) = 0;
            return SAL_SUCCESS;
         }
   } // switch

   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: LoadParamsFromHiperface
// Description:
//          Called From Background after Hiperface Handler Init is Done.
//          -- Loads saved parameters from Hiperface's EEPROM to ram. --
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int LoadParamsFromHiperface(int drive)
{
   int ret_val = 0;
   unsigned int checksum = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   switch (BGVAR(u16_Load_Params_From_Hiperface_State))
   {
      case 0:
         return SAL_SUCCESS;

      case 1:// Find/Create designated field for params in Hiperface's EEPROM
         ret_val = GetHiperfaceParamsField();
         if (1 == BGVAR(u16_Hiperface_Closed_Field_Found)) //no need to progress since no data is on the eeprom.
         {
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            BGVAR(s16_DisableInputs) &= ~HIFACE_DIS_MASK;
            return ret_val;
         }
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State)) | (0x100000000 * (long long)ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Load_Params_From_Hiperface_State)++;
            BGVAR(s32_Hiface_time_stamp) = Cntr_1mS;
         }
      break;

      case 2://check Stamp to see if params should be loaded
         if (!PassedTimeMS(200L,s32_Hiface_time_stamp)) break;
         s32_Hiface_time_stamp = 0;
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_READ_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_STAMP_OFFSET,2, 0);
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State)) | (0x100000000 * (long long)ret_val) |(0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            return ret_val;
         }
         BGVAR(u16_Hiperface_Stamp_Value_Mismatch) = 0;
         if ( HIFACE_STAMP_PARAMS_READY_FOR_LOAD == BGVAR(u16_Hiperface_Read_Returned_Value))
         {
            BGVAR(u16_Load_Params_From_Hiperface_State)++;
         }
         else if (HIFACE_STAMP_DISREGARD_PARAMETERS == BGVAR(u16_Hiperface_Read_Returned_Value))
         {
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            BGVAR(s16_DisableInputs) &= ~HIFACE_DIS_MASK;
            return SAL_SUCCESS;
         }
         else if (HIFACE_STAMP_COPYING_PARAMETERS == BGVAR(u16_Hiperface_Read_Returned_Value))
         {
            if (5 < BGVAR(u16_Hiperface_StampRead_Retry))
            {
               BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
               return HIFACE_ILLEGAL_STAMP;
            }
            BGVAR(u16_Hiperface_StampRead_Retry)++;
         }
         else
         {
            BGVAR(u16_Hiperface_Stamp_Value_Mismatch) = 1;// continue, use this later Vs Checksum, to decide if this is an error or first encoder use
            BGVAR(u16_Load_Params_From_Hiperface_State)++;
         }
      break;

      case 3://MPHASE read
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_READ_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_MPHASE_OFFSET,2, 0);
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State)) | (0x100000000 * ret_val) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Electrical_Phase_Offset_User) = BGVAR(u16_Hiperface_Temp_Electrical_Phase_Offset) = BGVAR(u16_Hiperface_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Hiperface_State)++;
         }
      break;

      case 4://PFBoffset read #1/4 (MSB bits --> 48-63)
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_READ_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_FIRST_PFB_OFFSET,2, 0);
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State)) | (0x100000000 * (long long)((LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) >> 16) & 0xffff)) |(0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Hiface_Temp_First_Pos_Fdbk_Offset) = (long)BGVAR(u16_Hiperface_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Hiperface_State)++;
         }
      break;

      case 5://PFBoffset read #2/4 (bits --> 32-47)
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_READ_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_SECOND_PFB_OFFSET,2, 0);
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State))|(0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Hiface_Temp_Second_Pos_Fdbk_Offset) = (long)BGVAR(u16_Hiperface_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Hiperface_State)++;
         }
      break;

      case 6://PFBoffset read #3/4 (bits --> 16-31)
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_READ_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_THIRD_PFB_OFFSET,2, 0);
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State))|(0x100000000*(long long)((LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) >> 16) & 0xffff))|(0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Hiface_Temp_Third_Pos_Fdbk_Offset) = (long)BGVAR(u16_Hiperface_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Hiperface_State)++;
         }
      break;

      case 7://PFBoffset read #4/4 (LSB bits --> 0-15)
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_READ_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_FOURTH_PFB_OFFSET,2, 0);
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State))|(0x100000000*(long long)ret_val)|(0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Hiface_Temp_Fourth_Pos_Fdbk_Offset) = (long)BGVAR(u16_Hiperface_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Hiperface_State)++;
         }
      break;

      case 8://Checksum test
         checksum = BGVAR(u16_Hiperface_Temp_Electrical_Phase_Offset)
                  + BGVAR(u16_Hiface_Temp_First_Pos_Fdbk_Offset)
                  + BGVAR(u16_Hiface_Temp_Second_Pos_Fdbk_Offset)
                  + BGVAR(u16_Hiface_Temp_Third_Pos_Fdbk_Offset)
                  + BGVAR(u16_Hiface_Temp_Fourth_Pos_Fdbk_Offset)
                  + HIFACE_STAMP_PARAMS_READY_FOR_LOAD;
         ret_val = HSaveLoadCallHIFACEREAD(HIFACE_READ_DATA_COMMAND, BGVAR(u16_Hiperface_Params_Field), HIFACE_CHECKSUM_OFFSET,2, 0);
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State)) | (0x100000000*(long long)checksum) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
            BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
            return ret_val;
         }
         else
         {
            if (checksum == BGVAR(u16_Hiperface_Read_Returned_Value))
            {
               if (BGVAR(u16_Hiperface_Stamp_Value_Mismatch))
               {  //for the case of STAMP wrong save/load (checksum passed hence we know the stamp is a problem)
                  BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State)) | (0x100000000*(long long)checksum) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
                  BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
                  return HIFACE_ILLEGAL_STAMP;
               }
               else
               { // copy loaded parameters
                  VAR(AX0_s16_Electrical_Phase_Offset) = (int)BGVAR(u16_Hiperface_Temp_Electrical_Phase_Offset); //mphase

                  LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) = (long)BGVAR(u16_Hiface_Temp_First_Pos_Fdbk_Offset);//PFBoffset copy #1/4 (MSB bits --> 48-63)
                  LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) <<= 16;

                  LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) |= (long)BGVAR(u16_Hiface_Temp_Second_Pos_Fdbk_Offset);//PFBoffset copy #2/4 (bits --> 32-47)

                  LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) = (long)BGVAR(u16_Hiface_Temp_Third_Pos_Fdbk_Offset);//PFBoffset copy #3/4 (bits --> 16-31)
                  LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) <<= 16;

                  LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) |= (long)BGVAR(u16_Hiface_Temp_Fourth_Pos_Fdbk_Offset);//PFBoffset copy #4/4 (LSB bits --> 0-15)
                  BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
                  BGVAR(u64_Hiperface_Load_Error) = 0;
                  BGVAR(s16_DisableInputs) &= ~HIFACE_DIS_MASK;
                  return SAL_SUCCESS;
               }
            }
            else //CheckSum Failed
            {
               BGVAR(u16_Load_Params_From_Hiperface_State) = 0;
               if (BGVAR(u16_Hiperface_Stamp_Value_Mismatch))
               {  //case of hiperface with no params saved (checksum failed hence we know STAMP was never written)
                  BGVAR(u64_Hiperface_Load_Error) = 0;
                  BGVAR(s16_DisableInputs) &= ~HIFACE_DIS_MASK;
                  return SAL_SUCCESS;
               }
               else
               {  //CheckSum Failed, don't copy params
                  BGVAR(u64_Hiperface_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Hiperface_State)) | (0x100000000*(long long)checksum) | (0x10000*(long long)(long long)((u8_Scic_Rx_Data[3] << 8) | u8_Scic_Rx_Data[2])) | (long long)((u8_Scic_Rx_Data[1] << 8) | u8_Scic_Rx_Data[0]));
                  return CHECKSUM_ERROR;
               }
            }
         }
   }// switch

   return SAL_NOT_FINISHED;
}

