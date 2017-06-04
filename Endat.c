#include "DSP2834x_Device.h"

#include "Design.def"
#include "Endat.def"
#include "Err_Hndl.def"
#include "FltCntrl.def"
#include "FPGA.def"
#include "ModCntrl.def"

#include "drive.var"
#include "Endat.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Hiface.var"
#include "Motor.var"
#include "MotorSetup.var"
#include "MultiAxis.def"
#include "Ser_Comm.var"

#include "Prototypes.pro"


//**********************************************************
// Function Name: SalEnDatRequest
// Description:  This function is called to send command to EnDat Encoder, using ENDATREQ
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalEnDatRequest(void)
{
   SendEndatRequest((unsigned long)s64_Execution_Parameter[0],
                    (unsigned int)s64_Execution_Parameter[1],
                    (unsigned int)s64_Execution_Parameter[2], 0, 0);
   return SAL_SUCCESS;
}


long s32_abs_pos_hi = 0L;
unsigned long u32_abs_pos_lo = 0L;
//**********************************************************
// Function Name: SendEndatPosition
// Description:  This function is called to send command to EnDat Encoder, using ENDATPOS
//
// Author: A. H.
// Algorithm:  Relying on detected Parameters to send Position Data Request and receive Data
// Revisions:
//**********************************************************
int SalEnDatPosition(drive)
{
   return ReadEndatAbsPosition(drive);
}


//**********************************************************
// Function Name: SendEndatReset
// Description:  This function is called to send reset the EnDat Encoder, ENDATRST
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalEnDatReset(void)
{
   SendEndatRequest(0x0055L, 8, 22, 0, 0);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SendEndatRequest
// Description:
//    This function is called to send command to ENDAT
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void SendEndatRequest(unsigned long cmnd, int cmnd_len, int rsp_len, int drive, int enc_source)
{
   unsigned int i;
   unsigned long cmnd_temp = 0;
   drive += 0;

   for(i = 0; i < 32; i++) // Zero all Receive Registers
      *(unsigned int *)(FPGA_MFB_RX_BUFFER_ADD + 0x0080 * enc_source + i) = 0;

   for(i = 0; i < 32; i++)
   { // Reverse Bit-Order of Command Data, to match Transmit Registers (LSB sent first)
      cmnd_temp = (cmnd_temp << 1) | (cmnd & 0x0001);
      cmnd >>= 1;
   }

   *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = cmnd_len;
   *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = rsp_len;

   for (i = 0; i <= cmnd_len / 16; i++)
      *(unsigned int *)((FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) + i) = ((cmnd_temp >> (16 * i)) & 0xFFFF);

   *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0; // Toggle Send Bit.
   *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;
}


//**********************************************************
// Function Name: SendEndatComm
// Description:
//    This function is called to send command to ENDAT
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void SendEndatComm(unsigned long cmnd, int cmnd_len, int rsp_len, int drive, int enc_source)
{
   unsigned int i;
   drive += 0;

   *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = cmnd_len;

   for (i = 0; i < cmnd_len / 16; i++)
      *(unsigned int *)((FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) + i) = ((cmnd >> (16 * i)) & 0xFFFF);

   *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = rsp_len;

   *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0; // Toggle Send Bit.
   *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;
}


//**********************************************************
// Function Name: MakeCrcNorm
// Description:
//    This function is called to calculate CRC of regular coommand
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
unsigned int MakeCrcNorm(unsigned int param8, unsigned int param16)
{
   unsigned long temp, code = 0;           // Data bit array
   unsigned int ex, i, crc = 0x01F;        // CRC Code Initial Value

   temp = ((unsigned long)param8 << 16) | param16;

   for (i = 0; i < 24; i++)
      code = (code << 1) | ((temp & (1L << i)) >> i);

   for (i = 0; i < 24; i++)   //Calculate the CRC
   {
      ex = (((code ^ crc) & 0x01) ? 0x01A : 0); // Generate EXOR Vector
      crc = (((crc >> 1) & 0x0F) ^ ex);
      code >>= 1;
   }

   crc ^= 0x01F ; // Invert CRC

   return crc;
}


//**********************************************************
// Function Name: MakeCrcPos
// Description:
//    This function is called to calculate CRC of Position Data (including Alarms)
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
unsigned int MakeCrcPos(unsigned int bits, unsigned long long data)
{
   unsigned long long code;         // Data bit array
   unsigned int ex, i, crc = 0x1F;  // CRC Code Initial Value

   code = data;

   for (i = 0; i < bits; i++)   //Calculate the CR
   {
      ex = (((code ^ crc) & 0x01) ? 0x01A : 0); // Generate EXOR Vector
      crc = (((crc >> 1) & 0x0F) ^ ex);
      code >>= 1;
   }

   crc ^= 0x01F ; // Invert CRC
   return crc;
}


int SalEnDatCRCCommand(void) // Use for Debugging CRC Calculations
{
   unsigned long long pos_lo, pos_hi;
   unsigned int ex, i, bits, crc;

   bits = (unsigned int)s64_Execution_Parameter[0];
   pos_lo = (unsigned long long)s64_Execution_Parameter[1];
   pos_hi = (unsigned long long)s64_Execution_Parameter[2];
   crc = 0x1F * (unsigned int)s64_Execution_Parameter[3];

   if (bits > 64)
   {
      for (i = 0; i < 64; i++)   // Calculate the CRC
      {
         ex = ((crc ^ (pos_lo >> (63 - i))) & 0x01) * 0x01A; // Generate EXOR Vector
         crc = ((crc >> 1) & 0x0F) ^ ex;
      }
      for (i = 0; i < (bits - 64); i++)   // Calculate the CRC
      {
         ex = ((crc ^ (pos_hi >> (63 - i))) & 0x01) * 0x01A; // Generate EXOR Vector
         crc = ((crc >> 1) & 0x0F) ^ ex;
      }
   }
   else
   {
      for (i = 0; i < bits; i++)   // Calculate the CRC
      {
         ex = ((crc ^ (pos_lo >> (63 - i))) & 0x01) * 0x01A; // Generate EXOR Vector
         crc = ((crc >> 1) & 0x0F) ^ ex;
      }
   }

   PrintUnsignedInteger(crc);
   PrintCrLf();
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SendEndatParam
// Description:
//    This function is called to send parameter to ENDAT
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************/
int SendEndatParam(int mode, int addr, int data, unsigned int* param1, unsigned int* param2, int drive, int enc_source)
{
   unsigned int temp_crc, param8, param16, calc_crc = 0, i;
   unsigned long rcvd_data, temp;

   switch (s16_Endat_Send_Param_State)
   {
      case 0:
         s32_Endat_Timer = Cntr_1mS;

         if (PassedTimeMS(10L, s32_Endat_Timer)) return ENDAT_READY_ERROR;

         temp = ((mode << 8) | (addr & 0xFF));
         temp = (temp << 16) | (unsigned int)data | 0x80000000; // Adding preceding '10' Start Bits

         SendEndatRequest(temp, 32, 32, drive, enc_source);  //29 = 8bit of addr + 16bit of data + 5bit of CRC

         s32_Endat_Timer = Cntr_1mS;
         s16_Endat_Send_Param_State++;
      break;

      case 1:
         if ( (PassedTimeMS(100L, s32_Endat_Timer) == 0)                                                &&
              ((*(unsigned int *)(FPGA_RX_DATA_READY_REG_ADD + 0x0540 * enc_source) & 0x0002) != 0x0002)  )
            return 0;
         if ((*(unsigned int *)(FPGA_RX_DATA_READY_REG_ADD + 0x0540 * enc_source) & 0x0002) != 0x0002) // Response timeout
         {
            s16_Endat_Send_Param_State = 0;
            return COMMUNICATION_ERROR;
         }

         u32_Fpga_Rx_Data = (*(unsigned int *)(FPGA_MFB_RX_BUFFER_ADD + 0x0080 * enc_source)) | ((unsigned long)(*(unsigned int *)((FPGA_MFB_RX_BUFFER_ADD + 0x0080 * enc_source) + 1)) << 16);

         for(i = 0; i < 32; i++)
            temp = (temp << 1) | ((u32_Fpga_Rx_Data & (1L << i)) >> i);

         u16_Fpga_Rx_Crc =  (unsigned int)((temp >> 2) & 0x000000000000001F);  //extract CRC
         rcvd_data = temp >> 7;

         param16 = (unsigned int)(rcvd_data & 0x0FFFF);         //extract param16

         param8 = (rcvd_data >> 16) & 0x00FF;         //extract param8

         temp_crc = MakeCrcNorm(param8, param16);         //calculate CRC
         for (i = 0; i < 5; i++) // Flip calculated CRC to match order of received CRC Bits
            calc_crc = (calc_crc << 1) | ((temp_crc & (1 << i)) >> i);

         s16_Endat_Send_Param_State = 0;

         //check if recieved crc is equal to calculated crc
         if (u16_Fpga_Rx_Crc != calc_crc) return ENDAT_CRC_ERROR;

         *param1 = (unsigned int)param8;          //store parameter in return buffer
         *param2 = (unsigned int)param16;         //store parameter in return buffer
         return SAL_SUCCESS;
   }

   return 0;
}


//**********************************************************
// Function Name: ReadEndatAbsPosition
// Description:
//    This function is called to read abs pos from ENDAT
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int ReadEndatAbsPosition(int drive)
{
   unsigned long pos_cmd;
   unsigned long long rsp_temp = 0LL, mask;
   unsigned int rsp_len, i;
   REFERENCE_TO_DRIVE;

   switch (s16_Endat_Read_Pos_State)
   {
      case 0:
         if (VAR(AX0_u16_ED_Alarm_Bits) == 1) // EnDat 2.1
            pos_cmd = ((unsigned long)(0x80 | READ_ABS_POS_CMND2_1) << 24); // Include Atart Bit, no Addr or Data
         else if (VAR(AX0_u16_ED_Alarm_Bits) == 2) // EnDat 2.2
            pos_cmd = ((unsigned long)(0x80 | READ_ABS_POS_CMND2_2) << 24); // Include Atart Bit, no Addr or Data
         else
            return VALUE_OUT_OF_RANGE;

         rsp_len = 1 + VAR(AX0_u16_ED_Alarm_Bits) + VAR(AX0_u16_ED_In_Turn_Bits) + VAR(AX0_u16_ED_Turns_Bits) + 5;
         // Start Bit + Alarm Bit(s) + Bits per rev + Mlti_Turn Bits + CRC Bits
         SendEndatRequest(pos_cmd, 8, rsp_len, 0, 0);
         VAR(AX0_s16_Sine_Enc_Bits) |= SW_SINE_READ_POS_ENA_MASK;
         s16_Endat_Read_Pos_State++;
         s32_Endat_Timer = Cntr_1mS;
      break;

      case 1: // Set minimum Waiting Time 1mSec, to ensure toggling of the Data Ready Bit
         if ( (PassedTimeMS(1L, s32_Endat_Timer))                           &&
              ((*(unsigned int *)FPGA_RX_DATA_READY_REG_ADD & 0x02) == 0x02)  )
         {
            rsp_len = 1 + VAR(AX0_u16_ED_Alarm_Bits) + VAR(AX0_u16_ED_In_Turn_Bits) + VAR(AX0_u16_ED_Turns_Bits) + 5;
            for (i = 0; i <= (rsp_len / 16); i++)
               rsp_temp |= ((long long)(*(unsigned int *)(FPGA_MFB_RX_BUFFER_ADD + i)) << (16 * i));
            rsp_temp >>= 1; // Remove Start Bit from Response
            mask = (1LL << (VAR(AX0_u16_ED_In_Turn_Bits) + VAR(AX0_u16_ED_Turns_Bits))) - 1; // Generate Mask to isolate Position Data
            LVAR(AX0_s32_Sw_Sine_Abs_Pos_32) = (unsigned long)(rsp_temp >> VAR(AX0_u16_ED_Alarm_Bits)) & mask;
            mask = (1LL << VAR(AX0_u16_ED_In_Turn_Bits)) - 1; // Generate Mask to isolate In-Turn Data
            BGVAR(u32_ED_Abs_Pos_Lo) = (unsigned long)((rsp_temp >> VAR(AX0_u16_ED_Alarm_Bits)) & mask);
            mask = (1LL << VAR(AX0_u16_ED_Turns_Bits)) - 1; // Mask to isolate Turns Data
            BGVAR(s32_ED_Abs_Pos_Hi) =(long)((rsp_temp >> (VAR(AX0_u16_ED_Alarm_Bits) + VAR(AX0_u16_ED_In_Turn_Bits))) & mask);
            mask = (1LL << (VAR(AX0_u16_ED_In_Turn_Bits) + VAR(AX0_u16_ED_Turns_Bits) + VAR(AX0_u16_ED_Alarm_Bits))) - 1; // Generate Mask to isolate Data
            // from Received CRC to calculate CRC locally
            u16_Endat_Rcv_Crc = (unsigned int)(rsp_temp >> (VAR(AX0_u16_ED_In_Turn_Bits) + VAR(AX0_u16_ED_Turns_Bits) + VAR(AX0_u16_ED_Alarm_Bits))) & 0x1F;
            u16_Endat_Calc_Crc = MakeCrcPos((VAR(AX0_u16_ED_In_Turn_Bits) + VAR(AX0_u16_ED_Turns_Bits) + VAR(AX0_u16_ED_Alarm_Bits)), (rsp_temp & mask));
            s16_Endat_Read_Pos_State = 0;
            if (u16_Endat_Rcv_Crc == u16_Endat_Calc_Crc)
               return SAL_SUCCESS;
            else
               return ENDAT_CRC_ERROR;
         }
         else
         {
            if (PassedTimeMS(30L, s32_Endat_Timer)) // Allow 30mSec for Time-Out
               return COMMUNICATION_ERROR;
         }
      break;
   }

   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: SalWriteEndatMemoryWord
// Description:
//          This function is called in response to the EMEMW command (Write function).
//
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteEndatMemoryWord (int drive)
{
   REFERENCE_TO_DRIVE;

   switch (BGVAR(s16_Endat_Write_Value_State))
   {
      case -1:
         BGVAR(s16_Endat_Write_Value_State) = 0;
         return (COMMUNICATION_ERROR);

      case  0:
         if ( (ENDAT2X_COMM_FDBK != BGVAR(u16_FdbkType)) &&
              ( (SW_SINE_FDBK != BGVAR(u16_FdbkType)) ||
                (VAR(AX0_s16_Motor_Enc_Type) != 9)      )  )
           return NOT_SUPPORTED_ON_FEEDBACK;
         if ( ( (0x0A0 < s64_Execution_Parameter[0]) &&
                (0x0BA > s64_Execution_Parameter[0]) &&
                (s64_Execution_Parameter[1] >= 0)    &&
                (64 > s64_Execution_Parameter[1])      ) ||
              ( (0x0A8 < s64_Execution_Parameter[0]) &&
                (0x0B0 > s64_Execution_Parameter[0]) &&
                (s64_Execution_Parameter[1] >= 0)    &&
                (255 > s64_Execution_Parameter[1])     )   )
         {
            BGVAR(u16_Endat_MRS_Adr) = (int)s64_Execution_Parameter[0];
            BGVAR(s16_Endat_ReadWriteTo_Offset_Adr) = (int)s64_Execution_Parameter[1];
            BGVAR(u16_Endat_Value_To_Write) = (int)s64_Execution_Parameter[2];
         }
         else
         {
           return VALUE_IS_NOT_ALLOWED;
         }

         if ( (0xB9 == BGVAR(u16_Endat_MRS_Adr)) || (0xA1 == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xA3 == BGVAR(u16_Endat_MRS_Adr)) || (0xA5 == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xA7 == BGVAR(u16_Endat_MRS_Adr)) || (0xA9 == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xAB == BGVAR(u16_Endat_MRS_Adr)) || (0xAF == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xB1 == BGVAR(u16_Endat_MRS_Adr)) || (0xB3 == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xB5 == BGVAR(u16_Endat_MRS_Adr)) || (0xB7 == BGVAR(u16_Endat_MRS_Adr))   )
         { //verify legit params
            s16_Endat_Send_Param_State = 0;
            BGVAR(s16_Endat_ReadWrite_RetryAttampts) = 0;
            BGVAR(s16_Endat_Write_Value_State)++;
         }
         else
         {
           return VALUE_IS_NOT_ALLOWED;
         }
      break;

      case 1:
      case 2:
         EndatWriteValue(0);
      break;

      case 3:
         BGVAR(s16_Endat_Write_Value_State) = 0;
         if (u16_Endat_Should_Print_EMEMW)
         {
            PrintDecInAsciiHex(BGVAR(u16_Dummy_Ptr2), 4);
            PrintCrLf();
         }
         return (SAL_SUCCESS);
   }
   return (SAL_NOT_FINISHED);
}


//**********************************************************
// Function Name: EndatWriteValue
// Description:
// Write specific value to Endat.
// Target page defined by u16_Endat_MRS_Adr, Target offset defined by s16_Endat_ReadWriteTo_Offset_Adr,
// desired value-to-write defined by u16_Endat_Value_To_Write
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
void EndatWriteValue (int drive)
{
   int ret_val = 0;

   switch (BGVAR(s16_Endat_Write_Value_State))
   {
      case 0:   //no write value requested
      break;

      case 1:
         // Memory Range Select Enc. Mfr. Params, 1st group;
         VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~(COMM_FDBK_READY_MASK);
         ret_val = SendEndatParam(MRS_CODE_CMND, BGVAR(u16_Endat_MRS_Adr), 0x00, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, 0);
         if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == BGVAR(u16_Endat_MRS_Adr)))
         {
            BGVAR(s16_Endat_ReadWrite_RetryAttampts) = 0;
            BGVAR(u64_Endat_ReadWrite_Error) = 0;
            BGVAR(s16_Endat_Write_Value_State)++;
            s32_Endat_Timer = Cntr_1mS;
         }
         else if (RETRY_NUMBER > BGVAR(s16_Endat_ReadWrite_RetryAttampts))
         {
            BGVAR(u64_Endat_ReadWrite_Error) = (0x100000000000000|(0x1000000000000 * (long long)BGVAR(s16_Endat_Write_Value_State))|(0x10000000000 * (long long)BGVAR(s16_Endat_ReadWrite_RetryAttampts)) | (0x100000000 *(long long) ret_val)|(10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(s16_Endat_ReadWrite_RetryAttampts)++;
         }
         else
         {
            BGVAR(u64_Endat_ReadWrite_Error) = (0x100000000000000|(0x1000000000000 * (long long)BGVAR(s16_Endat_Write_Value_State))|(0x10000000000 * (long long)BGVAR(s16_Endat_ReadWrite_RetryAttampts)) | (0x100000000 *(long long) ret_val)|(10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(s16_Endat_Write_Value_State) = -1; //TimeOut
         }
      break;

      case 2:
         if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...

         ret_val = SendEndatParam(WRITE_CODE_CMND, BGVAR(s16_Endat_ReadWriteTo_Offset_Adr), BGVAR(u16_Endat_Value_To_Write), &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, 0);
         if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == BGVAR(s16_Endat_ReadWriteTo_Offset_Adr)) && (BGVAR(u16_Dummy_Ptr2) == BGVAR(u16_Endat_Value_To_Write)))
         {
            BGVAR(s16_Endat_ReadWrite_RetryAttampts) = 0;
            BGVAR(u64_Endat_ReadWrite_Error) = 0;
            BGVAR(s16_Endat_Send_Param_State) = 0;
            s32_Endat_Timer = Cntr_1mS;
            VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_READY_MASK;
            BGVAR(s16_Endat_Write_Value_State)++;
         }
         else if (RETRY_NUMBER > BGVAR(s16_Endat_ReadWrite_RetryAttampts))
         {
            BGVAR(u64_Endat_ReadWrite_Error) = (0x100000000000000|(0x1000000000000 * (long long)BGVAR(s16_Endat_Write_Value_State))|(0x10000000000 * (long long)BGVAR(s16_Endat_ReadWrite_RetryAttampts)) | (0x100000000 *(long long) ret_val)|(10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(s16_Endat_ReadWrite_RetryAttampts)++;
         }
         else
         {
            BGVAR(u64_Endat_ReadWrite_Error) = (0x100000000000000|(0x1000000000000 * (long long)BGVAR(s16_Endat_Write_Value_State))|(0x10000000000 * (long long)BGVAR(s16_Endat_ReadWrite_RetryAttampts)) | (0x100000000 *(long long) ret_val)|(10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(s16_Endat_Write_Value_State) = -1; //TimeOut
         }
      break;
   }
}


//**********************************************************
// Function Name: ReadEndatMemoryWord
// Description:
//          This function is called in response to the EMEMW command (Read function)
//
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int ReadEndatMemoryWord(int drive)
{
   REFERENCE_TO_DRIVE;

   switch (BGVAR(s16_Endat_Read_Value_State))
   {
      case -1:
         BGVAR(s16_Endat_Read_Value_State) = 0;
         return (COMMUNICATION_ERROR);
      case  0:
         if ( (ENDAT2X_COMM_FDBK != BGVAR(u16_FdbkType)) &&
              ( (SW_SINE_FDBK != BGVAR(u16_FdbkType)) ||
                (VAR(AX0_s16_Motor_Enc_Type) != 9)      )  )
           return NOT_SUPPORTED_ON_FEEDBACK;
         if ( ( (0x0A0 < s64_Execution_Parameter[0]) &&
                (0x0BA > s64_Execution_Parameter[0]) &&
                (s64_Execution_Parameter[1] >= 0)    &&
                (64 > s64_Execution_Parameter[1])      ) ||
              ( (0x0A8 < s64_Execution_Parameter[0]) &&
                (0x0B0 > s64_Execution_Parameter[0]) &&
                (s64_Execution_Parameter[1] >= 0)    &&
                (255 > s64_Execution_Parameter[1])     )   )
         {
            BGVAR(u16_Endat_MRS_Adr) = (int)s64_Execution_Parameter[0];
            BGVAR(s16_Endat_ReadWriteTo_Offset_Adr) = (int)s64_Execution_Parameter[1];
         }
         else
         {
           return VALUE_IS_NOT_ALLOWED;
         }

         if ( (0xB9 == BGVAR(u16_Endat_MRS_Adr)) || (0xA1 == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xA3 == BGVAR(u16_Endat_MRS_Adr)) || (0xA5 == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xA7 == BGVAR(u16_Endat_MRS_Adr)) || (0xA9 == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xAB == BGVAR(u16_Endat_MRS_Adr)) || (0xAF == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xB1 == BGVAR(u16_Endat_MRS_Adr)) || (0xB3 == BGVAR(u16_Endat_MRS_Adr)) ||
              (0xB5 == BGVAR(u16_Endat_MRS_Adr)) || (0xB7 == BGVAR(u16_Endat_MRS_Adr))   )
         { // verify legit params
            s16_Endat_Send_Param_State = 0;
            BGVAR(s16_Endat_ReadWrite_RetryAttampts) = 0;
            BGVAR(s16_Endat_Read_Value_State)++;
         }
         else
         {
           return VALUE_IS_NOT_ALLOWED;
         }
      break;

      case 1:
      case 2:
         EndatReadValue(0);
      break;

      case 3:
         BGVAR(s16_Endat_Read_Value_State) = 0;
         if (u16_Endat_Should_Print_EMEMW)
         {
            PrintDecInAsciiHex(BGVAR(u16_Endat_Read_Returned_Value), 4);
            PrintCrLf();
         }
         return (SAL_SUCCESS);
   }
   return (SAL_NOT_FINISHED);
}


//**********************************************************
// Function Name: EndatReadValue
// Description:
// Read value from Endat.
// Target page defined by u16_Endat_MRS_Adr, Target offset defined by s16_Endat_ReadWriteTo_Offset_Adr,
// value is returned into u16_Endat_Read_Returned_Value
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
void EndatReadValue(int drive)
{
   int ret_val = 0;

   switch (BGVAR(s16_Endat_Read_Value_State))
   {
      case 0:   //no read value requested
      break;

      case 1:
         // Memory Range Select Enc. Mfr. Params, 1st group;
         VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~(COMM_FDBK_READY_MASK);
         ret_val = SendEndatParam(MRS_CODE_CMND, BGVAR(u16_Endat_MRS_Adr), 0x00, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, 0);
         if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == BGVAR(u16_Endat_MRS_Adr)))
         {
            BGVAR(u64_Endat_ReadWrite_Error) = 0;
            BGVAR(s16_Endat_ReadWrite_RetryAttampts) = 0;
            BGVAR(s16_Endat_Read_Value_State)++;
            s32_Endat_Timer = Cntr_1mS;
         }

         else if (RETRY_NUMBER > s16_Endat_ReadWrite_RetryAttampts)
         {
            BGVAR(u64_Endat_ReadWrite_Error) = ((0x1000000000000 * (long long)BGVAR(s16_Endat_Read_Value_State))|(0x10000000000 * (long long)BGVAR(s16_Endat_ReadWrite_RetryAttampts)) | (0x100000000 *(long long) ret_val)|(10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(s16_Endat_ReadWrite_RetryAttampts)++;
         }
         else
         {
            BGVAR(u64_Endat_ReadWrite_Error) = ((0x1000000000000 * (long long)BGVAR(s16_Endat_Read_Value_State))|(0x10000000000 * (long long)BGVAR(s16_Endat_ReadWrite_RetryAttampts)) | (0x100000000 *(long long) ret_val)|(10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(s16_Endat_Read_Value_State) = -1; //TimeOut
         }
      break;

      case 2:
         if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...

         ret_val = SendEndatParam(READ_CODE_CMND, BGVAR(s16_Endat_ReadWriteTo_Offset_Adr), 0x0000, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Endat_Read_Returned_Value), drive, 0);
         if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == BGVAR(s16_Endat_ReadWriteTo_Offset_Adr)))
         {
            BGVAR(s16_Endat_Send_Param_State) = 0;
            BGVAR(s16_Endat_ReadWrite_RetryAttampts) = 0;
            BGVAR(u64_Endat_ReadWrite_Error) = 0;
            s32_Endat_Timer = Cntr_1mS;
            BGVAR(s16_Endat_Read_Value_State)++;
            VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_READY_MASK;
         }
         else if (RETRY_NUMBER > BGVAR(s16_Endat_ReadWrite_RetryAttampts))
         {
            BGVAR(u64_Endat_ReadWrite_Error) = ((0x1000000000000 * (long long)BGVAR(s16_Endat_Read_Value_State))|(0x10000000000 * (long long)BGVAR(s16_Endat_ReadWrite_RetryAttampts)) | (0x100000000 *(long long) ret_val)|(10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Endat_Read_Returned_Value));
            BGVAR(s16_Endat_ReadWrite_RetryAttampts)++;
         }
         else
         {
            BGVAR(u64_Endat_ReadWrite_Error) = ((0x1000000000000 * (long long)BGVAR(s16_Endat_Read_Value_State))|(0x10000000000 * (long long)BGVAR(s16_Endat_ReadWrite_RetryAttampts)) | (0x100000000 *(long long) ret_val)|(10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Endat_Read_Returned_Value));
            BGVAR(s16_Endat_Read_Value_State) = -1; //TimeOut
         }
      break;
   }
}


//**********************************************************
// Function Name: EndatHandler
// Description:
//    ENDAT handler, run each background
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void EndatHandler(int drive, int enc_source, int fdbk_dest)
{
   FDBK_OFF;
   int ret_val = 0; //, pos_mismatch = 0;
   static unsigned int tmp1, tmp2;
   unsigned int u16_endat_msg_size = 0;
   long long s64_temp;

   if ( ( (BGVAR(u16_FdbkType) != ENDAT2X_COMM_FDBK)                                      &&
          ( (VAR(AX0_s16_Motor_Enc_Type) != 9) || (BGVAR(u16_FdbkType) != SW_SINE_FDBK)  )  ) &&
        (BGVAR(u16_SFBType) != ENDAT2X_COMM_FDBK)                                               )
         return;

   switch (BGVAR(s16_Endat_Init_State))
   {
      case ENDAT_IDLE_STATE:
         if (BGVAR(u16_SFBType) != ENDAT2X_COMM_FDBK)
         {
            ret_val = LoadParamsFromEndat(0); //Load MPHASE and PFBoffset from Endat's EEPROM
            if ((SAL_SUCCESS != ret_val) && (SAL_NOT_FINISHED != ret_val))
            {
               BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
               BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
            }
         }
      break;

      case ENDAT_REVERSAL_RESET_STATE:
         //Indicate State, Number of attempts, and Sub-Index...
         BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
         BGVAR(s16_DisableInputs) |= ENDAT_DIS_MASK;
         FDBKVAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= 0x0007; // Zero all Bits except Sync Symbol
         BGVAR(u32_Endat_Init_Error) = 0;
         if (enc_source == 0)
         {
            ResetEncState(drive);    // reset encoder state machine
            RemoveFeedback5Volt();   // Power down the 5V to the encoder
         }
         else
            RemoveFeedbackSecondary5Volt();

         s32_Endat_Timer = Cntr_1mS;
         BGVAR(s16_Endat_Init_State) = ENDAT_WAIT_REVERSAL_STATE;
      break;

      case ENDAT_WAIT_REVERSAL_STATE:  //  Wait with Encoder Power Off...
         // Set Clock Control Bit (Bit 1) to 0 to allow direct control of Clock State,
         // set Clock Signal to 1, has to be so for ~1sec. after Power is On.
         *(unsigned int *)(FPGA_ENDAT_ENCODER_CLOCK_REG_ADD + 0x0540 * enc_source) = 0x01;
         //Indicate State, Number of attempts, and Sub-Index...
         BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
         BGVAR(s16_DisableInputs) |= ENDAT_DIS_MASK;
         if (PassedTimeMS(100L, s32_Endat_Timer))
         {
            if (enc_source == 0)
               SetFeedback5Volt(); // Power up the 5V to the encoder
            else
               SetFeedbackSecondary5Volt(); // Power up the 5V to the encoder
            BGVAR(s16_Endat_Init_State) = ENDAT_DELAY_CLOCK_1;
            s32_Endat_Timer = Cntr_1mS;
         }
      break;

      case ENDAT_DELAY_CLOCK_1:
         if (PassedTimeMS(600L, s32_Endat_Timer))
         {
            // set Clock Signal to 0, has to be so for ~400mSec.
            *(unsigned int *)(FPGA_ENDAT_ENCODER_CLOCK_REG_ADD + 0x0540 * enc_source) = 0x00;
            BGVAR(s16_Endat_Init_State) = ENDAT_DELAY_CLOCK_0;
            s32_Endat_Timer = Cntr_1mS;
         }
      break;

      case ENDAT_DELAY_CLOCK_0:
         if (PassedTimeMS(400L, s32_Endat_Timer))
         {
            // set Clock Signal to 1, has to be so for more than 1mSec.
            *(unsigned int *)(FPGA_ENDAT_ENCODER_CLOCK_REG_ADD + 0x0540 * enc_source) = 0x01;
            BGVAR(s16_Endat_Init_State) = ENDAT_DELAY_CLOCK_DONE;
            s32_Endat_Timer = Cntr_1mS;
         }
      break;

      case ENDAT_DELAY_CLOCK_DONE:
         if (PassedTimeMS(4L, s32_Endat_Timer))
         {  // Set Clock Control Bit (Bit 1) to 1 to allow FPGA control of Clock
            *(unsigned int *)(FPGA_ENDAT_ENCODER_CLOCK_REG_ADD + 0x0540 * enc_source) = 0x02;
            BGVAR(s16_Endat_Init_State) = ENDAT_START_INIT;
            s32_Endat_Timer = Cntr_1mS;
         }
      break;

      case ENDAT_START_INIT:
         //Indicate State, Number of attempts, and Sub-Index...
         BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
         BGVAR(u16_EnDat_AbsPos_Bit_Num) = 0;
         BGVAR(s16_Endat_Init_Index) = 0;
         s16_Endat_Send_Param_State = 0;
         s16_Endat_Read_Pos_State  = 0;

         if ((BGVAR(u16_FdbkType) == SW_SINE_FDBK) && (VAR(AX0_s16_Motor_Enc_Type) == 9))
         { // Set Comm. Baud-Rate to 300kHz...
            *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = (int)((float)60000 / (float)300 + 0.5) - 1;
            *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x02; // FPGA EnDat with Sine Settings
         }
         else if (BGVAR(u16_FdbkType) == ENDAT2X_COMM_FDBK)
         {
            *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Fdbk_Clk_Interval);
            *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x23; // FPGA EnDat Comm. Only Settings
         }

         BGVAR(u16_Endat_Err_Cntr) = 0;
         s16_Endat_State = 0;
         tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
         tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
         s32_Endat_Timer = Cntr_1mS;

         BGVAR(s16_Endat_Init_State) = ENDAT_ENCODER_POWER_UP;
      break;

      case ENDAT_ENCODER_POWER_UP:
         //Indicate State, Number of attempts, and Sub-Index...
         BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
         if (PassedTimeMS(25L, s32_Endat_Timer)) // Readability delay...
         {
            BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ENCODER;
            s32_Endat_Timer = Cntr_1mS;
         }
      break;

      case ENDAT_INIT_ENCODER:
         if (!PassedTimeMS(250L, s32_Endat_Timer)) break; // Encoder Power-Up Recovery delay...
         // Address (or MRS Code) and Data are irrelevant for Reset, used only to test CRC Response
         ret_val = SendEndatParam(RESET_CODE_CMND, tmp1, tmp2, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, enc_source);
         if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == tmp1) && (BGVAR(u16_Dummy_Ptr2) == tmp2))
         {  //Indicate State, Number of attempts, and Sub-Index...
            BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
            tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
            tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
            BGVAR(s16_Endat_Init_State) = ENDAT_READ_ALARMS_WARNINGS;
            BGVAR(s16_Endat_Init_Index) = 1;
            s32_Endat_Timer = Cntr_1mS;
         }
         else if (ret_val != SAL_NOT_FINISHED)
         {
            //Indicate State, Number of attempts, and Sub-Index...
            BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
            BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
         }
      break;

      case ENDAT_READ_ALARMS_WARNINGS:
         switch (BGVAR(s16_Endat_Init_Index))
         {
            case 1:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Memory Range Select Operating Status; random Data to test CRC Response
               ret_val = SendEndatParam(MRS_CODE_CMND, 0xB9, tmp2, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, enc_source);
               if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == 0xB9) && (BGVAR(u16_Dummy_Ptr2) == tmp2))
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 2:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 0 for Alarms; no Random Data because returned Data is Alarms State
               ret_val = SendEndatParam(READ_CODE_CMND, ENDAT_ALARMS_ADDR, 0x00, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Endat_Error), drive, enc_source);
               if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == ENDAT_ALARMS_ADDR))
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 3:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 1 for Warnings; no Random Data because returned Data is Warnings State
               ret_val = SendEndatParam(READ_CODE_CMND, ENDAT_WARNINGS_ADDR, 0x00, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Endat_Warning), drive, enc_source);
               if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == ENDAT_WARNINGS_ADDR))
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index) = 1;
                  BGVAR(s16_Endat_Init_State) = ENDAT_CLEAR_ALARMS_WARNINGS;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;
         }
      break;

      case ENDAT_CLEAR_ALARMS_WARNINGS:
         switch (BGVAR(s16_Endat_Init_Index))
         {
            case 1:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Memory Range Select Operating Status; random Data to test CRC Response
               ret_val = SendEndatParam(MRS_CODE_CMND, 0xB9, tmp2, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, enc_source);
               if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == 0xB9) && (BGVAR(u16_Dummy_Ptr2) == tmp2))
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 2:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 0 for Alarms; no Random Data because returned Data is Alarms State
               ret_val = SendEndatParam(WRITE_CODE_CMND, ENDAT_ALARMS_ADDR, 0x00, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, enc_source);
               if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == ENDAT_ALARMS_ADDR) && (BGVAR(u16_Dummy_Ptr2) == 0x00))
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 3:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 1 for Warnings; no Random Data because returned Data is Warnings State
               ret_val = SendEndatParam(WRITE_CODE_CMND, ENDAT_WARNINGS_ADDR, 0x00, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, enc_source);
               if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == ENDAT_WARNINGS_ADDR) && (BGVAR(u16_Dummy_Ptr2) == 0x00))
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index) = 1;
                  BGVAR(s16_Endat_Init_State) = ENDAT_POSITION_RESOLUTION;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;
         }
      break;

      case ENDAT_POSITION_RESOLUTION: //initialize abs pos read
         switch (BGVAR(s16_Endat_Init_Index))
         {
            case 1:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Memory Range Select Enc. Mfr. Params, 1st group; random Data to test CRC Response
               ret_val = SendEndatParam(MRS_CODE_CMND, 0xA1, tmp2, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, enc_source);
               if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == 0xA1) && (BGVAR(u16_Dummy_Ptr2) == tmp2))
               {  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 2:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 13 for Position-Resolution; no Random Data because returned Data is Resolution (number of Bits)
               ret_val = SendEndatParam(READ_CODE_CMND, ENDAT_ABS_POS_BITS_ADDR, 0x0000, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_EnDat_AbsPos_Bit_Num), drive, enc_source);
               if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == ENDAT_ABS_POS_BITS_ADDR) &&
                    ((BGVAR(u16_EnDat_AbsPos_Bit_Num) & 0x8000) == 0x8000)                           ) // Bit 15 of this Address is always '1'.
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(u16_EnDat_AbsPos_Bit_Num) &= 0x7FFF ; // Remove Bit 15 from Variable
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 3:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; //  Readability delay...
               // Read Address 14 for EnDat Type; no Random Data because returned Data is EnDat Type
               ret_val = SendEndatParam(READ_CODE_CMND, ENDAT_ENC_TYPE_ADDR, 0x0000, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_EnDat_Type), drive, enc_source);
               if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == ENDAT_ENC_TYPE_ADDR) &&
                    ((BGVAR(u16_EnDat_Type) & 0x0001) == 0x0001)                                 )
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 4:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; //  Readability delay...
               // Read Address 15 for EnDat Periods / Turn; no Random Data because returned Data is EnDat Periods / Turn
               ret_val = SendEndatParam(READ_CODE_CMND, ENDAT_ENC_RES_ADDR, 0x0000, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_EnDat_Enc_Res_Lo), drive, enc_source);
               if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == ENDAT_ENC_RES_ADDR) )
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index) = 1;
                  if ( ((BGVAR(u16_EnDat_Type) & 0xF000) != 0xE000) &&  // Absolute Multi-Turn Encoder
                       ((BGVAR(u16_EnDat_Type) & 0xF000) != 0xC000) &&  // Absolute Single-Turn Encoder
                       ((BGVAR(u16_EnDat_Type) & 0xF000) != 0x4000))    //Absolute Linear Encoders
                     BGVAR(u16_Sys_Remarks) |= ENDAT_TYPE_NOT_SUPPORTED;
                  else
                     BGVAR(u16_Sys_Remarks) &= ~ENDAT_TYPE_NOT_SUPPORTED;
                  BGVAR(s16_Endat_Init_State) = ENDAT_READ_TURNS; // Read Number of Turns
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;
         }
      break;

      case ENDAT_READ_TURNS:
         switch (BGVAR(s16_Endat_Init_Index))
         {
            case 1:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Memory Range Select Enc. Mfr. Params, 2nd group; random Data to test CRC Response
               ret_val = SendEndatParam(MRS_CODE_CMND, 0xA3, tmp2, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, enc_source);
               if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == 0xA3) && (BGVAR(u16_Dummy_Ptr2) == tmp2))
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 2:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 13 for Position-Resolution; no Random Data because returned Data is Resolution (number of Bits)
               ret_val = SendEndatParam(READ_CODE_CMND, (ENDAT_REV_NUM_ADDR - 16), 0x0000, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_EnDat_Rev_Num), drive, enc_source);
               if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == (ENDAT_REV_NUM_ADDR - 16)) )
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(u16_EnDat_AbsPos_Bit_Num) &= 0x7FFF ; // Remove Bit 15 from Variable
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 3:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 13 for Position-Resolution; no Random Data because returned Data is Resolution (number of Bits)
               ret_val = SendEndatParam(READ_CODE_CMND, (ENDAT_DIGITAL_STEP_ADDR - 16), 0x0000, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_EnDat_Dig_Step_Lo), drive, enc_source);
               if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == (ENDAT_DIGITAL_STEP_ADDR - 16)) )
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 4:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 13 for Position-Resolution; no Random Data because returned Data is Resolution (number of Bits)
               ret_val = SendEndatParam(READ_CODE_CMND, (ENDAT_DIGITAL_STEP_ADDR - 16 + 1), 0x0000, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_EnDat_Dig_Step_Hi), drive, enc_source);
               if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == (ENDAT_DIGITAL_STEP_ADDR - 16 + 1)) )
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 5:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 13 for Position-Resolution; no Random Data because returned Data is Resolution (number of Bits)
               ret_val = SendEndatParam(READ_CODE_CMND, (ENDAT_ENC_RES_ADDR + 1 - 16), 0x0000, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_EnDat_Enc_Res_Hi), drive, enc_source);
               if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == (ENDAT_ENC_RES_ADDR + 1 - 16)) )
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_State) = ENDAT_22_SUPPORT;
                  BGVAR(s16_Endat_Init_Index) = 1;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;
         }

         if ( ((BGVAR(u16_EnDat_Rev_Num) > 1) > 0)                              ||
              (BGVAR(u16_Mpoles) > 20) || (BGVAR(u16_MotorType) == LINEAR_MOTOR)  )
            VAR(AX0_u16_Abs_Fdbk_Device) |= (0x0001 << fdbk_dest);
         else
            FDBKVAR(AX0_u16_Abs_Fdbk_Device) &= ~(0x0001 << fdbk_dest); // Manipulate Bit 0 only as indicator for Motor Feedback

         // Set Absolute Feedback Device Indication in Bit 0 or 1 according to the Feedback Destination (Motor or Load)
         // This updates the operational homing offset with the user non-volatile variable
         // according to the feedback multi turn absolute
         UpdateRefOffsetValue(drive);
      break;

      case ENDAT_22_SUPPORT:
         switch (BGVAR(s16_Endat_Init_Index))
         {
            case 1:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Memory Range Select Enc. Mfr. Params, 3rd group; random Data to test CRC Response
               ret_val = SendEndatParam(MRS_CODE_CMND, 0xA5, tmp2, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive, enc_source);
               if ((ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == 0xA5) && (BGVAR(u16_Dummy_Ptr2) == tmp2))
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 2:
               if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
               // Read Address 13 for Position-Resolution; no Random Data because returned Data is Resolution (number of Bits)
               ret_val = SendEndatParam(READ_CODE_CMND, (ENDAT_22_SUPPORT_ADDR - 32), 0x0000, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_EnDat_22_Supported), drive, enc_source);
               if ( (ret_val == SAL_SUCCESS) && (BGVAR(u16_Dummy_Ptr1) == (ENDAT_22_SUPPORT_ADDR - 32)) )
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  tmp1 = (unsigned int)(Cntr_3125 & 0xFF);
                  tmp2 = (unsigned int)((tmp1 * tmp1) ^ Cntr_3125);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_DONE;
                  BGVAR(s16_Endat_Init_Index)++;
                  s32_Endat_Timer = Cntr_1mS;
               }
               else if (ret_val != 0)
               {
                  //Indicate State, Number of attempts, and Sub-Index...
                  BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
                  BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
               }
            break;

            case 3:
            break;
         }
      break;

      case ENDAT_INIT_OEM_MRS:
      break;

      case ENDAT_INIT_OEM_PARAMS:
      break;

      case ENDAT_INIT_DONE:
         if (!PassedTimeMS(25L, s32_Endat_Timer)) break; // Readability delay...
         if ((BGVAR(u16_EnDat_Type) & 0xF000) == 0xE000) // If Multi-Turn Absolute
         {
            FDBKVAR(AX0_u16_ED_Turns_Bits) = GetBitNumber(BGVAR(u16_EnDat_Rev_Num)) - 1;
            FDBKVAR(AX0_u16_ED_In_Turn_Bits) = BGVAR(u16_EnDat_AbsPos_Bit_Num) - FDBKVAR(AX0_u16_ED_Turns_Bits);
         }
         else // If Single-Turn Absolute, or Linear
         {
            FDBKVAR(AX0_u16_ED_In_Turn_Bits) = BGVAR(u16_EnDat_AbsPos_Bit_Num);
            FDBKVAR(AX0_u16_ED_Turns_Bits) = 0;
         }

         // Prevent Drive Operation if EnDat In-Turn Resolution exceeds 31-Bits; this will make Feedback
         // Position Parsing erroneous because of rollover issues.  In case of Linear, see after Digital-Step calculation.
         if ((BGVAR(u16_EnDat_Type) & 0xC000) == 0xC000)
         {
            if (FDBKVAR(AX0_u16_ED_In_Turn_Bits) > 30)
               BGVAR(s64_SysNotOk_2) |= ENDAT_EXCESSIVE_RES_FAULT;
            else
               BGVAR(s64_SysNotOk_2) &= ~ENDAT_EXCESSIVE_RES_FAULT;
         }

         BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = (unsigned long) FDBKVAR(AX0_u16_ED_Turns_Bits);

         if ((BGVAR(u16_EnDat_22_Supported) & 0x0003) == 0x0001) // Bits '0' and '1' are 01,
         {                                                       // indicating EnDat2.2 Support
            FDBKVAR(AX0_u16_Abs_Enc_Command_1Frame) = 0x001D; // Mode Command Send Position & Additional Data
            FDBKVAR(AX0_u16_ED_Alarm_Bits) = 2;
            FDBKVAR(AX0_u16_Com_EnDat_Rx1_Length) = 3 + BGVAR(u16_EnDat_AbsPos_Bit_Num) + 5;
            // Response is Start-Bit + two Alarm-bits + Data Length + CRC Length
         }
         else // EnDat2.1 Support only
         {
            FDBKVAR(AX0_u16_Abs_Enc_Command_1Frame) = 0x00E1; // Mode Command Send Position (EnDat2.1)
            FDBKVAR(AX0_u16_ED_Alarm_Bits) = 1;
            FDBKVAR(AX0_u16_Com_EnDat_Rx1_Length) = 2 + BGVAR(u16_EnDat_AbsPos_Bit_Num) + 5;
            // Response is Start-Bit + one Alarm-Bit + Data Length + CRC Length
         }

         // This will be used to determine the relevant bits used for the CRC calculation
         // AX0_u16_EnDat_Msg_Sizex will determine how many relevant bits are there in each message word
         u16_endat_msg_size = FDBKVAR(AX0_u16_ED_Alarm_Bits) + FDBKVAR(AX0_u16_ED_In_Turn_Bits) + FDBKVAR(AX0_u16_ED_Turns_Bits);
         if (u16_endat_msg_size >= 16)
         { // Store Message Size for CRC Processing in RT Code
            FDBKVAR(AX0_u16_EnDat_Msg_Size) = 16;

            if (u16_endat_msg_size >= 32)
            {
               FDBKVAR(AX0_u16_EnDat_Msg_Size) |= ( (16 << 6) | ((u16_endat_msg_size - 32) << 12) );
            }
            else
            {
               FDBKVAR(AX0_u16_EnDat_Msg_Size) |= ((u16_endat_msg_size - 16) << 6);
            }
         }
         else
         {
            FDBKVAR(AX0_u16_EnDat_Msg_Size) = u16_endat_msg_size;
         }

         // calculate encoders resolution
         // Communication Feedback Digital-Step, data taken from words 20, 21, used in HWPOS
         if (((BGVAR(u16_EnDat_Type) & 0xF000) < 0x8000))//linear feedback --> read nm to step, switch to mm and devide by 4 to get lines per pitch
         {
            BGVAR(u32_EnDat_Digital_Step) = BGVAR(u32_Mpitch) * 250 / (((unsigned long)BGVAR(u16_EnDat_Dig_Step_Hi) << 16) | (unsigned long)BGVAR(u16_EnDat_Dig_Step_Lo)); // mpitch changed to decimal
            if (GetBitNumber(BGVAR(u32_EnDat_Digital_Step)) > 30) // Check if resulting Resolution exceeds limit.
               BGVAR(s64_SysNotOk_2) |= ENDAT_EXCESSIVE_RES_FAULT;
            else
               BGVAR(s64_SysNotOk_2) &= ~ENDAT_EXCESSIVE_RES_FAULT;
         }
         else //rotary feedback --> read steps per rev, devide by 4 to get lines per rev
            BGVAR(u32_EnDat_Digital_Step) =((((unsigned long)BGVAR(u16_EnDat_Dig_Step_Hi) << 16) | (unsigned long)BGVAR(u16_EnDat_Dig_Step_Lo)) >> 2);

         if ( (ENDAT2X_COMM_FDBK == BGVAR(u16_FdbkType)) || (ENDAT2X_COMM_FDBK == BGVAR(u16_SFBType)) )
         {// communication-only feedback, data the same as Digital Step
            BGVAR(u32_EnDat_Enc_Res) = BGVAR(u32_EnDat_Digital_Step);
         }
         else
         {// Sine-Signal feedback, data taken from words 15, 16
             if (((BGVAR(u16_EnDat_Type) & 0xF000) < 0x8000))//linear feedback --> read nm to step, switch to mm to get lines per pitch
               BGVAR(u32_EnDat_Enc_Res) = BGVAR(u32_Mpitch) * 1000 / (((unsigned long)BGVAR(u16_EnDat_Enc_Res_Hi) << 16) | (unsigned long)BGVAR(u16_EnDat_Enc_Res_Lo));//mpitch changed to decimal
             else //rotary feedback --> read lines per rev
               BGVAR(u32_EnDat_Enc_Res) = ((unsigned long)BGVAR(u16_EnDat_Enc_Res_Hi) << 16) | (unsigned long)BGVAR(u16_EnDat_Enc_Res_Lo);
         }

         FDBKVAR(AX0_u16_Abs_Enc_Command_2Frame) = 0x0000;
         FDBKVAR(AX0_u16_Abs_Enc_Command_3Frame) = 0x0000;
         FDBKVAR(AX0_u16_Com_EnDat_Tx1_Length) = 8; // Position Read Command only

         if ( (BGVAR(u16_FdbkType) == ENDAT2X_COMM_FDBK)                                     ||
              ( (VAR(AX0_s16_Motor_Enc_Type) == 9) && (BGVAR(u16_FdbkType) == SW_SINE_FDBK) )  )
         {
            if (BGVAR(u32_User_Motor_Enc_Res) == BGVAR(u32_EnDat_Enc_Res))
               BGVAR(u64_Sys_Warnings) &= ~ENDAT_MENCRES_MISMATCH_WRN;
            else
               BGVAR(u64_Sys_Warnings) |= ENDAT_MENCRES_MISMATCH_WRN;
         }
         else if (BGVAR(u16_SFBType) == ENDAT2X_COMM_FDBK)
         {
            if (BGVAR(u32_User_Sec_Enc_Res) == BGVAR(u32_EnDat_Enc_Res))
               BGVAR(u64_Sys_Warnings) &= ~ENDAT_MENCRES_MISMATCH_WRN;
            else
               BGVAR(u64_Sys_Warnings) |= ENDAT_MENCRES_MISMATCH_WRN;
         }

         if ( (BGVAR(u16_FdbkType) == ENDAT2X_COMM_FDBK) || (BGVAR(u16_SFBType) == ENDAT2X_COMM_FDBK) )
         {
            FDBKVAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK;
            //BGVAR(s16_DisableInputs) &= ~ENDAT_DIS_MASK;
            BGVAR(s64_SysNotOk) &= ~SIN_COMM_FLT_MASK;

            if(SFB_ENDAT)
               BGVAR(s16_DisableInputs) &= ~ENDAT_DIS_MASK;

            BGVAR(s16_Endat_Init_State) = ENDAT_IDLE_STATE;
            BGVAR (u16_Load_Params_From_Endat_State) = 1;
         }
         else if ( (VAR(AX0_s16_Motor_Enc_Type) == 9)                   &&
                   (LVAR(AX0_s32_Feedback_Ptr) == &SW_SINE_ENC_FEEDBACK)  )
         {
            s32_Endat_Timer = Cntr_1mS;
            BGVAR(s16_Endat_Init_State) = ENDAT_READ_ABS_POS_STATE;
         }
      break;

      case ENDAT_READ_ABS_POS_STATE:  //  Send the absolute position read command
         // Left Shift of Digital Data = 16 - (Sine-Signal Interpolation Level) + 2 (leaving
         // Quadrature Bit at bottom of AH after shift) = 18 - (InTurnBits - SinePeriodBits)
         if (((BGVAR(u16_EnDat_Type) & 0xF000) > 0x8000)) // Rotary Feedback --> rely on Absolute
            // Position Word and Encoder Resolution be Powers-of-Two; calculate required Shift using
         {  // In-Turn Data
            VAR(AX0_u16_Sine_Match_Shift) = 18 - FDBKVAR(AX0_u16_ED_In_Turn_Bits) + GetBitNumber(BGVAR(u32_EnDat_Enc_Res)) - 1;
            VAR(AX0_u16_EnDat_Type_Ind) = 0; // Indicate Processing of Powers-of-Two parameters
         }
         else // Linear Feedback --> perform Matching Calculations using Division
         {
            VAR(AX0_u16_Sine_Match_Div) = (BGVAR(u32_EnDat_Digital_Step) / BGVAR(u32_EnDat_Enc_Res));
            VAR(AX0_u16_EnDat_Type_Ind) = 1; // Indicate Processing of non-Powers-of-Two parameters
         }
         ret_val = ReadEndatAbsPosition(drive);
         if (ret_val == SAL_SUCCESS)
         {  // Initialize offsets between analog and digital vaules
            VAR(AX0_s16_Sine_Enc_Bits) |= SW_SINE_READ_POS_DONE_MASK;

            //Indicate State, Number of attempts, and Sub-Index...
            BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);

            BGVAR(s16_Endat_Init_State) = ENDAT_INIT_PFB_AND_COMM_STATE;
         }
         else if (ret_val != SAL_NOT_FINISHED)
         {
            //Indicate State, Number of attempts, and Sub-Index...
            BGVAR(u32_Endat_Init_Error) = (0x1000000 * (long)BGVAR(s16_Endat_Init_State)) | (0x10000 * (long)BGVAR(s16_Endat_Init_Index)) | (0x100 * ret_val) | BGVAR(s16_Endat_Machine_Retry);
            BGVAR(s16_Endat_Init_State) = ENDAT_INIT_ERR;
         }
      break;

      case ENDAT_INIT_PFB_AND_COMM_STATE:
         // Wait till the RT completes its init
         if ((VAR(AX0_s16_Sine_Enc_Bits) & SW_SINE_READ_POS_DONE_MASK) == 0)
         {  // Combined absolute value is stored at AX0_u32_Comm_Abs_Pos_32_Lo/Hi
            do {
               tmp1 = Cntr_3125;
               s64_temp = LLVAR(AX0_u32_Comm_Abs_Pos_32_Lo);
            } while (tmp1 != Cntr_3125);

            LVAR(AX0_s32_Pos_Fdbk_Hi) = (long)(s64_temp / (long long)LVAR(AX0_s32_Counts_Per_Rev));

            LVAR(AX0_u32_Fdbk_Accu) = (unsigned long)(s64_temp - ((long long)LVAR(AX0_s32_Pos_Fdbk_Hi) * (long long)LVAR(AX0_s32_Counts_Per_Rev)));
            LVAR(AX0_u32_Pos_Fdbk_Lo) = (unsigned long)((unsigned long long)LVAR(AX0_u32_Fdbk_Accu) << 32LL) / (long long)LVAR(AX0_s32_Counts_Per_Rev);

            // Signal RT it can start updating PFB
            VAR(AX0_s16_Skip_Flags) &= ~SW_SINE_UPDATING_PFB_MASK;

            BGVAR(s64_SysNotOk) &= ~SIN_COMM_FLT_MASK;
            s32_abs_pos_hi = 0;
            //BGVAR(s16_DisableInputs) &= ~ENDAT_DIS_MASK;    // Clear disable bit
            BGVAR(s16_Endat_Init_State) = ENDAT_IDLE_STATE;
            BGVAR (u16_Load_Params_From_Endat_State) = 1;
         }
      break;

      case ENDAT_INIT_ERR:
         BGVAR(s16_Endat_Machine_Retry)++;
         if (BGVAR(s16_Endat_Machine_Retry) >= NUMBER_OF_MACHINE_RETRY)
         {
            BGVAR(s16_Endat_Init_State) = ENDAT_ERR_IDLE_STATE;
            BGVAR(s16_Endat_Machine_Retry) = 0;
         }
         else
         {
            BGVAR(s16_Endat_Init_State) = ENDAT_REVERSAL_RESET_STATE; // start endat state machine
            VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_SW_SINE_ENC_ENDAT & 0xffff); // changed to avoid Remark
         }
      break;

      case ENDAT_ERR_IDLE_STATE:
      break;
   }
}


//**********************************************************
// Function Name: SalEnDatSaveParameters
// Description:
//          This function is called to save OEM params on ENDAT eeprom
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalEnDatSaveParameters(int drive)
{
   unsigned int tmp;
   int s16_val_0, s16_val_1;
   int drive_backup = drive;

   if (LVAR(AX0_s32_Feedback_Ptr) != &SW_SINE_ENC_FEEDBACK) return NOT_AVAILABLE;

   switch (BGVAR(s16_Endat_Save_State))
   {
      case ENDAT_SAVE_INIT_STATE:
         //do not allow to run this state machine with the endat init state machine
         drive = 0;
         s16_val_0 = BGVAR(s16_Endat_Init_State);
         drive = 1;
         s16_val_1 = BGVAR(s16_Endat_Init_State);
         drive = drive_backup;
         if ( (s16_val_0 != ENDAT_IDLE_STATE) || (s16_val_1 != ENDAT_IDLE_STATE)  )
         {
            if ( (s16_val_0 == ENDAT_ERR_IDLE_STATE) || (s16_val_1 == ENDAT_ERR_IDLE_STATE)  )
               BGVAR(u16_Endat_Error) = ENDAT_READY_ERROR;
            else BGVAR(u16_Endat_Error) = ENDAT_BUSY_ERROR;
               return (BGVAR(u16_Endat_Error));
         }

         BGVAR(s16_Endat_Save_State)++;
         BGVAR(u16_Endat_Error) = 0;
         BGVAR(u16_Endat_Err_Cntr) = 0;
         s16_Endat_Save_Ind = 0;
      break;

      case ENDAT_SAVE_MRS_STATE:
//         u16_Endat_Error = SendEndatParam(MRS_CODE_CMND, 0xA9 + BGVAR(u16_EnDat_Mem_Part_Select), 0, &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive);
         if (BGVAR(u16_Endat_Error) == SAL_SUCCESS) BGVAR(s16_Endat_Save_State)++;
         else if (BGVAR(u16_Endat_Error) == SAL_NOT_FINISHED) return SAL_NOT_FINISHED;
         else
         {
            BGVAR(u16_Endat_Err_Cntr)++;
            if (BGVAR(u16_Endat_Err_Cntr) > 3) BGVAR(s16_Endat_Save_State) = ENDAT_SAVE_ERR;
         }
      break;

      case ENDAT_SAVE_READ_ZEIDEL_STATE:
         BGVAR(s16_Endat_Save_State)++;
         //u16_Endat_Error = SendEndatParam(READ_CODE_CMND,0,0,&s16_EnDat_Zeidel_Mnum,drive);
         //if (u16_Endat_Error == SAL_SUCCESS) BGVAR(s16_Endat_Save_State)++;
         //else if (u16_Endat_Error == SAL_NOT_FINISHED) return SAL_NOT_FINISHED;
         //else  BGVAR(s16_Endat_Save_State) = ENDAT_SAVE_ERR;
      break;

      case ENDAT_SAVE_CALC_STATE:
         tmp = (unsigned int)VAR(AX0_s16_Electrical_Phase_Offset);
         Math32 = ((long)tmp * 0x5A00) + 0x200000;
         Math32 = Math32 >> 22;
         Math32 += 60;
         if( Math32 >= 360 ) Math32 -= 360;
         BGVAR(u16_Endat_Mphase) = Math32;

         BGVAR(u16_Endat_Pfb_Off1_Hi) = 0; //LVAR(AX0_s32_Pos_Fdbk_Offset_32)>> 16;
         BGVAR(u16_Endat_Pfb_Off1_Lo) = 0; //(int)LVAR(AX0_s32_Pos_Fdbk_Offset_32);
         BGVAR(u16_Endat_Pfb_Off2_Hi) = ~BGVAR(u16_Endat_Pfb_Off1_Hi);
         BGVAR(u16_Endat_Pfb_Off2_Lo) = ~BGVAR(u16_Endat_Pfb_Off1_Lo);

         BGVAR(s16_Endat_Save_State)++;
      break;

      case ENDAT_SAVE_PARAM_STATE:
//         u16_Endat_Error = SendEndatParam(WRITE_CODE_CMND, s_Endat_Init_Oem_Table[s16_Endat_Save_Ind].addr_or_mrs + BGVAR(s16_Endat_Mrs_Offset),
//                               *((int*)s_Endat_Init_Oem_Table[s16_Endat_Save_Ind].var_ptr + drive), &BGVAR(u16_Dummy_Ptr1), &BGVAR(u16_Dummy_Ptr2), drive);

         if (BGVAR(u16_Endat_Error) == SAL_SUCCESS) s16_Endat_Save_Ind++;
         else if (BGVAR(u16_Endat_Error) == SAL_NOT_FINISHED) break;
         else
         {
            BGVAR(u16_Endat_Err_Cntr)++;
            if (BGVAR(u16_Endat_Err_Cntr) > 10) BGVAR(s16_Endat_Save_State) = ENDAT_SAVE_ERR;
         }

         if (s16_Endat_Save_Ind == NUMBER_OF_ENDAT_OEM_STATES) BGVAR(s16_Endat_Save_State)++;
      break;

      case ENDAT_SAVE_DONE:
         BGVAR(s16_Endat_Save_State) = 0;
         return SAL_SUCCESS;
         //break;

      case ENDAT_SAVE_ERR:
         BGVAR(s16_Endat_Save_State) = 0;
         BGVAR(s64_SysNotOk) |= SIN_COMM_FLT_MASK;
         return BGVAR(u16_Endat_Error);
   }

   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: SalReadEndatAbsPos
// Description:
//          This function is called to read Endat HW position
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalReadEndatAbsPos(int drive)
{
   int ret_val = 0;
   static int state = 0;

   switch (state)
   {
      case 0:
         BGVAR(u16_Endat_Err_Cntr) = 0;
      break;

      case 1:
         ret_val = ReadEndatAbsPosition(drive);
         if (ret_val == SAL_SUCCESS)
         {

            //the digital and analog resolution may be different
            //get rid of extra bits
            //if (ret_val == SAL_SUCCESS)
            //{
            // temp = GetBitNumber(BGVAR(u32_EnDat_Digital_Step)) - GetBitNumber(BGVAR(u32_EnDat_Enc_Res)*4);
            // BGVAR(u32_Hw_Abs_Pos_32) >>= temp;
            //}

            state = 0;
         }
         else if (ret_val != SAL_NOT_FINISHED)
         {  //HWPOS Read failed
            BGVAR(u16_Endat_Err_Cntr)++;
            if (BGVAR(u16_Endat_Err_Cntr) > 3)            //try 3 times
            {
               BGVAR(s64_SysNotOk) |= SIN_COMM_FLT_MASK;
               state = 0;
            }
            else
               ret_val = SAL_NOT_FINISHED;
         }
      break;
   }
   return ret_val;
}


//**********************************************************
// Function Name: HwPosCommand
// Description:
//          This function is called in response to the HWPOS command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalHwPosCommand(long long* pos, int drive)
{
   unsigned int u16_dummy;
   return HwPosCommand(pos, &u16_dummy, drive);
}

// u16_hwpos_size_in_bits will indicate how many relevant bits are there in the HWPOS
int HwPosCommand(long long* pos, unsigned int *u16_hwpos_size_in_bits, int drive)
{
   unsigned int Local_Data_Frame0, Local_Data_Frame1, Local_Data_Frame2, Local_Data_Frame3, u16_temp_time;

   int return_value = SAL_SUCCESS;

   *u16_hwpos_size_in_bits = 32;

   if ( (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_MULTI_TURN_FDBK)  ||
        (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK) ||
        (BGVAR(u16_FdbkType) == PS_S_COMM_FDBK)                   )
   {
      BGVAR(u16_Tm_Enc_ID_Res) = VAR(AX0_u16_Abs_Enc_Data_1Frame) >> 8;
      do {
         u16_temp_time = Cntr_3125;
         *pos = (((unsigned long long)(LVAR(AX0_s32_Comm_Abs_Pos_32_Hi))) << (unsigned long long)BGVAR(u16_Tm_Enc_ID_Res))
                | ((unsigned long long)LVAR(AX0_u32_Comm_Abs_Pos_32_Lo) & ((1LL << BGVAR(u16_Tm_Enc_ID_Res)) - 1));
      } while (u16_temp_time != Cntr_3125);

      if (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK)
         *u16_hwpos_size_in_bits = BGVAR(u16_Tm_Enc_ID_Res);
      else if (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_MULTI_TURN_FDBK)
         *u16_hwpos_size_in_bits = BGVAR(u16_Tm_Enc_ID_Res) + 16;
      else
         *u16_hwpos_size_in_bits = 33;
   }
   else if (BGVAR(u16_FdbkType) == PS_P_G_COMM_FDBK)
      *pos = (long long)LVAR(AX0_u32_Comm_Abs_Pos_32_Lo);
   else if (BGVAR(u16_FdbkType) == NK_COMM_FDBK)
   {
      Local_Data_Frame0 = GetBitNumber(BGVAR(u32_User_Motor_Enc_Res)) + 1;
      do {
         u16_temp_time = Cntr_3125;
         *pos = ((unsigned long long)LVAR(AX0_s32_Comm_Abs_Pos_32_Hi) << (long)Local_Data_Frame0)
                | ((unsigned long long)LVAR(AX0_u32_Comm_Abs_Pos_32_Lo) & ((1LL << Local_Data_Frame0) - 1));
      } while (u16_temp_time != Cntr_3125);
      *u16_hwpos_size_in_bits = 16 + Local_Data_Frame0;
   }
   else if (BGVAR(u16_FdbkType) == ENDAT2X_COMM_FDBK)
   { // Transfer Encoder Data Frames to local Variables to ensure consistent values
      do {
         u16_temp_time = Cntr_3125;
         Local_Data_Frame0 = VAR(AX0_u16_Abs_Enc_Data_0Frame);
         Local_Data_Frame1 = VAR(AX0_u16_Abs_Enc_Data_1Frame);
         Local_Data_Frame2 = VAR(AX0_u16_Abs_Enc_Data_2Frame);
         Local_Data_Frame3 = VAR(AX0_u16_Abs_Enc_Data_3Frame);
      } while (u16_temp_time != Cntr_3125);
      *pos = ( (long long)Local_Data_Frame0 | ((long long)Local_Data_Frame1 << 16) |
               ((long long)Local_Data_Frame2 << 32)                                |
               ((long long)Local_Data_Frame3 << VAR(AX0_u16_ED_In_Turn_Bits))       );
      *u16_hwpos_size_in_bits = 48 + VAR(AX0_u16_ED_In_Turn_Bits);
   }
   else if (BGVAR(u16_FdbkType) == SW_SINE_FDBK)
   {
      if (VAR(AX0_s16_Motor_Enc_Type) == 9)
      {
         return_value = ReadEndatAbsPosition(drive);
         if (return_value == SAL_SUCCESS)
         {
            *pos = ((long long)BGVAR(u32_ED_Abs_Pos_Lo) | ((long long)BGVAR(s32_ED_Abs_Pos_Hi) << VAR(AX0_u16_ED_In_Turn_Bits)));
            *u16_hwpos_size_in_bits = VAR(AX0_u16_ED_Turns_Bits) + VAR(AX0_u16_ED_In_Turn_Bits);
         }
         else
            return return_value;
      }
      else if (VAR(AX0_s16_Motor_Enc_Type) == 10)
      {
         return_value = ReadHifaceAbsPos(drive);
         if (return_value == SAL_SUCCESS)
            *pos = (long long)LVAR(AX0_s32_Sw_Sine_Abs_Pos_32);
         else
            return return_value;
      }
      else if (((VAR(AX0_s16_Motor_Enc_Type) <= 6) && ((VAR(AX0_s16_Motor_Enc_Type) != 5))) || (VAR(AX0_s16_Motor_Enc_Type) == 11))
      {
         *pos = ((long long)((unsigned int)(VAR(AX0_s16_Qep)))) & 0x00000000FFFFFFFF;
         *u16_hwpos_size_in_bits = 16;
      }
      else return_value = NOT_AVAILABLE;
   }
   else if (BGVAR(u16_FdbkType) == INC_ENC_FDBK)
   {
      if (((VAR(AX0_s16_Motor_Enc_Type) <= 6) && ((VAR(AX0_s16_Motor_Enc_Type) != 5))) || (VAR(AX0_s16_Motor_Enc_Type) == 11))
      {
         *pos = ((long long)((unsigned int)(VAR(AX0_s16_Qep)))) & 0x00000000FFFFFFFF;
         *u16_hwpos_size_in_bits = 16;
      }
      else return_value = NOT_AVAILABLE;
   }
   else if (FEEDBACK_SERVOSENSE)
   {
      *pos = ((long long)LVAR(AX0_u32_SrvSns_Abs_Pos_Raw) & 0x00000000003FFFFF);
      *u16_hwpos_size_in_bits = 22;
   }
   else if (FEEDBACK_YASKAWA)
   {
      do {
         u16_temp_time = Cntr_3125;
         *pos = ((((unsigned long long)(LVAR(AX0_s32_Comm_Abs_Pos_32_Hi))) << 20LL)) & 0x00000000FFF00000
                | ((unsigned long long)LVAR(AX0_u32_Comm_Abs_Pos_32_Lo) & 0x00000000000FFFFF) ;
      } while (u16_temp_time != Cntr_3125);
      *u16_hwpos_size_in_bits = 32;
   }
   else if (FEEDBACK_SANKYO) // We are using 17bit of Inturn bits + 24 bits of Multiturn
   {
      do {
         u16_temp_time = Cntr_3125;
         *pos = ((((unsigned long long)(LVAR(AX0_s32_Comm_Abs_Pos_32_Hi))) << 17LL)) & 0x000001FFFFFE0000
                | ((unsigned long long)LVAR(AX0_u32_Comm_Abs_Pos_32_Lo) & 0x000000000001FFFF) ;
      } while (u16_temp_time != Cntr_3125);
      *u16_hwpos_size_in_bits = 41;
   }
   else if (BGVAR(u16_FdbkType) == BISSC_COMM_FDBK) // Anatoly TODO
   {
      s32_abs_pos_hi = LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) & 0x000000FF;
      do {
         u16_temp_time = Cntr_3125;
         *pos = (unsigned long long)(LVAR(AX0_s32_Comm_Abs_Pos_32_Hi) << s32_abs_pos_hi)
                | (unsigned long long)LVAR(AX0_u32_Comm_Abs_Pos_32_Lo);
      } while (u16_temp_time != Cntr_3125);
      *u16_hwpos_size_in_bits = GetBitNumber(LVAR(AX0_s32_Counts_Per_Rev_2));
   }
   else return_value = NOT_AVAILABLE;

   return return_value;
}


//**********************************************************
// Function Name: SalAbsPosModeCommand
// Description:
//          This function is called in response to the ABSPOSMODE command.
//          to set or query the way how to handle abs pos.
//          0 - abs pos is unsigned value
//          1 - abs pos is signed value
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalAbsPosModeCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;
   if (BGVAR(s16_Abs_Pos_Mode) == (int)lparam) return (SAL_SUCCESS);

   BGVAR(s16_Abs_Pos_Mode) = (int)lparam;

   if (LVAR(AX0_s32_Feedback_Ptr) == &SW_SINE_ENC_FEEDBACK)
   {
      VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;      // set no-comp fault
   }
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: HSaveLoadCallEMEMW
// Description:call the EMEMW Read/Write Sal Functions, depending on 'numOfParams'
//
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int HSaveLoadCallEMEMW (int drive, unsigned int mrs_adr, int offset, unsigned int value, int numOfParams)
{
   int ret_val = 0;
   STORE_EXECUTION_PARAMS_0_1_2;
   s64_Execution_Parameter[0] = (long long)mrs_adr;
   s64_Execution_Parameter[1] = (long long)offset;
   s16_Number_Of_Parameters = numOfParams;
   if (!BGVAR(u16_HsaveLoad_Should_Print_EMEMW))
      BGVAR(u16_Endat_Should_Print_EMEMW) = 0;

   if (2 == s16_Number_Of_Parameters)   //Read method
   {
      ret_val = ReadEndatMemoryWord(drive);
   }
   else                                //Write Method
   {
      s64_Execution_Parameter[2] = (long long)value;
      ret_val = SalWriteEndatMemoryWord(drive);
   }

   BGVAR(u16_Endat_Should_Print_EMEMW) = 1;
   RESTORE_EXECUTION_PARAMS_0_1_2;
   return ret_val;
}


//**********************************************************
// Function Name: SalHSaveToEndatFlash
// Description:
//          This function is called in response to the HSAVE command.
//          -- Parmas writing to Endat's EEPROM on OEM params memory location. --
//          number of params:
//          0 - write parameters (MPHASE, PFB_offset, checksum) to Endat's EEPROM, and mark the 'stamp' with READY_TO_LOAD value
//          1 - (should equal '-1') mark the 'stamp' with DONT_LOAD value
//
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int SalHSaveToEndatFlash (int drive)
{
   int ret_val = 0;
   unsigned int checksum = 0;

   switch (BGVAR(u16_HSave_State))
   {
      case 0://first run, check legit param
         if ((ENDAT2X_COMM_FDBK != BGVAR(u16_FdbkType)) && (SW_SINE_FDBK != BGVAR(u16_FdbkType)))
            return NOT_SUPPORTED_ON_FEEDBACK;
         if ((s16_Number_Of_Parameters == 1) && (s64_Execution_Parameter[0] != 1LL))
            return VALUE_IS_NOT_ALLOWED;
         if (0 == BGVAR(u16_Endat_Params_Mrs_Adr))
            return NOT_PROGRAMMABLE;
         BGVAR(u16_HSave_Num_Of_Params) = s16_Number_Of_Parameters;
         BGVAR(u16_HSave_State)++;
      break;

      case 1://STAMP write
         if (1 == BGVAR(u16_HSave_Num_Of_Params)) // reset STAMP in Endat so parameters will not be loaded to drive on Load()
         {
            ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset)+ ENDAT_STAMP_OFFSET), ENDAT_STAMP_DISREGARD_PARAMETERS, 3);
            if (SAL_NOT_FINISHED == ret_val) return ret_val;
            if (SAL_SUCCESS != ret_val)
               BGVAR(u64_HSave_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_State)) | (0x100000000 * (long long)ENDAT_STAMP_DISREGARD_PARAMETERS) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1))) | (long long)BGVAR(u16_Dummy_Ptr2));
            else
               BGVAR(u64_HSave_Error) = 0;
            BGVAR(u16_HSave_State) = 0;
            return ret_val;
         }
         else //No param - save all parameters to Endat
         {
             //write into Endat's STAMP a Value of 'starting-to-copy-parameters'
            ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset)+ ENDAT_STAMP_OFFSET), ENDAT_STAMP_COPYING_PARAMETERS, 3);
            if (SAL_NOT_FINISHED == ret_val) return ret_val;
            if (SAL_SUCCESS != ret_val)
            {
               BGVAR(u64_HSave_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_State)) | (0x100000000 * (long long)ENDAT_STAMP_COPYING_PARAMETERS) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1))) | (long long)BGVAR(u16_Dummy_Ptr2));
               BGVAR(u16_HSave_State) = 0;
               return ret_val;
            }
            else
               BGVAR(u16_HSave_State)++;
         }
      break;

      case 2://MPHASE write
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_MPHASE_OFFSET),(unsigned int)VAR(AX0_s16_Electrical_Phase_Offset), 3);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_State)) | (0x100000000 * (long long)VAR(AX0_s16_Electrical_Phase_Offset)) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1))) | (long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_HSave_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_State)++;
      break;

      case 3://PFBoffset write #1/4 (MSB bits --> 48-63)
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_FIRST_PFB_OFFSET),((unsigned int)((LVAR(AX0_s32_Pos_Fdbk_Offset_Hi)>>16) & 0xffff)), 3);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_State)) | (0x100000000 * (long long)((LVAR(AX0_s32_Pos_Fdbk_Offset_Hi)>>16) & 0xffff)) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_HSave_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_State)++;
      break;

      case 4://PFBoffset write #2/4 (bits --> 32-47)
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_SECOND_PFB_OFFSET),((unsigned int)(LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) & 0xffff)), 3);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_State))|(0x100000000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_HSave_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_State)++;
      break;

      case 5://PFBoffset write #3/4 (bits --> 16-31)
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_THIRD_PFB_OFFSET),((unsigned int)((LVAR(AX0_u32_Pos_Fdbk_Offset_Lo)>>16) & 0xffff)), 3);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_State))|(0x100000000*(long long)((LVAR(AX0_u32_Pos_Fdbk_Offset_Lo)>>16) & 0xffff))|(0x10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_HSave_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_State)++;
      break;

      case 6://PFBoffset write #4/4 (LSB bits --> 0-15)
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_FOURTH_PFB_OFFSET),((unsigned int)(LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) & 0xffff)), 3);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_State))|(0x100000000*(long long)(LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) & 0xffff))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_HSave_State) = 0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_State)++;
      break;

      case 7://Checksum Write
         checksum = (unsigned int)(ENDAT_STAMP_PARAMS_READY_FOR_LOAD) + (unsigned int)VAR(AX0_s16_Electrical_Phase_Offset) + ((unsigned int)((LVAR(AX0_s32_Pos_Fdbk_Offset_Hi)>>16) & 0xffff))
                    + ((unsigned int)(LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) & 0xffff)) + ((unsigned int)((LVAR(AX0_u32_Pos_Fdbk_Offset_Lo)>>16) & 0xffff))
                      + ((unsigned int)(LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) & 0xffff));
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_CHECKSUM_OFFSET), checksum, 3);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_State)) | (0x100000000*(long long)checksum) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_HSave_State)=0;
            return ret_val;
         }
         else
            BGVAR(u16_HSave_State)++;
      break;

      case 8://STAMP Write - to state parameters writing is done
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_STAMP_OFFSET), ENDAT_STAMP_PARAMS_READY_FOR_LOAD, 3);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_HSave_Error) = ((0x1000000000000 * (long long)BGVAR(u16_HSave_State)) | (0x100000000*(long long)ENDAT_STAMP_PARAMS_READY_FOR_LOAD) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_HSave_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u64_HSave_Error) = 0;
            BGVAR(u16_HSave_State) = 0;
            return SAL_SUCCESS;
         }
   }//switch

   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: LoadParamsFromEndat
// Description:
//          Called From Background after Endat Handler Init is Done.
//          -- Loads saved parameters from Endat's EEPROM to ram. --
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int LoadParamsFromEndat(int drive)
{
   int ret_val = 0;
   unsigned int checksum = 0;

   switch (BGVAR(u16_Load_Params_From_Endat_State))
   {
      case 0:
         BGVAR(u16_Endat_StampRead_Retry) = 0;
         return SAL_SUCCESS;

      case 1://find OEM partition mapping from word 9 (MRS 0xA1)
         ret_val = HSaveLoadCallEMEMW(drive, ENDAT_PARTITION_INFO_MRS_ADR, ENDAT_PARTITION_INFO_OFFSET, 0, 2);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State)) | (0x100000000 * (long long)ENDAT_STAMP_COPYING_PARAMETERS) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1))) | (long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return ret_val;
         }
         BGVAR(u16_EnDat_Mem_Part1) = BGVAR(u16_Endat_Read_Returned_Value);
         if (0x00FF != (0x00FF & u16_EnDat_Mem_Part1))
         {
            BGVAR(u16_Endat_Params_Mrs_Adr) = 0xA9;
            BGVAR(u16_Endat_OEM_Mem_Offset) = Endat_MEM_Partition_Start_End_Address[(0x000F & u16_EnDat_Mem_Part1)];   //those 4 bits represent the offset
            BGVAR(u16_Load_Params_From_Endat_State) = 3;
         }
         else if  (0xFF00 != (0xFF00 & u16_EnDat_Mem_Part1))
         {
            BGVAR(u16_Endat_Params_Mrs_Adr) = 0xAB;
            BGVAR(u16_Endat_OEM_Mem_Offset) = Endat_MEM_Partition_Start_End_Address[(0x0F00 & u16_EnDat_Mem_Part1)];   //those 4 bits represent the offset
            BGVAR(u16_Load_Params_From_Endat_State) = 3;
         }
         else
            BGVAR(u16_Load_Params_From_Endat_State)++;
      break;

      case 2://find OEM partition mapping from word 10 (MRS 0xA1)
         ret_val = HSaveLoadCallEMEMW(drive, ENDAT_PARTITION_INFO_MRS_ADR + 1, ENDAT_PARTITION_INFO_OFFSET, 0, 2);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State)) | (0x100000000 * (long long)ENDAT_STAMP_COPYING_PARAMETERS) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1))) | (long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return ret_val;
         }
         BGVAR(u16_EnDat_Mem_Part2) = BGVAR(u16_Endat_Read_Returned_Value);
         if (0x00FF != (0x00FF & u16_EnDat_Mem_Part2))
         {
            BGVAR(u16_Endat_Params_Mrs_Adr) = 0xAD;
            BGVAR(u16_Endat_OEM_Mem_Offset) = Endat_MEM_Partition_Start_End_Address[(0x000F & u16_EnDat_Mem_Part2)];   //those 4 bits represent the offset
            BGVAR(u16_Load_Params_From_Endat_State) = 3;
         }
         else if  (0xFF00 != (0xFF00 & u16_EnDat_Mem_Part2))
         {
            BGVAR(u16_Endat_Params_Mrs_Adr) = 0xAF;
            BGVAR(u16_Endat_OEM_Mem_Offset) = Endat_MEM_Partition_Start_End_Address[(0x0F00 & u16_EnDat_Mem_Part2)];   //those 4 bits represent the offset
            BGVAR(u16_Load_Params_From_Endat_State) = 3;
         }
         else
         {
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return FDBK_MEMORY_NOT_PARTITIONED; //no OEM space partitioned to write params
         }
      break;

      case 3://check Stamp to see if params should be loaded
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset)+ ENDAT_STAMP_OFFSET), 0, 2);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State)) | (0x100000000 * (long long)ENDAT_STAMP_COPYING_PARAMETERS) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1))) | (long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return ret_val;
         }
         BGVAR(u16_Endat_Stamp_Value_Mismatch) = 0;
         if ( ENDAT_STAMP_PARAMS_READY_FOR_LOAD == BGVAR(u16_Endat_Read_Returned_Value))
         {
            BGVAR(u16_Load_Params_From_Endat_State)++;
         }
         else if (ENDAT_STAMP_DISREGARD_PARAMETERS == BGVAR(u16_Endat_Read_Returned_Value))
         {
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            BGVAR(s16_DisableInputs) &= ~ENDAT_DIS_MASK;
            return SAL_SUCCESS;
         }
         else if (ENDAT_STAMP_COPYING_PARAMETERS == BGVAR(u16_Endat_Read_Returned_Value))
         {
            if (5 < BGVAR(u16_Endat_StampRead_Retry))
            {
               BGVAR(u16_Load_Params_From_Endat_State) = 0;
               return ENDAT_BUSY_ERROR;
            }
            BGVAR(u16_Endat_StampRead_Retry)++;
         }
         else
         {
            BGVAR(u16_Endat_Stamp_Value_Mismatch) = 1;// continue, use this later Vs Checksum, to decide if this is an error or first encoder use
            BGVAR(u16_Load_Params_From_Endat_State)++;
         }
      break;

      case 4://MPHASE read
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_MPHASE_OFFSET),0, 2);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State)) | (0x100000000 * (long long)VAR(AX0_s16_Electrical_Phase_Offset)) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1))) | (long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Endat_Temp_Electrical_Phase_Offset) = BGVAR(u16_Endat_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Endat_State)++;
         }
      break;

      case 5://PFBoffset read #1/4 (MSB bits --> 48-63)
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_FIRST_PFB_OFFSET),0, 2);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State)) | (0x100000000 * (long long)((LVAR(AX0_s32_Pos_Fdbk_Offset_Hi)>>16) & 0xffff)) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Endat_Temp_First_Pos_Fdbk_Offset) = (long)BGVAR(u16_Endat_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Endat_State)++;
         }
      break;

      case 6://PFBoffset read #2/4 (bits --> 32-47)
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_SECOND_PFB_OFFSET),0, 2);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State))|(0x100000000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Endat_Temp_Second_Pos_Fdbk_Offset) = (long)BGVAR(u16_Endat_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Endat_State)++;
         }
      break;

      case 7://PFBoffset read #3/4 (bits --> 16-31)
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_THIRD_PFB_OFFSET),0, 2);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State))|(0x100000000*(long long)((LVAR(AX0_u32_Pos_Fdbk_Offset_Lo)>>16) & 0xffff))|(0x10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Endat_Temp_Third_Pos_Fdbk_Offset) = (long)BGVAR(u16_Endat_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Endat_State)++;
         }
      break;

      case 8://PFBoffset read #4/4 (LSB bits --> 0-15)
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_FOURTH_PFB_OFFSET),0, 2);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State))|(0x100000000*(long long)(LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) & 0xffff))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return ret_val;
         }
         else
         {
            BGVAR(u16_Endat_Temp_Fourth_Pos_Fdbk_Offset) = (long)BGVAR(u16_Endat_Read_Returned_Value);
            BGVAR(u16_Load_Params_From_Endat_State)++;
         }
      break;

      case 9://Checksum test
         checksum = BGVAR(u16_Endat_Temp_Electrical_Phase_Offset) +  BGVAR(u16_Endat_Temp_First_Pos_Fdbk_Offset)
                     + BGVAR(u16_Endat_Temp_Second_Pos_Fdbk_Offset) + BGVAR(u16_Endat_Temp_Third_Pos_Fdbk_Offset)  + BGVAR(u16_Endat_Temp_Fourth_Pos_Fdbk_Offset)
                       +  ENDAT_STAMP_PARAMS_READY_FOR_LOAD;
         ret_val = HSaveLoadCallEMEMW(drive, BGVAR(u16_Endat_Params_Mrs_Adr), (BGVAR(u16_Endat_OEM_Mem_Offset) + ENDAT_CHECKSUM_OFFSET), 0, 2);
         if (SAL_NOT_FINISHED == ret_val) return ret_val;
         if (SAL_SUCCESS != ret_val)
         {
            BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State)) | (0x100000000*(long long)checksum) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
            BGVAR(u16_Load_Params_From_Endat_State) = 0;
            return ret_val;
         }
         else
         {
            if (checksum == BGVAR(u16_Endat_Read_Returned_Value))
            {
               if (BGVAR(u16_Endat_Stamp_Value_Mismatch))
               {  //for the case of STAMP wrong save/load (checksum passed hence we know the stamp is a problem)
                  BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State)) | (0x100000000*(long long)checksum) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
                  BGVAR(u16_Load_Params_From_Endat_State)=0;
                  return ENDAT_STAMP_MISMATCH;
               }
               else
               {
                  //     copy loaded  parameters
                  VAR(AX0_s16_Electrical_Phase_Offset) = (int)BGVAR(u16_Endat_Temp_Electrical_Phase_Offset); //mphase

                  LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) = (long)BGVAR(u16_Endat_Temp_First_Pos_Fdbk_Offset);//PFBoffset copy #1/4 (MSB bits --> 48-63)
                  LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) <<= 16;

                  LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) |= (long)BGVAR(u16_Endat_Temp_Second_Pos_Fdbk_Offset);//PFBoffset copy #2/4 (bits --> 32-47)

                  LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) = (long)BGVAR(u16_Endat_Temp_Third_Pos_Fdbk_Offset);//PFBoffset copy #3/4 (bits --> 16-31)
                  LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) <<= 16;

                  LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) |= (long)BGVAR(u16_Endat_Temp_Fourth_Pos_Fdbk_Offset);//PFBoffset copy #4/4 (LSB bits --> 0-15)
                  BGVAR(u16_Load_Params_From_Endat_State) = 0;
                  BGVAR(u64_Load_Error) = 0;
                  BGVAR(s16_DisableInputs) &= ~ENDAT_DIS_MASK;
                  return SAL_SUCCESS;
               }
            }
            else //CheckSum Failed
            {
               BGVAR(u16_Load_Params_From_Endat_State) = 0;
               if (BGVAR(u16_Endat_Stamp_Value_Mismatch))
               {  //case of endat with no params saved (checksum failed hence we know STAMP was never written)
                  BGVAR(u64_Load_Error) = 0;
                  BGVAR(s16_DisableInputs) &= ~ENDAT_DIS_MASK;
                  return SAL_SUCCESS;
               }
               else
               {  //CheckSum Failed, don't copy params
                  BGVAR(u64_Load_Error) = ((0x1000000000000 * (long long)BGVAR(u16_Load_Params_From_Endat_State)) | (0x100000000*(long long)checksum) | (0x10000*(long long)(BGVAR(u16_Dummy_Ptr1)))|(long long)BGVAR(u16_Dummy_Ptr2));
                  return CHECKSUM_ERROR;
               }
            }
         }
   }// switch

   return SAL_NOT_FINISHED;
}
