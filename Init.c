#include "DSP2834x_Device.h"
#include "DSP2834x_Device.h"
#include "utility.h"
#include <string.h>
#include "objects.h"

#include "MultiAxis.def"
#include "FltCntrl.def"
#include "Design.def"
#include "Endat.def"
#include "Display.def"
#include "Err_Hndl.def"
#include "Current.def"
#include "Homing.def"
#include "SysInit.def"
#include "FPGA.def"
#include "Exe_IO.def"
#include "I2c.def"
#include "PtpGenerator.def"
#include "Position.def"
#include "SensorlessBG.def"
#include "Init.def"
#include "DevTools.def"
#include "Velocity.def"
#include "ExFbVar.def"
#include "Modbus_comm.def"
#include "Flash.def"
#include "Drive_Table.def"
#include "Analog_If.def"

#include "Extrn_Asm.var"
#include "AN_FLTR.VAR"
#include "Drive.var"
#include "Units.var"
#include "User_Var.var"
#include "Zeroing.var"
#include "FltCntrl.var"
#include "Burnin.var"
#include "Foldback.var"
#include "Err_Hndl.var"
#include "ModCntrl.var"
#include "Endat.var"
#include "EncCnfg.var"
#include "FlashHandle.var"
#include "Display.var"
#include "PhaseFind.var"
#include "Exe_IO.var"
#include "Ser_Comm.var"
#include "Modbus_Comm.var"
#include "I2c.var"
#include "Velocity.var"
#include "PtpGenerator.var"
#include "Position.var"
#include "Motor.var"
#include "Exe_Hndl.var"
#include "SensorlessBG.var"
#include "Init.var"
#include "Homing.var"
#include "ExFbVar.var"
#include "AutoTune.var"
#include "Lxm_profile.var"

#include "Prototypes.pro"



extern IDENTITY_T p301_identity;
extern VALUE_DESC_T CO_CONST p301_manu_device_name_desc[];

long s32_Debug_Timer_Start, s32_Debug_Timer_End;
unsigned int u16_Debug_Counter, u16_Debug_FPGA, u16_Debug_Vbus_HW;

unsigned int *RxDpramPtrs_T[RX_DPRAM_PTRS_NUM] = {
     (unsigned int*)DPRAM_RX_LP_HEAD_CTRL_IDX_ADDR,
     (unsigned int*)DPRAM_RX_LP_HEAD_DATA_IDX_ADDR,
     (unsigned int*)DPRAM_TX_LP_TAIL_CTRL_IDX_ADDR,
     (unsigned int*)DPRAM_TX_LP_TAIL_DATA_IDX_ADDR,
     (unsigned int*)MOTI_COMMANDS_REGISTER,
     (unsigned int*)DPRAM_RX_HP_PDO_FIFO_ADDR};

unsigned int *TxDpramPtrs_T[TX_DPRAM_PTRS_NUM] = {
     (unsigned int*)DPRAM_TX_LP_HEAD_CTRL_IDX_ADDR,
     (unsigned int*)DPRAM_TX_LP_HEAD_DATA_IDX_ADDR,
     (unsigned int*)DPRAM_RX_LP_TAIL_CTRL_IDX_ADDR,
     (unsigned int*)DPRAM_RX_LP_TAIL_DATA_IDX_ADDR,
     (unsigned int*)TOMI_STATUS_REGISTER,
     (unsigned int*)DPRAM_TX_HP_PDO_FIFO_ADDR };

extern   void SET_DP_TO_AXIS_0(void);

EC_RPDO_ENTRY_T p_rpdo_entries[RPDO_MAX_NUM];
EC_TPDO_ENTRY_T p_tpdo_entries[TPDO_MAX_NUM];


typedef struct
{
   char s8_vendor_key1;
   char s8_vendor_key2;
   unsigned int u16_vendor_hi;
   unsigned int u16_vendor_lo;
   unsigned int u16_brand_addr1;
   unsigned int u16_brand_addr2;
   unsigned int u16_brand_addr3;
   unsigned int u16_brand_addr4;
} PrivateLabelStruct;

const PrivateLabelStruct Private_Label_Table[] = {
   {'C', 'D', 0x0002, 0x00E1, 0x0043 ,0x0044 ,0x0048, 0x0044}, //CDHD
   {'M', 'T', 0x0002, 0x00BA, 0x0020 ,0x004D ,0x0054, 0x0020}, //PBA
   {'A', 'S', 0x0002, 0x00E1, 0x0020 ,0x0041 ,0x0053, 0x0044}, //AKRIBIS
   {'F', 'P', 0x0006, 0x0078, 0x0046 ,0x0050 ,0x0052, 0x004F}, //MPC
   {'D', 'D', 0x0002, 0x00E1, 0x0044 ,0x0044 ,0x0048, 0x0044}, //DDHD
   {'R', 'T', 0x0002, 0x00E1, 0x00EC ,0x0052 ,0x0048, 0x0044}  //RENTONG
};


asm(" .c28_amode     ");  //  ; Tell assembler we are in C28x address mode

#pragma CODE_SECTION(SCIB_SendByte, "ramfunc_2");
void SCIB_SendByte(unsigned int data)
{
   while (ScibRegs.SCICTL2.bit.TXRDY != 1) {}
   ScibRegs.SCITXBUF = data;
}
/*
// use the above function for debug by using the following lines
//   SCIB_SendByte(CR);    // AZ debug
//   SCIB_SendByte(LF);    // AZ debug
//   SCIB_SendByte('1');   // AZ debug
*/

//**********************************************************
// Function Name: ShowSchneiderBootUpMsg
// Description:
//    This function Shows the boot-up message on the 7-segments
//    for a Lexium Hardware. This function is only supposed to be
//    called for a Schneider hardware.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void ShowSchneiderBootUpMsg(int drive)
{
   int s16_j;

   //REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Clear array.
   for (s16_j = 0; s16_j < 5; s16_j++)
   {
      // First clear the 7-segment completely
      BGVAR(u16_Local_Hmi_Display_Array)[s16_j] = NONE;
   }
   // Now write the string according to the mode of operation.
   switch(BGVAR(u16_P1_01_CTL_Current))
   {
      case (0x0000):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = P_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = t_LET;
      break;
      case (0x0001):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = P_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = S_LET;
      break;
      case (0x0002):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = S_LET;
      break;
      case (0x0003):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = t_LET;
      break;
      case (0x0004):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = S_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = TWO;    // 2 represents also a Z, see HMI character set
      break;
      case (0x0005):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = t_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = TWO;    // 2 represents also a Z, see HMI character set
      break;
      case (0x0006):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = P_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = t_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = DASH_CHAR;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = S_LET;
      break;
      case (0x0007):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = P_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = t_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = DASH_CHAR;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = t_LET;
      break;
      case (0x0008):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = P_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = S_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = DASH_CHAR;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = S_LET;
      break;
      case (0x0009):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = P_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = S_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = DASH_CHAR;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = t_LET;
      break;
      case (0x000a):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = S_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = DASH_CHAR;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = t_LET;
      break;
      case (0x000b):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = C_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = A_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = n_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = o_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[0] = P_LET;
      break;
      case (0x000d):  // Was defined by SE but then they regret. It is not in P1-01 range.
         BGVAR(u16_Local_Hmi_Display_Array)[4] = P_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = t_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = DASH_CHAR;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = P_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[0] = S_LET;
      break;
      case (0x0020):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = S_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = r_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = C_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = o_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[0] = S_LET;
      break;
      case (0x0030):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = E_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = t_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = H_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = I_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[0] = P_LET;
      break;
      case (0x0040):
         BGVAR(u16_Local_Hmi_Display_Array)[4] = E_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = t_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = C_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = A_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[0] = t_LET;
      break;
      default: // Unknown OPMODE, not supposed to happen -> Write HELLO
         BGVAR(u16_Local_Hmi_Display_Array)[4] = H_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[3] = E_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[2] = L_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[1] = L_LET;
         BGVAR(u16_Local_Hmi_Display_Array)[0] = ZERO;
      break;
   }
   for (s16_j = 0; s16_j < 5; s16_j++)
   {
      if (drive == 0) // For axis 0
      {
         // Write array content to 7-segments
         HWDisplay(BGVAR(u16_Local_Hmi_Display_Array)[s16_j] ,s16_j);
      }
      else
      {
         // Here insert code in case of dual axis Lexium
      }
   }
}


void CopyEEpromDataBetweenDDHDFpgas(void)
{
   unsigned long u32_timer;
   unsigned int u16_checksum, u16_index;
   if (u16_Axis_Num == 0)
   {
      // Check that the second FPGA is running.
      u16_Axis_To_Axis_Timeout = 0;
      AX0_u16_Axis_To_Axis_Bg_Word_Num = 0;
      *(unsigned int *)FPGA_BUFFER_BG_COUNTER_REG_ADD = 0;

      while (u16_Axis_To_Axis_Timeout < 1000)   // 8 sec timeout, or other DSP is ready
      {
         // Set the transmit request (triggers the transmit), which triggers also transmit from the other axis
         *(unsigned int *)FPGA_DSP_TRANSMIT_REQ_REG_ADD = 1;
         u32_timer = CpuTimer0Regs.TIM.all;        // wait 2ms
         while ((u32_timer - CpuTimer0Regs.TIM.all) < 600000) {}  // 600,000 = 2ms

         if (*(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD == 0xAC13)    // check if the FPGA buffer is ready
            u16_Axis_To_Axis_Timeout = 10000;

         // Clear the transmit request and wait 1 ms
         *(unsigned int *)FPGA_DSP_TRANSMIT_REQ_REG_ADD = 0;
         u32_timer = CpuTimer0Regs.TIM.all;        // wait 2ms
         while ((u32_timer - CpuTimer0Regs.TIM.all) < 600000)   {}  // 600,000 = 2ms

         u16_Axis_To_Axis_Timeout++;
      }

      if (u16_Axis_To_Axis_Timeout >= 10000)
      {
         // the FPGA buffer consists of 64 Words blocks. Use selector bits to select to the desired block.
         *(unsigned int *)FPGA_AXIS1_TO_AXIS2_MSB_ADDR_REG_ADD = 0;   // write the selector bits

         u16_checksum = 0;

         for (u16_index = 0; u16_index < 43; u16_index++)
         {
            *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 8 + u16_index) = u16_Power1_Board_Eeprom_Buffer[u16_index];
            u16_checksum += u16_Power1_Board_Eeprom_Buffer[u16_index];
         }

         for (u16_index = 43; u16_index < 56; u16_index++)
         {
            *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 8 + u16_index) = u16_Power1_Board_Eeprom_Buffer[u16_index + 42];
            u16_checksum += u16_Power1_Board_Eeprom_Buffer[u16_index + 42];
         }

         *(unsigned int *)FPGA_AXIS1_TO_AXIS2_MSB_ADDR_REG_ADD = 1;   // write the selector bits

         for (u16_index = 56; u16_index < 85; u16_index++)
         {
            *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + u16_index - 56) = u16_Power1_Board_Eeprom_Buffer[u16_index + 42];
            u16_checksum += u16_Power1_Board_Eeprom_Buffer[u16_index + 42];
         }

         *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 29) = BGVAR(u16_Power_Board_Eeprom_Fault);
         u16_checksum += BGVAR(u16_Power_Board_Eeprom_Fault);
         *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 30) = u16_checksum;

         *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 31) = 0x1234;

         // Write number of words to transmit to indicate the FPGA that the data is ready to be transferred to the other FPGA (it is the FPGA
         // responsibility to make sure the other FPGA is ready).
         *(unsigned int *)FPGA_BUFFER_BG_COUNTER_REG_ADD = 89;   // Write number of words to transmit + 1

         // Set the transmit request (triggers the transmit)
         *(unsigned int *)FPGA_DSP_TRANSMIT_REQ_REG_ADD = 1;

         *(unsigned int *)FPGA_AXIS1_TO_AXIS2_MSB_ADDR_REG_ADD = 0;   // write the selector bits

         // wait 2ms to be sure data transmitted, then override FPGA_AXIS2_TO_AXIS1_BUFFER_ADD (prepare it to BG data transfer)
         u32_timer = CpuTimer0Regs.TIM.all;
         while ((u32_timer - CpuTimer0Regs.TIM.all) < 600000)  {}  // 600,000 = 2ms
         *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 8) = 0;
         *(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD = 0;
      }
      else
      {
         u16_FPGA_Buffer_Not_Ready = 1;
      }
   }
   else
   {
      // Check that the data is ready in the FPGA. Arm timeout, and indicate fault if timeout expires in u16_Power_Board_Eeprom_Fault[0]
      *(unsigned int *)FPGA_RECEIVE_BUFFER_READY_REG_ADD = 1;               // clear the receive done bit
      *(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD = 0xAC13;
      *(unsigned int *)FPGA_RECEIVE_BUFFER_READY_REG_ADD = 0;               // clear the receive done bit

      u32_timer = CpuTimer0Regs.TIM.all;        // capture Timer0 value

      *(unsigned int *)FPGA_AXIS1_TO_AXIS2_MSB_ADDR_REG_ADD = 1;   // write the selector bits
      while ( (*(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 31) != 0x1234) &&
              ((u32_timer - CpuTimer0Regs.TIM.all) < 900000000)                    ) {}  // 900,000,000 = 3 sec

      if ((*(unsigned int *)FPGA_RECEIVE_BUFFER_READY_REG_ADD) & 0x0001)    // check if the FPGA buffer is ready
      {
         *(unsigned int *)FPGA_RECEIVE_BUFFER_READY_REG_ADD = 1;            // clear the receive done bit

         // the FPGA buffer consists of 64 Words blocks. Use selector bits to select to the desired block.
         *(unsigned int *)FPGA_AXIS1_TO_AXIS2_MSB_ADDR_REG_ADD = 0;   // write the selector bits

         u16_checksum = 0;

         for (u16_index = 0; u16_index < 56; u16_index++)
         {
            u16_Power1_Board_Eeprom_Buffer[u16_index] = *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 8 + u16_index);
            u16_checksum += u16_Power1_Board_Eeprom_Buffer[u16_index];
         }

         *(unsigned int *)FPGA_AXIS1_TO_AXIS2_MSB_ADDR_REG_ADD = 1;   // write the selector bits

         for (u16_index = 56; u16_index < 85; u16_index++)
         {
            u16_Power1_Board_Eeprom_Buffer[u16_index] = *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + u16_index - 56);
            u16_checksum += u16_Power1_Board_Eeprom_Buffer[u16_index];
         }

         BGVAR(u16_Power_Board_Eeprom_Fault) = *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 29);
         u16_checksum += BGVAR(u16_Power_Board_Eeprom_Fault);
         u16_checksum -= *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 30);

         BGVAR(u16_Power_Eeprom_Size_Byte) = u16_Power1_Board_Eeprom_Buffer[1];
         if (256 < BGVAR(u16_Power_Eeprom_Size_Byte ))
            BGVAR(u16_Power_Eeprom_Size_Byte ) = 256;

         if (u16_checksum != 0)
         {
            BGVAR(u16_Power_Board_Eeprom_Fault) |= FPGA_BUFFER_CHECKSUM_MASK;
         }

         // Indicate to the FPGA that the data was read
         *(unsigned int *)FPGA_RECEIVE_BUFFER_READY_REG_ADD = 0;      // clear the receive done bit

         *(unsigned int *)FPGA_AXIS1_TO_AXIS2_MSB_ADDR_REG_ADD = 0;   // write the selector bits
      }
      else
      {
         BGVAR(u16_Power_Board_Eeprom_Fault) |= FPGA_BUFFER_NOT_READY_MASK;
      }
      *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 8) = 0;
      *(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD = 0xAC14;//signal AXIS1 the reading is done, and available for BG transfers.
   }
}



unsigned long u32_Timer_Debug1, u32_Timer_Debug2, u32_Timer_Debug3, u32_Timer_Debug4, u32_Timer1_Debug1;
void Initialization(void)
{
   int s16_i, s16_j = 0, s16_temp_index = 0, s16_control = 1, drive = 0;
   long s32_timer, s32_timer_2;
   unsigned int u16_FPGA_flt_indication, u16_fpga_current_ver_lo = 0, u16_fpga_current_ver_hi = 0;
   unsigned long u32_timer;

   // Disable CPU interrupts and clear all CPU interrupt flags
   DINT;
   IER = 0x0000;
   IFR = 0x0000;

   AX0_s16_MTS_Task_Ptr = (int)((long)&MTS_1_TASKS & 0xFFFF);

   DisableDog();

   // Set the device mode from reset to C28x
   asm(" nop");
   asm(" SETC   OBJMODE ");  //  ; Enable C28x Object Mode (5 cycles)
   asm(" CLRC   AMODE   ");  //  ; Enable C28x Address Mode (1 cycle)
   asm(" SETC   M0M1MAP ");  //  ; (5 cycles)

   InitCpuTimers();
   u32_Timer_Debug1 = CpuTimer0Regs.TIM.all;        // capture Timer0 value
   u32_Timer1_Debug1 = CpuTimer1Regs.TIM.all;       // capture Timer1 value (started at the beginning of the resident code)

   InitGpio();
   InitSysCtrl();        // Includes InitRam() and MemCopy()
   u16_Axis_Num = (unsigned int)GpioDataRegs.GPBDAT.bit.GPIO53;  // relevant for DDHD

   u16_DSP_Type = DSP_TMS320C28344; // OLD DSP
   if (*(int *)DSP_ID_ADDR == 0xFFD0)   // read DSP ID register
   {
      u16_DSP_Type = DSP_TMS320C28346; // NEW DSP
   }

   u16_Flash_Type = 0;
   u16_Support_Axis_1_On_Flash = 1;
   s16_Number_Of_Flashes = 1;

   ReadProductID(0);  // Read u16_Vendor_ID[0] and u16_Device_ID[0]

   u32_Product_ID[0] = 0;
   u32_Product_ID[0] = (unsigned long)u16_Vendor_ID[0];
   u32_Product_ID[0] |= ((unsigned long)u16_Device_ID[0] << 16);

   if (  (u16_Vendor_ID[0] == 0x00BF)   /* SST Vendor ID */            &&
        ( (u16_Device_ID[0] == 0x234A) || (u16_Device_ID[0] == 0x234B) ||
          (u16_Device_ID[0] == 0x235A) || (u16_Device_ID[0] == 0x235B)   )  )
   {
      u16_Erase_32KW_Code = 0x50;
      u16_Erase__2KW_Code = 0x30;
   }
   else
   {
      u16_Erase_32KW_Code = 0x30;
      u16_Erase__2KW_Code = 0x50;
   }

   if ( ( (u16_Vendor_ID[0] == 0x0020) && (u16_Device_ID[0] == 0x22FD) ) ||      // MICRON - M29W640FB
        ( (u16_Vendor_ID[0] == 0x00C2) && (u16_Device_ID[0] == 0x22CB) ) ||      // MACRONIX - MX29LV640EB
        ( (u16_Vendor_ID[0] == 0x0001) && (u16_Device_ID[0] == 0x227E) ) ||      // SPANSION - S29GL064S70BHI040
        ( (u16_Vendor_ID[0] == 0x00BF) && (u16_Device_ID[0] == 0x227E) )   )   // MICROCHIP - SST38VF6403B
   {
      // 4M word device = 64Mb
      s16_Num_Of_32KW_Blocks = 64;   // access only the lower 32KW of this device
      s16_Num_Of__2KW_Sectors = 0;   // no support for 2K word sectors
      u16_Support_Axis_1_On_Flash = 0;
      u16_Flash_Type = 1;            // 8 4KW bottom blocks, and 32KW blocks all the rest
   }
   else if ( (u16_Vendor_ID[0] == 0x00BF) && ((u16_Device_ID[0] & 0xFFF0) == 0x2350)  )   /* SST Vendor ID */
   {
      // 2M word device = 32Mb
      s16_Num_Of_32KW_Blocks = 64;
      s16_Num_Of__2KW_Sectors = 1024;
   }
   else
   {  // 1M word device = 16Mb
      s16_Num_Of_32KW_Blocks = 32;
      s16_Num_Of__2KW_Sectors = 512;
   }

   if ( (u16_Vendor_ID[0] == 0x0001) && (u16_Device_ID[0] == 0x2249) )
   {
      u16_Support_Axis_1_On_Flash = 0;
   }

   Security_Released = -1;
   Display_State = 0;
   Test_Led_On = 0;
   drive = 0;
   BGVAR(u16_Prev_Script_Inp_State) = AX0_u16_Mode_Input_Arr[SCRIPT_INP];
   BGVAR(u16_Prev_Script_Sec_Inp_State) = AX0_u16_Mode_Input_Arr[SCRIPT_SEC_INP];

   u16_Digital_Outputs = 0;
   s32_Vbus_Timer = Cntr_1mS;

   BGVAR(u8_SwVersion)[0] = 0;

   for (drive = 0; drive < 2; ++drive)
   {
      // init foldback vars
      BGVAR(s32_FoldbackTimer) = Cntr_1mS;
      BGVAR(s16_HwEnableResetFaultsFlag) = 0;
      BGVAR(s16_SwEnableResetFaultsFlag) = 0;
      ResetCommandsAndStates(drive);
      BGVAR(s16_DisableInputs) |= SENSOR_ADJ_DIS_MASK;
      BGVAR(u8_Sensor_Adjust_State) = ADJ_WAIT;
      BGVAR(s16_Most_Recent_Error) = 0;
      BGVAR(s16_New_Error) = 0;
      BGVAR(s16_SinZeroSave) = -1;
      BGVAR(s16_SineZeroSaveState) = 0;
      BGVAR(s16_SineCDInitState) = 0;
      BGVAR(s16_Zero_Mode) = 0;
      BGVAR(s16_Endat_Init_State) = ENDAT_IDLE_STATE;
      BGVAR(s16_Flash_Checksum_Fault_Flag) = 0;
      BGVAR(u16_Display_Code) = ALL;
      BGVAR(u16_Display_Char_Number) = 1;
      BGVAR(s16_ControllerType) = CURRENT_CONTROL;
      BGVAR(s16_Endat_Machine_Retry) = 0;
      //BGVAR(s16_Cap_Mode) = 0;
      BGVAR(s16_Sensor_Cntr) = 0;
      BGVAR(s32_Sensor_U) = 0;
      BGVAR(s32_Sensor_V) = 0;
      BGVAR(s16_Adjust_Reference_State) = 0;
      BGVAR(u16_PhaseFind_Bits) = 0;
      BGVAR(s64_Faults_Mask) = INIT_FAULTS_MASK;
      BGVAR(s64_Faults_Mask_2) = INIT_FAULTS_2_MASK;
      BGVAR(u64_Fault_Action_Disable) = 0xFFFFFFFFFFFFFFFFLL; //on init any fault cause to disable
      BGVAR(u64_Fault_Action_Disable_2) = 0xFFFFFFFFFFFFFFFFLL; //on init any fault cause to disable
      BGVAR(u64_Fault_Action_Active_Disable) = ACTIVE_DISABLE_FAULTS_MASK;
      BGVAR(u64_Fault_Action_Active_Disable_2) = ACTIVE_DISABLE_FAULTS_2_MASK;
      BGVAR(s32_Vmax) = 0;
      BGVAR(u8_PhaseFind_State) = PHASE_FIND_IDLE;
      BGVAR(u16_BurninFrq) = 40;
      SalBurninFrqCommand(BGVAR(u16_BurninFrq),drive); // update rt varaible
      BGVAR(s64_Move_Inc_Position) = 0;
      BGVAR(s16_Move_Inc_Mode) = 0;
      BGVAR(s32_Move_Inc_Speed) = 0;
      BGVAR(s64_Move_Abs_Position) = 0;
      BGVAR(s16_Move_Abs_Mode) = 0;
      BGVAR(s32_Move_Abs_Speed) = 0;
      BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_IDLE;
      BGVAR(s16_Sensorless_Handler_State) = SL_STATE_IDLE;
      BGVAR(u8_Sl_Mode) = 0;
      BGVAR(u8_Sl_Vel_Mode) = 0;
      BGVAR(u8_Sl_Comm_Mode) = 0;
      BGVAR(s32_Fb_Target_Velocity_For_Pos) = 0L;
      BGVAR(s32_Fb_Target_Velocity_For_Vel) = 0L;
      BGVAR(s32_Fb_Target_Velocity_For_Vel_Raw_Data) = 0L;
      BGVAR(s16_Fb_Target_Torque)   = 0;
      BGVAR(s64_Fb_Target_Position) = 0LL;
//      BGVAR(s64_fb_position_window) = 0LL;
      BGVAR(u16_homing_pcmd_settled) = 0;
      BGVAR(u8_Script_Status_Str[0]) = 0;
      BGVAR(s16_PfbBackupWR) = PFB_BACKUP_WR_DONE;
      BGVAR(s16_PfbBackupRD) = 0;
      BGVAR(s64_Pfb_Backup) = 0;
//      BGVAR(u16_P2_27_Gain_Switching_Ctrl) = 0; // Initialized where the variable is defined
//      BGVAR(u16_P2_50_Cclr_Trigger) = 0;        // Initialized where the variable is defined
      BGVAR(s64_Position_Modulo)[0] = 0LL;
      BGVAR(s64_Position_Modulo)[1] = 0LL;

      // Here initialize variables for the runaway check
      BGVAR(u32_Runaway_Check_Vel_Thresh) = 536870; // 60[rpm] = 1[rps] = 536870[Counts32/125us]
      BGVAR(u16_Runaway_Error_Counter_Thresh) = 0;  // De-activate the feature per default

      BGVAR(u32_MotorFileName) = 0;
   }
   drive = 0;

   AX0_s16_Cycle_Dir = 0x7FFF;
   AX0_u16_Pos_Loop_Tsp = TSP_250_USEC;

   BGVAR(u64_CAN_Max_Acc) = MAX_ACC_DEC_TSP250;
   BGVAR(u64_CAN_Max_Dec) = MAX_ACC_DEC_TSP250;

   AX0_u32_Pos_Buff_Addr = (unsigned long)&s64_Pos_Buff;
   AX0_u32_Ptp_Pos_Buff_Addr_Vel_Dp6 = (unsigned long)&s64_Pos_Buff; // Used to save execution time

   // Here initialize the variables, which are used for the moving average filter
   // between the ramp-generator and the velocity loop.
   AX0_s32_Vel_Moving_Average_Sum = 0;
   AX0_u16_Vel_Moving_Average_Buffer_Depth = 1;
   AX0_s16_Vel_Moving_Average_Init_Value = 0;
   AX0_s16_Vel_Moving_Average_Init_Cntr = 1;
   AX0_u16_Vel_Moving_Average_Shift_Right  = 0;
   AX0_s32_Vel_Moving_Average_Fix = 0;

   // Here initialize the variable used for the "encoder frequency too high" error
   AX0_u32_Max_Speed_Encoder_Simul = 0x7FFFFFFF;

   // Here initialize variables used in divisions within the elctronic gearing code
   AX0_u32_Xencres = 1;
   AX0_s32_Gearin  = 1;
   AX0_u32_Gearout = 1;

   //Clearing the DriveScript contents
   for (drive = 0; drive < 2; drive++)
   {
      for (s16_j = 0; s16_j < 32; s16_j++)
      {
         BGVAR(u16_Script)[s16_j * 128] = ('~' | ('~' << 8));
         BGVAR(u16_Script)[s16_j * 128 + 1] = 0;
      }

      for (s16_j = 0; s16_j < 4; s16_j++)
      {
         BGVAR(s32_Sl_Speeds_For_User)[s16_j] = 0L;
      }

      for (s16_j = 0; s16_j < HISTOGRAM_SIZE; s16_j++)
      {
         BGVAR(s32_NlTune_Histogram)[s16_j] = 0L;
      }
   }
   drive = 0;


   AX0_s16_Pos_Polarity = 1;
   AX0_s16_Vel_Polarity = 1;

   AX0_s16_Cmd_Invert_On = 0;
   AX0_s16_Zclamp_On = 0;

   AX0_u32_Const_7fff_ffff = 0x7fffffff;
   AX0_u32_Const_ffff_ffff = 0xffffffff;
   AX0_u32_Const_8000_0001 = 0x80000001;

   AX0_u16_Int_Clr_Sample = 1;

   AX0_s32_Out_To_In_Vel_Fix = 1L;
   AX0_u16_Out_To_In_Vel_Shr = 0;

   // to be removed after testing
   //AX0_s16_AT_Safe_Window_Cntr = 0x7fff; // Turn off safe window counter
   //AX1_s16_AT_Safe_Window_Cntr = 0x7fff; // Turn off safe window counter

   AX0_s16_Stopped = -1;
   AX0_u32_Max_Time_Stamp_Period = 0x0;
   AX0_u32_Min_Time_Stamp_Period = 0xFFFFFFFF;
   AX0_u16_First_Time_Stamp_Location = 0;
   AX0_u16_Second_Time_Stamp_Location = 9;
   AX0_u16_MtsIsr_TimeStamp_Iteration = 9;

   AX0_u32_Motor_Stopped_Threshold_Lo = 1193046LL;   // 0.1 deg position threshold for motor stopped algorithm
   AX0_u16_Motor_Stopped_Time_ms = 1;                // 1 ms for motor stopped algorithm

   u16_Sudden_Stop_Base_Threshold = 4;

   for (s16_i = 0; s16_i < VBUS_BUFFER_SIZE; ++s16_i) u16_Vbus_Array[s16_i] = 0;

   // Set Units strings default
   for (drive = 0; drive < 2; drive++)
   {
      strcpy(BGVAR(s8_Units_Vel_Out_Loop), "rps");
      strcpy(BGVAR(s8_Units_Vel_Ptp), "rps");
      strcpy(BGVAR(s8_Units_Vel_In_Loop), "rps");
      strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale), "rps/V");
      strcpy(BGVAR(s8_Units_Pos), "rev");
      strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "rps/s");
      strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos), "rps/s");
      strcpy(BGVAR(s8_Units_Cur), "A");
      strcpy(BGVAR(s8_Units_Cur_Scale), "A/V");
      strcpy(BGVAR(s8_Units_Cur_Sec), "A/s");
      strcpy(BGVAR(s8_Units_Analog_IO), "V");
      strcpy(BGVAR(s8_Units_PWM_Volts), "V");
      strcpy(BGVAR(s8_Units_Vel_Out_Loop_Rot_Lin_RPM), "rpm");
      strcpy(BGVAR(s8_Units_Vel_Out_Loop_Rot_Lin_RPS), "rps");
      strcpy(BGVAR(s8_Units_Pos_Rot_Lin), "rev");
      strcpy(BGVAR(s8_Units_Rot_Lin_AMP_VEL), "A/rps");
      strcpy(BGVAR(s8_Units_Rot_Lin_MECHANGLE), "65536/rev");
      strcpy(BGVAR(s8_Units_Rot_Lin_KPP), "rps/rev");
      strcpy(BGVAR(s8_Units_Rot_Lin_ENCRES), "LPR");

      // Schneider units
      strcpy(BGVAR(s8_Units_Vel_0_1_RPM_Out_Loop), "0.1*rpm");   // "0.1rpm" unit
      strcpy(BGVAR(s8_Units_Vel_1000_RPM_Out_Loop), "rpm_fix/1000");   // "rpm * 1000" unit to fix the parser division by 1000
      strcpy(BGVAR(s8_Units_10_ms), "10*ms");   // "10ms" unit
      strcpy(BGVAR(s8_Units_0_5_ms), "0.5*ms");   // "0.5ms" unit
      strcpy(BGVAR(s8_Units_ms_for_Vel_ACC_DEC_6000_rpm), "ms_6000rpm<->0_vel");   // "acc dec in ms to and from 6000 rpm for velocity in loop
      strcpy(BGVAR(s8_Units_ms_for_Pos_ACC_DEC_6000_rpm), "ms_6000rpm<->0_pos");   // "acc dec in ms to and from 6000 rpm for position
      strcpy(BGVAR(s8_Units_PULSE), "PULSE");   // Position Pulse units
      strcpy(BGVAR(s8_Units_PUU), "PUU");   // Position Pulse User units
      strcpy(BGVAR(s8_Units_SFB_Pos), "SFB user unit");
      strcpy(BGVAR(s8_Units_SFB_Vel), "(SFB user unit)/s");
   }

   //PCOM arrays init
   for(s16_i=0;s16_i<PCOM_TABLE_SIZE;s16_i++)
   {
      s64_Pcom_Table1_Arr[s16_i] = 0LL;
      s64_Pcom_Table2_Arr[s16_i] = 0LL;
   }

   drive = 0;

   InitPieCtrl();

   DMAInitialize();

   InitECap();
   InitEPwm();
   InitEQep();
   InitI2C();
   InitMcbsp();

   InitSpi();
   InitXintf();          // should be called after MemCopy() since it runs from RAM
   InitSpi();

   EepromStartControl();  // should be called before LoadFpga(), since it holds the information which FPGA to load. It does not control the Write Protect signal

   // Read parameters from control EEPROM
   if (u16_Control_Board_Eeprom_Fault & (NO_READ_AT_INIT_FAULT_MASK | PARAM_STAMP_FAULT_MASK | PARAM_CHECKSUM_FAULT_MASK) )
   {
      u16_Fpga_Size = 0; // 9;
      u16_Product = 255;
      u16_Board_Type = 0; //1;
      // u16_Fpga_Image_Type = 5;
   }
   else
   {
      u16_Fpga_Size = (u16_Control_Board_Eeprom_Buffer[FPGA_SIZE__BOARD_TYPE__ADDR] >> 8) & 0xFF;
      u16_Product = (u16_Control_Board_Eeprom_Buffer[MORE_DIG_OUTPUTS__PRODUCT__ADDR] & 0xFF);
      u16_Board_Type = u16_Control_Board_Eeprom_Buffer[FPGA_SIZE__BOARD_TYPE__ADDR] & 0xFF;
      u16_Fpga_Image_Type = ((u16_Control_Board_Eeprom_Buffer[FPGA_IMAGE_TYPE__DRIVE_FW_CODE_TYPE__ADDR] >> 8) & 0xFF);
   }

   /*if ((u16_Product == SHNDR_HW) && (u16_Flash_Type == 1) && (u16_DSP_Type == 1))   // 2 * 64Mb flash
   {
      ReadProductID(1);  // Read u16_Vendor_ID and u16_Device_ID of the second flash (if exists) to show in INFO command

      u32_Product_ID[1] = 0;
      u32_Product_ID[1] = (unsigned long)u16_Vendor_ID[1];
      u32_Product_ID[1] |= ((unsigned long)u16_Device_ID[1] << 16);

      s16_Number_Of_Flashes = 2;
   }*/

   /*    Num Of Probes assignment    */
   if ((u16_Product == SHNDR_HW) || (u16_Product == SHNDR))
   {
      BGVAR(u16_Num_Of_Probes_On_Drive) = MAX_PROBE_NUM_ON_SHNDR;

      BGVAR(u16_HdTune_Adv_Mode) = 1;

      if (u16_Flash_Type == 1)   // 2 * 64Mb flash
      {
         BGVAR(u16_Forceable_Dig_Outputs) = 15;
         BGVAR(u16_P1_01_CTL_At_SPI_Flash) = SerialFlashReadByte(PARAM_P1_01_ADDR);
         switch (BGVAR(u16_P1_01_CTL_At_SPI_Flash))
         {
            case SE_OPMODE_SERCOS:
               u16_Fieldbus_Type = FB_SERCOS;
            break;

            case SE_OPMODE_ETHERNET_IP:
               u16_Fieldbus_Type = FB_ETHERNET_IP;
            break;

            default:
               u16_Fieldbus_Type = FB_ETHERCAT;
            break;
         }
      }
   }
   else
   {
      if(u16_DSP_Type == DSP_TMS320C28344)
      {
         BGVAR(u16_Num_Of_Probes_On_Drive) = MAX_PROBE_NUM_ON_CDHD_1;
      }
      else
      {
         BGVAR(u16_Num_Of_Probes_On_Drive) = MAX_PROBE_NUM_ON_CDHD_2;
      }

   }

   InitSerCommunication();
   InitGpioProductSpecific();

   // Detect if Power board is off - and we are on 5V from 485 cable - "In Box Programing"
   if (GpioDataRegs.GPBDAT.bit.GPIO60 == 0)
     u16_Power_Board_Off = 1;
   else
     u16_Power_Board_Off = 0;

   /* start timers(pwm, mts, etc...) */
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Enable TBCLK within the ePWM
   EDIS;
   asm(" nop");                             // 2 SYSCLKOUT cycle delay till the action is valid
   asm(" nop");

   u32_Timer_Debug2 = CpuTimer0Regs.TIM.all;        // capture Timer0 value

   while ( (u8_FPGA_ok == 0) && (u16_FPGA_Load_Retry_Counter < FPGA_LOAD_RETRY_NUM) )
   {
      u8_FPGA_ok = LoadFpga();
      u16_FPGA_Load_Retry_Counter++;
   }

   u32_Timer_Debug3 = CpuTimer0Regs.TIM.all;        // capture Timer0 value
   *((int*)FPGA_PWM_ENABLE_REG_ADD) = 0;   // Write to FPGA to route all "1" to PWM out

   //change functionallity of GpioDataRegs.GPBDAT.bit.GPIO55 from INIT_B to DSP_EQEP3B
   //this is done here after CRC error check using INIT_B was done
   *((int*)FPGA_NO_INIT_B_REG_ADD) = 0x01;

   //change functionallity of GpioDataRegs.GPBDAT.bit.GPIO55 from INIT_B to DSP_EQEP3B
   //this is done here after CRC error check using INIT_B was done
   *((int*)FPGA_NO_INIT_B_REG_ADD) = 0x01;

   // Write the axis number to the FPGA. Relevant for DDHD.
   // Write 1 to axis 0, and 2 to axis 1.
   *((int*)FPGA_FPGA_ID_REG_ADD) = u16_Axis_Num + 1;  // relevant for DDHD

   // Copy AB wire break state (GPIO6) to FPGA (relevant to Tamagawa 8 wires in DDHD HW)
   *((int*)FPGA_DSP_MFB_AX0IFAIL_REG_ADD) = (GpioDataRegs.GPADAT.all >> 6) & 0x0001;

   // Check if retro display installed, on CDHD1 it is always Retro
   if ( IS_RETRO_DISPLAY == 1 || (!IS_CDHD2_DRIVE)) u8_Is_Retro_Display = 1;
      else u8_Is_Retro_Display = 0;

   //Raise Analog Output voltage to 0 after initial -12V
   if ((SHNDR != u16_Product) && (SHNDR_HW != u16_Product))
   {
      BGVAR(s32_Analog_Out_Volt_Cmd) = 0;
      BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Analog_Out_Volt_Cmd);
      WriteAnalogOutputToDAC(drive);
   }
   // If the product is DDHD, and there is an indication that ember was running, stay in a loop
   // to allow embering the other DSP without interfiring the communication.
   // Indicate this on the display.
   // In the future - may get an indication from the other DSP to resume operation.
   if ( (u16_Product == DDHD) && (u32_Stamp_From_Ember == 0xAA55AC13) )
   {
      while (s16_control == 1)     // to defeat compiler remark. Usually it should be "while(1)"
      {
         HWDisplay(UPPER_DASH, 4);
         u32_timer = CpuTimer0Regs.TIM.all;                           // capture Timer0 value
         while ((u32_timer - CpuTimer0Regs.TIM.all) < 150000000)  {}  // 150,000,000 = 0.5 sec

         HWDisplay(DASH_CHAR, 4);
         u32_timer = CpuTimer0Regs.TIM.all;                           // capture Timer0 value
         while ((u32_timer - CpuTimer0Regs.TIM.all) < 150000000)  {}  // 150,000,000 = 0.5 sec

         HWDisplay(LOWER_DASH, 4);
         u32_timer = CpuTimer0Regs.TIM.all;                           // capture Timer0 value
         while ((u32_timer - CpuTimer0Regs.TIM.all) < 150000000)  {}  // 150,000,000 = 0.5 sec
      }
   }


   //in order to be able to read from rotary knob we need first to write to display (serializer)
   //rotary knob is read in function InitSerCommunication2 - Do not move this line of code
   if (u16_Product == SHNDR_HW)
   {
      HWDisplay(DASH_CHAR, 4);
      HWDisplay(DASH_CHAR, 3);
      HWDisplay(DASH_CHAR, 2);
      HWDisplay(DASH_CHAR, 1);
      HWDisplay(DASH_CHAR, 0);
   }
   else
   {
      HWDisplay(ALL, 4);
   }

   *(int*)FPGA_FB_POWER_SUPPLY_REG_ADD |= 0x03; // Power up 5V to 1st Encoder, 8V to Hiperface
   SetFeedback5Volt();                          // Power up the 5V to the encoder
   // Devices, and 5V to 2nd Encoder / differential halls

   EepromStartPower(); // should be called after LoadFpga(), since DDHD uses FPGA RAM to transfer power EEPROM data between DSPs
   EepromWriteTest();  // should be called after LoadFpga(), since FPGA controlles the Write Protect signal
   ExtractEepromContent();

   CurrentLoopInit(0);
   CurrentLoopInit(1);

   InitPieVectTable();         // should be called after ExtractEepromContent() to init u16_Product, and before EnableInterrupts()
   InitEPwmProductSpecific();  // should be called after ExtractEepromContent() to init u16_Product, and before EnableInterrupts()

   if ((u16_Product == DDHD) || (BGVAR(u16_Power_Hw_Features) & 0x0002))
   {
      BGVAR(s64_Faults_Mask_2) |= (LOGIC_AC_LOSS_MASK | BUS_AC_LOSS_MASK);
      drive = 1;
      BGVAR(s64_Faults_Mask_2) |= (LOGIC_AC_LOSS_MASK | BUS_AC_LOSS_MASK);
      drive = 0;
   }

   // Write to FPGA the Control Board HW Revision, calculate from ASCII
   s16_i = ((u16_Control_Board_Rev & 0xFF) - 0x30) * 10 + ((u16_Control_Board_Rev >> 8) - 0x30);
   if (s16_i < 1 || s16_i > 99)
      s16_i = 1; // Substitute Board HW Revision 1 if not a valid value.
   *((int*)FPGA_BOARD_REV_REG_ADD) = s16_i;

   if (IS_EC_DRIVE)
   {
      HandlePrivateLabel();
         }

   LoadMicroBlaze();

   drive = 1;
   BGVAR(s32_Icont) = 0L;
   BGVAR(s32_Derated_Drive_I_Cont) = 0L;
   BGVAR(u32_Stop_Current) = 0L;
   drive = 0;

   BGVAR(s32_Icont) = BGVAR(s32_Drive_I_Cont);
   BGVAR(s32_Derated_Drive_I_Cont) = BGVAR(s32_Drive_I_Cont);
   BGVAR(u32_Stop_Current) = BGVAR(s32_Drive_I_Peak);

   Adc_Shl = 2;
   if (u16_Adc_Resolution == 16) Adc_Shl = 0;
   if (u16_Adc_Resolution == 12) Adc_Shl = 4;

   InitExternalADC(); // should be called after init XINF, after LoadFpga(), and before interrupts enabled

   InitFlash(); // init flash handling variables

   Security();

   // reset done bit
   // AX0_s16_Sine_Enc_Bits &= ~SINE_ZERO_DONE_MASK;
   // AX1_s16_Sine_Enc_Bits &= ~SINE_ZERO_DONE_MASK;

   // enable sine zero process
   //AX0_s16_Sine_Enc_Bits |= RESET_SINE_ZERO_MASK;
   //AX1_s16_Sine_Enc_Bits |= RESET_SINE_ZERO_MASK;


   AX0_u32_Pos_Fdbk_Lo = 0L;
   AX0_s32_Pos_Fdbk_Hi = 0L;

   AX0_u32_VBL_Thresh = 0x0300000; // This value is multiplied by 2^8 in the RT
   AX0_u16_TQF_Weight_Shl = 3;

   SetPwmFrq(0);

   InitDigIOs(0);   // Init digital IOs states
   UpdateDigitalOutputOvercurrentMask(drive);

   /*Encoder Follower vars init*/
   AX0_u16_Enc_Follower_Mode = 1;
   AX0_s16_Delta_Qeps_At_Start = 0;

/* for (s16_temp_index=1; s16_temp_index>=0; s16_temp_index--)
   { */
      //  Initialize Fault log
      InitFaultLog(s16_temp_index);

      u16_Reset_All_Param_Executed = 0;

// uncomment only when using the debug log feature      InitDebugLog(); // should run before the first place data is logged

      // Reset all param if requested last time, OR if read from flash fails.
      if(LoadFromFlash(s16_temp_index) != SAL_SUCCESS)
           ResetAllParam(s16_temp_index);

      // If version stored does not match current version clear the NV - to avoid bad parameters
      if ( (s16_Factory_Restore_On_Power_Up == 1)               ||
           (strcmp(BGVAR(u8_SwVersion),p_s8_CDHD_Drive_Version) != 0)  )
      {
         s16_i = 0;
         while ((s16_i < 20) && (p_s8_CDHD_Drive_Version[s16_i] != 0))
         {
            BGVAR(u8_SwVersion)[s16_i] = p_s8_CDHD_Drive_Version[s16_i];
            s16_i++;
         }
         if (s16_i >= 20) s16_i = 19;
         BGVAR(u8_SwVersion)[s16_i] = 0;

         while (SalRestoreFactorySettingsCommand(s16_temp_index) == SAL_NOT_FINISHED);   // This function calls ResetAllParam()

         if(s16_Factory_Restore_On_Power_Up == 1)
         {
            s16_Factory_Restore_On_Power_Up = 0;
            BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_NV_SAVE;
         }
         else
         {
            if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
            {// for non LX28/26 set alarm
               BGVAR(s16_Flash_Checksum_Fault_Flag) = 1;
            }
            else
            {// for LX28/26 set Wn737 (drive is using default configuration)
               BGVAR(s16_Drive_Default_Config_Wrn_Flag) = 1;
            }
         }
      }
      else InitAll(s16_temp_index);
/* } */

   //Load serial number
   //SalLoadSerialNumber();

   // initialize power brake
    if (u16_Power_Brake_Exist)
      InitPWRBrakeOutput(s16_temp_index);

   for (drive = 0; drive < 2; ++drive)
   {
      BGVAR(s16_DisableInputs) |= SW_EN_MASK;
      if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
         BGVAR(s16_DisableInputs) |= FB_EN_MASK;
      if (BGVAR(u8_Auto_Home_Mode) != 1) // Prevent Homing on next Enable by marking Homing-
         BGVAR(u16_Homing_Attempted_Once) = 1;// -Attempted as if Homing already attempted.
   }
   drive = 0;

   InitRuntimeCounter(); // run time counter initialization

   /* start timers(pwm, mts, etc...) */
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Enable TBCLK within the ePWM
   EDIS;
   asm(" nop");                             // 2 SYSCLKOUT cycle delay till the action is valid
   asm(" nop");

//   AX0_s16_Power_Up_Timer = 0x7FFF; // Moved Halls Detection State-Machine Time Initialization to
//   AX1_s16_Power_Up_Timer = 0x7FFF; // later stage in Init to avoid Halls-Fault on Time-Out.

   SET_DP_TO_AXIS_0();

   if ((u16_Product == SHNDR_HW) || (u16_Product == SHNDR))
   {
      // if powered from RJ45 (in box programming), then do not activate CAN (force COMMODE=0) because it disturbes the serial communication.
      if (GpioDataRegs.GPBDAT.bit.GPIO60 == 0)
      {
         BGVAR(u8_Comm_Mode) = 0;
         drive = 1;
         BGVAR(u8_Comm_Mode) = 0;
         drive = 0;
      }
   }

   InitFieldBusVars();              // Call before interrupts enabled

   // On SE - Init SNDR conversions if TSP = 125 usec (default init is 250usec)
   /*if ( (u16_Product == SHNDR) || ((u16_Product == SHNDR_HW) && (AX0_u16_Pos_Loop_Tsp == TSP_125_USEC)) )
   {
      InitSndrConversions125us();
   }*/

// Recording on power up, make sure you use upper case in below strings
/*{
   s16_Number_Of_Parameters = 5;
   s64_Execution_Parameter[0] = 16LL;
   s64_Execution_Parameter[1] = 2000LL;
   strcpy(u8_Execution_String[0], "H03000C3");  // s16_DisableInputs
   strcpy(u8_Execution_String[1], "H0E480.S32");// AX0_u32_Pos_Fdbk_Lo
   strcpy(u8_Execution_String[2], "H0E4AB");    // AX0_u16_Encsim_Freq_Flags
   strcpy(u8_Execution_String[3], "H0E4BC");    // AX0_s16_Debug3
   RecordCommand();

   s16_Number_Of_Parameters = 1;
   strcpy(u8_Execution_String[0], "IMM"); // Mode
   //s64_Execution_Parameter[1] = 0LL;         // Level
   //s64_Execution_Parameter[2] = 1000LL;      // Location
   //s64_Execution_Parameter[3] = 1LL;         // Direction
   RecTrigCommand();
}*/

   // This is to avoid outputing encoder simulation till the valid feedback is read (especialy needed to feedback which required init process)
   AX0_u16_Encsim_Freq_Flags |= INHIBIT_ENC_SIM_PFB_NOT_READY_MASK;

   //initialize brake mode status
   AX0_u16_BrakeOn = 1;

   EnableInterrupts();

   FanHalfSpeed();

   *(int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) = 0xFFFF;   // Clear FPGA power up Faults

   //the below 2 function must be called after FPGA is loaded since they demand rotary knob data (taken from FPGA)
   InitSerCommunication2();

   if (u8_FPGA_ok)
   {
      u32_Fpga_Version = ((unsigned long)(*(unsigned int *)FPGA_FPGA_VERSION_REG_ADD) << 16L) | ((unsigned long)(*(unsigned int *)FPGA_FPGA_REVISION_REG_ADD) & 0xFFFF);
      // Translate BCD code to integer
      s16_i = *(int *)FPGA_MON_DAY_REG_ADD;
      u16_Fpga_Day = ((s16_i & 0x00F0) >> 4) * 10;
      u16_Fpga_Day += (s16_i & 0x000F);

      u16_Fpga_Month = ((s16_i & 0xF000) >> 12) * 10;
      u16_Fpga_Month += ((s16_i & 0x0F00) >> 8);

      s16_i = *(int *)FPGA_YEAR_REG_ADD;
      u16_Fpga_Year = ((s16_i & 0xF000) >> 12) * 1000;
      u16_Fpga_Year += ((s16_i & 0x0F00) >> 8) * 100;
      u16_Fpga_Year += ((s16_i & 0x00F0) >> 4) * 10;
      u16_Fpga_Year += (s16_i & 0x000F);

      // Test for FPGA version mismatch (check CAN,EC and Debug FPGA versions)
      u16_fpga_current_ver_hi = ((u32_Fpga_Version & 0xF0000000) >> 28L) * 1000 + ((u32_Fpga_Version & 0x0F000000) >> 24L) * 100 + ((u32_Fpga_Version & 0x00F00000) >> 20L) * 10 + ((u32_Fpga_Version & 0x000F0000) >> 16L);
      u16_fpga_current_ver_lo = ((u32_Fpga_Version & 0x0000F000) >> 12L) * 1000 + ((u32_Fpga_Version & 0x00000F00) >> 8L) * 100 + ((u32_Fpga_Version & 0x000000F0) >> 4L) * 10 + (u32_Fpga_Version & 0x0000000F);

      if (u16_Product == SHNDR_HW)
      {
         if (u16_Flash_Type == 1)   // 2 * 64Mb flash
         {
            switch (u16_Fieldbus_Type)
            {
               case FB_SERCOS:
                  u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_LXM28E_SERCOS * 100.0 + 0.5)) / 100);
                  u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_LXM28E_SERCOS * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
               break;

               case FB_ETHERNET_IP:
                  u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_LXM28E_ETHERNETIP * 100.0 + 0.5)) / 100);
                  u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_LXM28E_ETHERNETIP * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
               break;

               case FB_ETHERCAT:
               default:
                  u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_LXM28E_ETHERCAT * 100.0 + 0.5)) / 100);
                  u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_LXM28E_ETHERCAT * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
               break;
            }
         }
         else
         {
            u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_SE * 100.0 + 0.5)) / 100);
            u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_SE * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
         }
      }
      else if ( (IS_DDHD2_DRIVE) && !IS_DDHD_EC2_DRIVE )
      {
         u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_DDHD2 * 100.0 + 0.5)) / 100);
         u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_DDHD2 * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
      }
      else if (IS_DDHD_EC2_DRIVE)
      {
         u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_DDHD_EC2 * 100.0 + 0.5)) / 100);
         u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_DDHD_EC2 * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
      }
      else if ((u16_Product == DDHD) && (!IS_EC_DRIVE))
      {// for not ECAT DDHD
         u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_DDHD * 100.0 + 0.5)) / 100);
         u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_DDHD * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
      }
      else if (IS_EC_LITE_DRIVE)
      {
         u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_EC_LITE * 100.0 + 0.5)) / 100);
         u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_EC_LITE * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
      }
      else if (IS_EC2_DRIVE)
      {
         u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_EC2 * 100.0 + 0.5)) / 100);
         u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_EC2 * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
      }
      else if (IS_EC_DRIVE)
      {
         u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_EC * 100.0 + 0.5)) / 100);
         u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_EC * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
      }
      else if (IS_PN_DRIVE)
      {
         u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_PN * 100.0 + 0.5)) / 100);
         u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_PN * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
      }
      else if (IS_CAN2_DRIVE) //CAN2
      {
         u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_CAN2 * 100.0 + 0.5)) / 100);
         u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_CAN2 * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
      }
      else                           // CAN / analog
      {
         u16_FPGA_Supported_Ver_Hi = (int)(((long)(FPGA_VERSION_SUPPORTED_CAN * 100.0 + 0.5)) / 100);
         u16_FPGA_Supported_Ver_Lo = (int)((long)(FPGA_VERSION_SUPPORTED_CAN * 100.0 + 0.5) - ((long)100 * u16_FPGA_Supported_Ver_Hi));
      }
      if ( ( (u16_fpga_current_ver_hi != u16_FPGA_Supported_Ver_Hi) ||
             (u16_fpga_current_ver_lo != u16_FPGA_Supported_Ver_Lo)   ) &&
           ((u32_Fpga_Version & 0x00FF00FF) != 0x00FF00FF)                )
         u16_FPGA_ver_mismatch = 1;
   }

   //micro blaze version validation test
   if ((u16_Product == SHNDR_HW) && (u16_Flash_Type == 1))   // 2 * 64Mb flash
   {
      switch (u16_Fieldbus_Type)
      {
         case FB_SERCOS:
            if (strcmp(s8_FieldBus_Version, s8_Micro_Blaze_Supported_Lxm28e_Sercos_Ver) != 0)
               u16_UBLAZE_ver_mismatch = 1;
         break;

         case FB_ETHERNET_IP:
            if (strcmp(s8_FieldBus_Version, s8_Micro_Blaze_Supported_Lxm28e_EthernetIP_Ver) != 0)
               u16_UBLAZE_ver_mismatch = 1;
         break;

         case FB_ETHERCAT:
         default:
            if (strcmp(s8_FieldBus_Version, s8_Micro_Blaze_Supported_Lxm28e_EtherCAT_Ver) != 0)
               u16_UBLAZE_ver_mismatch = 1;
         break;
      }
   }
   else if (IS_EC_DRIVE_AND_COMMODE_1)
   {
      if (strcmp(s8_FieldBus_Version, s8_Micro_Blaze_Supported_Ver) != 0)
         u16_UBLAZE_ver_mismatch = 1;
   }

   for (drive = 0; drive < 2; ++drive)
   {
      // Operate SININIT on Power-Up according to SININITMODE.
      if (BGVAR(u8_SinInitMode)) SininitCommand(drive);
      // Set Out-of-Range to saved value
   }
   drive = 0;

   EQep1Regs.QCLR.bit.PHE = 1;  // Clear the encoder phase error bit on DSP peripheral

   // init parameters that are changed only after cycle power and update PUU units afterwards.
   InitCyclePowerPparams(0);

   // At this place we know how the P1-01 parameter "BGVAR(u16_P1_01_CTL_Current)" is configured.
   // Therefore this code must reside after the "InitCyclePowerPparams" function call.
   if (u16_Product == SHNDR_HW)
   {
      for (drive = 0; drive < 2; drive++)
      {
         ShowSchneiderBootUpMsg(drive);
      }
      drive = 0;
   }

   InitFilteredSignals(0);

// There are boards that it takes long time (more than 4 seconds) to their DC/DC to wake up when the drive is warm.
// Implement here a wait (with timeout) for the DC/DC to wake up.
// When the DC/DC does not work, the OC, IMP OT, and Vbus HW fault exist.
// These fault exist also when the STO is not connected, so check that the STO is connected.
// So, if OC or IPM OT or Vbus HW fault exist and STO exist, wait with 5 seconds timeout.

   s32_Debug_Timer_Start = Cntr_1mS;
   u16_Debug_Counter = 0;
   s32_timer = Cntr_1mS;

   u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
   u16_FPGA_flt_indication &= (PWR_AX0IPM_OTN | PWR_AX0OCn | PWR_STO_STAT);
   *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_flt_indication;   // Clear the fault on FPGA

   if ((BGVAR(u16_Power_Hw_Features) & 0x0040) != 0)
   {
      // polarity of power module OT indication is inverted
      u16_FPGA_flt_indication ^= PWR_AX0IPM_OTN;
   }

   while ( ( (u16_Vbus_HW_Measure_Flt == 1) || ( (u16_FPGA_flt_indication & (PWR_AX0IPM_OTN | PWR_AX0OCn ) ) != 0) )  &&
           ( ((u16_Product != DDHD) && ((u16_FPGA_flt_indication & PWR_STO_STAT) == 0)) ||(((u16_Product == DDHD)||(u16_Product == SHNDR_HW) )&& (GpioDataRegs.GPADAT.bit.GPIO27 == 0)) )  &&
           (PassedTimeMS(5000L, s32_timer) == 0)                                                                        )
   {
      s32_timer_2 = Cntr_1mS;
      while (PassedTimeMS(100L, s32_timer_2) == 0){};

      u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
      u16_FPGA_flt_indication &= (PWR_AX0IPM_OTN | PWR_AX0OCn | PWR_STO_STAT);
      *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_flt_indication;   // Clear the fault on FPGA
      if ((BGVAR(u16_Power_Hw_Features) & 0x0040) != 0)
      {
         // polarity of power module OT indication is inverted
         u16_FPGA_flt_indication ^= PWR_AX0IPM_OTN;
      }
      u16_Debug_Counter++;
   }
   u16_Debug_FPGA = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
   u16_Debug_Vbus_HW = u16_Vbus_HW_Measure_Flt;
   s32_Debug_Timer_End = Cntr_1mS;

   // On SE, calculate the size of each P_Params group
   if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
   {
      u16_Max_Param_Id_Code[0] = MAX_P_INDEX_0;
      u16_Max_Param_Id_Code[1] = MAX_P_INDEX_1;
      u16_Max_Param_Id_Code[2] = MAX_P_INDEX_2;
      u16_Max_Param_Id_Code[3] = MAX_P_INDEX_3;
      u16_Max_Param_Id_Code[4] = MAX_P_INDEX_4;
      u16_Max_Param_Id_Code[5] = MAX_P_INDEX_5;
      u16_Max_Param_Id_Code[6] = MAX_P_INDEX_6;
      u16_Max_Param_Id_Code[7] = MAX_P_INDEX_7;
      u16_Max_Param_Id_Code[8] = MAX_P_INDEX_8;
      u16_Max_Param_Id_Code[9] = MAX_P_INDEX_9;
   }

   // in this function the CANopen bootup message is sent, this means that the drive is ready,
   // so peform this function as close as possible to the end of init
   InitFieldBus();              // Call after interrupts enabled


   InitWatchDog();    // init & enable WD
   InitHmiMenuIndexArrays();
   // If the Drive is a Schneider Drive
   if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
   {
      if ( ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)        ||
          ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)   ||
          ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)         )
      {
      }
      // If the Lexium is a CAN product, and P1-01 is set to canopen mode
      else if (IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK) && ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_CANOPEN))
      {
         // State that it is a fieldbus Drive
         BGVAR(u8_Lexium_Drive_Type) = LEX_TYPE_FIELDBUS_DRIVE;
         drive = 1;
         BGVAR(u8_Lexium_Drive_Type) = LEX_TYPE_FIELDBUS_DRIVE;
         // Set the access-rights per default to fieldbus during booting
         drive = 0;
         BGVAR(u16_Lexium_Acces_Rights_State) = (LEX_CH_FIELDBUS<<8) & 0xFF00;
         drive = 1;
         BGVAR(u16_Lexium_Acces_Rights_State) = (LEX_CH_FIELDBUS<<8) & 0xFF00;
      }
      else // The Lexium is a IO product
      {
         // State that it is a IO Drive
         BGVAR(u8_Lexium_Drive_Type) = LEX_TYPE_IO_DRIVE;
         drive = 1;
         BGVAR(u8_Lexium_Drive_Type) = LEX_TYPE_IO_DRIVE;
         // Set the access-rights per default to IO during booting
         drive = 0;
         BGVAR(u16_Lexium_Acces_Rights_State) = (LEX_CH_IO_CONTROL<<8) & 0xFF00;
         drive = 1;
         BGVAR(u16_Lexium_Acces_Rights_State) = (LEX_CH_IO_CONTROL<<8) & 0xFF00;
      }
      drive = 0;
   }

   InitModbusSerCommunication(); // Udi 29 Aug 2013

   // Here determine if the baud-rate setting is different from its default and
   // if therefore something has to be displayed on the 7-segemnts.
   switch(BGVAR(u32_Serial_Baud_Rate))
   {
      case (9600):
         s8_Baud_Rate_Setting[5]  = NINE;
         s8_Baud_Rate_Setting[7]  = SIX;
         s8_Baud_Rate_Setting[9]  = ZERO;
         s8_Baud_Rate_Setting[11] = ZERO;
         s8_Baud_Rate_Setting[13] = NONE;
         u16_Display_Baud_Rate_Index = 0; // Display the baud-rate on the 7-segments
      break;
      case (19200):
         s8_Baud_Rate_Setting[5]  = ONE;
         s8_Baud_Rate_Setting[7]  = NINE;
         s8_Baud_Rate_Setting[9]  = TWO;
         s8_Baud_Rate_Setting[11] = ZERO;
         s8_Baud_Rate_Setting[13] = ZERO;
         u16_Display_Baud_Rate_Index = 0; // Display the baud-rate on the 7-segments
      break;
      case (38400):
         s8_Baud_Rate_Setting[5]  = THREE;
         s8_Baud_Rate_Setting[7]  = EIGHT;
         s8_Baud_Rate_Setting[9]  = FOUR;
         s8_Baud_Rate_Setting[11] = ZERO;
         s8_Baud_Rate_Setting[13] = ZERO;
         u16_Display_Baud_Rate_Index = 0; // Display the baud-rate on the 7-segments
      break;
      case (57600):
         s8_Baud_Rate_Setting[5]  = FIVE;
         s8_Baud_Rate_Setting[7]  = SEVEN;
         s8_Baud_Rate_Setting[9]  = SIX;
         s8_Baud_Rate_Setting[11] = ZERO;
         s8_Baud_Rate_Setting[13] = ZERO;
         u16_Display_Baud_Rate_Index = 0; // Display the baud-rate on the 7-segments
      break;
      case(115200):// Show nothing on the 7-segment in case of the default setting
      break;
      default: // This condition is never suppose to happen and just indicates,
               // that a developer forgot to add a 7-segment message for a
               // specific RS232 baud-rate setting in this switch-case instruction.
         s8_Baud_Rate_Setting[5]  = E_LET;
         s8_Baud_Rate_Setting[7]  = r_LET;
         s8_Baud_Rate_Setting[9]  = r_LET;
         s8_Baud_Rate_Setting[11] = o_LET;
         s8_Baud_Rate_Setting[13] = r_LET;
         u16_Display_Baud_Rate_Index = 0; // Display a not decoded baud rate
      break;
   }

   // set state for sending bootup message only and update port lib.
   BGVAR(s16_CAN_BootUp_State) = CAN_BOOTUP_INIT;
   BGVAR(u16_CAN_BootUp_Requests_Counter)++;

   do
   {
   CAN_SendBootUpMsg(BACKGROUND_CONTEXT, drive, 1);
   } while (BGVAR(s16_CAN_BootUp_State) != CAN_BOOTUP_IDLE);

   AX0_s16_Power_Up_Timer = 0x7FFF; // Moved Halls Detection State-Machine Time Initialization to
   BGVAR(u16_Halls_Invalid_Value) = 0; // Reset Halls Validity Check Counter
   // to avoid Halls-Invalid indication before Halls are initialized

   u16_RT_OL_Status_Bits = 0; // Reset RT Overload bits
   u16_RT_OL_Counter = 0;

   AX0_u16_CCW_LS = 0; // Reset Phantom Bits that may have been set by Transients on Limit Inputs
   AX0_u16_CW_LS = 0;

   EQep1Regs.QCLR.bit.PHE = 1;  // Clear the encoder phase error bit on DSP peripheral

   // ****************************************************************************
   // 2nd part of Bugzilla 5404 feature added.
   // Generate aa fault if firmware version is above 1.40.x in case of a
   // Googol Technology private label.
   // The 1st part of the Bugzilla item is done at inside  the function
   // "LoadFromFlashStateMachineInternal" where the BGVAR(u8_SwVersion_Binary)
   // array is being created.
   // ****************************************************************************
   // if GTHD or GTDD Drive
   if ((BGVAR(u16_Hw_Features_Private_Label) == 4) || (BGVAR(u16_Hw_Features_Private_Label) == 5))
   {
      // if firmware version >= 2.X.X or
      // if firmware version >= 1.40.X
      if ( (BGVAR(u8_SwVersion_Binary)[0] >= 2)                                                ||
          ((BGVAR(u8_SwVersion_Binary)[0] == 1) && (BGVAR(u8_SwVersion_Binary)[1] >= 40))
        )
      {
         BGVAR(s64_SysNotOk_2) |= FIRMWARE_VERSION_NOT_SUPPORTED;
      }
   }

   u16_Init_State = 0;   // indicated that init is done
   u32_Timer_Debug4 = CpuTimer0Regs.TIM.all;        // capture Timer0 value
}


void InitFieldBus(void)
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // if board is CAN and CAN HW exists on board
   if ( (IS_CAN_DRIVE) && (IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK)) )
   {
      //Init_CAN_Interrupts();
      InitECan(); // CAN init

      // restore current opmode
      p402_modes_of_operation_display = BGVAR(s16_CAN_Opmode);
      p402_modes_of_operation = BGVAR(s16_CAN_Opmode);

      //Enable_CAN_Interrupts();
      Start_CAN();
   }

   //private label handling for SDOs 0x1008 (Manufacturer device name) & 0x1018 sub 1 (Identity object vendor name)
   if ((s8_Control_Drive_Model_Number_1[0] == 'C') && (s8_Control_Drive_Model_Number_1[1] == 'D'))  //STX - CDHD
   {
      p301_identity.vendorId = 0x000002E1;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "CDHD drive");
     u16_Hw_Features_Private_Label = 1; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'D') && (s8_Control_Drive_Model_Number_1[1] == 'D'))  //STX - DDHD
   {
      p301_identity.vendorId = 0x000002E1;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "DDHD drive");
     u16_Hw_Features_Private_Label = 2; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'G') && (s8_Control_Drive_Model_Number_1[1] == 'B'))  //STX - GBOT
   {
      p301_identity.vendorId = 0x000002E1;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "GBOT drive");
     u16_Hw_Features_Private_Label = 3; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'G') && (s8_Control_Drive_Model_Number_1[2] == 'H'))  //STX - GTHD
   {
      p301_identity.vendorId = 0x000002E1;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "GTHD drive");
     u16_Hw_Features_Private_Label = 4; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'G') && (s8_Control_Drive_Model_Number_1[2] == 'D'))  //STX - GTDD
   {
      p301_identity.vendorId = 0x000002E1;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "GTDD drive");
     u16_Hw_Features_Private_Label = 5; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'A') && (s8_Control_Drive_Model_Number_1[1] == 'S')) //AKRIBIS
   {
      p301_identity.vendorId = 0x000002E1; // TBD
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "ASD Drive");
     u16_Hw_Features_Private_Label = 6; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'F') && (s8_Control_Drive_Model_Number_1[1] == 'P')) //MPC
   {
      p301_identity.vendorId = 0x00000678;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "FPRO Drive");
     u16_Hw_Features_Private_Label = 7; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'M') && (s8_Control_Drive_Model_Number_1[1] == 'T'))  //PBA
   {
      p301_identity.vendorId = 0x000002BA;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "MaxTune drive");
     u16_Hw_Features_Private_Label = 8; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'J') && (s8_Control_Drive_Model_Number_1[1] == 'S'))  //TECO
   {
      p301_identity.vendorId = 0x000002E1;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "JSDZ drive");
     u16_Hw_Features_Private_Label = 9; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'S') && (s8_Control_Drive_Model_Number_1[1] == 'D'))  //HIGEN
   {
      p301_identity.vendorId = 0x00000625;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "SDA Ethercat Drive");
     u16_Hw_Features_Private_Label = 10; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'R') && (s8_Control_Drive_Model_Number_1[1] == 'T'))  //RTDH
   {
      p301_identity.vendorId = 0x000002E1;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "RTDA Drive");
     u16_Hw_Features_Private_Label = 11; //ASCII COMMAND HWFEATURES
   }
   else if ((s8_Control_Drive_Model_Number_1[0] == 'E') && (s8_Control_Drive_Model_Number_1[1] == 'D'))  //EDA
   {
      p301_identity.vendorId = 0x000002E1;
      UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "EDA Ethercat Drive");
     u16_Hw_Features_Private_Label = 12; //ASCII COMMAND HWFEATURES
   }

   // Write vendor data in EEPROM (ESI)
//   if (IS_EC_DRIVE_AND_COMMODE_1)
//   {
//     if ()
//p301_identity.vendorId
//   }
}


//**********************************************************
// Function Name: Set_0_1_RpmPtpConversions
// Description: Recalculate the units conversion for 0.1rpm for ptp motion
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void Set_0_1_RpmPtpConversions(int drive, unsigned int u16_tsp)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // first set units conversion to default (250uSec) and if position loop is 125uSec update the shifter.
   BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_PTP_CONVERSION]).u16_unit_conversion_to_user_shr = 32;
   BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_PTP_CONVERSION]).u16_unit_conversion_to_internal_shr = 20;

   if(u16_tsp == TSP_125_USEC)
   {
      BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_PTP_CONVERSION]).u16_unit_conversion_to_user_shr--;
      BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_PTP_CONVERSION]).u16_unit_conversion_to_internal_shr++;
   }
}


void InitFieldBusVars(void)
{
   int drive = 0;
   // AXIS_OFF;
   int i,j = 0;
   float f_tmp = 0.0;

   // IPR 1602: Axis makes strange (scratchy) noicess in CSP mode
   // nitsan: On CANopen, read rx data only on slot 7 and write tx data only on slot 6
   //         On ECAT, read rx data on slot 3 and 7 and write tx data only on slot 2 and 6, to support 125 uSec cycle time
   // Therefore, manipulate _AX0_u16_Interpolation_Slot_Counter_Mask to mask the slot number to allow executing
   // the _ExecuteFieldbusRtRxObject and _ExecuteFieldbusRtTxObject in the desired slots. 

   // init interpolation RT slots according to fieldbus type
   if (IS_CAN_DRIVE)
   {
      VAR(AX0_u16_Interpolation_Slot_Counter_Mask) = 7;
      VAR(AX0_u16_Interpolation_RX_Slot_Mask) = 7;        
      VAR(AX0_u16_Interpolation_TX_Slot_Mask) = 6;
   }
   else  // for ECAT, sercos and non-canopen fieldbus
   {
      VAR(AX0_u16_Interpolation_Slot_Counter_Mask) = 3;
      VAR(AX0_u16_Interpolation_RX_Slot_Mask) = 3;
      VAR(AX0_u16_Interpolation_TX_Slot_Mask) = 2;
   }
   
   BGVAR(u16_Can_Error_State) = CANFLAG_ACTIVE;   // set error state to no-error

   u16_Last_ErrCode = 0x00FF;   // should be set to undefined code.

   // nitsan: init can acc and dec after eeprom is loaded, to take the acc/dec values
   for (drive = 0; drive < 2; drive++)
   {
      BGVAR(u64_CAN_Acc) = BGVAR(u64_AccRate);
      BGVAR(u64_CAN_Dec) = BGVAR(u64_DecRate);
      BGVAR(u64_CAN_Dec_Stop) = BGVAR(u64_DecStop);
//       u64_CAN_Max_Acc[i] = MAX_ACC_DEC;  // avoid limitation on acc (max acc definded in canopen DS402)
//       u64_CAN_Max_Dec[i] = MAX_ACC_DEC;  // avoid limitation on dec and decstop (max dec definded in canopen DS402)
//     Initializing CAN Max. Acc/Dec at Declaration, to avoid problems with u64_DecStop.

      BGVAR(u32_Fb_Motor_Rated_Current) = (unsigned long)BGVAR(s32_Motor_I_Cont);    // canopen micont and serial micont are both in mA
      f_tmp = (float)((float)BGVAR(u32_Fb_Motor_Rated_Current) * ((float)BGVAR(u32_Motor_Kt)/1000.0));
      p402_motor_rated_torque = (unsigned int)f_tmp;
   }
   drive = 0;

   // to setup units of current for canopen
   ConvertFbCanOpenCurrent(drive);
   ConvertFbCanOpenTorque(drive);

   // If the command COMMODE has been set to 1
   if (BGVAR(u8_Comm_Mode) == 1)
   {
      s16_Fieldbus_Flags |= 1; // Indicate this setting in the corresponding RT variable

     //for later RT decision regarding MOVESMOOTH
     VAR(AX0_u16_Com_Mode1) = 1;
   }

   if (IS_CAN_DRIVE)
   {
      s16_Fieldbus_Flags |= 0x0002;
      p_bg_rx_ctrl_buffer = bg_rx_ctrl_buffer;
      p_bg_tx_ctrl_buffer = bg_tx_ctrl_buffer;
      p_bg_rx_data_buffer = bg_rx_data_buffer;
      p_bg_tx_data_buffer = bg_tx_data_buffer;

      p_u16_bg_rx_ctrl_in_index  = &u16_bg_rx_ctrl_in_index;
      p_u16_bg_rx_ctrl_out_index = &u16_bg_rx_ctrl_out_index;
      p_u16_bg_tx_ctrl_in_index  = &u16_bg_tx_ctrl_in_index;
      p_u16_bg_tx_ctrl_out_index = &u16_bg_tx_ctrl_out_index;
      p_u16_bg_rx_data_in_index  = &u16_bg_rx_data_in_index;
      p_u16_bg_rx_data_out_index = &u16_bg_rx_data_out_index;
      p_u16_bg_tx_data_in_index  = &u16_bg_tx_data_in_index;
      p_u16_bg_tx_data_out_index = &u16_bg_tx_data_out_index;

      CAN_Msg_BG_buffer_wr_index = 0;
      CAN_Msg_BG_buffer_rd_index = 0;

      for(i = 0; i < 128; i++)
      {
         CAN_Msg_BG_buffer[i].cobId   = 0;
         CAN_Msg_BG_buffer[i].cobType = CO_COB_DISABLED;
         CAN_Msg_BG_buffer[i].length  = 0;
      }
   }
   else if (IS_EC_DRIVE || IS_PN_DRIVE)
   {
      SET_DPRAM_FPGA_MUX_REGISTER(0);      // init DPRAM fifo pointers for RX

      p_u16_tx_nmt_state = (unsigned int*)DPRAM_TX_NMT_STATE_ADDR;
      p_u16_tx_station_address = (unsigned int*)DPRAM_TX_STATION_ADDRESS_ADDR;
     p_u16_tx_foe_active = (unsigned int*)DPRAM_TX_FOE_ACTIVE_ADDR;


      for(i = 0; i < RX_DPRAM_PTRS_NUM; i++)
         *RxDpramPtrs_T[i] = 0 ;

      // init DPRAM fifo pointers for TX
      SET_DPRAM_FPGA_MUX_REGISTER(2);
      for(i = 0; i < TX_DPRAM_PTRS_NUM; i++)
         *TxDpramPtrs_T[i] = 0 ;

      s16_Fieldbus_Flags |= 0x0004;
      p_bg_rx_ctrl_buffer = (int*)DPRAM_RX_LP_CTRL_FIFO_ADDR;
      p_bg_tx_ctrl_buffer = (int*)DPRAM_TX_LP_CTRL_FIFO_ADDR;
      p_bg_rx_data_buffer = (int*)DPRAM_RX_LP_DATA_FIFO_ADDR;
      p_bg_tx_data_buffer = (int*)DPRAM_TX_LP_DATA_FIFO_ADDR;

      p_u16_bg_rx_ctrl_in_index  = (unsigned int*)DPRAM_RX_LP_HEAD_CTRL_IDX_ADDR;//Changed to avoid Remark
      p_u16_bg_rx_ctrl_out_index = (unsigned int*)DPRAM_RX_LP_TAIL_CTRL_IDX_ADDR;//Changed to avoid Remark
      p_u16_bg_rx_data_in_index  = (unsigned int*)DPRAM_RX_LP_HEAD_DATA_IDX_ADDR;//Changed to avoid Remark
      p_u16_bg_rx_data_out_index = (unsigned int*)DPRAM_RX_LP_TAIL_DATA_IDX_ADDR;//Changed to avoid Remark

      p_u16_bg_tx_ctrl_in_index  = (unsigned int*)DPRAM_TX_LP_HEAD_CTRL_IDX_ADDR;//Changed to avoid Remark
      p_u16_bg_tx_ctrl_out_index = (unsigned int*)DPRAM_TX_LP_TAIL_CTRL_IDX_ADDR;//Changed to avoid Remark
      p_u16_bg_tx_data_in_index  = (unsigned int*)DPRAM_TX_LP_HEAD_DATA_IDX_ADDR;//Changed to avoid Remark
      p_u16_bg_tx_data_out_index = (unsigned int*)DPRAM_TX_LP_TAIL_DATA_IDX_ADDR;//Changed to avoid Remark
      p_u16_bg_tx_emcy_register_index = (unsigned int*)DPRAM_RX_EMCY_REGISTER_ADDR;//Changed to avoid Remark

      p_EC_rt_rx_pdo_buffer = (int*)DPRAM_RX_HP_PDO_FIFO_ADDR;
      p_EC_rt_tx_pdo_buffer = (int*)DPRAM_TX_HP_PDO_FIFO_ADDR;

      for(i = 0; i < RPDO_MAX_NUM; i++)
      {
         p_rpdo_entries[i].u16_inhibit_time = 0;
         p_rpdo_entries[i].u16_trans_type = 0;
         p_rpdo_entries[i].u16_res1 = 0;
         p_rpdo_entries[i].u16_res2 = 0;
         p_rpdo_entries[i].u16_res3 = 0;
         p_rpdo_entries[i].u16_res4 = 0;
         p_rpdo_entries[i].u16_res5 = 0;
         p_rpdo_entries[i].u16_res6 = 0;
         p_rpdo_entries[i].u16_res7 = 0;
         p_rpdo_entries[i].u16_res8 = 0;
         p_rpdo_entries[i].u16_res9 = 0;
         p_rpdo_entries[i].u16_res10 = 0;
         p_rpdo_entries[i].u16_num_of_objects = 0;

         for(j = 0; j < RPDO_MAX_NUM_MAPPED_OBJECTS; j++)
         {
            p_rpdo_entries[i].map_array[j].u32_addr = 0;
            p_rpdo_entries[i].map_array[j].u16_size = 0;
            p_rpdo_entries[i].map_array[j].u16_id   = 0;
         }
      }

      for(i = 0; i < TPDO_MAX_NUM; i++)
      {
         p_tpdo_entries[i].u16_inhibit_time = 0;
         p_tpdo_entries[i].u16_trans_type = 0;
         p_tpdo_entries[i].u16_res1 = 0;
         p_tpdo_entries[i].u16_res2 = 0;
         p_tpdo_entries[i].u16_res3 = 0;
         p_tpdo_entries[i].u16_res4 = 0;
         p_tpdo_entries[i].u16_res5 = 0;
         p_tpdo_entries[i].u16_res6 = 0;
         p_tpdo_entries[i].u16_res7 = 0;
         p_tpdo_entries[i].u16_res8 = 0;
         p_tpdo_entries[i].u16_res9 = 0;
         p_tpdo_entries[i].u16_res10 = 0;
         p_tpdo_entries[i].u16_num_of_objects = 0;

         for(j = 0; j < TPDO_MAX_NUM_MAPPED_OBJECTS; j++)
         {
            p_tpdo_entries[i].map_array[j].u32_addr = 0;
            p_tpdo_entries[i].map_array[j].u16_size = 0;
            p_tpdo_entries[i].map_array[j].u16_id   = 0;
         }
      }
   }

   if (IS_CAN_DRIVE)
   {
      if ((s8_Control_Drive_Model_Number_1[0] == 'C') && (s8_Control_Drive_Model_Number_1[1] == 'D'))  //STX
      {
         p301_identity.vendorId = 0x000002E1;
         UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "CDHD drive");
      }
      else if ((s8_Control_Drive_Model_Number_1[0] == 'A') && (s8_Control_Drive_Model_Number_1[1] == 'S')) //AKRIBIS
      {
         p301_identity.vendorId = 0x000002E1; // TBD
         UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "ASD Drive");
      }
      else if ((s8_Control_Drive_Model_Number_1[0] == 'F') && (s8_Control_Drive_Model_Number_1[1] == 'P')) //MPC
      {
         p301_identity.vendorId = 0x00000678;
         UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "FPRO Drive");
      }
      if ((s8_Control_Drive_Model_Number_1[0] == 'M') && (s8_Control_Drive_Model_Number_1[1] == 'T'))  //PBA
      {
         p301_identity.vendorId = 0x000002BA;
         UpdateCanStrings(&p301_manu_device_name_desc, (char*)p301_manu_device_name, "MaxTune drive");
      }
   }

   // by default allow drive to activate canopen modes: interpolated and sync profile position, and send boot up message
   BGVAR(s16_CAN_Flags) |= CANOPEN_SYNC_PROF_POS_ACCESS_RIGHT_MASK;
   BGVAR(s16_CAN_Flags) |= CANOPEN_INTERPOLATED_ACCESS_RIGHT_MASK;

   //to have cuurect value of object 0x6060 on startup
   if (VAR(AX0_s16_Opmode) == 0)
      p402_modes_of_operation = 3;
   if (VAR(AX0_s16_Opmode) == 2)
      p402_modes_of_operation = 4;
   if (VAR(AX0_s16_Opmode) == 8)
      p402_modes_of_operation = 1;

   // update can opmode variable
   p402_modes_of_operation_display = p402_modes_of_operation;
   BGVAR(s16_CAN_Opmode) = p402_modes_of_operation;
   BGVAR(s16_CAN_Opmode_Temp) = p402_modes_of_operation;

   p402_controlword = 0;

   // init lexium profile drive vars
   u16_Lxm_DmCtrl_Prev.all = 0;
   u16_Lxm_DmCtrl.all = 0;
   u16_Lxm_MfStat.all = 0;
   u16_Lxm_DriveStat.all = 0;
   u16_Lxm_MfStat.all = 0;
   u16_Lxm_MotionStat.all = 0;
   u16_Lxm_DriveStat.all = 0;
   u16_Lxm_InputStat.all = 0;

   // event pdo init
   for (i=0; i<4; i++)
   {
       BGVAR(u64_CAN_Tpdo_Arr_Prev_Val)[i] = 0;
       for (j=0; j<4; j++)
       {
//            BGVAR(u32_CAN_Tpdo_Arr_Map_Addr)[i][j] = 0;
//            BGVAR(u32_CAN_Tpdo_Obj_Size)[i][j] = 0;

            BGVAR(s16_CAN_Tpdo_Arr_Pdo_Table_Index)[i][j] = -1;
       }
   }

   // all TPDOs need update (each bit is pdo flag)
   BGVAR(u16_CAN_Tpdo_Need_Update) = 0xf;

   // init can baud rate according to P3-01
   SchneiderInitCan(drive);
}


void HwFeaturesInit (void)
{
   if (!IS_HW_FUNC_ENABLED(SECONDARY_ENC_MASK))  BGVAR(s64_Faults_Mask)   &= ~(SEC_ENCODER_5V_OC_MASK);
   if (!IS_HW_FUNC_ENABLED(ANALOG_OUT_MASK   ))  BGVAR(u8_Analog_Out_Mode) = AN_OUT_NOT_SUPPORTED;
   if (!IS_HW_FUNC_ENABLED(FAULT_RELAY_MASK  ))  BGVAR(u16_Relay_Mode)     = 2;
}


void InitAll(int drive)
{
   // AXIS_OFF;
   unsigned int u16_shr,u16_temp;
   long long s64_fix, s64_half_for_rounding;
   float f_tmp = 0.0;

   // Configure Halls inputs according to u16_Halls_Type
   *((int *)FPGA_DIFFERENTIAL_HALL_SEL_REG_ADD) = BGVAR(u16_Halls_Type);

   if (BGVAR(u16_FdbkType_User) == AUTO_DETECTION_FDBK)
   {
      BGVAR(u16_FdbkType) = SERVOSENSE_SINGLE_TURN_COMM_FDBK;
      BGVAR(u16_MTP_Mode) = 3;  // Motor Type Plate for MPC Motors with ServoSense
      BGVAR(s16_Auto_FdbkType_Cntr) = 1;
      FdbkTypeCommand((long long)BGVAR(u16_FdbkType), drive);
   }
   else
      FdbkTypeCommand((long long)BGVAR(u16_FdbkType_User), drive);

   SFBTypeCommand(BGVAR(u16_SFBType_User), drive);

   /* update analog input related variables according to eeprom */
   VAR(AX0_u16_An_In_Internal_Dual_Gain) = IS_HW_FUNC_ENABLED(INTERNAL_DUAL_GAIN_MASK);

   if (VAR(AX0_u16_An_In_Internal_Dual_Gain))//if internal dual mode the mode is -1
      VAR(AX0_s16_An_In_2_Mode) = -1;

   AnalogInputConfig1(drive);
   AnalogInputConfig2(drive);
   AnalogTorqueConfig1(drive);
   AnalogTorqueConfig2(drive);
   AnalogVelocityConfig(drive);
   SalAnin1FiltModeCommand((long long)VAR(AX0_u16_Anin1_Msq_Fltr_Mode),drive);

   SalAnOutModeCommand((long long)BGVAR(u8_Analog_Out_Mode), drive);
   SalHomeTypeCommand((long long)(BGVAR(s16_Home_Type)), drive);

   SalAnin1OffsetCommand((long long)BGVAR(s16_An_In_1_Offset_User_Setting), drive); // Initialize "s16_An_In_1_Offset_User_Setting" and "AX0_s16_An_In_1_Offset"
   SalAnin2OffsetCommand((long long)BGVAR(s16_An_In_2_Offset_User_Setting), drive); // Initialize "s16_An_In_2_Offset_User_Setting" and "AX0_s16_An_In_2_Offset"

   BGVAR(s16_Hmi_Mode_Dspl_Stat) = BGVAR(s16_P0_02_Drive_Status); // Update the values displayed on the 7-segments according to P0-02 (e.g. after reboot)

   PtpFilterDesign(drive);

   if ((BGVAR(u16_MotorType) == LINEAR_MOTOR) || (LOAD_TYPE == LINEAR_MOTOR))// linear motor/load
   {
      SalUnitsVelLinearCommand((long long)BGVAR(u16_Units_Vel_Linear), drive);
      SalUnitsAccDecLinearCommand((long long)BGVAR(u16_Units_Acc_Dec_Linear), drive);
   }
   else  // rotary motor/load
   {
      SalUnitsVelRotaryCommand((long long)BGVAR(u16_Units_Vel_Rotary), drive);
      SalUnitsAccDecRotaryCommand((long long)BGVAR(u16_Units_Acc_Dec_Rotary), drive);
   }

   PositionModuloConfig(drive);
   DualLoopInit(drive);
   ConversionAninToUserUnitsPreperation(drive);

   BGVAR(u32_Fb_Motor_Rated_Current) = (unsigned long)BGVAR(s32_Motor_I_Cont);    // canopen micont and serial micont are both in mA

   f_tmp = (float)((float)BGVAR(u32_Fb_Motor_Rated_Current) * ((float)BGVAR(u32_Motor_Kt)/1000.0));
   p402_motor_rated_torque = (unsigned int)f_tmp;


   ConvertAmpToInternal(drive);
   ConvertInternalToAmp1000(drive);
   ConvertAnalogIOToInternal(drive);
   ConvertInternalToAnalogIO1000(drive);

   ConvertFbCanCurrentPerSecToUser(drive);
   ConvertFbCanCurrentPerSecToInternal(drive);
   ConvertInternalCurrentSlopeToUser(drive);
   ConvertInternalCurrentSlopeToInternal(drive);

   ConvertInternalToMsAccDecVel(drive);
   ConvertMsAccDecVelToInternal(drive);

   UnitsPosRotLinCommand(drive);
   UnitsVelRotLinRPSCommand(drive);
   UnitsMechAngleRotLinCommand(drive);
   UnitsVelRotLinRPMCommand(drive);
   UnitsRotLinKPPCommand(drive);
   UnitsRotLinENCRESCommand(drive);
   UnitsAmpVelRotLinCommand(drive);

   // Set all fieldbus units conversion.  All NV values are loaded, so just units conversion
   // setting are called directly.  This should depend on fieldbus type
   // if (ethercat || CAN) //CanOpen supported fieldbusses
   {
      ConvertFieldBusCountsToIntern(drive);
      ConvertInternToFieldBusCounts(drive);
      ConvertCanOpenFbusCycleTimeToSeconds(drive);
      ConvertFbCanOpenTorque(drive);
      ConvertFbCanOpenCurrent(drive);
      CalcFieldBusUserToIntern(drive);
      CalcInternToFieldBusUser(drive);
      ConvertFbCanOpenVelToInternalInLoop(drive);
   }
   UnitsPdenCommand(drive);
   UnitsPnumCommand(drive);
   // update crrnt_run_code according to limits
   // SalLimdisCommand((long?)BGVAR(s16_Limdis), drive);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;
   VAR(AX0_s16_Skip_Flags) |= SEC_ENC_CONFIG_NEEDED_MASK;
   CalcInternalParams(DRIVE_PARAM);

   SalHoldCommand(0LL, drive);

   BGVAR(s32_Prev_Counts_Per_Rev) = -1;

   DesignGearMsqFilter(drive);
   DesignHallsMsqFilter(drive);

   SalDecStopCommand(BGVAR(u64_DecStop), drive);
   SalDecStopTimeCommand(BGVAR(u16_DecStopTime), drive);

   // init event DEC for non LXM drive.
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
   {// not LXM drive configuration,
    // update all event decelerations to decstop value to maintain the decstop rate in all events
      BGVAR(u64_Motor_Stop_Dec_Rate) = BGVAR(u64_DecStop);
      BGVAR(u64_Ser_Comm_Time_Out_Dec_Rate) = BGVAR(u64_DecStop);
      BGVAR(u64_Pos_Comand_Overflow_Dec_Rate) = BGVAR(u64_DecStop);
      BGVAR(u64_N_SW_Limit_Dec_Rate) = BGVAR(u64_DecStop);
      BGVAR(u64_P_SW_Limit_Dec_Rate) = BGVAR(u64_DecStop);
      BGVAR(u64_P_HW_Limit_Dec_Rate) = BGVAR(u64_DecStop);
      BGVAR(u64_N_HW_Limit_Dec_Rate) = BGVAR(u64_DecStop);
   }

   //u16_Prev_Motor_Comm_Type is loaded with initialization indicator first
   u16_Prev_Motor_Comm_Type=0xffff;

   DesignRoutine(1, DRIVE_PARAM);

   if (BGVAR(u16_MotorType) == LINEAR_MOTOR)// linear motor
      SalUnitsPosLinearCommand((long long)BGVAR(u16_Units_Pos_Linear), drive);
   else// rotary motor or any other motor
      SalUnitsPosRotaryCommand((long long)BGVAR(u16_Units_Pos_Rotary), drive);

   ConvertInternalPWMCmdToVolts1000(drive);
   EncoderSimResCommand((long long)BGVAR(s32_Enc_Sim_Res), drive);
   if (IS_EC_LITE_DRIVE)
   {
      BGVAR(u8_EncoutMode) = 0;
   }
   SalEncoderSimMode((long long)BGVAR(u8_EncoutMode), drive);

   u16_temp = VAR(AX0_s16_Opmode);
   // This is to make sure all pointers are set if opmode change on-the-fly is used
   SetOpmode(drive, 8);
   SetOpmode(drive, 0);
   SetOpmode(drive, u16_temp); // Set the users opmode

   SalPoscontrolModeCommand((long long)VAR(AX0_u16_Pos_Control_Mode), drive);
   SalCompModeCommand((long long)BGVAR(u8_CompMode), drive);

   SalOutOfRangeCommand((long long)BGVAR(u16_Out_of_Range_Percent), drive);
   SalGearOutCommand((long long)LVAR(AX0_u32_Gearout_Design), drive);

   InitDigIOsFunctions(drive);
   ZPulseUpdate();
   WriteInputPolarityToFPGA();

   SalIstopCommand((long long)BGVAR(u32_Stop_Current), drive);

   // This is to cause SalDisableModeCommand to work
   u16_temp = BGVAR(u16_Disable_Mode_User);
   BGVAR(u16_Disable_Mode) = 0xFFFF;
   SalDisableModeCommand((long long)u16_temp, drive);

   CurrentFfLpfConfig(drive);

   PhaseFindCurrentCommand((long long)BGVAR(s32_PhaseFind_Current), drive);
   PhaseFindTimeCommand((long long)BGVAR(u16_PhaseFind_Time), drive);
   PhaseFindDeltaCommand((long long)BGVAR(u16_PhaseFind_Delta), drive);

   //init analog current limiter
   VAR(AX0_u16_Analog_Crrnt_Limit) = 26214;
   BGVAR(s32_Ilim_Actual) = BGVAR(s32_Ilim_User);
   CalcILim(DRIVE_PARAM);

   //Calc DICONT in internal units for the use of reduced PWM frequency
   s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s64_half_for_rounding = 0LL;
   if (u16_shr > 0)
      s64_half_for_rounding = 1LL << (u16_shr - 1);
   BGVAR(s32_Drive_I_Cont_Internal)  = (long)((BGVAR(s32_Drive_I_Cont) * s64_fix + s64_half_for_rounding) >> u16_shr);

   Update_CW_CCW_Ind(0, 0, 0);

   FastOutInv(drive);
//   SalForcedTorqueCommand((long long)BGVAR(s32_Forced_Torque_Cmd), drive); - Return when SL implementation is back
   SalMoveSmoothAvgNumCommand((long long)BGVAR(u32_Move_Smooth_Avg), drive);

   CalcTorqueCmdLimitTable(drive);
   // Init speed command/limit table, which holds the values in internal units
   CalcSpeedCmdLimitTable(drive);


   SalIndexDurationTimeCommand((long long)(BGVAR(u16_Index_Duration)),drive);

   UpdateTorqueSlope(drive);
   IcmdSlopeEn(drive);

   MoveSmoothModeCommand((long long)VAR(AX0_u16_Move_Smooth_Mode),drive);
   SalMoveSmoothLpfCommand((long long)BGVAR(s16_Move_Smooth_Lpf_Hz_User), drive);

   // gain table initialization
//   u16_temp = BGVAR(u16_KnlGT_Mode);
//   BGVAR(u16_KnlGT_Mode) = 0;
   BGVAR(u64_Sys_Warnings) &= ~(GT_LMJR_LO_WRN_MASK|GT_LMJR_HI_WRN_MASK|GT_INVALID_ORDER_WRN_MASK);
//   BGVAR(s32_KnlGT_Factor) = -2;
//   BGVAR(s16_KnlGT_Index) = -1;
   //DIS_INERTIA_IDENT;
//   SalKnlGTModeCommand((long long)u16_temp,drive);

   // Re-calculate the maximum allowed regeneration resistor on-time after
   // initializing the regen-resistor parameters with data from the Flash
   RecalcMaxRegenOnTime(drive);

   // Initialize the baud-rate after reading data from Flash. This call is needed since
   // CPU registers have to be set in case that the baud-rate variable has changed.
   SalSerialBaudRateCommand((long long)BGVAR(u32_Serial_Baud_Rate), drive);

   UpdateHystVal(drive);

   // On SE always set MTPMODE=1 regardless of the value stored on the NV
   if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
   {
      if (BGVAR(u16_MTP_Mode_User) == 0)
         BGVAR(u16_MTP_Mode_User) = 1;
   }

   if (BGVAR(u16_FdbkType_User) == AUTO_DETECTION_FDBK)
      MtpModeInit(BGVAR(u16_MTP_Mode), drive);
   else
      SalSetMtpMode((long long)BGVAR(u16_MTP_Mode_User), drive);

   // in CDHD drive , set home dec to home acc.
   // in lxm28 drive, SchneiderInit() will overwrite u64_HomeDecRate with path(0) deceleration value (which is saved in FLASH).
   BGVAR(u64_HomeDecRate) = BGVAR(u64_HomeAccRate);

   // Init Lexium moving average filter variables
   SalWriteVelMovingAverageFilterTime((long long)BGVAR(u32_Vel_Moving_Average_Filter_Time), drive);

   // init all internal params that are effected by schnider p params.
   SchneiderInit(drive);
   HwFeaturesInit();
   // Defualt variable to be probed is position
   VAR(AX0_u16_TProbe_Src)=1;

   // In case of Higen product,
   // At EC drive,the display will be according the node_id (displaymode is 2)
   // else, according to regular behavior (displaymode is 0)
   if (strstr(s8_Control_Drive_Model_Number_1, "SDA7") != NULL)
   {
      if (IS_EC_DRIVE)
         BGVAR(u16_Display_Mode) = 2;
      else
         BGVAR(u16_Display_Mode) = 0;
   }

   // Function to initialize the Gantry mode
   InitGantryMode(drive, VAR(AX0_u16_Gantry_Mode));

   SalSyncModeCommand((long long)BGVAR(u8_Sync_Mode_User), drive);
   //Customer ID Dependent paramters
   //This paramters must be set to non-default values after drive reset S.C.
   if (u16_OV_Threshold != u16_Power1_Board_Eeprom_Buffer[OV_THRESH_ADDR])
   {
      if (CHAKRATEC == u16_Customer_Id)
      {
         SalOVThresholdCommand((long long)u16_OV_Threshold, drive);
      }
      else if (DEFAULT_CUSTOMER_ID == u16_Customer_Id)
      {
         u16_OV_Threshold = u16_Power1_Board_Eeprom_Buffer[OV_THRESH_ADDR];
      }
   }
   SalMotorEncResCommand((long long)BGVAR(u32_User_Motor_Enc_Res), drive);

}


void ExtractEepromContent(void)
{
   unsigned int u16_index, u16_checksum, u16_read_value, u16_Interface_Offset = 0;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

// Read key
   if (u16_Control_Board_Eeprom_Fault & NO_READ_AT_INIT_FAULT_MASK)
   {
      for (u16_index = 0; u16_index <= 4; u16_index++)
         u16_Key_Buffer[u16_index] = 0;
   }
   else
   {  // Extract the key
      u16_checksum = 0;
      for (u16_index = 0; u16_index <= 4; u16_index++)
      {
         u16_read_value = u16_Control_Board_Eeprom_Buffer[KEY_INTEGRITY__KEY_INTEGRITY_0__ADDR + u16_index];
         u16_Key_Buffer[u16_index] = u16_read_value;
         u16_checksum += u16_read_value & 0x00ff;
         u16_checksum += (u16_read_value >> 8) & 0x00ff;
      }
      if (u16_checksum & 0x00ff)
      {
         for (u16_index = 0; u16_index <= 4; u16_index++)
            u16_Key_Buffer[u16_index] = u16_index;
      }
   }

   u16_index = KEY_INTEGRITY__KEY_INTEGRITY_0__ADDR;
   u16_Control_Board_Eeprom_Buffer[u16_index++] = 0xff09;
   u16_Control_Board_Eeprom_Buffer[u16_index++] = 0xffff;
   u16_Control_Board_Eeprom_Buffer[u16_index++] = 0xffff;
   u16_Control_Board_Eeprom_Buffer[u16_index++] = 0xffff;
   u16_Control_Board_Eeprom_Buffer[u16_index++] = 0xffff;


// Read parameters from control EEPROM
   if (u16_Control_Board_Eeprom_Fault & (NO_READ_AT_INIT_FAULT_MASK | PARAM_STAMP_FAULT_MASK | PARAM_CHECKSUM_FAULT_MASK) )
   {
      u16_Control_Board_Rev = 0xFFFF; // Initial value to allow printing in INFO Command
      u16_Control_Board_EE_Version = 1;
      u16_Num_Of_Axes       = 1;
      u16_Board_Type        = 0;
      u32_Hw_Features       = 0x000D7EF9L;
      u16_Adc_Resolution    = 14;
      u16_Serial_Flash      = 255;
      u16_Supported_Dig_Inputs_Mask = 0x7FF;
      u16_Supported_Dig_Outputs_Mask = 0x7F;
      u16_Product           = 255;
      s16_Num_Of_Inputs     = 11;
      s16_Num_Of_Outputs    = 8;
      u16_Fw_Features       = 0xFFFF;
      u16_More_Hw_Features  = 0xFFFF;
   }
   else
   {
      u16_Control_Board_Rev          = u16_Control_Board_Eeprom_Buffer[CONTROL_HW_REVISION_ADDR];
      u16_Control_Board_EE_Version   = (u16_Control_Board_Eeprom_Buffer[NUM_OF_AXES__EE_VERSION__ADDR] & 0xFF);
      u16_Num_Of_Axes                = ((u16_Control_Board_Eeprom_Buffer[NUM_OF_AXES__EE_VERSION__ADDR] >> 8) & 0xFF);
      u16_Board_Type                 = u16_Control_Board_Eeprom_Buffer[FPGA_SIZE__BOARD_TYPE__ADDR] & 0xFF;
      u32_Hw_Features                = (long)u16_Control_Board_Eeprom_Buffer[HW_FEATURES_LO_ADDR] | ((long)(u16_Control_Board_Eeprom_Buffer[DIG_OUTPUTS__HW_FEATURES_HI__ADDR] & 0xFF) << 16L);
      u16_Adc_Resolution             = (u16_Control_Board_Eeprom_Buffer[SERIAL_FLASH__ADC_RESOLUTION__ADDR] & 0xFF);
      u16_Serial_Flash               = ((u16_Control_Board_Eeprom_Buffer[SERIAL_FLASH__ADC_RESOLUTION__ADDR] >> 8) & 0xFF);
      u16_Supported_Dig_Inputs_Mask  = u16_Control_Board_Eeprom_Buffer[DIG_INPUTS_ADDR];
      u16_Supported_Dig_Outputs_Mask = ((u16_Control_Board_Eeprom_Buffer[DIG_OUTPUTS__HW_FEATURES_HI__ADDR] >> 8) & 0xFF);
      u16_Product                    = (u16_Control_Board_Eeprom_Buffer[MORE_DIG_OUTPUTS__PRODUCT__ADDR] & 0xFF);
      s16_Num_Of_Inputs              = (u16_Control_Board_Eeprom_Buffer[NUM_OF_DIG_OUT__NUM_OF_DIG_IN__ADDR] & 0xFF);;
      s16_Num_Of_Outputs             = ((u16_Control_Board_Eeprom_Buffer[NUM_OF_DIG_OUT__NUM_OF_DIG_IN__ADDR] >> 8) & 0xFF);
      u16_Fw_Features                = u16_Control_Board_Eeprom_Buffer[FW_FEATURES_ADDR];
      u16_More_Hw_Features           = (u16_Control_Board_Eeprom_Buffer[RES_FW_CODE_TYPE__MORE_HW_FEATURES__ADDR] & 0xFF);
      u16_Res_Code_Type              = ((u16_Control_Board_Eeprom_Buffer[RES_FW_CODE_TYPE__MORE_HW_FEATURES__ADDR] >> 8) & 0xFF);
      u16_Drive_Code_Type            = ((u16_Control_Board_Eeprom_Buffer[FPGA_IMAGE_TYPE__DRIVE_FW_CODE_TYPE__ADDR]) & 0xFF);

      u16_Dynamic_Pass               = ((u16_Control_Board_Eeprom_Buffer[DYN_PASS_ADDR] >> 8) & 0xFF);

      //M.A Debug need to be removed
//      u16_Fpga_Size = 0x2D;
//      u16_Product = 0x02;
//      u16_Board_Type = 0x02;

      if (s16_Num_Of_Inputs == 255) s16_Num_Of_Inputs     = 11;
      if (s16_Num_Of_Outputs == 255) s16_Num_Of_Outputs   = 8;

      if (s16_Num_Of_Outputs > 8)
      {
         u16_Supported_Dig_Outputs_Mask |= (u16_Control_Board_Eeprom_Buffer[MORE_DIG_OUTPUTS__PRODUCT__ADDR] & 0xFF00);
      }

      // Previous "enable" bit is now general purpose input 11
      if ( (255 == u16_Product) || (EC_LITE == u16_Product) )
      {
         u16_Supported_Dig_Inputs_Mask >>= 1;
         if (u16_Control_Board_Eeprom_Buffer[DIG_INPUTS_ADDR] & 0x01) u16_Supported_Dig_Inputs_Mask |= 0x0400;
      }

      // Indicate to FPGA whether to enable the controller i/f pulse & dir wire break detection (disabled by default)
      if (IS_HW_FUNC_ENABLED(CONTROLLER_PD_WIRE_BREAK_MASK))
         *((unsigned int*)FPGA_BOARD_REV_FEATURES_REG_ADD) |= ENABLE_CONTROLLER_PD_WIRE_BREAK_MASK;

      //Indicate to FPGA whether to enable the motor fb 5 volt failure detection (disabled by default)
      if (IS_HW_FUNC_ENABLED(MOTOR_FB_5V_FAILURE_MASK))
      {
         *((unsigned int*)FPGA_BOARD_REV_FEATURES_REG_ADD) |= ENABLE_MOTOR_FB_5V_FAILURE_MASK;
         BGVAR(s64_Faults_Mask) |= DRIVE_5V_FLT_MASK;
         drive = 1;
         BGVAR(s64_Faults_Mask) |= DRIVE_5V_FLT_MASK;
         drive = 0;
      }

      // Indicate to FPGA which i/f to use to read the rotary switches (undefined by default rotary read as 0)
      // Do not write to FPGA if the EEPROM content is not valid
      if (IS_HW_FUNC_ENABLED(ROTARY_IF_PARALLEL_MASK))
         *((unsigned int*)FPGA_BOARD_REV_FEATURES_REG_ADD) |= ROTARY_PARALLEL_IF_MASK;
      else
         *((unsigned int*)FPGA_BOARD_REV_FEATURES_REG_ADD) |= ROTARY_SERIAL_IF_MASK;

      //create product number string
      if (u16_Control_Board_Eeprom_Buffer[PRODUCT_NUMBER_START_ADDR] != 0xFFFF)
      { // if the eeprom allocated addr is not empty
         for (u16_index = 0; u16_index < 6; u16_index++)
         {
            u16_read_value = u16_Control_Board_Eeprom_Buffer[PRODUCT_NUMBER_START_ADDR + u16_index];

            s8_Product_Serial_Number[u16_index * 2]     = (u16_read_value & 0x00FF);
            s8_Product_Serial_Number[u16_index * 2 + 1] = ((u16_read_value >> 8) & 0x00FF);
         }
         s8_Product_Serial_Number[u16_index * 2] = 0;
      }
      else
      {
         // if eeprom is not initialized.
         for (u16_index = 0; u16_index < 6; u16_index++)
         {
            s8_Product_Serial_Number[u16_index * 2]     = '0';
            s8_Product_Serial_Number[u16_index * 2 + 1] = '0';
         }
         s8_Product_Serial_Number[u16_index * 2] = 0;
      }

      //create part number string
      if (u16_Control_Board_Eeprom_Buffer[PART_NUMBER_START_ADDR] != 0xFFFF)
      { // if the eeprom allocated addr is not empty
         for (u16_index = 0; u16_index < 7; u16_index++)
         {
            u16_read_value = u16_Control_Board_Eeprom_Buffer[PART_NUMBER_START_ADDR + u16_index];

            s8_Control_Board_Part_Number[u16_index * 2]     = (u16_read_value & 0x00FF);
            s8_Control_Board_Part_Number[u16_index * 2 + 1] = ((u16_read_value >> 8) & 0x00FF);
         }
         s8_Control_Board_Part_Number[u16_index * 2] =(u16_Control_Board_Eeprom_Buffer[PART_NUMBER_START_ADDR + u16_index] & 0xFF);
         s8_Control_Board_Part_Number[u16_index * 2 + 1] = 0;
      }

      //create serial number string
      if (u16_Control_Board_Eeprom_Buffer[SERIAL_NUMBER_START_ADDR] != 0xFFFF)
      { // if the eeprom allocated addr is not empty
         for (u16_index = 0; u16_index < 6; u16_index++)
         {
            u16_read_value = u16_Control_Board_Eeprom_Buffer[SERIAL_NUMBER_START_ADDR + u16_index];

            s8_Control_Board_Serial_Number[u16_index * 2]     = (u16_read_value & 0x00FF);
            s8_Control_Board_Serial_Number[u16_index * 2 + 1] = ((u16_read_value >> 8) & 0x00FF);
         }
         s8_Control_Board_Serial_Number[u16_index * 2] = 0;
      }

      // create drive model number part 1
      if (u16_Control_Board_Eeprom_Buffer[DRIVE_MODEL_1] != 0xFFFF)
      { // if the eeprom allocated addr is not empty
         for (u16_index = 0;u16_index < 3;u16_index++)
         {
            u16_read_value = u16_Control_Board_Eeprom_Buffer[DRIVE_MODEL_1 + u16_index];

            s8_Control_Drive_Model_Number_1[u16_index * 2]     = (u16_read_value & 0x00FF);
            s8_Control_Drive_Model_Number_1[u16_index * 2 + 1] = ((u16_read_value >> 8) & 0x00FF);
         }
         s8_Control_Drive_Model_Number_1[u16_index * 2 - 1] = 0;
      }

      // create drive model number part 3
      if (u16_Control_Board_Eeprom_Buffer[DRIVE_MODEL_3] != 0xFFFF)
      { // if the eeprom allocated addr is not empty
//         if (DDHD == u16_Product)
//         {
//            u16_read_value = u16_Control_Board_Eeprom_Buffer[DRIVE_MODEL_3];
//            s8_DDHD__Drive_Model_6th_Char[0] = (u16_read_value & 0x00FF);
//            s8_Control_Drive_Model_Number_3[0] = ((u16_read_value >> 8) & 0x00FF);
//            u16_Interface_Offset = 1; // DDHD I/F information in the ordering-information-string is located 1 char to the right.
//         }
         for (u16_index = 0; u16_index < 3; u16_index++)
         {
            u16_read_value = u16_Control_Board_Eeprom_Buffer[DRIVE_MODEL_3 + u16_index + u16_Interface_Offset];

            s8_Control_Drive_Model_Number_3[u16_index * 2 + u16_Interface_Offset]     = (u16_read_value & 0x00FF);
            s8_Control_Drive_Model_Number_3[u16_index * 2 + 1 + u16_Interface_Offset] = ((u16_read_value >> 8) & 0x00FF);
         }
         s8_Control_Drive_Model_Number_3[u16_index * 2] = 0;
      }
   }

   // Read parameters from power EEPROM
   drive = 0;
   if ( BGVAR(u16_Power_Board_Eeprom_Fault) &
        (NO_READ_AT_INIT_FAULT_MASK | PARAM_STAMP_FAULT_MASK | PARAM_CHECKSUM_FAULT_MASK |\
         NO_READ_BACK_FAULT_MASK | FPGA_BUFFER_CHECKSUM_MASK | FPGA_BUFFER_NOT_READY_MASK) )
   {
      BGVAR(u16_Power_Board_Rev)   = 0xFFFF; // Initial value to allow printing in INFO Command
      u16_Power_Board_EE_Version   = 1;
      BGVAR(u16_Vbus_Scale)        = 256;
      BGVAR(s32_Drive_I_Peak)      = 25455L;
      BGVAR(s32_Drive_I_Cont)      = 8485L;
      BGVAR(s16_Drive_I_Peak_Arms) = 180;
      BGVAR(s16_Drive_I_Cont_Arms) = 60;
      BGVAR(u16_Dead_Time)         = 2200;
      BGVAR(u32_Pwm_Freq)          = 16000L;
      BGVAR(u32_FoldD_Time)        = 2000L;
      BGVAR(u32_FoldT_Time)        = 2500L;
      BGVAR(u32_FoldR_Time)        = 31000L;
      BGVAR(s32_BurninCycleTime)   = 18000L;
      BGVAR(s32_BurninOnTime)      = 2000L;
      u16_Inrush_On_Value          = 110;
      u16_Inrush_Off_Value         = 85;
      u16_Regen_On_Value           = 400;
      u16_Regen_Off_Value          = 380;
      u16_OV_Threshold             = 420;
      BGVAR(u16_UV_Threshold_Default) = 100;
      u16_OT_Wrn_Threshold         = 100;
      u16_OT_Flt_Threshold         = 110;
      s16_Fan_Hi_Spd_Threshold     = 65;
      BGVAR(u16_Power_Hw_Features) = 0;
      BGVAR(u16_Power_Fw_Features) = 0;
      BGVAR(u16_Stall_Thresh_Temp) = 0xFFFF;
      BGVAR(u16_Current_Loop_Scale_Factor) = 0;
      BGVAR(u32_FoldD_At_Stall_Thresh_Temp) = 20;
      BGVAR(u32_FoldD_At_OT_Flt_Thresh_Temp) = 20;
      BGVAR(u32_Stall_Current_Thresh) = 0xFFFFFFFF;
      BGVAR(u16_Drive_Power_Watt) = 0;
      BGVAR(s16_Internal_Regen_Res_Ohm) = -1;
      BGVAR(s16_Internal_Regen_Power_Watt) = -1;
      BGVAR(u16_IPM_Temp_A_Coef) = 0;
      BGVAR(u16_IPM_Temp_B_Coef) = 0;
      BGVAR(u16_Current_Derating_Slope) = 15;
      BGVAR(u16_Current_Derating_Start_Temp) = 70;
      BGVAR(u16_IPM_Temp_Fan_Thresh) = 65;
      BGVAR(u16_IPM_Temp_OT_Thresh) = 115;
      BGVAR(u16_Drive_Power_Rating) = 1000;
   }
   else
   {
      BGVAR(u16_Power_Board_Rev)   = u16_Power1_Board_Eeprom_Buffer[POWER_HW_REVISION_ADDR];
      u16_Power_Board_EE_Version   = (u16_Power1_Board_Eeprom_Buffer[NUM_OF_AXES__EE_VERSION__ADDR] & 0xFF);
      BGVAR(u16_Vbus_Scale)        = u16_Power1_Board_Eeprom_Buffer[VBUS_SCALE_ADDR];
      BGVAR(s32_Drive_I_Peak)      = ((unsigned long)u16_Power1_Board_Eeprom_Buffer[DRIVE_I_PEAK_ADDR]) | ((unsigned long)u16_Power1_Board_Eeprom_Buffer[DRIVE_I_PEAK_ADDR + 1] << 16L);
      BGVAR(s32_Drive_I_Cont)      = ((unsigned long)u16_Power1_Board_Eeprom_Buffer[DRIVE_I_CONT_ADDR]) | ((unsigned long)u16_Power1_Board_Eeprom_Buffer[DRIVE_I_CONT_ADDR + 1] << 16L);

      BGVAR(s16_Drive_I_Peak_Arms) = u16_Power1_Board_Eeprom_Buffer[DRIVE_I_PEAK_ARMS_ADDR];
      BGVAR(s16_Drive_I_Cont_Arms) = u16_Power1_Board_Eeprom_Buffer[DRIVE_I_CONT_ARMS_ADDR];

      BGVAR(u16_Dead_Time)         = u16_Power1_Board_Eeprom_Buffer[DEAD_TIME_ADDR];
      BGVAR(u32_Pwm_Freq)          = ((unsigned long)u16_Power1_Board_Eeprom_Buffer[PWM_FRQ_ADDR]) | ((unsigned long)u16_Power1_Board_Eeprom_Buffer[PWM_FRQ_ADDR + 1] << 16L);
      BGVAR(u32_FoldD_Time)        = (unsigned long)u16_Power1_Board_Eeprom_Buffer[FOLDD_ADDR];
      BGVAR(u32_FoldT_Time)        = (unsigned long)u16_Power1_Board_Eeprom_Buffer[FOLDT_ADDR];
      BGVAR(u32_FoldR_Time)        = (unsigned long)u16_Power1_Board_Eeprom_Buffer[FOLDR_ADDR];
      BGVAR(s32_BurninCycleTime)   = u16_Power1_Board_Eeprom_Buffer[BURNIN_CYCLE_TIME_ADDR];
      BGVAR(s32_BurninOnTime)      = u16_Power1_Board_Eeprom_Buffer[BURNIN_ON_TIME_ADDR];
      u16_Inrush_On_Value          = u16_Power1_Board_Eeprom_Buffer[INRUSH_ON_TIME_ADDR];
      u16_Inrush_Off_Value         = u16_Power1_Board_Eeprom_Buffer[INRUSH_OFF_TIME_ADDR];
      u16_Regen_On_Value           = u16_Power1_Board_Eeprom_Buffer[REGEN_HI_TIME_ADDR];
      u16_Regen_Off_Value          = u16_Power1_Board_Eeprom_Buffer[REGEN_LO_TIME_ADDR];
      u16_OV_Threshold             = u16_Power1_Board_Eeprom_Buffer[OV_THRESH_ADDR];
      BGVAR(u16_UV_Threshold_Default) = u16_Power1_Board_Eeprom_Buffer[UV_THRESH_ADDR];
      u16_OT_Wrn_Threshold         = (u16_Power1_Board_Eeprom_Buffer[OT_FLT_THRESH__OT_WRN_THRESH__ADDR] & 0xFF);
      u16_OT_Flt_Threshold         = ((u16_Power1_Board_Eeprom_Buffer[OT_FLT_THRESH__OT_WRN_THRESH__ADDR] >> 8) & 0xFF);
      s16_Fan_Hi_Spd_Threshold     = (u16_Power1_Board_Eeprom_Buffer[POWER_HW_FEATURES__FAN_HI_SPD_THRESH__ADDR] & 0xFF);
      BGVAR(u16_Power_Hw_Features) = ((u16_Power1_Board_Eeprom_Buffer[POWER_HW_FEATURES__FAN_HI_SPD_THRESH__ADDR] >> 8) & 0xFF);
      if (BGVAR(u16_Power_Hw_Features) & 0x0001) // Test support for power (brake) output
      {
         u16_Power_Brake_Exist = 1;

         if (IS_EC_LITE_HV_DRIVE)
         {
            u16_HW_Supported_Dig_Outputs_Mask  = u16_Supported_Dig_Outputs_Mask;
            u16_Supported_Dig_Outputs_Mask |= 0x8;
         }
         else if ((u16_Product == 255) || (EC_LITE == u16_Product))
         {
            u16_Supported_Dig_Outputs_Mask |= 0x40;
         }
//         if (BGVAR(u16_Power_Hw_Features) & 0x0010)// power switch type
//         Note: only Fault 98 PWR_BRAKE_FAULT_MASK for both power switch types
         BGVAR(s64_Faults_Mask_2) |= PWR_BRAKE_FAULT_MASK;
      }
      AX0_u16_IPM_OT_DC_Enable = (BGVAR(u16_Power_Hw_Features) >> 3) & 0x0001;

       // Indicate to the FPGA if Regen_OC or fan_status is used.
       // In case of Regen_OC, some logic is used to consider Regen control
      if (BGVAR(u16_Power_Hw_Features) & 0x0020)
      {
         *((unsigned int*)FPGA_FAULT_RELAY_REG_ADD) |= 0x0002;  // fan_status is used.
      }
      else
      {
         *((unsigned int*)FPGA_FAULT_RELAY_REG_ADD) &= ~0x0002;  // Regen_OC is used.
      }

      BGVAR(u16_Power_Fw_Features) = (u16_Power1_Board_Eeprom_Buffer[STALL_THRESH_TEMP__POWER_FW_FEATURES__ADDR] & 0xFF);
      if (BGVAR(u16_Power_Fw_Features) == 0x00ff) // backward compatibility in case this memory location was not initialized
         BGVAR(u16_Power_Fw_Features) = 0;
      if ( (BGVAR(u16_Power_Fw_Features) & 0x0001) && (BGVAR(u32_Pwm_Freq) == 16000L) ) // Test need for reduce PWM freq at stall
         BGVAR(u16_Half_Pwm_Freq_Eeprom) = REDUCE_PWM_EEPROM;
      BGVAR(u16_Stall_Thresh_Temp) = ((u16_Power1_Board_Eeprom_Buffer[STALL_THRESH_TEMP__POWER_FW_FEATURES__ADDR] >> 8) & 0xFF);
      BGVAR(u16_Current_Loop_Scale_Factor) = u16_Power1_Board_Eeprom_Buffer[CURRENT_LOOP_SCALE_FACTOR];
      if (BGVAR(u16_Current_Loop_Scale_Factor) == 0xFFFF) BGVAR(u16_Current_Loop_Scale_Factor) = 0;
      BGVAR(u32_FoldD_At_Stall_Thresh_Temp) = (unsigned long)u16_Power1_Board_Eeprom_Buffer[FOLDD_AT_STALL_THRESH_TEMP_ADDR];
      BGVAR(u32_FoldD_At_OT_Flt_Thresh_Temp) = (unsigned long)u16_Power1_Board_Eeprom_Buffer[FOLDD_AT_OT_FLT_THRESH_ADDR];
      BGVAR(u32_Stall_Current_Thresh) = ((unsigned long)u16_Power1_Board_Eeprom_Buffer[STALL_CURRENT_THRESH_LO_ADDR]) | ((unsigned long)u16_Power1_Board_Eeprom_Buffer[STALL_CURRENT_THRESH_HI_ADDR] << 16L);
      BGVAR(u16_Drive_Power_Watt) = u16_Power1_Board_Eeprom_Buffer[CONTINUOUS_POWER_WATT_ADDR];
      if (BGVAR(u16_Drive_Power_Watt) == 0xffff)
      {
         BGVAR(u16_Drive_Power_Watt) = 0;
      }
      BGVAR(s16_Internal_Regen_Res_Ohm) = (int)u16_Power1_Board_Eeprom_Buffer[REGEN_RESISTANCE_OHM_ADDR];
      BGVAR(s16_Internal_Regen_Power_Watt) = (int)u16_Power1_Board_Eeprom_Buffer[REGEN_POWER_WATT_ADDR];

      BGVAR(u16_IPM_Temp_A_Coef) = (u16_Power1_Board_Eeprom_Buffer[IPM_TEMP_B_COEF__IPM_TEMP_A_COEF__ADDR] & 0xFF);
      BGVAR(u16_IPM_Temp_B_Coef) = ((u16_Power1_Board_Eeprom_Buffer[IPM_TEMP_B_COEF__IPM_TEMP_A_COEF__ADDR] >> 8) & 0xFF);
      BGVAR(u16_Current_Derating_Slope) = (u16_Power1_Board_Eeprom_Buffer[I_DERATING_START_TEMP__I_DERATING_SLOPE__ADDR] & 0xFF);
      BGVAR(u16_Current_Derating_Start_Temp) = ((u16_Power1_Board_Eeprom_Buffer[I_DERATING_START_TEMP__I_DERATING_SLOPE__ADDR] >> 8) & 0xFF);
      BGVAR(u16_IPM_Temp_Fan_Thresh) = (u16_Power1_Board_Eeprom_Buffer[IPM_TEMP_FLT_THRESH__IPM_TEMP_FAN_THRESH__ADDR] & 0xFF);
      BGVAR(u16_IPM_Temp_OT_Thresh) = ((u16_Power1_Board_Eeprom_Buffer[IPM_TEMP_FLT_THRESH__IPM_TEMP_FAN_THRESH__ADDR] >> 8) & 0xFF);
      BGVAR(u16_Drive_Power_Rating) = u16_Power1_Board_Eeprom_Buffer[CONTINUOUS_POWER_WATT_ADDR];

      //create part number string
      if (u16_Power1_Board_Eeprom_Buffer[PART_NUMBER_START_ADDR] != 0xFFFF)
      { // if the eeprom allocated addr is not empty
         for (u16_index = 0; u16_index < 7; u16_index++)
         {
            u16_read_value = u16_Power1_Board_Eeprom_Buffer[PART_NUMBER_START_ADDR + u16_index];
            s8_Power_Board_Part_Number[u16_index * 2]     = (u16_read_value & 0x00FF);
            s8_Power_Board_Part_Number[u16_index * 2 + 1] = ((u16_read_value >> 8) & 0x00FF);
         }
         s8_Power_Board_Part_Number[u16_index * 2] =(u16_Power1_Board_Eeprom_Buffer[PART_NUMBER_START_ADDR + u16_index] & 0xFF);
         s8_Power_Board_Part_Number[u16_index * 2 + 1] = 0;
      }

      //create serial number string
      if (u16_Power1_Board_Eeprom_Buffer[SERIAL_NUMBER_START_ADDR] != 0xFFFF)// if the eeprom allocated addr is not empty
      {
         for (u16_index = 0; u16_index < 6; u16_index++)
         {
            u16_read_value = u16_Power1_Board_Eeprom_Buffer[SERIAL_NUMBER_START_ADDR + u16_index];

            s8_Power_Board_Serial_Number[u16_index * 2]     = (u16_read_value & 0x00FF);
            s8_Power_Board_Serial_Number[u16_index * 2 + 1] = ((u16_read_value >> 8) & 0x00FF);
         }
         s8_Power_Board_Serial_Number[u16_index * 2] = 0;
      }

      // create drive model number part 3
      if (u16_Power1_Board_Eeprom_Buffer[DRIVE_MODEL_2] != 0xFFFF)// if the eeprom allocated addr is not empty
      {
         for (u16_index = 0; u16_index < 3; u16_index++)
         {
            u16_read_value = u16_Power1_Board_Eeprom_Buffer[DRIVE_MODEL_2 + u16_index];

            s8_Power_Drive_Model_Number_2[u16_index * 2]     = (u16_read_value & 0x00FF);
            s8_Power_Drive_Model_Number_2[u16_index * 2 + 1] = ((u16_read_value >> 8) & 0x00FF);
         }
         s8_Power_Drive_Model_Number_2[u16_index * 2] = 0;
         if  (u16_Product != DDHD) // Use 6 chars from Power-EEPROM in case of DDHD
            s8_Power_Drive_Model_Number_2[u16_index * 2 - 1] = 0;
      }
   }

   u16_Regen_Actual_On_Value = u16_Regen_On_Value;

   BGVAR(f_Stall_Foldd_Slope) = 0.0;
   if ( ((float)u16_OT_Flt_Threshold - (float)BGVAR(u16_Stall_Thresh_Temp)) != 0.0 )
   {
      BGVAR(f_Stall_Foldd_Slope) = ((float)BGVAR(u32_FoldD_At_OT_Flt_Thresh_Temp) -  (float)BGVAR(u32_FoldD_At_Stall_Thresh_Temp)) /
                               ((float)u16_OT_Flt_Threshold - (float)BGVAR(u16_Stall_Thresh_Temp));
   }
   if (BGVAR(f_Stall_Foldd_Slope) > 0.0) BGVAR(f_Stall_Foldd_Slope) = 0.0;

   SetDeatimeValue(BGVAR(u16_Dead_Time));

   u16_index = KEY_INTEGRITY__KEY_INTEGRITY_0__ADDR;
   u16_Power1_Board_Eeprom_Buffer[u16_index++] = 0xff09;
   u16_Power1_Board_Eeprom_Buffer[u16_index++] = 0xffff;
   u16_Power1_Board_Eeprom_Buffer[u16_index++] = 0xffff;
   u16_Power1_Board_Eeprom_Buffer[u16_index++] = 0xffff;
   u16_Power1_Board_Eeprom_Buffer[u16_index++] = 0xffff;
}


void EepromStartControl(void)
{
   unsigned int u16_index;

   for (u16_index = 0; u16_index < 128; u16_index++)
   {
      u16_Control_Board_Eeprom_Buffer[u16_index] = 0xffff;
   }

   u32_Init_Counter = 0;
   while ((CopyEepromToRam(CONTROL_BOARD_EEPROM, EEPROM_COPY) == NOT_FINISHED) && (++u32_Init_Counter < 1000000));
   // Call until copy sequence is completed. No timeout (for now).
   if (u32_Init_Counter >= 1000000)
   {
      u16_Control_Board_Eeprom_Fault |= NO_READ_AT_INIT_FAULT_MASK;
   }
}


void EepromStartPower(void)
{
   unsigned int u16_index;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for (u16_index = 0; u16_index < 128; u16_index++)
   {
      u16_Power1_Board_Eeprom_Buffer[u16_index] = 0xffff;
   }

   // All products have power EEPROM per axis, except DDHD.
   // DDHD has one power EEPROM that holds the information for both power stages. The EEPROM is accessed by DSP of u16_Axis_Num 0.
   if ( (u16_Product != DDHD) || (u16_Axis_Num == 0) )
   {
      u32_Init_Counter = 0;
      while ((CopyEepromToRam(POWER1_BOARD_EEPROM, EEPROM_COPY) == NOT_FINISHED) && (++u32_Init_Counter < 1000000));
      // Call until copy sequence is completed. No timeout (for now).
      if (u32_Init_Counter >= 1000000)
      {
         BGVAR(u16_Power_Board_Eeprom_Fault) |= NO_READ_AT_INIT_FAULT_MASK;
      }
   }

   if (u16_Product == DDHD)
      CopyEEpromDataBetweenDDHDFpgas();
   // In case of DDHD, if u16_Axis_Num 0 - copy EEPROM data to FPGA RAM, if u16_Axis_Num 1 - read EEPROM data from FPGA RAM
   //unsigned long u32_timer;
   // unsigned int u16_checksum, u16_index;
}


void EepromWriteTest(void)
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u32_Init_Counter = 0;
   while ((CopyEepromToRam(CONTROL_BOARD_EEPROM, EEPROM_TEST) == NOT_FINISHED) && (++u32_Init_Counter < 1000000));
   // Call until test sequence is completed. No timeout (for now).
   if (u32_Init_Counter >= 1000000)
   {
      u16_Control_Board_Eeprom_Fault |= NO_READ_AT_INIT_FAULT_MASK;
   }

   // All products have power EEPROM per axis, except DDHD.
   // DDHD has one power EEPROM that holds the information for both power stages. The EEPROM is accessed by DSP of u16_Axis_Num 0.
   if ( (u16_Product != DDHD) || (u16_Axis_Num == 0) )
   {
      u32_Init_Counter = 0;
      while ((CopyEepromToRam(POWER1_BOARD_EEPROM, EEPROM_TEST) == NOT_FINISHED) && (++u32_Init_Counter < 1000000));
      // Call until test sequence is completed. No timeout (for now).
      if (u32_Init_Counter >= 1000000)
      {
         BGVAR(u16_Power_Board_Eeprom_Fault) |= NO_READ_AT_INIT_FAULT_MASK;
      }
   }
}


void InitExternalADC(void)
{
   // if LXM28 and powered from RJ45 (in box programming), then do not activate the ADC in order to save power.
   if ( (u16_Product == SHNDR_HW) && (GpioDataRegs.GPBDAT.bit.GPIO60 == 0) )
   {
      GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;  // Put the ADC in stand by state
      return;
   }

   GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;     // Release the ADC from reset state
   DSP28x_usDelay(2000);                     // 20usec delay
   GpioDataRegs.GPBSET.bit.GPIO35 = 1;       // Set the reset signal of the ADC
   DSP28x_usDelay(2000);                     // 20usec delay
   GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;     // Release the ADC from reset state
   DSP28x_usDelay(2000);                     // 20usec delay

   // Set the ADC control register.
   // Internal conversion clock, range 4*Vref for all channels (+/-10 Volt), internal Vref of 2.5V
   // Write MSB (bits 31:16) then LSB (bits 15:0)
   if ((u16_More_Hw_Features & ADC_DEVICE_MASK) == 0)
   {  // Set the control register of the ADS855x.
      *(int *)FPGA_ADC_ADD = 0xe200;
      *(int *)FPGA_ADC_ADD = 0x03ff;
   }
   else
   {  // Set the control register of the ADS8548.
      *(int *)FPGA_ADC_ADD = 0x8000;
      *(int *)FPGA_ADC_ADD = 0x83ff;
   }
}


struct SECURITY_96_BIT
{
   unsigned long bits31_0;
   unsigned long bits63_32;
   unsigned long bits95_64;
} u96_code;


unsigned int Ones(unsigned long long u64_var) // Counts the number of 1's in the 56bit var
{
   unsigned int u16_ret_val = 0;
   unsigned long long u64_mask = 1LL;
   int i;

   for (i = 0; i < 56; i++)
   {
      if (u64_var & u64_mask) u16_ret_val++;
      u64_mask <<= 1LL;
   }

   return u16_ret_val;
}


// Shift the 56bit var padding the MSBs with the LSBs thrown for example: (X55X54..X0,4) = X3X2X1X0X55X54..X4
unsigned long long Rotate(unsigned long long u64_var, unsigned int u16_index)
{
   unsigned int i, bit;

   u64_var &= 0x00FFFFFFFFFFFFFF;
   for (i = 0; i < u16_index; i++)
   {
      bit = (unsigned int)(u64_var & 1LL);
      u64_var >>= 1LL;
      if (bit)
         u64_var |= 0x0080000000000000;
      else
         u64_var &= ~0xFF80000000000000;
   }

   return u64_var;
}


// Invert the bit value from 1-0 or vise versa, on locations which are a multiples of u16_index
unsigned long long Inverse(unsigned long long u64_var, unsigned int u16_index)
{
   unsigned int u16_mult = 1;

   if (u16_index == 0) return u64_var;

   while ((u16_index * u16_mult) < 56)
   {
      u64_var ^= 1LL << (unsigned long long)(u16_index * u16_mult);
      u16_mult++;
   }

   return u64_var;
}


unsigned long long u60_stored_value = 0LL;
void Security(void)
{
   unsigned long u32_timeout = 0L;
   unsigned int u16_dna_0 = 0, u16_dna_1 = 0, u16_dna_2 = 0, u16_dna_3 = 0;
   unsigned long long u56_fpga_code = 0LL, u64_xored_code = 0LL;
   unsigned long long u64_xored_code_odd = 0LL, u64_xored_code_even = 0LL;
   unsigned long long u56_calculated_key1 = 0LL, u56_stored_key = 0LL;
   unsigned long long u56_mask = 0x00DF5913CB582AFE;
   int i;

   // Check if FPGA is alive, if not dont release security
   *((int*)FPGA_TEST_REG_ADD) = 0x1234;
   if (*((int*)FPGA_TEST_REG_ADD) == 0x1234)
   {
      *((int*)FPGA_TEST_REG_ADD) = ~0x1234;
      if (*((int*)FPGA_TEST_REG_ADD) == ~0x1234)
      {
         // Wait for bit 57 of the FPGA's DNA to go high - means that the FPGA read the DNA
         while (((u16_dna_3 & 0x0100) == 0) && (u32_timeout < 1000000))
         {
            u16_dna_3 = *(int *)FPGA_DNA_REG_3_REG_ADD;
            u32_timeout++;
         }
         u16_dna_2 = *(int *)FPGA_DNA_REG_2_REG_ADD;
         u16_dna_1 = *(int *)FPGA_DNA_REG_1_REG_ADD;
         u16_dna_0 = *(int *)FPGA_DNA_REG_0_REG_ADD;
         u56_fpga_code = (((unsigned long long)u16_dna_3 & 0x00FF) << 48) | ((unsigned long long)u16_dna_2 << 32) | ((unsigned long long)u16_dna_1 << 16) | ((unsigned long long)u16_dna_0);

         if (u32_timeout < 1000000) // There was no timeout in the reading of the DNA
         {
            Security_Released = 0; // Indicate FGPA is alive

            // Calc Security algorithm and compare it to the key (see "CDHD Security Code Rev x_0.doc" for details)
            // if they match then Security_Released = 1

            // Create a 96bit code from FPGA's 56 bit code
            u64_xored_code = (u56_fpga_code >> 3) ^ (u56_fpga_code >> 13);
            u64_xored_code_odd = 0LL;
            u64_xored_code_even = 0LL;
            for (i = 54; i >= 0; i -= 2)
            {
               if ((u64_xored_code & (1LL << (i + 1))) > 0)
               u64_xored_code_odd |= 0x01;
               if ((u64_xored_code & (1LL << i)) > 0)
               u64_xored_code_even |= 0x01;
               if (i > 0)
               {
                  u64_xored_code_odd <<= 1;
                  u64_xored_code_even <<= 1;
               }
            }
            u96_code.bits31_0 = (unsigned long)(u64_xored_code_even & 0xFFFFF) | (((u56_fpga_code & 0xFFF) << 20LL) & 0xFFF00000);
            u96_code.bits63_32 = (unsigned long)((u56_fpga_code & 0xFFFFFFFF000) >> 12LL);
            u96_code.bits95_64 = (unsigned long)(((u56_fpga_code & 0xFFF00000000000) >> 44LL) & 0xFFF) | ((u64_xored_code_odd & 0x000FFFFF) << 12LL);

            u56_calculated_key1 = u56_fpga_code ^ Rotate(u56_mask, Ones(u56_fpga_code) + ((u96_code.bits31_0 + u96_code.bits63_32 + u96_code.bits95_64) % 7));
            u56_calculated_key1 = Inverse(u56_calculated_key1, Ones(u56_calculated_key1)) & 0x00FFFFFFFFFFFFFF;

            // Compare the calculated key to the stored key to decide if Security should be released
            u60_stored_value = u16_Key_Buffer[4];
            u60_stored_value <<= 16LL;
            u60_stored_value |= ((unsigned long long)u16_Key_Buffer[3]) & 0x0000FFFF;
            u60_stored_value <<= 16LL;
            u60_stored_value |= ((unsigned long long)u16_Key_Buffer[2]) & 0x0000FFFF;
            u60_stored_value <<= 16LL;
            u60_stored_value |= ((unsigned long long)u16_Key_Buffer[1]) & 0x0000FFFF;
            // Extract the writing times
            //debug_writing_trials = ((u60_stored_value >> 50) & 0x8) | ((u60_stored_value >> 39) & 0x4) | ((u60_stored_value >> 29) & 0x2) | ((u60_stored_value >> 14) & 0x1);

            // Extract the Key
            u56_stored_key = (u60_stored_value & 0x3FFF) | ((u60_stored_value & 0x3FFF8000) >> 1LL) | ((u60_stored_value & 0x1FF80000000) >> 2LL) | ((u60_stored_value & 0x1FFC0000000000) >> 3LL) | ((u60_stored_value & 0xFC0000000000000) >> 4LL);

            if (u56_stored_key == u56_calculated_key1) Security_Released = 1;
         }
      }
   }

   if (Security_Released <= 0) // This will cause the display to blink "b" if drive is not secured
   {
      if (u8_FPGA_ok) Display_State = 6;
      AX0_Security_Bit = 1;
   }
}


int ReadFPGACodeCommand(void)
{
   int i;
   unsigned long u32_temp = 0;

   if (Security_Released < 0) PrintString("NA - Make sure DSP and FPGA are running", 0);
   else
   {
      for (i = 28; i >= 0; i -= 4)
      {
         u32_temp = ((u96_code.bits95_64 & ((unsigned long)(0x000F) << (unsigned long)i)) >> (long)i) & 0xF;
         if (u32_temp < 0x0A) PrintChar('0' + (unsigned int)u32_temp);
         else PrintChar('A'+(unsigned int)(u32_temp-10L));
      }
      for (i = 28; i >= 0; i -= 4)
      {
         u32_temp = ((u96_code.bits63_32 & ((unsigned long)(0x000F) << (unsigned long)i)) >> (long)i) & 0xF;
         if (u32_temp < 0x0A) PrintChar('0' + (unsigned int)u32_temp);
         else PrintChar('A'+(unsigned int)(u32_temp - 10L));
      }
      for (i = 28; i >= 0; i -= 4)
      {
         u32_temp = ((u96_code.bits31_0 & ((unsigned long)(0x000F) << (unsigned long)i)) >> (long)i) & 0xF;
         if (u32_temp < 0x0A) PrintChar('0' + (unsigned int)u32_temp);
         else PrintChar('A'+(unsigned int)(u32_temp - 10L));
      }
   }
   PrintCrLf();

   return SAL_SUCCESS;
}


int SalWriteSecurityKeyCommand(void)
{
   unsigned long long u56_nv_key = 0LL, u60_temp_stored_value;
   unsigned int number_of_writing_trials = 0;
   unsigned int u16_return_value, u16_checksum, u16_temp_value, u16_index;

   u16_return_value = SAL_NOT_FINISHED;

   if (s16_Number_Of_Parameters == 0) return NOT_AVAILABLE; // Query

   if ((u16_Eeprom_Busy > 0) & (u16_Eeprom_Busy != KEY_AREA_EEPROM))  return (I2C_BUSY);

   if (u16_Eeprom_Busy == 0)
   {
      u56_nv_key = (s64_Execution_Parameter[2] >> 20LL) & ((unsigned long long)0x0000000000000FFF);
      u56_nv_key |= (s64_Execution_Parameter[1] & 0x00000000FFFFFFFF) << 12LL;
      u56_nv_key |= (s64_Execution_Parameter[0] & 0x0000000000000FFF) << 44LL;

      // Limit the number of writing trials
      // Read the number_of_writing_trials from the 60bit stored value
      number_of_writing_trials = ((u60_stored_value>>50) & 0x8) | ((u60_stored_value>>39) & 0x4) | ((u60_stored_value>>29) & 0x2) | ((u60_stored_value>>14) & 0x1);
      number_of_writing_trials++;
      if (number_of_writing_trials > 6) return NOT_AVAILABLE;

      // Implant the writing times to the key value at bits 14,30,41,53
      u60_stored_value = u56_nv_key;
      u60_stored_value = (u60_stored_value & 0x3FFF) | ((u60_stored_value & 0xFFFFFFFFFFFC000) << 1LL) | ((unsigned long long)(number_of_writing_trials & 0x01) << 14LL);
      u60_stored_value = (u60_stored_value & 0x3FFFFFFF) | ((u60_stored_value & 0xFFFFFFFFC0000000) << 1LL) | ((unsigned long long)(number_of_writing_trials & 0x02) << 29LL);
      u60_stored_value = (u60_stored_value & 0x1FFFFFFFFFF) | ((u60_stored_value & 0xFFFFFE0000000000) << 1LL) | ((unsigned long long)(number_of_writing_trials & 0x04) << 39LL);
      u60_stored_value = (u60_stored_value & 0x1FFFFFFFFFFFFF) | ((u60_stored_value & 0xFE0000000000000) << 1LL) | ((unsigned long long)(number_of_writing_trials & 0x08) << 50LL);

      // Arrange the data in buffer
      u16_checksum = 0;
      u60_temp_stored_value = u60_stored_value;
      for (u16_index = 1; u16_index <= 4; u16_index++)
      {
         u16_temp_value = (unsigned int)u60_temp_stored_value;
         u16_Key_Buffer[u16_index] = u16_temp_value;
         u16_checksum += u16_temp_value & 0x00ff;
         u16_checksum += (u16_temp_value >> 8) & 0x00ff;
         u60_temp_stored_value >>= 16;
      }
      u16_Key_Buffer[0] = (-u16_checksum) << 8;
      // Write code to EE
      p_u16_Eeprom_Buffer = u16_Key_Buffer;
      u16_Write_Eeprom_Board = CONTROL_BOARD_EEPROM;
      u16_Eeprom_Address = CONTROL_EEPROM_ADDR;
      u16_Eeprom_Busy = KEY_AREA_EEPROM;
      u16_Save_Eeprom_State = WRITE_PREPARATION_CONT;
   }
   else
   {
      // Wait for the write operation to end
      if (u16_Save_Eeprom_State == WRITE_EEPROM_IDLE)
      {
         u16_Eeprom_Busy = 0;
         u16_return_value = SAL_SUCCESS;
      }
   }

   return (u16_return_value);
}


void InitFilteredSignals(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   BGVAR(s16_Adc_Mfb_Motor_Temp_Filtered)    = VAR(AX0_s16_Adc_Mfb_Motor_Temp);
   s16_Adc_Plus_15v_Readout_Filtered  = 12287;  // make readout to be +15V
   s16_Adc_Minus_15v_Readout_Filtered = 3276;   // make readout to be -15V
}


unsigned int DpramRead(unsigned int address)
{
// the DPRAM addrss consists of 2KW blocks (bits 10-0), and 2 bits selector (bits 12-11).
   *(unsigned int*)UB_2_TI_BRAM_SELECT_REG_ADD = (unsigned int)((address >> 11) & 0x0003);   // write the selector bits
   address = DPRAM_BASE_ADDR + (address & 0x07ff);   // isolate the address within the 2KW block and add DPRAM base address
   return(*(unsigned int *)((unsigned long)address));
}


void DpramWrite(unsigned int address, unsigned int data)
{
// the DPRAM addrss consists of 2KW blocks (bits 10-0), and 2 bits selector (bits 12-11).
   *(unsigned int*)UB_2_TI_BRAM_SELECT_REG_ADD = (unsigned int)((address >> 11) & 0x0003);   // write the selector bits
   address = DPRAM_BASE_ADDR + (address & 0x07ff);   // isolate the address within the 2KW block and add DPRAM base address
   *(unsigned int *)((unsigned long)address) = data;
}


void LoadMicroBlaze(void)
{// At the moment, the srec handling is not general, but limitted to only few types that fit micro Blaze output file
   unsigned int u16_flash_word, u16_srec_type, u16_srec_length, u16_addr_continuous;
   unsigned int u16_fieldbus_version_str_copy_index = 0, u16_fieldbus_version_str_copy_len = 0;
   int s16_srec_word_length;

   u16_Load_Micro_Blaze_Exit_Code = 0;

   if ((u32_Hw_Features & ETHERNET_BASED_FIELDBUS_MASK) == 0)
   {
      u16_Load_Micro_Blaze_Exit_Code = 1;    // there is no uBlaze in this hardware
      return;
   }

   if (DpramRead(MB_DPRAM_CODE_ADDR) != MB_DPRAM_EMPTY)
   {
      u16_Load_Micro_Blaze_Exit_Code = 2;    // DPRAM is not empty
      return;
   }

   s32_Micro_Blaze_Time_Out = 1000000;
   u16_flash_word = DpramRead(MB_DPRAM_TOGGLE_ADDR);
   while ( (DpramRead(MB_DPRAM_TOGGLE_ADDR) == u16_flash_word) && (s32_Micro_Blaze_Time_Out-- > 0) ){}  // wait for the micro Blase write different values
   if (s32_Micro_Blaze_Time_Out > 0)
   {
      s32_Micro_Blaze_Time_Out = 80000;
      u16_flash_word = DpramRead(MB_DPRAM_TOGGLE_ADDR);
      while ( (DpramRead(MB_DPRAM_TOGGLE_ADDR) == u16_flash_word) && (s32_Micro_Blaze_Time_Out-- > 0) ){}  // wait for the micro Blase write different values
      if (s32_Micro_Blaze_Time_Out > 0)
      {
         s32_Micro_Blaze_Time_Out = 80000;
         u16_flash_word = DpramRead(MB_DPRAM_TOGGLE_ADDR);
         while ( (DpramRead(MB_DPRAM_TOGGLE_ADDR) == u16_flash_word) && (s32_Micro_Blaze_Time_Out-- > 0) ){}  // wait for the micro Blase write different values
         if (s32_Micro_Blaze_Time_Out <= 0)
         {
            u16_Load_Micro_Blaze_Exit_Code = 3;    // micro Blaze bootloader is not running (micro Blaze bootloader should write counter value to this location)
            return;
         }
      }
      else
      {
         u16_Load_Micro_Blaze_Exit_Code = 4;    // micro Blaze bootloader is not running (micro Blaze bootloader should write counter value to this location)
         return;
      }
   }
   else
   {
      u16_Load_Micro_Blaze_Exit_Code = 5;    // micro Blaze bootloader is not running (micro Blaze bootloader should write counter value to this location)
      return;
   }

   if ((u16_Flash_Type == 1) && ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)))   // 2 * 64Mb flash
   {
      if (u16_DSP_Type == DSP_TMS320C28346)   // NEW DSP (with more internal RAM)
      {
         // The resident verifies micro Blaze image integrity (checksum) and stamp before jumping to drive firmware.
         // The fieldbus type is stored in user parameter.
         // Execute from RAM since micro Blaze image resides in different flash page

         if (u16_Fieldbus_Type == FB_ETHERCAT)
         {
            LoadMicroBlazeOfLXM28E((unsigned long)LXM28E_EtherCAT_UBLAZE_START_ADDRESS, (unsigned int)LXM28E_EtherCAT_UBLAZE_FLASH_PAGE);
         }
         else if (u16_Fieldbus_Type == FB_SERCOS)
         {
            LoadMicroBlazeOfLXM28E((unsigned long)LXM28E_SERCOS_UBLAZE_START_ADDRESS, (unsigned int)LXM28E_SERCOS_UBLAZE_FLASH_PAGE);
         }
         else if (u16_Fieldbus_Type == FB_ETHERNET_IP)
         {
            LoadMicroBlazeOfLXM28E((unsigned long)LXM28E_EthernetIP_UBLAZE_START_ADDRESS, (unsigned int)LXM28E_EthernetIP_UBLAZE_FLASH_PAGE);
         }
         else
         {
            u16_Load_Micro_Blaze_Exit_Code = 22;
         }

         if (u16_Load_Micro_Blaze_Exit_Code > 0)
         {
            return;
         }

      }
      else
      {
         u16_Load_Micro_Blaze_Exit_Code = 21;    // When 64Mb flash is mounted, micro Blaze image is expected to reside in different flash page,
         return;                                 // and to access this page TMS320C28346 DPS is needed. Exit if the DSP is not TMS320C28346.
      }
   }
   else
   {
      InitReadMicroBlazeWordFromFlash((unsigned long)EtherCAT_UBLAZE_START_ADDRESS);
      u32_Micro_Blaze_Dpram_Total_Num_Of_Words = 0;    // for analysis

      // read the header of the first record
      u16_flash_word = ReadMicroBlazeWordFromFlash();
      u16_srec_type = (u16_flash_word >> 8) & 0x00ff;
      u16_srec_length = u16_flash_word & 0x00ff;
      s16_srec_word_length = ((u16_srec_type + u16_srec_length) >> 1) - 2;

      if (u16_srec_type != 0x0003)
      {
         u16_Load_Micro_Blaze_Exit_Code = 6;    // First record is not S3
         TerminateReadMicroBlazeWordFromFlash();
         return;
      }

      u16_Micro_Blaze_DDR3_Addr_Hi = ReadMicroBlazeWordFromFlash();
      s16_srec_word_length--;
      u16_Micro_Blaze_DDR3_Addr_Lo = ReadMicroBlazeWordFromFlash();
      s16_srec_word_length--;

      DisplayDuringMicroBlazeLoad((int)0);       // init the display

      while (u16_srec_type == 0x0003)
      {
         DisplayDuringMicroBlazeLoad((int)1);    // display during loading (end at TerminateReadMicroBlazeWordFromFlash() fuction)

         DpramWrite(MB_DPRAM_DDR_ARRD_HI_ADDR, u16_Micro_Blaze_DDR3_Addr_Hi);
         DpramWrite(MB_DPRAM_DDR_ARRD_LO_ADDR, u16_Micro_Blaze_DDR3_Addr_Lo);
         u16_Micro_Blaze_Dpram_Num_Of_Words = 0;
         u16_addr_continuous = 1;

         u16_Micro_Blaze_Dpram_Write_Addr = 4;
         u32_Micro_Blaze_DDR3_Addr = ((unsigned long)u16_Micro_Blaze_DDR3_Addr_Hi << 16) | ((unsigned long)u16_Micro_Blaze_DDR3_Addr_Lo & 0x0000ffffL);

         while ( (u16_srec_type == 0x0003) && (u16_Micro_Blaze_Dpram_Num_Of_Words < 8000) && (u16_addr_continuous == 1) )
         {
            while (s16_srec_word_length > 0)    // copy the data sequence of one record from flash to DPRAM
            {
               DpramWrite(u16_Micro_Blaze_Dpram_Write_Addr++, ReadMicroBlazeWordFromFlash());
               u16_Micro_Blaze_Dpram_Num_Of_Words++;
               u32_Micro_Blaze_Dpram_Total_Num_Of_Words++;    // for analysis
               u32_Micro_Blaze_DDR3_Addr += 2;
               s16_srec_word_length--;
            }

            // read the header of the next record
            u16_flash_word = ReadMicroBlazeWordFromFlash();
            u16_srec_type = (u16_flash_word >> 8) & 0x00ff;
            u16_srec_length = u16_flash_word & 0x00ff;
            s16_srec_word_length = u16_srec_type + u16_srec_length;
            if (s16_srec_word_length & 0x0001)   // if the byte number is odd, add 1 (srec2hex adds a byte in this case)
            {
               s16_srec_word_length++;
            }
            s16_srec_word_length = (s16_srec_word_length >> 1) - 2;

            u16_Micro_Blaze_DDR3_Addr_Hi = ReadMicroBlazeWordFromFlash();
            s16_srec_word_length--;
            u16_Micro_Blaze_DDR3_Addr_Lo = ReadMicroBlazeWordFromFlash();
            s16_srec_word_length--;

            if (u32_Micro_Blaze_DDR3_Addr != (((unsigned long)u16_Micro_Blaze_DDR3_Addr_Hi << 16) | ((unsigned long)u16_Micro_Blaze_DDR3_Addr_Lo & 0x0000ffffL)) )
            {
               u16_addr_continuous = 0;    // address jumps
            }
         }

         DpramWrite(MB_DPRAM_LENGTH_ADDR, u16_Micro_Blaze_Dpram_Num_Of_Words);

         if (u16_srec_type == 0x0003)      // data record of SREC file
         {
            DpramWrite(MB_DPRAM_CODE_ADDR, MB_DPRAM_DATA_AVAILABLE);
         }
         else if (u16_srec_type == 0x0007) // end record of SREC file (there are few options for end code, here we check only the option used by uBlaze)
         {
            DpramWrite(MB_DPRAM_CODE_ADDR, MB_DPRAM_LAST_DATA);
         }
         else
         {
            u16_Load_Micro_Blaze_Exit_Code = 8;              // record is not S3 (data) or S7 (end)
            DpramWrite(MB_DPRAM_CODE_ADDR, MB_DPRAM_ABORT);  // cause the uBlaze to clear the DPRAM
            TerminateReadMicroBlazeWordFromFlash();
            return;
         }

         s32_Micro_Blaze_Time_Out = 80000;
         while ( (DpramRead(MB_DPRAM_CODE_ADDR) != MB_DPRAM_EMPTY) && (s32_Micro_Blaze_Time_Out-- > 0) ){}  // wait for the micro Blase to read the DPRAM

         if (s32_Micro_Blaze_Time_Out <= 0)
         {
            u16_Load_Micro_Blaze_Exit_Code = 7;    // exit on timeout
            TerminateReadMicroBlazeWordFromFlash();
            return;
         }
      }

      TerminateReadMicroBlazeWordFromFlash();
   }

   // Read the fieldBus version from the MicroBlaze.
   s32_Micro_Blaze_Time_Out = 260000; // 130000 approx 1/2 sec (tested about 452 mSec) timeout value -> ( ~288 = 1mSec)
   while ( (DpramRead(MB_DPRAM_CODE_ADDR) == MB_DPRAM_EMPTY) && (s32_Micro_Blaze_Time_Out-- > 0) ){}  // wait for the micro Blaze to
                                                                                   // write the FieldBus version to the DPRAM
                                                                                   // and change the CODE section to value !=0.
   if (s32_Micro_Blaze_Time_Out > 0)
   {// the Micro Blaze sent the version without timeout, store the received FieldBus version string.
       u16_fieldbus_version_str_copy_len = (15);//stop the copy one index before the end of the string maxsize 30 -> (15*2)

      for(u16_fieldbus_version_str_copy_index=0;\
      u16_fieldbus_version_str_copy_index < u16_fieldbus_version_str_copy_len;\
      u16_fieldbus_version_str_copy_index++)
      {// read the MB version string from the DPRAM (the version is writen in the DPRAM as WORD)
         s8_FieldBus_Version[(u16_fieldbus_version_str_copy_index * 2)]     = (char)(DpramRead(2+u16_fieldbus_version_str_copy_index));
       s8_FieldBus_Version[(u16_fieldbus_version_str_copy_index * 2)] &= 0x00FF;
         s8_FieldBus_Version[(u16_fieldbus_version_str_copy_index * 2 + 1)] = (char)(DpramRead(2+u16_fieldbus_version_str_copy_index)>>8);
       s8_FieldBus_Version[(u16_fieldbus_version_str_copy_index * 2 + 1)] &= 0x00FF;

         //clear the DPRAM from the MB string after each word read.
         DpramWrite(2+u16_fieldbus_version_str_copy_index,0x0000);
      }
      DpramWrite(MB_DPRAM_CODE_ADDR,0);        //reset the code section to ZERO to let the microblaze continue running.
      u16_Load_Micro_Blaze_Exit_Code = 100;    // exit normally
   }
   else
   {// timedout
       u16_Load_Micro_Blaze_Exit_Code = 99;    // FieldBus version receive timedout
   }
   // exit code >= 90 inidcates successuflly (used in Background.c and in FltCntrl.c)
}


void InitReadMicroBlazeWordFromFlash(unsigned long u32_start_addr)
{
   u32_Micro_Blaze_Flash_Addr = u32_start_addr;
   u32_Micro_Blaze_Serial_Flash_Addr = 0L;

   if (IS_PN_DRIVE)
   {
      InitSpiaGpio();

      SerialFlashSetChipSelect(LOW);
      SerialFlashSendReceiveByte(READ_DATA_BYTE_CODE);
      SerialFlashSendReceiveByte(0);    // send bits 23-16 of the address
      SerialFlashSendReceiveByte(0);    // send bits 15-8 of the address
      SerialFlashSendReceiveByte(0);    // send bits 7-0 of the address
   }
}


unsigned int ReadMicroBlazeWordFromFlash(void)
{
   unsigned int u16_read_word;

   if (IS_PN_DRIVE)
   {
      // For now don't handle the case where read fails (returns 0xffff). Need to think how to handle it.
      u16_read_word  = (SerialFlashSendReceiveByte(0) << 8) & 0xff00;
      u16_read_word |= (SerialFlashSendReceiveByte(0) & 0x00ff);
      u32_Micro_Blaze_Serial_Flash_Addr += 2;    // for analysis
   }
   else
   {
      u16_read_word = *(unsigned int *)u32_Micro_Blaze_Flash_Addr++;
   }

   return (u16_read_word);
}


void TerminateReadMicroBlazeWordFromFlash(void)
{
   if (IS_PN_DRIVE)
   {
      SerialFlashSetChipSelect(HIGH);

      ReleaseSpiaGpio();
   }

   HWDisplay(ALL, 4);
}


void DisplayDuringMicroBlazeLoad(int mode)
{
   if (IS_PN_DRIVE)
   {
      if (mode == 0)   // init
      {
         u32_Micro_Blaze_Timer = CpuTimer0Regs.TIM.all;        // capture Timer0 value
         u16_Micro_Blaze_Display_State = 1;
         u16_Micro_Blaze_Display_Code = 0x00ff;
      }
      else
      {
         if ((u32_Micro_Blaze_Timer - CpuTimer0Regs.TIM.all) >= 100000000)    // 100,000,000 = 333.3 ms = 1/3 sec
         {
            u32_Micro_Blaze_Timer = CpuTimer0Regs.TIM.all;     // capture Timer0 value
            u16_Micro_Blaze_Display_Code <<= 1;
            if (u16_Micro_Blaze_Display_State >= 7) u16_Micro_Blaze_Display_Code |= 0x0001;
            u16_Micro_Blaze_Display_Code |= 0x00c0;
            u16_Micro_Blaze_Display_Code &= 0x00ff;
            HWDisplay(u16_Micro_Blaze_Display_Code, 4);
            u16_Micro_Blaze_Display_State++;
            if (u16_Micro_Blaze_Display_State > 12) u16_Micro_Blaze_Display_State = 1;
         }
      }
   }
}


#pragma CODE_SECTION(LoadMicroBlazeOfLXM28E, "ram346_1");
void LoadMicroBlazeOfLXM28E(unsigned long u32_start_addr, unsigned u16_ublaze_page)
{
   unsigned int u16_flash_word, u16_srec_type, u16_srec_length, u16_addr_continuous;
   int s16_srec_word_length;

   SetFlashPage(u16_ublaze_page);

   u32_Micro_Blaze_Flash_Addr = u32_start_addr;
   u32_Micro_Blaze_Dpram_Total_Num_Of_Words = 0;    // for analysis

   // read the header of the first record
   u16_flash_word = *(unsigned int *)u32_Micro_Blaze_Flash_Addr++;
   u16_srec_type = (u16_flash_word >> 8) & 0x00ff;
   u16_srec_length = u16_flash_word & 0x00ff;
   s16_srec_word_length = ((u16_srec_type + u16_srec_length) >> 1) - 2;

   if (u16_srec_type != 0x0003)
   {
      u16_Load_Micro_Blaze_Exit_Code = 26;    // First record is not S3
      SetFlashPage(0);
      return;
   }

   u16_Micro_Blaze_DDR3_Addr_Hi = *(unsigned int *)u32_Micro_Blaze_Flash_Addr++;
   s16_srec_word_length--;
   u16_Micro_Blaze_DDR3_Addr_Lo = *(unsigned int *)u32_Micro_Blaze_Flash_Addr++;
   s16_srec_word_length--;

   while (u16_srec_type == 0x0003)
   {
      DpramWriteRAM(MB_DPRAM_DDR_ARRD_HI_ADDR, u16_Micro_Blaze_DDR3_Addr_Hi);
      DpramWriteRAM(MB_DPRAM_DDR_ARRD_LO_ADDR, u16_Micro_Blaze_DDR3_Addr_Lo);
      u16_Micro_Blaze_Dpram_Num_Of_Words = 0;
      u16_addr_continuous = 1;

      u16_Micro_Blaze_Dpram_Write_Addr = 4;
      u32_Micro_Blaze_DDR3_Addr = ((unsigned long)u16_Micro_Blaze_DDR3_Addr_Hi << 16) | ((unsigned long)u16_Micro_Blaze_DDR3_Addr_Lo & 0x0000ffffL);

      while ( (u16_srec_type == 0x0003) && (u16_Micro_Blaze_Dpram_Num_Of_Words < 8000) && (u16_addr_continuous == 1) )
      {
         while (s16_srec_word_length > 0)    // copy the data sequence of one record from flash to DPRAM
         {
            DpramWriteRAM(u16_Micro_Blaze_Dpram_Write_Addr++, *(unsigned int *)u32_Micro_Blaze_Flash_Addr++);
            u16_Micro_Blaze_Dpram_Num_Of_Words++;
            u32_Micro_Blaze_Dpram_Total_Num_Of_Words++;    // for analysis
            u32_Micro_Blaze_DDR3_Addr += 2;
            s16_srec_word_length--;
         }

         // read the header of the next record
         u16_flash_word = *(unsigned int *)u32_Micro_Blaze_Flash_Addr++;
         u16_srec_type = (u16_flash_word >> 8) & 0x00ff;
         u16_srec_length = u16_flash_word & 0x00ff;
         s16_srec_word_length = u16_srec_type + u16_srec_length;
         if (s16_srec_word_length & 0x0001)   // if the byte number is odd, add 1 (srec2hex adds a byte in this case)
         {
            s16_srec_word_length++;
         }
         s16_srec_word_length = (s16_srec_word_length >> 1) - 2;

         u16_Micro_Blaze_DDR3_Addr_Hi = *(unsigned int *)u32_Micro_Blaze_Flash_Addr++;
         s16_srec_word_length--;
         u16_Micro_Blaze_DDR3_Addr_Lo = *(unsigned int *)u32_Micro_Blaze_Flash_Addr++;
         s16_srec_word_length--;

         if (u32_Micro_Blaze_DDR3_Addr != (((unsigned long)u16_Micro_Blaze_DDR3_Addr_Hi << 16) | ((unsigned long)u16_Micro_Blaze_DDR3_Addr_Lo & 0x0000ffffL)) )
         {
            u16_addr_continuous = 0;    // address jumps
         }
      }

      DpramWriteRAM(MB_DPRAM_LENGTH_ADDR, u16_Micro_Blaze_Dpram_Num_Of_Words);

      if (u16_srec_type == 0x0003)      // data record of SREC file
      {
         DpramWriteRAM(MB_DPRAM_CODE_ADDR, MB_DPRAM_DATA_AVAILABLE);
      }
      else if (u16_srec_type == 0x0007) // end record of SREC file (there are few options for end code, here we check only the option used by uBlaze)
      {
         DpramWriteRAM(MB_DPRAM_CODE_ADDR, MB_DPRAM_LAST_DATA);
      }
      else
      {
         u16_Load_Micro_Blaze_Exit_Code = 28;              // record is not S3 (data) or S7 (end)
         DpramWriteRAM(MB_DPRAM_CODE_ADDR, MB_DPRAM_ABORT);  // cause the uBlaze to clear the DPRAM
         SetFlashPage(0);
         return;
      }

      s32_Micro_Blaze_Time_Out = 800000;
      while ( (DpramReadRAM(MB_DPRAM_CODE_ADDR) != MB_DPRAM_EMPTY) && (s32_Micro_Blaze_Time_Out-- > 0) ){}  // wait for the micro Blase to read the DPRAM

      if (s32_Micro_Blaze_Time_Out <= 0)
      {
         u16_Load_Micro_Blaze_Exit_Code = 27;    // exit on timeout
         SetFlashPage(0);
         return;
      }
   }

   SetFlashPage(0);
}


#pragma CODE_SECTION(DpramWriteRAM, "ram346_1");
void DpramWriteRAM(unsigned int address, unsigned int data)
{
// the DPRAM addrss consists of 2KW blocks (bits 10-0), and 2 bits selector (bits 12-11).
   *(unsigned int*)UB_2_TI_BRAM_SELECT_REG_ADD = (unsigned int)((address >> 11) & 0x0003);   // write the selector bits
   address = DPRAM_BASE_ADDR + (address & 0x07ff);   // isolate the address within the 2KW block and add DPRAM base address
   *(unsigned int *)((unsigned long)address) = data;
}


#pragma CODE_SECTION(DpramReadRAM, "ram346_1");
unsigned int DpramReadRAM(unsigned int address)
{
// the DPRAM addrss consists of 2KW blocks (bits 10-0), and 2 bits selector (bits 12-11).
   *(unsigned int*)UB_2_TI_BRAM_SELECT_REG_ADD = (unsigned int)((address >> 11) & 0x0003);   // write the selector bits
   address = DPRAM_BASE_ADDR + (address & 0x07ff);   // isolate the address within the 2KW block and add DPRAM base address
   return(*(unsigned int *)((unsigned long)address));
}


//**********************************************************
// Function Name: InitCyclePowerPparams
// Description: init P params that should be updated only on init.
//              i.e. params value can be chanegd but new value takes effect only after cycle power.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void InitCyclePowerPparams(int drive)
{
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   BGVAR(u16_P2_68_Auto_Enable_Actual) = BGVAR(u16_P2_68_Auto_Enable);

   // Moshe: fix IPR#1104 allow nibble Z of P2-68 to be set only if drive is in FB mode otherwise clear it.
   if ( ((BGVAR(u16_P1_01_CTL) & 0x00FF) != SE_OPMODE_CANOPEN)     &&
        ((BGVAR(u16_P1_01_CTL) & 0x00FF) != SE_OPMODE_SERCOS)      &&
        ((BGVAR(u16_P1_01_CTL) & 0x00FF) != SE_OPMODE_ETHERNET_IP) &&
        ((BGVAR(u16_P1_01_CTL) & 0x00FF) != SE_OPMODE_ETHERCAT)      )
   {// drive not in FB mode - clear nibble Z of P2-68.
      BGVAR(u16_P2_68_Auto_Enable_Actual) &= 0xF0FF;
   }

   // this functions write into u16_P2_68_Auto_Enable, so dont send u16_P2_68_Auto_Enable_Actual as in parameter.
   // the function does not handle nibble Z, so sending u16_P2_68_Auto_Enable instead of u16_P2_68_Auto_Enable_Actual does nto matter.
   //SalWriteAutoEnableAutoLimitCommand(BGVAR(u16_P2_68_Auto_Enable),drive);

   BGVAR(u16_P3_00_ADR_Current) = BGVAR(u16_P3_00_ADR);
   BGVAR(u16_P1_01_CTL_Current) = BGVAR(u16_P1_01_CTL);
   SetSchneiderOpmode(drive , BGVAR(u16_P1_01_CTL_Current));

   // the next 2 functions depends on =s= opmode, so need to be executed after SetSchneiderOpmode() is called.
   InitSchneiderDigitalIODefaults(0);
   //SalWriteAutoRunDOMode(BGVAR(u16_P2_44_Autor_DoModeSet), drive);
}


//**********************************************************
// Function Name: SetSchneiderOpmode
// Description: this function sets the Drive opmode according to the value of PCL (P1-01)
//  in case of dual mode, opmode will be set to the first mode of the two.
//  in case of canopen mode, use opmode that is saved in nvram.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void SetSchneiderOpmode(int drive , int s16_se_opmode)
{
   int s16_cdhd_opmode = -1, s16_doOpmode = 1, s16_use_canopen_opmode_change = 0, drive_backup = drive;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   s16_se_opmode = s16_se_opmode & 0xFF;

   switch(s16_se_opmode)
   {
      case SE_OPMODE_PT:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_PT;
         s16_cdhd_opmode = 4;
      break;
      case SE_OPMODE_PS:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_PS;
         s16_cdhd_opmode = 8;
      break;
      case SE_OPMODE_S:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_S;
         s16_cdhd_opmode = 0;
      break;
      case SE_OPMODE_T:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_T;
         s16_cdhd_opmode = 3;
      break;
      case SE_OPMODE_SZ:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_SZ;
         s16_cdhd_opmode = 0;
      break;
      case SE_OPMODE_TZ:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_TZ;
         s16_cdhd_opmode = 2;
      break;
      case SE_OPMODE_PT_S:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_PT;
         s16_cdhd_opmode = 4;
      break;
      case SE_OPMODE_PT_T:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_PT;
         s16_cdhd_opmode = 4;
      break;
      case SE_OPMODE_PS_S:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_PS;
         s16_cdhd_opmode = 8;
      break;
      case SE_OPMODE_PS_T:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_PS;
         s16_cdhd_opmode = 8;
      break;
      case SE_OPMODE_S_T:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_S;
         s16_cdhd_opmode = 1;
      break;
      case SE_OPMODE_CANOPEN:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_CANOPEN;
         s16_cdhd_opmode = 8;    // set OPMODE=8 by default for canopen mode.
         s16_use_canopen_opmode_change = 1;
      break;
      case SE_OPMODE_SERCOS:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_SERCOS;
         s16_cdhd_opmode = 8;
      break;
      case SE_OPMODE_ETHERNET_IP:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_ETHERNET_IP;
         s16_cdhd_opmode = 8;
      break;
      case SE_OPMODE_ETHERCAT:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_ETHERCAT;
         s16_cdhd_opmode = 8;
      break;
      case SE_OPMODE_PT_PR:
         BGVAR(u16_LXM28_Opmode) = SE_OPMODE_PT;
         s16_cdhd_opmode = 4;
      break;
      default:
         BGVAR(u16_LXM28_Opmode) = 0;
         s16_doOpmode = 0;
      break;
   }

   // if powered from RJ45 (in box programming), then do not activate CAN (force COMMODE=0) because it disturbes the serial communication.
   if (GpioDataRegs.GPBDAT.bit.GPIO60 == 0)
   {
      drive = 0;
      BGVAR(u8_Comm_Mode) = 0;
      drive = 1;
      BGVAR(u8_Comm_Mode) = 0;
      drive = drive_backup;
   }

   // if opmode was set to a valid value.
   if (s16_doOpmode)
   {
      // IPR 1034: init canopen opmode (0x6061) to same value as serial opmode var after factory restore
      if (s16_use_canopen_opmode_change)
      {
         BGVAR(s16_CAN_Opmode_Temp) = 1;  // profile position mode
         BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
         SetCanOpmode(drive);

         p402_modes_of_operation = 0;
         //IPR1631: Don't clear p402_modes_of_operation_display so the user
         //         will know what is the internal opmode
         //p402_modes_of_operation_display = 0;
         //BGVAR(s16_CAN_Opmode_Temp) = 0;
         //BGVAR(s16_CAN_Opmode) = 0;
      }
      else
      {
         SetOpmode(drive, s16_cdhd_opmode);
      }
   }
}


//**********************************************************
// Function Name: SchneiderInit
// Description: this function includes all the Schneider P parameters init actions
//              this function is called after eeprom is loaded or after default values are set (if eeprom load fails).
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void SchneiderInit(int drive)
{
   int s16_i;
   //long long ptr_temp;

   // AXIS_OFF;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   if (!IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK))
   {
      // turn CAN Leds OFF
      CANLed(0);
   }

   STORE_EXECUTION_PARAMS_0_15

   // init values for s16_Pos_Tune_KnlAfrc_Mode and s16_Pos_Tune_NlPeAff_Mode since they are marked as "not in dump" in the excel
   // and therefore not included in SE_DEFAULT_OPR_VALUES macro.
   BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 0;
   BGVAR(s16_Pos_Tune_NlPeAff_Mode) = 1;

   // fix default value for P9-31 (PTACCDEC), since value is not saved in dump and should be set by user
   //SalPosTuneTrajAccDecCommandViaPParam(0x017701770, drive);

   ConvertToPUURecalculation(drive); // need to be called before next line because it sets the units conversion
   //SalReadExcessiveDeviationConditionCommand(&ptr_temp, drive);
   //BGVAR(u32_P2_35_PDEV) = (unsigned long)ptr_temp;

   //First thing before init Schneider parameters- recalculate PUU units for schneider
   SchneiderCalcPuuUnits(drive);

   //Update Schnider product info and sw version by hardware:
   SetProductCode();

   // for Schneider drive P5-16 (PFBOFFSET) is not saved in flash and it's default 0 after every powerup.
   SalPfbOffCommand(0LL, drive); // set PFBOffset to Zero, cannot assign Default

// rewrite the same value to save the currrent SON input state
   //SalWriteAutoEnableAutoLimitCommand(BGVAR(u16_P2_68_Auto_Enable_Actual),drive);
   GetActualDigitalIOStateIntoSchneiderIOsParams(drive);// get the current IO's configuration into Schneider format from CDHD inmodes
   //SalWriteExtPulseInputTypeCommand((long long)BGVAR(u16_P1_00_Ext_Pulse_Input_Type) ,drive);
   SalGearActiveCommand((long long)BGVAR(AX0_u8_Gear_Active), drive);  // set gear to engaged by default
// move to fail load-> SalWriteStopModeCommand(s16_P1_32_Stop_Mode[s16_i], s16_i); // set stop mode for schnieder

   // set the POSCONTROLMODE in Schneider drive mode to "non linear"
   SalPoscontrolModeCommand((long long)VAR(AX0_u16_Pos_Control_Mode), drive);

   // set covel control mode for Schneider default
   SalCompModeCommand((long long)BGVAR(u8_CompMode), drive);

   // Set VCM value in the CDHD way (update s32_Velocity_Scale_In)
   //SalWriteVCMCommand((long long)BGVAR(s32_P1_40_VCM),drive);

   // Set TCM value in the CDHD way (update s32_Current_Scale_In)
   //SalWriteTCMCommand((long long)BGVAR(u16_P1_41_TCM),drive);

   // set zspd (zero speed)
   //SalWriteZeroSpeedWindowCommand((long long)BGVAR(s32_P1_38_Zero_Speed_Window_Out_Loop), drive);
   //SalWriteSpeedReachedOutputRangeCommand((long long)BGVAR(s32_P1_47_SPOK_Out_Loop), drive);

   for (s16_i = 0; s16_i < 3; s16_i++)
   {
       s64_Execution_Parameter[0] = s16_i;
       s64_Execution_Parameter[1] = BGVAR(s32_P1_09to11_Spd_Cmd_Or_Limit)[s16_i];
       //SalWriteSpeedCmdLimTableValueCommand(drive);

       s64_Execution_Parameter[1] = BGVAR(s16_P1_12_Trq_Cmd_Lim)[s16_i];
       //SalWriteTorqueCmdLimTableValueCommand(drive);
   }

   //SalWriteBounceFilterCommand((long long)BGVAR(u16_P2_09_Bounce_Filter), drive);
   //SalWriteFastBounceFilterCommand((long long)BGVAR(u16_P2_24_Fast_Bounce_Filter), drive);
   UpdatePulseDirectionFiltersFromPTTParameter(drive); // update the pulse and direction filters according to P1-00 nibble "B"
   //SalIMaxHaltCommand((long long)BGVAR(s32_P1_70_IMAXHALT), drive);
   //SalWritePulseOutputPolaritySettingCommand((long long)BGVAR(u16_P1_03_AOUT), drive);
   //SalWriteStopModeCommand((long long)BGVAR(s16_P1_32_Stop_Mode), drive); // set stop mode for schnieder
   //SalWriteVCMCommand((long long)BGVAR(s32_P1_40_VCM), drive);
   //SalWriteTCMCommand((long long)BGVAR(u16_P1_41_TCM), drive);
   //SalWriteCRSHACommand((long long)BGVAR(s16_P1_57_CRSHA), drive);

   // P2-08 is set to zero on drive startup.
   s64_Execution_Parameter[0] = 0;
   //SalPCTLparamWriteCommand(drive);
   //SalWriteSpecialFunction1Command((long long)BGVAR(u16_P2_65_GBIT), drive);
   //SalWriteHomingTypeCommand((long long)BGVAR(u16_P5_04_Homing_Mode), drive);
   //SalWriteDriveStatusCommand((long long)BGVAR(s16_P0_02_Drive_Status), drive);

   // Init the Lexium analog output variables
   //SalWriteAnalogMonitorOutputCommand((long long)BGVAR(u16_Analog_Output_Monitor), drive);
   // Init the Lexium analog output 1 proportion
   //SalWriteAnalogMonitorOutputProportion1Command((long long)BGVAR(u16_Analog_Output1_Proportion), drive);
   // Init the Lexium analog output 1 proportion
   //SalWriteAnalogMonitorOutputProportion2Command((long long)BGVAR(u16_Analog_Output2_Proportion), drive);

   SalHomeSpeed1Command((long long)BGVAR(u32_Home_Switch_Speed), drive);
   SalHomeSpeed2Command((long long)BGVAR(u32_Home_Zero_Speed), drive);

   // Use Path 0 parameters for Homing...
   BGVAR(u64_HomeAccRate) = BGVAR(u64_Path_Acceleration)[0];
   BGVAR(u64_HomeDecRate) = BGVAR(u64_Path_Deceleration)[0];

   // HOMEOFSTMOVE must be zero for lxm28 always (has no p param)
//   BGVAR(u16_Home_Ofst_Move) = 0;

   // touch probe for schnieder is set for position capture only.
   VAR(AX0_u16_TProbe_Src) = 1;

   // update canopen gearin and gearout variables according to the actual value
   // take absolute value of AX0_s32_Gearin_Design since it can be negative due to polarity of gear (nibble C in P1-00)
   BGVAR(s32_Fb_Gear_In)  = labs(LVAR(AX0_s32_Gearin_Design));
   BGVAR(s32_Fb_Gear_Out) = LVAR(AX0_u32_Gearout_Design);

   // set event pdo mask (array of 4 p params: P3-18 to P3-21).
   for (s16_i = 0; s16_i < 4; s16_i++)
   {
       s16_Number_Of_Parameters = 2;
       s64_Execution_Parameter[0] = s16_i;
       s64_Execution_Parameter[1] = BGVAR(u16_P3_18_Pdo_Event_Mask)[s16_i];
       //SalWritePdoEventMaskCommand(drive);
   }

   // set internal variable for SW limit according to schnieder values
   //SalWritePUUPosLim(BGVAR(s32_PUU_SW_Pos_Limit), drive);
   //SalWritePUUNegLim(BGVAR(s32_PUU_SW_Neg_Limit), drive);

   // update the internal profile internal parameters according to the last saved user units.
   //SalWriteInternalProfileAccDec((long long) BGVAR(u32_IntProfAccDec_UserUnits),drive);
   //SalWriteInternalProfileVelocity((long long) BGVAR(u16_IntProfVel_UserUnits),drive);
   RecalculateSchneiderInternalProfilePositionToGearRatio(drive);

   // update hd tune vcruise (P9-29) to 300rpm for each path (default value for every startup).
   //SalPosTuneTrajSpdCommandViaPParam(0x0BB80BB8, drive);

   //SalWriteSpecialFunction2Command((long long)BGVAR(u16_P2_66_GBIT2), drive);

   // update poslimmode for lxm28
   VAR(AX0_u16_SW_Pos_lim) = LxmPosLimModeConvert(BGVAR(u16_P5_13_SW_Pos_lim));

   // "0" - update the LS locations from all inputs
   update_LS_Location(drive,0);

   // Update the ZSPD filter Design
   //SalZSPDLpfHzCommand(BGVAR(u16_ZSPD_LPF_Hz),drive);

   SchneiderInitCan(drive);

   RESTORE_EXECUTION_PARAMS_0_15
}


//**********************************************************
// Function Name: SchneiderInitAfterFlashLoadFailed
// Description: this function inits schneider parameters in case of flash load fail
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void SchneiderInitAfterFlashLoadFailed(int drive)
{// called from "ResetAllParams" function
   int drive_backup = drive;
   int s16_dummy;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // Default value of UVTHRESH.
   // Do it here and not from EEPROM because here it is multiplied by sqrt(2).
   //SalWriteLowVoltageAlaramDetectionLevel(160LL, drive);

   BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[0] = 128;
   BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[1] = 128;
   BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[2] = 128;
   BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[3] = 128;

   // to allow canopen get the correct default value (gear in is in object 0x4FA5sub1).
   SalGearInCommand(BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[0], drive);

   //BGVAR(u16_P1_00_Ext_Pulse_Input_Type) = 2;

   //BGVAR(u16_P2_50_Cclr_Trigger) = 0;

   // Default FEEDBACKTYPE for SE is ServoSense
   SalFdbkCommand((long long)BGVAR(u16_FdbkType), drive);
   // Set default MENCRES value for ServoSense
   SalMotorEncResCommand((long long)BGVAR(u32_User_Motor_Enc_Res), drive);

   // Default value of DISTIME for SE is 0
//   BGVAR(u16_DisTime) = 0;

   // GEAROUT default for schnieder is different than CDHD
   SalGearOutCommand((long long)BGVAR(AX0_u32_Gearout_Design), drive);


   // enable torque slope by default
   //SalSetTorqueSlopeEnableCommand((long long)BGVAR(u16_Icmd_Slope_Use), drive);// Should be "1" in Schneider drive but set to "0" for now

   if (u16_Flash_Type == 1)   // 2 * 64Mb flash, LXM28E
   {
     if (u16_Fw_Upgrade_Factory_Restore != 2)
     {
        BGVAR(u16_P1_01_CTL) = 0x1000 | SE_OPMODE_ETHERCAT; //set drive opmode as EtherCAT default
     }
     else
     {
        // BGVAR(u16_P1_01_CTL) was set before
        BGVAR(u16_P1_01_CTL) |= 0x1000;
     }
     // Dont call CMOMODE Sal function because it's rising power cycle request alarm
     BGVAR(u8_Comm_Mode) = 1;
   }
   else if (IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK))
   {
     // State that it is a fieldbus Drive
     BGVAR(u16_P1_01_CTL) = 0x1000 | SE_OPMODE_CANOPEN; //set drive opmode as CANOpen default
     // Dont call CMOMODE Sal function because it's rising power cycle request alarm
     BGVAR(u8_Comm_Mode) = 1;
   }
   else
   {
     // The Lexium is a I/O drive
     BGVAR(u16_P1_01_CTL) = 0x1000 | SE_OPMODE_PT; //set drive opmode as I/O drive default
     // Dont call CMOMODE Sal function because it's rising power cycle request alarm
     BGVAR(u8_Comm_Mode) = 0;
   }
   u16_Fw_Upgrade_Factory_Restore = 0;

   //SalWriteAccFromMs(30LL, drive);    // set defualt value
   //SalWriteDecFromMs(30LL, drive);    // set defualt value
   //SalWriteDecStopViaPParam(30LL, drive);    // set defualt value for decstop

   ConvertFromPUURecalculation(drive); // need to be called before next line because it sets the units conversion
   //SalWriteExcessiveDeviationConditionCommand(100000LL, drive);

   VLimCommand(53687091LL, drive, &s16_dummy); // for P1-55 set VLIM=6000 [RPM]

   SalHomeSpeed1Command((long long)BGVAR(u32_Home_Switch_Speed), drive);
   SalHomeSpeed2Command((long long)BGVAR(u32_Home_Zero_Speed), drive);
   //SalPERWrite((long long)BGVAR(BGVAR(u32_P1_54_PER_User_Value)),drive); // for P1-54(PER) = 12800

// default SW limit switch values are now set by the Excel file
//   BGVAR(s32_PUU_SW_Neg_Limit) = -2147483647L;
//   BGVAR(s32_PUU_SW_Pos_Limit) = 2147483647L;
   RecalculatePUUSWLim(drive);

   //SalWriteSpeedReachedOutputRangeCommand((long long)(8947.8 * (float)BGVAR(s32_P1_47_SPOK_Out_Loop)),drive); // P1-47 = 10

//   VAR(AX0_u8_Gear_Limits_Mode) = 4; // For Schneider the default value of the GEARLIMITSMODE parameter is 4 (in the CDHD it is 0)

   //Set echo =0 as 485 communication is half-duplex
   SalEchoCommand((long long)BGVAR(u16_Echo),drive);

   // if powered from RJ45 (in box programming), then do not activate CAN (force COMMODE=0) because it disturbes the serial communication.
   if (GpioDataRegs.GPBDAT.bit.GPIO60 == 0)
   {
      drive = 0;
      BGVAR(u8_Comm_Mode) = 0;
      drive = 1;
      BGVAR(u8_Comm_Mode) = 0;
      drive = drive_backup;
   }

   // schneider ask to have default value of 1280000 to canopen object 0x6092sub1 and 1 to 0x6092sub2
   SalScalingPnumerator(1280000LL, drive);
   SalScalingPdenominator(1LL, drive);

   // set Home Acc & DEC for Schneider to 200|200 [ms | ms]
   //SalWriteHomeAccDec(0xC800C8,drive);
}

//**********************************************************
// Function Name: SalPCTLparamWriteCommand
// Description: set the Special Factory Setting (PCTL) value.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPCTLparamWriteCommand(int drive)
{
   int u16_ret_val = SAL_SUCCESS;
   unsigned int temp_PCTL = (unsigned int)s64_Execution_Parameter[0];

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch(temp_PCTL)
   {
     case 0:
       // do nothing, this is default value.
     break;

     case 10:  // reset drive to factory defaults
         BGVAR(u16_Special_Factory_Settings) = 999; //temp_PCTL;
        // BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_FACTORY_RESTORE;
        s16_Factory_Restore_On_Power_Up = 1;
        BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_NV_SAVE;
     break;

     case 11: // Trigger a SAVE, new request by Schneider
        // Set the do nv-save bit
        BGVAR(u16_Special_Factory_Settings) = temp_PCTL;
        BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_NV_SAVE;
     break;

     case 406: // turn on forced output option and reset P4-06 FOT
         BGVAR(u16_ForcedOutputs) = 0;
         BGVAR(u16_Special_Factory_Settings) = temp_PCTL;
         BGVAR(u16_Force_Output_Enable) = 1;

         //FIX IPR#1396: signal to RT that ZSPD output is now forced by P4-07
         // Set bit10 of "AX0_s16_ZSPD_Out_Num_Forced" to "1"
         VAR(AX0_s16_ZSPD_Out_Num_Forced) |= 0x0400;
     break;

     case 400: // turn off forced output option
         BGVAR(u16_Special_Factory_Settings) = temp_PCTL;
         BGVAR(u16_Force_Output_Enable) = 0;

         //FIX IPR#1396: signal to RT that ZSPD output is now not forced by P4-07
         // Reset bit10 of "AX0_s16_ZSPD_Out_Num_Forced" to "0"
         if (((BGVAR(u16_P2_44_Autor_DoModeSet) & 0x1) == 0) || (BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PS))
         {// reset the force bit only if AUTOR outmode is not enabled
            VAR(AX0_s16_ZSPD_Out_Num_Forced) &= ~0x0400;
         }
     break;

     default:
         u16_ret_val = (VALUE_OUT_OF_RANGE);
     break;
   }
   return(u16_ret_val);
}


//**********************************************************
// Function Name: SchneiderInitCan
// Description: set the CAN params for schneider
//              should be called before init of canopen library
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void SchneiderInitCan(int drive)
{
 if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // set max acc and max dec for lxm28 (not lxm26)
   /*if (IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK))
   {
       BGVAR(u64_CAN_Max_Acc) = MAX_ACC_DEC_IN_CANOPEN_UNITS;
       BGVAR(u64_CAN_Max_Dec) = MAX_ACC_DEC_IN_CANOPEN_UNITS;
   }*/

   // set bit rate according to P param
   SetSchneiderCanBitRate(drive);
}


//**********************************************************
// Function Name: SetSchneiderCanBitRate
// Description: set the can bit rate according to P3-01
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void SetSchneiderCanBitRate(int drive)
{
   int s16_cdhd_bitrate;
   int s16_shneider_bitrate;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;


   /* =s= bit rates codes:
   0: 125 kBits / second
   1: 250 kBits / second
   2: 500 kBits / second
   3: Reserved
   4: 1.0 MBits / second

   cdhd bit rates codes:
   1 = 125 kbps
   2 = 250 kbps
   3 = 500 kbps
   4 = 1000 kbps
   */

   // can bit rate is in bits 8-11
   s16_shneider_bitrate = (BGVAR(u16_P3_01_BRT) >> 8) & 0x0F;

   // convert =s= can bit rates to cdhd can bit rates
   switch (s16_shneider_bitrate)
   {
       case 0:
            s16_cdhd_bitrate = 1;
            break;
       case 1:
            s16_cdhd_bitrate = 2;
            break;
       case 2:
            s16_cdhd_bitrate = 3;
            break;
       case 4:
            s16_cdhd_bitrate = 4;
            break;
       default:
            s16_cdhd_bitrate = 3;
            break;
   }

   SalCanBitRate(s16_cdhd_bitrate, drive);
}


//**********************************************************
// Function Name: SchneiderCalcPuuUnits
// Description: update PUU units accroding to new schiender opmode and gear ratio
//              (only for P1-44 (GR1) and P1-45 (GR2) ratio change)
//              function should be called after InitCyclePowerPparams() call
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void SchneiderCalcPuuUnits(int drive)
{
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // update the new gear ratio and PUU unit conversion fix shift depends on "u16_P1_01_CTL_Current"
   ConvertToPUURecalculation(drive);
   ConvertFromPUURecalculation(drive);

   // calculate Schneider PUU positions to new gear ratio
   RculateSchneidrPUUPositionToGearRatio(drive,-1);
   RecalculateExcessiveDeviationConditionToCDHDPositionUnits(drive);

   // calculate the new S/W limit value according to the new gear ratio
   RecalculatePUUSWLim(drive);

   // calculate the new PER (PEINPOS) value according to the new gear ratio
   Recalculate_PER_PUU_Units(drive);

   // calculate the new Pos-Tune position value according to the new gear ratio
   RecalculatePUUPTPos(drive);

   // calculate the new Internal profile position value.
   RecalculateSchneiderInternalProfilePositionToGearRatio(drive);

   // calculate the new AXEN value in [PUU] according to the gear ratio.
   RecalculatePUUAXEN(drive);
}


//**********************************************************
// Function Name: GetCanBaudRate
// Description:
//          This function returns the CAN baud rate according to the given baud rate code
//          Returns 0xffff if no valid code is given
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
unsigned int GetCanBaudRate(int u16_brCode)
{
   //1 = 125Kb per second
   //2 = 250Kb per second
   //3 = 500Kb per second
   //4 = 1000Kb per second
   switch(u16_brCode)
   {
      case 1:
        return 125;
      case 2:
        return 250;
      case 3:
        return 500;
      case 4:
        return 1000;
   }

   return 0xffff;
}


void HandlePrivateLabel()
{
   int ret_value = SAL_NOT_FINISHED;
   char s8_private_label_1, s8_private_label_2;
   unsigned int u16_esi_lo, u16_esi_hi, u16_private_label, u16_table_size;

   //ESI version validation test
   u32_ESI_Eeprom_Read_Version = (unsigned long)SerialFlashReadByte(0x0010001B);
   u32_ESI_Eeprom_Read_Version <<= 8;
   u32_ESI_Eeprom_Read_Version |= (unsigned long)SerialFlashReadByte(0x0010001A);
   u32_ESI_Eeprom_Read_Version <<= 8;
   u32_ESI_Eeprom_Read_Version |= (unsigned long)SerialFlashReadByte(0x00100019);
   u32_ESI_Eeprom_Read_Version <<= 8;
   u32_ESI_Eeprom_Read_Version |= (unsigned long)SerialFlashReadByte(0x00100018);

   if (u32_ESI_Supported_Ver != u32_ESI_Eeprom_Read_Version)//EEPROM is devided into 2 parts - if version was not found in part 1 search part 2
   {
      u16_ESI_ver_mismatch = 1;
   }

   // read vendor ID from ESI
   u16_esi_lo = SerialFlashReadByte(ESI_VENDOR_ID_LO_ADDR);
   u16_esi_hi = SerialFlashReadByte(ESI_VENDOR_ID_HI_ADDR);

   // read private label data from drive eeprom
   s8_private_label_1 = s8_Control_Drive_Model_Number_1[0];
   s8_private_label_2 = s8_Control_Drive_Model_Number_1[1];

   u16_table_size = sizeof(Private_Label_Table) / sizeof(PrivateLabelStruct);

   //analyze Drive private label data
   for (u16_private_label = 0; u16_private_label < u16_table_size; u16_private_label++)
   {
      if ((s8_private_label_1 == Private_Label_Table[u16_private_label].s8_vendor_key1) && (s8_private_label_2 == Private_Label_Table[u16_private_label].s8_vendor_key2))
         break;
   }

   // If first run or first run after ESI file was flashed then the vendor id data in ESI is empty
   if ( ( (u16_esi_lo == 0xFF) || (u16_esi_hi == 0xFF) ) && (u16_private_label < u16_table_size) )
   { // Avoid writing to ESI if Model-Number was not identified in Private-Label Table.
      //Write vendor ID to ESI
      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_VENDOR_ID_HI_ADDR,Private_Label_Table[u16_private_label].u16_vendor_hi);}

      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_VENDOR_ID_LO_ADDR, Private_Label_Table[u16_private_label].u16_vendor_lo);}

      //Write first brand name to ESI
      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR1, Private_Label_Table[u16_private_label].u16_brand_addr1);}

      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR2, Private_Label_Table[u16_private_label].u16_brand_addr2);}

      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR3, Private_Label_Table[u16_private_label].u16_brand_addr3);}

      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR4, Private_Label_Table[u16_private_label].u16_brand_addr4);}

      //Write second brand name to ESI
      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR1, Private_Label_Table[u16_private_label].u16_brand_addr1);}

      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR2, Private_Label_Table[u16_private_label].u16_brand_addr2);}

      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR3, Private_Label_Table[u16_private_label].u16_brand_addr3);}

      ret_value = SAL_NOT_FINISHED;
      while (ret_value == SAL_NOT_FINISHED) {ret_value = SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR4, Private_Label_Table[u16_private_label].u16_brand_addr4);}

      // read vendor ID from ESI again
      u16_esi_lo = SerialFlashReadByte(ESI_VENDOR_ID_LO_ADDR);
      u16_esi_hi = SerialFlashReadByte(ESI_VENDOR_ID_HI_ADDR);
   }

   //if ESI vendor ID does not equal private label data
   if ( (u16_esi_hi != Private_Label_Table[u16_private_label].u16_vendor_hi) ||
        (u16_esi_lo != Private_Label_Table[u16_private_label].u16_vendor_lo)   )
   {
      u16_ESI_vendor_mismatch = 1;      //report vendor mismatch in non maskable fault
   }
}
