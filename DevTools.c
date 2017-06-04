#include "DSP2834x_Device.h"

#include "Err_Hndl.def"
#include "DevTools.def"
#include "FPGA.def"
#include "i2c.def"

#include "Extrn_Asm.var"
#include "Ser_Comm.var"
#include "Drive.var"
#include "ExFbVar.var"
#include "User_Var.var"
#include "FltCntrl.var"


#include "Prototypes.pro"


void SerialFlashSetChipSelect(int level)
{
   int time_out;

   if (level == 0)
   {
      GpioDataRegs.GPADAT.bit.GPIO19 = 0;
      for(time_out = 5; time_out > 0; time_out--);
   }
   else
   {
      GpioDataRegs.GPADAT.bit.GPIO19 = 1;
      for(time_out = 33; time_out > 0; time_out--);   // spec requires 100 ns min for CS deselect time
   }
}


unsigned int SerialFlashSendReceiveByte(unsigned int u16_data)
{
   int time_out;

   u16_data = (u16_data << 8) & 0xff00;                  // place the data at the 8 MSB (bits 15-8)
   SpiaRegs.SPITXBUF = u16_data;                     // Initiate data transmit.

   time_out = 400;
   while ( (SpiaRegs.SPISTS.bit.INT_FLAG == 0) && (time_out-- > 0) ){}   // wait as long as the data has not transmitted/received and placed in SPIRXBUF

   u16_data = SpiaRegs.SPIRXBUF;                     // This also clears the INT_FLAG
   u16_data = u16_data & 0x00ff;
   if (time_out == 0)
      u16_data = 0xffff;
   return(u16_data);
}


void InitSpiaGpio(void)
{
   *(int *)FPGA_DSP_IS_SPI_MASTER_REG_ADD |= 0x0001;   // Set DSP as master of the SPI signals to the serial flash

   EALLOW;

   //  GPIO-16 - PIN FUNCTION = serial flash SPI SIMO A
   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;  // Configure GPIO16 as SPISIMOA
   GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;   // Configure GPIO16 as output

   //  GPIO-17 - PIN FUNCTION = serial flash SPI SOMI A
   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;  // Configure GPIO17 as SPISOMIA
   GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;   // Configure GPIO17 as input

   //  GPIO-18 - PIN FUNCTION = serial flash SPI CLK A
   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;  // Configure GPIO18 as SPICLKA
   GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;   // Configure GPIO18 as output

   //  GPIO-19 - PIN FUNCTION = serial flash CS
   GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;  // Configure GPIO19 as GPIO (Chip Select)
   GpioDataRegs.GPADAT.bit.GPIO19 = 1;   // Set GPIO19 to output high
   GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;   // Configure GPIO19 as output

   EDIS;
}


void ReleaseSpiaGpio(void)
{
   EALLOW;

   //  GPIO-16 - PIN FUNCTION = serial flash SPI SIMO A
   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;       // 1=OUTput,  0=INput

   //  GPIO-17 - PIN FUNCTION = serial flash SPI SOMI A
   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;       // 1=OUTput,  0=INput

   //  GPIO-18 - PIN FUNCTION = serial flash SPI CLK A
   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;       // 1=OUTput,  0=INput

   //  GPIO-19 - PIN FUNCTION = serial flash CS
   GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;       // 1=OUTput,  0=INput

   EDIS;

   *(int *)FPGA_DSP_IS_SPI_MASTER_REG_ADD &= ~0x0001;   // Set FPGA as master of the SPI signals to the serial flash
}


unsigned int SerialFlashReadByte(unsigned long addr)
{
   unsigned int u16_temp_value, u16_read_value;

   InitSpiaGpio();

   SerialFlashSetChipSelect(LOW);
   SerialFlashSendReceiveByte(READ_DATA_BYTE_CODE);
   u16_temp_value = ((Uint16)(addr >> 16)) & 0x00ff;  // isolate and send bits 23-16 of the address
   SerialFlashSendReceiveByte(u16_temp_value);
   u16_temp_value = ((Uint16)(addr >> 8)) & 0x00ff;   // isolate and send bits 15-8 of the address
   SerialFlashSendReceiveByte(u16_temp_value);
   u16_temp_value = ((Uint16)addr) & 0x00ff;          // isolate and send bits 7-0 of the address
   SerialFlashSendReceiveByte(u16_temp_value);
   u16_read_value = SerialFlashSendReceiveByte(0);
   SerialFlashSetChipSelect(HIGH);

   ReleaseSpiaGpio();

   return(u16_read_value);
}


int SerialFlashWriteByte(Uint32 addr, Uint16 data)
{
   Uint16 temp_value, return_value;

   return_value = SAL_NOT_FINISHED;

   switch (u16_Serial_Flash_Write_Byte_State)
   {
      case 0:
         InitSpiaGpio();

         SerialFlashWriteEnable(WRITE_ENABLE);      // enable write and erase

         SerialFlashSetChipSelect(LOW);
         SerialFlashSendReceiveByte(PAGE_PROGRAM_CODE);
         temp_value = ((Uint16)(addr >> 16)) & 0x00ff;  // isolate and send bits 23-16 of the address
         SerialFlashSendReceiveByte(temp_value);
         temp_value = ((Uint16)(addr >> 8)) & 0x00ff;   // isolate and send bits 15-8 of the address
         SerialFlashSendReceiveByte(temp_value);
         temp_value = ((Uint16)addr) & 0x00ff;          // isolate and send bits 7-0 of the address
         SerialFlashSendReceiveByte(temp_value);

         temp_value = data & 0x00ff;                    // isolate and send bits 7-0 of the data
         SerialFlashSendReceiveByte(temp_value);

         SerialFlashSetChipSelect(HIGH);

         s32_Serial_Flash_Write_Byte_Timer = Cntr_1mS;
         u16_Serial_Flash_Write_Byte_State++;
      break;

      case 1:
         if (SerialFlashReadWIP() == 0)                // read WIP (write in process) status bit. 0-completed  1-in process
         {
            return_value = FLASH_STATUS_SUCCESS;
         }
         else if (PassedTimeMS(5500L, s32_Serial_Flash_Erase_Sector_Timer))      // set timeoutof at least 5 seconds
         {
            return_value = STATUS_FLASH_FAIL;
         }

         if (return_value != SAL_NOT_FINISHED)
         {
            SerialFlashWriteEnable(WRITE_DISABLE);     // disable write and erase
            ReleaseSpiaGpio();
            u16_Serial_Flash_Write_Byte_State = 0;
         }
      break;
   }

   return(return_value);
}


int SerialFlashReadWIP(void)
{
   int read_value;

   SerialFlashSetChipSelect(LOW);
   SerialFlashSendReceiveByte(READ_STATUS_REGISTER_CODE);
   read_value = SerialFlashSendReceiveByte(0);
   SerialFlashSetChipSelect(HIGH);

   if (read_value != 0xffff)
      read_value = read_value & WIP_MASK;

   return(read_value);           // isolate the WIP bit
}


void SerialFlashWriteEnable(int action)
{
   SerialFlashSetChipSelect(LOW);

   if (action == WRITE_ENABLE)
      SerialFlashSendReceiveByte(WRITE_ENABLE_CODE);
   else
      SerialFlashSendReceiveByte(WRITE_DISABLE_CODE);

   SerialFlashSetChipSelect(HIGH);
}

//**********************************************************
// Function Name: MicroBlazeReadMemoryByte
// Description:
//          This function is called in response to the MBMEMB command
//          in case that the user provides one argument.
//          This function is used in order to read a 8-bit value from
//          any uBlaze memory address.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int MicroBlazeReadMemoryByte(void)
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Check if the address is within a 32-bit range
   if ((s64_Execution_Parameter[0] < 0) || (s64_Execution_Parameter[0] > 0xFFFFFFFFLL))
   {
      return VALUE_OUT_OF_RANGE ;
   }

   // Set debug value, which is in this case the address
   BGVAR(s64_Ecat_Debug_Value) = s64_Execution_Parameter[0];

   // Set the function code to 5 (read 5-bit from any uBlaze address)
   BGVAR(u16_Ecat_Debug_Function) = 5;

   // Call the EtherCAT debug function
   return (SalEtherCATDebugChannel());
}
//**********************************************************
// Function Name: MicroBlazeWriteMemoryByte
// Description:
//          This function is called in response to the MBMEMB command
//          in case that the user provides two arguments.
//          This function is used in order to write a 8-bit value to
//          any uBlaze memory address.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int MicroBlazeWriteMemoryByte(long long s64_addr,long long s64_data)
{
   int drive = 0;
   long long s64_temp_debug_val;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Check if the address is within a 32-bit range
   if ((s64_addr < 0) || (s64_addr > 0xFFFFFFFFLL))
   {
      return VALUE_OUT_OF_RANGE ;
   }

   // Check if data is just of 8-bit size
   if ((s64_data < 0) || (s64_data > 0x00FF))
   {
      return VALUE_OUT_OF_RANGE ;
   }

   // generate a debug value with lower 32-bit = address and upper 32-bit = data
   s64_temp_debug_val = s64_addr & 0x00000000FFFFFFFFLL;
   s64_temp_debug_val |= (s64_data << 32) & 0xFFFFFFFF00000000LL;

   // Set debug value, which is in this case a mixture of the address and the data
   BGVAR(s64_Ecat_Debug_Value) = s64_temp_debug_val;

   // Set the function code to 8 (write 8-bit to any uBlaze address)
   BGVAR(u16_Ecat_Debug_Function) = 8;

   // Call the EtherCAT debug function
   return (SalEtherCATDebugChannel());
}
//**********************************************************
// Function Name: MicroBlazeReadMemoryWord
// Description:
//          This function is called in response to the MBMEMW command
//          in case that the user provides one argument.
//          This function is used in order to read a 16-bit value from
//          any uBlaze memory address.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int MicroBlazeReadMemoryWord(void)
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Check if the address is within a 32-bit range
   if ((s64_Execution_Parameter[0] < 0) || (s64_Execution_Parameter[0] > 0xFFFFFFFFLL))
   {
      return VALUE_OUT_OF_RANGE ;
   }

   // Set debug value, which is in this case the address
   BGVAR(s64_Ecat_Debug_Value) = s64_Execution_Parameter[0];

   // Set the function code to 6 (read 16-bit from any uBlaze address)
   BGVAR(u16_Ecat_Debug_Function) = 6;

   // Call the EtherCAT debug function
   return (SalEtherCATDebugChannel());
}
//**********************************************************
// Function Name: MicroBlazeWriteMemoryWord
// Description:
//          This function is called in response to the MBMEMW command
//          in case that the user provides two arguments.
//          This function is used in order to write a 16-bit value to
//          any uBlaze memory address.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int MicroBlazeWriteMemoryWord(long long s64_addr,long long s64_data)
{
   int drive = 0;
   long long s64_temp_debug_val;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Check if the address is within a 32-bit range
   if ((s64_addr < 0) || (s64_addr > 0xFFFFFFFFLL))
   {
      return VALUE_OUT_OF_RANGE ;
   }

   // Check if data is just of 16-bit size
   if ((s64_data < 0) || (s64_data > 0xFFFF))
   {
      return VALUE_OUT_OF_RANGE ;
   }

   // generate a debug value with lower 32-bit = address and upper 32-bit = data
   s64_temp_debug_val = s64_addr & 0x00000000FFFFFFFFLL;
   s64_temp_debug_val |= (s64_data << 32) & 0xFFFFFFFF00000000LL;

   // Set debug value, which is in this case a mixture of the address and the data
   BGVAR(s64_Ecat_Debug_Value) = s64_temp_debug_val;

   // Set the function code to 9 (write 16-bit to any uBlaze address)
   BGVAR(u16_Ecat_Debug_Function) = 9;

   // Call the EtherCAT debug function
   return (SalEtherCATDebugChannel());
}
//**********************************************************
// Function Name: MicroBlazeReadMemoryLong
// Description:
//          This function is called in response to the MBMEML command
//          in case that the user provides one argument.
//          This function is used in order to read a 32-bit value from
//          any uBlaze memory address.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int MicroBlazeReadMemoryLong(void)
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Check if the address is within a 32-bit range
   if ((s64_Execution_Parameter[0] < 0) || (s64_Execution_Parameter[0] > 0xFFFFFFFFLL))
   {
      return VALUE_OUT_OF_RANGE ;
   }

   // Set debug value, which is in this case the address
   BGVAR(s64_Ecat_Debug_Value) = s64_Execution_Parameter[0];

   // Set the function code to 7 (read 32-bit from any uBlaze address)
   BGVAR(u16_Ecat_Debug_Function) = 7;

   // Call the EtherCAT debug function
   return (SalEtherCATDebugChannel());
}
//**********************************************************
// Function Name: MicroBlazeWriteMemoryLong
// Description:
//          This function is called in response to the MBMEML command
//          in case that the user provides two arguments.
//          This function is used in order to write a 32-bit value to
//          any uBlaze memory address.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int MicroBlazeWriteMemoryLong(long long s64_addr,long long s64_data)
{
   int drive = 0;
   long long s64_temp_debug_val;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Check if the address is within a 32-bit range
   if ((s64_addr < 0) || (s64_addr > 0xFFFFFFFFLL))
   {
      return VALUE_OUT_OF_RANGE ;
   }

   // Check if data is just of 32-bit size
   if ((s64_data < 0) || (s64_data > 0xFFFFFFFFLL))
   {
      return VALUE_OUT_OF_RANGE ;
   }

   // generate a debug value with lower 32-bit = address and upper 32-bit = data
   s64_temp_debug_val = s64_addr & 0x00000000FFFFFFFFLL;
   s64_temp_debug_val |= (s64_data << 32) & 0xFFFFFFFF00000000LL;

   // Set debug value, which is in this case a mixture of the address and the data
   BGVAR(s64_Ecat_Debug_Value) = s64_temp_debug_val;

   // Set the function code to 10 (write 32-bit to any uBlaze address)
   BGVAR(u16_Ecat_Debug_Function) = 10;

   // Call the EtherCAT debug function
   return (SalEtherCATDebugChannel());
}

//**********************************************************
// Function Name: WriteMessageBufferFilterVariable
// Description:
//          This function is called in response to the ECMSGFLT command
//          write access. This function sets the variable in the uBlaze,
//          that is used as an SDO/Mailbox filter.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int WriteMessageBufferFilterVariable(void)
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 0LL) || (s64_Execution_Parameter[0] > 65535LL))
   {
      return SYNTAX_ERROR ;
   }

   // Set filter value
   BGVAR(s64_Ecat_Debug_Value) = s64_Execution_Parameter[0];

   // Set the function code to 3 (set Mailbox capture filter variable)
   BGVAR(u16_Ecat_Debug_Function) = 3;

   // Call the EtherCAT debug function
   return (SalEtherCATDebugChannel());
}

//**********************************************************
// Function Name: ReadMessageBufferFilterVariable
// Description:
//          This function is called in response to the ECMSGFLT command
//          read access. This function reads the variable in the uBlaze, that
//          is used as an SDO/Mailbox filter.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ReadMessageBufferFilterVariable(void)
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Set the function code to 4 (read Mailbox capture filter variable)
   BGVAR(u16_Ecat_Debug_Function) = 4;

   // Call the EtherCAT debug function
   return (SalEtherCATDebugChannel());
}

//**********************************************************
// Function Name: PrintOrClearEcatMessageBuffer
// Description:
//          This function is called in response to the ECMSGBUF command
//          in case that the user provides one argument to this command.
//          This function is used in order to either clear the message
//          buffer (argument = 0) or to print the whole message buffer
//          (argument = 1).
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int PrintOrClearEcatMessageBuffer(void)
{
   static int s16_state_machine = 0;
   int s16_return_value = SAL_NOT_FINISHED;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If no EtherCAT Drive or COMMODE unequal 1
   if(!IS_EC_DRIVE_AND_COMMODE_1)
   {
      PrintStringCrLf("No EtherCAT Drive or communication-mode unequal 1.",0);
      return SAL_SUCCESS ;
   }

   if ((s64_Execution_Parameter[0] < 0LL) || (s64_Execution_Parameter[0] > 1LL))
   {
      return SYNTAX_ERROR ;
   }
   else if ((s64_Execution_Parameter[0] == 0LL) && (s16_state_machine == 0))
   {
      s16_state_machine = 1; // Clear the message buffer
   }
   else if ((s64_Execution_Parameter[0] == 1LL) && (s16_state_machine == 0))
   {
      s16_state_machine = 3; // Print all the messages
   }

   if (u8_Output_Buffer_Free_Space < 80)
   {
      return SAL_NOT_FINISHED;
   }

   switch(s16_state_machine)
   {
      case 0: // Idle state, do nothing
      break;

      case (1): // Prepare variables to clear buffer
         BGVAR(s64_Ecat_Debug_Value) = 0LL;
         BGVAR(u16_Ecat_Debug_Function) = 2;
         s16_state_machine = 2;
      break;

      case (2):
         // Call the EtherCAT debug function
         s16_return_value = SalEtherCATDebugChannel();
      break;

      case (3):
         BGVAR(s64_Ecat_Debug_Value) = 1LL;
         BGVAR(u16_Ecat_Debug_Function) = 2;
         s16_state_machine = 4;
      break;

      case (4):
         if(BGVAR(s64_Ecat_Debug_Value) & 0x1LL)
         {
            PrintString(" Message ",0);PrintUnsignedInt16((unsigned int)(BGVAR(s64_Ecat_Debug_Value)>>1) + 1);PrintCrLf();PrintCrLf();
         }
         s16_state_machine = 5;
      break;

      case (5):
         // Call the EtherCAT debug function
         s16_return_value = SalEtherCATDebugChannel();

         // If 8 byte have been read
         if(s16_return_value == SAL_SUCCESS)
         {
            s16_return_value = SAL_NOT_FINISHED;
            BGVAR(s64_Ecat_Debug_Value)++;
            s16_state_machine = 6;
         }
      break;

      case (6):
         // If printing is done
         if(BGVAR(s64_Ecat_Debug_Value) > 32LL)
         {
            s16_return_value = SAL_SUCCESS;
         }
         else
         {
            if(BGVAR(s64_Ecat_Debug_Value) & 0x1LL)
            {
               PrintCrLf();PrintStringCrLf("*************",0);PrintCrLf();
            }
            s16_state_machine = 4;
         }
      break;
   }

   // If the function is done
   if(s16_return_value != SAL_NOT_FINISHED)
   {
      s16_state_machine = 0; // Reset state machine
   }

   return (s16_return_value);
}



int SerialFlashWriteBuffer(unsigned long address, unsigned int packet_size, unsigned int *data_buffer)  // address is expected to hold even value
{
   unsigned int temp_value, index, return_value;
   unsigned long time_out, addr, addr_in_page;

   index = 0;
   return_value = FLASH_STATUS_SUCCESS;
   addr = address;
   addr_in_page = addr & 0x00ffL;

   InitSpiaGpio();

   SerialFlashWriteEnable(WRITE_ENABLE);      // enable write and erase

   while ((index < packet_size) && (return_value == FLASH_STATUS_SUCCESS))
   {
      if (addr_in_page >= 0x0100L)      // cross page boundary
      {
         addr_in_page = 0L;
         addr = (addr + 0x0100L) & 0xffffff00;
         SerialFlashWriteEnable(WRITE_ENABLE);      // enable write and erase
      }

      SerialFlashSetChipSelect(LOW);
      SerialFlashSendReceiveByte(PAGE_PROGRAM_CODE);
      temp_value = ((Uint16)(addr >> 16)) & 0x00ff;  // isolate and send bits 23-16 of the address
      SerialFlashSendReceiveByte(temp_value);
      temp_value = ((Uint16)(addr >> 8)) & 0x00ff;   // isolate and send bits 15-8 of the address
      SerialFlashSendReceiveByte(temp_value);
      temp_value = ((Uint16)addr) & 0x00ff;          // isolate and send bits 7-0 of the address
      SerialFlashSendReceiveByte(temp_value);
      while ((addr_in_page < 0x0100L) && (index < packet_size))
      {
         temp_value = (data_buffer[index] >> 8)  & 0x00ff;   // isolate and send bits 15-8 of the data
         SerialFlashSendReceiveByte(temp_value);
         temp_value = data_buffer[index] & 0x00ff;           // isolate and send bits 7-0 of the data
         SerialFlashSendReceiveByte(temp_value);
         addr_in_page += 2;
         index ++;
      }
      SerialFlashSetChipSelect(HIGH);

      time_out = 2000000;      // set timeoutof at least 5 seconds, based on read WIP routine time (16 bits at 6.25 MHz)
      while ( (SerialFlashReadWIP() == 1) && (time_out-- > 0) ){}   // read WIP (write in process) status bit. 0-completed  1-in process
      if ( (time_out > 0) && (SerialFlashReadWIP() == 0) )
         return_value = FLASH_STATUS_SUCCESS;
      else
         return_value = STATUS_FLASH_FAIL;
   }

   SerialFlashWriteEnable(WRITE_DISABLE);     // disable write and erase

   return(return_value);
}

int SerialFlashEraseSector(unsigned long addr)
{
   unsigned int temp_value, return_value;

   return_value = SAL_NOT_FINISHED;

   switch (u16_Serial_Flash_Erase_Sector_State)
   {
      case 0:
         InitSpiaGpio();

         SerialFlashWriteEnable(WRITE_ENABLE);      // enable write and erase

         SerialFlashSetChipSelect(LOW);
         SerialFlashSendReceiveByte(SECTOR_ERASE_CODE);
         temp_value = ((Uint16)(addr >> 16)) & 0x00ff;  // isolate and send bits 23-16 of the address
         SerialFlashSendReceiveByte(temp_value);
         temp_value = ((Uint16)(addr >> 8)) & 0x00ff;   // isolate and send bits 15-8 of the address
         SerialFlashSendReceiveByte(temp_value);
         temp_value = ((Uint16)addr) & 0x00ff;          // isolate and send bits 7-0 of the address
         SerialFlashSendReceiveByte(temp_value);
         SerialFlashSetChipSelect(HIGH);

         s32_Serial_Flash_Erase_Sector_Timer = Cntr_1mS;
         u16_Serial_Flash_Erase_Sector_State++;
      break;

      case 1:
         if (SerialFlashReadWIP() == 0)                // read WIP (write in process) status bit. 0-completed  1-in process
         {
            return_value = FLASH_STATUS_SUCCESS;
         }
         else if (PassedTimeMS(5500L, s32_Serial_Flash_Erase_Sector_Timer))      // set timeoutof at least 5 seconds
         {
            return_value = STATUS_FLASH_FAIL;
         }

         if (return_value != SAL_NOT_FINISHED)
         {
            SerialFlashWriteEnable(WRITE_DISABLE);     // disable write and erase
            ReleaseSpiaGpio();
            u16_Serial_Flash_Erase_Sector_State = 0;
         }
      break;
   }

   return(return_value);
}


//**********************************************************
// Function Name: ReadMemoryWord
// Description:
//          This function is called in response to the MEMW command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int ReadMemoryWord(void)
{
   int* addr;
   unsigned long u32_address;
   long s32_time_capture;

   if ( ( (u16_Serial_Flash == 32)  && (s64_Execution_Parameter[0] >= 0x00400000LL) && (s64_Execution_Parameter[0] <= 0x007fffffLL) )  ||
        ( (u16_Serial_Flash == 64)  && (s64_Execution_Parameter[0] >= 0x00400000LL) && (s64_Execution_Parameter[0] <= 0x00bfffffLL) )  ||
        ( (u16_Serial_Flash == 128) && (s64_Execution_Parameter[0] >= 0x00400000LL) && (s64_Execution_Parameter[0] <= 0x013fffffLL) )       )
   {
      u32_address = (unsigned long)s64_Execution_Parameter[0] - 0x00400000L;
      PrintDecInAsciiHex((unsigned long)SerialFlashReadByte(u32_address),4);
      PrintCrLf();
   }
   else
   {
      u32_address = (unsigned long)s64_Execution_Parameter[0];

      u16_Flash_Page = ((unsigned int)(u32_address >> 28)) & 0x0003;  // isolate bits 28 and 29 that indicate which flash page to access
      u32_Flash_Addr = u32_address & 0xCFFFFFFF;                      // clear bits 28 and 29 from the address

      if ( (u16_Flash_Type == 1) && (u16_Flash_Page > 0) && (u32_Flash_Addr >= 0x00100000LL) && (u32_Flash_Addr < 0x00300000LL) )
      {
         // 2 * 64Mb flashes are mounted, and the address is in the flash range and in the flash page out of the DSP range
         // In this case, perform the read from RAM in RT since it is not possible to execute code from flash while switching the page
         s32_time_capture = Cntr_1mS;
         u16_Flash_Read_Request = 1;
         while ((!PassedTimeMS(2L,s32_time_capture)) && (u16_Flash_Read_Request == 1)){};  // wait for RT to read (RT clears this var upon read)
         PrintDecInAsciiHex((unsigned long)u16_Flash_Read_Value,4);
         PrintCrLf();
      }
      else
      {
         addr = (int*)(long)s64_Execution_Parameter[0];

         PrintDecInAsciiHex((unsigned long)*addr,4);
         PrintCrLf();
      }
   }

   return (SAL_SUCCESS);
}


int SalWriteMemoryWord(long long addr,long long data)
{
   int return_value;

   return_value = SAL_SUCCESS;

   if ( ( (u16_Serial_Flash == 32)  && (addr >= 0x00400000LL) && (addr <= 0x007fffffLL) )  ||
        ( (u16_Serial_Flash == 64)  && (addr >= 0x00400000LL) && (addr <= 0x00bfffffLL) )  ||
        ( (u16_Serial_Flash == 128) && (addr >= 0x00400000LL) && (addr <= 0x013fffffLL) )       )
   {
      if ((data > 255) || (data < 0))
      {
         return_value = VALUE_OUT_OF_RANGE;
      }
      else
      {
         addr = addr - 0x00400000LL;
         return_value = SerialFlashWriteByte((unsigned long)addr, (unsigned int)data);
         if (return_value != SAL_NOT_FINISHED)
         {
            return_value = SAL_SUCCESS;
         }
      }
   }
   else
   {
      EALLOW;
      *(int*)(long)addr = (int)data;
      EDIS;
   }

   return (return_value);
}

//**********************************************************
// Function Name: ReadMemoryLong
// Description:
//          This function is called in response to the MEML command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int ReadMemoryLong(void)
{
   unsigned long* addr;
   addr = (unsigned long*)(long)(s64_Execution_Parameter[0]  & 0xffffffff);

   PrintDecInAsciiHex(*addr,8);
   PrintCrLf();

   return (SAL_SUCCESS);
}

int SalWriteMemoryLong(long long addr,long long data)
{
    EALLOW;
   *(long*)((long)addr ) = (long)data;
   EDIS;
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: ReadMemoryLongLong
// Description:
//          This function is called in response to the MEMQ command.
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int ReadMemoryLongLong(void)
{
   unsigned long long* addr;

   addr =(unsigned long long*)(unsigned long)(s64_Execution_Parameter[0]);

   PrintDecInAsciiHex(*addr,16);

   PrintCrLf();

   return (SAL_SUCCESS);
}

int ReadMemoryFloat(void)
{
   long s32_temp;
   int s16_temp;
   unsigned int u16_temp;
   float f_temp = *(float*)(long)(s64_Execution_Parameter[0]);
   float f_temp2;

   // Scale the value so it can be printed as X.YYY E+/-Z
   f_temp2 = f_temp;
   s16_temp = 0;

   if (f_temp2 < 0.0) f_temp2 = -f_temp2;

   if (f_temp2 != 0.0)
   {
      if (f_temp2 < 0.001)
      {
         while (f_temp2 < 1.0) {s16_temp--; f_temp2 *= 10.0;}
      }
      else if (f_temp > (float)(0x7FFFFFFF / 1000))
      {
         while (f_temp2 > 1.0) {s16_temp++; f_temp2 /= 10.0;}
      }
   }

   if (f_temp < 0.0) f_temp2 = -f_temp2;

   s32_temp = (long)(((float)1000 * f_temp2) + 0.5);
   PrintString(DecimalPoint32ToAscii(s32_temp),0);

   if (s16_temp)
   {
      PrintChar('E');
      PrintSignedInt16(s16_temp);
   }
   PrintChar(SPACE);

   // Display Fix-Shr value as well
   FloatToFixS32Shift16(&s32_temp, &u16_temp, f_temp);
   PrintDecInAsciiHex((unsigned long)s32_temp,8);
   PrintChar('>');
   PrintChar('>');
   PrintDecInAsciiHex((unsigned long)u16_temp,4);
   PrintCrLf();

   return (SAL_SUCCESS);
}

int SalWriteMemoryFloat(void)
{
   float f_temp;

   f_temp = ((float)s64_Execution_Parameter[1])/1000.0;
   if (s16_Number_Of_Parameters == 2)
   {
      if (f_temp > (float)0x7FFFFFFF) return VALUE_TOO_HIGH;
      if (f_temp < (float)((long)0x80000000)) return VALUE_TOO_LOW;

      *(float*)(long)(s64_Execution_Parameter[0]) = f_temp;
   }

   if (s16_Number_Of_Parameters == 3)
   {
      *(float*)(long)(s64_Execution_Parameter[0]) = (float)(s64_Execution_Parameter[1]/1000) / power(2.0,(int)s64_Execution_Parameter[2]);
   }

   return (SAL_SUCCESS);
}

int SalWriteMemoryLongLong(void)
{
   unsigned long long *addr = (unsigned long long *)(unsigned long)(s64_Execution_Parameter[0]);
   EALLOW;
   *addr = (unsigned long long)(s64_Execution_Parameter[1]);
   EDIS;
   return (SAL_SUCCESS);
}

// Read a single bit from a 16bit address
int ReadMemoryBit(void)
{
   int *addr;
   unsigned int u16_mask, u16_bit_value = 0;

   if (s64_Execution_Parameter[1] > 15LL) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[1] < 0LL)  return (VALUE_TOO_LOW);

   addr = (int*)(long)s64_Execution_Parameter[0];
   u16_mask = 1<<(int)s64_Execution_Parameter[1];

   if ((*addr) & u16_mask) u16_bit_value = 1;

   PrintUnsignedInteger(u16_bit_value);
   PrintCrLf();

   return (SAL_SUCCESS);
}

// Set a single bit on a 16bit address
int SalWriteMemoryBit(void)
{
   int *addr;
   unsigned int u16_mask;

   if (s64_Execution_Parameter[1] > 15LL) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[1] < 0LL)  return (VALUE_TOO_LOW);
   if (s64_Execution_Parameter[2] > 1LL)  return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[2] < 0LL)  return (VALUE_TOO_LOW);

   addr = (int*)(long)s64_Execution_Parameter[0];
   u16_mask = 1<<(int)s64_Execution_Parameter[1];

   if (s64_Execution_Parameter[2] == 1LL)
      *addr |= u16_mask;
   else
      *addr &= ~u16_mask;

   return (SAL_SUCCESS);
}


#pragma CODE_SECTION(SetFlashPage, "ram346_1");
void SetFlashPage(unsigned int u16_flash_page)
{
   switch (u16_flash_page)
   {
      case 0:     // lower 32Mb of first flash
         GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;    // FLASH_Bank_paging = 0
         GpioDataRegs.GPBSET.bit.GPIO34 = 1;      // FLASH_Select = 1
      break;

      case 1:     // upper 32Mb of first flash
         GpioDataRegs.GPBSET.bit.GPIO50 = 1;      // FLASH_Bank_paging = 1
         GpioDataRegs.GPBSET.bit.GPIO34 = 1;      // FLASH_Select = 1
      break;

      case 2:     // lower 32Mb of second flash
         GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;    // FLASH_Bank_paging = 0
         GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;    // FLASH_Select = 0
      break;

      case 3:     // upper 32Mb of second flash
         GpioDataRegs.GPBSET.bit.GPIO50 = 1;      // FLASH_Bank_paging = 1
         GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;    // FLASH_Select = 0
      break;

      default:
      break;
   }
}


