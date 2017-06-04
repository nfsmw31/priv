#include "DSP2834x_Device.h"

#include "I2C.def"
#include "FPGA.def"
#include "Err_Hndl.def"
#include "Design.def"

#include "I2C.var"
#include "Extrn_Asm.var"
#include "Ser_Comm.var"
#include "Drive.var"
#include "Init.var"
#include "FltCntrl.var"

#include "Prototypes.pro"


/******************************************************************************
* Function Name:        ReadFromI2CDevice
* Description:          Read the data from the specified I2C device
* Input Parameters:     U16 - Device address
*                       U16 - Number of bytes to read
* Output Parameters:    U16 - Read value
* Return Value:         U8 - status:
*                       - Not finished
*                       - Finished OK
*                       - Finished with faults
*******************************************************************************/
unsigned int ReadFromI2CDevice(unsigned int u16_device_address, unsigned int u16_num_of_bytes_to_read, unsigned int *u16_device_read_value)
{
   unsigned int u16_return_value = NOT_FINISHED;

   switch (u16_Read_I2C_Device_State)
   {
      case INITIATE_DEVICE_READ:
         I2caRegs.I2CMDR.all = 0x0020;                // Enable I2C module (set IRS bit)
         I2caRegs.I2CSAR = u16_device_address;        // Specify the destination address of the I2C device.
         I2caRegs.I2CCNT = u16_num_of_bytes_to_read;
         I2caRegs.I2CMDR.all = 0x2C20;         // Set the follwing bits: STT(bit 13), STP(bit 11), MST(bit 10), IRS(bit 5). This kicks off the master transfer.

         s32_Read_I2C_Device_Timer = Cntr_1mS;  // arm timeout

         u16_Read_I2C_Device_State = WAIT_FOR_DEVICE_READ;  // advance state machine
      break;

      case WAIT_FOR_DEVICE_READ:
         if (I2caRegs.I2CSTR.bit.NACK == 1)         // No Ack from a missing or faulty device
         {
            if (I2caRegs.I2CISRC.bit.INTCODE == 0x02) { }; // Read Interrupt Code to reset Interrupt
            u16_return_value = FINISHED_ERROR;

            I2caRegs.I2CMDR.all = 0x0000;  // Reset/disable I2C the module (clear IRS bit)
            u16_Read_I2C_Device_State = INITIATE_DEVICE_READ; // advance state machine

            u16_Read_I2C_Device_Nack_Fault_Counter++;
         }
         else if (PassedTimeMS(100L,s32_Read_I2C_Device_Timer))          // Timeout
         {
            u16_Read_I2C_Device_Timeout_Fault_Counter++;
            s16_I2C_Clocks_To_Release_Stuck = 16;
            u16_Read_I2C_Device_State = DEVICE_READ_STUCK; // advance state machine
         }
         else
         {
            if (I2caRegs.I2CFFRX.bit.RXFFST == u16_num_of_bytes_to_read)    // wait for the receive FIFO to contain the expected number of data bytes.
            {
               if (u16_num_of_bytes_to_read == 2)
               {
                  *u16_device_read_value  = I2caRegs.I2CDRR << 8;    // read MSByte
                  *u16_device_read_value |= I2caRegs.I2CDRR;         // read LSByte
               }
               else   // consider all the rest as 1
               {
                  *u16_device_read_value = I2caRegs.I2CDRR;          // read Byte
               }

               I2caRegs.I2CMDR.all = 0x0000;             // Reset/disable I2C the module (clear IRS bit)
               u16_Read_I2C_Device_State = INITIATE_DEVICE_READ;   // advance state machine
               u16_return_value = FINISHED_OK;

               if (u16_Read_I2C_Device_Stuck_Release_Flag == 1)
               {
                  u16_Read_I2C_Device_Stuck_Release_Flag = 0;
                  u16_Read_I2C_Device_Read_After_Stuck_Counter++;
               }
            }
         }
      break;

      case DEVICE_READ_STUCK:
         ReleaseStuckBus();

         u16_Read_I2C_Device_Stuck_Release_Counter++;
         u16_Read_I2C_Device_Stuck_Release_Flag = 1;

         u16_Read_I2C_Device_State = INITIATE_DEVICE_READ; // advance state machine
         u16_return_value = FINISHED_ERROR;
      break;
   }

   return (u16_return_value);
}


void ReleaseStuckBus()
{
// Try to release stuck bus. This is an empiric solution.
// If the data signal is low, issue 16 clocks and then reset the I2C module.
// If the data gignal is high, return to the normal state machine.
//
// Nov 10, 2014 - following tests by Mark, issue 16 clocks and then reset the I2C module regardless the data signal level.
//

   unsigned long u32_timer;

   while (s16_I2C_Clocks_To_Release_Stuck > 0)
   {
      s16_I2C_Clocks_To_Release_Stuck--;

      GpioDataRegs.GPBDAT.bit.GPIO33 = 0;
      EALLOW;
      GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;    // output - active low
      GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;
      EDIS;

      u32_timer = CpuTimer0Regs.TIM.all;                      // capture Timer0 value
      while ((u32_timer - CpuTimer0Regs.TIM.all) < 1500)  {}  // 1,500 = 5 usec

      EALLOW;
      GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;    // output - High Z with pull up
      EDIS;

      u32_timer = CpuTimer0Regs.TIM.all;                      // capture Timer0 value
      while ((u32_timer - CpuTimer0Regs.TIM.all) < 1500)  {}  // 1,500 = 5 usec
   }
   s16_I2C_Clocks_To_Release_Stuck = 16;

   EALLOW;
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;
   EDIS;

   u32_timer = CpuTimer0Regs.TIM.all;                      // capture Timer0 value
   while ((u32_timer - CpuTimer0Regs.TIM.all) < 1500)  {}  // 1,500 = 5 usec

   if (I2caRegs.I2CISRC.bit.INTCODE == 0x02) { }; // Read Interrupt Code to reset Interrupt
   I2caRegs.I2CMDR.all = 0x0000;  // Reset/disable I2C the module (clear IRS bit)
}


/******************************************************************************
* Function Name:        WriteToI2CDevice
* Description:          Write data to the specified I2C device
* Input Parameters:     U16 - Device address
*                       U16 - Number of bytes to write
*                       U16 - Data value
* Output Parameters:    None
* Return Value:         U8 - status:
*                       - Not finished
*                       - Finished OK
*                       - Finished with faults
*******************************************************************************/
unsigned int WriteToI2CDevice(unsigned int u16_device_address, unsigned int u16_num_of_bytes_to_write, unsigned int u16_device_write_value)
{
   unsigned int u16_return_value = NOT_FINISHED;

   switch (u16_Write_I2C_Device_State)
   {
      case INITIATE_DEVICE_WRITE:
         I2caRegs.I2CMDR.all = 0x0020;                // Enable I2C module (set IRS bit)
         I2caRegs.I2CSAR = u16_device_address;        // Specify the destination address of the I2C device.
         I2caRegs.I2CCNT = u16_num_of_bytes_to_write;

         if (u16_num_of_bytes_to_write == 2)
         {
            I2caRegs.I2CDXR = (u16_device_write_value >> 8) & 0x00ff;      // Write MSByte
            I2caRegs.I2CDXR = u16_device_write_value & 0x00ff;             // Write LSByte
         }
         else   // consider all the rest as 1
         {
            I2caRegs.I2CDXR = u16_device_write_value & 0x00ff;             // Write Byte
         }

         I2caRegs.I2CMDR.all = 0x2E20;         // Set the follwing bits: STT(bit 13), STP(bit 11), MST(bit 10), TRX(bit 9), IRS(bit 5). This kicks off the master transfer.

         u16_Write_I2C_Device_State = WAIT_FOR_DEVICE_WRITE;             // advance state machine
      break;

      case WAIT_FOR_DEVICE_WRITE:
         if (I2caRegs.I2CSTR.bit.NACK == 1) // No Ack from a missing or faulty device
         {
            if (I2caRegs.I2CISRC.bit.INTCODE == 0x02) { }; // Read Interrupt Code to reset Interrupt

            I2caRegs.I2CMDR.all = 0x0000; // Reset/disable I2C module (clear IRS bit)
            u16_Write_I2C_Device_State = INITIATE_DEVICE_WRITE; // advance state machine
            u16_return_value = FINISHED_ERROR;
         }
         else
         {
            if ((I2caRegs.I2CFFTX.bit.TXFFST == 0) && (I2caRegs.I2CSTR.bit.BB == 0))   // wait for the transmit FIFO to be empty, and bus to be free.
            {
               I2caRegs.I2CMDR.all = 0x0000;             // Reset/disable I2C the module (clear IRS bit)
               u16_Write_I2C_Device_State = INITIATE_DEVICE_WRITE;   // advance state machine
               u16_return_value = FINISHED_OK;
            }
         }
      break;
   }

   return (u16_return_value);
}


/******************************************************************************
* Function Name:        ReadByteFromEeprom
* Description:          Read one byte from an Eeprom device
* Input Parameters:     U16 - Eeprom device address
*                       U16 - Address within the Eeprom device
* Output Parameters:    U16 - Read value
* Return Value:         U16 - status:
*                       - Not finished
*                       - Finished OK
*                       - Finished with faults
*******************************************************************************/
unsigned int ReadByteFromEeprom(unsigned int u16_device_address, unsigned int u16_eeprom_address, unsigned int *u16_eeprom_read_value)
{
   unsigned int  u16_return_value = NOT_FINISHED;

   switch (u16_Read_Eeprom_Byte_State)
   {
      case INITIATE_EEPROM_READ_BYTE_WRITE_ADDR:
         I2caRegs.I2CMDR.all = 0x0020;                // Enable I2C module (set IRS bit)
         I2caRegs.I2CSAR = u16_device_address;        // Specify the destination address of the I2C device.
         I2caRegs.I2CCNT = 1;

         I2caRegs.I2CDXR = u16_eeprom_address & 0x00ff;          // Write EEPROM address

         I2caRegs.I2CMDR.all = 0x2620;         // Set the follwing bits: STT(bit 13), MST(bit 10), TRX(bit 9), IRS(bit 5). This kicks off the master transfer.

         u16_Read_Eeprom_Byte_State = INITIATE_EEPROM_READ_BYTE;  // advance state machine

         u32_Read_Eeprom_Byte_Timer = CpuTimer0Regs.TIM.all;          // capture Timer0 value
      break;

      case INITIATE_EEPROM_READ_BYTE:
         if (I2caRegs.I2CSTR.bit.NACK == 1) // No Ack from a missing or faulty device
         {
            if (I2caRegs.I2CISRC.bit.INTCODE == 0x02) { }; // Read Interrupt Code to reset Interrupt

            I2caRegs.I2CMDR.all = 0x0000; // Reset/disable I2C module (clear IRS bit)
            u16_Read_Eeprom_Byte_State = INITIATE_EEPROM_READ_BYTE_WRITE_ADDR;
            // advance state machine
            u16_return_value = FINISHED_ERROR;
         }
         else if ((u32_Read_Eeprom_Byte_Timer - CpuTimer0Regs.TIM.all) > 30000000)     // 30,000,000 = 100 ms
         {
            u16_Read_Eeprom_Byte_Timeout_Fault_Counter++;
            s16_I2C_Clocks_To_Release_Stuck = 16;
            u16_Read_Eeprom_Byte_State = EEPROM_READ_BYTE_STUCK; // advance state machine
         }
         else
         {
            if (I2caRegs.I2CFFTX.bit.TXFFST == 0)   // wait for the transmit FIFO to be empty.
            {
               I2caRegs.I2CMDR.all = 0x2C20;         // Set the follwing bits: STT(bit 13), STP(bit 11), MST(bit 10), IRS(bit 5). This kicks off the master transfer.

               u16_Read_Eeprom_Byte_State = WAIT_FOR_EEPROM_READ_BYTE;      // advance state machine

               u32_Read_Eeprom_Byte_Timer = CpuTimer0Regs.TIM.all;          // capture Timer0 value
            }
         }
      break;

      case WAIT_FOR_EEPROM_READ_BYTE:
         if (I2caRegs.I2CSTR.bit.NACK == 1) // No Ack from a missing or faulty device
         {
            if (I2caRegs.I2CISRC.bit.INTCODE == 0x02) { }; // Read Interrupt Code to reset Interrupt

            I2caRegs.I2CMDR.all = 0x0000; // Reset/disable I2C module (clear IRS bit)
            u16_Read_Eeprom_Byte_State = INITIATE_EEPROM_READ_BYTE_WRITE_ADDR;
            // advance state machine
            u16_return_value = FINISHED_ERROR;

            u16_Read_Eeprom_Byte_Nack_Fault_Counter++;
         }
         else if ((u32_Read_Eeprom_Byte_Timer - CpuTimer0Regs.TIM.all) > 30000000)     // 30,000,000 = 100 ms
         {
            u16_Read_Eeprom_Byte_Timeout_Fault_Counter++;
            s16_I2C_Clocks_To_Release_Stuck = 16;
            u16_Read_Eeprom_Byte_State = EEPROM_READ_BYTE_STUCK; // advance state machine
         }
         else
         {
            if ( I2caRegs.I2CFFRX.bit.RXFFST == 1)    // wait for the receive FIFO to contain the expected number of data bytes.
            {
               *u16_eeprom_read_value = I2caRegs.I2CDRR;          // read Byte
               I2caRegs.I2CMDR.all = 0x0000;             // Reset/disable I2C the module (clear IRS bit)
               u16_Read_Eeprom_Byte_State = INITIATE_EEPROM_READ_BYTE_WRITE_ADDR;   // advance state machine
               u16_return_value = FINISHED_OK;

               if (u16_Read_Eeprom_Byte_Stuck_Release_Flag == 1)
               {
                  u16_Read_Eeprom_Byte_Stuck_Release_Flag = 0;
                  u16_Read_Eeprom_Byte_Read_After_Stuck_Counter++;
               }
            }
         }
      break;

      case EEPROM_READ_BYTE_STUCK:
         ReleaseStuckBus();

         u16_Read_Eeprom_Byte_Stuck_Release_Counter++;
         u16_Read_Eeprom_Byte_Stuck_Release_Flag = 1;

         u16_Read_Eeprom_Byte_State = INITIATE_EEPROM_READ_BYTE_WRITE_ADDR;   // advance state machine
         u16_return_value = FINISHED_ERROR;
      break;
   }

   // Apply retry mechanism
   if (u16_return_value == FINISHED_ERROR)
   {
      if (u16_Read_Eeprom_Byte_Retry_Counter < READ_EEPROM_BYTE_RETRY_NUM)
      {
         u16_return_value = NOT_FINISHED;
         u16_Read_Eeprom_Byte_Retry_Counter++;
         u16_Read_Eeprom_Byte_Retry_Global_Counter++;
      }
      else
      {
         u16_Read_Eeprom_Byte_Retry_Counter = 0;
      }
   }
   else if (u16_return_value == FINISHED_OK)
   {
      u16_Read_Eeprom_Byte_Retry_Counter = 0;
   }

   return (u16_return_value);
}


/******************************************************************************
* Function Name:        WriteByteToEeprom
* Description:          Write one byte to an Eeprom device
* Input Parameters:     U16 - Eeprom device address
*                       U16 - Address within the Eeprom device
*                       U16 - Value to write
* Output Parameters:    None
* Return Value:         U16 - status:
*                       - Not finished
*                       - Finished OK
*                       - Finished with faults
*******************************************************************************/
unsigned int WriteByteToEeprom(unsigned int u16_device_address, unsigned int u16_eeprom_address, unsigned int u16_eeprom_write_byte)
{
   unsigned int u16_return_value = NOT_FINISHED, u16_eeprom_read_back;

   switch (u16_Write_Eeprom_Byte_State)
   {
      case INITIATE_EEPROM_WRITE_BYTE:
         I2caRegs.I2CMDR.all = 0x0020;                // Enable I2C module (set IRS bit)
         I2caRegs.I2CSAR = u16_device_address;        // Specify the destination address of the I2C device.
         I2caRegs.I2CCNT = 2;

         I2caRegs.I2CDXR = u16_eeprom_address & 0x00ff;          // Write EEPROM address
         I2caRegs.I2CDXR = u16_eeprom_write_byte & 0x00ff;       // Write data

         I2caRegs.I2CMDR.all = 0x2E20;         // Set the follwing bits: STT(bit 13), STP(bit 11), MST(bit 10), TRX(bit 9), IRS(bit 5). This kicks off the master transfer.

         u16_Write_Eeprom_Byte_State = WAIT_FOR_EEPROM_WRITE_BYTE;  // advance state machine

         u32_Write_Eeprom_Byte_Timer = CpuTimer0Regs.TIM.all;          // capture Timer0 value
      break;

      case WAIT_FOR_EEPROM_WRITE_BYTE:
         if (I2caRegs.I2CSTR.bit.NACK == 1) // No Ack from a missing or faulty device
         {
            if (I2caRegs.I2CISRC.bit.INTCODE == 0x02) { }; // Read Interrupt Code to reset Interrupt

            I2caRegs.I2CMDR.all = 0x0000; // Reset/disable I2C module (clear IRS bit)
            u16_Write_Eeprom_Byte_State = INITIATE_EEPROM_WRITE_BYTE;
            // advance state machine
            u16_return_value = FINISHED_ERROR;
         }
         else if ((u32_Write_Eeprom_Byte_Timer - CpuTimer0Regs.TIM.all) > 30000000)     // 30,000,000 = 100 ms
         {
            u16_Write_Eeprom_Byte_Timeout_Fault_Counter++;
            s16_I2C_Clocks_To_Release_Stuck = 16;
            u16_Write_Eeprom_Byte_State = EEPROM_WRITE_BYTE_STUCK; // advance state machine
         }
         else
         {
            if ((I2caRegs.I2CFFTX.bit.TXFFST == 0) && (I2caRegs.I2CSTR.bit.BB == 0))   // wait for the transmit FIFO to be empty, and bus to be free.
            {
               I2caRegs.I2CMDR.all = 0x0000;             // Reset/disable I2C the module (clear IRS bit)

               s32_I2C_Timer = Cntr_1mS;                // Arm timeout
               u32_I2C_Init_Counter = u32_Init_Counter;

               u16_Write_Eeprom_Byte_State = WAIT_FOR_EEPROM_WRITE_BYTE_TIMEOUT;   // advance state machine
            }
         }
      break;

      case WAIT_FOR_EEPROM_WRITE_BYTE_TIMEOUT:
         if ( (PassedTimeMS(6L,s32_I2C_Timer))    ||     // wait 6 ms to allow the EEPROM to complete the internal write cycle;
              ((u32_Init_Counter -  u32_I2C_Init_Counter) > 1000) ||     // however, when called from init (RT counter doesn't run) use approximation (measured ~11 ms)
              (ReadByteFromEeprom(u16_device_address, u16_eeprom_address, &u16_eeprom_read_back) == FINISHED_OK) ) // or receive valid response from eeprom.
         {
            u16_Write_Eeprom_Byte_State = EEPROM_WRITE_BYTE_READ_BACK;   // advance state machine
         }
      break;

      case EEPROM_WRITE_BYTE_READ_BACK:
         u16_return_value = ReadByteFromEeprom(u16_device_address, u16_eeprom_address, &u16_eeprom_read_back);

         if (u16_return_value == FINISHED_OK)
         {
            if (u16_eeprom_read_back != u16_eeprom_write_byte)
            {
               // Write operation failed. Try again, up to max number to tries, then fault
               u16_Number_Of_Tries++;
               if (u16_Number_Of_Tries == MAX_NUMBER_OF_TRIES)
               {
                  u16_return_value = FINISHED_ERROR;
               }
               else
               {
                  u16_return_value = NOT_FINISHED;
               }
            }
            u16_Write_Eeprom_Byte_State = INITIATE_EEPROM_WRITE_BYTE;   // advance state machine
         }
      break;

      case EEPROM_WRITE_BYTE_STUCK:
         ReleaseStuckBus();

         u16_Write_Eeprom_Byte_Stuck_Release_Counter++;

         u16_Write_Eeprom_Byte_State = INITIATE_EEPROM_WRITE_BYTE;   // advance state machine
         u16_return_value = FINISHED_ERROR;
      break;
   }

   // Apply retry mechanism
   if (u16_return_value == FINISHED_ERROR)
   {
      if (u16_Write_Eeprom_Byte_Retry_Counter < WRITE_EEPROM_BYTE_RETRY_NUM)
      {
         u16_return_value = NOT_FINISHED;
         u16_Write_Eeprom_Byte_Retry_Counter++;
         u16_Write_Eeprom_Byte_Retry_Global_Counter++;
      }
      else
      {
         u16_Write_Eeprom_Byte_Retry_Counter = 0;
      }

      u16_Write_Eeprom_Byte_State = INITIATE_EEPROM_WRITE_BYTE;   // advance state machine
   }
   else if (u16_return_value == FINISHED_OK)
   {
      u16_Write_Eeprom_Byte_Retry_Counter = 0;
      u16_Write_Eeprom_Byte_State = INITIATE_EEPROM_WRITE_BYTE;   // advance state machine
   }

   return (u16_return_value);
}


/******************************************************************************
* Function Name:        CopyEepromToRamStateMachine
* Description:          State machine that copies the content of an EEPROM to its RAM image
* Input Parameters:     None
* Output Parameters:    None
* Return Value:         None
*******************************************************************************/
void CopyEepromToRamStateMachine(void)
{
   unsigned int  u16_checksum, u16_returned_value, u16_eeprom_read_value, u16_i;
   unsigned int  u16_temp;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( u16_Read_Eeprom_Board != IDLE_BOARD_EEPROM )
   {
      switch (u16_Read_Eeprom_State)
      {
         case READ_EEPROM_IDLE: // Do nothing
         break;

         case READ_TEST_BYTE_0:
            u16_returned_value = ReadByteFromEeprom(u16_Eeprom_Address, TEST_BYTE_0_ADDR, &u16_eeprom_read_value);

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                       // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;   // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               if (u16_eeprom_read_value == BYTE_0_VALUE)
               {
                  u16_Read_Eeprom_State = READ_TEST_BYTE_1;              // Advance the state machine
               }
               else
               {
                  u16_Read_Eeprom_State = READ_TEST_BYTE_0_RETRY;        // Retry once
               }
            }
         break;

         case READ_TEST_BYTE_0_RETRY:
            u16_returned_value = ReadByteFromEeprom(u16_Eeprom_Address, TEST_BYTE_0_ADDR, &u16_eeprom_read_value);

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                       // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;   // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               if (u16_eeprom_read_value == BYTE_0_VALUE)
               {
                  u16_Read_Eeprom_State = READ_TEST_BYTE_1;              // Advance the state machine
               }
               else
               {
                  u16_Number_Of_Tries = 0;
                  RemoveWriteProtectSignal();            // remove write protect signal
                  for (u16_i = 0; u16_i < 1000; u16_i++) // delay after removing Write Protect
                     asm(" NOP ");
                  u16_Read_Eeprom_State = WRITE_TEST_BYTE_0;    // Advance the state machine
               }
            }
         break;

         case WRITE_TEST_BYTE_0:
            u16_returned_value = WriteByteToEeprom(u16_Eeprom_Address,   // device address
                                                   TEST_BYTE_0_ADDR,     // address
                                                   BYTE_0_VALUE);        // data

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                                  // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;              // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               SetWriteProtectSignal();                        // set write protect signal
               u16_Read_Eeprom_State = READ_TEST_BYTE_1;       // Advance the state machine
            }
         break;

         case READ_TEST_BYTE_1:
            u16_returned_value = ReadByteFromEeprom(u16_Eeprom_Address, TEST_BYTE_1_ADDR, &u16_eeprom_read_value);

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                       // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;   // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               if (u16_eeprom_read_value == BYTE_1_VALUE)
               {
                  u16_Read_Eeprom_State = READ_EEPROM_END;     // Advance the state machine
               }
               else
               {
                  u16_Read_Eeprom_State = READ_TEST_BYTE_1_RETRY;        // Retry once
               }
            }
         break;

         case READ_TEST_BYTE_1_RETRY:
            u16_returned_value = ReadByteFromEeprom(u16_Eeprom_Address, TEST_BYTE_1_ADDR, &u16_eeprom_read_value);

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                       // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;   // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               if (u16_eeprom_read_value == BYTE_1_VALUE)
               {
                  u16_Read_Eeprom_State = READ_EEPROM_END;     // Advance the state machine
               }
               else
               {
                  u16_Number_Of_Tries = 0;
                  RemoveWriteProtectSignal();            // remove write protect signal
                  for (u16_i = 0; u16_i < 1000; u16_i++) // delay after removing Write Protect
                     asm(" NOP ");
                  u16_Read_Eeprom_State = WRITE_TEST_BYTE_1;       // Advance the state machine
               }
            }
         break;

         case WRITE_TEST_BYTE_1:
            u16_returned_value = WriteByteToEeprom(u16_Eeprom_Address,   // device address
                                                   TEST_BYTE_1_ADDR,     // address
                                                   BYTE_1_VALUE);        // data

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                                  // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;              // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               SetWriteProtectSignal();                                  // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_END;                  // Advance the state machine
            }
         break;

         case READ_EEPROM_SIZE_0:
            u16_returned_value = ReadByteFromEeprom(u16_Eeprom_Address, SIZE_0_ADDR, &u16_Eeprom_Read_Byte_0);

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                       // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;   // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               u16_Read_Eeprom_State = READ_EEPROM_SIZE_1;    // Advance the state machine
            }
         break;

         case READ_EEPROM_SIZE_1:
            u16_returned_value = ReadByteFromEeprom(u16_Eeprom_Address, SIZE_1_ADDR, &u16_Eeprom_Read_Byte_1);

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                       // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;   // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               u16_Eeprom_Size_Byte = (u16_Eeprom_Read_Byte_1 << 8) | u16_Eeprom_Read_Byte_0;
               if (u16_Eeprom_Size_Byte > 256)
               {
                  u16_Eeprom_Size_Byte = 256;
               }

               if ((u16_Eeprom_Size_Byte & 0x0001) != 0)      // Make sure it is an even value
               {
                  u16_Eeprom_Size_Byte--;
               }

               if ( u16_Read_Eeprom_Board == CONTROL_BOARD_EEPROM )
               {
                  u16_Control_Eeprom_Size_Byte = u16_Eeprom_Size_Byte;
               }
               else if ( u16_Read_Eeprom_Board == POWER1_BOARD_EEPROM )
               {
                  BGVAR(u16_Power_Eeprom_Size_Byte) = u16_Eeprom_Size_Byte;
               }
               else
               {
                  drive = 1;
                  BGVAR(u16_Power_Eeprom_Size_Byte) = u16_Eeprom_Size_Byte;
                  drive = 0;
               }

               u16_Read_Eeprom_Byte_Index = 0;
               u16_Read_Eeprom_Word_Index = 0;
               u16_Read_Eeprom_State = READ_EEPROM_CONTENT_BYTE_0;   // Advance the state machine
            }
         break;

         case READ_EEPROM_CONTENT_BYTE_0:
            u16_returned_value = ReadByteFromEeprom(u16_Eeprom_Address, u16_Read_Eeprom_Byte_Index, &u16_Eeprom_Read_Byte_0);

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                       // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;   // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               u16_Read_Eeprom_Byte_Index++;
               u16_Read_Eeprom_State = READ_EEPROM_CONTENT_BYTE_1;   // Advance the state machine
            }
         break;

         case READ_EEPROM_CONTENT_BYTE_1:
            u16_returned_value = ReadByteFromEeprom(u16_Eeprom_Address, u16_Read_Eeprom_Byte_Index, &u16_Eeprom_Read_Byte_1);

            if (u16_returned_value == FINISHED_ERROR)
            {
               SetWriteProtectSignal();                       // set write protect signal
               u16_Read_Eeprom_State = READ_EEPROM_FAILURE;   // Test failed. Jump to the end of the state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               p_u16_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] = (u16_Eeprom_Read_Byte_1 << 8) | u16_Eeprom_Read_Byte_0;
               u16_Read_Eeprom_Byte_Index++;
               u16_Read_Eeprom_Word_Index++;
               if (u16_Read_Eeprom_Byte_Index >= u16_Eeprom_Size_Byte)
               {
                  u16_Read_Eeprom_State = READ_EEPROM_CHECK_INTEGRITY;   // Advance the state machine
               }
               else
               {
                  u16_Read_Eeprom_State = READ_EEPROM_CONTENT_BYTE_0;    // Advance the state machine
               }
            }
         break;

         case READ_EEPROM_CHECK_INTEGRITY:
            if (u16_Read_Eeprom_Board == CONTROL_BOARD_EEPROM)
            {
               u16_checksum = 0;
               u16_temp = u16_Control_Board_Eeprom_Buffer[PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR] & 0x00ff;
               if (u16_temp != CONTROL_PRODUCTION_STAMP)
               {
                  if ( (u16_temp == u16_Control_Board_Last_Prod_Stamp_Read) || (u16_Control_Board_Prod_Stamp_Retry > GLOBAL_EEPROM_RETRY_NUM) )
                  {
                     // Twice wrong stamp
                     u16_Control_Board_Eeprom_Fault |= PRODUCTION_STAMP_FAULT_MASK;
                  }
                  else
                  {
                     // Capture the read stamp, and retry reading the eeprom
                     u16_Control_Board_Last_Prod_Stamp_Read = u16_temp;
                     u16_Control_Board_Prod_Stamp_Retry++;
                     u16_Eeprom_Read_Retry_Flag = 1;
                  }
               }
               else
               {
                  for (u16_Read_Eeprom_Word_Index = PRODUCTION_START_ADDR; u16_Read_Eeprom_Word_Index <= PRODUCTION_END_ADDR; u16_Read_Eeprom_Word_Index++)
                  {
                     u16_checksum += u16_Control_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] & 0x00ff;
                     u16_checksum += (u16_Control_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] >> 8) & 0x00ff;
                  }

                  if ((u16_checksum & 0x00FF) != 0)
                  {
                     if ( (u16_checksum == u16_Control_Board_Last_Prod_Checksum) || (u16_Control_Board_Prod_Checksum_Retry > GLOBAL_EEPROM_RETRY_NUM) )
                     {
                        // Twice the same wrong checksum
                        u16_Control_Board_Eeprom_Fault |= PRODUCTION_CHECKSUM_FAULT_MASK;
                     }
                     else
                     {
                        // Capture the checksum, and retry reading the eeprom
                        u16_Control_Board_Last_Prod_Checksum = u16_checksum;
                        u16_Control_Board_Prod_Checksum_Retry++;
                        u16_Eeprom_Read_Retry_Flag = 1;
                     }
                  }
               }

               u16_checksum = 0;
               u16_temp = (u16_Control_Board_Eeprom_Buffer[PARAM_STAMP__PARAM_INTEGRITY__ADDR] >> 8) & 0x00ff;
               if (u16_temp != CONTROL_PARAM_STAMP)
               {
                  if ( (u16_temp == u16_Control_Board_Last_Param_Stamp_Read) || (u16_Control_Board_Param_Stamp_Retry > GLOBAL_EEPROM_RETRY_NUM) )
                  {
                     // Twice wrong stamp
                     u16_Control_Board_Eeprom_Fault |= PARAM_STAMP_FAULT_MASK;
                  }
                  else
                  {
                     // Capture the read stamp, and retry reading the eeprom
                     u16_Control_Board_Last_Param_Stamp_Read = u16_temp;
                     u16_Control_Board_Param_Stamp_Retry++;
                     u16_Eeprom_Read_Retry_Flag = 1;
                  }
               }
               else
               {
                  for (u16_Read_Eeprom_Word_Index = PARAM_START_ADDR; u16_Read_Eeprom_Word_Index < u16_Number_Of_Words_To_Write; u16_Read_Eeprom_Word_Index++)
                  {
                     u16_checksum += u16_Control_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] & 0x00ff;
                     u16_checksum += (u16_Control_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] >> 8) & 0x00ff;
                  }

                  if ((u16_checksum & 0x00FF) != 0)
                  {
                     if ( (u16_checksum == u16_Control_Board_Last_Param_Checksum) || (u16_Control_Board_Param_Checksum_Retry > GLOBAL_EEPROM_RETRY_NUM) )
                     {
                        // Twice the same wrong checksum
                        u16_Control_Board_Eeprom_Fault |= PARAM_CHECKSUM_FAULT_MASK;
                     }
                     else
                     {
                        // Capture the checksum, and retry reading the eeprom
                        u16_Control_Board_Last_Param_Checksum = u16_checksum;
                        u16_Control_Board_Param_Checksum_Retry++;
                        u16_Eeprom_Read_Retry_Flag = 1;
                     }
                  }
               }
            }
            else if (u16_Read_Eeprom_Board == POWER1_BOARD_EEPROM)
            {
               drive = 0;
               u16_checksum = 0;
               u16_temp = u16_Power1_Board_Eeprom_Buffer[PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR] & 0x00ff;
               if (u16_temp != POWER_PRODUCTION_STAMP)
               {
                  if ( (u16_temp == BGVAR(u16_Power_Board_Last_Prod_Stamp_Read)) || (BGVAR(u16_Power_Board_Prod_Stamp_Retry) > GLOBAL_EEPROM_RETRY_NUM) )
                  {
                     // Twice wrong stamp
                     BGVAR(u16_Power_Board_Eeprom_Fault) |= PRODUCTION_STAMP_FAULT_MASK;
                  }
                  else
                  {
                     // Capture the read stamp, and retry reading the eeprom
                     BGVAR(u16_Power_Board_Last_Prod_Stamp_Read) = u16_temp;
                     BGVAR(u16_Power_Board_Prod_Stamp_Retry)++;
                     u16_Eeprom_Read_Retry_Flag = 1;
                  }
               }
               else
               {
                  for (u16_Read_Eeprom_Word_Index = PRODUCTION_START_ADDR; u16_Read_Eeprom_Word_Index <= PRODUCTION_END_ADDR; u16_Read_Eeprom_Word_Index++)
                  {
                     u16_checksum += u16_Power1_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] & 0x00ff;
                     u16_checksum += (u16_Power1_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] >> 8) & 0x00ff;
                  }

                  if ((u16_checksum & 0x00FF) != 0)
                  {
                     if ( (u16_checksum == BGVAR(u16_Power_Board_Last_Prod_Checksum)) || (BGVAR(u16_Power_Board_Prod_Checksum_Retry) > GLOBAL_EEPROM_RETRY_NUM) )
                     {
                        // Twice the same wrong checksum
                        BGVAR(u16_Power_Board_Eeprom_Fault) |= PRODUCTION_CHECKSUM_FAULT_MASK;
                     }
                     else
                     {
                        // Capture the checksum, and retry reading the eeprom
                        BGVAR(u16_Power_Board_Last_Prod_Checksum) = u16_checksum;
                        BGVAR(u16_Power_Board_Prod_Checksum_Retry)++;
                        u16_Eeprom_Read_Retry_Flag = 1;
                     }
                  }
               }

               u16_checksum = 0;
               u16_temp = (u16_Power1_Board_Eeprom_Buffer[PARAM_STAMP__PARAM_INTEGRITY__ADDR] >> 8) & 0x00ff;
               if (u16_temp != POWER_PARAM_STAMP)
               {
                  if ( (u16_temp == BGVAR(u16_Power_Board_Last_Param_Stamp_Read)) || (BGVAR(u16_Power_Board_Param_Stamp_Retry) > GLOBAL_EEPROM_RETRY_NUM) )
                  {
                     // Twice wrong stamp
                     BGVAR(u16_Power_Board_Eeprom_Fault) |= PARAM_STAMP_FAULT_MASK;
                  }
                  else
                  {
                     // Capture the read stamp, and retry reading the eeprom
                     BGVAR(u16_Power_Board_Last_Param_Stamp_Read) = u16_temp;
                     BGVAR(u16_Power_Board_Param_Stamp_Retry)++;
                     u16_Eeprom_Read_Retry_Flag = 1;
                  }
               }
               else
               {
                  for (u16_Read_Eeprom_Word_Index = PARAM_START_ADDR; u16_Read_Eeprom_Word_Index < u16_Number_Of_Words_To_Write; u16_Read_Eeprom_Word_Index++)
                  {
                     u16_checksum += u16_Power1_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] & 0x00ff;
                     u16_checksum += (u16_Power1_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] >> 8) & 0x00ff;
                  }

                  if ((u16_checksum & 0x00FF) != 0)
                  {
                     if ( (u16_checksum == BGVAR(u16_Power_Board_Last_Param_Checksum)) || (BGVAR(u16_Power_Board_Param_Checksum_Retry) > GLOBAL_EEPROM_RETRY_NUM) )
                     {
                        // Twice the same wrong checksum
                        BGVAR(u16_Power_Board_Eeprom_Fault) |= PARAM_CHECKSUM_FAULT_MASK;
                     }
                     else
                     {
                        // Capture the checksum, and retry reading the eeprom
                        BGVAR(u16_Power_Board_Last_Param_Checksum) = u16_checksum;
                        BGVAR(u16_Power_Board_Param_Checksum_Retry)++;
                        u16_Eeprom_Read_Retry_Flag = 1;
                     }
                  }
               }
            }
            else if (u16_Read_Eeprom_Board == POWER2_BOARD_EEPROM)
            {
               drive = 1;
               u16_checksum = 0;
               u16_temp = u16_Power2_Board_Eeprom_Buffer[PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR] & 0x00ff;
               if (u16_temp != POWER_PRODUCTION_STAMP)
               {
                  if ( (u16_temp == BGVAR(u16_Power_Board_Last_Prod_Stamp_Read)) || (BGVAR(u16_Power_Board_Prod_Stamp_Retry) > GLOBAL_EEPROM_RETRY_NUM) )
                  {
                     // Twice wrong stamp
                     BGVAR(u16_Power_Board_Eeprom_Fault) |= PRODUCTION_STAMP_FAULT_MASK;
                  }
                  else
                  {
                     // Capture the read stamp, and retry reading the eeprom
                     BGVAR(u16_Power_Board_Last_Prod_Stamp_Read) = u16_temp;
                     BGVAR(u16_Power_Board_Prod_Stamp_Retry)++;
                     u16_Eeprom_Read_Retry_Flag = 1;
                  }
               }
               else
               {
                  for (u16_Read_Eeprom_Word_Index = PRODUCTION_START_ADDR; u16_Read_Eeprom_Word_Index <= PRODUCTION_END_ADDR; u16_Read_Eeprom_Word_Index++)
                  {
                     u16_checksum += u16_Power2_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] & 0x00ff;
                     u16_checksum += (u16_Power2_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] >> 8) & 0x00ff;
                  }

                  if ((u16_checksum & 0x00FF) != 0)
                  {
                     if ( (u16_checksum == BGVAR(u16_Power_Board_Last_Prod_Checksum)) || (BGVAR(u16_Power_Board_Prod_Checksum_Retry) > GLOBAL_EEPROM_RETRY_NUM) )
                     {
                        // Twice the same wrong checksum
                        BGVAR(u16_Power_Board_Eeprom_Fault) |= PRODUCTION_CHECKSUM_FAULT_MASK;
                     }
                     else
                     {
                        // Capture the checksum, and retry reading the eeprom
                        BGVAR(u16_Power_Board_Last_Prod_Checksum) = u16_checksum;
                        BGVAR(u16_Power_Board_Prod_Checksum_Retry)++;
                        u16_Eeprom_Read_Retry_Flag = 1;
                     }
                  }
               }

               u16_checksum = 0;
               u16_temp = (u16_Power2_Board_Eeprom_Buffer[PARAM_STAMP__PARAM_INTEGRITY__ADDR] >> 8) & 0x00ff;
               if (u16_temp != POWER_PARAM_STAMP)
               {
                  if ( (u16_temp == BGVAR(u16_Power_Board_Last_Param_Stamp_Read)) || (BGVAR(u16_Power_Board_Param_Stamp_Retry) > GLOBAL_EEPROM_RETRY_NUM) )
                  {
                     // Twice wrong stamp
                     BGVAR(u16_Power_Board_Eeprom_Fault) |= PARAM_STAMP_FAULT_MASK;
                  }
                  else
                  {
                     // Capture the read stamp, and retry reading the eeprom
                     BGVAR(u16_Power_Board_Last_Param_Stamp_Read) = u16_temp;
                     BGVAR(u16_Power_Board_Param_Stamp_Retry)++;
                     u16_Eeprom_Read_Retry_Flag = 1;
                  }
               }
               else
               {
                  for (u16_Read_Eeprom_Word_Index = PARAM_START_ADDR; u16_Read_Eeprom_Word_Index < u16_Number_Of_Words_To_Write; u16_Read_Eeprom_Word_Index++)
                  {
                     u16_checksum += u16_Power2_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] & 0x00ff;
                     u16_checksum += (u16_Power2_Board_Eeprom_Buffer[u16_Read_Eeprom_Word_Index] >> 8) & 0x00ff;
                  }

                  if ((u16_checksum & 0x00FF) != 0)
                  {
                     if ( (u16_checksum == BGVAR(u16_Power_Board_Last_Param_Checksum)) || (BGVAR(u16_Power_Board_Param_Checksum_Retry) > GLOBAL_EEPROM_RETRY_NUM) )
                     {
                        // Twice the same wrong checksum
                        BGVAR(u16_Power_Board_Eeprom_Fault) |= PARAM_CHECKSUM_FAULT_MASK;
                     }
                     else
                     {
                        // Capture the checksum, and retry reading the eeprom
                        BGVAR(u16_Power_Board_Last_Param_Checksum) = u16_checksum;
                        BGVAR(u16_Power_Board_Param_Checksum_Retry)++;
                        u16_Eeprom_Read_Retry_Flag = 1;
                     }
                  }
               }
               drive = 0;
            }

            // Retry if requested
            if (u16_Eeprom_Read_Retry_Flag == 1)
            {
               u16_Eeprom_Read_Retry_Flag = 0;
               u16_Read_Eeprom_State = READ_EEPROM_SIZE_0;   // Restart the state machine
            }
            else
            {
               u16_Read_Eeprom_State = READ_EEPROM_END;  // Advance the state machine
            }

         break;

         case READ_EEPROM_FAILURE:
            u16_Read_Eeprom_State = READ_EEPROM_END;              // Test failed. Jump to the end of the state machine
            if (u16_Read_Eeprom_Board == CONTROL_BOARD_EEPROM)   // Indicate fault
            {
               u16_Control_Board_Eeprom_Fault |= NO_READ_BACK_FAULT_MASK;
            }
            else if (u16_Read_Eeprom_Board == POWER1_BOARD_EEPROM)
            {
               BGVAR(u16_Power_Board_Eeprom_Fault) |= NO_READ_BACK_FAULT_MASK;
            }
            else if (u16_Read_Eeprom_Board == POWER2_BOARD_EEPROM)
            {
               drive = 1;
               BGVAR(u16_Power_Board_Eeprom_Fault) = BOARD_NOT_EXIST;
               drive = 0;
            }
         break;

         case READ_EEPROM_END: // Do nothing
         break;
      }
   }
}


/******************************************************************************
* Function Name:        CopyEepromToRam
* Description:          Copy the content of one of the Eeprom devices to RAM buffer
* Input Parameters:     U16 - the board from which to read the Eeprom (control, power1, power2)
* Output Parameters:    None
* Return Value:         EStatus
*******************************************************************************/
unsigned int CopyEepromToRam(unsigned int u16_board_eeprom, unsigned int u16_mode)
{
   unsigned int u16_return_value;

   u16_return_value = NOT_FINISHED;

   CopyEepromToRamStateMachine();

   if (u16_Read_Eeprom_Board == IDLE_BOARD_EEPROM)
   {
      // It is possible to start reading the EEPROM only if it is not during previous read or write operation

      // Determine the address of the device according to the board
      if ( u16_board_eeprom == CONTROL_BOARD_EEPROM )
      {
         u16_Read_Eeprom_Board = CONTROL_BOARD_EEPROM;
         p_u16_Eeprom_Buffer = u16_Control_Board_Eeprom_Buffer;
         u16_Eeprom_Address = CONTROL_EEPROM_ADDR;
      }
      else if ( u16_board_eeprom == POWER1_BOARD_EEPROM )
      {
         u16_Read_Eeprom_Board = POWER1_BOARD_EEPROM;
         p_u16_Eeprom_Buffer = u16_Power1_Board_Eeprom_Buffer;
         u16_Eeprom_Address = POWER1_EEPROM_ADDR;
      }

      u16_Number_Of_Tries = 0;
      if (u16_mode == EEPROM_COPY)
      {
         u16_Read_Eeprom_State = READ_EEPROM_SIZE_0;  // Advance the state machine to trigger the copy operation
      }
      else
      {
         u16_Read_Eeprom_State = READ_TEST_BYTE_0;    // Advance the state machine to trigger the test operation
      }
   }
   else if (u16_Read_Eeprom_State == READ_EEPROM_END)
   {
      u16_Read_Eeprom_Board = IDLE_BOARD_EEPROM;
      u16_Read_Eeprom_State = READ_EEPROM_IDLE;  // Reset the state machine
      u16_return_value = FINISHED_OK;
   }

   return (u16_return_value);
}


int SalIOExpanderDeviceCom (int drive)
{
   int s16_destination_register, s16_num_of_bytes, s16_data_to_write, s16_combined_word_to_write;
   unsigned int u16_returned_value, u16_expander_returned_value;
   unsigned long s32_expander_device_timer = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s16_destination_register = (unsigned int)s64_Execution_Parameter[0];
   if ( (s16_destination_register < 0) || (s16_destination_register > 3) )
      return VALUE_OUT_OF_RANGE;

   s16_num_of_bytes         = (unsigned int)s64_Execution_Parameter[1];
   if ( (s16_num_of_bytes < 1) || (s16_num_of_bytes > 2) )
      return VALUE_OUT_OF_RANGE;

   if (3 == s16_Number_Of_Parameters)
   {
      s16_data_to_write     = (unsigned int)s64_Execution_Parameter[2];
      if ( (s16_data_to_write < 0) || (s16_data_to_write > 255) )
        return VALUE_OUT_OF_RANGE;
   }

   BGVAR(u16_Sal_Expander_In_Use) |= 1;
   if (SAL_IO_EXPANDER_USE == u16_Scan_Device_State)
   {
      s32_expander_device_timer = 0;
      u16_Scan_Device_State = SAL_IO_EXPANDER_USE;
      if (2 == s16_Number_Of_Parameters)//read
      {
         switch (u16_Expander_Read_Write_State)
         {
            case 1://WRITE_ADDRESS_TO_COMMAND_REGISTER
               u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, 1, s16_destination_register);
               if (u16_returned_value == FINISHED_ERROR)
               {
                  BGVAR(u16_Sal_Expander_In_Use) &= ~0x0001;
                  return COMMUNICATION_ERROR;
               }
               else if (u16_returned_value == FINISHED_OK)
                  u16_Expander_Read_Write_State = 2;
            break;

            case 2://READ_DATA_FROM_DESIRED_REGISTER
               u16_returned_value = ReadFromI2CDevice(POWER_IO_PORT_ADDR, s16_num_of_bytes, &u16_expander_returned_value);
               if (u16_returned_value == FINISHED_ERROR)
               {
                  u16_Expander_Read_Write_State = 1;
                  BGVAR(u16_Sal_Expander_In_Use) &= ~0x0001;
                  return COMMUNICATION_ERROR;
               }
               else if (u16_returned_value == FINISHED_OK)
               {
                  PrintUnsignedInteger(u16_expander_returned_value);
                  PrintCrLf();
                  u16_Expander_Read_Write_State = 3;
               }
            break;

            case 3://RETURN_REGISTER_0_AS_DEFAULT_READ_ADDRESS (Input) - for the use of ScanI2CDevices()
               u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, 1, 0);
               if (u16_returned_value == FINISHED_ERROR)
               {
                  u16_Expander_Read_Write_State = 1;
                  BGVAR(u16_Sal_Expander_In_Use) &= ~0x0001;
                  return COMMUNICATION_ERROR;
               }
               else if (u16_returned_value == FINISHED_OK)
               {
                  u16_Expander_Read_Write_State = 1;
                  BGVAR(u16_Sal_Expander_In_Use) &= ~0x0001;
                  return SAL_SUCCESS;
               }
            break;
         }
      }
      else if (3 == s16_Number_Of_Parameters)//write
      {
         switch (u16_Expander_Read_Write_State)
         {
            case 1: //CHOOSE_DESIRED_REGISTER_AND_WRITE_DATA
               s16_data_to_write = (unsigned int)s64_Execution_Parameter[2];
               s16_combined_word_to_write = (((s16_destination_register & 0x00ff) << 8) | (s16_data_to_write & 0x00ff));
               u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, s16_num_of_bytes, s16_combined_word_to_write);
               if (u16_returned_value == FINISHED_ERROR)
               {
                  BGVAR(u16_Sal_Expander_In_Use) &= ~0x0001;
                  return COMMUNICATION_ERROR;
               }
               else if (u16_returned_value == FINISHED_OK)
                 u16_Expander_Read_Write_State = 2;
            break;

            case 2://RETURN_REGISTER_0_AS_DEFAULT_READ_ADDRESS (Input) -  for the use of ScanI2CDevices()
               u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, 1, 0);
               if (u16_returned_value == FINISHED_ERROR)
               {
                  u16_Expander_Read_Write_State = 1;
                  BGVAR(u16_Sal_Expander_In_Use) &= ~0x0001;
                  return COMMUNICATION_ERROR;
               }
               else if (u16_returned_value == FINISHED_OK)
               {
                  u16_Expander_Read_Write_State = 1;
                  BGVAR(u16_Sal_Expander_In_Use) &= ~0x0001;
                  return SAL_SUCCESS;
               }
            break;
         }
      }
   }
   else
   {
      if (0 == s32_expander_device_timer) s32_expander_device_timer = Cntr_1mS;  // arm timeout
      if (PassedTimeMS(1000L,s32_expander_device_timer))          // Timeout
      {
         BGVAR(u16_Sal_Expander_In_Use) &= ~0x0001;
         return I2C_BUSY;
      }
   }
   return SAL_NOT_FINISHED;
}


int ConfigIoExpanderUnit (void)
{
   unsigned int u16_returned_value;
   switch (u16_Config_Io_Expander_State)
   {
      case CONFIG_FOURTH_I_O_BIT_AS_OUTPUT:
         u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, 2, 0x03f7); // 0x03 - config register. 0xf7 - bit 3 as output
         if (u16_returned_value == FINISHED_ERROR)
         {
            u16_Config_Io_Expander_State = CONFIG_FOURTH_I_O_BIT_AS_OUTPUT;
            return FINISHED_ERROR;
         }
         else if (u16_returned_value == FINISHED_OK)
           u16_Config_Io_Expander_State = CONFIG_ALL_BIT_INVERT_TO_0;
      break;

      case CONFIG_ALL_BIT_INVERT_TO_0:
         u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, 2, 0x0200); // 0x02 - Polarity register. 0x00 - all Bits not inverted
         if (u16_returned_value == FINISHED_ERROR)
         {
            u16_Config_Io_Expander_State = CONFIG_FOURTH_I_O_BIT_AS_OUTPUT;
            return FINISHED_ERROR;
         }
         else if (u16_returned_value == FINISHED_OK)
           u16_Config_Io_Expander_State = CONFIG_FOURTH_I_O_BIT_VALUE_TO_0;
      break;

      case CONFIG_FOURTH_I_O_BIT_VALUE_TO_0:
         u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, 2, 0x01f7); // 0x01 - output register. 0xf7 - set bit 3 value to 0
         if (u16_returned_value == FINISHED_ERROR)
         {
            u16_Config_Io_Expander_State = CONFIG_FOURTH_I_O_BIT_AS_OUTPUT;
            return FINISHED_ERROR;
         }
         else if (u16_returned_value == FINISHED_OK)
           u16_Config_Io_Expander_State = CONFIG_READ_FROM_INPUT_REGISTER;
      break;

      case CONFIG_READ_FROM_INPUT_REGISTER:
         u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, 1, 0); // 0 - set input register as address for data reads
         if (u16_returned_value == FINISHED_ERROR)
         {
            u16_Config_Io_Expander_State = CONFIG_FOURTH_I_O_BIT_AS_OUTPUT;
            return FINISHED_ERROR;
         }
         else if (u16_returned_value == FINISHED_OK)
         {
           u16_Config_Io_Expander_State = CONFIG_FOURTH_I_O_BIT_AS_OUTPUT;
           u16_Expander_Configured_Flag = 1;
           return FINISHED_OK;
         }
      break;
   }
    return NOT_FINISHED;
}


/******************************************************************************
* Function Name:        ScanI2CDevices
* Description:          Read data from I2C devices
* Input Parameters:     None
* Output Parameters:    None
* Return Value:         None
*******************************************************************************/
void ScanI2CDevices(void)
{
   static unsigned int u16_next_state;
   unsigned int u16_checksum, u16_returned_value, u16_temperature_read_value;
   unsigned int u16_resolver_exc_read_value, u16_power1_io_port_read_value, u16_power1_io_invert_read_value;
   unsigned int u16_no_ac_flt_indication_mask = 0x00F7;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch (u16_Scan_Device_State)
   {
      case READ_CONTROL_TEMPERATURE:
         if ((!IS_DDHD_EC2_DRIVE) || (u16_Axis_Num == 0))
         {
            if (IS_HW_FUNC_ENABLED(TEMP_SENSOR_MASK))
            {
               u16_returned_value = ReadFromI2CDevice(CONTROL_TEMP_SENSOR_ADDR, 2, &u16_temperature_read_value);

               if (u16_returned_value == FINISHED_ERROR)
               {
                  // Read failed. Proceed to the next sensor
                  if (10 > BGVAR(u16_CtrlTempRead_Fail_Wrn_Flag)) BGVAR(u16_CtrlTempRead_Fail_Wrn_Flag)++;
                  u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_1;   // advance state machine
               }
               else if (u16_returned_value == FINISHED_OK)
               {
                  s16_Control_Temperature_Deg_x256 = u16_temperature_read_value;
                  BGVAR(s16_Control_Temperature_Deg) = (s16_Control_Temperature_Deg_x256 + 128) >> 8;
                  u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_1;   // advance state machine
                  u16_I2C_Scan_Flags |= 0x0001; // to indicated that at least one successful read occurred
                  BGVAR(u16_CtrlTempRead_Fail_Wrn_Flag) = 0;
               }
            }
            else
            {
               s16_Control_Temperature_Deg_x256 = 0x8000;         // indicate that this temperature sensor is not available
               BGVAR(s16_Control_Temperature_Deg) = -128;
               u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_1;   // advance state machine
               u16_I2C_Scan_Flags |= 0x0001;                      // to indicated that at least one read occurred (even if not exists)
            }
         }
         else   // DDHD2-EC with u16_Axis_Num == 1
         {
            u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_1;   // advance state machine
            if (u16_FPGA_BG_Buffer_Transfer_Success != 0)
               u16_I2C_Scan_Flags |= 0x0001; // to indicated that at least one successful read occurred
         }
      break;

      case READ_POWER1_IO_PORT_ITER_1:
      case READ_POWER1_IO_PORT_ITER_2:
      case READ_POWER1_IO_PORT_ITER_3:
      case READ_POWER1_IO_PORT_ITER_4:
         // As this state is called several times in a cycle of the I2C, decide which is the next step on the cycle
         if (u16_Scan_Device_State == READ_POWER1_IO_PORT_ITER_2) u16_next_state = WRITE_RESOLVER_EXC_GAIN;
         else if (u16_Scan_Device_State == READ_POWER1_IO_PORT_ITER_3) u16_next_state = EEPROM_STATE_MACHINE;
         else if (u16_Scan_Device_State == READ_POWER1_IO_PORT_ITER_4) u16_next_state = SAL_IO_EXPANDER_USE;
         else u16_next_state = READ_POWER1_TEMPERATURE;

         if (BGVAR(u16_Power_Hw_Features) & 0x0002) // Test support for power I2C IO port (AC phase loss)
         {
            /*In order to detect a state where the I/O expander takes reset due to noise on the bus, the 4th I/O bit is being
            configured as output low. This way, if we detect this bit as high, we know the I/O expander needs a reset-release, and to be re-configured.
            u16_Expander_Configured_Flag is set to 0 on init, and whenever a reset on I/O is detected.
            u16_Expander_Configured_Flag is set back to 1 in ConfigIoExpanderUnit() */

            if (u16_Expander_Configured_Flag == 0)
            {
               u16_returned_value = ConfigIoExpanderUnit();
               //u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, (unsigned int)1, (unsigned int)0);

               if (u16_returned_value == FINISHED_ERROR)
               {  // Write failed. Proceed to the next sensor
                  u16_Scan_Device_State = SET_POWER_IO_PORT_INVERT;   // advance state machine
               }
               else if (u16_returned_value == NOT_FINISHED)
                  break;
            }
            else
            {
               u16_returned_value = ReadFromI2CDevice(POWER_IO_PORT_ADDR, 1, &u16_power1_io_port_read_value);

               if (u16_returned_value == FINISHED_ERROR)
               {  // Read failed. Proceed to the next sensor
                  u16_Scan_Device_State = SET_POWER_IO_PORT_INVERT;   // advance state machine
               }
               else if (u16_returned_value == FINISHED_OK)
               {
                  // If there is a BUS/LOGIC ac-loss indication, retry in order to filter out noises (found in UIC)
                  if (BGVAR(u16_Power_Hw_Features) & 0x0004) u16_no_ac_flt_indication_mask &= ~0x0004; // If 3 phase bus power supply is not supported, ignore the indication of L2-L3 OK
                  if (BGVAR(u16_Line_Loss_Type) == 1) u16_no_ac_flt_indication_mask &= ~0x0002;

                  if (((u16_power1_io_port_read_value & u16_no_ac_flt_indication_mask) != u16_no_ac_flt_indication_mask) && (!u16_I2C_Power1_Retry))
                     u16_I2C_Power1_Retry = 1;
                  else
                  {
                     if ((u16_power1_io_port_read_value & u16_no_ac_flt_indication_mask) == u16_no_ac_flt_indication_mask)
                        u16_I2C_Power1_Retry = 0;

                     // test if I/O Port Upper Bits are properly configured and functioning
                     if ((u16_power1_io_port_read_value & 0xF8) == 0xF0)
                     {
                        u16_Power1_Input_Port_Temp = u16_power1_io_port_read_value;
                        u16_Power1_Input_Port_Read_Counter++;     // Advance the counter to indicate new read
                     }
                     else
                     {
                        u32_IO_Expander_Reset++;
                        u16_Expander_Configured_Flag = 0;   //in case the I/O expander took reset - reset release by re-configure
                     }
                     u16_Scan_Device_State = SET_POWER_IO_PORT_INVERT;   // Advance state machine
                  }
               }
            }
         }
         else
            u16_Scan_Device_State = SET_POWER_IO_PORT_INVERT;   // advance state machine
      break;

      case SET_POWER_IO_PORT_INVERT:
         u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, 1, 2); // 2 - set Polarity register as address for data reads
         if (u16_returned_value == FINISHED_ERROR)
         {  // Write failed. Proceed to the next state
            u16_Scan_Device_State = SET_POWER_IO_PORT_INPUT;   // advance state machine
         }
         else if (u16_returned_value == FINISHED_OK)
            u16_Scan_Device_State = READ_POWER_IO_PORT_INVERT;   // advance state machine
      break;

      case READ_POWER_IO_PORT_INVERT:
         u16_returned_value = ReadFromI2CDevice(POWER_IO_PORT_ADDR, 1, &u16_power1_io_invert_read_value);
         if (u16_returned_value == FINISHED_ERROR)
         {  // Write failed. Proceed to the next state
            u16_Scan_Device_State = SET_POWER_IO_PORT_INPUT;   // advance state machine            
         }
         else if (u16_returned_value == FINISHED_OK)
         {
            if (u16_power1_io_invert_read_value != 0)
            {
               u32_IO_Expander_Invert_Reset++;
               u16_Expander_Configured_Flag = 0;   // if I/O expander is mis-configured
               u16_Scan_Device_State = SET_POWER_IO_PORT_INPUT;   // advance state machine
            }
            else
            {
               u16_Power1_Input_Port = u16_Power1_Input_Port_Temp;
               if (((u16_Power1_Input_Port & 0x00F1) == 0x00F0) && (3 > u16_Logic_Ac_Fall_Ctr))
               {
                  WriteToFaultLog(drive, GetFaultId((unsigned long long)LOGIC_AC_LOSS_MASK, (int)1));
                  WriteFaultLogToNvram(DRIVE_PARAM);
                  u16_Logic_Ac_Fall_Ctr++;
               }
               else
               {
                  u16_Logic_Ac_Fall_Ctr = 0;
                  u16_Scan_Device_State = SET_POWER_IO_PORT_INPUT;
               }
            }           
         }
      break;

      case SET_POWER_IO_PORT_INPUT:
         u16_returned_value = WriteToI2CDevice(POWER_IO_PORT_ADDR, 1, 0); // 0 - set Polarity register as address for data reads
         if (u16_returned_value != NOT_FINISHED)
            u16_Scan_Device_State = u16_next_state;   // advance state machine
      break;

      case READ_POWER1_TEMPERATURE:
         if ((u16_Product != DDHD) || (u16_Axis_Num == 0))
         {
            u16_returned_value = ReadFromI2CDevice(POWER_TEMP_SENSOR_ADDR, 2, &u16_temperature_read_value);

            if (u16_returned_value == FINISHED_ERROR)
            {
               // Read failed. Proceed to the next sensor
               if (10 > BGVAR(u16_PowerTempRead_Fail_Wrn_Flag)) BGVAR(u16_PowerTempRead_Fail_Wrn_Flag)++;
               u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_2;   // advance state machine
            }
            else if (u16_returned_value == FINISHED_OK)
            {
               FilterPowerTemperatureReadout((int)u16_temperature_read_value);  // This functions writes to s16_Power_Temperature_Deg_x256
               BGVAR(s16_Power_Temperature_Deg) = (s16_Power_Temperature_Deg_x256 + 128) >> 8;
               u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_2;   // advance state machine
               u16_I2C_Scan_Flags |= 0x0002; // to indicated that at least one successful read occurred
               BGVAR(u16_PowerTempRead_Fail_Wrn_Flag) = 0;
            }
         }
         else   // DDHD with u16_Axis_Num == 1
         {
            u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_2;   // advance state machine
            if (u16_FPGA_BG_Buffer_Transfer_Success != 0)
               u16_I2C_Scan_Flags |= 0x0002; // to indicated that at least one successful read occurred
         }
      break;

      case WRITE_RESOLVER_EXC_GAIN:
         if (IS_HW_FUNC_ENABLED(RESOLVER_FEEDBACK_MASK))
         {
            if ( (u16_Desired_Resolver_Exc_Gain != u16_Actual_Resolver_Exc_Gain) && (u16_Desired_Resolver_Exc_Gain <= 0x7f) )
            {
               u16_returned_value = WriteToI2CDevice(RESOLVER_EXCITATION_RESISTOR_ADDR, (unsigned int)1, u16_Desired_Resolver_Exc_Gain);

               if (u16_returned_value == FINISHED_ERROR)
               {  // Write failed. Proceed to the next sensor
                  u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_3;   // advance state machine
               }
               else if (u16_returned_value == FINISHED_OK)
                  u16_Scan_Device_State = READ_BACK_RESOLVER_EXC_GAIN; // advance state machine
            }
            else
               u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_3;   // advance state machine
         }
         else
         {
            u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_3;   // advance state machine
         }
      break;

      case READ_BACK_RESOLVER_EXC_GAIN:
         u16_returned_value = ReadFromI2CDevice(RESOLVER_EXCITATION_RESISTOR_ADDR, 1, &u16_resolver_exc_read_value);

         if (u16_returned_value == FINISHED_ERROR)
         {  // Read failed. Proceed to the next sensor
            u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_3;   // advance state machine
         }
         else if (u16_returned_value == FINISHED_OK)
         {
            u16_Actual_Resolver_Exc_Gain = u16_resolver_exc_read_value;
            u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_3;   // advance state machine
         }
      break;

      case EEPROM_STATE_MACHINE:
         if ( u16_Write_Eeprom_Board != IDLE_BOARD_EEPROM )
         {
            switch (u16_Save_Eeprom_State)
            {
               case WRITE_EEPROM_IDLE:
                  // Do nothing
               break;

               case WRITE_PREPARATION:
                  // Calculate key section checksum word and store it in the buffer.
                  u16_checksum = 0;
                  p_u16_Eeprom_Buffer[KEY_INTEGRITY__KEY_INTEGRITY_0__ADDR] = 0;
                  for (u16_Write_Eeprom_Word_Index = KEY_START_ADDR; u16_Write_Eeprom_Word_Index <= KEY_END_ADDR; u16_Write_Eeprom_Word_Index++)
                  {
                     u16_checksum += p_u16_Eeprom_Buffer[u16_Write_Eeprom_Word_Index] & 0x00ff;
                     u16_checksum += (p_u16_Eeprom_Buffer[u16_Write_Eeprom_Word_Index] >> 8) & 0x00ff;
                  }
                  p_u16_Eeprom_Buffer[KEY_INTEGRITY__KEY_INTEGRITY_0__ADDR] |= (-u16_checksum << 8) & 0xff00;   // write the negated value so the sum when checked should be 0

                  // Calculate production section checksum word and store it in the buffer.
                  u16_checksum = 0;
                  p_u16_Eeprom_Buffer[PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR] &= 0x00ff;
                  for (u16_Write_Eeprom_Word_Index = PRODUCTION_START_ADDR; u16_Write_Eeprom_Word_Index <= PRODUCTION_END_ADDR; u16_Write_Eeprom_Word_Index++)
                  {
                     u16_checksum += p_u16_Eeprom_Buffer[u16_Write_Eeprom_Word_Index] & 0x00ff;
                     u16_checksum += (p_u16_Eeprom_Buffer[u16_Write_Eeprom_Word_Index] >> 8) & 0x00ff;
                  }
                  p_u16_Eeprom_Buffer[PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR] |= (-u16_checksum << 8) & 0xff00;   // write the negated value so the sum when checked should be 0

                  // Calculate parameter section checksum word and store it in the buffer.
                  u16_checksum = 0;
                  p_u16_Eeprom_Buffer[PARAM_STAMP__PARAM_INTEGRITY__ADDR] &= 0xff00;
                  for (u16_Write_Eeprom_Word_Index = PARAM_START_ADDR; u16_Write_Eeprom_Word_Index < u16_Number_Of_Words_To_Write; u16_Write_Eeprom_Word_Index++)
                  {
                     u16_checksum += p_u16_Eeprom_Buffer[u16_Write_Eeprom_Word_Index] & 0x00ff;
                     u16_checksum += (p_u16_Eeprom_Buffer[u16_Write_Eeprom_Word_Index] >> 8) & 0x00ff;
                  }
                  p_u16_Eeprom_Buffer[PARAM_STAMP__PARAM_INTEGRITY__ADDR] |= (-u16_checksum) & 0x00ff;   // write the negated value so the sum when checked should be 0

                  // Backup the stamp words, and replace it with 0xff.
                  u16_Backup_Production_Stamp = p_u16_Eeprom_Buffer[PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR];
                  u16_Backup_Param_Stamp = p_u16_Eeprom_Buffer[PARAM_STAMP__PARAM_INTEGRITY__ADDR];
                  p_u16_Eeprom_Buffer[PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR] |= 0x00ff;
                  p_u16_Eeprom_Buffer[PARAM_STAMP__PARAM_INTEGRITY__ADDR] |= 0xff00;

               case WRITE_PREPARATION_CONT:
                  RemoveWriteProtectSignal();              // remove write protect signal

                  u16_Number_Of_Tries = 0;
                  u16_Write_Eeprom_Word_Index = 0;
                  u16_Write_Eeprom_Byte_Index = 0;
                  if (u16_Eeprom_Busy == KEY_AREA_EEPROM)
                     u16_Write_Eeprom_Byte_Index = KEY_INTEGRITY_ADDR;
                  u16_Save_Eeprom_State = WRITE_LS_BYTES;  // Advance the state machine
               break;

               case WRITE_LS_BYTES:
                  u16_Eeprom_Write_Word = p_u16_Eeprom_Buffer[u16_Write_Eeprom_Word_Index];
                  u16_Eeprom_Write_Byte = u16_Eeprom_Write_Word & 0x00ff;

                  u16_returned_value = WriteByteToEeprom(u16_Eeprom_Address,          // device address
                                                         u16_Write_Eeprom_Byte_Index, // address
                                                         u16_Eeprom_Write_Byte);      // data

                  if (u16_returned_value == FINISHED_ERROR)
                  {
                     u16_Save_Eeprom_State_Before_Fault = WRITE_LS_BYTES;
                     u16_Save_Eeprom_State = WRITE_SET_FAULT;  // Write operation failed.
                  }
                  else if (u16_returned_value == FINISHED_OK)
                  {
                     u16_Number_Of_Tries = 0;
                     u16_Save_Eeprom_State = WRITE_MS_BYTES;  // Advance the state machine
                     u16_Write_Eeprom_Byte_Index++;
                  }
               break;

               case WRITE_MS_BYTES:
                  u16_Eeprom_Write_Word = p_u16_Eeprom_Buffer[u16_Write_Eeprom_Word_Index];
                  u16_Eeprom_Write_Byte = (u16_Eeprom_Write_Word >> 8) & 0x00ff;

                  u16_returned_value = WriteByteToEeprom(u16_Eeprom_Address,            // device address
                                                         u16_Write_Eeprom_Byte_Index,   // address
                                                         u16_Eeprom_Write_Byte);        // data

                  if (u16_returned_value == FINISHED_ERROR)
                  {
                     u16_Save_Eeprom_State_Before_Fault = WRITE_MS_BYTES;
                     u16_Save_Eeprom_State = WRITE_SET_FAULT;  // Write operation failed.
                  }
                  else if (u16_returned_value == FINISHED_OK)
                  {
                     u16_Number_Of_Tries = 0;
                     u16_Write_Eeprom_Byte_Index++;
                     u16_Write_Eeprom_Word_Index++;
                     if (u16_Write_Eeprom_Word_Index == u16_Number_Of_Words_To_Write)
                        u16_Save_Eeprom_State = WRITE_PRODUCTION_STAMP;  // Advance the state machine
                     else if ((u16_Eeprom_Busy == KEY_AREA_EEPROM) && (u16_Write_Eeprom_Word_Index == 5))
                        u16_Save_Eeprom_State = WRITE_EEPROM_END;  // Advance the state machine
                     else if ((u16_Eeprom_Busy == CONTROL_BOARD_EEPROM) && (u16_Write_Eeprom_Word_Index == KEY_INTEGRITY__KEY_INTEGRITY_0__ADDR))
                     {
                        u16_Write_Eeprom_Byte_Index = PRODUCTION_STAMP_ADDR;
                        u16_Write_Eeprom_Word_Index = PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR;
                        u16_Save_Eeprom_State = WRITE_LS_BYTES;  // Advance the state machine
                     }
                     else
                        u16_Save_Eeprom_State = WRITE_LS_BYTES;      // Advance the state machine
                  }
               break;

               case WRITE_PRODUCTION_STAMP:
                  u16_Eeprom_Write_Byte = u16_Backup_Production_Stamp & 0x00ff;

                  u16_returned_value = WriteByteToEeprom(u16_Eeprom_Address,                  // device address
                                                         (unsigned int)PRODUCTION_STAMP_ADDR, // address
                                                         u16_Eeprom_Write_Byte);              // data

                  if (u16_returned_value == FINISHED_ERROR)
                  {
                     u16_Save_Eeprom_State_Before_Fault = WRITE_PRODUCTION_STAMP;
                     u16_Save_Eeprom_State = WRITE_SET_FAULT;  // Write operation failed.
                  }
                  else if (u16_returned_value == FINISHED_OK)
                  {
                     // Restore the stamp words to the RAM buffer.
                     p_u16_Eeprom_Buffer[PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR] &= 0xff00;
                     p_u16_Eeprom_Buffer[PRODUCY_INTEGRITY__PRODUCTION_STAMP__ADDR] |= (u16_Backup_Production_Stamp & 0x00ff);

                     u16_Number_Of_Tries = 0;
                     u16_Save_Eeprom_State = WRITE_PARAM_STAMP;  // Advance the state machine
                  }
               break;

               case WRITE_PARAM_STAMP:
                  u16_Eeprom_Write_Byte = (u16_Backup_Param_Stamp >> 8) & 0x00ff;

                  u16_returned_value = WriteByteToEeprom(u16_Eeprom_Address,             // device address
                                                         (unsigned int)PARAM_STAMP_ADDR, // address
                                                         u16_Eeprom_Write_Byte);         // data

                  if (u16_returned_value == FINISHED_ERROR)
                  {
                     u16_Save_Eeprom_State_Before_Fault = WRITE_PARAM_STAMP;
                     u16_Save_Eeprom_State = WRITE_SET_FAULT;  // Write operation failed.
                  }
                  else if (u16_returned_value == FINISHED_OK)
                  {
                     // Restore the stamp words to the RAM buffer.
                     p_u16_Eeprom_Buffer[PARAM_STAMP__PARAM_INTEGRITY__ADDR] &= 0x00ff;
                     p_u16_Eeprom_Buffer[PARAM_STAMP__PARAM_INTEGRITY__ADDR] |= (u16_Backup_Param_Stamp & 0xff00);

                     u16_Number_Of_Tries = 0;
                     u16_Save_Eeprom_State = WRITE_EEPROM_END;  // Advance the state machine
                  }
               break;

               case WRITE_SET_FAULT:
                  if (u16_Write_Eeprom_Board == CONTROL_BOARD_EEPROM)   // Indicate fault
                     u16_Control_Board_Eeprom_Fault |= EEPROM_WRITE_FAULT_MASK;
                  else if (u16_Write_Eeprom_Board == POWER1_BOARD_EEPROM)
                     BGVAR(u16_Power_Board_Eeprom_Fault) |= EEPROM_WRITE_FAULT_MASK;
                  else if (u16_Write_Eeprom_Board == POWER2_BOARD_EEPROM)
                  {
                     drive = 1;
                     BGVAR(u16_Power_Board_Eeprom_Fault) |= EEPROM_WRITE_FAULT_MASK;
                     drive = 0;
                  }
                  u16_Save_Eeprom_State = WRITE_EEPROM_END;  // Jump to the end of the state machine
               break;

               case WRITE_EEPROM_END:
                  SetWriteProtectSignal();                    // set write protect signal
                  u16_Write_Eeprom_Board = IDLE_BOARD_EEPROM;
                  u16_Save_Eeprom_State = WRITE_EEPROM_IDLE;  // Reset the state machine
               break;
            }
         }
         else
            u16_Scan_Device_State = READ_POWER1_IO_PORT_ITER_4;                  // advance state machine
      break;

      case SAL_IO_EXPANDER_USE:
         if (!BGVAR(u16_Sal_Expander_In_Use))
             u16_Scan_Device_State = I2C_IDLE_STATE;
      break;

      case I2C_IDLE_STATE:
         // Idle state to allow jump to resident code not during i2c scan.
         // If the jump occurs during i2c scan, it might lock the i2c bus, and after returning to the
         // drive firmware, it can't access the EEPROM, so it can't read the key, and 'b' fault is
         // generated.
         u16_Scan_Device_State = READ_CONTROL_TEMPERATURE;      // advance state machine
      break;
   }
}


void FilterPowerTemperatureReadout(int s16_temperature_read_value)
{
   int s16_power_temperature_diff;
   long s32_power_temperature_sum;
   unsigned int u16_index;

   // Clac the absolute difference between the newly measured temperaure and the last one, in order to reject temperature
   // readout which are too far from the last temperature.
   if (s16_Power_Temperature_Deg_x256 > s16_temperature_read_value)
      s16_power_temperature_diff = s16_Power_Temperature_Deg_x256 - s16_temperature_read_value;
   else
      s16_power_temperature_diff = s16_temperature_read_value - s16_Power_Temperature_Deg_x256;

   if ( ((u16_I2C_Scan_Flags & 0x0002) == 0)                                ||   // First time, or
        (s16_power_temperature_diff < s16_Power_Temperature_Diff_Threshold) ||   // temperature difference is less than pre-defined threshold (ignore temperature readout if change in one sample more than threshold), or
        (u16_Power_Temperature_Diff_Counter > POWER_TEMPERATURE_BUFFER_SIZE)   ) // Force buffering in case temp diff for too long
   {
      s16_Power_Temperature_Buffer[u16_Power_Temperature_Index] = s16_temperature_read_value;  // Store the new temperature in a buffer
      u16_Power_Temperature_Index++;                                                           // Advance buffer's index
      if (u16_Power_Temperature_Index >= POWER_TEMPERATURE_BUFFER_SIZE)                        // Check if index rollover is needed
      {
         u16_Power_Temperature_Index = 0;
         u16_Power_Temperature_Buffer_Flag = 1;       // Indicate that the buffer was completely filled at least once
      }

      if (u16_Power_Temperature_Buffer_Flag == 1)     // Calc average only if the buffer was completely filled at least once
      {
         s32_power_temperature_sum = POWER_TEMPERATURE_BUFFER_ROUNDING;
         for (u16_index = 0; u16_index < POWER_TEMPERATURE_BUFFER_SIZE; u16_index++)
            s32_power_temperature_sum += (long)s16_Power_Temperature_Buffer[u16_index];
         s16_temperature_read_value = (int)(s32_power_temperature_sum >> POWER_TEMPERATURE_BUFFER_SHR);
      }

      s16_Power_Temperature_Deg_x256 = s16_temperature_read_value;

      if (u16_Power_Temperature_Diff_Counter > 0)
         u16_Power_Temperature_Diff_Counter--;
   }
   else
   {  // Increment counter that indicates this situation (temp diff greater than threshold)
      u16_Power_Temperature_Diff_Counter++;
      u32_Power_Temperature_Diff_Counter++;   // This counter only counts up. It is for debug information.
   }
}


// Read I2C EEPROM memory location
// Parameters:
//    Board: 1 - control, 2 - power1
//    Address: 0 to (EEPROM byte size - 1)
int SalEepromReadCommand(void)
{
   unsigned int u16_board, u16_addr, u16_data, u16_eeprom_size_byte;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (s64_Execution_Parameter[0] > 2LL) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[0] < 1LL)  return (VALUE_TOO_LOW);

   u16_board = (unsigned int)s64_Execution_Parameter[0];
   if (u16_board == 1)
      u16_eeprom_size_byte = u16_Control_Eeprom_Size_Byte;
   else
      u16_eeprom_size_byte = BGVAR(u16_Power_Eeprom_Size_Byte);

   if (s64_Execution_Parameter[1] >= (long long)u16_eeprom_size_byte) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[1] < 0LL)  return (VALUE_TOO_LOW);

   u16_addr = (unsigned int)s64_Execution_Parameter[1];
   u16_data = GetEEpromRightByteFromAddress(u16_board , u16_addr);

   PrintUnsignedInteger(u16_data);
   PrintCrLf();

   return (SAL_SUCCESS);
}


// Write I2C EEPROM memory location
// Parameters:
//    Board: 1 - control, 2 - power1
//    Address: 0 to (EEPROM byte size - 1)
//    Data: 0 to 255
int SalEepromWriteCommand(void)
{
   unsigned int u16_board, u16_addr, u16_index, u16_byte, u16_data, u16_eeprom_size_byte;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (s64_Execution_Parameter[0] > 2LL) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[0] < 1LL)  return (VALUE_TOO_LOW);

   u16_board = (unsigned int)s64_Execution_Parameter[0];

   if ((u16_Product == DDHD) && (u16_Axis_Num == 1) && (u16_board == 2))  return (NOT_AVAILABLE);

   if (u16_board == 1)
      u16_eeprom_size_byte = u16_Control_Eeprom_Size_Byte;
   else
      u16_eeprom_size_byte = BGVAR(u16_Power_Eeprom_Size_Byte);

   if (s64_Execution_Parameter[1] >= (long long)u16_eeprom_size_byte) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[1] < 0LL)  return (VALUE_TOO_LOW);

   if (s64_Execution_Parameter[2] > 255LL) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[2] < 0LL)  return (VALUE_TOO_LOW);

   u16_board = (unsigned int)s64_Execution_Parameter[0];
   u16_addr = (unsigned int)s64_Execution_Parameter[1];
   u16_data = (unsigned int)s64_Execution_Parameter[2];

   if ((0x64 == u16_addr) && (1 == u16_board))
   {
       if (2 != s16_Password_Flag) //do not allow Fw_Feature upgrade by overwrite without Admin password
         return (SAL_SUCCESS);
       else //admin mode: allow Fw_Feature upgrade by overwrite. But first: advance dynamic password value for next attempt.
       {
         u16_Dynamic_Pass += 1;
         u16_Control_Board_Eeprom_Buffer[DYN_PASS_ADDR] &= 0x00ff;
         u16_Control_Board_Eeprom_Buffer[DYN_PASS_ADDR] |= (u16_Dynamic_Pass << 8);
       }
   }
   u16_byte = u16_addr & 0x0001;
   u16_index = u16_addr >> 1;

   if (u16_board == 1)
   {
      if (u16_byte == 0)
      {
         u16_Control_Board_Eeprom_Buffer[u16_index] &= 0xff00;
         u16_Control_Board_Eeprom_Buffer[u16_index] |= u16_data;
      }
      else
      {
         u16_Control_Board_Eeprom_Buffer[u16_index] &= 0x00ff;
         u16_Control_Board_Eeprom_Buffer[u16_index] |= (u16_data << 8);
      }
   }
   else
   {
      if (u16_byte == 0)
      {
         u16_Power1_Board_Eeprom_Buffer[u16_index] &= 0xff00;
         u16_Power1_Board_Eeprom_Buffer[u16_index] |= u16_data;
      }
      else
      {
         u16_Power1_Board_Eeprom_Buffer[u16_index] &= 0x00ff;
         u16_Power1_Board_Eeprom_Buffer[u16_index] |= (u16_data << 8);
      }
   }
   return (SAL_SUCCESS);
}


// Read HW features from EEPROM memory
// Parameters:
//    Board: 1 - control, 2 - power1
//    Address: 86 - 255
int SalHwFeaturesCommand(int drive)
{
   unsigned int u16_board, u16_addr;

   int s16_response1=0,s16_response2=0,s16_response3=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (s16_Number_Of_Parameters == 2)// 2 parameters (BOARD,ADDR or Special code).
   {// regular EEPROM read (1- from control board , 2- from power board) , eeprom addr or special code.
      if (s64_Execution_Parameter[0] > 02L) return (VALUE_TOO_HIGH);
      if (s64_Execution_Parameter[0] < 01L)  return (VALUE_TOO_LOW);

      if (s64_Execution_Parameter[1] > 0x1009) return (VALUE_TOO_HIGH);
      if(s64_Execution_Parameter[1] < 0x1000)
      {
         if (s64_Execution_Parameter[1] > 255LL) return (VALUE_TOO_HIGH);
         if (s64_Execution_Parameter[1] < 86LL)  return (VALUE_TOO_LOW);

         u16_board = (unsigned int)s64_Execution_Parameter[0];
         u16_addr  = (unsigned int)s64_Execution_Parameter[1];
         s16_response1 = (int)GetEEpromRightByteFromAddress(u16_board,u16_addr);
      }
      else
      {
         u16_addr  = (unsigned int)s64_Execution_Parameter[1];
         s16_response1 = (int)HwFeaturesFeaturesValue(u16_addr);
      }
      PrintSignedInteger(s16_response1);
   }
   else if (s16_Number_Of_Parameters == 1)
   {
      if (s64_Execution_Parameter[0] < 0x1000) return (VALUE_TOO_LOW);
      if (s64_Execution_Parameter[0] > 0x1009) return (VALUE_TOO_HIGH);

      u16_addr  = (unsigned int)s64_Execution_Parameter[0];
      s16_response1 = (int)HwFeaturesFeaturesValue(u16_addr);
      PrintSignedInteger(s16_response1);
   }
   else if(s16_Number_Of_Parameters == 0)
   {// no parameters return the value of 0x1006 , 0x1007, 0x1008 in one line.
      s16_response1 = (int)HwFeaturesFeaturesValue(0x1006);
      PrintSignedInteger(s16_response1);
      PrintChar(',');
      s16_response2 = (int)HwFeaturesFeaturesValue(0x1007);
      PrintSignedInteger(s16_response2);
      PrintChar(',');
      s16_response3 = (int)HwFeaturesFeaturesValue(0x1008);
      PrintSignedInteger(s16_response3);
   }
   PrintCrLf();
   return SAL_SUCCESS;
}


// Dump I2C EEPROM memory
// Parameters:
//    Board: 1 - control, 2 - power1
int SalEepromDumpCommand(void)
{
   unsigned int u16_return_value, u16_board, u16_eeprom_size_word;
   unsigned int *p_u16_eeprom_buffer;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u16_return_value = SAL_NOT_FINISHED;

   if (s64_Execution_Parameter[0] > 2LL) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[0] < 1LL)  return (VALUE_TOO_LOW);

   if (u8_Output_Buffer_Free_Space < 60) return SAL_NOT_FINISHED;

   u16_board = (unsigned int)s64_Execution_Parameter[0];
   if (u16_board == 1)
   {
      u16_eeprom_size_word = u16_Control_Eeprom_Size_Byte >> 1;
      p_u16_eeprom_buffer = u16_Control_Board_Eeprom_Buffer;
   }
   else
   {
      u16_eeprom_size_word = BGVAR(u16_Power_Eeprom_Size_Byte) >> 1;
      p_u16_eeprom_buffer = u16_Power1_Board_Eeprom_Buffer;
   }

   if (u16_eeprom_size_word == 0) return NOT_AVAILABLE;

   switch (u16_Dump_Eeprom_State)
   {
      case 0:
         PrintStringCrLf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F", 0);
         u16_Dump_Eeprom_State++;
      break;

      case 1:
         PrintStringCrLf("    -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --", 0);
         u16_Dump_Eeprom_Word_Index = 0;
         u16_Dump_Eeprom_Word_Index_Line_End = 8;
         u16_Dump_Eeprom_State++;
      break;

      case 2:
         PrintHexChar((u16_Dump_Eeprom_Word_Index >> 3) & 0x000f);
         PrintString("0:", 0);
         while ( (u16_Dump_Eeprom_Word_Index < u16_eeprom_size_word) && (u16_Dump_Eeprom_Word_Index < u16_Dump_Eeprom_Word_Index_Line_End) )
         {
            PrintChar(' ');
            PrintHexChar((p_u16_eeprom_buffer[u16_Dump_Eeprom_Word_Index] >> 4) & 0x000f);
            PrintHexChar( p_u16_eeprom_buffer[u16_Dump_Eeprom_Word_Index] & 0x000f);
            PrintChar(' ');
            PrintHexChar((p_u16_eeprom_buffer[u16_Dump_Eeprom_Word_Index] >> 12) & 0x000f);
            PrintHexChar((p_u16_eeprom_buffer[u16_Dump_Eeprom_Word_Index] >> 8) & 0x000f);
            u16_Dump_Eeprom_Word_Index++;
         }

         if (u16_Dump_Eeprom_Word_Index >= u16_eeprom_size_word)
         {
            u16_Dump_Eeprom_State = 0;
            u16_return_value = SAL_SUCCESS;
         }
         else
            u16_Dump_Eeprom_Word_Index_Line_End += 8;
         PrintCrLf();
      break;
   }

   return (u16_return_value);
}


// Save I2C EEPROM memory (copy RAM buffer to EEPROM)
// Parameters:
//    Board: 1 - control, 2 - power1
int SalEepromSaveCommand(void)
{
   unsigned int u16_return_value, u16_board;

   if (Enabled(0)) return (DRIVE_ACTIVE);
   u16_return_value = SAL_NOT_FINISHED;

   if (s64_Execution_Parameter[0] > 2LL) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[0] < 1LL)  return (VALUE_TOO_LOW);

   u16_board = (unsigned int)s64_Execution_Parameter[0];

   if ((u16_Product == DDHD) && (u16_Axis_Num == 1) && (u16_board == 2))  return (NOT_AVAILABLE);

   if ((u16_Eeprom_Busy > 0) & (u16_Eeprom_Busy != u16_board))  return (I2C_BUSY);

   if (u16_Eeprom_Busy == 0)
   {
      // Trigger the write operation
      if (u16_board == 1)
      {
         p_u16_Eeprom_Buffer = u16_Control_Board_Eeprom_Buffer;
         u16_Write_Eeprom_Board = CONTROL_BOARD_EEPROM;
         u16_Eeprom_Address = CONTROL_EEPROM_ADDR;
      }
      else
      {
         p_u16_Eeprom_Buffer = u16_Power1_Board_Eeprom_Buffer;
         u16_Write_Eeprom_Board = POWER1_BOARD_EEPROM;
         u16_Eeprom_Address = POWER1_EEPROM_ADDR;
      }
      u16_Eeprom_Busy = u16_board;
      u16_Save_Eeprom_State = WRITE_PREPARATION;
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


void SetWriteProtectSignal()
{
   if((u16_Product == DDHD) && (IS_EC_DRIVE))
   {// currently for DDHD ECAT drive Set and RESET the write protect by both writing to the FPGA register and changing the GPIO48 state.
    // after the required HW changes the write protect for DDHD ECAT will be controlled direct from DSP GPIO48 without using the FPGA
    // in order to be able to write the EEPROM with no FPGA loaded first.
      GpioDataRegs.GPBDAT.bit.GPIO48 = 1;
      *((int*)(FPGA_POWER_CONTROL_REG_ADD)) |= 0x08; // set write protect signal
   }
   else if ( (u16_Product == DDHD) || (u16_Product == SHNDR_HW) || (IS_CAN2_DRIVE) || (IS_EC2_DRIVE))
   {
      GpioDataRegs.GPBDAT.bit.GPIO48 = 1;
   }
   else
   {
      *((int*)(FPGA_POWER_CONTROL_REG_ADD)) |= 0x08; // set write protect signal
   }
}


void RemoveWriteProtectSignal()
{
   if((u16_Product == DDHD) && (IS_EC_DRIVE))
   {// currently for DDHD ECAT drive Set and RESET the write protect by both writing to the FPGA register and changing the GPIO48 state.
    // after the required HW changes the write protect for DDHD ECAT will be controlled direct from DSP GPIO48 without using the FPGA
    // in order to be able to write the EEPROM with no FPGA loaded first.
      GpioDataRegs.GPBDAT.bit.GPIO48 = 0;
      *((int*)(FPGA_POWER_CONTROL_REG_ADD)) &= ~0x08; // remove write protect signal
   }
   else if ( (u16_Product == DDHD) || (u16_Product == SHNDR_HW) ||  (IS_CAN2_DRIVE) || (IS_EC2_DRIVE))
   {
      GpioDataRegs.GPBDAT.bit.GPIO48 = 0;
   }
   else
   {
      *((int*)(FPGA_POWER_CONTROL_REG_ADD)) &= ~0x08; // remove write protect signal
   }
}

// The GetEEpromRightByteFromAddress function will return the relevand data BYTE according to the
// provided EEPROM address


unsigned int GetEEpromRightByteFromAddress(unsigned int u16_board, unsigned int u16_eeprom_address)
{
   unsigned int u16_data, u16_index, u16_byte;

   u16_byte = u16_eeprom_address & 0x0001;
   u16_index = u16_eeprom_address >> 1;

   if (u16_board == CONTROL_BOARD_EEPROM)//control board
      u16_data = u16_Control_Board_Eeprom_Buffer[u16_index];
   else //power board
      u16_data = u16_Power1_Board_Eeprom_Buffer[u16_index];

   if (u16_byte == 1)
      u16_data = (u16_data >> 8);

   return u16_data & 0x00ff;
}



