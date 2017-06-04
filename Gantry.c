#include "DSP2834x_Device.h"

#include "Err_Hndl.def"
#include "MultiAxis.def"
#include "FPGA.def"
#include "i2c.def"

#include "Extrn_Asm.var"
#include "Drive.var"
#include "User_Var.var"
#include "FltCntrl.var"
#include "i2c.var"

#include "Prototypes.pro"


//**********************************************************
// Function Name: InitGantryMode
// Description:
//    This function is called in order to initialize certain
//    components of the Drive, such as the FPGA, in case that
//    settings have been changed regarding the Gantry mode.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void InitGantryMode(int drive, unsigned int u16_gantry_mode)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_gantry_mode == 1)
   {
      // Tell the FPGA to work in Gantry Y1 mode
      *(unsigned int*)FPGA_GANTRY_GENERAL_PURPOSE_REG_ADD = u16_gantry_mode << 3;  // Set value of 1 in bits 3...4, indicate axis Y1.
                                                                                   // Clear bit 10, which stops the FIFO reset.
   }
   else if(u16_gantry_mode == 2)
   {
      // Tell the FPGA to work in Gantry Y2 mode
      *(unsigned int*)FPGA_GANTRY_GENERAL_PURPOSE_REG_ADD = u16_gantry_mode << 3;   // Set value of 2 in bits 3...4, indicate axis Y2.
                                                                                    // Clear bit 10, which stops the FIFO reset.
   }
   else
   {
      // tell the FPGA that no Gantry mode has been selected
      *(unsigned int*)FPGA_GANTRY_GENERAL_PURPOSE_REG_ADD = 0x0400;     // Set value of 0 in bits 3...4, indicate that no Y1 and no Y2 mode.
                                                                        // Set bit 10, which starts the FIFO reset.
      *(unsigned int*)FPGA_PLL_LOCKED_REG_ADD = 0;                      // State that the Drive (Y2) is not locked.
   }
}

//**********************************************************
// Function Name: SalGantryModeCommand
// Description:
//    This function is called upon the GANTRYMODE command.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalGantryModeCommand(long long param, int drive)
{
   // AXIS_OFF;

   if ((FILEDBUS_MODE) && (param == 2))
   {
      return (WRONG_COMMODE);
   }
   else if( (param != 0) &&
            ( (u16_DSP_Type != DSP_TMS320C28346) ||                                                                              // If not new DSP TMS320C28346
              ((u16_Control_Board_Eeprom_Buffer[RES_FW_CODE_TYPE__MORE_HW_FEATURES__ADDR] & 0x0004) == 0x0004)    // If no Gantry hardware available
            )
          )
   {
      return (NOT_SUPPORTED_ON_HW);
   }

   VAR(AX0_u16_Gantry_Mode) = (unsigned int)param;

   InitGantryMode(drive, VAR(AX0_u16_Gantry_Mode));

   return (SAL_SUCCESS);
}


