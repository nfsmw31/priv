#include <string.h>
#include "Math.h"

#include "cal_conf.h"
#include "objects.h"
#include "402fsa.h"
#include "emerg.h"

#include "Design.def"
#include "Err_Hndl.def"
#include "Exe_IO.def"
#include "FPGA.def"
#include "Homing.def"
#include "MultiAxis.def"
#include "Ser_Comm.def"
#include "Velocity.def"
#include "PtpGenerator.def"
#include "Init.def"
#include "Display.def"
#include "AutoTune.def"
#include "402fsa.def"
#include "ModCntrl.def"
#include "ExFbVar.def"

#include "An_Fltr.var"
#include "Drive.var"
#include "Exe_IO.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Foldback.var"
#include "Homing.var"
#include "i2c.def"
#include "ModCntrl.var"
#include "Motor.var"
#include "MotorSetup.var"
#include "PtpGenerator.var"
#include "Ser_Comm.var"
#include "Units.var"
#include "Velocity.var"
#include "Drive_Table.var"
#include "Init.var"
#include "position.var"
#include "ExFbVar.var"
#include "Display.var"
#include "AutoTune.var"
#include "Zeroing.var"
#include "User_Var.var"

#include "Prototypes.pro"
#include "ExSwVar.pro"


#pragma CODE_SECTION(InxValue, "ramcan");
int InxValue(int index,int drive)
{
   // AXIS_OFF;
   int mask = (1 << (index - 1));
   REFERENCE_TO_DRIVE;
   
   if ((VAR(AX0_u16_Input_State) & mask) == 0) return 0;
   return 1;
}


//**********************************************************
// Function Name: OxValue
// Description:
//          This function returns either a 0 or a 1 depending
//          on, if the requested output-state is true or false
//          considering also the polarity parameter. The function
//          argument "index" indicates the number of the digital
//          input to query the state from.
// Author:  S.F
// Algorithm:
// Revisions:
//**********************************************************
int OxValue(int index)
{
   int drive = 0;
   int mask = (1 << index);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(u16_Digital_Outputs_Processed) & mask) == 0) return 0;
   else return 1;
}



//**********************************************************
// Function Name: OxValue2
// Description: Copy of 0xValue, used for record and trigger
// Author:  S.F
// Algorithm:
// Revisions:
//**********************************************************
int OxValue2(int index, unsigned int u16_Out_Data)
{
   int drive = 0;
   int mask = (1 << index);
   int value = u16_Out_Data ^ BGVAR(u16_Dig_Out_Polar);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((value & mask) == 0) return 0;
   else return 1;
}



//**********************************************************
// Function Name: SalReadInputsCommand
// Description:
//          This function is called in response to the IN command.
// Author:  S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalReadInputsCommand(void)
{
   int i, drive = 0;

   for (i = 1; i <= s16_Num_Of_Inputs; i++)
   {
      PrintSignedInteger(i);
      PrintChar(' ');
   }
   PrintCrLf();

   for (i = 1; i <= s16_Num_Of_Inputs; i++)
   {
      if (u16_Supported_Dig_Inputs_Mask & (1 << (i - 1)))
         PrintChar('0' + InxValue(i, drive));
      else
         PrintChar('X');

      PrintChar(' ');
      if (i >= 9)
         PrintChar(' ');
   }

   PrintCrLf();
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadInPolarCommand
// Description:
//    This function is called in response to the DINXPOLAR command.
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalReadInPolarCommand(void)
{
   unsigned int u16_mask;
   long long index;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   index = s64_Execution_Parameter[0];

   if ((index < 1) || (index > (long long)s16_Num_Of_Inputs)) return (VALUE_OUT_OF_RANGE);
   u16_mask = (1 << (int)(index - 1));

   if (!(u16_Supported_Dig_Inputs_Mask & u16_mask)) return (I_O_NOT_SUPPORTED);

   if ((BGVAR(u16_Dig_In_Polar) & u16_mask) != 0) PrintChar('1');
   else PrintChar('0');
   PrintCrLf();

   return (SAL_SUCCESS);
}


int SalInPolarCommand(void)
{
   unsigned int u16_mask;
   long long index, value;
   unsigned int u16_value;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   index = s64_Execution_Parameter[0];
   value = s64_Execution_Parameter[1];

   if ((index < 1LL) || (index > (long long)s16_Num_Of_Inputs)) return (VALUE_OUT_OF_RANGE);
   if ((value < 0LL) || (value > 1LL)) return (VALUE_OUT_OF_RANGE);

   u16_mask = (1 << (int)(index-1LL));

   if (!(u16_Supported_Dig_Inputs_Mask & u16_mask)) return (I_O_NOT_SUPPORTED);

   if (value == 0) BGVAR(u16_Dig_In_Polar) &= ~u16_mask;
   else BGVAR(u16_Dig_In_Polar) |= u16_mask;

   // This is to make sure both axes has the same value, it is defined as BGVAR for "SAVE" purposes
   u16_value = BGVAR(u16_Dig_In_Polar);
   drive = 1;
   BGVAR(u16_Dig_In_Polar) = u16_value;

   WriteInputPolarityToFPGA();

   return (SAL_SUCCESS);
}

int SalOutBrakeReadCommand(long long *data,int drive)
{
   //int value = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   
   *data = (long long)(!BGVAR(u16_Manual_Out_Brake_Value));
   return SAL_SUCCESS;
}

int SalOutBrakeWriteCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;// Defeat compilation error
   if ( param == 1LL || param == 0LL)
   {
      BGVAR(u16_Manual_Out_Brake_Value) = (unsigned int)(!param);
      return SAL_SUCCESS;
      
   }
   else
   {
      return (VALUE_OUT_OF_RANGE);
   }  
}
void ZPulseUpdate(void)
{
   if (u16_Out_Mode[3 - 1] == u16_Out_Mode[6 - 1] && u16_Out_Mode[6]== Z_PULSE_OUT)
   {
      *(int*)FPGA_FPGA_C_FAST_OUT_3_REG_ADD = 0;
      *(int*)FPGA_FPGA_M_FAST_OUT_6_REG_ADD = 0;
   }
   else
   {
      *(int*)FPGA_FPGA_C_FAST_OUT_3_REG_ADD = (u16_Out_Mode[3 - 1]== Z_PULSE_OUT) ? 0x1 : 0x0;
      *(int*)FPGA_FPGA_M_FAST_OUT_6_REG_ADD = (u16_Out_Mode[6 - 1]== Z_PULSE_OUT) ? 0x1 : 0x0;
      *(int*)FPGA_DIGITAL_OUTPUT_7_MUX_ADD  = (u16_Out_Mode[FAST_OUTPUT_1 - 1]== Z_PULSE_OUT) ? 0x3 : 0x0;
      *(int*)FPGA_DIGITAL_OUTPUT_8_MUX_ADD  = (u16_Out_Mode[FAST_OUTPUT_2 - 1]== Z_PULSE_OUT) ? 0x3 : 0x0;
   }
}

void WriteInputPolarityToFPGA(void)
{
   int drive = 0;
   int u16_used_input;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *(int*)FPGA_AX0ENABLE_POLARITY_REG_ADD     =  BGVAR(u16_Dig_In_Polar)       & 0x0001;
   *(int*)FPGA_CONTROL_INPUT_POLARITY_REG_ADD = (BGVAR(u16_Dig_In_Polar) >> 1) & 0x001F;
   *(int*)FPGA_MACHINE_INPUT_POLARITY_REG_ADD = (BGVAR(u16_Dig_In_Polar) >> 6) & 0x001F;

   // Dealing with the Probes polarity cases
   // polarity will be invoked for digital inputs only.
   u16_used_input = *(int *)FPGA_INPUT_CAPTURE_SELECT_2_REG_ADD;
   if ((u16_used_input > 0) && (u16_used_input < INDEX_INPUT))
      *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_2_REG_ADD = ((BGVAR(u16_Dig_In_Polar) ^ VAR(AX0_u16_LX28_LS_Location)) & (1<<(u16_used_input-1)))==0?1:0;

   u16_used_input = *(int *)FPGA_INPUT_CAPTURE_SELECT_3_REG_ADD;
   if ((u16_used_input > 0) && (u16_used_input < INDEX_INPUT))
      *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_3_REG_ADD = ((BGVAR(u16_Dig_In_Polar) ^ VAR(AX0_u16_LX28_LS_Location)) & (1<<(u16_used_input-1)))==0?1:0;
}


int SalReadOutPolarCommand(void)
{
   unsigned int u16_mask;
   long long index;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   index = s64_Execution_Parameter[0];
   if ((index < 1LL) || (index > (long long)s16_Num_Of_Outputs)) return (VALUE_OUT_OF_RANGE); 
   u16_mask = (1 << (int)(index - 1));

   if (!(u16_Supported_Dig_Outputs_Mask & u16_mask)) return (I_O_NOT_SUPPORTED);

   if ((BGVAR(u16_Dig_Out_Polar) & u16_mask) != 0) PrintChar('1');
   else PrintChar('0');
   PrintCrLf();

   return (SAL_SUCCESS);
}


int SalOutPolarCommand(void)
{
   unsigned int u16_mask;
   long long index, value;
   unsigned int u16_value;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   index = s64_Execution_Parameter[0];
   value = s64_Execution_Parameter[1];

   if ((index < 1LL) || (index > (long long)s16_Num_Of_Outputs)) return (VALUE_OUT_OF_RANGE);
   if ((value < 0LL) || (value > 1LL)) return (VALUE_OUT_OF_RANGE);
   if ( (index == PWR_BRAKE_OUT_NUM) && (u16_Power_Brake_Exist) )  // we don't allow inversion for power brake
       return NOT_PROGRAMMABLE;

   u16_mask = (1 << (int)(index - 1LL));

   if (!(u16_Supported_Dig_Outputs_Mask & u16_mask)) return (I_O_NOT_SUPPORTED);

   if (value == 0LL) BGVAR(u16_Dig_Out_Polar) &= ~u16_mask;
   else BGVAR(u16_Dig_Out_Polar) |= u16_mask;

   // This is to ensure both axes has the same value, it is defined as BGVAR for "SAVE" purposes
   u16_value = BGVAR(u16_Dig_Out_Polar);
   drive = 1;
   BGVAR(u16_Dig_Out_Polar) = u16_value;

   FastOutInv(0);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadOutputStatusDisplay
// Description:
//          This function is called in response to the read P4-09 (the CDHD
//          equivalent is the OUTPUTS command). This function generates a
//          variable where each digital output state is represented in one bit
//          of the return value. This function considers also the selected polarity
//          of each digital output since it uses the function "OxValue".
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadOutputStatusDisplay(long long* data)
{

   int i;
   unsigned int u16_temp_val = 0;

   // Generate a bit-variable where each bit represents an output state.
   // Bit 0 - Digital Output State 1
   // Bit 1 - Digital Output State 2...
   for (i = 0; i < s16_Num_Of_Outputs; i++)
      u16_temp_val |= OxValue(i) << i;

   *data = u16_temp_val;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: GetOutputStatusDisplay
// Description:
//          Copy of SalReadOutputStatusDisplay, to be used for record and trigger
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int GetOutputStatusDisplay(long long* data, unsigned int u16_Out_Data)
{

   int i;
   unsigned int u16_temp_val = 0;

   // Generate a bit-variable where each bit represents an output state.
   // Bit 0 - Digital Output State 1
   // Bit 1 - Digital Output State 2...
   for (i = 0; i < s16_Num_Of_Outputs; i++)
      u16_temp_val |= OxValue2(i,u16_Out_Data) << i;

   *data = u16_temp_val;

   return SAL_SUCCESS;
}



//**********************************************************
// Function Name: SalReadOutputsCommand
// Description:
//          This function is called in response to the IN command.
// Author:  S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalReadOutputsCommand(void)
{
   int i;
   unsigned int u16_temp_val = 0;

   for (i = 1; i <= s16_Num_Of_Outputs; i++)
   {
      PrintSignedInteger(i);
      PrintChar(' ');
   }
   PrintCrLf();

   for (i = 0; i < s16_Num_Of_Outputs; i++)
   {      
      u16_temp_val = OxValue(i);

      if (u16_Supported_Dig_Outputs_Mask & (1 << i))
         PrintChar('0' + u16_temp_val);
      else
         PrintChar('X');

      PrintChar(' ');
      if (i >= 8)
         PrintChar(' ');
   }

   PrintCrLf();
   return (SAL_SUCCESS);
}


int SalReadOutCommand(void)
{
   unsigned int u16_temp = 0;
   long long index;

   index = s64_Execution_Parameter[0];

   if ((index < 1LL) || (index > (long long)s16_Num_Of_Outputs)) return (VALUE_OUT_OF_RANGE);
   if (!(u16_Supported_Dig_Outputs_Mask & (1 << (int)(index - 1LL)))) return (I_O_NOT_SUPPORTED);

   u16_temp = OxValue((int)(index - 1LL));

   PrintSignedInteger(u16_temp);
   PrintCrLf();

   return (SAL_SUCCESS);
}


int SalOutCommand(void)
{
   long long index, value;

   index = s64_Execution_Parameter[0];
   value = s64_Execution_Parameter[1];

   return OutCommand(index, value);
}


int OutCommand(long long index,long long value)
{
   if ((index < 1LL) || (index > (long long)s16_Num_Of_Outputs)) return (VALUE_OUT_OF_RANGE);
   if ((value != 0LL) && (value != 1LL)) return (VALUE_OUT_OF_RANGE);
   if (!(u16_Supported_Dig_Outputs_Mask & (1 << (int)(index - 1LL)))) return (I_O_NOT_SUPPORTED);

   if (u16_Out_Mode[index - 1LL] != NO_FUNC_OUT) return (NOT_PROGRAMMABLE);
   u16_Out_State[index - 1LL] = (int)value;

//ITAI - had to remove the below 4 lines. it made problems wit fieldbus PDOs and I think they are wrong.
/*   if (value)
      u16_Digital_Outputs |= 1 << (int)(index - 1LL);
   else
      u16_Digital_Outputs &= (~1 << (int)(index - 1LL));
*/
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadInCommand
// Description:
//    This function is called in response to the DINX command.
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalReadInCommand(void)
{
   long long index;
   int drive=0;

   index = s64_Execution_Parameter[0];
   if ((index < 1LL) || (index > (long long)s16_Num_Of_Inputs)) return (VALUE_OUT_OF_RANGE);

   if (!(u16_Supported_Dig_Inputs_Mask & (1 << (int)(index - 1LL)))) return (I_O_NOT_SUPPORTED);

   PrintSignedInteger(InxValue((int)(index), drive));
   PrintCrLf();

   return (SAL_SUCCESS);
}


int SalRelayCommand(long long* relay)
{
   *relay = *(unsigned int*)FPGA_FAULT_RELAY_REG_ADD & 0x0001LL;

   return SAL_SUCCESS;
}


// Checks if a certain INXMODE is configured
#pragma CODE_SECTION(IsInFunctionalityConfigured, "ramcan");
int IsInFunctionalityConfigured(int func,int drive)
{
   int i;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   
   for (i = 1; i <= s16_Num_Of_Inputs; i++)
   {
      if ((VAR(AX0_u16_Input_Mode_Arr[i]) == func) && (u16_Supported_Dig_Inputs_Mask & (1 << (i - 1))))
         return (i);
   }

   return 0;
}


// Checks if a certain OXMODE is configured
int IsOutFunctionalityConfigured(int func)
{
   int i;

   for (i = 0; i < s16_Num_Of_Outputs; i++)
   {
      if ((u16_Out_Mode[i] == func) && (u16_Supported_Dig_Outputs_Mask & (1 << i)))
         return (i + 1);
   }
   return 0;
}


//**********************************************************
// Function Name: SalReadInModeCommand
// Description:
//    This function is called in response to the DINXMODE command.
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalReadInModeCommand(void)
{
   long long index;
   
   // AXIS_OFF;
   index = s64_Execution_Parameter[0];
   if ((index < 1LL) || (index > (long long)s16_Num_Of_Inputs)) return (VALUE_OUT_OF_RANGE);

   if (!(u16_Supported_Dig_Inputs_Mask & (1 << (int)(index - 1)))) return (I_O_NOT_SUPPORTED);

   PrintSignedInteger(VAR(AX0_u16_Input_Mode_Arr[index]));
   PrintCrLf();

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalInModeCommand
// Description:
//          This function is called in response to the DINXMODE command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalInModeCommand(void)
{
    long long index = s64_Execution_Parameter[0], func = s64_Execution_Parameter[1];
    long  s32_Timer_Start_Capture_1ms = 0;
    unsigned int i;
    int drive = 0, s16_default_update_limit = 0;
    // AXIS_OFF;

    if ((index < 1LL) || (index > (long long)s16_Num_Of_Inputs))
    {
        return VALUE_OUT_OF_RANGE;
    }

    if ((func < 0LL)  || (func >= CDHD_IN_MODE_TABLE_SIZE))
    {
        return VALUE_OUT_OF_RANGE;
    }

    if (!(u16_Supported_Dig_Inputs_Mask & (1 << (int)(index - 1LL))))
    {
        return I_O_NOT_SUPPORTED;
    }

    if (DDHD == u16_Product)
    {
//      if ((func == HIGH_SPD_PUSLE_INP) && (index != 1) && (index != 3))  return FUNC_NOT_SUPPORTED_ON_IO;
//      if ((func == HIGH_SPD_DIR_INP) && (index != 2) && (index != 4))  return FUNC_NOT_SUPPORTED_ON_IO;
        if ((func == HIGH_SPD_PUSLE_INP) && (index > 5))
        {
            return FUNC_NOT_SUPPORTED_ON_IO;
        }

        if ((func == HIGH_SPD_DIR_INP) && (index > 4) && (index != 6))
        {
            return FUNC_NOT_SUPPORTED_ON_IO;
        }
    }
    else
    {
        if ((func == HIGH_SPD_PUSLE_INP) && (index != 5))
        {
            return FUNC_NOT_SUPPORTED_ON_IO;
        }

        if ((func == HIGH_SPD_DIR_INP) && (index != 6))
        {
            return FUNC_NOT_SUPPORTED_ON_IO;
        }
    }

    //  Itai - Temporary block of modes 15 & 16 (dec on input) since they do not work
    if ((func == DEC_ON_INPUT) || (func == DEC_ON_INPUT2))
    {
        return VALUE_OUT_OF_RANGE;
    }

    //  Set the same inmode --> do nothing
    if (VAR(AX0_u16_Input_Mode_Arr[index]) == func)
    {
        return SAL_SUCCESS;
    }

    //  Check that function is not occupied (unless emergency stop or no_func)
    if (((func != NO_FUNC_INP) && (func != EMERGENCY_STOP_INP)) && (IsInFunctionalityConfigured(func, drive)))
    {
        return FUNCTIONALITY_OCCUPIED;
    }

    if (((func == HIGH_SPD_PUSLE_INP) || (func == HIGH_SPD_DIR_INP)) && (!IS_HW_FUNC_ENABLED(PULSE_DIRECTION_INPUT_MASK)))
    {
        return NOT_SUPPORTED_ON_HW;
    }

    //  Accept inmode and Store prev function
    i = VAR(AX0_u16_Input_Mode_Arr[index]);

    UpdateIndexFunc(drive,(int)func,index);

    //  make sure AX0_u16_Input_Mode_Arr was updated by RT as requested (this is added after observing BG proceeds before ready)
    while ((func != VAR(AX0_u16_Input_Mode_Arr[index])) && (5 > s16_default_update_limit))
    {
        s32_Timer_Start_Capture_1ms= Cntr_1mS;
        while (!PassedTimeMS(1, s32_Timer_Start_Capture_1ms));
        s16_default_update_limit++;
    }

    if (VAR(AX0_u16_Input_Mode_Arr[index]) == RESET_INP)
    {
        //  this input associated with clr fault funaction
        s16_Clr_Flt_In = (int)(index);
    }
    if (i == RESET_INP)
    {
        //  no input is associated with clr fault function
        s16_Clr_Flt_In = -1;
    }

    //  Set P&D Mux on FPGA depending on I/O 5,6 inmodes (1,2 / 3,4 in DDHD)
    PulseandDirectionFpgaMUX(drive);

    //  if remote-en functionality was removed then remove sw en
    if (i == REMOTE_ENABLE_INP)
    {
        BGVAR(s16_DisableInputs) |= SW_EN_MASK;
    }

    return SAL_SUCCESS;
}


void UpdateIndexFunc(int drive, int func, int index)
{
   long  s32_Timer_Start_Capture_1ms= Cntr_1mS;

   // AXIS_OFF;
   VAR(AX0_u16_InMode_Update)=index + (func << 8);
   //Wait till the update register is cleared at RT
   while ((VAR(AX0_u16_InMode_Update)!=0) && !PassedTimeMS(1, s32_Timer_Start_Capture_1ms));
   Update_CW_CCW_Ind(drive, func, index);
}


void Update_CW_CCW_Ind(int drive, int func, int index)
{
   int i;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (index != 0)
   {
      //in case the cell pointed on ccw/cw need to erase it
      if ((VAR(AX0_u16_CW_CCW_Inputs) & 0xff) == index)
      {
         VAR(AX0_u16_CW_CCW_Inputs) &= 0xff00;
         BGVAR(u64_Sys_Warnings) &= (~CW_HW_LS_WRN_MASK);
      }
       if ((VAR(AX0_u16_CW_CCW_Inputs) >> 8) == index)
      {
         VAR(AX0_u16_CW_CCW_Inputs) &= 0x00ff;
         BGVAR(u64_Sys_Warnings) &= (~CCW_HW_LS_WRN_MASK);
      }
      if (VAR(AX0_u16_CW_CCW_Inputs) == 0)
         BGVAR(u64_Sys_Warnings) &= (~CW_HW_LS_WRN_MASK & ~CCW_HW_LS_WRN_MASK & ~CW_CCW_HW_LS_WRN_MASK);

      //in case the cell supposed to point on ccw/cw
      if (func == CW_LIMIT_SW_INP || func == CCW_LIMIT_SW_INP)
      {
         VAR(AX0_u16_CW_CCW_Inputs)= (VAR(AX0_u16_CW_CCW_Inputs) & ((func - CW_LIMIT_SW_INP)?0x0ff:0xff00))
              | (index<<8*(func - CW_LIMIT_SW_INP));
      }

      if (func == SCRIPT_INP)
         BGVAR(u16_Prev_Script_Inp_State) = VAR(AX0_u16_Mode_Input_Arr[SCRIPT_INP]);
      if (func == SCRIPT_SEC_INP)
         BGVAR(u16_Prev_Script_Sec_Inp_State) = VAR(AX0_u16_Mode_Input_Arr[SCRIPT_SEC_INP]);
      return;
   }

   if (index == 0 && u16_Init_Indicator) return;

   // Dealing with initialization
   // In case of index==0
   //Loading the CW_LS_input to LSB and CCW_LS_input to MSB
   for (i = 1; i < 12; i++)
   {
      if (VAR(AX0_u16_Input_Mode_Arr[i])== CW_LIMIT_SW_INP || VAR(AX0_u16_Input_Mode_Arr[i])== CCW_LIMIT_SW_INP)
         VAR(AX0_u16_CW_CCW_Inputs)= (VAR(AX0_u16_CW_CCW_Inputs) & ((VAR(AX0_u16_Input_Mode_Arr[i]) - CW_LIMIT_SW_INP)?0x0ff:0xff00))
         | (i << 8 * (VAR(AX0_u16_Input_Mode_Arr[i]) - CW_LIMIT_SW_INP));
   }
   u16_Init_Indicator = 1;
}


void InitDigIOs(int drive)
{
   int i;
   drive+=0;
   STORE_EXECUTION_PARAMS_0_1
   for(i = 0; ((i <= s16_Num_Of_Inputs) && (i < 12)); i++)
   {
      s64_Execution_Parameter[0] = i;
      s64_Execution_Parameter[1] = NO_FUNC_INP;
      SalInModeCommand();
   }
   RESTORE_EXECUTION_PARAMS_0_1

   for(i = 0; ((i < s16_Num_Of_Outputs) && (i < MAX_NUM_OF_OUTPUTS)); i++)
   {
      u16_Out_Mode[i] = NO_FUNC_OUT;
      u16_Out_State[i] = 0;
   }
}


void InitDigIOsFunctions(int drive)
{
   int i;
    // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   for(i = 0; i <= s16_Num_Of_Inputs; i++)
   {
      // This input is associated with clr fault funaction
      if (VAR(AX0_u16_Input_Mode_Arr[i]) == RESET_INP) s16_Clr_Flt_In = i;
   }
}


void InitPWRBrakeOutput(int drive)
{
   drive += 0;// for remark avoidance
   // Initialize power brake output
   *(int*)FPGA_PWR_BRK_CMD_REG_ADD |= 0x0001;// lock the brake
}


int SalReadOutModeCommand(void)
{
   long long index;
   index = s64_Execution_Parameter[0];

   if ((index < 1LL) || (index > (long long)s16_Num_Of_Outputs)) return (VALUE_OUT_OF_RANGE);
   if (!(u16_Supported_Dig_Outputs_Mask & (1 << (int)(index - 1LL)))) return (I_O_NOT_SUPPORTED);

   PrintSignedInteger(u16_Out_Mode[index - 1LL]);
   PrintCrLf();

   return (SAL_SUCCESS);
}

int SalReadDifPortModeCommand(long long *data,int drive)
{
   long long index;
   index = s64_Execution_Parameter[0];
   drive += 0;

   if ((index < 1LL) || (index > 3LL)) return (VALUE_OUT_OF_RANGE);

   *data = u16_Dif_Port_Mode[index - 1LL];

   return (SAL_SUCCESS);
}


void FastOutInv(int drive)
{
   drive += 0;

   // if AUTOR is enable ignore this call to prevent change of polarity while AUTOR mode is enable,
   // this function will be recall as soon as AUTOR mode will be disabled.

   // P2-44 is ON in position mode and it's a Lexium drive - AUTOR output mode is enable
   if (((BGVAR(u16_P2_44_Autor_DoModeSet) & 0x01) == 1) &&
              ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)) &&
              (BGVAR(u16_LXM28_Opmode) == SE_OPMODE_PS)) return;

   if (u16_Out_Mode[3 - 1] == Z_PULSE_OUT &&  (BGVAR(u16_Dig_Out_Polar) & (1 << (3 - 1))))
      *(unsigned int*)FPGA_FPGA_C_OUT_POLARITY_REG_ADD |= 0x04;
   else
      *(unsigned int*)FPGA_FPGA_C_OUT_POLARITY_REG_ADD &= 0xFB;

   if (u16_Out_Mode[6 - 1] == Z_PULSE_OUT &&  (BGVAR(u16_Dig_Out_Polar) & (1 << (6 - 1))))
      *(unsigned int*)FPGA_FPGA_M_FAST_OUT_POLARITY_REG_ADD |= 0x04;
   else
      *(unsigned int*)FPGA_FPGA_M_FAST_OUT_POLARITY_REG_ADD &= 0xFB;

   return;
}

// OUTMODE
int SalOutModeCommand(void)
{
    long long index;
    long long func;
    int temp_state = 0;

    index = s64_Execution_Parameter[0];
    func = s64_Execution_Parameter[1];

    if ((index < 1LL) || (index > (long long)s16_Num_Of_Outputs))
    {
        return (VALUE_OUT_OF_RANGE);
    }

    //  DDHD mode drive
    if (DDHD == u16_Product)
    {
        if ((func < NOT_IN_USE) || (func >= CDHD_OUT_MODE_TABLE_SIZE))
        {
            return (VALUE_OUT_OF_RANGE);
        }
    }
    //  CDHD mode drive
    else
    {
        if ((func < 0LL) || (func >= CDHD_OUT_MODE_TABLE_SIZE))
        {
            return (VALUE_OUT_OF_RANGE);
        }
    }

    //  In case of fast_output,only output 3/6 are valid for this functionality.
    //  if one of the outputs owns the Z_PULSE functionality,then reject the request
    if (func == Z_PULSE_OUT)
    {
        if (!IS_HW_FUNC_ENABLED(EEO_MASK))
        {
            return NOT_SUPPORTED_ON_HW;
        }
        if (!((index == 3) || (index == 6)))
        {
            return (VALUE_OUT_OF_RANGE);
        }
        if (index == 6 && u16_Out_Mode[3 - 1] == Z_PULSE_OUT && (u16_Supported_Dig_Outputs_Mask & (1 << (3 - 1))))
        {
            return (FUNCTIONALITY_OCCUPIED);
        }
        if (index == 3 && u16_Out_Mode[6 - 1] == Z_PULSE_OUT && (u16_Supported_Dig_Outputs_Mask & (1 << (6 - 1))))
        {
            return (FUNCTIONALITY_OCCUPIED);
        }
    }
    //In case of fast_output,only output 7 & 8 are valid for this functionality.
    //if one of the outputs owns the PCOM1 functionality,then reject the request
    else if (((func == PCOM1_OUT) || (func == PCOM2_OUT)) &&
             (!(index == FAST_OUTPUT_1 || index == FAST_OUTPUT_2)))
    {
        return (VALUE_OUT_OF_RANGE);
    }

    if (!(u16_Supported_Dig_Outputs_Mask & (1 << (int)(index - 1LL))))
    {
        return (I_O_NOT_SUPPORTED);
    }

    //  Set the same outmode --> do nothing
    if (u16_Out_Mode[index - 1LL] == (int)func)
    {
        return (SAL_SUCCESS);
    }

    //  If no functionality set then init out value to current state
    //  Support for CDHD HW is needed
    if (func == NO_FUNC_OUT)
    {
        temp_state = OxValue((int)(index - 1LL));
        u16_Out_State[index - 1LL] = temp_state;
    }

    //  Accept outmode
    u16_Out_Mode[index - 1LL] = func;

    //  DDHD: if DO not in use, make sure value is 0 so other axis is not affected
    if ((DDHD == u16_Product) && (NOT_IN_USE == func))
    {
        OutCommand(index, 0LL);
    }

    if (func == Z_PULSE_OUT)
    {
        if (index == 3) *(int*)FPGA_FPGA_C_FAST_OUT_3_REG_ADD = 0x1;
        else if (index == 6) *(int*)FPGA_FPGA_M_FAST_OUT_6_REG_ADD = 0x1;
        else if (index == FAST_OUTPUT_1) *(int*)FPGA_DIGITAL_OUTPUT_7_MUX_ADD = 0x3; // mux value 3 for Encoder Simulation Index
        else if (index == FAST_OUTPUT_2) *(int*)FPGA_DIGITAL_OUTPUT_8_MUX_ADD = 0x3; // mux value 3 for Encoder Simulation Index
    }

    if(index == FAST_OUTPUT_1) //output 7
    {
        if(func == PCOM1_OUT)
        {
            *((unsigned int*)FPGA_DIGITAL_OUTPUT_7_MUX_ADD) = 1; //output 7 is occupied by PCOM1
        }
        else if(func == PCOM2_OUT)
        {
           *((unsigned int*)FPGA_DIGITAL_OUTPUT_7_MUX_ADD) = 2; //output 7 is occupied by PCOM2
        }
        else
	    {
           *((unsigned int*)FPGA_DIGITAL_OUTPUT_7_MUX_ADD) = 0; //output 7 is occupied by DSP
	    }
    }

    if(index == FAST_OUTPUT_2)
    {
        if(func == PCOM1_OUT)
        {
           *((unsigned int*)FPGA_DIGITAL_OUTPUT_8_MUX_ADD) = 1; //output 8 is occupied by PCOM1
        }
        else if(func == PCOM2_OUT)
        {
           *((unsigned int*)FPGA_DIGITAL_OUTPUT_8_MUX_ADD) = 2; //output 8 is occupied by PCOM2
        }
        else
	    {
           *((unsigned int*)FPGA_DIGITAL_OUTPUT_8_MUX_ADD) = 0; //output 8 is occupied by DSP
	    }
    }

    //  EEZ_OUT is available only for SE drive and only for output 6.
    if ((func == EEZ_OUT) && (index == 6))
    {
        //  route EEZ (Emulated Encoder Index) to DO6
        *(int*)FPGA_EEO_Z_OC_MUX_REG_ADD = 0;
    }
    //  route out6 mode to be taken from outmode and not from
    else if ((index == 6) && (func == NO_FUNC_OUT))
    {
        //  route OUT 6 (GP configurable)
        *(int*)FPGA_EEO_Z_OC_MUX_REG_ADD = 1;
    }
   
    // mark/unmark whether brake output is defiend in the drive
    BGVAR(u16_Brake_Output_Is_Defined) = IsOutFunctionalityConfigured(BRAKE_OUT);

    FastOutInv(0);

    return (SAL_SUCCESS);
}

// DIFPortMODE
int SalDifPortModeCommand(void)
{
   long long index, func;
   int s16_mask = 0;

   index = s64_Execution_Parameter[0];
   func = s64_Execution_Parameter[1];

   if ((index < 1LL) || (index > PCOM_MAX_DIFFERENTIAL_PORTS)) return (VALUE_OUT_OF_RANGE);
   if ((func < 0LL)  || (func  > PCOM_MAX_DIFFERENTIAL_FUNCS)) return (VALUE_OUT_OF_RANGE);

   if(index == 1) //Encoder Simulation A
   {
      //nothing to do for func = 0

      if (func == 1)
	  {
         //bits 12-15 = 1
		 s16_mask = 0x1000;
	  }
      else if (func == 2)
	  {
         //bits 12-15 = 2
		 s16_mask = 0x2000;
	  }

	  //clear nibble 4
	  *(int*)FPGA_EMULATED_ENCODER_MODE_ADD &= 0x0FFF;
   }
   else if(index == 2) //Encoder Simulation B
   {
      //nothing to do for func = 0

      if (func == 1)
	  {
         //bits 8-11 = 1
		 s16_mask = 0x0100;
	  }
      else if (func == 2)
	  {
         //bits 8-11 = 2
		 s16_mask = 0x0200;
	  }

	  //clear nibble 3
	  *(int*)FPGA_EMULATED_ENCODER_MODE_ADD &= 0xF0FF;
   }
   else if(index == 3) //Encoder Simulation I
   {
      //nothing to do for func = 0
      
      if (func == 1)
	  {
         //bits 3-7 = 1
		 s16_mask = 0x0010;
	  }
      else if (func == 2)
	  {
         //bits 3-7 = 2
		 s16_mask = 0x0020;
	  }
	  //clear nibble 2
	  *(int*)FPGA_EMULATED_ENCODER_MODE_ADD &= 0xFF0F;
   }

   //save value
   u16_Dif_Port_Mode[index - 1LL] = func;

   *(int*)FPGA_EMULATED_ENCODER_MODE_ADD |= s16_mask;
   return (SAL_SUCCESS);
}

// P4-27
int SalWriteDigOutputForceMask(long long lparam,int drive)
{
   drive += 0;

   if ( (u16_Flash_Type == 1) && (lparam > 15LL) ) // 2 * 64Mb flash, the drive is LXM28E: DO5 does not exist, so upper value of mask is 15
   {
      return VALUE_TOO_HIGH;
   }
   else if (lparam > 31LL)   // the drive is LXM26 or LXM28A: DO5 exists, so upper value of mask is 31
   {
      return VALUE_TOO_HIGH;
   }
   else
   {
      u16_Dig_Output_Force_Mask = (unsigned int)lparam;
   }

   return (SAL_SUCCESS);
}


// P4-28
int SalWriteDigOutputForceValue(long long lparam,int drive)
{
   drive += 0;

   if ( (u16_Flash_Type == 1) && (lparam > 15LL) ) // 2 * 64Mb flash, the drive is LXM28E: DO5 does not exist, so upper value of the value is 15
   {
      return VALUE_TOO_HIGH;
   }
   else if (lparam > 31LL)   // the drive is LXM26 or LXM28A: DO5 exists, so upper value of the value is 31
   {
      return VALUE_TOO_HIGH;
   }
   else
   {
      u16_Dig_Output_Force_Value = (unsigned int)lparam;
   }

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: ProcessDigitalOutputs
// Description:
//    This function is called to handle digital Outputs.
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
void ProcessDigitalOutputs(int drive)
{
   unsigned int i, u16_out_on, u16_mask, u16_local_inpos, u16_temp_time;
   long long fault_relay_close = 0LL, s64_temp, s64_local_pos_cmd_User;
   long s32_local_vel_var_fb_0;
   int  s16_cmd_val = 0, s16_local_stopped, s16_local_vel_var_fb;
   // AXIS_OFF;

   switch (BGVAR(u16_Relay_Mode))   // If fault exist open fault relay, otherwise - close
   {
      case 0:  // Relay opens whenever there is a fault
         if (u16_fault_control_delay > FAULT_CONTROL_DELAY)
         {
            if ((BGVAR(s64_SysNotOk) != 0) || (BGVAR(s64_SysNotOk_2) != 0))
               fault_relay_close = 0;      // open fault relay
            else
               fault_relay_close = 1;      // close fault relay
         }
      break;

      case 1:  // Relay opens whenever drive is not active
         fault_relay_close = (long long)VAR(AX0_s16_Active_Indication);
      break;
   }

   if (IS_HW_FUNC_ENABLED(FAULT_RELAY_MASK))
      *((unsigned int*)FPGA_FAULT_RELAY_REG_ADD) = (int)fault_relay_close;

   // do not change seven segments display & fault relay state until we allow enough time to
   // decide if faults exist or not
   if (u16_fault_control_delay <= FAULT_CONTROL_DELAY)
      u16_fault_control_delay++;

   do
   {// get RT variables value
      u16_temp_time = Cntr_3125; // the same RT Interrupt
      s64_local_pos_cmd_User = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
      s32_local_vel_var_fb_0 = LVAR(AX0_s32_Vel_Var_Fb_0);
      s16_local_stopped = VAR(AX0_s16_Stopped);
      s16_local_vel_var_fb = VAR(AX0_s16_Vel_Var_Fb_0);
      u16_local_inpos = VAR(AX0_u16_Inpos);
   } while (u16_temp_time != Cntr_3125);


   // Set outputs according to their functionality
   for (i = 0; i < (unsigned int)s16_Num_Of_Outputs; i++)
   {
      u16_out_on = 0;

      u16_mask = 1 << i;
      // check if the output is in overcurrent turn it off
      if( BGVAR(u16_Digital_Outputs_Overcurrent) & u16_mask)
      {// over current detected check the output state and if it's on set it to off
            if( BGVAR(u16_Digital_Outputs_Processed) & u16_mask )
            {// the output is on flip the output state bit to "0" or "1" depends on the polarity
                 u16_Out_State[i] ^= 1;
            }
      }

      if(((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)) &&
         (BGVAR(u16_Force_Output_Enable) == 1))
      {// drive in output force mode overide all outputs with P4-06 bits status.
         u16_out_on = (BGVAR(u16_ForcedOutputs) & u16_mask);
      }
      else
      {
         switch (u16_Out_Mode[i])
         {
            case ACTIVE_OUT:
               u16_out_on = (VAR(AX0_s16_Active_Indication) > 0);
            break;

            case BRAKE_OUT: // If drive enabled open brake relay
               if (!VAR(AX0_u16_BrakeOn))  u16_out_on = 1;
            break;

            case ALARM_OUT:
               u16_out_on = (BGVAR(s64_SysNotOk) != 0x00LL) || (BGVAR(s64_SysNotOk_2) != 0x00LL);
            break;

            case ALARM_AND_DISABLE_OUT:
               u16_out_on = (BGVAR(s64_SysNotOk) != 0x00LL) || (BGVAR(s64_SysNotOk_2) != 0x00LL) || (!Enabled(drive));
            break;

            case NO_FUNC_OUT:
               u16_out_on = u16_Out_State[i];
            break;

            case IN_POS_OUT: // If In-Position
               u16_out_on = (u16_local_inpos == 1);
            break;

            case STOPPED_OUT: // Stopped at the end of PTP Trajectory (for LXM it is called MC_OK)
               if((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))// the drive is Schneider drive and
               {
                  // leave the MCOK output in ON state no matter what is the real status
                  if (BGVAR(u16_P1_48_MCOK_Output_Selection) & 0x0f)
                  {
                     u16_out_on = ((VAR(AX0_s16_Opmode) == 8) && (s16_local_stopped == 2));
                  }
                  else
                  {
                     u16_out_on = ((VAR(AX0_s16_Opmode) == 8) && (u16_local_inpos == 1) && (s16_local_stopped == 2));
                  }
               }
               else
               {
                  if (0L != BGVAR(s32_Stopped_Output_Duration))
                  {
                     if (2 == VAR(AX0_s16_Stopped))
                     {
                        if ((PassedTimeMS(1, BGVAR(s32_Stopped_Armed_Timer))) && BGVAR(u16_Stopped_Flag_Armed))
                        {
                           u16_Stopped_flag = 1; 
                           BGVAR(s32_Stopped_Output_Duration_Timer) = Cntr_1mS;
                           BGVAR(u16_Stopped_Flag_Armed) = 0;
                        }
                     }
                     else
                        u16_Stopped_flag = 0;                  
                  
                     if (u16_Stopped_flag)
                     {
                        if (PassedTimeMS(BGVAR(s32_Stopped_Output_Duration), BGVAR(s32_Stopped_Output_Duration_Timer)))
                        {
                           u16_Stopped_flag = 0; 
                        }
                     }
                  }
                  else
                  {
                     u16_Stopped_flag = (2 == VAR(AX0_s16_Stopped));
                  }
                  u16_out_on = u16_Stopped_flag; 
               }
            break;

            case FOLDBACK_OUT: // Fold, Motor_Fold, Fold_Fault, or Motor_Fold_Fault
               u16_out_on = ( (BGVAR(u16_Fold) == 0x01) || (BGVAR(u16_Motor_Fold) == 0x01)                           ||
                              ((BGVAR(s64_SysNotOk) & (DRIVE_FOLDBACK_FLT_MASK | MOTOR_FOLDBACK_FLT_MASK)) != 0x00LL)  );
            break;

            case I_LEVEL_OUT: // If Actual Current is above OUTILVL1
               u16_out_on = (BGVAR(u32_EqCrrnt_Avg) > BGVAR(s32_Out_I_Level_1));
            break;

            case I_RANGE_OUT: // If Actual Current is above OUTILVL1 and below OUTILVL2
               u16_out_on = ( (BGVAR(u32_EqCrrnt_Avg) > BGVAR(s32_Out_I_Level_1)) &&
                              (BGVAR(u32_EqCrrnt_Avg) < BGVAR(s32_Out_I_Level_2))   );
            break;

            case V_LEVEL_OUT: // If Actual Velocity is above OUTVLVL1
               u16_out_on = (s32_local_vel_var_fb_0 > BGVAR(s32_Out_V_Level_1));
            break;

            case V_RANGE_OUT: // If Actual Velocity is above OUTVLVL1 and below OUTVLVL2
               u16_out_on = ( (s32_local_vel_var_fb_0 > BGVAR(s32_Out_V_Level_1)) &&
                              (s32_local_vel_var_fb_0 < BGVAR(s32_Out_V_Level_2))   );
            break;

            case P_LEVEL_OUT: // If Command Position is above OUTPLVL1
               u16_out_on = (s64_local_pos_cmd_User > BGVAR(s64_Out_P_Level_1));
            break;

            case P_RANGE_OUT: // If Command Position is above OUTPLVL1 and below OUTPLVL2
               u16_out_on = ( (s64_local_pos_cmd_User > BGVAR(s64_Out_P_Level_1)) &&
                              (s64_local_pos_cmd_User < BGVAR(s64_Out_P_Level_2))   );
            break;

            case BATTERY_LOW_FLT_OUT: // This indicates Low Battery Fault on Tamagawa 17Bit Enc
               u16_out_on = ((BGVAR(s64_SysNotOk) & ABS_ENC_BATT_LOW_MASK) != 0);
            break;

            case WARNING_OUT:
               if((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
               {// for LXM28/26 turn WRN output on only for LS, operational stop, serial communication error or under voltage error

                  u16_out_on = ((BGVAR(u64_Sys_Warnings) & (CW_HW_LS_WRN_MASK | CCW_HW_LS_WRN_MASK | MODBUS_NODEGUARD_WRN_MASK | UNDER_VOLTAGE_WRN_MASK)) ||
                                (BGVAR(s64_SysNotOk)     & (EMERGENCY_STOP_FLT_MASK | UNDER_VOLTAGE_FLT_MASK)));
               }
               else
               {
                  u16_out_on = (BGVAR(u64_Sys_Warnings) != 0x00);
               }
            break;

            case BATTERY_LOW_WRN_FLT_OUT: // This indicates Low Battery Warning on Tamagawa 17Bit Enc
               u16_out_on = ( ((BGVAR(u64_Sys_Warnings) & TM_BATT_LOW_VOLTAGE_WRN) != 0x00) ||
                              ((BGVAR(s64_SysNotOk) & ABS_ENC_BATT_LOW_MASK) != 0)            );
            break;

            case POST_WNS_OUT: // This indicates if Wake No Shake process succeeded or not
               PhaseFindStatusCommand(&s64_temp, drive); // 0 -idle. 1 - in proccess. 2-d one 3 - fault
               u16_out_on = (s64_temp == 2LL) ? 1 : 0;
            break;

            case OVR_C_FLT_OUT: // This indicates over current fault
               u16_out_on = ((BGVAR(s64_SysNotOk) & OVER_CRRNT_FLT_MASK) != 0x00LL);
            break;

            case OVR_V_FLT_OUT: // This indicates over voltage fault
               u16_out_on = ((BGVAR(s64_SysNotOk) & OVER_VOLTAGE_FLT_MASK) != 0x00LL);
            break;

            case UNDR_V_FLT_OUT: // This indicates under voltage fault
               u16_out_on = ((BGVAR(s64_SysNotOk) & UNDER_VOLTAGE_FLT_MASK) != 0x00LL);
            break;

            case PHASE_FIND_FLT_OUT: // This indicates phase find failure fault
               u16_out_on = ((BGVAR(s64_SysNotOk) & PHASE_FIND_FLT_MASK) != 0x00LL);
               u16_out_on |= ((BGVAR(u64_Sys_Warnings) & PHASE_FIND_REQ_WRN_MASK) != 0x00LL) ? 1 : 0;
            break;

            case AVOID_PHASE_FIND_FLT_OUT: // This indicates fault without dealing nphase find failure
              u16_out_on = ((BGVAR(s64_SysNotOk) & ~PHASE_FIND_FLT_MASK) != 0x00LL);
            break;

            case HOME_DONE_OUT: // This indicates home done
              u16_out_on = (BGVAR(u8_Homing_State) == HOMING_TARGET_REACHED);
            break;

            case Z_PULSE_OUT:
               if (i == (3 - 1)) // OUT 3
               {
                  u16_out_on = ((*(int*)FPGA_CONTROL_OUTPUT_REG_ADD & 0x4) ? 1 : 0);
               }
               else if (i == (6 - 1)) // OUT 6
               {
                  u16_out_on = ((*(int*)FPGA_MACHINE_OUTPUT_REG_ADD & 0x4) ? 1 : 0);
               }
               /*else if (i == (FAST_OUTPUT_1 - 1)) // Fast OUT 1
               {
                  u16_out_on = ((*(int*)FPGA_WAITING_FOR_BARUCH7 & 0x4) ? 1 : 0);
               }
               else if (i == (FAST_OUTPUT_2 - 1)) // Fast OUT 2
               {
                  u16_out_on = ((*(int*)FPGA_WAITING_FOR_BARUCH8 & 0x4) ? 1 : 0);
               }*/
               // This is a special case where if OUTINV is used that the output value and and INV value are both
               // changed - to solve this, the following line is added
               if (BGVAR(u16_Dig_Out_Polar) & (0x1<<i))
                  u16_out_on = 1 - u16_out_on;
            break;

            case ZERO_POSITION_OUT: // If ZERO position after homing
               u16_out_on = (((BGVAR(u8_Homing_State) == HOMING_TARGET_REACHED)) && VAR(AX0_u16_Inpos));            
            break;
            
            case PCMD_OUT:
               u16_out_on = (0L != LVAR(AX0_s32_Pos_Vcmd)); 
            break;            

            case PCOM1_OUT:
               //set by FPGA
            break;        

            case PCOM2_OUT:
               //set by FPGA
            break;        

            // Schneider added output functionalities
            // SRDY, SON, ZSPD, TSPD, TQL, ALRM, BRKR, HOME, OLW and WARN are reflected respectively from u16_P0_46_SVSTS bits.
                                         // CDHD | Schneider
            case SRDY:                    //  25   0x01
               u16_out_on = (BGVAR(u16_P0_46_SVSTS) & 0x01);
            break;

            case ZSPD:                   //  26   0x03
               u16_out_on = ((BGVAR(u16_P0_46_SVSTS) & 0x04) >> 2);
            break;

            case TSPD:                   //  27   0x04
               u16_out_on = ((BGVAR(u16_P0_46_SVSTS) & 0x08) >> 3);
            break;

            case TPOS:                  //   28   0x05

               // TPOS will be updated from RT
               u16_out_on = (BGVAR(u16_P0_46_SVSTS) & 0x0010);
            break;

            case TQL:                  //   29   0x06
               u16_out_on = ((BGVAR(u16_P0_46_SVSTS) & 0x020) >> 5);
            break;

            case OLW:                    //  30   0x10
               u16_out_on = ((BGVAR(u16_P0_46_SVSTS) & 0x0200) >> 9);
            break;

            case OVF:                    //  31   0x12
            break;

            case SNL:                    //  32   0x13
               u16_out_on = (VAR(AX0_u16_CCW_LS) & 0x1);
            break;

            case SPL:                    //  33   0x14
               u16_out_on = (VAR(AX0_u16_CW_LS) & 0x1);
            break;

            case CMD_OK:                 //  34   0x15
               if(VAR(AX0_s16_Opmode) == 8)
                  u16_out_on = (s16_local_stopped > 0); // PS mode
            break;

            case CAP_OK:                 //  35   0x16
               if(VAR(AX0_s16_Opmode) == 8)
                  u16_out_on = (((VAR(AX0_u16_TProbe_Cmnd) & 0x2) == 0) && ((VAR(AX0_u16_TProbe_Status) & 0x6) != 0))?1:0;
               else
                  u16_out_on = 0;
            break;

            case CAP_2_OK:                 //  0x18
               if(VAR(AX0_s16_Opmode) == 8)
                  u16_out_on = (((VAR(AX0_u16_TProbe_Cmnd) & 0x200) == 0) && ((VAR(AX0_u16_TProbe_Status) & 0x600) != 0))?1:0;
               else
                  u16_out_on = 0;
            break;

            case SP_OK:                //  36   0x19
               if(((VAR(AX0_s16_Opmode) == 1) || (VAR(AX0_s16_Opmode) == 0)) && (BGVAR(s32_P1_47_SPOK_In_Loop) !=0) && Enabled(drive) ) // speed or speed zero opmodes
               {
                  s16_cmd_val=*(int*)((VAR(AX0_s16_Acc_Dec_Cmnd_Ptr) & 0xFFFF));
                  u16_out_on =(abs(s16_cmd_val - s16_local_vel_var_fb) <=  BGVAR(s32_P1_47_SPOK_In_Loop));
               }
            break;

           // P parameter controlled outputs: mode 0x30 to 0x3F
            /*
            // Fix IPR 1153: Signal Output Functions 30h - 37h has no effect
            // it was decided to cancel inmodes SDO_8 to SDO_F
            case SDO_F_OUT:
            case SDO_E_OUT:
            case SDO_D_OUT:
            case SDO_C_OUT:
            case SDO_B_OUT:
            case SDO_A_OUT:
            case SDO_9_OUT:
            case SDO_8_OUT:
            */
            case SDO_7_OUT:
            case SDO_6_OUT:
            case SDO_5_OUT:
            case SDO_4_OUT:
            case SDO_3_OUT:
            case SDO_2_OUT:
            case SDO_1_OUT:
            case SDO_0_OUT:
               u16_out_on = (BGVAR(u16_ForcedOutputs) >> (u16_Out_Mode[i] - SDO_0_OUT)) & 0x0001;
            break;

            default:
               u16_out_on = 0;
            break;
         }
      }

      if (u16_out_on)
         u16_Digital_Outputs |= u16_mask;
      else
         u16_Digital_Outputs &= ~u16_mask;
   }
   if (!u16_Out_Brake_Mode) //manual mode
      VAR(AX0_u16_BrakeOn) = BGVAR(u16_Manual_Out_Brake_Value);
}


//**********************************************************
// Function Name: UpdateDigitalOutputsOvercurrent
// Description:
//    This function is called to updatethe digital Outputs overcurrent condition
//
// Author: Moshe Artzi
// Algorithm:
// Revisions:
//**********************************************************
void UpdateDigitalOutputsOvercurrent(int drive)
{
   unsigned int u16_FPGA_value = *(unsigned int *)FPGA_OC_DIG_OUT_REG_ADD;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // get only the available output overcurrent indications

   BGVAR(u16_Digital_Outputs_Overcurrent) = (u16_FPGA_value & BGVAR(u16_Digital_Outputs_Overcurrent_Mask));
}


void UpdateDigitalOutputOvercurrentMask(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (IS_EC_LITE_HV_DRIVE)
      BGVAR(u16_Digital_Outputs_Overcurrent_Mask) = u16_HW_Supported_Dig_Outputs_Mask; // DO 1,2,3,7 since DO 4 is actually DO7 in HW and considered DO4 in SW
   else if (IS_DDHD_EC2_DRIVE)
      BGVAR(u16_Digital_Outputs_Overcurrent_Mask) = 0; // No Over-Current Detection Circuits in DDHD-EC
   else
      BGVAR(u16_Digital_Outputs_Overcurrent_Mask) = u16_Supported_Dig_Outputs_Mask;
}


void ScanDigitalOutputs()
{
   unsigned int i, u16_temp_time=0, u16_temp_force;
   unsigned long u32_temp_flt_msk;

   int drive = 0;
   // AXIS_OFF;

   // update the digital output overcurrent condition
   UpdateDigitalOutputsOvercurrent(drive);

   ProcessDigitalOutputs(0);

   // Structure of u16_Digital_Outputs:
   // OUT  bit  signal
   // ---  ---  -------------
   //  1    0   c_gpout_1
   //  2    1   c_gpout_2
   //  3    2   c_fast_out1
   //  4    3   m_gpout_3
   //  5    4   m_gpout_4
   //  6    5   m_fast_out2
   //  7    6   Pwr_Brk_Cntrl

   BGVAR(u16_Digital_Outputs_Processed) = (u16_Digital_Outputs) ^ BGVAR(u16_Dig_Out_Polar);

   if((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
   {// not LXM drive
      // Implementation of OUTFLTLVL
      for (i = 0; i < (unsigned int)s16_Num_Of_Outputs; i++)
      {
         // If a fault is pending and the Drive is disabled
         if (((BGVAR(s64_SysNotOk)  != 0) || (BGVAR(s64_SysNotOk_2) != 0)) && (!Enabled(drive)))
         {
            u32_temp_flt_msk = 1L << ((2 * i) + 1); // Check bit 1, 3, 5, 7...
            if(BGVAR(u32_Digital_Outputs_After_Fault) & u32_temp_flt_msk) // If the digital output needs to be forced to a value
            {
               // At this point we are supposed to force a digital output to become either true or false
               u32_temp_flt_msk = 1L << (2 * i);
               if(BGVAR(u32_Digital_Outputs_After_Fault) & u32_temp_flt_msk) // If output is supposed to be forced to true
               {
                  u32_temp_flt_msk = 1L << i;
                  BGVAR(u16_Digital_Outputs_Processed) |= (unsigned int)u32_temp_flt_msk;
               }
               else  // If output is supposed to be forced to false
               {
                  u32_temp_flt_msk = 1L << i;
                  BGVAR(u16_Digital_Outputs_Processed) &= (unsigned int)(~u32_temp_flt_msk);
               }
            }
         }
      }
   }
   else
   {// LXM drive
      // If SE drive && AUTOR mode is on && in position mode, overwrite the outputs.
      if(((BGVAR(u16_P2_44_Autor_DoModeSet) & 0x01) == 1) &&
          (BGVAR(u16_LXM28_Opmode) == SE_OPMODE_PS))// P2-44 is ON in position mode and it's a Lexium drive - AUTOR output mode is enable
      {// If AUTOR special output mode is enable, force the outputs according to AUTOR feature.

         if(BGVAR(s16_DisableInputs) & 0xFFFC)  // If something prevents the Drive from getting enabled except the HW and SW enable (what the Lexium usually uses).
         {
            // State that the Drive is not ready
            BGVAR(u16_AutoR_OutMode) = AUTOR_OUT_MODE_ALARM;
         }
         else if(BGVAR(u16_AutoR_OutMode) == AUTOR_OUT_MODE_ALARM)
         {
            // If drive was in fault and recover from it set it to drive ready.
            // State that the Drive is ready to get enabled
            BGVAR(u16_AutoR_OutMode) = AUTOR_OUT_MODE_SERVO_READY;
         }

         switch(BGVAR(u16_AutoR_OutMode))
         {
            case AUTOR_OUT_MODE_ALARM:
            case AUTOR_OUT_MODE_SERVO_READY:
            case AUTOR_OUT_MODE_HOMING_RUNNING:
            case AUTOR_OUT_MODE_HOMING_COMPLETED:
            case AUTOR_OUT_MODE_CHANGING_INDEX:
               BGVAR(u16_Digital_Outputs) = BGVAR(u16_AutoR_OutMode);
               break;

            case AUTOR_OUT_MODE_INDEX_IN_POS:
               BGVAR(u16_Digital_Outputs) = (BGVAR(s16_Running_Path_Index) + 4);
               break;

            default:
               break;
         }

         BGVAR(u16_Digital_Outputs_Processed) = BGVAR(u16_Digital_Outputs);
      }

      // Implement forced digital output for SE (P4-26, P4-27, and P4-28)
      u16_temp_force = BGVAR(u16_Forceable_Dig_Outputs) & BGVAR(u16_Dig_Output_Force_Mask);          // isolate to outputs to force
      BGVAR(u16_Digital_Outputs_Processed) &= ~u16_temp_force;                                       // clear all forced bits
      BGVAR(u16_Digital_Outputs_Processed) |= (BGVAR(u16_Dig_Output_Force_Value) & u16_temp_force);  // set forced bits with force value of 1
   }

   // copy the current BG outputs into the RT variable to handle RT outputs.
   VAR(AX0_u16_Output_Control_Machine) = BGVAR(u16_Digital_Outputs_Processed);

   do
   {// update u16_Digital_Outputs_Processed BG variable after RT to make sure the RT data wont change while copying the data
     u16_temp_time = Cntr_3125; // the same RT Interrupt
     BGVAR(u16_Digital_Outputs_Processed) = VAR(AX0_u16_Output_Control_Machine);
   } while (u16_temp_time != Cntr_3125);
}

// nitsan: assume this function is executed from background context only.
// if moved to interrupt, need to update first param in writeEmcyReq() call
void ProcessClrFaultInput(int drive)
{
   static unsigned int u16_prev_clr_fault;
   int s16_value;
   unsigned int u16_tmp;

   // Check if access-rights are given. In case of a CDHD Drive (no Lexium),
   // the following function call returns always SAL_SUCCESS
   if(CheckLexiumAccessRights(drive, LEX_AR_FCT_FLT_RESET, LEX_CH_IO_CONTROL) != SAL_SUCCESS)
   {
      // Leave function since no access-rights for clearing
      // a fault via digital input are given to the Lexium.
      return;
   }

   if (s16_Clr_Flt_In < 0) return; //no input associated with clr fault funaction

   //Process Clear faults input, detect rising edge of the input from 0 to 1
   s16_value = InxValue(s16_Clr_Flt_In, drive);
   if ((s16_value == 1) && (u16_prev_clr_fault == 0)) // If rising edge on a clearfault input
   {
      // For a Lexium product
      if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      {

	     //clear faults
		 ClearFaultsCommand(0);

         // Call the function for Schneider that clears the fault
         //SalClearAlarmViaPParam(0, drive);

         if (Enabled(DRIVE_PARAM))
         {
            u16_tmp = p402_statusword;
            u16_tmp &= ~ (PPM_TARGET_REACHED | PPM_SET_POINT_ACK | STATUSWORD_STATE); // same as TempStatusWord &= ~HM_ATTAINED;
            u16_tmp |=  OPERATION_ENABLED;
            p402_statusword = u16_tmp;
         }
         else
         {
            u16_tmp = p402_statusword;
            u16_tmp &= ~(STATUSWORD_STATE | FOLLOWING_ERROR_BIT); //(For position mode, homing...)
            u16_tmp |= SWITCH_ON_DISABLED;
            p402_statusword = u16_tmp;
         }

         // if no faults detected and no LS alarm, fire EMCY to indicate no alarms.
         if ( (BGVAR(s64_SysNotOk) == 0) && (BGVAR(s64_SysNotOk_2) == 0) && (BGVAR(u8_Show_Lex_LS_Warning) == LEX_LS_NO_WARNING) )
         {
             p301_error_register = 0;
             writeEmcyReq(BACKGROUND_CONTEXT, 0,(UNSIGNED8 *)"\0\0\0\0\0" CO_COMMA_LINE_PARA);
         }
      }
      // Do here things for the regular CDHD product
      else if ( ((BGVAR(s64_SysNotOk) != 0) || (BGVAR(s64_SysNotOk_2) != 0))  &&
           ((BGVAR(s16_DisableInputs) & (HW_EN_MASK | SW_EN_MASK)) != 0 )  )
      {
         FaultsHandler(drive, FLT_CLR);
      }
   }
   u16_prev_clr_fault = s16_value;
}


void ProcessHomingCmdInput(int drive)
{
   static unsigned int u16_prev_Homing_Cmd;
   unsigned int u16_Homing_Cmd, u16_start_homing = 1;
   // AXIS_OFF;

   // Process Homing Command input, detect any change
   u16_Homing_Cmd = (VAR(AX0_u16_Mode_Input_Arr[HOMING_CMD_INP]) != 0);

   if ((u16_Homing_Cmd == 1) && (u16_prev_Homing_Cmd == 0))
   { // If Rising Edge (0 to 1) issue HOMECMD with no parameters to start Homing

      // nitsan: in schneider drive this dig input is relevant only if in PS mode (opmode 8)
      if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      {
         u16_start_homing = (VAR(AX0_s16_Opmode) == 8);
      }

      if (u16_start_homing)
      {
         // start homing
         if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
         {
            // execute homing using the Path Handler State Machine
            // check if homing is not in final state reset the homing process
            if (IsHomingProcessRunning(drive))
            {// reset the homing process
               STORE_EXECUTION_PARAM_0
               s16_Number_Of_Parameters = 1;
               s64_Execution_Parameter[0] = 0;
               HomeCommand(drive);
               RESTORE_EXECUTION_PARAM_0
               // check if running if so reset the path state machine
               if((VAR(AX0_s16_Stopped) != 2) || (BGVAR(u16_Path_Exec_State) != PATH_STATE_MACHINE_IDLE))
               {// stop the PTP generator
                  PtpAbort(drive);
                  // set the state machine to idle
                  BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE;
               }
            }
            else
            {// execute the homing process
               BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE;
               BGVAR(s16_Path_Internal_Execution) = 0;
               BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;
            }
         }
         else
         {// CDHD home execute
            s16_Number_Of_Parameters = 0;
            HomeCommand(drive);
         }
      }
   }
   else if ( (u16_Homing_Cmd == 0) && (u16_prev_Homing_Cmd == 1) &&
             (BGVAR(u8_Homing_State) != HOMING_TARGET_REACHED)     )
   {  // If Falling Edge (1 to 0) issue "HOMECMD 0" to abort Homing
      // Avoid resetting Homing State Machine if successful, to maintain "Homed" indication.

      // nitsan: in schneider drive we dont abort homing on falling edge
      if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
      {
         STORE_EXECUTION_PARAM_0
         s16_Number_Of_Parameters = 1;
         s64_Execution_Parameter[0] = 0;
         HomeCommand(drive);
         RESTORE_EXECUTION_PARAM_0
      }
   }

   u16_prev_Homing_Cmd = u16_Homing_Cmd;
}

int s16_done = 0;
void ProcessDecOnFallingEdge(int drive)
{
   // AXIS_OFF;
   static unsigned char u8_f_state=0;

   unsigned int u16_decfromsig_in = 0;

   //*(int *)FPGA_CAPTURE_FLAG_REG_ADD;
   //check if function is configured to an input
   if(u16_decfromsig_in = IsInFunctionalityConfigured(DEC_ON_INPUT, drive))
   {
      switch(u8_f_state)      //state machine handler
      {
         case 0:         //this state arms the capture mechanizm
         {
            int s16_inx_value;

            s16_inx_value = InxValue(u16_decfromsig_in, drive);   // capture input status
           
            if (s16_inx_value == 1) 
               s16_done = 1;
            else
               return;
            
            if(s16_done == 0) return;

            //arm capture mechanism
            *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = u16_decfromsig_in; // Trigger Source
            *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_REG_ADD = 0; // falling Edge

           // VAR(AX0_s16_Crrnt_Run_Code) |= ARM_HOMING_CAPTURE_MASK; // arm the capture (execute capture continously);
           // VAR(AX0_s16_dec_fall_capture) = 1;

            u8_f_state = 1;            //go to next state waiting for cpture
            break;
         }
         
         case 1:
         {
            VAR(AX0_s16_dec_fall_capture) = 1;
            u8_f_state = 2;
            break;
         }
         
         

         case 2:         //this state waits for the capture to happen.
         {     
            // In case there's no change in PCMD avoid the DECDIST feature
            if (LVAR(AX0_s32_Delta_Pos_Cmd)==0)
            {
               u8_f_state = 0;
               break;
            }
            //look for capture flag
            //if (!(VAR(AX0_s16_Crrnt_Run_Code) & ARM_HOMING_CAPTURE_MASK))
            if (!(VAR(AX0_s16_dec_fall_capture)))
            {
               unsigned long  u32_dec_val1, u32_acc_rounded, u32_t_dec = 0;//, u32_dec_rounded;
               long s32_vel1 = 0;
               long long temp_pos, s64_Capt_Pos, s64_New_Target_Pos;

               //set the acc/dec units for usage
               u32_acc_rounded =  (long)((BGVAR(u64_AccRate) + (unsigned long long)0x080000000) >> 32);
               //u32_dec_rounded =  (long)((BGVAR(u64_DecRate) + (unsigned long long)0x080000000) >> 32);

               //calculate the captured position
               // Captured_Position = Prev_Captured_PFB + (Captured_PFB - Prev_Captured_PFB) * Capture_Time / 1875.
               s64_Capt_Pos = ((long long)(LVAR(AX0_s32_Prev_Captured_Pfb_Hi) - LVAR(AX0_s32_Captured_Pfb_Hi)) << 32) +
                              LVAR(AX0_u32_Prev_Captured_Pfb_Lo) - LVAR(AX0_u32_Captured_Pfb_Lo);
               s64_Capt_Pos = (long long)(s64_Capt_Pos * VAR(AX0_u16_Home_Capture_Time) / 1875) -
                              ((long long)LVAR(AX0_s32_Prev_Captured_Pfb_Hi) << 32) - LVAR(AX0_u32_Prev_Captured_Pfb_Lo);
               s64_Capt_Pos = -s64_Capt_Pos;
               s64_New_Target_Pos=(long long)(s64_Capt_Pos + BGVAR(s64_Dec_Dist));

               //use current velocity in ptp move. if negative need to invert sign since value is taken from velocity loop.
               //(in position PTP command velocity is unsigned)
               s32_vel1 = ( LVAR(AX0_s32_Vel_Var_Fb_0) > 0 ) ? (LVAR(AX0_s32_Vel_Var_Fb_0)) : -(LVAR(AX0_s32_Vel_Var_Fb_0));
               s32_vel1 <<= 1;
               temp_pos = ( BGVAR(s64_Dec_Dist) > 0 ) ? (BGVAR(s64_Dec_Dist)) : -(BGVAR(s64_Dec_Dist));

               //calculate deceleration so that it will start roughly at this point in the PTP change
               //P = ( Tdec * V ) / 2  =>  Tdec = 2P / V.
               u32_t_dec = (2 * temp_pos) / s32_vel1;
               u32_dec_val1 = 2 * (s32_vel1 / u32_t_dec);
               ///enter target position from capture
               BGVAR(s16_Move_Abs_Issued) = 0;                                                     //u32_dec_rounded
               InitiatePtpMove(drive, (long long)(s64_New_Target_Pos), s32_vel1, u32_acc_rounded, u32_dec_val1, PTP_IMMEDIATE_TRANSITION);

               s16_done = 1;
               u8_f_state = 0;
            }

            break;
         }

      }
   }
   else
   {
      u8_f_state = 0;   //clear state
      //clear capture mechanism if active
   }
}


int s16_done2 = 0;
void ProcessDecOnFallingEdge2(int drive)
{
   // AXIS_OFF;
   static unsigned char u8_f_state2 = 0;

   unsigned int u16_decfromsig_in = 0;

   //check if function is configured to an input
   if(u16_decfromsig_in = IsInFunctionalityConfigured(DEC_ON_INPUT2, drive))
   {
      switch(u8_f_state2)   //state machine handler
      {
         case 0:         //this state arms the capture mechanizm
         {
            int s16_inx_value;

            //capture input status
            s16_inx_value = InxValue(u16_decfromsig_in, drive);
            //do not continue if input has not gone low yet or if input is low
            if (s16_inx_value == 1) 
               s16_done2 = 1;
            else
               return;
            
            if(s16_done2 == 0) return;

            //arm capture mechanism
            *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = u16_decfromsig_in; // Trigger Source
            *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_REG_ADD = 0; // falling Edge

          //  VAR(AX0_s16_Crrnt_Run_Code) |= ARM_HOMING_CAPTURE_MASK; // arm the capture (execute capture continously); 
          //  VAR(AX0_s16_dec_fall_capture) = 1;

            u8_f_state2=1;            //go to next state waiting for cpture
            break;
         }
         
         case 1:
         {
            VAR(AX0_s16_dec_fall_capture) = 1;
            u8_f_state2 = 2;
            break;
         }

         case 2:         //this state waits for the capture to happen.
         {
            // In case there's no change in PCMD avoid the DECDIST feature
            //if (LVAR(AX0_s32_Delta_Pos_Cmd) == 0)
            if (LVAR(AX0_s32_Delta_Pos_Cmd)==0)
            {
               u8_f_state2 = 0;
               break;
            }
            //look for capture flag           
            if (!(VAR(AX0_s16_dec_fall_capture)))
            {
               unsigned long u32_dec_val1, u32_acc_rounded, u32_t_dec = 0;//, u32_dec_rounded;
               long s32_vel1 = 0;
               long long temp_pos, s64_Capt_Pos, s64_New_Target_Pos;

               //set the acc/dec units for usage
               u32_acc_rounded =  (long)((BGVAR(u64_AccRate) + (unsigned long long)0x080000000) >> 32);
               //u32_dec_rounded =  (long)((BGVAR(u64_DecRate) + (unsigned long long)0x080000000) >> 32);

               //calculate the captured position
               // Captured_Position = Prev_Captured_PFB + (Captured_PFB - Prev_Captured_PFB) * Capture_Time / 1875.
               s64_Capt_Pos = ((long long)(LVAR(AX0_s32_Prev_Captured_Pfb_Hi) - LVAR(AX0_s32_Captured_Pfb_Hi)) << 32) +
                          LVAR(AX0_u32_Prev_Captured_Pfb_Lo) - LVAR(AX0_u32_Captured_Pfb_Lo);
               s64_Capt_Pos = (long long)(s64_Capt_Pos * VAR(AX0_u16_Home_Capture_Time) / 1875) -
                           ((long long)LVAR(AX0_s32_Prev_Captured_Pfb_Hi) << 32) - LVAR(AX0_u32_Prev_Captured_Pfb_Lo);
               s64_Capt_Pos = -s64_Capt_Pos;
               s64_New_Target_Pos=(long long)(s64_Capt_Pos + BGVAR(s64_Dec_Dist2));

               //use current velocity in ptp move. if negative need to invert sign since value is taken from velocity loop.
               //(in position PTP command velocity is unsigned)
               s32_vel1 = ( LVAR(AX0_s32_Vel_Var_Fb_0) > 0 ) ? (LVAR(AX0_s32_Vel_Var_Fb_0)) : -(LVAR(AX0_s32_Vel_Var_Fb_0));
               s32_vel1 <<= 1;
               temp_pos = ( BGVAR(s64_Dec_Dist2) > 0 ) ? (BGVAR(s64_Dec_Dist2)) : -(BGVAR(s64_Dec_Dist2));

               //calculate deceleration so that it will start roughly at this point in the PTP change
               //P = ( Tdec * V ) / 2  =>  Tdec = 2P / V.
               u32_t_dec = (2 * temp_pos) /  s32_vel1;
               u32_dec_val1 = 2 * (s32_vel1 / u32_t_dec);

               ///enter target position from capture
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, (long long)(s64_New_Target_Pos), s32_vel1, u32_acc_rounded, u32_dec_val1/*u32_dec_rounded*/, PTP_IMMEDIATE_TRANSITION);

               s16_done2 = 1;
               u8_f_state2 = 0;
            }
            break;
         }

      }
   }
   else
   {   
      u8_f_state2 = 0;      //clear state
      //clear capture mechanism if active
   }
}


void ProcessDigitalInputs(int drive)
{
   // this func must be called before handling torque, vel, PTP command asignments by inputs,
   // since it set/reset allow_motion flag when brake is released and motino should be ignored for a certain time interval.
   BrakeRelease(drive);
   ProcessSonInput(drive);
   ProcessClrFaultInput(drive);
   ProcessHomingCmdInput(drive);
   ProcessDecOnFallingEdge(drive);
   ProcessDecOnFallingEdge2(drive);
   ProcessGearCclr(drive);
   ProcessJogInputs(drive);
   ProcessTorqueCommand(drive);
   ProcessVelocityCommand(drive); // Velocity command or limit from speed selection table
   ProcessZClamp(drive);
   ProcessZclampFunction(drive);
   ProcessCmdInvert(drive);
   ProcessGainSwitch(drive);
   ProcessStopInput(drive);
   ProcessGearInNumerators(drive);
   ProcessPtcmsInput(drive);
   ProcessDualModeInput(drive);
   ProcessHaltInput(drive);
   ProcessPauseInput(drive);
   ProcessOpmodeSwitchInput(drive);
   ProcessScriptTriggerInput(drive);
   ProcessSfbPfbComparisonEnable(drive);
   ProcessEncoderFollowerInputs(drive);
   ProcessUserJogModesInputs(drive);
   ProcessPcomInputs(drive);
}


//**********************************************************
// Function Name: ScanDigitalInputs
// Description:
//    This function is called to scan the digital Inputs.
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************

void ScanDigitalInputs(int drive)
{
   // AXIS_OFF;
   unsigned int u16_Path_Trigger_Mode_Flag = 0,u16_Index;

   //u16_temp_control = *(int*)FPGA_CONTROL_INPUT_STATUS_REG_ADD;
   //u16_temp_machine = *(int*)FPGA_MACHINE_INPUT_REG_ADD;
   //u16_temp_remote  = *(int*)FPGA_REMOTE_ENABLE_REG_ADD;

   // Structure of u16_Digital_Inputs:
   //  IN  bit  signal
   // ---  ---  ---------------
   //  1    0   Enable (remote)
   //  2    1   c_gpin_1
   //  3    2   c_gpin_2
   //  4    3   c_gpin_3
   //  5    4   c_fast_in_1
   //  6    5   c_fast_in_2
   //  7    6   m_gpin_4
   //  8    7   m_gpin_5
   //  9    8   m_gpin_6
   // 10    9   m_gpin_7
   // 11   10   m_fast_in_3

   ProcessDigitalInputs(drive);

   for (u16_Index = 1; u16_Index <= (unsigned int)s16_Num_Of_Inputs; u16_Index++)
   {  // look for script trigger assign
      if (VAR(AX0_u16_Input_Mode_Arr[u16_Index]) == PATH_CTRG_INP)
         u16_Path_Trigger_Mode_Flag++;
   }

   if(u16_Path_Trigger_Mode_Flag)
   {
      BGVAR(s16_Path_Triggered) = isPathTriggered(drive);
      if(BGVAR(s16_Path_Triggered))
      {// handle pathes selection for pathhendler
         BGVAR(s16_Selected_Path_Index) = getSelectedPathIndex(drive);
      }
   }
}


//**********************************************************
// Function Name: ScriptHandler
// Description:
//    This Handler deals with the script at execution time
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
void ScriptHandler(int drive)
{
   char Pre_Script[] = "Start:";
   char Post_Script[] = "Stop:";

   if (BGVAR(s16_Script_Edge_State) != 0 && BGVAR(u16_Script_Handler_State) == 0)
   {
      if (BGVAR(s16_Script_Edge_State) == 1)
      {
          BGVAR(s16_Script_Executed_Edge) = 1;
          BGVAR(s16_Script_Executed_Command) = BGVAR(u16_Script_Rise_cmd);
      }
      else
      {
          BGVAR(s16_Script_Executed_Edge) = 0;
          BGVAR(s16_Script_Executed_Command) = BGVAR(u16_Script_Fall_cmd);
      }

      BGVAR(s16_Script_Edge_State) = 0;
      BGVAR(u16_Script_Handler_State) = 1;
      BGVAR(u8_Execution_String_Script)[0] = 0;
      BGVAR(u16_Script_Str_Cntr) = 0;

      BGVAR(u8_Script_Status_Str)[0] = 0;

      for (BGVAR(u16_Script_Str_Cntr) = 0; BGVAR(u16_Script_Str_Cntr) <= 19; BGVAR(u16_Script_Str_Cntr)++)
         BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_Str_Cntr)] = u8_Script_Header[BGVAR(u16_Script_Str_Cntr)];
      for (; BGVAR(u16_Script_Str_Cntr) <= 25; BGVAR(u16_Script_Str_Cntr)++)
         BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_Str_Cntr)] = Pre_Script[BGVAR(u16_Script_Str_Cntr)-20];

      BGVAR(u16_Script_Str_Cntr)= 26;
      BGVAR(u8_Script_Status_Str)[26] = 0;
      BGVAR(u8_Script_Status_Str)[17] = BGVAR(s16_Script_Executed_Edge) + '0';
      BGVAR(u8_Script_Status_Str)[13] = BGVAR(s16_Script_Executed_Command)/10 + '0';
      BGVAR(u8_Script_Status_Str)[14] = (BGVAR(s16_Script_Executed_Command)-10*(BGVAR(u8_Script_Status_Str)[13]-'0')) + '0';
   }

   //state 0 ->Ignore
   //state 1 ->Begin and load the script
   //state 2 ->load each of the sub commands & Execute the sub command
   switch (BGVAR(u16_Script_Handler_State))
   {
      case 0:
         BGVAR(u16_Script_Exec_State) = 0;
      break;

      case 1:
     {
         BGVAR(u16_Script_Exec_State) = 1;
         //////
         ExpandString(BGVAR(s16_Script_Executed_Command), BGVAR(s16_Script_Executed_Edge), &BGVAR(u8_Execution_String_Script)[0], drive);

         // Bug-fix for Bugzilla 4608. An ASCII-command which is just one letter (like e.g. A, K or L)
         // was not executed since BGVAR(u16_Script_J_index) holds a value of 0 due to the previous
         // for-loop and therefore no script was being processed in stage 2 of this function.
         if ((BGVAR(u8_Execution_String_Script)[0] >= 'A') && (BGVAR(u8_Execution_String_Script)[0] <= 'Z') &&
             (BGVAR(u8_Execution_String_Script)[1] == 0))
         {
            // Here we identified a one character ASCII-command and therefore we add manually
            // a ~ sign to the execution string in order to process the script in the next stage.
            BGVAR(u8_Execution_String_Script)[1] = '~'; // Add ~ after the one letter command
            BGVAR(u8_Execution_String_Script)[2] = 0;   // Zero terminate the string
         }

         ///////////////////////////////////////////////
         // measuring the string length
         ///////////////////////////////////////////////
         for (BGVAR(u16_Script_I_index) = 0; BGVAR(u8_Execution_String_Script)[BGVAR(u16_Script_I_index)] != 0; BGVAR(u16_Script_I_index)++)
            BGVAR(u16_Script_J_index) = BGVAR(u16_Script_I_index);
         BGVAR(u16_Script_I_index) = 0;
         BGVAR(u16_Script_Handler_State) = 2;
         break;
      }

      case 2:
      {
         ///////////////////////////////////////////////
         if (BGVAR(u16_Script_I_index) < BGVAR(u16_Script_J_index) && s16_Comms_Processor_State == PRE_PROCESSOR)
         {
            BGVAR(u16_Script_K_index) = BGVAR(u16_Script_I_index);
            ///////////////////////////////////////////////
            //CHECKING EACH STRING BETWEEN ~ OR TILL NULL
            ///////////////////////////////////////////////
            while (BGVAR(u8_Execution_String_Script)[BGVAR(u16_Script_K_index)] != '~' && BGVAR(u16_Script_K_index) <= BGVAR(u16_Script_J_index) )
               BGVAR(u16_Script_K_index)++;

            BGVAR(u16_Script_ParamCount) = strlen(BGVAR(u8_Execution_String_Script));
            BGVAR(u16_Script_ParamCount) = BGVAR(u16_Script_K_index);

            for (BGVAR(u16_Script_L_index) = 0; BGVAR(u16_Script_L_index) < (BGVAR(u16_Script_K_index) - BGVAR(u16_Script_I_index)); BGVAR(u16_Script_L_index)++)
            {
               u8_Message_Buffer[BGVAR(u16_Script_L_index)] = BGVAR(u8_Execution_String_Script)[BGVAR(u16_Script_I_index) + BGVAR(u16_Script_L_index)];
               BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_Str_Cntr)++] = u8_Message_Buffer[BGVAR(u16_Script_L_index)];
            }

            BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_Str_Cntr)++] = ':';
            BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_Str_Cntr)] = 0;

            u8_Message_Buffer[BGVAR(u16_Script_L_index)] = 0;

            p_u8_Message_Buffer_Ptr = u8_Message_Buffer;
            u8_Parser_State = PARSE_MNEMONIC;
            s16_Comms_Processor_State = PARSING;
            BGVAR(u16_Script_I_index) = BGVAR(u16_Script_K_index) + 1;
         }

         if (BGVAR(u16_Script_I_index) >= BGVAR(u16_Script_J_index))
         {
            BGVAR(u16_Script_I_index) = BGVAR(u16_Script_Str_Cntr);

            for (; BGVAR(u16_Script_Str_Cntr) <= (BGVAR(u16_Script_I_index) + 19); BGVAR(u16_Script_Str_Cntr)++)
               BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_Str_Cntr)] = u8_Script_Header[BGVAR(u16_Script_Str_Cntr) - BGVAR(u16_Script_I_index)];
            for (; BGVAR(u16_Script_Str_Cntr) <= (BGVAR(u16_Script_I_index) + 24); BGVAR(u16_Script_Str_Cntr)++)
               BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_Str_Cntr)] = Post_Script[BGVAR(u16_Script_Str_Cntr) - BGVAR(u16_Script_I_index) - 20];

            BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_Str_Cntr)] = 0;
            BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_I_index) + 17] = BGVAR(s16_Script_Executed_Edge) + '0';
            BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_I_index) + 13] = (BGVAR(s16_Script_Executed_Command)) / 10 + '0';
            BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_I_index) + 14] = (BGVAR(s16_Script_Executed_Command) - 10 * (BGVAR(u8_Script_Status_Str)[BGVAR(u16_Script_I_index) + 13] - '0')) + '0';
            BGVAR(u16_Script_Handler_State) = 0;
            BGVAR(u16_Script_Exec_State) = 0;
            BGVAR(u16_Script_I_index) = 0;
         }
         break;
      }
   }
}


int DriveScriptStatusCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (DRIVE_ADDRESSED)
   {
      if (u8_Output_Buffer_Free_Space != COMMS_BUFFER_SIZE)
         return (SAL_NOT_FINISHED);
      PrintStringCrLf(&BGVAR(u8_Script_Status_Str)[0], 0);
   }
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: getSelectedPathIndex
// Description:
//    This Function reads the current PATH selected index from the related Digital Inputs
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int getSelectedPathIndex(int drive)
{
   // AXIS_OFF;
   int s16_index = 0, s16_temp = 0;
   REFERENCE_TO_DRIVE;
   s16_temp = (VAR(AX0_u16_Mode_Input_Arr[PATH_BIT_0_INP]) != 0);
   s16_index |= s16_temp;
   s16_temp = (VAR(AX0_u16_Mode_Input_Arr[PATH_BIT_1_INP]) != 0);
   s16_index |= s16_temp<<1;
   s16_temp = (VAR(AX0_u16_Mode_Input_Arr[PATH_BIT_2_INP]) != 0);
   s16_index |= s16_temp<<2;
   s16_temp = (VAR(AX0_u16_Mode_Input_Arr[PATH_BIT_3_INP]) != 0);
   s16_index |= s16_temp<<3;
   s16_temp = (VAR(AX0_u16_Mode_Input_Arr[PATH_BIT_4_INP]) != 0);
   s16_index |= s16_temp<<4;


   // inc the index by "1" because home(index 0) can't preform by the path selecters
   // and there are 32 paths so we need to skip index 0 and allow 1-32 options using 5 selectors.
   return (s16_index+1);
}


//**********************************************************
// Function Name: isPathTriggered
// Description:
//    This Function check if path trigger preformed by detecting PATH_CTRG_INP rising edge("0" -> "1")
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int isPathTriggered(int drive)
{
   int s16_triggered = 0;
   // AXIS_OFF;

   // if drive is disable ignore the rising edge to prevent unexpected movment when drive is enabled
   if (!Enabled(drive))
   {
      BGVAR(s16_Prev_Path_Trigger_State) = BGVAR(s16_Current_Path_Trigger_State) = (VAR(AX0_u16_Mode_Input_Arr[PATH_CTRG_INP]) != 0);
      return 0;
   }

   // nitsan: ignore trigger if brake is released and motino should be ignored during the operation.
   if (BGVAR(u16_Allow_Motion) == 0) return 0;

   // Check if there are access-rights for triggering a motion task
   if(CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_PS, BGVAR(s16_Access_Channel_That_Triggers_Path_Input_Level)) != SAL_SUCCESS)
   {
      return 0; // Ignore the trigger since the access-rights are missing
   }

   // get current ptah trigger state
   BGVAR(s16_Current_Path_Trigger_State) = (VAR(AX0_u16_Mode_Input_Arr[PATH_CTRG_INP]) != 0);

//   BGVAR(s16_Current_Path_Trigger_State) = InxValue(IsInFunctionalityConfigured(PATH_CTRG_INP, drive), drive);
   // check if triggered high
   if((BGVAR(s16_Prev_Path_Trigger_State) == 0) && (BGVAR(s16_Current_Path_Trigger_State) == 1))
      s16_triggered = 1;

   BGVAR(s16_Prev_Path_Trigger_State) = BGVAR(s16_Current_Path_Trigger_State);
   return s16_triggered;
}


//**********************************************************
// Function Name: ProcessGearCclr
// Description:
//    This function process the cclr din and clear accumulators if IO is set.
//    If P2-50 is 1: reset continuously. if 0: reset on rising edge.
//    This functionality is available in opmode Pt and PS (position command with internal or external command).
//    Pt is opmode position gear mode (4), PS is position mode (8).
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void ProcessGearCclr(int drive)
{
   // AXIS_OFF;
   static unsigned int u16_prev_cclr_In;
   REFERENCE_TO_DRIVE;
   
   if (VAR(AX0_s16_Opmode) == 4 || VAR(AX0_s16_Opmode) == 8)
   {
      // digital input for CCLR is set
      if (VAR(AX0_u16_Mode_Input_Arr[GEAR_CCLR_INP]))
      {
         // check P2-50 (0: reset continuously on hi level. 1: reset on rising edge)
         // if hi level or rising edge detected
         if (BGVAR(u16_P2_50_Cclr_Trigger) || (u16_prev_cclr_In == 0))
         {
            // Instruct the gearing function to clear the position deviation due to rising edge or on level.
            VAR(AX0_u16_Gear_BG_Flags) |= 0x0004; // Set bit 2, the flag is cleared automatically in the gearing code.
         }
      }
   }

   u16_prev_cclr_In = VAR(AX0_u16_Mode_Input_Arr[GEAR_CCLR_INP]);
}



//**********************************************************
// Function Name: SalWriteSpeedTorqueLimitCommand
// Description:
//    This function writes the param P1-02
//    valid values: 0x00, 0x01, 0x10, 0x11 (described in schnider LXM23 manual)
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteSpeedTorqueLimitCommand(long long param, int drive)
{
   unsigned int u16_temp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u16_temp = (unsigned int)param;
   if (u16_temp != 0x00 && u16_temp != 0x01 && u16_temp != 0x10 && u16_temp != 0x11)
   {
       return VALUE_OUT_OF_RANGE;
   }

   BGVAR(s16_P1_02_Spd_Trq_lim) = u16_temp;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: ProcessVelocityCommand
// Description:
//    This function handles the following (Lex...) signals:
//       a) SPDLM = Speed limit enable signal coming from an digital input.
//       b) P1-02 = Speed limit enable parameter (lower priority than SPDLM).
//       c) SPD0 & SPD1 = Speed command selection by assigned digital input modes.

//    This function is responsible for handling the speed commands in the following manner:
//       a) In torque mode (CDHD OPMODE=2 & 3) the speed limit functionality enables a PI-Loop,
//          which is responsible for adjusting continuously a current limitation variable in
//          order to not exceed a certain velocity threshold even if the commanded current
//          would usually be high enough to let the motor run into an overspeed situation.
//       b) In velocity mode (CDHD OPMODE 0 & 1) the speed command selection feature just applies
//          the commanded velocity to the velocity loop.
//
//    This function MUST run out of Background since it uses switch-case instructions.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void ProcessVelocityCommand(int drive)
{
   // AXIS_OFF;

   int s16_command_selector, s16_speed_lim_en, s16_spdlm;
   int tmp1, tmp0;
   int s16_tmp_vcmd, s16_serial_vcmd_no_analog;
   int s16_lex_ar_func;

   int         s16_IO_control_access_rights;


   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))  return; // Do nothing if the Drive is not a Lexium

   if (BGVAR(u16_Schneider_Bypass) & 2) return;

   /***********************************/
   /* Here check for IO access-rights */
   /***********************************/
   // First select the action to be checked
   switch (BGVAR(u16_LXM28_Opmode))
   {
       case SE_OPMODE_TZ:     // torque mode (Tz)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_TZ;
            break;
       case SE_OPMODE_T:      // analog torque mode (T)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_T;
            break;
       case SE_OPMODE_SZ:     // serial velocity mode (Sz)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_SZ;
            break;
         case SE_OPMODE_S:    // analog velocity mode (S)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_S;
            break;
       default:
            s16_lex_ar_func = LEX_AR_FCT_RESERVED;
            break;
    }
    // Ask for access-rights on behalf of the digital inputs.
    s16_IO_control_access_rights = CheckLexiumAccessRights(drive, s16_lex_ar_func, LEX_CH_IO_CONTROL);
    // Check if acces-rights just have been refused.
            // Set CDHD OPMODE to 0. This function automatically restores the correct
            // OPMODE further down in case that access-rights are given back to IO.
            // Block motion by setting serial command to 0
    if(s16_IO_control_access_rights == LEXIUM_INVALID_ACCESS_RIGHTS)
    {
        return; // Leave function
    }
    /*** End of access-rights handling ***/


   // Get selector from the digital inputs of the input modes "speed command selection".
   // - If e.g. input 3 is assigned in mode SPD_CMD_SELECT_0_INP, then bit 2 in the array
   //   field SPD_CMD_SELECT_0_INP is set to true.
   // - If e.g. input 7 is assigned in mode SPD_CMD_SELECT_1_INP, then bit 6 in the array
   //   field SPD_CMD_SELECT_1_INP is set to true.
   tmp1 = ((VAR(AX0_u16_Mode_Input_Arr[SPD_CMD_SELECT_1_INP]) != 0) << 1); // tmp1 is either 0 or 1
   tmp0 = (VAR(AX0_u16_Mode_Input_Arr[SPD_CMD_SELECT_0_INP]) != 0);        // tmp0 is either 0 or 1

   s16_command_selector = tmp1 | tmp0; // Command selector is either 0b0000, 0b0001, 0b0010 or 0b0011

   s16_speed_lim_en =  BGVAR(s16_P1_02_Spd_Trq_lim) & 0x0001; // Ask for low-byte of Parameter P1-02
                                                              // (PSTL, speed torque limit enable)
                                                              // Speed limit enable via digital input

   s16_spdlm = VAR(AX0_u16_Mode_Input_Arr[SPD_LIM_EN]); // Speed limit enable via digital input


   switch (BGVAR(u16_LXM28_Opmode))
   {
      case SE_OPMODE_TZ:   // torque mode (Tz)
      case SE_OPMODE_T:    // analog torque mode (T)
         if (BGVAR(u8_CDHD_Normal_Operation))
         {
            // If either P1-02 Byte 0 is true OR if digital input mode enables
            // the speed limitation feature.
            if (s16_speed_lim_en || s16_spdlm)
            {
               // fix to bug 3284
               // If the digital inputs in mode "speed command select 1/0" have
               // a bitcombination of != 0
               if (s16_command_selector != 0)
               {
                  // Take the velocity limit out of table fields 0, 1, 2
                  BGVAR(s16_Torque_Mode_Vel_Limit_Velocity_Thresh) = BGVAR(s16_P1_09to11_Spd_Cmd_Or_Limit_Internal)[s16_command_selector - 1];
               }
               else // Command selection bits are 0b00, take always the analog input 1 velocity
               {
                  // Here the Analog Input 1 value scaled to a velocity has to be taken.
                  BGVAR(s16_Torque_Mode_Vel_Limit_Velocity_Thresh) = VAR(AX0_s16_Analog_Vel_Cmnd);
               }
            }
            else // No speed limitation in torque mode
            {
               // Use VLIM as a velocity limit.
               BGVAR(s16_Torque_Mode_Vel_Limit_Velocity_Thresh) = 22750;
            }

            // Enable the PI-Loop which does the speed limitation in torque mode
            BGVAR(u16_Run_Torque_Mode_Vel_Limit_Pi_Loop) = 1;
         }
      break;


      case SE_OPMODE_SZ:        // velocity mode (Sz)
      case SE_OPMODE_S:         // analog velocity mode (S)

         // if active disable is in progress, dont set vel command
         // Fix BUG#4111: if drive is about to be disable, dont set vel command
         if ((BGVAR(u16_AD_State) != AD_IDLE) || BGVAR(s16_DisableInputs))
         {
            // nitsan - dont set vel command if active disable is running
         }
         else if (BGVAR(u16_Lex_Jog_Or_Autotune_Active) != 0)
         {
            // nitsan - dont set vel command if jog inputs are in control
         }
         else if(IsOpmodeChangeInProgress(drive) || BGVAR(u16_Block_Opmode_Change))
         {
            // nitsan - dont set vel command if =s= opmode change in progress or
            // if the function HoldScan temporarily changed the OPMODE to 0 in order
            // to ramp down in a controlled manner to zero velocity.
         }
         else if (BGVAR(u16_Allow_Motion) == 0)
         {
            // brake is being released, so no motion allowed at this time.
            // set command to zero.
            VAR(AX0_s16_Serial_Vel_Cmnd) = 0;
            VAR(AX0_s16_Acc_Dec_Cmnd_Ptr) = (int)((long)&VAR(AX0_s16_Serial_Vel_Cmnd) & 0xffff);
         }
         else
         {
            // command source is serial or analog. analog is cell zero and opmode SZ
            s16_serial_vcmd_no_analog = ((s16_command_selector != 0) || (BGVAR(u16_LXM28_Opmode) == SE_OPMODE_SZ));



            if (s16_serial_vcmd_no_analog)
            {
               // if both SPD command selector IOs are zero, use zero command, else go to P1-09 table
               if (s16_command_selector == 0)
               {
                  s16_tmp_vcmd = 0;
               }
               else
               {
                  // if s16_command_selector == 1, use array index zero, and so on
                  s16_tmp_vcmd = BGVAR(s16_P1_09to11_Spd_Cmd_Or_Limit_Internal)[s16_command_selector - 1];
               }

               //set the zclamp selector to serial speed command input AX0_s16_Zclamp_On bit2 = 0
               VAR(AX0_s16_Zclamp_On) &= ~ZCLAMP_ANALOG_MASK;
            }
            else
            {
                 s16_tmp_vcmd = VAR(AX0_s16_Analog_Vel_Cmnd);
                 //set the zclamp selector to analog speed command input AX0_s16_Zclamp_On bit3 = 1
                 VAR(AX0_s16_Zclamp_On) |= ZCLAMP_ANALOG_MASK;
            }

            // check if CMDINV is on (in this case direction of commnd is inverted).
            s16_tmp_vcmd *= ((VAR(AX0_u16_Mode_Input_Arr[CMD_INV_INP])) ? (-1) : 1);

            if (s16_serial_vcmd_no_analog)
            {
                 // set opmode to serial velocity if needed
                 if (0 != VAR(AX0_s16_Opmode))
                 {
                      SetOpmode(drive, 0);
                 }

                 //Fix IPR#1321: use command after ACC/DEC limit instead of user command.
                 // check if ZCLAMP is active for serial speed command
                 VAR(AX0_s16_Serial_Vel_Cmnd) = checkZclampForSerialVelocity(drive,s16_tmp_vcmd);


                 VAR(AX0_s16_Acc_Dec_Cmnd_Ptr) = (int)((long)&VAR(AX0_s16_Serial_Vel_Cmnd) & 0xffff);
            }
            else
            {
                 // set opmode to analog velocity if needed
                 if (1 != VAR(AX0_s16_Opmode))
                 {
                      SetOpmode(drive, 1);
                 }

                 // Put the address of the analog velocity-command variable into a 16-bit variable, which
                 // acts as a pointer. It is ensured by the linker command file, that the variable
                 // AX0_s16_Analog_Vel_Cmnd will be accessible by a 16-bit adddress variable.
                 if (BGVAR(s16_Zero_Mode) == 0) // This is to avoid reseting this pointer during ZERO procedure, causing the ZERO to fail
                    VAR(AX0_s16_Acc_Dec_Cmnd_Ptr) = (int)((long)&VAR(AX0_s16_Analog_Vel_Cmnd) & 0xffff);
            }
         }
         /*
         else
         {
            // Put the address of the analog velocity-command variable into a 16-bit variable, which
            // acts as a pointer. It is ensured by the linker command file, that the variable
            // AX0_s16_Analog_Vel_Cmnd will be accessible by a 16-bit adddress variable.
            VAR(AX0_s16_Acc_Dec_Cmnd_Ptr) = (int)((long)&VAR(AX0_s16_Analog_Vel_Cmnd) & 0xffff);
         }
         */

         // Disable the PI-Loop which does the speed limitation in torque mode
         BGVAR(u16_Run_Torque_Mode_Vel_Limit_Pi_Loop) = 0;
      break;

      case SE_OPMODE_CANOPEN:        // CANopen mode
         if (BGVAR(u8_CDHD_Normal_Operation))
         {
            // fix bugzilla 4384: Velocity limit behavior is not  the same in modbus and CanOpen in Torque Mode
            // enable the vlim saturation for torque mode when in canopen mode.
            if (p402_modes_of_operation_display == PROFILE_TORQUE_MODE ||
                p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_TORQUE_MODE ||
                p402_modes_of_operation_display == ANALOG_TORQUE_MODE)
            {
               // If either P1-02 Byte 0 is true enable
               // the speed limitation feature.
               if (s16_speed_lim_en)
               {
                  // fix to bug 3284
                  // If the digital inputs in mode "speed command select 1/0" have
                  // a bitcombination of != 0
                  if (BGVAR(u16_P1_82_Can_Trq_Mode_VLim_Selector) != 0)
                  {
                     // Take the velocity limit out of table fields 0, 1, 2 or 3
                     BGVAR(s16_Torque_Mode_Vel_Limit_Velocity_Thresh) = BGVAR(s16_P1_09to11_Spd_Cmd_Or_Limit_Internal)[BGVAR(u16_P1_82_Can_Trq_Mode_VLim_Selector) - 1];
                  }
                  else // Command selection bits are 0b00, take always the analog input 1 velocity
                  {
                     // Here the Analog Input 1 value scaled to a velocity has to be taken.
                     BGVAR(s16_Torque_Mode_Vel_Limit_Velocity_Thresh) = VAR(AX0_s16_Analog_Vel_Cmnd);
                  }
               }
               else // No speed limitation in torque mode
               {
                  // Use VLIM as a velocity limit.
                  BGVAR(s16_Torque_Mode_Vel_Limit_Velocity_Thresh) = 22750;
               }

               // Enable the PI-Loop which does the speed limitation in torque mode
               BGVAR(u16_Run_Torque_Mode_Vel_Limit_Pi_Loop) = 1;
         }
         else
         {
            // disable velocity saturation for other modes.
            BGVAR(u16_Run_Torque_Mode_Vel_Limit_Pi_Loop) = 0;
            }
         }

         break;

      default: // In all other opmodes the speed command values out of the table have no impact
         // Disable the PI-Loop which does the speed limitation in torque mode
         BGVAR(u16_Run_Torque_Mode_Vel_Limit_Pi_Loop) = 0;
      break;
   }
}


//**********************************************************
// Function Name: ProcessTorqueCommand
// Description:
//    This function process the TRQLM, TLLM, TRLM dins and sets the current limit (or current command)
//    P1-02 is enable/disable torque limit/command (the upper byte).
//    P1-12 - P1-14 are torque limit values. selector of values is determined by TCM0 and TCM1 dins
//    TRQLM enables/disables the limit function and TLLM, TRLM can override TRQLM.
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void ProcessTorqueCommand(int drive)
{
   // AXIS_OFF;
   int s16_command_selector, s16_torque_lim_en, s16_trqlm, tmp1, tmp0;
   int s16_tmp_tcmd, s16_serial_vcmd_no_analog;
   int s16_lex_ar_func;

   int   s16_IO_control_access_rights;


   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   if (BGVAR(u16_Schneider_Bypass) & 1) return;

   /***********************************/
   /* Here check for IO access-rights */
   /***********************************/
   // First select the action to be checked
   switch (BGVAR(u16_LXM28_Opmode))
   {
       case SE_OPMODE_TZ:     // torque mode (Tz)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_TZ;
            break;
       case SE_OPMODE_T:      // analog torque mode (T)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_T;
            break;
       case SE_OPMODE_SZ:     // velocity mode (Sz)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_SZ;
            break;
         case SE_OPMODE_S:    // analog velocity mode (S)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_S;
            break;
       case SE_OPMODE_PT:     // gearing mode (Pt)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_PT;
            break;
       case SE_OPMODE_PS:     // position mode (PS)
            s16_lex_ar_func = LEX_AR_FCT_ACTIVATE_PS;
            break;
       default:
            s16_lex_ar_func = LEX_AR_FCT_RESERVED;
            break;
    }
    // Ask for access-rights on behalf of the digital inputs.
    s16_IO_control_access_rights = CheckLexiumAccessRights(drive, s16_lex_ar_func, LEX_CH_IO_CONTROL);
    // Check if acces-rights just have been refused.
            // Set CDHD OPMODE to 2. This function automatically restores the correct
            // OPMODE further down in case that access-rights are given back to IO.
            // Block motion by setting serial command to 0
    if(s16_IO_control_access_rights == LEXIUM_INVALID_ACCESS_RIGHTS)
    {
        return; // Leave function
    }
    /*** End of access-rights handling ***/



   // get selector from inputs (each input is a bit. combination of 4 bits)
   tmp1 = (((VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_1_INP]) & 0xFF) != 0) << 1);
   tmp0 = ((VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_0_INP]) & 0xFF) != 0);

   //s16_command_selector = ((VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_1_INP])!=0) << 1) | (VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_0_INP])!=0);
   s16_command_selector = tmp1 | tmp0;
   s16_trqlm = VAR(AX0_u16_Mode_Input_Arr[TRQ_LIM_EN]);
   s16_torque_lim_en =  BGVAR(s16_P1_02_Spd_Trq_lim) & 0x0010;

   switch (BGVAR(u16_LXM28_Opmode))
   {
      case SE_OPMODE_TZ:   // torque mode
      case SE_OPMODE_T:    // analog torque mode
         // No current saturation in OMODE torque
         BGVAR(s16_I_Sat_Hi_En) = 0;
         BGVAR(s16_I_Sat_Lo_En) = 0;

         // if active disable in progress or if drive is about to be disable
         // Fix BUG#4111: if drive is about to be disable, dont set vel command
         if ((BGVAR(u16_AD_State) != AD_IDLE) || BGVAR(s16_DisableInputs))
         {
            // set to zero command.
        //    VAR(AX0_s16_Serial_Crrnt_Cmnd) = 0;
        //    LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Serial_Crrnt_Cmnd);
         }
         else if (BGVAR(u16_Lex_Jog_Or_Autotune_Active) != 0)
         {
            // do nothing, jog digital inputs are in control
         }
         else if(IsOpmodeChangeInProgress(drive) || BGVAR(u16_Block_Opmode_Change))
         {
            // nitsan - dont set torque command if =s= opmode change is in progress or
            // if the function HoldScan temporarily changed the OPMODE to 0 in order
            // to ramp down in a controlled manner to zero velocity.
         }
         else if (BGVAR(u16_Allow_Motion) == 0)
         {
            // dont zero command if testing motor phase disconnection on enable (the test gives serial current command, so dont zero it).
            if (BGVAR(u16_Brake_Release_State) == BRAKE_RELEASE_IDLE)
            {
               // brake is being released, so no motion allowed at this time.
               // set command to zero.
               VAR(AX0_s16_Serial_Crrnt_Cmnd) = 0;
            }
               
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Serial_Crrnt_Cmnd);
         }
         else if (BGVAR(u8_CDHD_Normal_Operation))
         {
            // in opmode T, use analog input as command. in opmode Tz use zero as command
            s16_serial_vcmd_no_analog = (s16_command_selector || (BGVAR(u16_LXM28_Opmode) == SE_OPMODE_TZ));
            if (s16_serial_vcmd_no_analog)
            {
               // if both trq command selector IOs are zero, use zero command, else go to P1-12 table
               if (s16_command_selector == 0)
               {
                  s16_tmp_tcmd = 0;
               }
               else
               {
                  // if s16_command_selector == 1, use array index zero, and so on
                  s16_tmp_tcmd = BGVAR(s16_P1_12_Trq_Cmd_Lim_Internal)[s16_command_selector - 1];
               }
            }
            else
            {
                 s16_tmp_tcmd = VAR(AX0_s16_Analog_Crrnt_Cmnd);
            }


            if (s16_serial_vcmd_no_analog)
            {
                 // set opmode to serial velocity if needed
                 if (2 != VAR(AX0_s16_Opmode))
                 {
                      SetOpmode(drive, 2);
                 }

                 // table unit is percentage
                 VAR(AX0_s16_Serial_Crrnt_Cmnd) = s16_tmp_tcmd;
                 LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Serial_Crrnt_Cmnd);
            }
            else
            {
                 // set opmode to serial velocity if needed
                 if (3 != VAR(AX0_s16_Opmode))
                 {
                      SetOpmode(drive, 3);
                 }

                 // analog comamnd when selector==0 and opmode is T
                 LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Analog_Crrnt_Cmnd);
            }
         }
      break;

      case SE_OPMODE_S:         // velocity mode (S)
      case SE_OPMODE_SZ:        // analog velocity mode (Sz)
      case SE_OPMODE_PT:        // gearing mode (Pt)
      case SE_OPMODE_PS:        // position mode (PS)
         // check if need to use torque limit function
         if (s16_torque_lim_en || s16_trqlm)
         {
            BGVAR(s16_I_Sat_Hi_En) = 1;
            BGVAR(s16_I_Sat_Lo_En) = 1;
         }
         else
         {
            // No torque limitation if not requested by inputs or P1-02
            BGVAR(s16_I_Sat_Hi_En) = 0;
            BGVAR(s16_I_Sat_Lo_En) = 0;
/* schneider decided to cancel TLLM and TRLM modes
            // if torque limit function is overridden by DIN
            if (VAR(AX0_u16_Mode_Input_Arr[TRQ_LIM_POS]))     // TLLM
               BGVAR(s16_I_Sat_Hi_En) = 1;
            if (VAR(AX0_u16_Mode_Input_Arr[TRQ_LIM_NEG]))     // TRLM
               BGVAR(s16_I_Sat_Lo_En) = 1; */
         }

         // fix to bug 3284
         // when command selector is zero, torque limit always use analog reading as limit
         if (BGVAR(s16_I_Sat_Hi_En))
         {
            if (s16_command_selector)
            {  // table unit is percentage
               BGVAR(s16_I_Sat_Hi) = BGVAR(s16_P1_12_Trq_Cmd_Lim_Internal)[s16_command_selector - 1];
            }
            else
            {
               BGVAR(s16_I_Sat_Hi) = VAR(AX0_s16_Analog_Crrnt_Cmnd);
            }
            if (BGVAR(s16_I_Sat_Hi) < 0) BGVAR(s16_I_Sat_Hi) = -BGVAR(s16_I_Sat_Hi);
         }

         if (BGVAR(s16_I_Sat_Lo_En))
         {
            if (s16_command_selector)
            {  // table unit is percentage
               BGVAR(s16_I_Sat_Lo) = BGVAR(s16_P1_12_Trq_Cmd_Lim_Internal)[s16_command_selector - 1];
            }
            else
            {
               BGVAR(s16_I_Sat_Lo) = VAR(AX0_s16_Analog_Crrnt_Cmnd);
            }
            if (BGVAR(s16_I_Sat_Lo) < 0) BGVAR(s16_I_Sat_Lo) = -BGVAR(s16_I_Sat_Lo);
         }
      break;

      default: // In OPMODE fieldbus
         // No current saturation in OMODE fieldbus, needs to be checked...
         BGVAR(s16_I_Sat_Hi_En) = 0;
         BGVAR(s16_I_Sat_Lo_En) = 0;
      break;
   }
}


void  CalcTorqueCmdLimitTable(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s16_P1_12_Trq_Cmd_Lim_Internal)[0] = ((long)BGVAR(s16_P1_12_Trq_Cmd_Lim)[0] * BGVAR(s32_Motor_I_Cont_Internal)) / 100;
   BGVAR(s16_P1_12_Trq_Cmd_Lim_Internal)[1] = ((long)BGVAR(s16_P1_12_Trq_Cmd_Lim)[1] * BGVAR(s32_Motor_I_Cont_Internal)) / 100;
   BGVAR(s16_P1_12_Trq_Cmd_Lim_Internal)[2] = ((long)BGVAR(s16_P1_12_Trq_Cmd_Lim)[2] * BGVAR(s32_Motor_I_Cont_Internal)) / 100;
}


//*****************************************************************************************
// Function Name: CalcSpeedCmdLimitTable
// Description: This function converts the P1-09...P1-11 settings (speed command/limit values)
//              from the unit [0.1*rpm] into the Drive internal unit of VLIM=22750 (velocity
//              unit in the loop). This function needs to be called in case that:
//               a) VLIM changes
//               b) The parameter P1-09...P1-11 changes
//
// Author: APH
// Algorithm:
// Revisions:
//*****************************************************************************************
void  CalcSpeedCmdLimitTable(int drive)
{
   long long s64_temp;
   int s16_index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error


   // ************************************************************************************************************************************
   // Convert the remaining 3 fields from [0.1*rpm] into velocity unit internal (in the loop --> 22750[Counts] = VLIM)
   // First convert from [0.1*rpm] into [Counts32/125us] since the variable s32_V_Lim_Design is in this unit.
   // Then take this value, multiply it with 22750 and divide it by s32_V_Lim_Design.
   // 2^32 / (60 * 8000 * 10) = 894.784 853 (2^32 = 1rev; 60 for rpm to rps; 8000 for 1/s to 1/125[us]; 10 since user value is in 0.1*rpm
   // ************************************************************************************************************************************
   for (s16_index = 0; s16_index < 3; s16_index++)
   {
      // Convert from [0.1*rpm] into [Counts32/125us] and check if we exceed s32_V_Lim_Design (VLIM in [Counts32/125us]
      s64_temp = 895LL * (long long)BGVAR(s32_P1_09to11_Spd_Cmd_Or_Limit)[s16_index];
      // User parameter is lower than VLIM
      if(llabs(s64_temp) < BGVAR(s32_V_Lim_Design))
         BGVAR(s16_P1_09to11_Spd_Cmd_Or_Limit_Internal)[s16_index] = (int)(s64_temp * (long long)22750 / BGVAR(s32_V_Lim_Design));
      else // Limit to VLIM
      {
         if (s64_temp >= 0)
            BGVAR(s16_P1_09to11_Spd_Cmd_Or_Limit_Internal)[s16_index] = 22750;
         else
            BGVAR(s16_P1_09to11_Spd_Cmd_Or_Limit_Internal)[s16_index] = -22750;
      }
   }
}


//**********************************************************
// Function Name: SalWriteAutoEnableAutoLimitCommand
// Description:
//    This function writes the param P2-68
//    valid values: 0x00, 0x01, 0x10, 0x11 (described in schnider LXM23 manual)
//
//    24/4/2015: due to IPR 1187, nibble z will be updated immedialt and not after power cycle.
//               all other nibbles are effective only after power cycle
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteAutoEnableAutoLimitCommand(long long param, int drive)
{
   // AXIS_OFF;
   unsigned int u16_temp;

   u16_temp = (unsigned int)param;
   if (u16_temp != 0x0000 && u16_temp != 0x0001 && u16_temp != 0x0010 && u16_temp != 0x0011 &&
       u16_temp != 0x0100 && u16_temp != 0x0101 && u16_temp != 0x0110 && u16_temp != 0x0111)
   {
       return VALUE_OUT_OF_RANGE;
   }

   // nibble z is updated immediatly.
   if ((u16_temp & 0x0F00) != (BGVAR(u16_P2_68_Auto_Enable_Actual) & 0x0F00))
   {
      // nibble z (change is effective immediatly) can be only changed if drive is disabled
      if (Enabled(drive))
      {
         return DRIVE_ACTIVE;
      }

      // update the actual value for change take effect immediatly.
      // due to ipr 1104, keep z nibble always zero if not in canopen mode
      if( ((BGVAR(u16_P1_01_CTL) & 0x00FF) == SE_OPMODE_CANOPEN)       ||
          ((BGVAR(u16_P1_01_CTL) & 0x00FF) == SE_OPMODE_SERCOS)        ||
          ((BGVAR(u16_P1_01_CTL) & 0x00FF) == SE_OPMODE_ETHERNET_IP)   ||
          ((BGVAR(u16_P1_01_CTL) & 0x00FF) == SE_OPMODE_ETHERCAT)         )
      {
         if (u16_temp & 0x0100)
            BGVAR(u16_P2_68_Auto_Enable_Actual) |= 0x0100;
         else
            BGVAR(u16_P2_68_Auto_Enable_Actual) &= ~0x0100;
      }
      else
      {
         // reset z nibble
         BGVAR(u16_P2_68_Auto_Enable_Actual) &= 0xF0FF;
      }
   }

   BGVAR(u16_P2_68_Auto_Enable) = u16_temp;
   if(IsInFunctionalityConfigured(SON_INP,drive))
   {
       // here we need to check  u16_P2_68_Auto_Enable and not u16_P2_68_Auto_Enable_Actual
       // because the user is wiriting a new value to P2-68.
       if((BGVAR(u16_P2_68_Auto_Enable) & 0x01) == 0)
       {// looking for SON rising edge so save the current SON input state
            BGVAR(s16_Prev_Son_State) = VAR(AX0_u16_Mode_Input_Arr[SON_INP]);
       }
   }
   // need to check the MSB of P2-68 to set the limit switch behavior.
   // or handle it on the actual limit switch fault handling ?
   // if(BGVAR(u16_P2_68_Auto_Enable) & 0x10)
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: ProcessZClamp
// Description:
//    This function processes the ZCLAMP DIN. if opmode==velocity and command is from analog1,
//    if DIN ZCLAMP is set deadband will be equal to P1-38, else deadband is equal to the ANIN1DB command value.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void ProcessZClamp(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   
   if(VAR(AX0_u16_Mode_Input_Arr[ZCLAMP_0_INP])) // to efect only bit 0 of "AX0_s16_Zclamp_On"
   {// set bit 0 to "1"
       VAR(AX0_s16_Zclamp_On) |= ZCLAMP_DI_MASK;
   }
   else
   {// clear bit 0 to "0"
       VAR(AX0_s16_Zclamp_On) &= ~ZCLAMP_DI_MASK;
   }
}

//**********************************************************
// Function Name: checkZclampForSerialVelocity
// Description:
//    This function checks if zclamp is activated for serial speed command
//    when this check for analog speed command is in RT.
// Author: Moshe
// Algorithm:
// Revisions:
// 6.12.15 Moshe A
// Fix IPR#1321: use velocity command after ACC/DEC limit for ZCLAMP activation
// Release ZCLAMP with user command.
//**********************************************************
int checkZclampForSerialVelocity(int drive, int s16_serial_vel_command)
{
   // AXIS_OFF;
   unsigned int u16_temp_time;
   REFERENCE_TO_DRIVE;
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))  return s16_serial_vel_command; // Do nothing if the Drive is not a Lexium

   // check bit 2 if serial speed is active
   if((VAR(AX0_s16_Zclamp_On) & ZCLAMP_ANALOG_MASK) != 0) return s16_serial_vel_command;    // If analog command is active return

   // check bit 0 if ZCLAMP DI is ON
   if((VAR(AX0_s16_Zclamp_On) & ZCLAMP_DI_MASK) == 0)
   {// reset bits 1 & 3 and abort
      VAR(AX0_s16_Zclamp_On) &= ~ZCLAMP_ACTIVE_MASK;
      VAR(AX0_s16_Zclamp_On) &= ~ZCLAMP_RELEASE_HYST;
      return s16_serial_vel_command;
   }

   // check bit 3 if hysteresis need to be checked
   if(VAR(AX0_s16_Zclamp_On) & ZCLAMP_RELEASE_HYST)
   {// hysteresis need to be checked for ZCLAMP release
      if(abs(VAR(AX0_s16_NL_Vel_Loop_Cmnd)) <= VAR(AX0_s16_P1_38_Zero_Speed_Window))
      {
         if(abs(s16_serial_vel_command) <= VAR(AX0_s16_P1_38_Zero_Speed_Window))
         {
            return 0;
         }
      }
      else
      {
         VAR(AX0_s16_Zclamp_On) &= ~ZCLAMP_ACTIVE_MASK;
         VAR(AX0_s16_Zclamp_On) &= ~ZCLAMP_RELEASE_HYST;
         return s16_serial_vel_command;
      }
   }

   // check bit 1 if ZCLAMP already executed
   if(VAR(AX0_s16_Zclamp_On) & ZCLAMP_ACTIVE_MASK)
   {
      if(abs(s16_serial_vel_command) > VAR(AX0_s16_P1_38_Zero_Speed_Window))
      {
         VAR(AX0_s16_Zclamp_On) |= ZCLAMP_RELEASE_HYST;
         return s16_serial_vel_command;
      }
      return 0;
   }

   // check if loop control is below ZSPD threshold and if so preform ZCLAMP
   if(abs(VAR(AX0_s16_NL_Vel_Loop_Cmnd)) < VAR(AX0_s16_P1_38_Zero_Speed_Window))   // VCMD of the loop is within "Zero speed window" (P1-38)
   {
      if((BGVAR(u16_P2_65_GBIT) & 0x400) == 0)
      {
         // capture the current position
         do
         {
            u16_temp_time = Cntr_3125; // the same RT Interrupt
            LVAR(AX0_u32_Zclamp_PFB_Capture_Lo) = LVAR(AX0_u32_Pfb_Internal_After_Mod_Lo);
            LVAR(AX0_s32_Zclamp_PFB_Capture_Hi) = LVAR(AX0_s32_Pfb_Internal_After_Mod_Hi);
         } while (u16_temp_time != Cntr_3125);
      }
      // activate ZCLAMP
      VAR(AX0_s16_Zclamp_On) |= ZCLAMP_ACTIVE_MASK;
      return 0;
   }
   return s16_serial_vel_command;
}


//**********************************************************
// Function Name: ProcessZclampFunction
// Description:
//    This function processes the ZCLAMP function depends on GBIT (P2-65) bit 10
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void ProcessZclampFunction(drive)
{
   // AXIS_OFF;

   unsigned int u16_temp_time;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))  return; // Do nothing if the Drive is not a Lexium

   if((BGVAR(u16_P2_65_GBIT) & 0x400) &&
        (VAR(AX0_u16_Motor_Stopped) == 2))
   {  // FIX BUG#4329
      //set capture position to current position to prevent position
      //fix to capture position when changing P2-65 bit 10 while drive disabled.
      do
      {
         u16_temp_time = Cntr_3125; // the same RT Interrupt
         VAR(AX0_u32_Zclamp_PFB_Capture_Lo) = LVAR(AX0_u32_Pfb_Internal_After_Mod_Lo);
         VAR(AX0_s32_Zclamp_PFB_Capture_Hi) = LVAR(AX0_s32_Pfb_Internal_After_Mod_Hi);
      } while (u16_temp_time != Cntr_3125);
   }

   if((BGVAR(u16_P2_65_GBIT) & 0x400) && (BGVAR(u16_Zclamp_State) == 0))// if GBIT bit 10 is "1" this is regular 0 speed command which handle in the RT so do nothing
   {// FIX BUG#4291 if bit 10 changed while state machine is running, let state machine finish this position correction.
       return;
   }

   if(((VAR(AX0_s16_Zclamp_On) & ZCLAMP_ACTIVE_MASK) == 0) &&
      (BGVAR(u16_Zclamp_State) > 0) &&
      (BGVAR(u16_Zclamp_State) < 4))//if zclamp aborted or finished and state machine not in idle or already in abort sequence
   {// Zclamp is released abort state machine
       BGVAR(u16_Zclamp_State) = 5;
   }

   switch(BGVAR(u16_Zclamp_State))
   {
       case 0:
            // if Zclamp is active
            if(VAR(AX0_s16_Zclamp_On) & ZCLAMP_ACTIVE_MASK)
            {
               BGVAR(u16_Zclamp_Last_Opmode) = BGVAR(u16_LXM28_Opmode);
               BGVAR(u16_Zclamp_State)++;
            }
            break;

       case 1:
            // wait for stand still and change opmode to position
            if(VAR(AX0_u16_Motor_Stopped) == 2)
            {// fix position to the CLAMP position.
               //MASK all PE faults to prevent PE faults in velocity modes
               //FIX IPR#1287
               // mask PEMAX_FLT_MASK before changing to position mode
               if((BGVAR(s64_Faults_Mask) & PEMAX_FLT_MASK) != 0)
               {// fault is not masked, mask it for ZCLAMP feature and unmask it when done
                  BGVAR(u16_Zclamp_PE_Masked_Flag) = 1;
                  BGVAR(s64_Faults_Mask) &= ~PEMAX_FLT_MASK;
               }

               // in 1.40 change mode on the fly need to copy PFB to PCMD to avoid shaft jumps.
               // Tell the OPMODE change function to copy PFB to PCMD during the OPMODE switch FIX IPR#1287
               //VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_COPY_PFB_TO_PCMD_MASK;

               SetSchneiderOpmode(drive , SE_OPMODE_PS);
               BGVAR(u16_Zclamp_State)++;
            }
            break;

       case 2:
            // preform position movement to the captured position
               if(BGVAR(u16_LXM28_Opmode) == SE_OPMODE_PS)
               {// if in position mode preform position move in the maximus alowed speed
                 BGVAR(s16_Move_Abs_Issued) = 1;
                 ExecuteMoveCommand(drive, LLVAR(AX0_u32_Zclamp_PFB_Capture_Lo) ,BGVAR(s32_V_Lim_Design) , PTP_IMMEDIATE_TRANSITION);
                 BGVAR(u16_Zclamp_State)++;
               }
            break;

       case 3:
            // wait for PTP generator to finish or if finished
               if(VAR(AX0_u16_Motor_Stopped) == 2)
               {// wait for motor to stop and retrive the original opmode (analog or internal speed)
                 SetSchneiderOpmode(drive , BGVAR(u16_Zclamp_Last_Opmode));
                 BGVAR(u16_Zclamp_State)++;
               }
            break;

       case 4:
            // wait for opmode to be back to the original opmode
            if(BGVAR(u16_LXM28_Opmode) == BGVAR(u16_Zclamp_Last_Opmode))
            {
               // remask the mask FIX IPR#1287
               if(BGVAR(u16_Zclamp_PE_Masked_Flag) == 1)
               {// fault was masked,unmask it when ZCLAMP done
                  BGVAR(u16_Zclamp_PE_Masked_Flag) = 0;
                  BGVAR(s64_Faults_Mask) |= PEMAX_FLT_MASK;
               }

               // wait for zclamp to be disabled (bit 0 or bit 1 is "0")
               if ((VAR(AX0_s16_Zclamp_On) & ZCLAMP_ACTIVE_MASK ) != ZCLAMP_ACTIVE_MASK )
               {// set the state back to idle
                  BGVAR(u16_Zclamp_State) = 0;
               }
            }
            break;

//****************** ABORT STATE MACHINE - THIS PROCEDURE SHOULD NoT BE INTERRUPTED ****************************
       case 5:
            // abort the zclamp and set opmode back to speed
              if(BGVAR(u16_LXM28_Opmode) == SE_OPMODE_PS)
              {
                  BGVAR(u16_Ptp_Abort_Flags) |= 1;    // stop motion
              }
              BGVAR(u16_Zclamp_State)++;
            break;

       case 6:
              if(VAR(AX0_u16_Motor_Stopped) == 2)
              {// wait for motor to stop
                  SetSchneiderOpmode(drive , BGVAR(u16_Zclamp_Last_Opmode));
                  BGVAR(u16_Zclamp_State)++;
              }
            break;

       case 7:
              // wait for opmode to be back to the original opmode
              if(BGVAR(u16_LXM28_Opmode) == BGVAR(u16_Zclamp_Last_Opmode))
              {// set state machine back to idle
                 // remask the mask FIX IPR#1287
                  if(BGVAR(u16_Zclamp_PE_Masked_Flag) == 1)
                  {// fault was masked,unmask it when ZCLAMP done
                     BGVAR(u16_Zclamp_PE_Masked_Flag) = 0;
                     BGVAR(s64_Faults_Mask) |= PEMAX_FLT_MASK;
                  }
                  BGVAR(u16_Zclamp_State) = 0;
              }
            break;

       default:
            break;
   }
}

//**********************************************************
// Function Name: SalReadProbe1CounterCommand
// Description:
// This function provides the probe #1 events counter,for p_param 5-38
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalReadProbe1CounterCommand(long long *data, int drive)
{
   REFERENCE_TO_DRIVE;
   *data = (unsigned  long)(BGVAR(Probe_1_Rise_Record).u16_edge_counter + BGVAR(Probe_1_Fall_Record).u16_edge_counter);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadProbe2CounterCommand
// Description:
// This function provides the probe #2 events counter,for p_param 5-58
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalReadProbe2CounterCommand(long long *data, int drive)
{
   REFERENCE_TO_DRIVE;
    if (2 > BGVAR(u16_Num_Of_Probes_On_Drive))//CDHD currently supports only 1 probe
      return NOT_SUPPORTED_ON_HW;
    *data = (unsigned  long)(BGVAR(Probe_2_Rise_Record).u16_edge_counter + BGVAR(Probe_2_Fall_Record).u16_edge_counter);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteInxmode32OpmodesValue
// Description:
//    Sal function that handles the OPMODE parameter for
//    digital input mode 32.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteInxmode32OpmodesValue(long long param,int drive)
{
   int u16_temp;
   int u16_index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for (u16_index = 0; u16_index < 2; u16_index++)
   {
      u16_temp = (int)((param >> (8*u16_index)) & 0x00000000000000FFLL);

      if( ((u16_temp > 4) && (u16_temp < 8)) ||
          (u16_temp > 8)
        )
      {
         return VALUE_OUT_OF_RANGE;
      }
   }

   BGVAR(u16_Opmode_Switch_InXMode32_Setting) = (unsigned int)param;

   return SAL_SUCCESS;
}
//**********************************************************
// Function Name: SalReadInxmode32OpmodesValue
// Description:
//    Sal function that handles the OPMODE parameter for
//    digital input mode 32.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadInxmode32OpmodesValue(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Print all 4 nibbles
   PrintDecInAsciiHex(BGVAR(u16_Opmode_Switch_InXMode32_Setting),4);
   PrintCrLf();

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: ProcessOpmodeSwitchInput
// Description:
//    This function processes the input that let's the Drive
//    switch between two modes of operation. This function uses
//    the 'new' way of OPMODE switching, which means involving
//    the OPMODE switch background function.
//
//    This function has been implemented due to a request from
//    Akribis (via M. Rotelmann).
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void ProcessOpmodeSwitchInput(int drive)
{
   // AXIS_OFF;

   int s16_restore_command_value = 0;
   long long s64_temp;
   unsigned int u16_temp_time;

   // In case no input is assigned to OPMODE_SWITCH_INPUT   OR
   // A fieldbus is running the Drive                       OR
   // The Drive is disabled
   if( ((IsInFunctionalityConfigured(OPMODE_SWITCH_INP,drive)) == 0) ||
       (BGVAR(u8_Comm_Mode) == 1)                                    ||
       (!Enabled(drive))
     )
   {
      BGVAR(u16_InXMode32_Input_State) = OPMODE_SWITCH_INPUT_STATE_RESET_STATE_MACHINE;
      return;
   }

   switch(BGVAR(u16_InXMode32_Input_State))
   {
      case OPMODE_SWITCH_INPUT_STATE_IDLE:   // Evaluate the action

         // Check what is the demanded OPMODE
         if(VAR(AX0_u16_Mode_Input_Arr[OPMODE_SWITCH_INP]))
         {
            BGVAR(s16_InXMode32_New_Opmode) = (BGVAR(u16_Opmode_Switch_InXMode32_Setting) >> 8) & 0x00FF;
         }
         else
         {
            BGVAR(s16_InXMode32_New_Opmode) = BGVAR(u16_Opmode_Switch_InXMode32_Setting) & 0x00FF;
         }

         // If OPMODE does not match the demanded mode
         if(VAR(AX0_s16_Opmode) != BGVAR(s16_InXMode32_New_Opmode))
         {
            // Go to next stage
            BGVAR(u16_InXMode32_Input_State) = OPMODE_SWITCH_INPUT_STATE_CAPTURE_COMMAND_VAL;
         }
      break;

      case OPMODE_SWITCH_INPUT_STATE_CAPTURE_COMMAND_VAL:
         // Save the actual command value depending on the OPMODE
         switch(VAR(AX0_s16_Opmode))
         {
            case(0):
               s64_temp = (long long)VAR(AX0_s16_Serial_Vel_Cmnd);
            break;

            case(2):
               s64_temp = (long long)VAR(AX0_s16_Serial_Crrnt_Cmnd);
            break;

            case(4):
               do
               {
                  u16_temp_time = Cntr_3125;
                  s64_temp = LLVAR(AX0_u32_Gear_Not_Lim_Pos_Lo);
               }
               while (u16_temp_time != Cntr_3125);
            break;

            case(8):
               do
               {
                  u16_temp_time = Cntr_3125;
                  s64_temp = LLVAR(AX0_u32_Pos_Fdbk_Dual_Lo);
               }
               while (u16_temp_time != Cntr_3125);
            break;

            default:
               s64_temp = 0;
            break;
         }

         // If the "new" OPMODE is supposed to be according to the low-byte of the OPMODE setting parameter
         if(BGVAR(s16_InXMode32_New_Opmode) == (BGVAR(u16_Opmode_Switch_InXMode32_Setting) & 0x00FF))
         {
            // The present OPMODE is according to the high-byte, save the value accordingly.
            BGVAR(s64_InXMode32_Prev_Cmd_Of_Hi_Byte) = s64_temp;
            // If the captured value is supposed to be applied upon switching back
            if(BGVAR(u16_Opmode_Switch_InXMode32_Restore_Cmd) & 0x0002)
            {
               // Indicate that the variable "BGVAR(s64_InXMode32_Prev_Cmd_Of_Hi_Byte)" contains valid data.
               BGVAR(u16_InxMode32_Prev_Cmd_Valid) |= 0x0002;
            }
            else
            {
               // Indicate that the variable "BGVAR(s64_InXMode32_Prev_Cmd_Of_Hi_Byte)" contains invalid data.
               BGVAR(u16_InxMode32_Prev_Cmd_Valid) &= ~0x0002;
            }
         }
         else
         {
            // The present OPMODE is according to the low-byte, save the value accordingly.
            BGVAR(s64_InXMode32_Prev_Cmd_Of_Lo_Byte) = s64_temp;
            // If the captured value is supposed to be applied upon switching back
            if(BGVAR(u16_Opmode_Switch_InXMode32_Restore_Cmd) & 0x0001)
            {
               // Indicate that the variable "BGVAR(s64_InXMode32_Prev_Cmd_Of_Lo_Byte)" contains valid data.
               BGVAR(u16_InxMode32_Prev_Cmd_Valid) |= 0x0001;
            }
            else
            {
               // Indicate that the variable "BGVAR(s64_InXMode32_Prev_Cmd_Of_Lo_Byte)" contains invalid data.
               BGVAR(u16_InxMode32_Prev_Cmd_Valid) &= ~0x0001;
            }
         }

         // Initiate an OPMODE change
         BGVAR(s16_Start_Opmode_Change) = (BGVAR(s16_InXMode32_New_Opmode) | OPMODE_CHANGE_SOURCE_OPMODE_SWITCH_INP);

         // Go to next stage
         BGVAR(u16_InXMode32_Input_State) = OPMODE_SWITCH_INPUT_STATE_WAIT_FOR_MODECHANGE;
      break;

      case OPMODE_SWITCH_INPUT_STATE_WAIT_FOR_MODECHANGE:
         if (!IsOpmodeChangeInProgress(drive))
         {
            // OPMODE change has either been performed or been rejected by the
            // function "OpmodeChangeBackgroundHandler" (for whatever reason).
            BGVAR(u16_InXMode32_Input_State) = OPMODE_SWITCH_INPUT_STATE_RESTORE_COMMAND_VAL;
         }
      break;

      case OPMODE_SWITCH_INPUT_STATE_RESTORE_COMMAND_VAL:
         // If the OPMODE change has been really performed
         if(BGVAR(s16_InXMode32_New_Opmode) == VAR(AX0_s16_Opmode))
         {
            // If the actual OPMODE is according to the low-byte of the OPMODE setting parameter
            if(BGVAR(s16_InXMode32_New_Opmode) == (BGVAR(u16_Opmode_Switch_InXMode32_Setting) & 0x00FF))
            {
               // If a restore of the low-byte OPMODE is demanded AND
               // if the variable "BGVAR(s64_InXMode32_Prev_Cmd_Of_Lo_Byte)" contains valid data
               if((BGVAR(u16_Opmode_Switch_InXMode32_Restore_Cmd) & 0x0001) && (BGVAR(u16_InxMode32_Prev_Cmd_Valid) & 0x0001))
               {
                  // Read the low-byte OPMODE command
                  s64_temp = BGVAR(s64_InXMode32_Prev_Cmd_Of_Lo_Byte);
                  // Restore the command value
                  s16_restore_command_value = 1;
               }

            }
            else // Actual OPMODE is according to the high-byte of the OPMODE setting parameter
            {
               // If a restore of the high-byte OPMODE is demanded AND
               // if the variable "BGVAR(s64_InXMode32_Prev_Cmd_Of_Hi_Byte)" contains valid data
               if((BGVAR(u16_Opmode_Switch_InXMode32_Restore_Cmd) & 0x0002) && (BGVAR(u16_InxMode32_Prev_Cmd_Valid) & 0x0002))
               {
                  // Read the high-byte OPMODE command
                  s64_temp = BGVAR(s64_InXMode32_Prev_Cmd_Of_Hi_Byte);
                  // Restore the command value
                  s16_restore_command_value = 1;
               }
            }

            // If the command value is supposed to be restored
            if(s16_restore_command_value == 1)
            {
               // Restore the actual command value depending on the OPMODE
               switch(VAR(AX0_s16_Opmode))
               {
                  case(0):
                     VAR(AX0_s16_Serial_Vel_Cmnd) = (int)s64_temp;
                  break;

                  case(2):
                     VAR(AX0_s16_Serial_Crrnt_Cmnd) = (int)s64_temp;
                  break;

                  case(4):
                     // Update only if ACC/DEC/VLIM is being applied in electronic gearing,
                     // because otherwise we introduce an uncontrolled jump when setting the
                     // electronic gearing position.
                     if((VAR(AX0_u8_Gear_Limits_Mode) &  0x0004))
                     {
                        LLVAR(AX0_u32_Gear_Not_Lim_Pos_Lo) = s64_temp;
                     }
                  break;

                  case(8):
                     // Trigger a motion task only in case that the variable "BGVAR(s32_Ptp_Move_Trgt_Vel)"
                     // is unequal 0, which is an indication that the function "ExecuteMoveCommandInternal"
                     // is at least one time called and therefore a motion task was triggered
                     if(BGVAR(s32_Ptp_Move_Trgt_Vel) != 0)
                     {
                        InitiatePtpMove(drive, s64_temp, (BGVAR(s32_Ptp_Move_Trgt_Vel) << 1), BGVAR(s32_Ptp_Acc_Rounded), BGVAR(s32_Ptp_Dec_Rounded), PTP_IMMEDIATE_TRANSITION);
                     }
                  break;

                  default:
                     // Do nothing
                  break;
               }
            }

         }

         // Go back to initial stage
         BGVAR(u16_InXMode32_Input_State) = OPMODE_SWITCH_INPUT_STATE_IDLE;
      break;

      case OPMODE_SWITCH_INPUT_STATE_RESET_STATE_MACHINE:
      default: // Not supposed to happen, illegal state.
         BGVAR(u16_InxMode32_Prev_Cmd_Valid) = 0;   // No valid data inside the variables "BGVAR(s64_InXMode32_Prev_Cmd_Of_Lo_Byte)" & "BGVAR(s64_InXMode32_Prev_Cmd_Of_Hi_Byte)"
         BGVAR(s64_InXMode32_Prev_Cmd_Of_Lo_Byte) = 0;
         BGVAR(s64_InXMode32_Prev_Cmd_Of_Hi_Byte) = 0;
         BGVAR(u16_InXMode32_Input_State) = OPMODE_SWITCH_INPUT_STATE_IDLE;
      break;
   }
}

int IsMotorInPosOnHold (int drive)
{
   // AXIS_OFF;
   long long s64_pe = 0LL, s64_pfb = 0LL;
   REFERENCE_TO_DRIVE;
   
   s64_pfb = LLVAR(AX0_u32_Pos_Fdbk_Dual_Lo);
   if (s64_pfb > BGVAR(s64_Pause_Inp_General_Purpose_1))
      s64_pe = s64_pfb - BGVAR(s64_Pause_Inp_General_Purpose_1); //calc pe = |pfb - target pos| (before HOLD)
   else
      s64_pe = BGVAR(s64_Pause_Inp_General_Purpose_1) - s64_pfb; 
	  
   return (s64_pe < LLVAR(AX0_u32_Inpos_Threshold_Lo)); //return true if pe < peinpos   
}

//**********************************************************
// Function Name: ProcessPauseInput
// Description:
//    This function processes the input of the Pause functionality.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void ProcessPauseInput(int drive)
{
   // AXIS_OFF;

   // In case no input is assigned to PAUSE_INPUT OR
   // Drive is in a disable state OR
   // A fieldbus is running the Drive
   if( ((IsInFunctionalityConfigured(PAUSE_INP,drive)) == 0) ||
       (!Enabled(drive))                                     ||
       (BGVAR(u8_Comm_Mode) == 1)
     )
   {
      BGVAR(u16_Paused_State) = PAUSE_STATE_RESET_STATE_MACHINE;
      return;
   }

   // Check if the OPMODE has been changed while a Hold/Resume is pending
   if(BGVAR(u16_Interrupted_Motion) != PAUSE_INPUT_NO_MOTION_INTERRUPTED)
   {
      if( ((BGVAR(u16_Interrupted_Motion) == PAUSE_INPUT_JOG_MOVE_INTERRUPTED) && (VAR(AX0_s16_Opmode) != 0)) ||
          ((BGVAR(u16_Interrupted_Motion) == PAUSE_INPUT_PTP_MOVE_INTERRUPTED) && (VAR(AX0_s16_Opmode) != 8)) ||
          ((BGVAR(u16_Interrupted_Motion) == PAUSE_INPUT_HOMING_INTERRUPTED)   && (VAR(AX0_s16_Opmode) != 8))
        )
      {
         BGVAR(u16_Paused_State) = PAUSE_STATE_RESET_STATE_MACHINE;
      }
   }

   switch(BGVAR(u16_Paused_State))
   {
      case PAUSE_STATE_IDLE:   // Evaluate the action
       /*  if(BGVAR(u16_Prev_Pause_Input_Lvl) == 0x7FFF) // If the previous level was not yet determined (e.g. after power-up)
         {
            BGVAR(u16_Prev_Pause_Input_Lvl) = VAR(AX0_u16_Mode_Input_Arr[PAUSE_INP]);
         }
         else*/
         if(BGVAR(u16_Prev_Pause_Input_Lvl) != VAR(AX0_u16_Mode_Input_Arr[PAUSE_INP])) // Edge detection
         {           
            BGVAR(u16_Homing_Sm_Took_Reset) = 0; // if a change occured on HOLD - reset HOMING SM               
            if(VAR(AX0_u16_Mode_Input_Arr[PAUSE_INP]) == 0) // If falling edge -> Stop motion via activate a hold
            {
               BGVAR(u16_Paused_State) = PAUSE_STATE_ACTIVATE_HOLD;
            }
            else // Rising edge -> Terminate HOLD and possibly resume motion
            {
               BGVAR(u16_Paused_State) = PAUSE_STATE_DEACTIVATE_HOLD;
            }
            BGVAR(u16_Prev_Pause_Input_Lvl) = VAR(AX0_u16_Mode_Input_Arr[PAUSE_INP]);
         }
         
         if ((PAUSE_INPUT_PTP_MOVE_INTERRUPTED == BGVAR(u16_Interrupted_Motion)) || (PAUSE_INPUT_HOMING_INTERRUPTED == BGVAR(u16_Interrupted_Motion)))
         {
            if (IsHomingProcessRunning(drive)) //for the case that HOLD was activated before HOMECMD given
            {               
               BGVAR(u16_Interrupted_Motion) = PAUSE_INPUT_HOMING_INTERRUPTED; // Indicate that a homing has been interrupted
               if (!BGVAR(u16_Homing_Sm_Took_Reset))
               {
                  u16_Homing_Sm_Took_Reset = 1;
                  // first reset the homing process
                  STORE_EXECUTION_PARAM_0
                  s16_Number_Of_Parameters = 1;
                  s64_Execution_Parameter[0] = 0;
                  HomeCommand(drive);
                  RESTORE_EXECUTION_PARAM_0
               }
            }
            else if (IsMotorInPosOnHold(drive)) // for the case that target is achieved although HOLD was raised
            {
               VAR(AX0_s16_Stopped) = 2;
               BGVAR(u16_Paused_State) = PAUSE_STATE_RESET_STATE_MACHINE;
            }
         }
      break;

      case PAUSE_STATE_ACTIVATE_HOLD:
         if(VAR(AX0_s16_Opmode) == 0)
         {
            BGVAR(u16_Interrupted_Motion) = PAUSE_INPUT_JOG_MOVE_INTERRUPTED; // Indicate that a jog move in OPMODE 0 has been interrupted
            BGVAR(s64_Pause_Inp_General_Purpose_1) = (long long)VAR(AX0_s16_Serial_Vel_Cmnd); // Save the current internal velocity command
         }
         else if(VAR(AX0_s16_Opmode) == 8)
         {
            // If a homing procedure is currently running
            if (IsHomingProcessRunning(drive))
            {
               BGVAR(u16_Interrupted_Motion) = PAUSE_INPUT_HOMING_INTERRUPTED; // Indicate that a homing has been interrupted
            }
            else
            {
               BGVAR(s64_Pause_Inp_General_Purpose_1) = LLVAR(AX0_u32_Target_Position_Lo);
               BGVAR(s32_Pause_Inp_General_Purpose_2) = LVAR(AX0_s32_Vlimit);
               BGVAR(u16_Interrupted_Motion) = PAUSE_INPUT_PTP_MOVE_INTERRUPTED; // Indicate that a regular motion task has been interrupted
            }
         }
         else
         {
            BGVAR(u16_Interrupted_Motion) = PAUSE_INPUT_NO_MOTION_INTERRUPTED;
         }

         // Stop motion only in case that OPMODE 0 and 8 (PTP-move or homing) is running
         if(BGVAR(u16_Interrupted_Motion) != PAUSE_INPUT_NO_MOTION_INTERRUPTED)
         {
            BGVAR(u16_Deceleration_Event_ID) = EVENT_DECSTOP;
            BGVAR(u16_Hold_Bits) |= HOLD_PAUSE_INPUT_MASK;
            BGVAR(u16_Paused_State) = PAUSE_STATE_IDLE;
         }
      break;

      case PAUSE_STATE_DEACTIVATE_HOLD:

         if(BGVAR(u16_Hold_Bits) & HOLD_PAUSE_INPUT_MASK) // Only if a HOLD caused by the input is really active
         {
            BGVAR(u16_Hold_Bits) &= ~HOLD_PAUSE_INPUT_MASK;

            if(BGVAR(u16_Hold_Mode) == 0)
            {
               // If an interrupted homing will not be resumed
               if(BGVAR(u16_Interrupted_Motion) == PAUSE_INPUT_HOMING_INTERRUPTED)
               {
                  // Let the homing state machine switch to homing failure.
                  BGVAR(u16_Switch_To_Homing_Failed) = 1;
               }

               BGVAR(u16_Paused_State) = PAUSE_STATE_IDLE;
            }
            else
            {
               /*// If an interrupted homing will be resumed
               if(BGVAR(u16_Interrupted_Motion) == PAUSE_INPUT_HOMING_INTERRUPTED)
               {
                  // first reset the homing process
                  STORE_EXECUTION_PARAM_0
                  s16_Number_Of_Parameters = 1;
                  s64_Execution_Parameter[0] = 0;
                  HomeCommand(drive);
                  RESTORE_EXECUTION_PARAM_0
               }*/

               BGVAR(u16_Paused_State) = PAUSE_STATE_RESUME_MOTION;
            }
         }
         else // If a hold caused by the digital input is not active
         {
            // Just jump to idle state
            BGVAR(u16_Paused_State) = PAUSE_STATE_IDLE;
         }
      break;

      case PAUSE_STATE_RESUME_MOTION:
         if(!IsHoldActive(drive)) // Wait if hold has been terminated
         {
            if(BGVAR(u16_Interrupted_Motion) == PAUSE_INPUT_JOG_MOVE_INTERRUPTED) // A jog move in OPMODE 0 has been interrupted
            {
               // Restore the current internal velocity command
               VAR(AX0_s16_Serial_Vel_Cmnd) = (int)BGVAR(s64_Pause_Inp_General_Purpose_1);
            }
            else if(BGVAR(u16_Interrupted_Motion) == PAUSE_INPUT_PTP_MOVE_INTERRUPTED) // A regular motion task has been interrupted
            {
               // Initiate the previous PTP move
               InitiatePtpMove(drive, BGVAR(s64_Pause_Inp_General_Purpose_1), BGVAR(s32_Pause_Inp_General_Purpose_2), BGVAR(s32_Ptp_Acc_Rounded), BGVAR(s32_Ptp_Dec_Rounded), PTP_IMMEDIATE_TRANSITION);
            }
            else if(BGVAR(u16_Interrupted_Motion) == PAUSE_INPUT_HOMING_INTERRUPTED) // A homing has been interrupted
            {
               // Restart the homing
               STORE_EXECUTION_PARAM_0;
               s16_Number_Of_Parameters = 0;
               HomeCommand(drive);
               RESTORE_EXECUTION_PARAM_0;
            }
            BGVAR(u16_Paused_State) = PAUSE_STATE_IDLE;
            BGVAR(u16_Interrupted_Motion) = PAUSE_INPUT_NO_MOTION_INTERRUPTED;
         }
      break;

      case PAUSE_STATE_RESET_STATE_MACHINE:
      default: // Not supposed to happen, illegal state
         BGVAR(u16_Prev_Pause_Input_Lvl) = 0x7FFF;                            // Put variable back to init-state
         BGVAR(u16_Hold_Bits) &= ~HOLD_PAUSE_INPUT_MASK;                      // Release a potential hold
         BGVAR(u16_Interrupted_Motion) = PAUSE_INPUT_NO_MOTION_INTERRUPTED;   // Reset the variable that indicates the interrupted motion
         BGVAR(s64_Pause_Inp_General_Purpose_1) = 0;                          // Clear the general purpose variable 1
         BGVAR(s32_Pause_Inp_General_Purpose_2) = 0;                          // Clear the general purpose variable 2
         BGVAR(u16_Paused_State) = PAUSE_STATE_IDLE;                          // Reset the state machine
      break;
   }
}

//**********************************************************
// Function Name: ProcessScriptTriggerInput
// Description:
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
void ProcessScriptTriggerInput(int drive)
{
   // AXIS_OFF;
   unsigned int u16_present, u16_present_Sec, u16_Index = 0, u16_Script_Trigger_Mode_Flag = 0;
   static unsigned int u16_First_Pass = 0, u16_Script_Inmode_Update = 0;
   REFERENCE_TO_DRIVE;
   
   u16_present=VAR(AX0_u16_Mode_Input_Arr[SCRIPT_INP]);
   u16_present_Sec=VAR(AX0_u16_Mode_Input_Arr[SCRIPT_SEC_INP]);

   if (u16_First_Pass == 0)
   {
      BGVAR(u16_Prev_Script_Inp_State) = u16_present;
      u16_First_Pass = 1;
      return;
   }

   for (u16_Index = 1; u16_Index <= (unsigned int)s16_Num_Of_Inputs; u16_Index++)
   {  // look for script trigger assign
      if (VAR(AX0_u16_Input_Mode_Arr[u16_Index]) == SCRIPT_INP || VAR(AX0_u16_Input_Mode_Arr[u16_Index]) == SCRIPT_SEC_INP)
         u16_Script_Trigger_Mode_Flag++;
   }

   if (u16_Script_Trigger_Mode_Flag)
   {// script
      if (u16_Script_Inmode_Update == 0)
      {
         u16_Script_Inmode_Update = 1;
         BGVAR(u16_Prev_Script_Inp_State) = u16_present;
         BGVAR(u16_Prev_Script_Sec_Inp_State) = u16_present_Sec;
         return;
      }
   }
   else
   {
      u16_Script_Inmode_Update = 0;
      BGVAR(u16_Prev_Script_Inp_State) = u16_present;
      BGVAR(u16_Prev_Script_Sec_Inp_State) = u16_present_Sec;
   }

   if (u16_Script_Inmode_Update == 1)
   {
      if (BGVAR(u16_Prev_Script_Inp_State) < u16_present)                 //Rising edge
      {         //Storing the command input values
         BGVAR(u16_Script_Rise_cmd) = (unsigned int)((VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_0_INP])>0?1:0) | (VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_1_INP])>0?2:0) | (VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_2_INP])>0?4:0) | (VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_3_INP]) > 0?8:0) | (VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_4_INP]) > 0?16:0)) ;
         BGVAR(s16_Script_Edge_State) = 1;
      }

      else if (BGVAR(u16_Prev_Script_Inp_State) > u16_present)            //Falling edge
      {         //Storing the command input values
         BGVAR(u16_Script_Fall_cmd) = (unsigned int)((VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_0_INP]) > 0?1:0) | (VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_1_INP])>0?2:0) | (VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_2_INP])>0?4:0) | (VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_3_INP])>0?8:0) | (VAR(AX0_u16_Mode_Input_Arr[SCRIPT_BIT_4_INP]) > 0?16:0)) ;
         BGVAR(s16_Script_Edge_State) = -1;
      }

      if (BGVAR(u16_Prev_Script_Sec_Inp_State) < u16_present_Sec)
      {         //Storing the command input values
         BGVAR(u16_Script_Rise_cmd) = (unsigned int)(SCRIPT_BIT_MAX);
         BGVAR(s16_Script_Edge_State) = 1;
      }

      else if (BGVAR(u16_Prev_Script_Sec_Inp_State) > u16_present_Sec)    //Falling edge
      {         //Storing the command input values
         BGVAR(u16_Script_Fall_cmd) = (unsigned int)(SCRIPT_BIT_MAX);
         BGVAR(s16_Script_Edge_State) = -1;
      }
   }

   BGVAR(u16_Prev_Script_Inp_State) = u16_present;
   BGVAR(u16_Prev_Script_Sec_Inp_State)= u16_present_Sec;
}

//**********************************************************
// Function Name: ProcessDualModeInput
// Description:
//    This function processes the dual mode inputs for "on the fly" mode change
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void ProcessDualModeInput(int drive)
{
   // AXIS_OFF;
   int s16_cdhd_opmode_dual=0;
   int s16_on_access_rights_func;   // access rights fucntionality when dual input set to on
   int s16_off_access_rights_func;  // access rights fucntionality when dual input set to on
   int s16_allow_opmode_change = 0;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // if JOG state machine is in process don't alow mode change wait for JOG mode to finish
   if (BGVAR(u16_JogInputState) != 0) return;
   if (BGVAR(u16_AD_State) != AD_IDLE)
   {
      return;
   }


   switch (BGVAR(u16_P1_01_CTL_Current) & 0xFF)
   {
       case SE_OPMODE_PT_S:// S-Pt dual mode (speed to gearing)
            BGVAR(u16_LXM28_Off_State_Opmode) = SE_OPMODE_S;
            // Figure out if the we are supposed to work in CDHD OPMODE 0 or 1 for Lexium Mode S.
            // This needs to be done for an OPMODE change on the fly in order to avoid that the functions
            // "ProcessVelocityCommand" and "ProcessTorqueCommand" need to correct the new OPMODE.
            if((VAR(AX0_u16_Mode_Input_Arr[SPD_CMD_SELECT_0_INP]) != 0) || (VAR(AX0_u16_Mode_Input_Arr[SPD_CMD_SELECT_1_INP]) != 0))
            {
               BGVAR(s16_CDHD_Off_State_Opmode)  = 0;
            }
            else
            {
            BGVAR(s16_CDHD_Off_State_Opmode)  = 1;
            }
            BGVAR(u16_LXM28_On_State_Opmode)  = SE_OPMODE_PT;
            BGVAR(s16_CDHD_On_State_Opmode)   = 4;
            BGVAR(u16_DualModeInputToScan)    = S_P_INP;
            s16_on_access_rights_func  = LEX_AR_FCT_ACTIVATE_PT;
            s16_off_access_rights_func = LEX_AR_FCT_ACTIVATE_S;
       break;

       case SE_OPMODE_PT_T:// T-Pt dual mode (torque to gearing)
            BGVAR(u16_LXM28_Off_State_Opmode) = SE_OPMODE_T;
            // Figure out if the we are supposed to work in CDHD OPMODE 2 or 3 for Lexium Mode T.
            // This needs to be done for an OPMODE change on the fly in order to avoid that the functions
            // "ProcessVelocityCommand" and "ProcessTorqueCommand" need to correct the new OPMODE.
            if(((VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_0_INP]) & 0xFF) != 0) && ((VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_1_INP]) & 0xFF) != 0))
            {
               BGVAR(s16_CDHD_Off_State_Opmode)   = 2;
            }
            else
            {
            BGVAR(s16_CDHD_Off_State_Opmode)  = 3;
            }
            BGVAR(u16_LXM28_On_State_Opmode)  = SE_OPMODE_PT;
            BGVAR(s16_CDHD_On_State_Opmode)   = 4;
            BGVAR(u16_DualModeInputToScan)    = T_P_INP;

            s16_on_access_rights_func  = LEX_AR_FCT_ACTIVATE_PT;
            s16_off_access_rights_func = LEX_AR_FCT_ACTIVATE_T;
       break;

       case SE_OPMODE_PS_S:// S-PS dual mode (speed to position)
            BGVAR(u16_LXM28_Off_State_Opmode) = SE_OPMODE_S;
            // Figure out if the we are supposed to work in CDHD OPMODE 0 or 1 for Lexium Mode S.
            // This needs to be done for an OPMODE change on the fly in order to avoid that the functions
            // "ProcessVelocityCommand" and "ProcessTorqueCommand" need to correct the new OPMODE.
            if((VAR(AX0_u16_Mode_Input_Arr[SPD_CMD_SELECT_0_INP]) != 0) || (VAR(AX0_u16_Mode_Input_Arr[SPD_CMD_SELECT_1_INP]) != 0))
            {
               BGVAR(s16_CDHD_Off_State_Opmode)  = 0;
            }
            else
            {
               BGVAR(s16_CDHD_Off_State_Opmode)  = 1;
            }
            BGVAR(u16_LXM28_On_State_Opmode)  = SE_OPMODE_PS;
            BGVAR(s16_CDHD_On_State_Opmode)   = 8;
            BGVAR(u16_DualModeInputToScan)    = S_P_INP;

            s16_on_access_rights_func  = LEX_AR_FCT_ACTIVATE_PS;
            s16_off_access_rights_func = LEX_AR_FCT_ACTIVATE_S;
       break;

       case SE_OPMODE_PS_T:// T-PS dual mode (torque to position)
            BGVAR(u16_LXM28_Off_State_Opmode) = SE_OPMODE_T;
            // Figure out if the we are supposed to work in CDHD OPMODE 2 or 3 for Lexium Mode T.
            // This needs to be done for an OPMODE change on the fly in order to avoid that the functions
            // "ProcessVelocityCommand" and "ProcessTorqueCommand" need to correct the new OPMODE.
            if(((VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_0_INP]) & 0xFF) != 0) && ((VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_1_INP]) & 0xFF) != 0))
            {
               BGVAR(s16_CDHD_Off_State_Opmode)   = 2;
            }
            else
            {
               BGVAR(s16_CDHD_Off_State_Opmode)   = 3;
            }
            BGVAR(u16_LXM28_On_State_Opmode)  = SE_OPMODE_PS;
            BGVAR(s16_CDHD_On_State_Opmode)   = 8;
            BGVAR(u16_DualModeInputToScan)    = T_P_INP;

            s16_on_access_rights_func  = LEX_AR_FCT_ACTIVATE_PS;
            s16_off_access_rights_func = LEX_AR_FCT_ACTIVATE_T;
       break;

       case SE_OPMODE_S_T:// S-T dual mode
            BGVAR(u16_LXM28_Off_State_Opmode) = SE_OPMODE_S;
            // Figure out if the we are supposed to work in CDHD OPMODE 0 or 1 for Lexium Mode S.
            // This needs to be done for an OPMODE change on the fly in order to avoid that the functions
            // "ProcessVelocityCommand" and "ProcessTorqueCommand" need to correct the new OPMODE.
            if((VAR(AX0_u16_Mode_Input_Arr[SPD_CMD_SELECT_0_INP]) != 0) || (VAR(AX0_u16_Mode_Input_Arr[SPD_CMD_SELECT_1_INP]) != 0))
            {
               BGVAR(s16_CDHD_Off_State_Opmode)  = 0;
            }
            else
            {
               BGVAR(s16_CDHD_Off_State_Opmode)  = 1;
            }
            BGVAR(u16_LXM28_On_State_Opmode)  = SE_OPMODE_T;
            // Figure out if the we are supposed to work in CDHD OPMODE 2 or 3 for Lexium Mode T.
            // This needs to be done for an OPMODE change on the fly in order to avoid that the functions
            // "ProcessVelocityCommand" and "ProcessTorqueCommand" need to correct the new OPMODE.
            if((VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_0_INP]) != 0) || (VAR(AX0_u16_Mode_Input_Arr[TRQ_CMD_SELECT_1_INP]) != 0))
            {
               BGVAR(s16_CDHD_On_State_Opmode)   = 2;
            }
            else
            {
            BGVAR(s16_CDHD_On_State_Opmode)   = 3;
            }
            BGVAR(u16_DualModeInputToScan)    = S_T_INP;

            s16_on_access_rights_func  = LEX_AR_FCT_ACTIVATE_T;
            s16_off_access_rights_func = LEX_AR_FCT_ACTIVATE_S;
       break;

       case SE_OPMODE_PT_PR:
            BGVAR(u16_LXM28_Off_State_Opmode) = SE_OPMODE_PT;
            BGVAR(s16_CDHD_Off_State_Opmode)  = 4;
            BGVAR(u16_LXM28_On_State_Opmode)  = SE_OPMODE_PS;
            BGVAR(s16_CDHD_On_State_Opmode)   = 8;
            BGVAR(u16_DualModeInputToScan)    = Pt_PS_INP;

            s16_on_access_rights_func  = LEX_AR_FCT_ACTIVATE_PS;
            s16_off_access_rights_func = LEX_AR_FCT_ACTIVATE_PT;
       break;

       default:
            return;
    //   break;
   }


   if (BGVAR(u16_DualModeState) == 0)   // not in middle of mode change
   {
      // if in PT mode and HALT io is on, allow changing =s= mode
      // because HALT is applicable only for PT mode and should be cleared when switching to the other mode.
      if ((VAR(AX0_u16_Mode_Input_Arr[HALT_INP]) != 0) && (BGVAR(u16_LXM28_Opmode) == SE_OPMODE_PT))
      {
         s16_allow_opmode_change = 1;
      }

      // dont change opmode when hold is active since it may lead to runaway (e.g. if changing to opmode 3)
      // currentyl hold is not checked inside the loops
      if ((BGVAR(u16_Hold_Bits) == 0) || (s16_allow_opmode_change == 1))
      {
         // switch is off
         if (VAR(AX0_u16_Mode_Input_Arr[BGVAR(u16_DualModeInputToScan)]) == 0)
         {
            // opmode does not match the expected off mode OR access rights has been just released
            if ((BGVAR(u16_LXM28_Opmode) != BGVAR(u16_LXM28_Off_State_Opmode)) || (BGVAR(u16_Lexium_Acces_Rights_Released) == 1))
            {
               // Check if acces-rights are granted
               if (CheckLexiumAccessRights(drive, s16_off_access_rights_func, LEX_CH_IO_CONTROL) == SAL_SUCCESS)
               {
                  // change to switch off mode
                  BGVAR(s16_Dual_Mode_Switched_On) = 0;
                  BGVAR(u16_DualModeState)=1;
               }
            }
         }
         // switch is on
         else if (VAR(AX0_u16_Mode_Input_Arr[BGVAR(u16_DualModeInputToScan)]) != 0)
         {
            // opmode does not match the expected on mode OR access rights has been just released
            if ((BGVAR(u16_LXM28_Opmode) != BGVAR(u16_LXM28_On_State_Opmode)) || (BGVAR(u16_Lexium_Acces_Rights_Released) == 1))
            {
               // Check if acces-rights are granted
               if (CheckLexiumAccessRights(drive, s16_on_access_rights_func, LEX_CH_IO_CONTROL) == SAL_SUCCESS)
               {
                  // change to switch on mode
                  BGVAR(s16_Dual_Mode_Switched_On) = 1;
                  BGVAR(u16_DualModeState)=1;
               }
            }
         }

         BGVAR(u16_Lexium_Acces_Rights_Released) = 0;
      }
   }

   // preform the mode change state machine
   switch(BGVAR(u16_DualModeState))
   {
       case 0:
       break;

       case 1:// activate the OPMODE change background handler

            if(BGVAR(s16_Dual_Mode_Switched_On) == 1)
            {// switched on
                 BGVAR(u16_LXM28_Opmode) = BGVAR(u16_LXM28_On_State_Opmode);
                 s16_cdhd_opmode_dual    = BGVAR(s16_CDHD_On_State_Opmode);
            }
            else if(BGVAR(s16_Dual_Mode_Switched_On) == 0)
            {// switched off
                 BGVAR(u16_LXM28_Opmode) = BGVAR(u16_LXM28_Off_State_Opmode);
                 s16_cdhd_opmode_dual    = BGVAR(s16_CDHD_Off_State_Opmode);
            }
            // Start the OPMODE background handler and indicate that it was the dual-loop input
            // that changed the Opmode
            BGVAR(s16_Start_Opmode_Change) = (s16_cdhd_opmode_dual | OPMODE_CHANGE_SOURCE_DUAL_MODE_IO);

            BGVAR(u16_DualModeState)++;
       break;

       case 2:
            // fix bugzilla 4484: Hold not released when change from Pt to V/T when Drive Halted in Pt mode.
            // if opmode change finished, and current mode is not PT, verify HALT is off.
            if (!IsOpmodeChangeInProgress(drive))
            {
               if (BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PT)
               {
                  if (BGVAR(u16_Hold_Bits) & HOLD_USER_MASK)
                  {
                     // if the selected opmode is NOT PT and the drive is not in the middle of opmode change and hold is set (due to HALT io)
                     // then release the hold to allow motion.
                     BGVAR(u16_Hold_Bits) &= ~HOLD_USER_MASK;
                  }
               }

            BGVAR(u16_DualModeState)++;
            }
       break;

       default:// reset state machine back to idle
            BGVAR(s16_Dual_Mode_Switched_On)=-1;
            BGVAR(u16_DualModeState)=0;
       break;
   }
}


//**********************************************************
// Function Name: SalJogCmdCommand
// Description:
//    Trigger Jog for Modbus. This command is called upon P-Parameter P10-25.
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalJogCmdCommand(long long value, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Check if currently an auto-tuning process or a internal profile process is
   // running. This code has been introduced due to "BYang00000752" & "BYang00000753",
   // in which Juergen Fiess starts several kinds of motion with his commissioning tool
   // without aborting a different currently pending motion.
   if( ((BGVAR(u16_Lexium_Autotune_State) != LEX_ATUNE_STATE_IDLE) && (BGVAR(u16_Lexium_Autotune_State) < LEX_ATUNE_STATE_A_ADAPT_ACIVATE)) ||
       (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_IDLE) )
   {
      return OTHER_PROCEDURE_RUNNING;
   }

   BGVAR(s16_P10_25_Jog_Cmd) = (long)value;
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: ProcessJogInputs
// Description:
//    This function processes the JOGD and JOGU DIN. if opmode!=CANopen
//    these digital inputs will take priority over torque and vel commands.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void ProcessJogInputs(int drive)
{
   // AXIS_OFF;

   int s16_jogCmd = 0;
   unsigned int u16_jogu_input = VAR(AX0_u16_Mode_Input_Arr[JOGU_INP]); // Read digital input state into a temporary variable.
   unsigned int u16_jogd_input = VAR(AX0_u16_Mode_Input_Arr[JOGD_INP]); // Read digital input state into a temporary variable.
   int s16_temp_P10_25 = BGVAR(s16_P10_25_Jog_Cmd);                     // Read the jog move command into a temporary variable

   // If no Schneider Drive or software
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;
   // if Dual mode change state machine is in process don't alow JOG mode wait for mode change to finish
   if (BGVAR(u16_DualModeState) != 0) return;

   // Now check the access-rights for the Lexium digital input jog move at this place.
   // If the digital inputs, which triggers a jog-move, have NO access-rights.
   if(CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_JOG_MODE, LEX_CH_IO_CONTROL) != SAL_SUCCESS)
   {
      u16_jogu_input = u16_jogd_input = 0; // Clear the temporary input-state variables so they are not considered.
   }

   // Check if currently an auto-tuning process or a internal profile process is
   // running. This code has been introduced due to "BYang00000752" & "BYang00000753",
   // in which Juergen Fiess starts several kinds of motion with his commissioning tool
   // without aborting a different currently pending motion.
   if( ((BGVAR(u16_Lexium_Autotune_State) != LEX_ATUNE_STATE_IDLE) && (BGVAR(u16_Lexium_Autotune_State) < LEX_ATUNE_STATE_A_ADAPT_ACIVATE)) ||
       (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_IDLE) )
   {
      u16_jogu_input = u16_jogd_input = 0; // Clear the temporary input-state variables so they are not considered.
   }

   // Now check the access-rights for the Lexium commissioning tool at this place.
   // If the commissioning tool, which triggers a jog-move, has NO access-rights.
   if(CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_JOG_MODE, LEX_CH_MODBUS_RS485) != SAL_SUCCESS)
   {
      s16_temp_P10_25 = 0; // Clear the temporary jog-state variable so it is not considered.
   }

   if ((u16_jogu_input) || (s16_temp_P10_25 == 1))
   {
      s16_jogCmd = 1; // Indicate positive (slow) motion
   }
   else if ((u16_jogd_input) || (s16_temp_P10_25 == 2))
   {
      s16_jogCmd = 2; // Indicate negative (slow) motion
   }

   switch(BGVAR(u16_JogInputState))
   {
      case (0):
         if(s16_jogCmd != 0) // If jog motion is requested
         {
            // Setup CDHD parameters for Lexium jog move
            PrepareCdhdJogMoveVariables(drive, 1);
            BGVAR(u16_JogInputState)++; // Jump to next state
         }
      break;

      case (1):
         if(s16_jogCmd != 0) // If jog motion is requested
         {
            // Check if both digital inputs are set to true
            if((u16_jogu_input!=0) && (u16_jogd_input!=0))
            {
               // If both inputs are true, then initiate the JOG-move handler to come to a stop.
               BGVAR(u16_Lexium_Jog_Move_Activate) = 0;
            }
            else
            {
               // Here initiate jog in a certain direction
               BGVAR(u16_Lexium_Jog_Move_Activate) = s16_jogCmd;
            }
         }
         else // Variable is 0
         {
            BGVAR(u16_JogInputState)++; // Jump to next state
         }
      break;

      default:
         // Tell the jog move handler to come to a stop
         BGVAR(u16_Lexium_Jog_Move_Activate) = 0;
         // If the PTO hunter reached velocity 0, so if he came to a stop. Otherwise we would switch
         // The mode of operation during the deceleration process, which is not desired. Gil identified
         // this issue.
         if(LVAR(AX0_s32_Pos_Vcmd) == 0L)
         {
            // First restore the original settings (undo the changes when stepping out of JOG mode)
            PrepareCdhdJogMoveVariables(drive, 0);
            // Go back to original state
            BGVAR(u16_JogInputState) = 0;
         }
      break;
   }
}

// s16_P1_38_Zero_Speed_Window is in 0.1*RPM units.
// convert s32_V_Lim_Design to RPM units.
// return value of s16_P1_38_Zero_Speed_Window in volts (deadband internal units)
int ConvertZeroSpeedToAnalogUnits(int drive)
{
   // AXIS_OFF;

   long long vlim_rpm;
   float tmp;
   REFERENCE_TO_DRIVE;
   // convert internal to rpm: vlim[rpm] =  vlim[internal] * 8000 * 60 / 2^32
   vlim_rpm = (long long)BGVAR(s32_V_Lim_Design) * 8000LL * 60LL / 4294967296LL;

   // add 0.5 for rounding
   vlim_rpm = (long long)((float)vlim_rpm + 0.5);
   tmp = (float)VAR(AX0_s16_P1_38_Zero_Speed_Window) / ((float)vlim_rpm * 10.0);

   // value[volt] = (P1-38[rpm/10] / (vlim_rpm[rpm] * 10) 22750 * aninVelScale_fix) >> aninVelScale_shift
   return ((long long)(tmp * 22750LL * BGVAR(s16_Vel_To_Volt_Scale_Fix)) >> BGVAR(u16_Vel_To_Volt_Scale_Shr));
}


//**********************************************************
// Function Name: SalWriteSpeedCmdLimTableValueCommand
// Description:
//    This function writes the parameters P1-09 to P1-11
//    to an array
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteSpeedCmdLimTableValueCommand (int drive)
{
   long long index = s64_Execution_Parameter[0];
   long long value = s64_Execution_Parameter[1];

   if ((index < 0LL) || (index > 2LL))
       return (VALUE_OUT_OF_RANGE);

   // FIX BUG# 3718 (C.Q. 340)
   if (value < -60000LL) // 6000[rpm] = 60000[0.1*rpm]
      return (VALUE_TOO_LOW);

   if(value > 60000LL)
      return (VALUE_TOO_HIGH);

   BGVAR(s32_P1_09to11_Spd_Cmd_Or_Limit)[index] = (long)value;

   // Convert the values of s32_P1_09to11_Spd_Cmd_Or_Limit into internal units (in the loop)
   CalcSpeedCmdLimitTable(drive);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadSpeedCmdLimTableValueCommand
// Description:
//    This function reads the parameters P1-09 to P1-11
//    from an array.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadSpeedCmdLimTableValueCommand (long long *data,int drive)
{
   long long index = s64_Execution_Parameter[0];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((index < 0LL) || (index > 2LL))
       return (VALUE_OUT_OF_RANGE);

   *data = (BGVAR(s32_P1_09to11_Spd_Cmd_Or_Limit)[index]);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalWriteTorqueCmdLimTableValueCommand
// Description:
//    This function writes the param P1-12 to P1-14
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteTorqueCmdLimTableValueCommand(int drive)
{
   long long index = s64_Execution_Parameter[0];
   long long value = s64_Execution_Parameter[1];

   if ((index < 0LL) || (index > 2LL))
       return (VALUE_OUT_OF_RANGE);

   if (value < -300LL)
       return VALUE_TOO_LOW;

   if (value > 300LL)
       return (VALUE_TOO_HIGH);

   //    array starts from index 0
   BGVAR(s16_P1_12_Trq_Cmd_Lim)[index] = (int)value;
   CalcTorqueCmdLimitTable(drive);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadTorqueCmdLimTableValueCommand
// Description:
//    This function reads the param P1-12 to P1-14
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalReadTorqueCmdLimTableValueCommand(long long *data,int drive)
{
   long long index = s64_Execution_Parameter[0];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((index < 0LL) || (index > (long long)2))
       return (VALUE_OUT_OF_RANGE);

   *data = (BGVAR(s16_P1_12_Trq_Cmd_Lim)[index]);
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalWriteZeroSpeedWindowCommand
// Description:
//    This function writes the param P1-38 (ZSPD). user units are 0.1*rpm
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteZeroSpeedWindowCommand(long long value, int drive)
{
   // AXIS_OFF;
   long long s64_half_for_rounding = 0;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return NOT_PROGRAMMABLE;

   // convert from internal velocity (out of the loop) to internal units (in the loop)
   if (VAR(AX0_u16_Out_To_In_Vel_Shr) > 0)
      s64_half_for_rounding = 1LL << ((long long)VAR(AX0_u16_Out_To_In_Vel_Shr) - 1);

   VAR(AX0_s16_P1_38_Zero_Speed_Window) = (int)(((value * (long long)LVAR(AX0_s32_Out_To_In_Vel_Fix)) + s64_half_for_rounding) >> (long long)VAR(AX0_u16_Out_To_In_Vel_Shr));

   // P1-38 value in out loop units
   BGVAR(s32_P1_38_Zero_Speed_Window_Out_Loop) = (long)value;

   //VELOCITY_PTP_CONVERSION "out Loop"
   LVAR(AX0_s32_P1_38_Zero_Speed_Window_PTP) = BGVAR(s32_P1_38_Zero_Speed_Window_Out_Loop) << 1; // PTPVCMD units equal to 2 times velocity out loop units.

   // update disable mode if zero-speed P1-38 is changed.
   return SetSchneiderDismode(BGVAR(s16_P1_32_Stop_Mode), drive);
}


//**********************************************************
// Function Name: SalWriteSpeedReachedOutputRangeCommand
// Description:
//    This function writes the param P1-47 (SPOK). user units are 0.1*rpm
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteSpeedReachedOutputRangeCommand(long long value, int drive)
{
   // AXIS_OFF;
   long long s64_half_for_rounding = 0;
   REFERENCE_TO_DRIVE;
   // convert from internal velocity (out of the loop) to internal units (in the loop)
   if (VAR(AX0_u16_Out_To_In_Vel_Shr) > 0)
      s64_half_for_rounding = 1LL << ((long long)VAR(AX0_u16_Out_To_In_Vel_Shr) - 1);

   BGVAR(s32_P1_47_SPOK_In_Loop) = (long)(((value * (long long)LVAR(AX0_s32_Out_To_In_Vel_Fix)) + s64_half_for_rounding) >> (long long)VAR(AX0_u16_Out_To_In_Vel_Shr));

   // P1-47 value in out loop units
   BGVAR(s32_P1_47_SPOK_Out_Loop) = (long)value;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: ProcessCmdInvert
// Description:
//    This function processes the CMDINV DIN. if opmode==velocity or torque ,
//    if DIN CMDINV is set, invert the command in vel loop or current loop.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void ProcessCmdInvert(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (VAR(AX0_s16_Opmode) == 0 || VAR(AX0_s16_Opmode) == 1)
   {   // if velocity mode, flag is on bit 1
       VAR(AX0_s16_Cmd_Invert_On) = (VAR(AX0_u16_Mode_Input_Arr[CMD_INV_INP]) != 0) << 1;
   }
   else if (VAR(AX0_s16_Opmode) == 2 || VAR(AX0_s16_Opmode) == 3)
   {   // if current mode, flag is on bit zero
       VAR(AX0_s16_Cmd_Invert_On) = (VAR(AX0_u16_Mode_Input_Arr[CMD_INV_INP]) != 0);
   }
   else
   {   // disable the feature for all other opmodes
       VAR(AX0_s16_Cmd_Invert_On) = 0;
   }
}


//**********************************************************
// Function Name: ProcessGainSwitch
// Description:
//    This function is the gainSwitch trigger
//
// Author: Nitsan
// Algorithm:
// Revisions:Moshe
//**********************************************************
void ProcessGainSwitch(int drive)
{
   // AXIS_OFF;
//   static int gain_switch_on_prev = 0;
   int s16_temp, s16_ctrl_val = 0, s16_gain_switch_on = 0;
   long long s64_pe, s64_p2_29_pe_treshhold=0;
   long s32_vel, s32_delta_counts, s32_P2_29_ve_treshhold=0;
   unsigned long u32_freq_switch_threshold;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   s16_ctrl_val = BGVAR(u16_P2_27_Gain_Switching_Ctrl) & 0x0F ;//(autotune can not work together with gain switching, so when re-enabling this feature, need to block autotune);
   switch (s16_ctrl_val)
   {
      case 0:
       // Gain Switch feature is disabled.
      break;

      case 1:
         s16_gain_switch_on = (VAR(AX0_u16_Mode_Input_Arr[GAIN_UP_INP]) != 0);
      break;
      case 5:
         s16_gain_switch_on = (VAR(AX0_u16_Mode_Input_Arr[GAIN_UP_INP]) == 0);
      break;

      case 2:
      case 6:
         if (VAR(AX0_s16_Opmode) == 4 || VAR(AX0_s16_Opmode) == 8)
         {
            // convert P2-29 from position units (PULSE) to internal units
            s64_p2_29_pe_treshhold = MultS64ByFixS64ToS64((long long)(BGVAR(u32_P2_29_Gain_Switching_Condition)),
                                     BGVAR(Unit_Conversion_Table[POSITION_PULSE_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                     BGVAR(Unit_Conversion_Table[POSITION_PULSE_CONVERSION]).u16_unit_conversion_to_internal_shr);
            // compare P2-29 to position error
            do {
               s16_temp = Cntr_3125;
               s64_pe = LLVAR(AX0_u32_Int_Loop_Pos_Err_Lo);
            } while (s16_temp != Cntr_3125);
            if (s16_ctrl_val == 2)
               s16_gain_switch_on = (llabs(s64_pe) > s64_p2_29_pe_treshhold);
            else
               s16_gain_switch_on = (llabs(s64_pe) < s64_p2_29_pe_treshhold);
         }
      break;

      case 3:
      case 7:
         s32_delta_counts = BGVAR(s32_Encoder_Follower2_Counter_Delta_1ms);

         u32_freq_switch_threshold = BGVAR(u32_P2_29_Gain_Switching_Condition);

         // fix BUG#4118
         if(BGVAR(u16_Gear_In_Mode))
         {// if P8-30 (MULTIPLIER) is on multiply the freq threshold by 16
            u32_freq_switch_threshold <<= 4;
         }

         if((BGVAR(u16_P1_00_Ext_Pulse_Input_Type) & 0xf) == 0)
         {// if P1-00 nibble "A" == 0 -> A quad B so multiply the threshold freq by 4
            u32_freq_switch_threshold <<= 2;
         }

         if (s16_ctrl_val == 3) // counts in 1ms >  kpps
            s16_gain_switch_on = (labs(s32_delta_counts) > u32_freq_switch_threshold);
         else                   // counts in 1ms <  kpps
            s16_gain_switch_on = (labs(s32_delta_counts) < u32_freq_switch_threshold);
      break;

      case 4:
      case 8:
         // convert P2-29 from vel units (rpm) to internal units
         // mult speed by 1000 to match the parser unit conversion to velocity out of the loop
         s32_P2_29_ve_treshhold = MultS64ByFixS64ToS64((long long)(BGVAR(u32_P2_29_Gain_Switching_Condition)*1000L),
                                  BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                  BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

         // out of the loop conversion
         s32_vel = LVAR(AX0_s32_Vel_Var_Fb_0);
         if (s16_ctrl_val == 4)
              s16_gain_switch_on = (labs(s32_vel) > s32_P2_29_ve_treshhold);
         else
              s16_gain_switch_on = (labs(s32_vel) < s32_P2_29_ve_treshhold);
      break;
   }

   if (s16_gain_switch_on != BGVAR(u16_Gain_Switch_On_Prev))
   {
       // call gain switching function here.
       SwitchGain(s16_gain_switch_on , drive);
   }

   BGVAR(u16_Gain_Switch_On_Prev) = s16_gain_switch_on;
}


//**********************************************************
// Function Name: SwitchGain
// Description:
//    This function is the actual gain switcher
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void SwitchGain(int gain_switch_on ,int drive)
{
   // AXIS_OFF;

   unsigned long u32_local_kpi = BGVAR(u32_Gain_Switch_Kpi_SE),
                 u32_local_kpiv = BGVAR(u32_Gain_Switch_Kpiv_SE);

   float f_local_gain_switch_factor = BGVAR(f_Gain_Switch_Factor);

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   BGVAR(f_Gain_Switch_Factor) = 1.0;

   // check the Gain Switch Control Setting
   if(BGVAR(u16_P2_27_Gain_Switching_Ctrl) & 0x10)
   {// P2-27 nibble B is "1" - "P -> PI switching"
      if(gain_switch_on)// switch ON
      {
         BGVAR(u32_Gain_Switch_Kpiv_SE) = 1;
         BGVAR(u32_Gain_Switch_Kpi_SE)  = 1;
      }
      else// switch OFF if P2-27 == 0x10 before switch the integral is 0 and when switching the integral return to it's value.
      {
         if ((VAR(AX0_s16_Opmode) == 4) || (VAR(AX0_s16_Opmode) == 8))
         {// position
          // zero the integral
            BGVAR(u32_Gain_Switch_Kpiv_SE) = 0;
            BGVAR(u32_Gain_Switch_Kpi_SE)  = 0;
         }
         else if ((VAR(AX0_s16_Opmode) == 0) || (VAR(AX0_s16_Opmode) == 1))
         {// Speed
          // zero the integral
            BGVAR(u32_Gain_Switch_Kpiv_SE) = 0;
            BGVAR(u32_Gain_Switch_Kpi_SE)  = 1;
         }
         else
         {// not supported opmode
            BGVAR(u32_Gain_Switch_Kpiv_SE) = 1;
            BGVAR(u32_Gain_Switch_Kpi_SE)  = 1;
         }
      }
   }
   else
   {// P2-27 nibble B is "0" - "Gain multiple switching"
      if(gain_switch_on)// switch ON
      {
         if ((VAR(AX0_s16_Opmode) == 4) || (VAR(AX0_s16_Opmode) == 8))
         {// position
          // update gain but dont zero the integral
            // current value * P2-01(%) value
            BGVAR(f_Gain_Switch_Factor) = (float)(BGVAR(u16_P2_01_PPR)/100.0);
            BGVAR(u32_Gain_Switch_Kpiv_SE) = 1;
            BGVAR(u32_Gain_Switch_Kpi_SE)  = 1;
         }
         else if ((VAR(AX0_s16_Opmode) == 0) || (VAR(AX0_s16_Opmode) == 1))
         {// Speed
          // update gain and zero the integral
            // current value * P2-05(%) value
            BGVAR(f_Gain_Switch_Factor) = (float)(BGVAR(u16_P2_05_SPR)/100.0);
            BGVAR(u32_Gain_Switch_Kpiv_SE) = 0;
            BGVAR(u32_Gain_Switch_Kpi_SE)  = 1;
         }
         else
         {// not supported opmode
            BGVAR(u32_Gain_Switch_Kpiv_SE) = 1;
            BGVAR(u32_Gain_Switch_Kpi_SE)  = 1;
         }
      }
      else// switch OFF
      {// retrive original values to the control loop
         BGVAR(u32_Gain_Switch_Kpiv_SE) = 1;
         BGVAR(u32_Gain_Switch_Kpi_SE)  = 1;
      }
   }

   if(u32_local_kpi  != BGVAR(u32_Gain_Switch_Kpi_SE) ||
      u32_local_kpiv != BGVAR(u32_Gain_Switch_Kpiv_SE)||
      f_local_gain_switch_factor != BGVAR(f_Gain_Switch_Factor))
   {// call the position config only if one of the above changed in the current function call.
      // issue the new values into the control loop.
      PositionConfig(drive,0);
   }
}

//**********************************************************
// Function Name: SalWriteGainSwitchingCtrlCommand
// Description:
//    This function writes the param P2-27
//    valid values: lower byte: 0 to 8, higher byte: 0,1
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteGainSwitchingCtrlCommand(long long param, int drive)
{
   unsigned int u16_temp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u16_temp = (unsigned int)param;
   // check lower byte
   if ((u16_temp & 0x0f) > 8)
       return VALUE_OUT_OF_RANGE;
   // max valid value can be 0x18
   if (u16_temp > 0x18)
       return VALUE_OUT_OF_RANGE;

   BGVAR(u16_P2_27_Gain_Switching_Ctrl) = u16_temp;
   SwitchGain((int)BGVAR(u16_Gain_Switch_On_Prev), drive);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: GetExternalEncDelta
// Description:
//    This function calculates the delta of external encoder reading and the prev one, each 1 ms
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(GetExternalEncDelta, "ramfunc_2");
void GetExternalEncDelta(int drive)
{
   // AXIS_OFF;

   long s32_Follower2_Counter_Shadow = LVAR(AX0_s32_Encoder_Follower2_Counter);
   long s32_Follower3_Counter_Shadow = LVAR(AX0_s32_Encoder_Follower3_Counter);
   REFERENCE_TO_DRIVE;
   BGVAR(s32_Encoder_Follower2_Counter_Delta_1ms) = s32_Follower2_Counter_Shadow - BGVAR(s32_Encoder_Follower2_Counter_Prev);
   BGVAR(s32_Encoder_Follower2_Counter_Prev) = s32_Follower2_Counter_Shadow;
   BGVAR(s32_Encoder_Follower3_Counter_Delta_1ms) = s32_Follower3_Counter_Shadow - BGVAR(s32_Encoder_Follower3_Counter_Prev);
   BGVAR(s32_Encoder_Follower3_Counter_Prev) = s32_Follower3_Counter_Shadow;

   /***********************************************************************************/
   /* Special code for Lexium 28, some functions require the encoder delta per second */
   /***********************************************************************************/
   // Accumulate the 1[ms] delta
   BGVAR(s32_Encoder_Follower2_Counter_Accul) += BGVAR(s32_Encoder_Follower2_Counter_Delta_1ms);
   BGVAR(s32_Encoder_Follower3_Counter_Accul) += BGVAR(s32_Encoder_Follower3_Counter_Delta_1ms);
   // Increment timer
   BGVAR(u16_Encoder_Follower_1s_Timer)++;
   // After 1[s]
   if(BGVAR(u16_Encoder_Follower_1s_Timer) >= 1000)
   {
      // Store accumulated value
      BGVAR(s32_Encoder_Follower2_Counter_Delta_1s) = BGVAR(s32_Encoder_Follower2_Counter_Accul);
      BGVAR(s32_Encoder_Follower3_Counter_Delta_1s) = BGVAR(s32_Encoder_Follower3_Counter_Accul);
      // Clear accumulated value
      BGVAR(s32_Encoder_Follower2_Counter_Accul) = 0;
      BGVAR(s32_Encoder_Follower3_Counter_Accul) = 0;
      // Clear timer
      BGVAR(u16_Encoder_Follower_1s_Timer) = 0;
   }
}


//**********************************************************
// Function Name: ProcessStopInput
// Description:
//    This function processes the STOP DIN.
//    if DIN STOP is set on rising edge, then stop the motor using the selectd deceleration. drive stays enabled
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void ProcessStopInput(int drive)
{
   // AXIS_OFF;
   static int s16_input_prev = 0;

   // PS mode only (position)
   if (BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PS)
   {
      return;
   }

   // looking for rising edge
   if (VAR(AX0_u16_Mode_Input_Arr[STOP_INP]) && s16_input_prev == 0)
      MotorStop(drive);

   s16_input_prev = VAR(AX0_u16_Mode_Input_Arr[STOP_INP]);
}




//**********************************************************
// Function Name: ProcessPtcmsInput
// Description:
//    INMODE 0x2C input change
//    This function processes the PTCMS INMODE DIN:
//                 if PTCMS input==1 set the high speed pulse input as external command  for Pt mode
//                 if PTCMS input==0 set the low speed pulse input as external command  for Pt mode
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void ProcessPtcmsInput(int drive)
{
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;
   if(IsInFunctionalityConfigured(PTCMS_INP,drive) == 0) return;
   if ((VAR(AX0_u16_Mode_Input_Arr[PTCMS_INP]) != 0) && BGVAR(u16_Schneider_Input_Pulse_Speed) == 0)
   {// If was OFF and now is ON
      BGVAR(u16_Schneider_Input_Pulse_Speed) = 1;
      SetupGear(drive);// dont change the earmode only the mux
   }
   else if ((VAR(AX0_u16_Mode_Input_Arr[PTCMS_INP]) == 0) && BGVAR(u16_Schneider_Input_Pulse_Speed) == 1)
   {// If was ON and now is OFF
      BGVAR(u16_Schneider_Input_Pulse_Speed) = 0;
      SetupGear(drive);// dont change the earmode only the mux
   }
}



//**********************************************************
// Function Name: ProcessSonInput
// Description:
//    INMODE 0x01 input change
//    This function processes the SON DIN:
//                 if SON == 1 and the drive isn't anabled -> try to enable the drive
//                 if SON == 0 and the drive is anabled -> disable the drive
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void ProcessSonInput(int drive)
{
   unsigned int u16_input_state;

   // AXIS_OFF;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;
   if (IsInFunctionalityConfigured(SON_INP,drive) == 0) return;
   //Comment next line and therefore let "CheckLexiumAccessRights" decide what to do...
   //if((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_CANOPEN) return;

   // check that I/O channel has access right to enable drive

   u16_input_state = VAR(AX0_u16_Mode_Input_Arr[SON_INP]);

   if(BGVAR(u16_P2_68_Auto_Enable_Actual) & 0x01)
   {// look for SON level to enable the drive (AEAL nibble 'X' == 1)
      if ((u16_input_state != 0) && (!Enabled(drive)))
      {
         if (CheckLexiumAccessRights(drive, LEX_AR_FCT_ENABLE, LEX_CH_IO_CONTROL) == SAL_SUCCESS)
         {
            // try to enable the drive
            EnableCommand(drive);
         }
      }
      // Check also if no JOG move and auto-tuning is running since an "implicit enable" of the local HMI jog or auto-tuner is not supposed to be disturbed (BYang00000531)
      else if ((u16_input_state == 0) && (Enabled(drive)) &&
               (BGVAR(u16_Local_Hmi_State) != STATE_HMI_MODE_JOG_MOVE) &&
               (BGVAR(u16_Lexium_Autotune_State) == LEX_ATUNE_STATE_IDLE) &&
               (BGVAR(u16_AD_State) == AD_IDLE)) // call the disable function only if AD is idle state to prevent multiple function calls and immidiate disable without AD.
      {// disable the drive
         if (CheckLexiumAccessRights(drive, LEX_AR_FCT_DISABLE, LEX_CH_IO_CONTROL) == SAL_SUCCESS)
         {
            DisableCommand(drive);
         }
      }
   }
   else
   {// look for SON rising adge to enable the drive (AEAL nibble 'X' == 0)
      if (((BGVAR(s16_Prev_Son_State) == 0) && u16_input_state) || (BGVAR(u16_Son_Rising_Edge_detected) == 2))
      {// try to enable the drive
         if(BGVAR(u16_Son_Rising_Edge_detected) != 2)
         {// enable not pending set it to "1" to indicate enable pending
            BGVAR(u16_Son_Rising_Edge_detected) = 1;
         }

         if (CheckLexiumAccessRights(drive, LEX_AR_FCT_ENABLE, LEX_CH_IO_CONTROL) == SAL_SUCCESS)
         {
            EnableCommand(drive);
         }
      }
      else if (BGVAR(s16_Prev_Son_State) && (u16_input_state == 0) && (BGVAR(u16_Son_Rising_Edge_detected) != 3))
      {// disable the drive
         BGVAR(u16_Son_Rising_Edge_detected) = 3;// set it to 3 to indicate disable execute and will not call again until AD state machine finish
         if (CheckLexiumAccessRights(drive, LEX_AR_FCT_DISABLE, LEX_CH_IO_CONTROL) == SAL_SUCCESS)
         {
            DisableCommand(drive);
         }
      }
      BGVAR(s16_Prev_Son_State) = u16_input_state;
   }
}

//**********************************************************
// Function Name: SalReadDITerminal
// Description: read the input mode for inputs DI1 - DI8 (P2-10 - P2-17) in Schneider format(0xABB)
// A: input polarity (1: normaly open 0: normly closed)
// B: input functionality (discribed in EXE_IO.def file)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadDITerminal(long long *data,int drive)
{

   long long index;//, s64_ret_val=0;
  // unsigned int u16_mask=0;
 //  // AXIS_OFF;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   index = s64_Execution_Parameter[0];
   /*
   if ((index < 1LL) || (index > (long long)s16_Num_Of_Inputs)) return (VALUE_OUT_OF_RANGE);
   s64_ret_val = (long long)DigitalInModeCDHDToSchneiderConvert(VAR(AX0_u16_Input_Mode_Arr[index]));;
    u16_mask = (1 << (int)(index - 1));

   if (!(u16_Supported_Dig_Inputs_Mask & u16_mask)) return (I_O_NOT_SUPPORTED);

   if ((BGVAR(u16_Dig_In_Polar) & u16_mask) == 0)
   {
       s64_ret_val |= 0x100; // set 0x1XX in the in mode to indicate this inupt is not inverted
   }
   *data = s64_ret_val;
   */
   *data = BGVAR(u16_In_State_Temp)[index];
   return (SAL_SUCCESS);
}






//**********************************************************
// Function Name: SalWriteDITerminal
// Description:  write the input mode for inputs DI1- DI8 (P2-10 - P2-17)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteDITerminal(int drive)
{
   // AXIS_OFF;

   unsigned int u16_input_index = (unsigned int)s64_Execution_Parameter[0],
             u16_input_polarity = (unsigned int)(s64_Execution_Parameter[1] & 0x100), // save the polarity bit
             u16_lastValue = 0;
   int ret_Value = 0;

   // Frank & Alexander Badura ask to not allow input functionality change while the drive is enbled.
   if (Enabled(drive)) return DRIVE_ACTIVE;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

// save the entered Schneider input mode in temp variable for case of non supported in mode
   u16_lastValue = BGVAR(u16_In_State_Temp)[u16_input_index];
   BGVAR(u16_In_State_Temp)[u16_input_index] = (unsigned int)s64_Execution_Parameter[1];
   s64_Execution_Parameter[1] = (s64_Execution_Parameter[1] & ~0x100);// reset the polarity bit save the inmode number
   s64_Execution_Parameter[1] = (long long)DigitalInModeSchneiderToCDHDConvert((unsigned int)s64_Execution_Parameter[1]);

   // restrict PROBE1 and PROBE2 inmodes to IN7 and IN6 in this order.
   if(((s64_Execution_Parameter[1] & 0xFF) == TOUCH_PROBE_1_INP && (u16_input_index != 6)) || // PROBE1 can be assign to DI7 only
      ((s64_Execution_Parameter[1] & 0xFF) == TOUCH_PROBE_2_INP && (u16_input_index != 5)) )  // PROBE2 can be assign to DI6 only
   {
      BGVAR(u16_In_State_Temp)[u16_input_index] = u16_lastValue;
      return I_O_NOT_SUPPORTED;
   }

   if((u16_input_index > (unsigned int) s16_Num_Of_Inputs) ||
      (s64_Execution_Parameter[1] < 0)                     ||
      (s64_Execution_Parameter[1] >= SCHNEIDER_IN_MODE_TABLE_SIZE))
   {// operation faild set the old value
      BGVAR(u16_In_State_Temp)[u16_input_index] = u16_lastValue;
      return (VALUE_OUT_OF_RANGE);
   }

   s64_Execution_Parameter[0]++; // CDHD handles inputs 1 to num of inputs
   ret_Value = SalInModeCommand();
   if(ret_Value != SAL_SUCCESS)
   {// operation faild set the old value
      BGVAR(u16_In_State_Temp)[u16_input_index] = u16_lastValue;
      return ret_Value;
   }

   if((u16_input_polarity & 0x100) == 0)// if bit 8 == 0 normally closed (reverse polarity)
      s64_Execution_Parameter[1] = (long long)1;
   else // else normally open
      s64_Execution_Parameter[1] = (long long)0;

   ret_Value = SalInPolarCommand();
   if(ret_Value != SAL_SUCCESS)
   {// operation faild set the old value
      BGVAR(u16_In_State_Temp)[u16_input_index] = u16_lastValue;
   }

   UpdateWhoTriggeredPath(drive, VAR(AX0_u16_P3_06_SDI), VAR(AX0_u16_P4_07_ITST), (1 << u16_input_index));

   return ret_Value;
}


//**********************************************************
// Function Name: update_LS_Location
// Description: find the inputs that holds the LS functionality
// and set in these locations "1" bitwise in AX0_u16_LX28_LS_Location variable
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void update_LS_Location(int drive, unsigned int u16_specific_input_num)
{
   unsigned int u16_input_mask = 1 , u16_counter = 1 , u16_num_of_iterations = s16_Num_Of_Inputs;
   REFERENCE_TO_DRIVE;
   // AXIS_OFF;
   if(u16_specific_input_num != 0)
   {// preform only one iteration
      u16_input_mask = 1 << u16_specific_input_num;
      u16_counter = (u16_specific_input_num + 1);
      u16_num_of_iterations = u16_counter;
   }

   for(;u16_counter <= u16_num_of_iterations ;u16_counter++ )
   {
      // if limit switchs or Emergency Stop inputs polarity is reversed
      if((VAR(AX0_u16_Input_Mode_Arr[u16_counter]) == CW_LIMIT_SW_INP) ||
         (VAR(AX0_u16_Input_Mode_Arr[u16_counter]) == CCW_LIMIT_SW_INP))
      {
         VAR(AX0_u16_LX28_LS_Location) |= u16_input_mask;
      }
      else
      {
         VAR(AX0_u16_LX28_LS_Location) &= ~u16_input_mask;
      }
      u16_input_mask <<= 1;
   }
}

//**********************************************************
// Function Name: SalReadDOTerminal
// Description: read the output mode for outputs DO1- DO5 (P2-18 - P2-22) in Schneider format(0xABB)
// A: outpout polarity (1: normaly open 0: normly closed)
// B: output functionality (discribed in EXE_IO.def file)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadDOTerminal(long long *data , int drive)
{
   long long index;//, s64_ret_val=0;
//   unsigned int u16_mask=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   index = s64_Execution_Parameter[0];
/*
   if ((index < 1LL) || (index > (long long)s16_Num_Of_Outputs)) return (VALUE_OUT_OF_RANGE);
   if (!(u16_Supported_Dig_Outputs_Mask & (1 << (int)(index - 1LL)))) return (I_O_NOT_SUPPORTED);
   s64_ret_val = (long long)DigitalOutModeCDHDToSchneiderConvert(u16_Out_Mode[index - 1LL]);

   u16_mask = (1 << (int)(index - 1));

   if (!(u16_Supported_Dig_Outputs_Mask & u16_mask)) return (I_O_NOT_SUPPORTED);

   if ((BGVAR(u16_Dig_Out_Polar) & u16_mask) == 0)
   {
       s64_ret_val |= 0x100; // set 0x1XX in the out mode to indicate this output is not inverted
   }

   *data = s64_ret_val;
   */
   *data = (long long)BGVAR(u16_Out_State_Temp)[index];
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalWriteDOTerminal
// Description:  write the output mode for outputs DO1- DO5 (P2-18 - P2-23)

// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteDOTerminal(int drive)
{
   unsigned int u16_output_index = (unsigned int)s64_Execution_Parameter[0],
                u16_output_polarity = (s64_Execution_Parameter[1] & 0x100) /*save the polarity bit*/,
                u16_lastValue = 0, u16_new_outmode=0;
   int ret_Value = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error
// save the entered Schneider output mode in temp variable for case of non supported out mode
   u16_lastValue =  BGVAR(u16_Out_State_Temp)[u16_output_index];
   BGVAR(u16_Out_State_Temp)[u16_output_index] = (unsigned int) s64_Execution_Parameter[1];

   s64_Execution_Parameter[1] = (s64_Execution_Parameter[1] & ~0x100);// reset the polarity bit save the outmode number

   // nitsan: check max limit (bug 3686).
   if (s64_Execution_Parameter[1] > MAX_NUM_OF_OUTPUTS_FUNCTIONALITIES)
   {
      BGVAR(u16_Out_State_Temp)[u16_output_index] = u16_lastValue;
      return VALUE_TOO_HIGH;
   }

   s64_Execution_Parameter[1] = (long long)DigitalOutModeSchneiderToCDHDConvert((unsigned int)s64_Execution_Parameter[1]);

   u16_new_outmode = (unsigned int)s64_Execution_Parameter[1];

   if( (u16_output_index >= (unsigned int) s16_Num_Of_Outputs)         ||
       (s64_Execution_Parameter[1] < 0LL)                             ||
       (s64_Execution_Parameter[1] >= (long long)SCHNEIDER_OUT_MODE_TABLE_SIZE)  )
   {// operation faild set the old value
      BGVAR(u16_Out_State_Temp)[u16_output_index] = u16_lastValue;
      return (VALUE_OUT_OF_RANGE);
   }
   // IPR#972 IPR#1080 fix - DO6 (P2-23) can be set only to IDLE (0x00) or "EEZ_OUT" (0x40) or to "AutoR" MSB PATH INDEX indication (outmode = 0x?? unknown yet)
   if((u16_output_index == 5) &&
     ((s64_Execution_Parameter[1] & 0x00FF) != EEZ_OUT) &&
     ((s64_Execution_Parameter[1] & 0x00FF) != NO_FUNC_OUT))
   {// operation faild set the old value
      BGVAR(u16_Out_State_Temp)[u16_output_index] = u16_lastValue;
      return (VALUE_OUT_OF_RANGE);
   }

   // Only DO6 (P2-23) can be set to EEZ_OUT, and if so, no polarity control is available (EEZ_OUT implemented inFPGA)
   if (s64_Execution_Parameter[1] == EEZ_OUT)
   {
      if ( (u16_output_index != 5) || ( (u16_output_index == 5)  && (u16_output_polarity != 0) ) )
      {// worng configuration retrive the old value
         BGVAR(u16_Out_State_Temp)[u16_output_index] = u16_lastValue;
         return (NOT_SUPPORTED_ON_HW);
      }
   }

   s64_Execution_Parameter[0]++;// to match the CDHD index that starts from 1 to num of outputs
   ret_Value = SalOutModeCommand();
   if(ret_Value != SAL_SUCCESS)
   {// operation faild set the old value
      BGVAR(u16_Out_State_Temp)[u16_output_index] = u16_lastValue;
      return ret_Value;
   }

   if((u16_output_polarity & 0x100) == 0)// if bit 8 == 0 normally closed (reverse polarity)
      s64_Execution_Parameter[1] = (long long)1;
   else // else normally open
      s64_Execution_Parameter[1] = (long long)0;

   OutCommand(s64_Execution_Parameter[0], 0); // set the output to "0" by default to clear any leftovers from last operation.
   ret_Value = SalOutPolarCommand();
   if(ret_Value != SAL_SUCCESS)
   {// operation faild set the old value
      BGVAR(u16_Out_State_Temp)[u16_output_index] = u16_lastValue;
      return ret_Value;
   }

   //FIX IPR#1396
   // In case the outmode is "ZSPD" save
   // ZSPD is 0x3 in SE outmode

   if (u16_new_outmode == ZSPD)
   {// save ZSPD index (0 - 5) 6 outputs
      // reset the low 8 bits and dont change the high 8 bits (indicate force output state)
      // save the ZSPD outputs bitwise to the LOW 8 bits.
      VAR(AX0_s16_ZSPD_Out_Num_Forced) |= ((0x1 << u16_output_index) & 0x01F);
      // clear Bit4-Bit8
      VAR(AX0_s16_ZSPD_Out_Num_Forced) &= 0xFC1F;
      // save polarity to Bit4-Bit8 only for the outputs that assign as ZSPD
      VAR(AX0_s16_ZSPD_Out_Num_Forced) |= ((BGVAR(u16_Dig_Out_Polar) & VAR(AX0_s16_ZSPD_Out_Num_Forced))<< 5);
   }
   else if((u16_lastValue & 0x0FF) == 0x03)
   {//reset the ZSPD out number and set value to (0x00FF)-> LOW 8 bits "1" , HIGH 8 bits "0"
      //set the low 8 bits to "1" and dont change the high 8 bits (indicate force output state).
      VAR(AX0_s16_ZSPD_Out_Num_Forced) &= ~(0x1 << u16_output_index);
      // clear Bit4-Bit8
      VAR(AX0_s16_ZSPD_Out_Num_Forced) &= 0xFC1F;
      // save polarity to Bit4-Bit8 only for the outputs that assign as ZSPD
      VAR(AX0_s16_ZSPD_Out_Num_Forced) |= ((BGVAR(u16_Dig_Out_Polar) & VAR(AX0_s16_ZSPD_Out_Num_Forced))<< 5);
   }
   // FIX IPR#1437
   // move TPOS output handle to RT.
   // In case the outmode is "TPOS" write the assigned outputs and polarities to RT variable for RT handling
   // TPOS is 0x5 in SE outmode
   if (u16_new_outmode == TPOS)
   {// save TPOS index (0 - 5) 6 outputs
      // reset the low 8 bits and dont change the high 8 bits (indicate force output state)
      // save the TPOS outputs bitwise to the LOW 8 bits.
      VAR(AX0_s16_TPOS_Out_Num_Forced) |= ((0x1 << u16_output_index) & 0x01F);
      // clear Bit4-Bit8
      VAR(AX0_s16_TPOS_Out_Num_Forced) &= 0xFC1F;
      // save polarity to Bit4-Bit8 only for the outputs that assign as TPOS
      VAR(AX0_s16_TPOS_Out_Num_Forced) |= ((BGVAR(u16_Dig_Out_Polar) & VAR(AX0_s16_TPOS_Out_Num_Forced))<< 5);
   }
   else if((u16_lastValue & 0x0FF) == 0x05)
   {//reset the TPOS out number and set value to (0x00FF)-> LOW 8 bits "1" , HIGH 8 bits "0"
      //set the low 8 bits to "1" and dont change the high 8 bits (indicate force output state).
      VAR(AX0_s16_TPOS_Out_Num_Forced) &= ~(0x1 << u16_output_index);
      // clear Bit4-Bit8
      VAR(AX0_s16_TPOS_Out_Num_Forced) &= 0xFC1F;
      // save polarity to Bit4-Bit8 only for the outputs that assign as TPOS
      VAR(AX0_s16_TPOS_Out_Num_Forced) |= ((BGVAR(u16_Dig_Out_Polar) & VAR(AX0_s16_TPOS_Out_Num_Forced))<< 5);
   }
   return ret_Value;
}


//**********************************************************
// Function Name: DigitalInModeSchneiderConvert
// Description:  convert form CDHD in modes to Schneider in modes

// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
unsigned int DigitalInModeCDHDToSchneiderConvert(unsigned int CDHD_mode)
{
   unsigned int u16_schneider_value = 0;
   switch(CDHD_mode)
   {
     case PATH_BIT_0_INP:
         u16_schneider_value = 0x11;
      break;

      case PATH_BIT_1_INP:
         u16_schneider_value = 0x12;
      break;

      case PATH_BIT_2_INP:
         u16_schneider_value = 0x13;
      break;

      case PATH_BIT_3_INP:
         u16_schneider_value = 0x1A;
      break;

      case PATH_BIT_4_INP:
         u16_schneider_value = 0x1B;
      break;

      case TOUCH_PROBE_1_INP:
         u16_schneider_value = 0x1C;
      break;

      case TOUCH_PROBE_2_INP:
         u16_schneider_value = 0x1D;
      break;

      case EMERGENCY_STOP_INP:
         u16_schneider_value = 0x21;
      break;

      case CW_LIMIT_SW_INP:
         u16_schneider_value = 0x23;
      break;

      case CCW_LIMIT_SW_INP:
         u16_schneider_value = 0x22;
      break;

      case HOME_SWITCH_INP:
         u16_schneider_value = 0x24;
      break;

       case HOMING_CMD_INP:
         u16_schneider_value = 0x27;
      break;

      case HALT_INP:
         u16_schneider_value = 0x07;
      break;

      case PATH_CTRG_INP:
         u16_schneider_value = 0x08;
      break;

     case ZCLAMP_0_INP:
         u16_schneider_value = 0x05;
      break;

      case TRQ_CMD_SELECT_0_INP:
         u16_schneider_value = 0x16;
      break;

      case TRQ_CMD_SELECT_1_INP:
         u16_schneider_value = 0x17;
      break;

      case TRQ_LIM_EN:
         u16_schneider_value = 0x09;
      break;

/*   scnieder decided to cancel these modes
      case TRQ_LIM_POS:
         u16_schneider_value = 0x25;
      break;

      case TRQ_LIM_NEG:
         u16_schneider_value = 0x26;
      break; */

     case GEAR_CCLR_INP:
         u16_schneider_value = 0x04;
      break;

      case CMD_INV_INP:
         u16_schneider_value = 0x06;
      break;

      case GAIN_UP_INP:
         u16_schneider_value = 0x03;
      break;

      case STOP_INP:
         u16_schneider_value = 0x46;
      break;

     case SPD_CMD_SELECT_0_INP: // SPD0
         u16_schneider_value = 0x14;
      break;

      case SPD_CMD_SELECT_1_INP: // SPD1
         u16_schneider_value = 0x15;
      break;

      case RESET_INP:
         u16_schneider_value = 0x02;
      break;

     case GNUM0_INP:
         u16_schneider_value = 0x43;
      break;

     case GNUM1_INP:
         u16_schneider_value = 0x44;
      break;

      case PTCMS_INP: // PTCMS external Pt selector
         u16_schneider_value = 0x2C;
      break;

      case SON_INP: // SON
         u16_schneider_value = 0x01;
      break;

      case JOGD_INP: // JOGD
         u16_schneider_value = 0x38;
      break;

      case JOGU_INP: // JOGU
         u16_schneider_value = 0x37;
      break;

      case INHP_INP: // INHP
         u16_schneider_value = 0x45;
      break;

      case S_P_INP: // S-P
         u16_schneider_value = 0x18;
      break;

      case S_T_INP: // S-T
         u16_schneider_value = 0x19;
      break;

      case T_P_INP: // T-P
         u16_schneider_value = 0x20;
      break;

      case Pt_PS_INP: // Pt-PS
         u16_schneider_value = 0x2B;
      break;

      case SPD_LIM_EN: // SPDLM
         u16_schneider_value = 0x10;
      break;

      case STEPU_INP: // STEPU
         u16_schneider_value = 0x39;
      break;

      case STEPD_INP: // STEPD
         u16_schneider_value = 0x40;
      break;

      case STEPB_INP: // STEPB
         u16_schneider_value = 0x41;
      break;

      case AUTOR_INP: // AUTOR
         u16_schneider_value = 0x42;
      break;

      default:
         u16_schneider_value = 0;
      break;
   }
   return u16_schneider_value;
}


//**********************************************************
// Function Name: DigitalInModeSchneiderToCDHDConvert
// Description:  convert form Schneider in modes to CDHD in modes numbers

// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int DigitalInModeSchneiderToCDHDConvert(unsigned int schneider_mode)
{
   int s16_CDHD_value = 0;

   switch(schneider_mode)
   {
      case 0: // unsign
         s16_CDHD_value = 0;
      break;

      case 0x01: // SON
         s16_CDHD_value = SON_INP;
      break;

      case 0x02: // ARST
         s16_CDHD_value = RESET_INP;
      break;

      case 0x03: // GAINUP
         s16_CDHD_value = GAIN_UP_INP;
      break;

      case 0x04: // CCLR
         s16_CDHD_value = GEAR_CCLR_INP;
      break;

      case 0x05: // ZCLAMP
         s16_CDHD_value = ZCLAMP_0_INP;
      break;

      case 0x06: // CMDINV
         s16_CDHD_value = CMD_INV_INP;
      break;

      case 0x7: // HALT
         s16_CDHD_value = HALT_INP;
      break;

      case 0x08: // CTRG
         s16_CDHD_value = PATH_CTRG_INP;
      break;

      case 0x09: // TRQLM
         s16_CDHD_value = TRQ_LIM_EN;
      break;

      case 0x10: // SPDLM
         s16_CDHD_value = SPD_LIM_EN;
      break;

      case 0x11: // POS0
         s16_CDHD_value = PATH_BIT_0_INP;
      break;

      case 0x12: // POS1
         s16_CDHD_value = PATH_BIT_1_INP;
      break;

      case 0x13: // POS2
         s16_CDHD_value = PATH_BIT_2_INP;
      break;

      case 0x1A: // POS3
         s16_CDHD_value = PATH_BIT_3_INP;
      break;

      case 0x1B: // POS4
         s16_CDHD_value = PATH_BIT_4_INP;
      break;

      case 0x1C: // Touch probe 1
         s16_CDHD_value = TOUCH_PROBE_1_INP;
      break;

      case 0x1D: // Touch probe 2
         s16_CDHD_value = TOUCH_PROBE_2_INP;
      break;


     case 0x14: // SPD0
         s16_CDHD_value = SPD_CMD_SELECT_0_INP;
      break;

      case 0x15: // SPD1
         s16_CDHD_value = SPD_CMD_SELECT_1_INP;
      break;

      case 0x16: // TCM0
         s16_CDHD_value = TRQ_CMD_SELECT_0_INP;
      break;

      case 0x17: // TCM1
         s16_CDHD_value = TRQ_CMD_SELECT_1_INP;
      break;

      case 0x18: // S-P
         s16_CDHD_value = S_P_INP;
      break;

      case 0x19: // S-T
         s16_CDHD_value = S_T_INP;
      break;

      case 0x20: // T-P
         s16_CDHD_value = T_P_INP;
      break;

      case 0x21: // OPST
         s16_CDHD_value = EMERGENCY_STOP_INP;
      break;

      case 0x23: // PL (CWL)
         s16_CDHD_value = CW_LIMIT_SW_INP;
      break;

      case 0x22: // NL (CCWL)
         s16_CDHD_value = CCW_LIMIT_SW_INP;
      break;

      case 0x24: // ORGP
         s16_CDHD_value = HOME_SWITCH_INP;
      break;

/*   schnieder decided to cancel these modes, so return invalid value
      case 0x25: // TLLM
         s16_CDHD_value = NO_FUNC_INP;
      break;

      case 0x26: // TRLM
         s16_CDHD_value = NO_FUNC_INP;
      break; */

      case 0x27: // SHOM
         s16_CDHD_value = HOMING_CMD_INP;
      break;

      case 0x2B: // Pt-PS
         s16_CDHD_value = Pt_PS_INP;
      break;

      case 0x2C: // PTCMS external Pt selector
         s16_CDHD_value = PTCMS_INP;
      break;

      case 0x37: // JOGU
         s16_CDHD_value = JOGU_INP;
      break;

      case 0x38: // JOGD
         s16_CDHD_value = JOGD_INP;
      break;

      case 0x43: // GNUM0
         s16_CDHD_value = GNUM0_INP;
      break;

      case 0x44: // GNUM1
         s16_CDHD_value = GNUM1_INP;
      break;

      case 0x45: // INHP
         s16_CDHD_value = INHP_INP;
      break;

      case 0x46: // STOP
         s16_CDHD_value = STOP_INP;
      break;

      case 0x39: // STEPU
         s16_CDHD_value = STEPU_INP;
      break;

      case 0x40: // STEPD
         s16_CDHD_value = STEPD_INP;
      break;

      case 0x41: // STEPB
         s16_CDHD_value = STEPB_INP;
      break;

      case 0x42: // AUTOR
         s16_CDHD_value = AUTOR_INP;
      break;

      default:
        // return invalid value error
         s16_CDHD_value = -1;
      break;
   }
   return s16_CDHD_value;
}


//**********************************************************
// Function Name: DigitalOutModeSchneiderConvert
// Description:  convert form CDHD out modes to Schneider out modes

// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
unsigned int DigitalOutModeCDHDToSchneiderConvert(unsigned int CDHD_mode)
{
   unsigned int u16_schneider_value = 0;
   switch(CDHD_mode)
   {
     case SRDY: // SRDY
         u16_schneider_value = 0x01;
      break;

      case ACTIVE_OUT: // SON
         u16_schneider_value = 0x02;
      break;

      case ZSPD: // ZSPD
         u16_schneider_value = 0x03;
      break;

      case TSPD: // TSPD
         u16_schneider_value = 0x04;
      break;

      case TPOS: // TPOS         // fix IPR 935
         u16_schneider_value = 0x05;
      break;

      case TQL: // TQL
         u16_schneider_value = 0x06;
      break;

      case ALARM_OUT: // ALRM
         u16_schneider_value = 0x07;
      break;

      case BRAKE_OUT: // BRKR
         u16_schneider_value = 0x08;
      break;

      case HOME_DONE_OUT: // HOME
         u16_schneider_value = 0x09;
      break;

      case OLW: // OLW
         u16_schneider_value = 0x10;
      break;

      case WARNING_OUT: // WARN
         u16_schneider_value = 0x11;
      break;

      case OVF: // OVF
         u16_schneider_value = 0x12;
      break;

      case SNL: // SNL
         u16_schneider_value = 0x13;
      break;

      case SPL: // SPL
         u16_schneider_value = 0x14;
      break;

      case CMD_OK: // CMD_OK
         u16_schneider_value = 0x15;
      break;

      case CAP_OK: // CAP_OK
         u16_schneider_value = 0x16;
      break;

      case MC_OK: // MC_OK
         u16_schneider_value = 0x17;
      break;

      case CAP_2_OK:
         u16_schneider_value = 0x18;
      break;

      case SP_OK: // SP_OK
         u16_schneider_value = 0x19;
      break;

      case AUTOR_OUT: // AUTOR_OUT
         u16_schneider_value = 0x21;
      break;

      case SDO_0_OUT: // SDO_0_OUT
         u16_schneider_value = 0x30;
      break;

      case SDO_1_OUT: // SDO_1_OUT
         u16_schneider_value = 0x31;
      break;

      case SDO_2_OUT: // SDO_2_OUT
         u16_schneider_value = 0x32;
      break;

      case SDO_3_OUT: // SDO_3_OUT
         u16_schneider_value = 0x33;
      break;

      case SDO_4_OUT: // SDO_4_OUT
         u16_schneider_value = 0x34;
      break;

      case SDO_5_OUT: // SDO_5_OUT
         u16_schneider_value = 0x35;
      break;

      case SDO_6_OUT: // SDO_6_OUT
         u16_schneider_value = 0x36;
      break;

      case SDO_7_OUT: // SDO_7_OUT
         u16_schneider_value = 0x37;
      break;
/*
// Fix IPR 1153: Signal Output Functions 30h - 37h has no effect
// it was decided to cancel inmodes SDO_8 to SDO_F
      case SDO_8_OUT: // SDO_8_OUT
         u16_schneider_value = 0x38;
      break;

      case SDO_9_OUT: // SDO_9_OUT
         u16_schneider_value = 0x39;
      break;

      case SDO_A_OUT: // SDO_A_OUT
         u16_schneider_value = 0x3A;
      break;

      case SDO_B_OUT: // SDO_B_OUT
         u16_schneider_value = 0x3B;
      break;

      case SDO_C_OUT: // SDO_C_OUT
         u16_schneider_value = 0x3C;
      break;

      case SDO_D_OUT: // SDO_D_OUT
         u16_schneider_value = 0x3D;
      break;

      case SDO_E_OUT: // SDO_E_OUT
         u16_schneider_value = 0x3E;
      break;

      case SDO_F_OUT: // SDO_F_OUT
         u16_schneider_value = 0x3F;
      break;
*/

      case EEZ_OUT: // EEZ_OUT
         u16_schneider_value = 0x40;
      break;

      default:
         u16_schneider_value = 0;
      break;
   }
   return   u16_schneider_value;
}


//**********************************************************
// Function Name: DigitalOutModeSchneiderToCDHDConvert
// Description:  convert form Schneider out modes to CDHD out modes numbers

// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
unsigned int DigitalOutModeSchneiderToCDHDConvert(unsigned int schneider_mode)
{
   unsigned int u16_CDHD_value=0xFFFF;
   switch(schneider_mode)
   {
      case 0x00:
         u16_CDHD_value = NO_FUNC_OUT;
      break;

      case 0x01: // SRDY
         u16_CDHD_value = SRDY;
      break;

      case 0x02: // SON -> ACTIVE_OUT
         u16_CDHD_value = SON;
      break;

      case 0x03: // ZSPD
         u16_CDHD_value = ZSPD;
      break;

      case 0x04: // TSPD
         u16_CDHD_value = TSPD;
      break;

      case 0x05: // TPOS      // fix IPR 935
            u16_CDHD_value = TPOS;
      break;

      case 0x06: // TQL
         u16_CDHD_value = TQL;
      break;

      case 0x07: // ALRM
         u16_CDHD_value = ALARM_OUT;
      break;

      case 0x08: // BRKR
         u16_CDHD_value = BRAKE_OUT;
      break;

      case 0x09: // HOME
         u16_CDHD_value = HOME_DONE_OUT;
      break;

      case 0x10: // OLW
         u16_CDHD_value = OLW;
      break;

      case 0x11: // WARN
         u16_CDHD_value = WARNING_OUT;
      break;

      case 0x12: // OVF
         u16_CDHD_value = OVF;
      break;

      case 0x13: // SNL
         u16_CDHD_value = SNL;
      break;

      case 0x14: // SPL
         u16_CDHD_value = SPL;
      break;

      case 0x15: // CMD_OK
         u16_CDHD_value = CMD_OK;
      break;

      case 0x16: // CAP_OK
         u16_CDHD_value = CAP_OK;
      break;

      case 0x17: // MC_OK
         u16_CDHD_value = MC_OK;
      break;

      case 0x18: // CAP_2_OK
         u16_CDHD_value = CAP_2_OK;
      break;

      case 0x19: // SP_OK
         u16_CDHD_value = SP_OK;
      break;

      case 0x21: // AUTOR_OUT
//         u16_CDHD_value = AUTOR_OUT; remove comment when AutoR feature will be implemented.
      break;

      case 0x30: // SDO_0_OUT
         u16_CDHD_value = SDO_0_OUT;
      break;

      case 0x31: // SDO_1_OUT
         u16_CDHD_value = SDO_1_OUT;
      break;

      case 0x32: // SDO_2_OUT
         u16_CDHD_value = SDO_2_OUT;
      break;

      case 0x33: // SDO_3_OUT
         u16_CDHD_value = SDO_3_OUT;
      break;

      case 0x34: // SDO_4_OUT
         u16_CDHD_value = SDO_4_OUT;
      break;

      case 0x35: // SDO_5_OUT
         u16_CDHD_value = SDO_5_OUT;
      break;

      case 0x36: // SDO_6_OUT
         u16_CDHD_value = SDO_6_OUT;
      break;

      case 0x37: // SDO_7_OUT
         u16_CDHD_value = SDO_7_OUT;
      break;
/*
// Fix IPR 1153: Signal Output Functions 30h - 37h has no effect
// it was decided to cancel inmodes SDO_8 to SDO_F
      case 0x38: // SDO_8_OUT
         u16_CDHD_value = SDO_8_OUT;
      break;

      case 0x39: // SDO_9_OUT
         u16_CDHD_value = SDO_9_OUT;
      break;

      case 0x3A: // SDO_A_OUT
         u16_CDHD_value = SDO_A_OUT;
      break;

      case 0x3B: // SDO_B_OUT
         u16_CDHD_value = SDO_B_OUT;
      break;

      case 0x3C: // SDO_C_OUT
         u16_CDHD_value = SDO_C_OUT;
      break;

      case 0x3D: // SDO_D_OUT
         u16_CDHD_value = SDO_D_OUT;
      break;

      case 0x3E: // SDO_E_OUT
         u16_CDHD_value = SDO_E_OUT;
      break;

      case 0x3F: // SDO_F_OUT
         u16_CDHD_value = SDO_F_OUT;
      break;
*/
      case 0x40: // EEZ_OUT
         u16_CDHD_value = EEZ_OUT;
      break;

      default:
         u16_CDHD_value = 0xFFFF;
      break;
   }
   return u16_CDHD_value;
}

//**********************************************************
// Function Name: InitSchneiderDigitalIODefaults
// Description:  Set the deafult inputs/outputs configurations acording to te new opmode
//               It manged by nibble "D" in CTL (P1-01) parameter.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************

void InitSchneiderDigitalIODefaults(int drive)
{
   int s16_input_idx=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // check if nibble "D" of the CTL parameter is "1" update the IO defaults else return
   if((BGVAR(u16_P1_01_CTL_Current) & 0x1000) == 0x0)
   {
      // update if brake output is defiend
      BGVAR(u16_Brake_Output_Is_Defined) = IsOutFunctionalityConfigured(BRAKE_OUT);
      return;
   }

   // if the default IO bit (nibble "D") was "1" reset it back to "0" for next power cycle;
   BGVAR(u16_P1_01_CTL) &= 0x0FFF;
   BGVAR(u16_P1_01_CTL_Current) = BGVAR(u16_P1_01_CTL);

   // reset all inverted inputs to normal
   for(s16_input_idx=0 ;s16_input_idx < s16_Num_Of_Inputs ; s16_input_idx++ )
   {
       STORE_EXECUTION_PARAMS_0_1
       s64_Execution_Parameter[0] = (s16_input_idx+1);
       s64_Execution_Parameter[1] = 0x0;
       SalInPolarCommand();
       RESTORE_EXECUTION_PARAMS_0_1
   }

   // init digital inputs 1-8
   switch(BGVAR(u16_P1_01_CTL_Current) & 0xFF)
   {
       case SE_OPMODE_PT:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x104;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x116;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x117;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x102;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x22;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x23;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_PS:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x108;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x111;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x112;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x102;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x22;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x23;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_S:
       case SE_OPMODE_SZ:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x109;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x114;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x115;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x102;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x22;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x23;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_T:
       case SE_OPMODE_TZ:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x110;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x116;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x117;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x102;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x22;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x23;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_PT_S:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x104;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x114;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x115;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x118;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_PT_T:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x104;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x116;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x117;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x120;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_PS_S:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x108;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x111;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x112;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x114;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x115;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x118;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_PS_T:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x108;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x111;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x112;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x116;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x117;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x120;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_S_T:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x114;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x115;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x116;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x117;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x119;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_CANOPEN:
       case SE_OPMODE_SERCOS:
       case SE_OPMODE_ETHERNET_IP:
       case SE_OPMODE_ETHERCAT:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x24;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x22;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x23;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;


       case SE_OPMODE_PT_PR:
            // DI 1
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 0;
            s64_Execution_Parameter[1] = 0x101;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x110;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x116;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x117;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x102;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 6
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 5;
            s64_Execution_Parameter[1] = 0x22;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DI 7
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 6;
            s64_Execution_Parameter[1] = 0x23;
            SalWriteDITerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       default:
            break;
   }

   // DI 8 is Operational Stop for all the opmodes
   STORE_EXECUTION_PARAMS_0_1
   s64_Execution_Parameter[0] = 7;
   s64_Execution_Parameter[1] = 0x21;
   SalWriteDITerminal(0);
   RESTORE_EXECUTION_PARAMS_0_1

   // init digital outputs 1-6
   switch(BGVAR(u16_P1_01_CTL_Current) & 0xFF)
   {
       case SE_OPMODE_PT:
       case SE_OPMODE_PS:
       case SE_OPMODE_PT_PR:
            // DO 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x103;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x109;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x105;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x07;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_S:
       case SE_OPMODE_T:
       case SE_OPMODE_SZ:
       case SE_OPMODE_TZ:
            // DO 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x103;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x104;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x108;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x07;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_PT_S:
       case SE_OPMODE_PT_T:
       case SE_OPMODE_PS_S:
       case SE_OPMODE_PS_T:
            // DO 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x103;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x104;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x105;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x07;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_S_T:
            // DO 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x103;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x104;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x07;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_CANOPEN:
            // DO 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x07;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       case SE_OPMODE_SERCOS:
       case SE_OPMODE_ETHERNET_IP:
       case SE_OPMODE_ETHERCAT:
            // DO 2
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 1;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 3
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 2;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 4
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 3;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            // DO 5
            STORE_EXECUTION_PARAMS_0_1
            s64_Execution_Parameter[0] = 4;
            s64_Execution_Parameter[1] = 0x100;
            SalWriteDOTerminal(0);
            RESTORE_EXECUTION_PARAMS_0_1
            break;

       default:
            break;
   }

   // these are the default DO for DO 1 and DO 6
   // DO 1
   STORE_EXECUTION_PARAMS_0_1
   s64_Execution_Parameter[0] = 0;
   s64_Execution_Parameter[1] = 0x101;
   SalWriteDOTerminal(0);
   RESTORE_EXECUTION_PARAMS_0_1
   // DO 6
   STORE_EXECUTION_PARAMS_0_1
   s64_Execution_Parameter[0] = 5;
   s64_Execution_Parameter[1] = 0x040;//EEZ_OUT encoder simulation output
   SalWriteDOTerminal(0);
   RESTORE_EXECUTION_PARAMS_0_1
   
   
   // update if brake output is defiend
   BGVAR(u16_Brake_Output_Is_Defined) = IsOutFunctionalityConfigured(BRAKE_OUT);
}


//**********************************************************
// Function Name: GetActualDigitalIOStateIntoSchneiderIOsParams
// Description:  this function get the current inputs outputs modes from the CDHD (saved in inmode, outmode)
// and the inv state into Schneider's display variables
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void GetActualDigitalIOStateIntoSchneiderIOsParams(int drive)
{
   int s16_index = 1;

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // read all inputs modes from drive into Schneider digital inputs variables
   for (s16_index = 1 ; s16_index <= s16_Num_Of_Inputs ; s16_index++)
      BGVAR(u16_In_State_Temp)[s16_index - 1] = GetActualCDHDInputsModes(s16_index);

   // read all oututs modes from drive into Schneider digital outputs variables
   for (s16_index = 1 ; s16_index <= s16_Num_Of_Outputs ; s16_index++)
   {
      BGVAR(u16_Out_State_Temp)[s16_index-1] = GetActualCDHDOutputsModes(s16_index);

      //FIX IPR#1396: update the outputs where ZSPD outmode is assigned. (can be more than single output).
      if((BGVAR(u16_Out_State_Temp)[s16_index-1] & 0xFF) == 0x03)
      {// "0x03" is the Lexium outmode for ZSPD
         VAR(AX0_s16_ZSPD_Out_Num_Forced) |= ((0x1 << (s16_index-1)) & 0x001F);
         if((BGVAR(u16_Out_State_Temp)[s16_index-1] & 0x0100) == 0)
         {// update the RT output polarity bits (bit5-bit9)
            VAR(AX0_s16_ZSPD_Out_Num_Forced) |= (0x1 << (s16_index + 4));
         }
      }

      // update the outputs where TPOS outmode is assigned. (can be more than single output).
      if((BGVAR(u16_Out_State_Temp)[s16_index-1] & 0xFF) == 0x05)
      {// "0x05" is the Lexium outmode for TPOS
         VAR(AX0_s16_TPOS_Out_Num_Forced) |= ((0x1 << (s16_index-1)) & 0x001F);
         if((BGVAR(u16_Out_State_Temp)[s16_index-1] & 0x0100) == 0)
         {// update the RT output polarity bits (bit5-bit9)
            VAR(AX0_s16_TPOS_Out_Num_Forced) |= (0x1 << (s16_index + 4));
         }
      }
   }
}


//**********************************************************
// Function Name: GetActualCDHDInputsModes
// Description:  this function get the current input modes from the CDHD (saved in inmode)
// and the in inv state into Schneider's display variables
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
unsigned int GetActualCDHDInputsModes(int s16_in_index)
{// get the current CDHD input modes and input polarity in Schneider digital input format
   // AXIS_OFF;
   unsigned int u16_ret_val = 0, u16_mask = 0;
   
   if ((s16_in_index < 1LL) || (s16_in_index > (long long)s16_Num_Of_Inputs))
      return (VALUE_OUT_OF_RANGE);
   u16_ret_val = (long long)DigitalInModeCDHDToSchneiderConvert(VAR(AX0_u16_Input_Mode_Arr[s16_in_index]));
   u16_mask = (1 << (int)(s16_in_index - 1));

   // if func is INHP_INP update the FPGA.
   if(VAR(AX0_u16_Input_Mode_Arr[s16_in_index]) == INHP_INP)
   {
      *(int*)FPGA_DIG_IN_INHIBIT_COUNT_REG_ADD = s16_in_index;
   }

   if (!(u16_Supported_Dig_Inputs_Mask & u16_mask)) return (I_O_NOT_SUPPORTED);

   if ((BGVAR(u16_Dig_In_Polar) & u16_mask) == 0)
      u16_ret_val |= 0x100; // set 0x1XX in the in mode to indicate this inupt is not inverted

   return u16_ret_val;
}


//**********************************************************
// Function Name: GetActualCDHDOutputsModes
// Description:  this function get the current output modes from the CDHD (saved in outmode)
// and the out inv state into Schneider's display variables
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
unsigned int GetActualCDHDOutputsModes(int s16_out_index)
{// get the current CDHD output modes and output polarity in Schneider digital output format
   unsigned int s16_ret_val = 0, u16_mask = 0;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s16_out_index < 1LL) || (s16_out_index > (long long)s16_Num_Of_Outputs))
      return (VALUE_OUT_OF_RANGE);
   if (!(u16_Supported_Dig_Outputs_Mask & (1 << (int)(s16_out_index - 1LL))))
      return (I_O_NOT_SUPPORTED);
   s16_ret_val = (long long)DigitalOutModeCDHDToSchneiderConvert(u16_Out_Mode[s16_out_index - 1LL]);

   u16_mask = (1 << (int)(s16_out_index - 1));

   if (!(u16_Supported_Dig_Outputs_Mask & u16_mask)) return (I_O_NOT_SUPPORTED);

   if ((BGVAR(u16_Dig_Out_Polar) & u16_mask) == 0)
      s16_ret_val |= 0x100; // set 0x1XX in the out mode to indicate this output is not inverted

   return s16_ret_val;
}


//**********************************************************
// Function Name: ProcessGearInNumerators
// Description:
//    This function processes the GNUM0 and GNUM1 DIN. if opmode==Pt (gearing) or PS (position),
//    these selectors select the value to be used for the GEARIN parameter.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void ProcessGearInNumerators(int drive)
{
   // AXIS_OFF;

   static int s16_command_selector_prev = -1;
   static unsigned int u16_ext_polarity_prev = 0x0;
   int s16_command_selector, tmp0, tmp1;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // GNUM inputs are relevant for these modes only (in lxm23 manual).
   if (BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PT &&
       BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PS &&
       BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PT_S &&
       BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PS_S)
   {
       return;
   }


   if (VAR(AX0_s16_Opmode) == 4)
   {
       // get selector from inputs (each input is a bit. combination of 2 bits)
       tmp1 = ((VAR(AX0_u16_Mode_Input_Arr[GNUM1_INP]) != 0) << 1);
       tmp0 = (VAR(AX0_u16_Mode_Input_Arr[GNUM0_INP]) != 0);

       s16_command_selector = tmp1 | tmp0;
   }
   else if (VAR(AX0_s16_Opmode) == 8)
       s16_command_selector = 0;       // in PS mode, there is only P1-44
   else
       return;

   // if gearin selector or polarity has changed. or value of any of gearin P params has changed
   if ((s16_command_selector_prev != s16_command_selector)    ||
       BGVAR(s16_Gear_Changed_Flag)                           ||
       ((BGVAR(u16_P1_00_Ext_Pulse_Input_Type) & 0x0f00) != u16_ext_polarity_prev)  )
   {
      BGVAR(s16_Gear_Changed_Flag) = 0;

      if (SalGearInCommand((long long)BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[s16_command_selector], drive) == SAL_SUCCESS)
      {
         s16_command_selector_prev = s16_command_selector;
         u16_ext_polarity_prev = BGVAR(u16_P1_00_Ext_Pulse_Input_Type) & 0x0f00;
      }
   }
}

//**********************************************************
// Function Name: ProcessHaltInput
// Description:
//    This function processes the HALT DIN. if HALT==1 the drive is in HALT using the Activate hold.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void ProcessHaltInput(int drive)
{
   // AXIS_OFF;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // nitsan 29/3/2015: HALT input is active only in PT mode (by =s= request).
   if (BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PT)
   {
      // keep the state variable updated (required for dual mode: if toggling input while in non-PT mode)
      BGVAR(u16_Hold_Input_State) = 0;
      return;
   }

   if ((VAR(AX0_u16_Mode_Input_Arr[HALT_INP]) != 0) && BGVAR(u16_Hold_Input_State) == 0)// If was OFF and now is ON turn HALT(HOLD) ON.
   {
      BGVAR(u16_Hold_Input_State) = 1;
      SalHoldCommand(0x1,drive);
      BGVAR(u16_Deceleration_Event_ID) = EVENT_DECSTOP;
   }
   else if ((VAR(AX0_u16_Mode_Input_Arr[HALT_INP]) == 0) &&   // if halt input is off
            (BGVAR(u16_Hold_Input_State) == 1)  &&            // and it was on before
            (BGVAR(u16_Hold_Stop_In_Process) == 0))           // and hold process is not active (i.e. drive finished decelerating).
   {
      BGVAR(u16_Hold_Input_State) = 0;

      // Instruct the gearing function to clear the position deviation.
      VAR(AX0_u16_Gear_BG_Flags) |= 0x0004; // Set bit 2, the flag is cleared automatically in the gearing code.
      SalHoldCommand(0,drive);
   }
}

//*****************************************************************************************
// Function Name: BrakeRelease
// Description: This function handles releasing the brake when enabling the drive.
//              Need to take time measurement between releasing the brake output and accept new motion commands.
//
//         Fix IPR 1429: Motor phase monitor activate,but no response when motor UVW lost and in standstill status.
//         On enable command, dont release the brake, give toruqe and read current feedback.
//         if more than half of given torque, motor phase are connected and continue with brake release and count brake relase time.
//         if less, motor phases are disconnected and fault handler will issue a fault (dont release brake at all).
//
// Author: Nitsan
// Algorithm:
// Revisions:
//*****************************************************************************************
void BrakeRelease(int drive)
{
   static long s32_brake_release_start_time = 0;
   static int s16_opmode_backup = 0;
   int s16_brakeOn_time;
   long s32_t_cmd;

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // u16_Motor_Brake_Release_Time - time that takes the brake to release (from MTP)
   s16_brakeOn_time = BGVAR(u16_P1_42_Brake_Time_On) + BGVAR(u16_Motor_Brake_Release_Time);
   
   // if no brake defiend, no need to inhibit the motion
   if (BGVAR(u16_Brake_Output_Is_Defined) == 0)
   {
      // set flag to true if output mode is changed while break is being released.
      // or if brake time is zero.
      BGVAR(u16_Allow_Motion) = 1;
      BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_IDLE;
      VAR(AX0_s16_Motor_Phase_Disconnect_Offset) = 0;  // reset to original mphase
      return;
   }
   
   // in this state machine, the drive will attempt give torque command while brake is applied,
   // in order to determine if the motor phases are disconnected (prevent fall down on vertical axes).
   // this will be done by switching to serial current opmode and change motor phase by 90 deg.
   // then, give torque comamnd and read current feedback.
   switch (BGVAR(u16_Brake_Release_State))
   {
      case BRAKE_RELEASE_IDLE:
      case BRAKE_RELEASE_REPORT_FAILURE:
         // do nothing
         break;
      
      case BRAKE_RELEASE_CHANGE_OPMODE:
         // enabling is in process, so wait until we are enabled and then change opmode
         if (!Enabled(drive))
         {
            break;
         }
         
         // change opmode
         
         /*if ( IS_LXM28A_DRIVE_CANOPEN  || IS_LXM28E_DRIVE_ECAT )
         {
            s16_opmode_backup = p402_modes_of_operation_display;
            BGVAR(s16_CAN_Opmode_Temp) = 4;  // 4 is profile torque (internal opmode 2)
            BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
            CheckCanOpmode(drive);
         }   
         else*/
         {
            s16_opmode_backup = VAR(AX0_s16_Opmode);
            BGVAR(s16_Start_Opmode_Change) = 2;
         }
         
         BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_WAIT_OPMODE_CHANGED;
         break;
         
      case BRAKE_RELEASE_WAIT_OPMODE_CHANGED:
         if (!IsOpmodeChangeInProgress(drive))
         {
            BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_APPLY_TORQUE;
         }
         
         break;
      case BRAKE_RELEASE_APPLY_TORQUE:
         // opmode changed to serial current mode, set commutation by 90 deg and give current command
         VAR(AX0_s16_Motor_Phase_Disconnect_Offset) = 0x4000;  // 90 degrees offset is 0x4000 in internal units
         
         // give serial current command - comamnd should be 7.5% of dipeak 
		 // because motor phase disconnection works only when current is at least 5% of dipeak
         s32_t_cmd = (long)(((long long)BGVAR(s32_Drive_I_Peak) * 75LL) / 1000.0);
         if (s32_t_cmd > BGVAR(s32_Ilim_Actual))
         {
            s32_t_cmd = BGVAR(s32_Ilim_Actual);
         }
         
         TorqueCommand(s32_t_cmd, drive);
         BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_READ_I_FEEDBACK;
         break;
      
      case BRAKE_RELEASE_READ_I_FEEDBACK:
         // 5% * dipeak
         s32_t_cmd = (long)(((long long)BGVAR(s32_Drive_I_Peak) * 5LL) / 100.0);
		 
         // check if comamnd that arrived to current loop is at least 5% of dipeak (due to saturation from other features).
         if ((long)VAR(AX0_s16_Crrnt_Q_Ref_0) >  s32_t_cmd)
         { 
            if (VAR(AX0_s16_Crrnt_Q_Act_0) > (VAR(AX0_s16_Crrnt_Q_Ref_0) >> 1))
            {
               // success, motor phass connected
               BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_SUCCESS_MOTOR_PHASE_TEST;
            }
            else
            {
               if (BGVAR(s16_DisableInputs) == 0)
               {
                  // failure, motor phases are disconnected
                  BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_FAILURE_MOTOR_PHASE_TEST;
               }
               else
               {
                  // since drive is not enabled, i cannot determine if test failed ro not, so set it to success to prevent false alarm
                  // brake will not be relesed since i check s16_DisableInputs again before releasing it (see below)
                  BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_SUCCESS_MOTOR_PHASE_TEST;
               }
            }
         }
         else
         {
            // cannot determine if motor phases are connected or not
            // release brake (assume test is successfull).
            BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_SUCCESS_MOTOR_PHASE_TEST;
         }
        
         
         // set current comamnd to zero
         TorqueCommand(0LL, drive);
         // restore mphase
         VAR(AX0_s16_Motor_Phase_Disconnect_Offset) = 0;
         // restore opmode
         /*if ( IS_LXM28A_DRIVE_CANOPEN  || IS_LXM28E_DRIVE_ECAT )
         {
            BGVAR(s16_CAN_Opmode_Temp) = s16_opmode_backup;  
            BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
            CheckCanOpmode(drive);
         }
         else*/
         {
            BGVAR(s16_Start_Opmode_Change) = s16_opmode_backup;
         }
         break;
         
      case BRAKE_RELEASE_SUCCESS_MOTOR_PHASE_TEST:
      case BRAKE_RELEASE_FAILURE_MOTOR_PHASE_TEST:
         if (!IsOpmodeChangeInProgress(drive))
         {
            if (BGVAR(u16_Brake_Release_State) == BRAKE_RELEASE_SUCCESS_MOTOR_PHASE_TEST)
            {
               // release brake only if still enabled. to avoid releasing brake if drive got into fault during the state machine
               // use s16_DisableInputs instaed of Enabled() because on active disabled drive is still enabled but fault exists
               if (BGVAR(s16_DisableInputs) == 0)
               {
                  VAR(AX0_u16_BrakeOn) = 0;
               }
                  
               BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_MEASURE_TIME;
               s32_brake_release_start_time = Cntr_1mS;
            }
            else  // failure
            {
               // report fauilure to fault handler 
               BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_REPORT_FAILURE;
            }
         }
         
         break;

      case BRAKE_RELEASE_MEASURE_TIME:
         if (PassedTimeMS(s16_brakeOn_time, s32_brake_release_start_time))
         {
              // time elapsed, allow motion
              BGVAR(u16_Allow_Motion) = 1;
              BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_IDLE;
         }

         break;
   }
}


//**********************************************************
// Function Name: TouchProbeHandler
// Description: Interpolation of the probed samples.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
//#pragma CODE_SECTION(TouchProbeHandler, "ramfunc_5");
void TouchProbeHandler(int drive)
{
   long long s64_temp, s64_temp1, s64_temp2;
   long *s32_used_probe_ptr;
   unsigned int u16_offset,u16_loop_ind;
   int s16_temp;
   Probe_Var_Strct *probe_struct_ptr;
   // AXIS_OFF;
   int axis_offset = drive;

   //In case new configuration word is in being loaded,the probe handler will be cleared,
   if ((VAR(AX0_u16_TProbe_Cmnd) & (PROBE_FUNC_ENABLE_LO | PROBE_FUNC_ENABLE_HI)) == 0)
      return;

   for (u16_loop_ind = 0;u16_loop_ind < 4;u16_loop_ind++)
   {
      ////////////////////////////////////////////////////////////////////////////////////////////////
      // If the RT task has increment at the event counter inside the "AX0_u16_TProbe_Cntr_Arr" array.
      ////////////////////////////////////////////////////////////////////////////////////////////////
      if (BGVAR(u16_Probe_Event_Counter_Hist)[u16_loop_ind] != VAR(AX0_u16_TProbe_Cntr_Arr[u16_loop_ind]))
      {
         BGVAR(u16_Probe_Event_Counter_Hist)[u16_loop_ind] = VAR(AX0_u16_TProbe_Cntr_Arr[u16_loop_ind]);
         // The interpolated samples will be stored according to edge event configuration
         switch(u16_loop_ind)
         {
            case 0:
               probe_struct_ptr = &BGVAR(Probe_1_Rise_Record);
               s32_used_probe_ptr = (long *)(&AX0_s32_TProbe_Arr1[0] + (axis_offset>>1));
            break;
            case 1:
               probe_struct_ptr = &BGVAR(Probe_1_Fall_Record);
               s32_used_probe_ptr = (long *)(&AX0_s32_TProbe_Arr1[0] + PROBE_FALL_ARR_OFFSET + (axis_offset>>1));
            break;
            case 2:
               probe_struct_ptr = &BGVAR(Probe_2_Rise_Record);
               s32_used_probe_ptr = (long *)(&AX0_s32_TProbe_Arr2[0] + (axis_offset>>1));
            break;
            case 3:
               probe_struct_ptr = &BGVAR(Probe_2_Fall_Record);
               s32_used_probe_ptr = (long *)(&AX0_s32_TProbe_Arr2[0] + PROBE_FALL_ARR_OFFSET + (axis_offset>>1));
            break;
         }

         // Status word
         //set the appropriate bit
         // probe #1 bit 1(rise),2(fall)
         // probe #2 bit 9(rise),10(fall)
         //toggle the appropriate bit
         // probe #1 bit 6(rise),7(fall)
         // probe #2 bit 14(rise),15(fall)

         ////////////////////////////////////////////////////////////////////////////////
         // Interpolation
         // f(t)=Y_old*(t-t_old)/(t_old-t_new)-Y_new*(t-t_old)/(t_old-t_new)
         // At our case,t_new=T , t_old=0
         // Therefore f(t)=Y_old+ (Y_new-Y_old)t/T
         // T =1875 -> 0x45E7 >> 25; = 1/1875 (in 16bit representation)
         /////////////////////////////////////////////////////////////////////////////
         // Position samples
         /////////////////////////////////////////////////////////////////////////////
         if ((VAR(AX0_u16_TProbe_Src) & 1) != 0)
         {
            u16_offset = 0;
            // Read PFB and previousPFB thread-safe out of the arrau that has the captured data.
            do
            {
               s16_temp = Cntr_3125;
               s64_temp  = *(long long *)(s32_used_probe_ptr + u16_offset);                        // Read PFB
               s64_temp1 = *(long long *)(s32_used_probe_ptr + PROBE_PREV_ARR_OFFSET +u16_offset); // read previous PFB
               s64_temp2 = s64_temp1;                                                              // save previous PFB
            }
            while (s16_temp != Cntr_3125);

            // Calculate: capturedValue = Value[n-1] + (Value[n] - Value[n-1]) * AX0_u16_TProbe_Capture_Time_Arr[x] * 17895 / 2^25
            //                          =  Value[n-1] + (Value[n] - Value[n-1]) * AX0_u16_TProbe_Capture_Time_Arr[x] / 1875
            // The value 1875 is due to the fact that AX0_u16_TProbe_Capture_Time_Arr[x] holds the time difference
            // between the previous MTS and the capture event in 60[MHz] clock counts and 60,000,000[counts/s]
            // represent 60,000,000 / 32000 = 1875 [Counts/31.25us] (1875 Counts per MTS).
            probe_struct_ptr->s64_position =
            ((long long)((s64_temp - s64_temp1) * VAR(AX0_u16_TProbe_Capture_Time_Arr[u16_loop_ind]) * 0x45E7) >> 25) + s64_temp2;

            BGVAR(s64_TProbe_Value[(int)(u16_loop_ind/2)]) = probe_struct_ptr->s64_position; //Continue the support for Schneider
         }
         /////////////////////////////////////////////////////////////////////////////
         // Position error samples
         /////////////////////////////////////////////////////////////////////////////
         if ((VAR(AX0_u16_TProbe_Src) & 2) != 0)
         {
            u16_offset = 2;
            do
            {
               s16_temp = Cntr_3125;
               s64_temp  = *(long long *)(s32_used_probe_ptr + u16_offset);
               s64_temp1 = *(long long *)(s32_used_probe_ptr + PROBE_PREV_ARR_OFFSET +u16_offset);
               s64_temp2 = s64_temp1;
            }
            while (s16_temp != Cntr_3125);

            // Calculate: capturedValue = Value[n-1] + (Value[n] - Value[n-1]) * AX0_u16_TProbe_Capture_Time_Arr[x] * 17895 / 2^25
            //                          =  Value[n-1] + (Value[n] - Value[n-1]) * AX0_u16_TProbe_Capture_Time_Arr[x] / 1875
            // The value 1875 is due to the fact that AX0_u16_TProbe_Capture_Time_Arr[x] holds the time difference
            // between the previous MTS and the capture event in 60[MHz] clock counts and 60,000,000[counts/s]
            // represent 60,000,000 / 32000 = 1875 [Counts/31.25us] (1875 Counts per MTS).
            probe_struct_ptr->s64_position_err =
            ((long long)((s64_temp - s64_temp1) * VAR(AX0_u16_TProbe_Capture_Time_Arr[u16_loop_ind]) * 0x45E7) >> 25) + s64_temp2;
         }
         /////////////////////////////////////////////////////////////////////////////
         // Velocity samples
         /////////////////////////////////////////////////////////////////////////////
         if ((VAR(AX0_u16_TProbe_Src) & 4) != 0)
         {
            u16_offset = 4;

            // Calculate: capturedValue = Value[n-1] + (Value[n] - Value[n-1]) * AX0_u16_TProbe_Capture_Time_Arr[x] * 17895 / 2^25
            //                          =  Value[n-1] + (Value[n] - Value[n-1]) * AX0_u16_TProbe_Capture_Time_Arr[x] / 1875
            // The value 1875 is due to the fact that AX0_u16_TProbe_Capture_Time_Arr[x] holds the time difference
            // between the previous MTS and the capture event in 60[MHz] clock counts and 60,000,000[counts/s]
            // represent 60,000,000 / 32000 = 1875 [Counts/31.25us] (1875 Counts per MTS).
            probe_struct_ptr->s32_velocity =
            (long)(((((long long)(*(long *)(s32_used_probe_ptr + u16_offset) - *(long *)(s32_used_probe_ptr + PROBE_PREV_ARR_OFFSET + u16_offset)))
            * VAR(AX0_u16_TProbe_Capture_Time_Arr[u16_loop_ind]) * 0x45E7)>>25) + *(long *)(s32_used_probe_ptr + PROBE_PREV_ARR_OFFSET + u16_offset));
         }
         /////////////////////////////////////////////////////////////////////////////
         // IQ samples
         /////////////////////////////////////////////////////////////////////////////
         if ((VAR(AX0_u16_TProbe_Src) & 8) != 0)
         {
            u16_offset = 5;

            // Calculate: capturedValue = Value[n-1] + (Value[n] - Value[n-1]) * AX0_u16_TProbe_Capture_Time_Arr[x] * 17895 / 2^25
            //                          =  Value[n-1] + (Value[n] - Value[n-1]) * AX0_u16_TProbe_Capture_Time_Arr[x] / 1875
            // The value 1875 is due to the fact that AX0_u16_TProbe_Capture_Time_Arr[x] holds the time difference
            // between the previous MTS and the capture event in 60[MHz] clock counts and 60,000,000[counts/s]
            // represent 60,000,000 / 32000 = 1875 [Counts/31.25us] (1875 Counts per MTS).
            probe_struct_ptr->s16_current = (long)(((((long long)(*(long *)(s32_used_probe_ptr + u16_offset) - *(long *)(s32_used_probe_ptr + PROBE_PREV_ARR_OFFSET + u16_offset))) * VAR(AX0_u16_TProbe_Capture_Time_Arr[u16_loop_ind]) * 0x45E7)>>25) + *(long *)(s32_used_probe_ptr + PROBE_PREV_ARR_OFFSET + u16_offset));
         }

         if ((SHNDR == u16_Product) || (SHNDR_HW == u16_Product))
         {
            if (probe_struct_ptr->u16_edge_counter != VAR(AX0_u16_TProbe_Cntr_Arr[u16_loop_ind]))
               { // if event has occured, disable the relevant probe's config p-param (by zeroing the LSB)
                  if ((0 == u16_loop_ind)||(1 == u16_loop_ind))
                     BGVAR(u16_P5_39_CACT) &= 0xFFF0;// the event is on probe#1 - relevant p-param is p5-39
                  else if  ((2 == u16_loop_ind)||(3 == u16_loop_ind))
                     BGVAR(u16_P5_59_CACT2) &= 0xFFF0;// the event is on probe#2 - relevant p-param is p5-59
               }
         }
         // Write also the event counter value to a variable, which is part of the final
         // structure that holds the interpolated values.
         probe_struct_ptr->u16_edge_counter = VAR(AX0_u16_TProbe_Cntr_Arr[u16_loop_ind]);
         ///////////////////////////////////////////////////////////////////////////////////////////////////////////
         // The time stamp is according to FPGA Ts + the previous MTS time. (The FPGA notifies event occured
         // only an MTS afterwards. Therefore AX0_u16_TProbe_Event_Time_Arr - 1_MTS + AX0_u16_TProbe_Capture_Time_Arr.
         // So here we calculate a timestamp in nano-seconds based on _Cntr_3125, which is stored inside the
         // AX0_u16_TProbe_Event_Time_Arr "after" the event has happened (therefore the subtraction with MTS_IN_NANO_SEC).
         ///////////////////////////////////////////////////////////////////////////////////////////////////////////
         probe_struct_ptr->u32_time_stamp =
         (unsigned long)(VAR(AX0_u16_TProbe_Event_Time_Arr[u16_loop_ind]))* MTS_IN_NANO_SEC - MTS_IN_NANO_SEC + (unsigned long)(VAR(AX0_u16_TProbe_Capture_Time_Arr[u16_loop_ind])) * FPGA_TS_IN_NANO_SEC;

         // Updating the Status Word
         VAR(AX0_u16_TProbe_Status) |= ((((u16_loop_ind%2)==0)?((u16_loop_ind==0)?PROBE_STATUS_POS_EDGE_STORED_LO:PROBE_STATUS_POS_EDGE_STORED_HI):((u16_loop_ind==1)?PROBE_STATUS_NEG_EDGE_STORED_LO:PROBE_STATUS_NEG_EDGE_STORED_HI)));
         VAR(AX0_u16_TProbe_Status) ^= ((((u16_loop_ind%2)==0)?((u16_loop_ind==0)?STATUS_RISE_TOGGLE_LO:STATUS_RISE_TOGGLE_HI):((u16_loop_ind==1)?STATUS_FALL_TOGGLE_LO:STATUS_FALL_TOGGLE_HI)));

         //Releasing the probe storage cells ,setting an indicator bit only at continuous configuration
         if ((VAR(AX0_u16_Temp_Cmd) & ((u16_loop_ind <2)?PROBE_FUNC_CONTINUOUS_LO:PROBE_FUNC_CONTINUOUS_HI)) != 0)
         {
            // This place is reached in case that we are doing continuous probing. Here we are
            // clearing the lower bit of the bit-pair inside AX0_u16_TProbe_Lock, in order to
            // unlock the probe functionality in 2[ms] from now (the assembly code takes care of it).
            VAR(AX0_u16_TProbe_Lock) &= ~(SIGNAL_WAIT_TWO_MS<<(u16_loop_ind * 2));
      }
   }
   }

   // nitsan: update the both edges counter for objects 0x4526, 0x453A TPDO support
   BGVAR(u16_Probe_1_Both_Edges_Counter) = BGVAR(Probe_1_Rise_Record).u16_edge_counter + BGVAR(Probe_1_Fall_Record).u16_edge_counter;
   BGVAR(u16_Probe_2_Both_Edges_Counter) = BGVAR(Probe_2_Rise_Record).u16_edge_counter + BGVAR(Probe_2_Fall_Record).u16_edge_counter;
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeCounterReadCommand

// Description:
//          This function is called in response to the PROBECOUNTER command.
//          The function provides two counters per probe.
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbeCounterReadCommand(int drive)
{
   int u16_array_index;
   // AXIS_OFF;
   //////////////////////////////////////////////////////////////////
   REFERENCE_TO_DRIVE;
   for (u16_array_index=0;u16_array_index<2;u16_array_index++)
   {
      PrintCrLf();
      if (2 == BGVAR(u16_Num_Of_Probes_On_Drive))
      {
         if (u16_array_index==0)
            PrintStringCrLf("Probe #1", 0);
         else
            PrintStringCrLf("Probe #2", 0);
      }

      PrintString("Rise edge events: ", 0);
      PrintUnsignedLong(VAR(AX0_u16_TProbe_Cntr_Arr[2*u16_array_index]));
      PrintCrLf();
      PrintString("Fall edge events: ", 0);
      PrintUnsignedLong(VAR(AX0_u16_TProbe_Cntr_Arr[2*u16_array_index+1]));
      PrintCrLf();
      if (2 > BGVAR(u16_Num_Of_Probes_On_Drive)) //CDHD supports only 1 probe
         u16_array_index++;//to exit the for loop
   }
   return SAL_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeStatusCommand
// Description:
//          This function is called in response to the PROBESTATUS command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbeStatusCommand(int drive)
{
   static int u16_index=1;
   // AXIS_OFF;
   char u16_temp_str[40];
   REFERENCE_TO_DRIVE;
   
   if (u8_Output_Buffer_Free_Space < 60) return SAL_NOT_FINISHED;

   PrintCrLf();
   strcpy((char*)&u16_temp_str[0],"Probe -");
   strcat((char*)&u16_temp_str[0],SignedIntegerToAscii(u16_index));
   strcat((char*)&u16_temp_str[0]," Is Switched");
   strcat((char*)&u16_temp_str[0],(VAR(AX0_u16_TProbe_Status) & ((u16_index == 1)?PROBE_STATUS_ENABLE_LO:PROBE_STATUS_ENABLE_HI))?" On":" Off");
   PrintStringCrLf(&u16_temp_str[0],0);
   if ((VAR(AX0_u16_TProbe_Status) & ((u16_index == 1)?PROBE_STATUS_POS_EDGE_STORED_LO:PROBE_STATUS_POS_EDGE_STORED_HI)) != 0)
      PrintStringCrLf("Positive edge data stored",0);
   else
      PrintStringCrLf("No positive edge data stored",0);

   if ((VAR(AX0_u16_TProbe_Status) & ((u16_index == 1)?PROBE_STATUS_NEG_EDGE_STORED_LO:PROBE_STATUS_NEG_EDGE_STORED_HI)) != 0)
      PrintStringCrLf("Negative edge data stored",0);
   else
      PrintStringCrLf("No negative edge data stored",0);

   if ((u16_index==2) || (2 > BGVAR(u16_Num_Of_Probes_On_Drive)))
   {
      u16_index = 1;
      return (SAL_SUCCESS);
   }
   else
   {
      u16_index = 2;
      return SAL_NOT_FINISHED;
   }
}


//**********************************************************
// Function Name: SalProbe1LvlPrdWriteCommand
// Description: Return the value of the probe level period (stabilization time in mts).
//              The array consists of 2 elements as follows:
//                              P param | array index |probe number
//                              --------|----------------------
//                              P5-12   | 0           | 1
//                              P5-77   | 1           | 2
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalProbe1LvlPrdWriteCommand(long long param, int drive)
{
   //SalProbeLevelPeriod gets probenumber 1:2
   //whereas the p_param function,interprets it as 0:1
   s64_Execution_Parameter[0] = 1;
   s64_Execution_Parameter[1] = param;

   SalProbeLevelPeriods(drive);
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalProbe2LvlPrdWriteCommand
// Description: Return the value of the probe level period (stabilization time in mts).
//              The array consists of 2 elements as follows:
//                              P param | array index |probe number
//                              --------|----------------------
//                              P5-12   | 0           | 1
//                              P5-77   | 1           | 2
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalProbe2LvlPrdWriteCommand(long long param, int drive)
{
   //SalProbeLevelPeriod gets probenumber 1:2
   //whereas the p_param function,interprets it as 0:1
   s64_Execution_Parameter[0] = 2;
   s64_Execution_Parameter[1] = param;

   SalProbeLevelPeriods(drive);
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalProbe1LvlPrdReadCommand
// Description: Return the value of the probe level period (stabilization time in mts).
//              The array consists of 2 elements as follows:
//                              P param | array index |probe number
//                              --------|----------------------
//                              P5-12   | 0           | 1
//                              P5-77   | 1           | 2
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalProbe1LvlPrdReadCommand(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *data = (unsigned long long)BGVAR(u16_Probe_Lvl_Prd[0]);
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalProbe2LvlPrdReadCommand
// Description: Return the value of the probe level period (stabilization time in mts).
//              The array consists of 2 elements as follows:
//                              P param | array index |probe number
//                              --------|----------------------
//                              P5-12   | 0           | 1
//                              P5-77   | 1           | 2
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalProbe2LvlPrdReadCommand(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   if (2 > BGVAR(u16_Num_Of_Probes_On_Drive))
      return NOT_SUPPORTED_ON_HW;
   *data = (unsigned long long)BGVAR(u16_Probe_Lvl_Prd[1]);
   return (SAL_SUCCESS);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeConfigWriteCommand
//
// Description: This function is called in response to the PROBECONFIG write command.
//
// PROBECONFIG  <probe_number> ,<enable_use> ,<capture_method>. <probe source> ,<capture_edge> , <sampled_variable>
//
// * probe_number : 1 ,2 (Not supported now)
// * enable_use : 0-disable,1-enable
// * capture_method : 0-one shot,1-continuous
// * probe_source : 0-Digital input,1-Index
// * capture_edge 0-None,1-Rising,2-Falling ,3-Both (Not supported)
// * sampled_variable : bit representation
//   [b]0000-None, [b]0001-actual position, [b]0010-PE, [b]0100-Velocity, [b]1000 - Current, combinations [b]0000:[b]1111
//
// Example : PROBECONFIG 1 1 0 0 1 h03
//
// Author: Gil
// Algorithm:
// Revisions:
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int SalProbeConfigWriteCommand(int drive)
{
   long long    s64_probe_number      = s64_Execution_Parameter[0];
   long long    s64_enable_use        = s64_Execution_Parameter[1];
   long long    s64_capture_method    = s64_Execution_Parameter[2];
   long long    s64_probe_source      = s64_Execution_Parameter[3];
   long long    s64_capture_edge      = s64_Execution_Parameter[4];
   long long    s64_sampled_var       = s64_Execution_Parameter[5];
   int          ret_val;

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (s16_Number_Of_Parameters != 6)
      return SYNTAX_ERROR;

   // For CDHD and Schneider allow positive and negative edge capture
      if ((s64_capture_edge > 3LL) || (s64_capture_edge < 1LL))
         return VALUE_OUT_OF_RANGE;

   if ( ((s64_probe_number < 1LL)   || (s64_probe_number > BGVAR(u16_Num_Of_Probes_On_Drive))) ||
        ((s64_enable_use > 1LL)     || (s64_enable_use < 0LL))                                 ||
        ((s64_capture_method > 1LL) || (s64_capture_method < 0LL))                             ||
        ((s64_sampled_var > 15LL)   || (s64_sampled_var < 0LL))                                ||
        ((s64_probe_source>1LL)     || (s64_probe_source < 0LL))
      )
      return VALUE_OUT_OF_RANGE;

   if ((ret_val=ProbeConfiguration((int)s64_probe_number,(int)s64_enable_use,(int)s64_capture_method,(int)s64_probe_source,(int)s64_capture_edge,PROBE_SLOW_CONF))== SAL_SUCCESS)
      VAR(AX0_u16_TProbe_Src) = (unsigned int)s64_sampled_var;

   return ret_val;
}

//**********************************************************
// Function Name: ProbeConfiguration
// Description:
// This function handles the configuration of the touch probe
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(ProbeConfiguration, "ramcan_2");
int ProbeConfiguration(int s16_probe_number,int s16_enable_use,int s16_capture_method,int s16_probe_source,int s16_capture_edge,int s16_activator)
{
   unsigned int u16_probe_source_input = 0;
   unsigned int u16_config_word_cpy = 0;
   unsigned int u16_temp_config_word = 0;
   unsigned int u16_DS402_Object_60D0 = 0;
   int drive=0;
   int retVal = SAL_SUCCESS;

   // AXIS_OFF;
   //Avoiding conflicts by disabling the PROBE_FUNC_ENABLE till the end of the configuration
   u16_config_word_cpy =VAR(AX0_u16_TProbe_Cmnd);
   VAR(AX0_u16_TProbe_Cmnd) &= ~(PROBE_FUNC_ENABLE_LO | PROBE_FUNC_ENABLE_HI);
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Probe source: digital input OR index (possible under feedback with hw index) OR DS402 object 0x60D0
   // 0-digital input
   // 1 - index signal of the motor feedback
   // 2 - DS402 object 0x60D0 indicates what needs to be used
   switch (s16_enable_use | (s16_probe_source << 1))
   {
      case 1://enabled ,use digital input
         // The following line ensures that the variable "u16_probe_source_input" gets the number of the digital input number,
         // which is assigned to be used for the capture (from 1...s16_Num_Of_Inputs). The FPGA capture input select registers
         // arm the FPGA for a capture via input 1...s16_Num_Of_Inputs in case that a value of 1...s16_Num_Of_Inputs is being
         // written into the register.
         if ((u16_probe_source_input=IsInFunctionalityConfigured(((s16_probe_number==1)?TOUCH_PROBE_1_INP:TOUCH_PROBE_2_INP), drive)) == 0)
         {
            retVal = FUNCTIONALITY_NOT_OCCUPIED;
            break;
         }
      case 0://disabled ,use digital input
         u16_temp_config_word &= ~((s16_probe_number==1)?PROBE_FUNC_INPUT_LO:PROBE_FUNC_INPUT_HI);
      break;

      case 3://enabled ,use index
         if (!IS_DUAL_LOOP_ACTIVE)
         {
            if (((VAR(AX0_s16_Motor_Enc_Type) <= 2) && FEEDBACK_WITH_ENCODER) || FEEDBACK_TAMAGAWA)
            {
               // Via writing the value 12 to the FPGA capture input select registers,
               // we are arming the FPGA for a capture via the main feedback index signal.
               u16_probe_source_input = INDEX_INPUT; //index input
            }
            else if (IS_HW_FUNC_ENABLED(EEO_MASK))
            {
               if (BGVAR(u8_EncoutMode) == 0)
               {
                  retVal = NOT_AVAILABLE;
                  break;
               }
               else
               {
                  // fix IPR 1446: MC_Touchprobe 1(TP1) does not work if MC_Touchprobe 2(TP2) has an Error
                  // restore original capture command if probe 1 is armed and configuration of probe 2 fails
                  VAR(AX0_u16_TProbe_Cmnd) = u16_config_word_cpy;
                  u16_probe_source_input = SIMULATED_INDEX_INPUT; //index input
               }
            }
            else
            {
               retVal = NOT_SUPPORTED_ON_HW; // no index simulation feature supported
               break;
            }
         }
         else //active dual loop
         {
            if (SFB_WITH_INDEX)
            {
               u16_probe_source_input = SFB_INDEX_INPUT;//SFB_INDEX_INPUT; //BARUCH_index input
            }
            else
            {
               return NOT_SUPPORTED_ON_FEEDBACK;//if no index on SFB, don't support index trigger
            }
         }
         if (SAL_SUCCESS == retVal)
            u16_temp_config_word |= ((s16_probe_number==1)?PROBE_FUNC_INPUT_LO:PROBE_FUNC_INPUT_HI);
      break;
      
      case 2://disabled ,use index
         u16_temp_config_word |= ((s16_probe_number==1)?PROBE_FUNC_INPUT_LO:PROBE_FUNC_INPUT_HI);
      break;

      case 5: // enable, consider object 0x60D0 regarding the source
         if(s16_probe_number == 1)
         {
            u16_DS402_Object_60D0 = BGVAR(s16_Touch_Probe_Source_1);
         }
         else if (s16_probe_number == 2)
         {
            u16_DS402_Object_60D0 = BGVAR(s16_Touch_Probe_Source_2);
         }
         else
         {
            return VALUE_OUT_OF_RANGE;
         }

         // See definition of object 0x60D0 inside the ETG document "Directive for using IEC 61800-7-201
         // within EtherCAT-based servo drives" for more details regarding object 0x60D0 (e.g. inside the
         // document "ETG6010_V1i0i0_D_R_CiA402_ImplDirective_no_key.pdf").
         if((u16_DS402_Object_60D0 >= 1) && ((u16_DS402_Object_60D0 <= 4)))
         {
            u16_probe_source_input = u16_DS402_Object_60D0;
         }
         else if(u16_DS402_Object_60D0 == 5)
         {
            u16_probe_source_input = SIMULATED_INDEX_INPUT; //index input
         }
         else
         {
         return VALUE_OUT_OF_RANGE;
   }
      break;

      case 4: // disable, consider object 0x60D0 regarding the source
         if(s16_probe_number == 1)
         {
            u16_DS402_Object_60D0 = BGVAR(s16_Touch_Probe_Source_1);
         }
         else if (s16_probe_number == 2)
         {
            u16_DS402_Object_60D0 = BGVAR(s16_Touch_Probe_Source_2);
         }
         else
         {
            return VALUE_OUT_OF_RANGE;
         }
         if(u16_DS402_Object_60D0 == 5)
         {
            // Set index bit although no meaning since the capture is anyhow disabled
            u16_temp_config_word |= ((s16_probe_number==1)?PROBE_FUNC_INPUT_LO:PROBE_FUNC_INPUT_HI);
         }
         else
         {
            // Clear index bit although no meaning since the capture is anyhow disabled
            u16_temp_config_word &= ~((s16_probe_number==1)?PROBE_FUNC_INPUT_LO:PROBE_FUNC_INPUT_HI);
         }
      break;

      default:
         retVal = VALUE_OUT_OF_RANGE;
         break;
   }

   if (retVal != SAL_SUCCESS)
   {
      // fix IPR 1446: MC_Touchprobe 1(TP1) does not work if MC_Touchprobe 2(TP2) has an Error
      // restore original capture command if probe 1 is armed and configuration of probe 2 fails
      // i restore directly and not using AX0_u16_Temp_Cmd (design var) and PROBE_LOAD_CMND flag
      // because if a previous touch probe is confgured, so in this point all configuration is valid and only
      // the enable var (AX0_u16_TProbe_Cmnd) is set to disable, so enable it here directly is fine.
      VAR(AX0_u16_TProbe_Cmnd) = u16_config_word_cpy;
      return retVal;
   }


   /////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // loading the FPGA register with the used input for probing
   // and setting the probe level period for stabilization (for the FPGA filer)
    ///////////////////////////////////////////////////////////////////////////////////
   if (s16_probe_number == 1)
   {
      *(int *)FPGA_INPUT_CAPTURE_SELECT_2_REG_ADD   = u16_probe_source_input;     // Select trigger Source for input 1
      *(int *)FPGA_CAPTURE2_FILTER_REG_ADD = BGVAR(u16_Probe_Lvl_Prd[0])*1875;    // Select FPGA filter depth for the trigger signal
      if ((u16_probe_source_input > 0) && (u16_probe_source_input < INDEX_INPUT)) // Consider the polarity setting for digital input also in the capture engine
         *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_2_REG_ADD = ((BGVAR(u16_Dig_In_Polar) ^ VAR(AX0_u16_LX28_LS_Location)) & (1<<(u16_probe_source_input-1)))==0?1:0;

   }
   else
   {
      *(int *)FPGA_INPUT_CAPTURE_SELECT_3_REG_ADD = u16_probe_source_input;       // Select trigger Source for input 2
      *(int *)FPGA_CAPTURE3_FILTER_REG_ADD = BGVAR(u16_Probe_Lvl_Prd[1])*1875;    // Select FPGA filter depth for the trigger signal
      if ((u16_probe_source_input > 0) && (u16_probe_source_input < INDEX_INPUT)) // Consider the polarity setting for digital input also in the capture engine
         *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_3_REG_ADD = ((BGVAR(u16_Dig_In_Polar) ^ VAR(AX0_u16_LX28_LS_Location)) & (1<<(u16_probe_source_input-1)))==0?1:0;
   }

   ///////////////////////////////////////////////////////////////////////////////////
   // Capture method: 0 single shot / 1 continuous
   ///////////////////////////////////////////////////////////////////////////////////
   if (s16_capture_method == 1) //1 continuous
      u16_temp_config_word |= ((s16_probe_number == 1)?PROBE_FUNC_CONTINUOUS_LO:PROBE_FUNC_CONTINUOUS_HI);
   if (s16_capture_method == 0) //0 one shot
      u16_temp_config_word &= ~(((s16_probe_number == 1)?PROBE_FUNC_CONTINUOUS_LO:PROBE_FUNC_CONTINUOUS_HI));

   // to reset capture status word when disabling capture control (=S= IPR: BYang00000393)
   // if (s16_enable_use == 0) s16_capture_edge = 0;

   if (s16_capture_edge == 1)  //rise
   {
      VAR(AX0_u16_TProbe_Status) &= ~((s16_probe_number == 1)?PROBE_STATUS_POS_EDGE_STORED_LO:PROBE_STATUS_POS_EDGE_STORED_HI);
      u16_temp_config_word |=  ((s16_probe_number == 1)?PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_LO:PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_HI);
      VAR(AX0_u16_TProbe_Lock) &= ~((s16_probe_number == 1)?0x0003:0x0030);
      if((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      {
         u16_temp_config_word &=  ~((s16_probe_number == 1)?PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_LO:PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_HI);
         VAR(AX0_u16_TProbe_Status) &= ~((s16_probe_number == 1)?PROBE_STATUS_NEG_EDGE_STORED_LO:PROBE_STATUS_NEG_EDGE_STORED_HI);
      }
   }

   if (s16_capture_edge == 2) //fall
   {
      VAR(AX0_u16_TProbe_Status) &= ~((s16_probe_number == 1)?PROBE_STATUS_NEG_EDGE_STORED_LO:PROBE_STATUS_NEG_EDGE_STORED_HI);
      u16_temp_config_word |=  ((s16_probe_number == 1)?PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_LO:PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_HI);
      VAR(AX0_u16_TProbe_Lock) &= ~((s16_probe_number == 1)?0x000C:0x0C0);
      if((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      {
         u16_temp_config_word  &=  ~((s16_probe_number == 1)?PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_LO:PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_HI);
         VAR(AX0_u16_TProbe_Status) &= ~((s16_probe_number == 1)?PROBE_STATUS_POS_EDGE_STORED_LO:PROBE_STATUS_POS_EDGE_STORED_HI);
      }
   }

   if (s16_capture_edge == 3) //rise and fall
   {
      VAR(AX0_u16_TProbe_Status) &= ~((s16_probe_number == 1)?
      (PROBE_STATUS_NEG_EDGE_STORED_LO | PROBE_STATUS_POS_EDGE_STORED_LO):(PROBE_STATUS_NEG_EDGE_STORED_HI | PROBE_STATUS_POS_EDGE_STORED_HI));
      u16_temp_config_word |=  ((s16_probe_number == 1)?
      (PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_LO | PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_LO):(PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_HI | PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_HI));
      VAR(AX0_u16_TProbe_Lock) &= ~((s16_probe_number == 1)?0x000F:0x00F0);
   }

   /////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Enable/Disable touch probe
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////
   if (s16_enable_use == 0)
   {
      //Stop probing
      VAR(AX0_u16_TProbe_Status) &= ~((s16_probe_number == 1)?
      (PROBE_STATUS_ENABLE_LO | PROBE_STATUS_POS_EDGE_STORED_LO | PROBE_STATUS_NEG_EDGE_STORED_LO | STATUS_RISE_TOGGLE_LO | STATUS_FALL_TOGGLE_LO):
      (PROBE_STATUS_ENABLE_HI | PROBE_STATUS_POS_EDGE_STORED_HI | PROBE_STATUS_NEG_EDGE_STORED_HI | STATUS_RISE_TOGGLE_HI | STATUS_FALL_TOGGLE_HI));

      u16_temp_config_word &= ~((s16_probe_number == 1)?PROBE_FUNC_ENABLE_LO:PROBE_FUNC_ENABLE_HI);
      VAR(AX0_u16_TProbe_Lock) &= ~((s16_probe_number == 1)?0x000F:0x00F0);
   }
   else
   {
      u16_temp_config_word |= ((s16_probe_number == 1)?PROBE_FUNC_ENABLE_LO:PROBE_FUNC_ENABLE_HI);
   }
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //Avoid false recognition at FPGA mechanism. In the following line we keep the setting for the other touch
   //probe and just modify the setting of the touch probe, which is supposed to be changed.
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////
   VAR(AX0_u16_Temp_Cmd) = u16_temp_config_word | (u16_config_word_cpy & ((s16_probe_number==1)?0xFF00:0x00FF));

   if (s16_activator == PROBE_SLOW_CONF)
      VAR(AX0_u16_TProbe_Lock) |= PROBE_LOAD_CMND; // Indicate RT task to load "_AX0_u16_TProbe_Cmnd" with value of "_AX0_u16_Temp_Cmd"
   return(SAL_SUCCESS);
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbe1CaptureControlCommand
// Description:
//          This function is called in response to the P_P5-39_CACT write command.
//
//
// Nibble validity check is now preformed in function
// IsPParamValueValid as part of "ExecutePParam" function
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbe1CaptureControlCommand(long long lparam,int drive)
{
   int ret_val;
   unsigned int u16_temp;
   u16_temp = (unsigned int)lparam ;

   STORE_EXECUTION_PARAMS_0_15
   // loading the FPGA register with the required edge event polarity
   // ZYX
   // At Lxm NC=1,NO=0

   s64_Execution_Parameter[0] = 1;
   s64_Execution_Parameter[1] = (u16_temp & 0x000F);
   s64_Execution_Parameter[2] = 0;
   s64_Execution_Parameter[3] = 0;  //use only the touch probe
   s64_Execution_Parameter[4] = ((u16_temp & 0x0F00) >0)?2:1;//fall -2,rise -1
   s64_Execution_Parameter[5] = 1;  //PFB .At schneider ,only position is to be sampled
   s16_Number_Of_Parameters = 6;

   ret_val = SalProbeConfigWriteCommand(drive);
   if (ret_val == SAL_SUCCESS)
   {
      BGVAR(u16_P5_39_CACT) = (unsigned int)lparam ;
      BGVAR(u8_P5_35_Probes_Polarity) = GetProbesPolarity(drive);
   }

   RESTORE_EXECUTION_PARAMS_0_15
   return ret_val;
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbe2CaptureControlCommand
// Description:
//          This function is called in response to the P_P5-59_CACT2 write command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
 //////////////////////////////////////////////////////////////////////////
int SalProbe2CaptureControlCommand(long long lparam,int drive)
{
   int ret_val;
   unsigned int u16_temp;
   u16_temp = (unsigned int)lparam ;

   if (2 > BGVAR(u16_Num_Of_Probes_On_Drive))
      return NOT_SUPPORTED_ON_HW;
   if (((u16_temp & 0x000F)>1) || (u16_temp & 0x0F00)>0x100)
      return VALUE_OUT_OF_RANGE;

   STORE_EXECUTION_PARAMS_0_15
   // loading the FPGA register with the required edge event polarity
   // ZYX
   // At Lxm NC=1,NO=0

   s64_Execution_Parameter[0] = 2;
   s64_Execution_Parameter[1] = (u16_temp & 0x000F);
   s64_Execution_Parameter[2] = 0;
   s64_Execution_Parameter[3] = 0;  //use only the touch probe
   s64_Execution_Parameter[4] = ((u16_temp & 0x0F00) >0)?2:1;//fall -2,rise -1
   s64_Execution_Parameter[5] = 1;  //PFB .At schneider ,only position is to be sampled
   s16_Number_Of_Parameters = 6;

   ret_val = SalProbeConfigWriteCommand(drive);
   if (ret_val == SAL_SUCCESS)
   {
      BGVAR(u16_P5_59_CACT2) = (unsigned int)lparam;
      BGVAR(u8_P5_35_Probes_Polarity) = GetProbesPolarity(drive);
   }

   RESTORE_EXECUTION_PARAMS_0_15
   return ret_val;
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeConfigReadCommand
// Description:
//          This function is called in response to the PROBECAPTURE read command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbeConfigReadCommand(int drive)
{
   int index=0,temp_var=0;
   char u16_temp_str[40];
   // AXIS_OFF;
   drive+=0;

   if ((s64_Execution_Parameter[0]>BGVAR(u16_Num_Of_Probes_On_Drive)) || (s64_Execution_Parameter[0]<1))
      return VALUE_OUT_OF_RANGE;

   if (u8_Output_Buffer_Free_Space < COMMS_BUFFER_SIZE - 10)
      return SAL_NOT_FINISHED;

   index = (unsigned int)s64_Execution_Parameter[0];
   PrintStringCrLf(" ",0);
   // PROBECONFIG  <probe_number> ,<enable_use> ,<capture_method>. <probe source> ,<capture_edge> , <sampled_variable>
   // This function should support two probes.but due to FPGA resources,ony one is supported.

   if (1 < BGVAR(u16_Num_Of_Probes_On_Drive))
   {
      strcpy((char*)&u16_temp_str[0],"Probe number: ");
      strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(index));
      PrintStringCrLf(&u16_temp_str[0],0);
   }

   strcpy((char*)&u16_temp_str[0],"Enable trigger: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii((( VAR(AX0_u16_TProbe_Cmnd) & ((index==1)?PROBE_FUNC_ENABLE_LO:PROBE_FUNC_ENABLE_HI))!=0)?1:0));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"Capture method: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii((( VAR(AX0_u16_TProbe_Cmnd) & ((index==1)?PROBE_FUNC_CONTINUOUS_LO:PROBE_FUNC_CONTINUOUS_HI))!=0)?1:0));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"Probe source: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(((VAR(AX0_u16_TProbe_Cmnd) & ((index==1)?PROBE_FUNC_INPUT_LO:PROBE_FUNC_INPUT_HI))!=0)?1:0));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"Capture edge : ");
   temp_var = 0;
   // Event edge detection
   // Rise edge 0, fall edge 1,both 2
   if ((VAR(AX0_u16_TProbe_Cmnd) & ((index==1)?PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_LO:PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_HI)) != 0)
      temp_var = 1;
   if ((VAR(AX0_u16_TProbe_Cmnd) & ((index==1)?PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_LO:PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_HI)) != 0)
      temp_var |= 2;

   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(temp_var));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"Sampled variable: ");
   strcat((char*)&u16_temp_str[0],UnsignedIntegerToAscii(VAR(AX0_u16_TProbe_Src)));
   PrintStringCrLf(&u16_temp_str[0],0);

   return(SAL_SUCCESS);
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeLevelPeriod
// Description:
//          This function is called in response to the PROBELEVELPRD command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbeLevelPeriod(long long param,int drive)
{
   drive+=0;
   BGVAR(u16_Probe_Lvl_Prd[0]) = (unsigned int)param;
   *(int *)FPGA_CAPTURE2_FILTER_REG_ADD = BGVAR(u16_Probe_Lvl_Prd[0])*1875;
   return SAL_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeLevelPeriods
// Description:
//          This function is called in response to the PROBELEVELPRDX command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbeLevelPeriods(int drive)
{
   drive+=0;
   if ((s64_Execution_Parameter[0]<1) || (s64_Execution_Parameter[0]>BGVAR(u16_Num_Of_Probes_On_Drive)))
      return (VALUE_OUT_OF_RANGE);

   if (s64_Execution_Parameter[1]<2)
      return (VALUE_TOO_LOW);
   if (s64_Execution_Parameter[1]>32)
      return (VALUE_TOO_HIGH);

   BGVAR(u16_Probe_Lvl_Prd[(unsigned int)(s64_Execution_Parameter[0] -1)]) = (unsigned int)s64_Execution_Parameter[1];

   if (s64_Execution_Parameter[0] == 1)
      *(int *)FPGA_CAPTURE2_FILTER_REG_ADD = BGVAR(u16_Probe_Lvl_Prd[0])*1875;
   else
      *(int *)FPGA_CAPTURE3_FILTER_REG_ADD = BGVAR(u16_Probe_Lvl_Prd[1])*1875;
   return SAL_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeLevelFltReadCommand
// Description:
//          This function is called in response to the PROBELEVELFLT command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbeLevelFltReadCommand(int drive)
{
   int u16_probe_number=(unsigned int)s64_Execution_Parameter[0];
   drive+=0;
   if ((s64_Execution_Parameter[0]<1) || (s64_Execution_Parameter[0]>BGVAR(u16_Num_Of_Probes_On_Drive)))
      return (NOT_SUPPORTED_ON_HW);

   PrintUnsignedInteger(BGVAR(u16_Probe_Lvl_Prd[u16_probe_number -1]));
   PrintString(" [31.25 us]", 0);
   PrintCrLf();
   return SAL_SUCCESS;
}


//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeLevelPrdWriteCommand
// Description:
//          This function is called in response to the PROBELEVELFLT command.
//
//
// Author: APH
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbeLevelFltWriteCommand(int drive)
{
   int u16_probe_number;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0]<1) || (s64_Execution_Parameter[0]>BGVAR(u16_Num_Of_Probes_On_Drive)))
   {
      return (NOT_SUPPORTED_ON_HW);
   }

   if ((s64_Execution_Parameter[1]<2) || (s64_Execution_Parameter[1]>32))
   {
      return (VALUE_OUT_OF_RANGE);
   }

   u16_probe_number = (unsigned int)s64_Execution_Parameter[0];

   BGVAR(u16_Probe_Lvl_Prd[u16_probe_number -1]) = (unsigned int)s64_Execution_Parameter[1];

   *(int *)FPGA_CAPTURE2_FILTER_REG_ADD = BGVAR(u16_Probe_Lvl_Prd[0])*1875; // 1875[60MHz-Counts] per 31.25[us]
   *(int *)FPGA_CAPTURE3_FILTER_REG_ADD = BGVAR(u16_Probe_Lvl_Prd[1])*1875; // 1875[60MHz-Counts] per 31.25[us]

   return SAL_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeDataRiseCommand
// Description:
//          This function is called in response to the PROBEDATARISE command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbeDataRiseCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0]>2) || (s64_Execution_Parameter[0]<1))
      return VALUE_OUT_OF_RANGE;

   return(ProbeDataCommand(((s64_Execution_Parameter[0] == 1LL)?&BGVAR(Probe_1_Rise_Record):&BGVAR(Probe_2_Rise_Record))));
}

//////////////////////////////////////////////////////////////////////////
// Function Name: SalProbeDataFallCommand
// Description:
//          This function is called in response to the PROBEDATAFALL command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalProbeDataFallCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0]>2) || (s64_Execution_Parameter[0]<1))
      return VALUE_OUT_OF_RANGE;
   return(ProbeDataCommand((s64_Execution_Parameter[0] == 1)?&BGVAR(Probe_1_Fall_Record):&BGVAR(Probe_2_Fall_Record)));
}
//////////////////////////////////////////////////////////////////////////
// Function Name: ProbeDataCommand
// Description:
//          This function is prints the probed data
//
// Author: Gil
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int ProbeDataCommand(void *data_strct_ptr)
{
   int loop_index = 0;
   long long s64_converted_val,s64_raw_value=0;
   int  conv_index=0, drive = 0;
   char u16_temp_str[40];
   // AXIS_OFF;

   if (u8_Output_Buffer_Free_Space < COMMS_BUFFER_SIZE - 10)
      return SAL_NOT_FINISHED;
   //////////////////////////////////////////////////////////////////////////
   // In case nothing was configured to be sampled
   //////////////////////////////////////////////////////////////////////////
   if (VAR(AX0_u16_TProbe_Src)== 0)
   {
      PrintStringCrLf("No variable is probed", 0);
     return(SAL_SUCCESS);
   }
   //////////////////////////////////////////////////////////////////////////
   // Scaning which variable was required to be sampled
   //////////////////////////////////////////////////////////////////////////
   for (;loop_index<4;loop_index++)
   {
     s64_raw_value = 0;
     if ((VAR(AX0_u16_TProbe_Src) & 1<<loop_index) != 0)
      {
         switch (loop_index)
         {
           // cases 1:2 both use the same unit conversion
            case 0:
             s64_raw_value= (((Probe_Var_Strct *)data_strct_ptr)->s64_position);
             if (!IS_DUAL_LOOP_ACTIVE)
             {
               conv_index = POSITION_CONVERSION;
               strcpy((char*)&u16_temp_str[0],(char*)&s8_Units_Pos[0]);
             }
             else
             {
               conv_index = LOAD_POSITION_CONVERSION;
               strcpy((char*)&u16_temp_str[0],(char*)&s8_Units_Pos_Sfb[0]);
             }
           
           break;
           case 1:
             s64_raw_value= (((Probe_Var_Strct *)data_strct_ptr)->s64_position_err);
             if (!IS_DUAL_LOOP_ACTIVE)
             {
               conv_index = POSITION_CONVERSION;
               strcpy((char*)&u16_temp_str[0],(char*)&s8_Units_Pos[0]);
             }
             else
             {
               conv_index = LOAD_POSITION_CONVERSION;
               strcpy((char*)&u16_temp_str[0],(char*)&s8_Units_Pos_Sfb[0]);
             }  

           break;
           case 2:
             s64_raw_value= (long long)(((Probe_Var_Strct *)data_strct_ptr)->s32_velocity);
             if (!IS_DUAL_LOOP_ACTIVE)
             {
               conv_index = VELOCITY_OUT_OF_LOOP_CONVERSION;
               strcpy((char*)&u16_temp_str[0],(char*)&s8_Units_Vel_Out_Loop[0]);
             }
             else
             {
               conv_index = VELOCITY_LOAD_OUT_OF_LOOP_CONVERSION;
               strcpy((char*)&u16_temp_str[0],(char*)&s8_Units_Vel_Out_Loop_Sfb[0]);
             }

           break;
           case 3:
             s64_raw_value= (long long)(((Probe_Var_Strct *)data_strct_ptr)->s16_current);
               conv_index = CURRENT_CONVERSION;
               strcpy((char*)&u16_temp_str[0],(char*)&s8_Units_Cur[0]);

           break;
          default:
          break;
        }
         s64_converted_val = MultS64ByFixS64ToS64(s64_raw_value ,
                                       BGVAR(Unit_Conversion_Table[conv_index]).s64_unit_conversion_to_user_fix,
                                       BGVAR(Unit_Conversion_Table[conv_index]).u16_unit_conversion_to_user_shr);

         PrintSignedLongLong(s64_converted_val);
         PrintChar(SPACE);
         PrintChar('[');

         PrintString((char *)&u16_temp_str[0] + (drive * UNITS_STRING_SIZE),0);
         PrintChar(']');
         PrintCrLf();
      }
   }
   return(SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadTouchProbe1Value
// Description:
//   This function returns P_P5-37_CAAX.
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalReadTouchProbe1Value(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *data = BGVAR(s64_TProbe_Value[0]);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadTouchProbe2Value
// Description:
//   This function returns P_P5-57_CAAX2.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalReadTouchProbe2Value(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   if (2 > BGVAR(u16_Num_Of_Probes_On_Drive))
      return NOT_SUPPORTED_ON_HW;

   *data = BGVAR(s64_TProbe_Value[1]);

   return SAL_SUCCESS;
}

// Revisions:
//**********************************************************
unsigned char GetProbesPolarity(int drive)
{
   unsigned char u8_bit_0, u8_bit_1;

   drive += 0;

   // open/close polarity is specificed in Z nibble of P5-39 (probe 1) and P5-59 (probe 2).
   // z Nibbble == 1 -> normally open
   // z Nibbble == 2 -> normally closed
   u8_bit_0 = ((BGVAR(u16_P5_39_CACT)  & 0x0100) != 0);
   u8_bit_1 = ((BGVAR(u16_P5_59_CACT2) & 0x0100) != 0);
   return (u8_bit_0 | (u8_bit_1 << 1));
}

//**********************************************************
// Function Name: SalReadTouchProbe1ValueCanOpenUnits
// Description:
//   This function returns the captured position in CANopen position user units.
//   for P5-36
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalReadTouchProbe1ValueCanOpenUnits(long long *data,int drive)
{
   drive += 0;

   // convert captured position from internal units to canopen units
   *data = (long)MultS64ByFixS64ToS64(BGVAR(s64_TProbe_Value[0]),
                   BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                   BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

   return SAL_SUCCESS;
}

//**********************************************************
//   for P5-56
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalReadTouchProbe2ValueCanOpenUnits(long long *data,int drive)
{
   drive += 0;

   // convert captured position from internal units to canopen units
   *data = (long)MultS64ByFixS64ToS64(BGVAR(s64_TProbe_Value[1]),
                   BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                   BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

   return SAL_SUCCESS;
}

int SalWriteMCOKCommand(long long param,int drive)
{// valid value is 0x00 , 0x01 , 0x10 , 0x11 , 0x21
 // values that are higher the 0x11 or smaller then 0 are validated by the excel range check.
   REFERENCE_TO_DRIVE;
   if((param & 0x0FLL) > 0x01LL) return (VALUE_OUT_OF_RANGE);
   if((param & 0xF0LL) > 0x20LL) return (VALUE_OUT_OF_RANGE);
   BGVAR(u16_P1_48_MCOK_Output_Selection) = (unsigned int)param;
   return(SAL_SUCCESS);
}

void ProcessPcomInputs(int drive)
{
    REFERENCE_TO_DRIVE;

    //  Checking if Pcom 1 was enabled from input
    if (VAR(AX0_u16_Mode_Input_Arr[PCOM_1_EN_DIS_INP]) && (!BGVAR(s16_Pcom_1_En_Des_Prev_State)))
    {
        SalWritePcomCntrl1Command((long long)PCOM_ENABLE_BIT, drive);
    }
    //  Checking if Pcom 1 was disabled from input
    else if (!VAR(AX0_u16_Mode_Input_Arr[PCOM_1_EN_DIS_INP]) && (BGVAR(s16_Pcom_1_En_Des_Prev_State)))
    {
        SalWritePcomCntrl1Command((long long)PCOM_DISABLE_BIT, drive);
    }

    //  Checking if Pcom 2 was enabled from input
    if (VAR(AX0_u16_Mode_Input_Arr[PCOM_2_EN_DIS_INP]) && (!BGVAR(s16_Pcom_2_En_Des_Prev_State)))
    {
        SalWritePcomCntrl2Command((long long)PCOM_ENABLE_BIT, drive);
    }
    //  Checking if Pcom 2 was disabled from input
    else if (!VAR(AX0_u16_Mode_Input_Arr[PCOM_2_EN_DIS_INP]) && (BGVAR(s16_Pcom_2_En_Des_Prev_State)))
    {
        SalWritePcomCntrl2Command((long long)PCOM_DISABLE_BIT, drive);
    }

    //  Set the Prev state of Pcom 1 & 2
    BGVAR(s16_Pcom_1_En_Des_Prev_State) = VAR(AX0_u16_Mode_Input_Arr[PCOM_1_EN_DIS_INP]);
    BGVAR(s16_Pcom_2_En_Des_Prev_State) = VAR(AX0_u16_Mode_Input_Arr[PCOM_2_EN_DIS_INP]);
}

//**********************************************************
// Function Name: ProcessSfbPfbComparisonEnable
// Description:
//   This function is responsible for some settings needed
//   by the "SFBPositionErrorHandler" function
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void ProcessSfbPfbComparisonEnable(int drive)
{
   if ((BGVAR(s16_SFBMode) != 0) && IsInFunctionalityConfigured(EN_SFB_PFB_COMPARE, drive))
   {
      // A digital input is assigned to trigger realignment and activate the PFB-SFB comparison.
      BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) |= SFB_PFB_COMPARE_DIG_INPUT_CONFIGURED;
   }
   else
   {
      // No digital input is assigned to trigger realignment and activate the PFB-SFB comparison.
      BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) &= ~SFB_PFB_COMPARE_DIG_INPUT_CONFIGURED;
   }
}

void ProcessUserJogModesInputs (int drive)
{
   unsigned int u16_user_jog_mode_bits = 0;
   static unsigned int u16_User_Jog_Mode_Bits_Previous = 0x00, u16_Enable_First_User_Jog = 0;
   long long s64_user_jog_vel_temp = s64_User_Jog_Vel, s64_user_jog_vel_temp2 = s64_User_Jog_Vel2;
   // AXIS_OFF;
   
   u16_user_jog_mode_bits &= 0;
   u16_user_jog_mode_bits |=  (VAR(AX0_u16_Mode_Input_Arr[JOG_POSITIVE_DIR_INP1]) > 0);
   u16_user_jog_mode_bits |= ((VAR(AX0_u16_Mode_Input_Arr[JOG_NEGATIVE_DIR_INP1]) > 0) << 1);
   u16_user_jog_mode_bits |= ((VAR(AX0_u16_Mode_Input_Arr[JOG_POSITIVE_DIR_INP2]) > 0) << 2);
   u16_user_jog_mode_bits |= ((VAR(AX0_u16_Mode_Input_Arr[JOG_NEGATIVE_DIR_INP2]) > 0) << 3);
   
   if ((!u16_Enable_First_User_Jog) && (0 == u16_user_jog_mode_bits))// This is to avoid Jog on power up when DI is left ON. must go threw mode 0 once.
      u16_Enable_First_User_Jog = 1;
   
   if ((u16_user_jog_mode_bits == u16_User_Jog_Mode_Bits_Previous) || (!u16_Enable_First_User_Jog)) //return if nothing changed, or if drive woke up with DI ON,
      return;
      
   u16_User_Jog_Mode_Bits_Previous =  u16_user_jog_mode_bits; //aging
      
   if (0 < (u16_user_jog_mode_bits & (u16_user_jog_mode_bits - 1))) // if more than 1 input is raised, treet as 0.
   {
      u16_user_jog_mode_bits = 0;
      BGVAR(u64_Sys_Warnings) |= CONTRADICT_DIGITAL_INPUT_ON_WRN_MASK;  
      u16_Contradict_Func_DI_On |= 0x2;    
   }      
      
   else
   {
      u16_Contradict_Func_DI_On &= ~(0x2);
      if (0 == u16_Contradict_Func_DI_On)
         BGVAR(u64_Sys_Warnings) &= ~CONTRADICT_DIGITAL_INPUT_ON_WRN_MASK;
      if (0 != VAR(AX0_s16_Opmode))
        SalOpmodeCommand(0LL, drive); // if jog command needed, first change opmode to 0 
   }
   
   if (s64_User_Jog_Vel > (long long)BGVAR(s32_V_Lim_Design))
      s64_user_jog_vel_temp = (long long)BGVAR(s32_V_Lim_Design);
   if (s64_User_Jog_Vel2 > (long long)BGVAR(s32_V_Lim_Design))
      s64_user_jog_vel_temp2 = (long long)BGVAR(s32_V_Lim_Design);
   
   STORE_EXECUTION_PARAMS_0_1
      
   switch (u16_user_jog_mode_bits) 
   {
      case (0)://u16_user_jog_mode_bits 0000
      s64_Execution_Parameter[0] = 0LL;
      break;
      
      case (1)://u16_user_jog_mode_bits 0001
      s64_Execution_Parameter[0] = s64_user_jog_vel_temp;
      break;
      
      case (2)://u16_user_jog_mode_bits 0010
      s64_Execution_Parameter[0] = -s64_user_jog_vel_temp;
      break;
      
      case (4)://u16_user_jog_mode_bits 0100
      s64_Execution_Parameter[0] = s64_user_jog_vel_temp2;
      break; 
      
      case (8)://u16_user_jog_mode_bits 1000
      s64_Execution_Parameter[0] = -s64_user_jog_vel_temp2;
      break;       
   }
   JogCommand(s64_Execution_Parameter[0],drive);   //give the JOG command 
   RESTORE_EXECUTION_PARAMS_0_1  
}

void ProcessEncoderFollowerInputs (int drive)
{
   unsigned int u16_enc_follower_mode_bits = 0, u16_mode = 0;
   // AXIS_OFF;
   static unsigned int u16_Enc_Follower_Mode_Bits_Previous = 1; //in case no input is assigned, dont change inital state (default mode 1) 
   //create the mode bits - LSB is follower_input1, next is follower_input2, next follower_input3
   u16_enc_follower_mode_bits &= 0;
   u16_enc_follower_mode_bits |=  (VAR(AX0_u16_Mode_Input_Arr[ENC_FOLLOWER_INP_1]) > 0);
   u16_enc_follower_mode_bits |= ((VAR(AX0_u16_Mode_Input_Arr[ENC_FOLLOWER_INP_2]) > 0) << 1);
   u16_enc_follower_mode_bits |= ((VAR(AX0_u16_Mode_Input_Arr[ENC_FOLLOWER_INP_3]) > 0) << 2);
   u16_enc_follower_mode_bits |= ((VAR(AX0_u16_Mode_Input_Arr[ENC_FOLLOWER_INP_4]) > 0) << 3);
   u16_enc_follower_mode_bits |= ((VAR(AX0_u16_Mode_Input_Arr[ENC_FOLLOWER_INP_5]) > 0) << 4);
   
   if (( 0 == u16_enc_follower_mode_bits)                       &&
      (0 == (IsInFunctionalityConfigured(ENC_FOLLOWER_INP_1, drive))) &&
      (0 == (IsInFunctionalityConfigured(ENC_FOLLOWER_INP_2, drive))) &&
      (0 == (IsInFunctionalityConfigured(ENC_FOLLOWER_INP_3, drive))) &&
      (0 == (IsInFunctionalityConfigured(ENC_FOLLOWER_INP_4, drive))) &&
      (0 == (IsInFunctionalityConfigured(ENC_FOLLOWER_INP_5, drive))) ) //if no input is assigned as ENC_FOLLOWER - go to default state
         u16_enc_follower_mode_bits = 1;
      
   if (0 < (u16_enc_follower_mode_bits & (u16_enc_follower_mode_bits - 1))) // if more than 1 input is raised, treet as 0.
   {
      BGVAR(u64_Sys_Warnings) |= CONTRADICT_DIGITAL_INPUT_ON_WRN_MASK;
      u16_Contradict_Func_DI_On |= 0x1;
      u16_enc_follower_mode_bits = 0;
   }
   else
   {
      u16_Contradict_Func_DI_On &= ~(0x1);
      if (0 == u16_Contradict_Func_DI_On)
         BGVAR(u64_Sys_Warnings) &= ~CONTRADICT_DIGITAL_INPUT_ON_WRN_MASK;
   }
   
   if (u16_enc_follower_mode_bits == u16_Enc_Follower_Mode_Bits_Previous) //nothing to do
      return;
           
   else 
   {  
      VAR(AX0_s16_Delta_Qeps_At_Start) = VAR(AX0_s16_2ndQep_Original) - VAR(AX0_s16_2ndQep_Prev);
      u16_Enc_Follower_Mode_Bits_Previous =  u16_enc_follower_mode_bits; //aging
      VAR(AX0_s16_One_Dir_Gear_DB_Value) = VAR(AX0_s16_One_Dir_Gear_DB_Limit_Value)>>1;
   }
   
   while (u16_enc_follower_mode_bits > 0)
   {
      u16_enc_follower_mode_bits >>= 1;
      u16_mode++;      
   }
   VAR(AX0_u16_Enc_Follower_Mode) = u16_mode;
   
   /*   
   switch (u16_enc_follower_mode_bits) 
   {
      case (0)://u16_enc_follower_mode_bits 00000
      VAR(AX0_u16_Enc_Follower_Mode) = 0;
      break;
      
      case (1)://u16_enc_follower_mode_bits 00001
      VAR(AX0_u16_Enc_Follower_Mode) = 1;
      break;
      
      case (2)://u16_enc_follower_mode_bits 00010
      VAR(AX0_u16_Enc_Follower_Mode) = 2;
      break;
      
      case (4)://u16_enc_follower_mode_bits 00100
      VAR(AX0_u16_Enc_Follower_Mode) = 3;
      break;
      
      case (8)://u16_enc_follower_mode_bits 01000
      VAR(AX0_u16_Enc_Follower_Mode) = 4;
      break;
      
      case (16)://u16_enc_follower_mode_bits 10000
      VAR(AX0_u16_Enc_Follower_Mode) = 5;
      break;
      
      default://u16_enc_follower_mode_bits = everything else. 
      VAR(AX0_u16_Enc_Follower_Mode) = 0;
   }  */
   if (0 == VAR(AX0_u8_Gear_Active)) //activate gearing
      VAR(AX0_u8_Gear_Active) = 1;
   if ((4 != VAR(AX0_s16_Opmode)) && (0 != u16_mode))
      SalOpmodeCommand(4LL, drive); // if gear mode needed, change opmode to 4
       
}


//**********************************************************
// Function Name: UpdateWhoTriggeredPath
// Description:
//   This function is responsible for identifying the access-channel that determines
//   the input level of the digital input, that triggers a path command. This information
//   is needed for the access-rights management.
//
//   This function sets the variable "BGVAR(s16_Access_Channel_That_Triggers_Path_Input_Level)".
//
//    u16_expected_P3_06_value - Value that the P3-06 Sal function is about to set.
//    u16_expected_P4_07_value - Value that the P4-07 Sal function is about to set.
//    u16_input_mode_changed - Variable that indicates which input mode recently changed (bit 0 = input 1, bit 1 = input 2...)
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void UpdateWhoTriggeredPath(int drive, unsigned int u16_expected_P3_06_value, unsigned int u16_expected_P4_07_value, unsigned int u16_input_mode_changed)
{
   // AXIS_OFF;

   unsigned int u16_index  = 1; // Index to go through all forced input modes
   int s16_channel_that_triggers_path_input = LEX_CH_IO_CONTROL;
   unsigned int u16_function_state = 0;   // 0 = No input has been assigned to trigger a path
                                          // 1 = An input has been assigned to trigger a path but no need to change "BGVAR(s16_Access_Channel_That_Triggers_Path_Input_Level)"
                                          // 2 = An input has been assigned to trigger a path and "BGVAR(s16_Access_Channel_That_Triggers_Path_Input_Level)" needs to be updated.

   // Here figure out what bits are supposed to be changed in P3-06 or P4-07 via an XOR instruction
   unsigned int u16_P3_06_changes = u16_expected_P3_06_value ^ VAR(AX0_u16_P3_06_SDI);
   unsigned int u16_P4_07_changes = u16_expected_P4_07_value ^ VAR(AX0_u16_P4_07_ITST);
   REFERENCE_TO_DRIVE;
   // Scan through all available inputs
   for (u16_index = 1; u16_index <= (unsigned int)s16_Num_Of_Inputs; u16_index++)
   {
      // If input level is supposed to be forced to true and if there is a change in either P3-06 or P4-07
      if((u16_expected_P3_06_value & (1 << (u16_index-1))) && ((u16_P3_06_changes & (1 << (u16_index-1)))  || (u16_P4_07_changes & (1 << (u16_index-1)))))
      {
         // Save the history. What is the last channel that forced digital input "u16_index" to a certain state.
         BGVAR(s16_Last_Channel_That_Issued_Forcing_IOs)[u16_index-1] = BGVAR(s16_Lexium_ExecPParam_Acces_Channel);
      }

      // If the input mode is set to trigger a path. NOTE THAT ONLY ONE INPUT SHOULD BE ASSIGNED TO TRIGGER A PATH!
      if (VAR(AX0_u16_Input_Mode_Arr[u16_index]) == PATH_CTRG_INP)
      {
         u16_function_state = 1;// State that at least one input is supposed to trigger a path

         // If input level is supposed to be forced to true and if there is a change in either P3-06 or P4-07
         if((u16_expected_P3_06_value & (1 << (u16_index-1))) && ((u16_P3_06_changes & (1 << (u16_index-1)))  || (u16_P4_07_changes & (1 << (u16_index-1)))))
         {
            // Save the channel that forced right now the input to true or false, which is supposed to triggeres a path.
            s16_channel_that_triggers_path_input = BGVAR(s16_Lexium_ExecPParam_Acces_Channel);
            u16_function_state = 2; // Apply the value
         }
         // If input mode has just been changed in order to trigger path and if the input is forced
         else if ((u16_input_mode_changed & (1 << (u16_index-1))) && (u16_expected_P3_06_value & (1 << (u16_index-1))))
         {
            // Save the channel that has changed either P3-06 or P4-07 for the last time.
            s16_channel_that_triggers_path_input = BGVAR(s16_Last_Channel_That_Issued_Forcing_IOs)[u16_index-1];
            u16_function_state = 2; // Apply the value
         }
         else if((u16_expected_P3_06_value & (1 << (u16_index-1))) == 0x0000) // No forcing of input that triggers a path
         {
            // If the input that triggers a path is not forced at all, it means that the hardware IO has control.
            s16_channel_that_triggers_path_input = LEX_CH_IO_CONTROL;
            u16_function_state = 2; // Apply the value
         }

         break; // Leave the while loop after 1 input was assigned to trigger a path
      }
   }

   if(u16_function_state == 0) // No input assigned to trigger a path
   {
      // Put variable to default
      BGVAR(s16_Access_Channel_That_Triggers_Path_Input_Level) = LEX_CH_IO_CONTROL;
   }
   else if(u16_function_state == 2) // An update of "BGVAR(s16_Access_Channel_That_Triggers_Path_Input_Level)" is required
   {
      BGVAR(s16_Access_Channel_That_Triggers_Path_Input_Level) = s16_channel_that_triggers_path_input;
   }
}

//**********************************************************
// Function Name: SalWriteForceInputCommand
// Description:
//   This function is called upon writing to P3-06.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteForceInputEnableCommand(long long param, int drive)
{
   // AXIS_OFF;

   UpdateWhoTriggeredPath(drive, (unsigned int)param, VAR(AX0_u16_P4_07_ITST), 0);

   VAR(AX0_u16_P3_06_SDI) = (unsigned int)param;

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteForceInputStateCommand
// Description:
//   This function is called upon writing to P4-07.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteForceInputStateCommand(long long param, int drive)
{
   // AXIS_OFF;

   UpdateWhoTriggeredPath(drive, VAR(AX0_u16_P3_06_SDI), (unsigned int)param, 0);

   VAR(AX0_u16_P4_07_ITST) = (unsigned int)param;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SetInmodeFromConfigFile
// Description:
//   This function is called upon writing to P2-10 - P2-17 (set inmode for LXM drive)
//   If the required INMODE is already ocupied to one of the input parameters
//   remove the old assignment and set the INMODE to the required INPUT.
//
// Author: Moshe A
// Algorithm:
// Revisions:
//**********************************************************

int SetInmodeFromConfigFile(int drive, int s16_p_param, long s32_inmode)
{
   int s16_return_value, s16_inmode_assigned;
   long s32_idle_input = 0x100;

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
   { // Not LXM drive
      return NOT_PROGRAMMABLE;
   }

   s16_inmode_assigned = IsInFunctionalityConfigured(DigitalInModeSchneiderToCDHDConvert((s32_inmode & 0xFF)), drive);

   if(s16_inmode_assigned != 0)
   {// inmode is occupied: Remove functionality (Set to Idle).
      s16_return_value = ExecutePParam(0, (209 + s16_inmode_assigned), OP_TYPE_WRITE, &s32_idle_input, LEX_CH_MODBUS_RS485);
   }
   // Now inmode is not occupied assign it to the requested input.
   s16_return_value = ExecutePParam(0, s16_p_param, OP_TYPE_WRITE, &s32_inmode, LEX_CH_MODBUS_RS485);


   return s16_return_value;
}


//**********************************************************
// Function Name: ProcessAutoRInputs
// Description:
//   This function is processing AutoR, StepB, StepU and StepD
//   input combinations for AutoR feature operating.
// Author: Moshe A
// Algorithm:
// Revisions:
//**********************************************************
void ProcessAutoRInputs(drive)
{
   // AXIS_OFF;
   if(VAR(AX0_u16_Mode_Input_Arr[AUTOR_INP]))
   {// If AutoR input is active check if operating mode is AutoR (mode 1)
      if(BGVAR(u16_AutoR_State) == AUTOR_IDLE_STAT)
      {// start AUTOR sequence
         // reset the finished path
         BGVAR(u16_AutoR_Finished_Path) = 0;
         BGVAR(u16_AutoR_State) = AUTOR_EXEC_STAT;
      }
   }
   else
   {// pause the AurtoR
      if(BGVAR(u16_AutoR_State) != AUTOR_IDLE_STAT)
      {
         BGVAR(u16_AutoR_State) = AUTOR_IDLE_STAT;
      }
   }

   // Detect StepB rising edge
   if(VAR(AX0_u16_Mode_Input_Arr[STEPB_INP]) && (BGVAR(u16_AutoR_StepB_Last_State) == 0))
   {// If Step Back is active
      // reset the auto run to the first PATH and run it immidiatly
      BGVAR(u16_AutoR_Running_Path) = 1;
      BGVAR(u16_AutoR_Finished_Path) = 0;
      BGVAR(u16_AutoR_State) = AUTOR_RUN_STAT;
      AutoRPathExec(drive, 1);
   }
   // save the current input state
   BGVAR(u16_AutoR_StepB_Last_State) = VAR(AX0_u16_Mode_Input_Arr[STEPB_INP]);

   // Detect StepU rising edge - execute the next path immidiatlly
   if(VAR(AX0_u16_Mode_Input_Arr[STEPU_INP]) && (BGVAR(u16_AutoR_StepU_Last_State) == 0))
   {// If Step Up is active
      //inc the running path by 1
      BGVAR(u16_AutoR_Running_Path)++;
      if(BGVAR(u16_AutoR_Running_Path) < NUM_OF_PATHS)
      {//Execute the next PATH
         AutoRPathExec(drive, 1);
      }
      else
      {
         BGVAR(u16_AutoR_Running_Path) = 32;
      }
   }
   // save the current input state
   BGVAR(u16_AutoR_StepU_Last_State) = VAR(AX0_u16_Mode_Input_Arr[STEPU_INP]);

   // Detect StepD rising edge
   if(VAR(AX0_u16_Mode_Input_Arr[STEPD_INP]) && (BGVAR(u16_AutoR_StepD_Last_State) == 0))
   {// If Step Down is active
      //dec the running path by 1
      BGVAR(u16_AutoR_Running_Path)--;
      if(BGVAR(u16_AutoR_Running_Path) > 0)
      {//Execute the previous PATH
         AutoRPathExec(drive, 1);
      }
      else
      {
         BGVAR(u16_AutoR_Running_Path) = 1;
      }
   }
   // save the current input state
   BGVAR(u16_AutoR_StepD_Last_State) = VAR(AX0_u16_Mode_Input_Arr[STEPD_INP]);
}


//**********************************************************
// Function Name: SalWriteAutoRunDOMode
// Description:
//   This function sets ON or OFF the AUTOR Digital outputs special mode.
//                                                      When P2-44 = 0 Digital outputs is in regular mode.
//                                                      When P2-44 = 1 Digital outputs is in AOUTOR Special mode.
//   The original OUTPUT6 (fast output) polaritywill be saved in BIT1 of "u16_P2_44_Autor_DoModeSet" variable so it will be saved in the flash memory
//   the orginial polarity configuration will not be lost after power cycle and will revert by call "FastOutInv" when P2-44 will reset back
//
// Author: Moshe A
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteAutoRunDOMode(long long param,int drive)
{
   // AXIS_OFF;
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return NOT_PROGRAMMABLE;

   // change P2-44 On/Off only in position mode.
   //if(VAR(AX0_s16_Opmode) != 8) return INVALID_OPMODE;
   
   // Fix IPR 1431: LXM26 Downloading offline project into the drive always fails on P2-35
   // dont return INVALID_OPMODE, instead save the p-param but dont execute the actual functionality.
   // only perform the actual functionality if drive is in PS mode.
   if (BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PS)
   {
      BGVAR(u16_P2_44_Autor_DoModeSet) = param;
      return SAL_SUCCESS;
   }
   

   if (param == 0)
   {// revert the original fast output mux condition and reset the fast output polarity

      //REVERT output 6 functionalitY
      *(int*)FPGA_EEO_Z_OC_MUX_REG_ADD = ((BGVAR(u16_P2_44_Autor_DoModeSet) >> 1) & 0X01);

      BGVAR(u16_P2_44_Autor_DoModeSet) = 0;

      // reset the "FORCED" bit in order to update ZSPD output.
      if((BGVAR(u16_Force_Output_Enable) & 0x1) == 0)
      {// reset the force bit only if outputs are not force (when P2-08 != 406)
         VAR(AX0_s16_ZSPD_Out_Num_Forced) &= ~0x0400;
      }

      // The "FastOutInv" call will revert the polarity changed for OUT3 and OUT6 to original according to "u16_Dig_Out_Polar" variable which was not changed.
      FastOutInv(drive);
   }
   else
   {// if Autor Output mode save the original fast output mux condition in bit1 of "u16_P2_44_Autor_DoModeSet" and force the fast output polarity to normal.

      // Force the fast outputs polarity to normal and revert it by call the "FastOutInv" function.
      *(unsigned int*)FPGA_FPGA_C_OUT_POLARITY_REG_ADD &= 0xFB;
      *(unsigned int*)FPGA_FPGA_M_FAST_OUT_POLARITY_REG_ADD &= 0xFB;

      //save output 6 for functionality mux (in bits 1 and above)
      BGVAR(u16_P2_44_Autor_DoModeSet) = (((*(int*)FPGA_EEO_Z_OC_MUX_REG_ADD) << 1) & 0X02);

      // after saving mux to bit 1 and above, enable the feature in bit 0
      BGVAR(u16_P2_44_Autor_DoModeSet) |= 0x1;

      // set the "FORCED" bit in order to ignore ZSPD output update
      VAR(AX0_s16_ZSPD_Out_Num_Forced) |= 0x400;

      //set output6 for GPIO functionality
      *(int*)FPGA_EEO_Z_OC_MUX_REG_ADD = 1;

      // write "0" to "u16_AutoR_OutMode" to reset all outputs
      BGVAR(u16_AutoR_OutMode) = AUTOR_OUT_MODE_ALARM;
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadAutoRunDOMode
// Description:
//   This function return the AUTOR Digital outputs special mode condition.
//                                                      When P2-44 = 0 Digital outputs is in regular mode.
//                                                      When P2-44 = 1 Digital outputs is in AOUTOR Special mode.
//
// Author: Moshe A
// Algorithm:
// Revisions:
//**********************************************************
int SalReadAutoRunDOMode(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   // return only the first bit which indicates if the special output mode is active(1) or not(0)
   *data = (long long) (BGVAR(u16_P2_44_Autor_DoModeSet) & 0x1);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadZSPDLpfHz
// Description:
//   This function Read the value of the ZSPD Low Pass Filter.
//
// Author: Moshe A
// Algorithm:
// Revisions:
//**********************************************************
int SalReadZSPDLpfHz(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   *data = (long long) BGVAR(u16_ZSPD_LPF_Hz);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadZSPDLpfHz
// Description:
//   This function Sets the value of the ZSPD Low Pass Filter.
//
// Author: Moshe A
// Algorithm:
// Revisions:
//**********************************************************
int SalZSPDLpfHzCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;
   BGVAR(u16_ZSPD_LPF_Hz) = (unsigned int) param;

   //call the design function to calculate Alpha & Beta.
   ZSPDLpfAlphaBetaRecalculation(drive);

   return SAL_SUCCESS;
}

void ZSPDLpfAlphaBetaRecalculation(int drive)
{
   // AXIS_OFF;
   // ZSPD LPF design

   REFERENCE_TO_DRIVE;
   if(BGVAR(u16_ZSPD_LPF_Hz) < 1000)
   {
      VAR(AX0_u16_ZSPD_Lpf_Alpha) = (unsigned int) (32768.0 * exp(-2 * 3.1416 * 0.000125 * (float)BGVAR(u16_ZSPD_LPF_Hz)) + 0.5);
      VAR(AX0_u16_ZSPD_Lpf_Beta) = (unsigned int) (32768L - (long)VAR(AX0_u16_ZSPD_Lpf_Alpha));
      VAR(AX0_u16_ZSPD_Lpf_Shr) = 14; // 1 additional SHR is preformed in RT
   }
   else
   {// freq == 1000Hz -> transparent filter
      VAR(AX0_u16_ZSPD_Lpf_Alpha) = 0;
      VAR(AX0_u16_ZSPD_Lpf_Beta) = 0x4000;
      VAR(AX0_u16_ZSPD_Lpf_Shr) = 13; // 1 additional SHR is preformed in RT
   }
}

//***********************PCOM******************************/
int SalWritePcomDirCommand1(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM1_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(u16_PCom_Dir1) = (unsigned int)param;  
   return(SAL_SUCCESS);
}

int SalWritePcomDirCommand2(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM2_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(u16_PCom_Dir2) = (unsigned int)param;  
   return(SAL_SUCCESS);
}

int SalWritePcomEndCommand1(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM1_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(s64_PCom_End1) = param;  
   return(SAL_SUCCESS);
}

int SalWritePcomEndCommand2(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM2_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(s64_PCom_End2) = param;  
   return(SAL_SUCCESS);
}

int SalWritePcomGapCommand1(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM1_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(s64_PCom_Peirod1) = param;  
   return(SAL_SUCCESS);
}

int SalWritePcomGapCommand2(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM2_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(s64_PCom_Peirod2) = param;  
   return(SAL_SUCCESS);
}

int SalWritePcomStartCommand1(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM1_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(s64_PCom_Start1) = param;  
   return(SAL_SUCCESS);
}

int SalWritePcomStartCommand2(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM2_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(s64_PCom_Start2) = param;  
   return(SAL_SUCCESS);
}

int SalWritePcomTableLenCommand1(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM1_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(u16_Pcom_Table_Len1) = (unsigned int)param;  
   return(SAL_SUCCESS);
}

int SalWritePcomTableLenCommand2(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM2_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(u16_Pcom_Table_Len2) = (unsigned int)param;  
   return(SAL_SUCCESS);
}

int SalWritePcomWidthCommand1(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM1_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(u32_Pcom_Pulse_Width1) = (unsigned long)param;  
   return(SAL_SUCCESS);
}

int SalWritePcomWidthCommand2(long long param,int drive)
{
   drive += 0;
   //do not allow to change when PCOM feature is enabled
   if(IS_PCOM2_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   BGVAR(u32_Pcom_Pulse_Width2) = (unsigned long)param;  
   return(SAL_SUCCESS);
}

int SalReadPcomTable1Command(void)
{
   long long index = s64_Execution_Parameter[0];
   long long s64_temp = 0LL;

   //data validation
   if ((index < 1LL) || (index > PCOM_TABLE_SIZE)) return (VALUE_OUT_OF_RANGE);

   //units conversion
   s64_temp = MultS64ByFixS64ToS64(s64_Pcom_Table1_Arr[index-1],
                       BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                       BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);

   PrintString(DecimalPoint32ToAscii((long)s64_temp), 0);   
   PrintChar(SPACE);
   PrintChar('[');
   PrintString((char *)s8_Units_Pos,0);
   PrintChar(']');
   PrintCrLf(); 

   return (SAL_SUCCESS);
}

int SalWritePcomTable1Command(void)
{
   long long index = s64_Execution_Parameter[0], value = s64_Execution_Parameter[1];

   //data validation
   if ((index < 1LL) || (index > PCOM_TABLE_SIZE)) return (VALUE_OUT_OF_RANGE);

   //do not allow to change point when PCOM feature is enabled
   if(IS_PCOM1_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   //Units conversion + update array
   s64_Pcom_Table1_Arr[index-1] = MultS64ByFixS64ToS64(value,
                                           BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                           BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_internal_shr);
   return(SAL_SUCCESS);
}

int SalReadPcomTable2Command(void)
{
   long long index = s64_Execution_Parameter[0];
   long long s64_temp = 0LL;

   //data validation
   if ((index < 1LL) || (index > PCOM_TABLE_SIZE)) return (VALUE_OUT_OF_RANGE);

   //units conversion
   s64_temp = MultS64ByFixS64ToS64(s64_Pcom_Table2_Arr[index-1],
                       BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                       BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);


   PrintString(DecimalPoint32ToAscii((long)s64_temp), 0);  
   PrintChar(SPACE);
   PrintChar('[');
   PrintString((char *)s8_Units_Pos,0);
   PrintChar(']');
   PrintCrLf(); 

   return (SAL_SUCCESS);
}

int SalWritePcomTable2Command(void)
{
   long long index = s64_Execution_Parameter[0], value = s64_Execution_Parameter[1];

   //data validation
   if ((index < 1LL) || (index > PCOM_TABLE_SIZE)) return (VALUE_OUT_OF_RANGE);

   //do not allow to change point when PCOM feature is enabled
   if(IS_PCOM2_EN)
      return (NOT_ALLOWED_ON_PCOM_EN);

   //Units conversion + update array
   s64_Pcom_Table2_Arr[index-1] = MultS64ByFixS64ToS64(value,
                                           BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                           BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_internal_shr);
   return(SAL_SUCCESS);
}

int SalWritePcomCntrl1Command(long long param,int drive)
{
   unsigned int u16_init_pcom = 0;   
   unsigned int u16_init_error = 0;

   drive+=0;

   if(((((unsigned int)param & PCOM_CTRL_TYPE_MASK) >> 1) & 0x0003) == 3) //type range is 0 - 2 - reject the command if avove range
   {

      u16_init_error = PCOM_TYPE_NOT_EXIST;
	  BGVAR(u16_PCom_Stat1) &= PCOM_CLEAR_ERRORS_MASK;// clear errors
      BGVAR(u16_PCom_Stat1) |= ((u16_init_error - PCOM_ERRORS_START_ADDRESS)<< 1); //write new error
      return u16_init_error;
   }

   // if PCOM feature is diasble and a command is given to enable PCOM - Run the PCOM init function.
   if((!IS_PCOM1_EN) && ((unsigned int)param & 0x0001))
   {
      u16_init_pcom = 1;
   }
   
   // update PCOM control variable.
   BGVAR(u16_PCom_Cntrl1) = (unsigned int)param;

   // update PCOM struct
   BGVAR(Pcom_1_Cntrl_Word).u16_en          = ((BGVAR(u16_PCom_Cntrl1)  & PCOM_CTRL_EN_MASK) & 0x0001);                    // en/dis bit (bit 0)
   BGVAR(Pcom_1_Cntrl_Word).u16_type        = (((BGVAR(u16_PCom_Cntrl1) & PCOM_CTRL_TYPE_MASK) >> 1) & 0x0003);            // type bit (bit 1&2)
   BGVAR(Pcom_1_Cntrl_Word).u16_start_cond  = (((BGVAR(u16_PCom_Cntrl1) & PCOM_CTRL_START_CONDITION_MASK) >> 3) & 0x0001); // start condition bit (bit 3)
   BGVAR(Pcom_1_Cntrl_Word).u16_output_type = (((BGVAR(u16_PCom_Cntrl1) & PCOM_CTRL_OUTPUT_TYPE_MASK) >> 4) & 0x0001);     // output type bit (bit 4)


   // handle bit 0 - en/dis PCOM bit
   if(!(BGVAR(Pcom_1_Cntrl_Word).u16_en)) PcomDisable(1);

   // handle bits 1&2 - type PCOM bits - range testing was done above
   *((unsigned int*)FPGA_PCOM1_TYPE_REG_ADD) =  BGVAR(Pcom_1_Cntrl_Word).u16_type;

   // handle bit 3 - 
   //Handled on PcomInit()

   // handle bit 4 - PCOM pulse/level output signal bits
   *((unsigned int*)FPGA_PCOM1_PULSE_LEVEL_REG_ADD) = BGVAR(Pcom_1_Cntrl_Word).u16_output_type;

   if(u16_init_pcom)
   {
      u16_init_error = PcomInit(1);
   }

   if(u16_init_error == PCOM_NO_ERROR)
      return(SAL_SUCCESS);
   else
      return u16_init_error;
}

int SalWritePcomCntrl2Command(long long param,int drive)
{
   unsigned int u16_init_pcom = 0;   
   unsigned int u16_init_error = 0;

   drive+=0;

   if(((((unsigned int)param & PCOM_CTRL_TYPE_MASK) >> 1) & 0x0003) == 3) //type range is 0 - 2 - reject the command if avove range
   {
      u16_init_error = PCOM_TYPE_NOT_EXIST;
	  BGVAR(u16_PCom_Stat2) &= PCOM_CLEAR_ERRORS_MASK;// clear errors
      BGVAR(u16_PCom_Stat2) |= ((u16_init_error - PCOM_ERRORS_START_ADDRESS)<< 1);
      return u16_init_error;
   }

   // if PCOM feature is diasble and a command is given to enable PCOM - Run the PCOM init function.
   if((!IS_PCOM2_EN) && ((unsigned int)param & 0x0001))
   {
      u16_init_pcom = 1;
   }
   
   // update PCOM control variable.
   BGVAR(u16_PCom_Cntrl2) = (unsigned int)param;

   // update PCOM struct
   BGVAR(Pcom_2_Cntrl_Word).u16_en          = ((BGVAR(u16_PCom_Cntrl2)  & PCOM_CTRL_EN_MASK) & 0x0001);                    // en/dis bit (bit 0)
   BGVAR(Pcom_2_Cntrl_Word).u16_type        = (((BGVAR(u16_PCom_Cntrl2) & PCOM_CTRL_TYPE_MASK) >> 1) & 0x0003);            // type bit (bit 1&2)
   BGVAR(Pcom_2_Cntrl_Word).u16_start_cond  = (((BGVAR(u16_PCom_Cntrl2) & PCOM_CTRL_START_CONDITION_MASK) >> 3) & 0x0001); // start condition bit (bit 3)
   BGVAR(Pcom_2_Cntrl_Word).u16_output_type = (((BGVAR(u16_PCom_Cntrl2) & PCOM_CTRL_OUTPUT_TYPE_MASK) >> 4) & 0x0001);     // output type bit (bit 4)

   // handle bit 0 - en/dis PCOM bit
   if(!BGVAR(Pcom_2_Cntrl_Word.u16_en)) PcomDisable(2);

   // handle bits 1&2 - type PCOM bits - range testing was done above
   *((unsigned int*)(FPGA_PCOM1_TYPE_REG_ADD + FPGA_PCOM2_OFFSET)) =  BGVAR(Pcom_2_Cntrl_Word).u16_type;

   // handle bit 3 - 
   //Handled on PcomInit()

   // handle bit 4 - PCOM pulse/level output signal bits
   *((unsigned int*)(FPGA_PCOM1_PULSE_LEVEL_REG_ADD + FPGA_PCOM2_OFFSET)) = BGVAR(Pcom_2_Cntrl_Word).u16_output_type;

   if(u16_init_pcom)
   {
      u16_init_error = PcomInit(2);
   }

   if(u16_init_error == PCOM_NO_ERROR)
      return(SAL_SUCCESS);
   else
      return u16_init_error;
}


int SalReadPcomCntrl1Command(int drive)
{
   char u16_temp_str[40];

   drive+=0;

   if(u8_is_dump_command_active)
   {
      PrintUnsignedInteger(BGVAR(u16_PCom_Cntrl1));
      PrintStringCrLf(" ",0);
      return (SAL_SUCCESS);
   }

   if (u8_Output_Buffer_Free_Space < COMMS_BUFFER_SIZE - 10)
      return SAL_NOT_FINISHED;

   PrintStringCrLf(" ",0);
   strcpy((char*)&u16_temp_str[0],"PCOM Number 1: ");
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"Enable PCOM: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(Pcom_1_Cntrl_Word).u16_en));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Type: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(Pcom_1_Cntrl_Word).u16_type));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Start Condition: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(Pcom_1_Cntrl_Word).u16_start_cond));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Output Type: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(Pcom_1_Cntrl_Word).u16_output_type));
   PrintStringCrLf(&u16_temp_str[0],0);

   return (SAL_SUCCESS);
}

int SalReadPcomCntrl2Command(int drive)
{
   char u16_temp_str[40];
   drive+=0;

   if(u8_is_dump_command_active)
   {
      PrintUnsignedInteger(BGVAR(u16_PCom_Cntrl2)); 
      PrintStringCrLf(" ",0);
      return (SAL_SUCCESS);
   }

   if (u8_Output_Buffer_Free_Space < COMMS_BUFFER_SIZE - 10)
      return SAL_NOT_FINISHED;

   PrintStringCrLf(" ",0);
   strcpy((char*)&u16_temp_str[0],"PCOM Number 2: ");
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"Enable PCOM: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(Pcom_2_Cntrl_Word).u16_en));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Type: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(Pcom_2_Cntrl_Word).u16_type));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Start Condition: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(Pcom_2_Cntrl_Word).u16_start_cond));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Output Type: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(Pcom_2_Cntrl_Word).u16_output_type));
   PrintStringCrLf(&u16_temp_str[0],0);

   return (SAL_SUCCESS);
}

int SalReadPcomStatus1Command()
{
   char u16_temp_str[40];

   if (u8_Output_Buffer_Free_Space < COMMS_BUFFER_SIZE - 10)
      return SAL_NOT_FINISHED;

   PrintStringCrLf(" ",0);
   strcpy((char*)&u16_temp_str[0],"PCOM Number 1: ");
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Status: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(u16_PCom_Stat1) & 0x0001));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Last Error: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii((BGVAR(u16_PCom_Stat1) >> 1) & 0x000F));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Last Warning: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii((BGVAR(u16_PCom_Stat1) >> 5) & 0x000F));
   PrintStringCrLf(&u16_temp_str[0],0);

   return SAL_SUCCESS;
}

int SalReadPcomStatus2Command(void)
{
   char u16_temp_str[40];

   if (u8_Output_Buffer_Free_Space < COMMS_BUFFER_SIZE - 10)
      return SAL_NOT_FINISHED;

   PrintStringCrLf(" ",0);
   strcpy((char*)&u16_temp_str[0],"PCOM Number 2: ");
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Status: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii(BGVAR(u16_PCom_Stat2) & 0x0001));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Last Error: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii((BGVAR(u16_PCom_Stat2) >> 1) & 0x000F));
   PrintStringCrLf(&u16_temp_str[0],0);

   strcpy((char*)&u16_temp_str[0],"PCOM Last Warning: ");
   strcat((char*)&u16_temp_str[0], UnsignedIntegerToAscii((BGVAR(u16_PCom_Stat2) >> 5) & 0x000F));
   PrintStringCrLf(&u16_temp_str[0],0);

   return SAL_SUCCESS;
}

int PcomInit(int s16_pcom_num)
{
   int s16_ret = 0;
   long long* p_s64_start;
   long long* p_s64_period;
   long long  s64_end;
   long long  s64_new_end   = 0x7FFFFFFFFFFFFFFFLL; // init for start condition 0 
   long long  s64_new_start = 0x8000000000000000LL; // init for start condition 0 
   unsigned int* p_u16_table_len;
   unsigned long* p_u32_width;
   unsigned int* p_u16_dir;
   unsigned int* p_u16_stat;
   long long s64_N = 0LL;
   unsigned int ret_value = PCOM_NO_ERROR;

   long long *p_s64_table;
   int i = 0;
   static unsigned int u16_homing_failed = 0;

   PCOM_Cntrl_Word* p_cntrl_word;
   // AXIS_OFF;

   //PCOM DIFPORTMODE init
   STORE_EXECUTION_PARAMS_0_1;
   s16_Number_Of_Parameters = 2;

   for(i=1;i<=PCOM_MAX_DIFFERENTIAL_PORTS;i++)
   {
      s64_Execution_Parameter[0] = i;
      s64_Execution_Parameter[1] = u16_Dif_Port_Mode[i - 1LL];
      SalDifPortModeCommand();
   }
   RESTORE_EXECUTION_PARAMS_0_1;

   if(s16_pcom_num == 1)
   {
      p_s64_start     = &BGVAR(s64_PCom_Start1);
      s64_end         = BGVAR(s64_PCom_End1);
	  p_s64_period    = &BGVAR(s64_PCom_Peirod1);
	  p_u16_table_len = &BGVAR(u16_Pcom_Table_Len1);
	  p_s64_table     = s64_Pcom_Table1_Arr;
	  p_cntrl_word    = &BGVAR(Pcom_1_Cntrl_Word);
	  p_u16_dir       = &BGVAR(u16_PCom_Dir1);
	  p_u32_width     = &BGVAR(u32_Pcom_Pulse_Width1);
	  p_u16_stat      = &BGVAR(u16_PCom_Stat1);

   }
   else if(s16_pcom_num == 2)
   {
      p_s64_start     = &BGVAR(s64_PCom_Start2);
      s64_end         = BGVAR(s64_PCom_End2);
	  p_s64_period    = &BGVAR(s64_PCom_Peirod2);
      p_u16_table_len = &BGVAR(u16_Pcom_Table_Len2);
	  p_s64_table     = s64_Pcom_Table2_Arr;
	  p_cntrl_word    = &BGVAR(Pcom_2_Cntrl_Word);
	  p_u16_dir       = &BGVAR(u16_PCom_Dir2);
	  p_u32_width     = &BGVAR(u32_Pcom_Pulse_Width2);
	  p_u16_stat      = &BGVAR(u16_PCom_Stat2);

   }
   else //not supposed to happen
   {
      return UNKNOWN_COMMAND;
   }

   // Run "PCOM Table mode" check (only on Table type)
   if((p_cntrl_word->u16_type == PCOM_POSITIONS_TABLE) && (!PcomValidateTable(s16_pcom_num)))
   {
       ret_value = PCOM_TABLE_NOT_SORTED;
   }
   // Run "PCOM Periodic mode" check (only on Periodic type)
   else if((p_cntrl_word->u16_type == PCOM_PERIODIC) && (*p_s64_start > s64_end))
   {
      ret_value = PCOM_PERIODIC_VALUES_WRONG;
   }
   // Run output mode allocated check
   else if((s16_pcom_num == 1) && (!IsOutFunctionalityConfigured(PCOM1_OUT)) && (u16_Dif_Port_Mode[0] != 1) && (u16_Dif_Port_Mode[1] != 1) && (u16_Dif_Port_Mode[2] != 1))
   {
      ret_value = PCOM_OUTPUT_MODE_NOT_ALLOCATED;
   }

   else if((s16_pcom_num == 2) && (!IsOutFunctionalityConfigured(PCOM2_OUT)) && (u16_Dif_Port_Mode[0] != 2) && (u16_Dif_Port_Mode[1] != 2) && (u16_Dif_Port_Mode[2] != 2))
   {
      ret_value = PCOM_OUTPUT_MODE_NOT_ALLOCATED;
   }
   // Run "Motor homed" check
   else if((!VAR(AX0_u16_Abs_Fdbk_Device)) && (!VAR(AX0_u16_Home_Ind)))// ABS Feedback - no need to home PCOM
   {
      ret_value =  PCOM_DRIVE_NOT_HOMED;
   }

   // Run "Feedback faults" check
   else if (((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0) ||((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0))   
   {
      ret_value =  PCOM_FEEDBACK_NOT_DEFINED;
   }
   else
   {
      do
      {
         s16_ret = PcomHoming(s16_pcom_num);	                            
	     if(s16_ret != PCOM_NO_ERROR)
	     {
            u16_homing_failed++;
	     }
      }while((s16_ret != PCOM_NO_ERROR) && (u16_homing_failed < PCOM_MAX_HOMING_FAILED_ALLOWED));
   }
   
   if(u16_homing_failed >= PCOM_MAX_HOMING_FAILED_ALLOWED)
   {
      ret_value = s16_ret;
   }
   //All tests passed successfully.  Start writing values to FPGA.
   //Set Periodic or table mode data to FPGA
   else if(ret_value == PCOM_NO_ERROR)
   {   
      if(p_cntrl_word->u16_type == PCOM_PERIODIC)//periodic mode
      {
         if(p_cntrl_word->u16_start_cond)//PCOM enabled between PCOMSTART and PCOMEND. PCOMCNTRL bit3 = 1
	     {
            //we must set "end point" so that: "end point" = "start point" + X * "Period value" (FPGA can not do that)
            s64_N = (long long)((s64_end - *p_s64_start) / *p_s64_period);
		    if(s64_N<0) s64_N = -s64_N; 
            s64_new_end = *p_s64_start + (s64_N * (*p_s64_period));
	     
            s64_new_start = *p_s64_start;	     	  
	     }
	     // else PCOM enabled starting on current psoition -  PCOMCNTRL bit3 = 0 - nothing to do (variables are initialized with those values)
	  	          
	     //We use "Table" FPGA registers to save "periodic" data since "Table" & "Periodic" can not co-exist and we want to save FPGA registers
         //We use first five table mode registers to save periodic data according to the below:
	     //1st entry - Init start value
	     //2nd entry - Init end value
	     //3rd entry - start value
	     //4th entry - end value
	     //5th entry - N / Gap value      
	     //Write init start value
         PcomWriteTableEntryToFpga(s64_new_start,2,s16_pcom_num);
         PcomWriteTableEntryToFpga(s64_new_end,3,s16_pcom_num);
         PcomWriteTableEntryToFpga((*p_s64_period),4,s16_pcom_num);

         //Set low/high table index value on FPGA - Also in periodic we must tell FPGA the index to start
         *((unsigned int*)(FPGA_PCOM1_DSP_SET_TABLE_ADDRESS_LOW_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1)))  = 0;
         *((unsigned int*)(FPGA_PCOM1_DSP_SET_TABLE_ADDRESS_HIGH_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = 1;	  	  
      }
      else if(p_cntrl_word->u16_type == PCOM_POSITIONS_TABLE)//table mode
      {
         //Download positions table to FPGA
         for(i=0;i<*p_u16_table_len;i++)
         {   
	        PcomWriteTableEntryToFpga((long long)p_s64_table[i],i,s16_pcom_num);
         }

         // Write PCOM table length value
         *((unsigned int*)(FPGA_PCOM1_NUMBER_OF_POSITIONS_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = *p_u16_table_len;

      }
      //else// time based
      //{
         //TBD
      //}

      //Write to FPGA data which is common for all PCOM types (table, periodic, time based)

      //write compare direction:0-neg,1-pos,2-bi
      *((unsigned int*)(FPGA_PCOM1_FEEDBACK_MOVEMENT_DIRECTION_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = *p_u16_dir;

      //write pulse width in FPGA units
      *((unsigned long*)(FPGA_PCOM1_PULSE_WIDTH_16LSB_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = ((*(unsigned long *)p_u32_width) * 6); //1 unit on FPGA register = 16.666ns hence 0.1us = 6 units on FPGA register

      //Set A/B Encoder or not A/B Encoder
      if(BGVAR(u16_FdbkType) == INC_ENC_FDBK)// A/B Encoder
         *((unsigned int*)(FPGA_PCOM1_A_QUAD_B_INPUT_SELECT_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = 0;
      else// Not A/B Encoder
         *((unsigned int*)FPGA_PCOM1_A_QUAD_B_INPUT_SELECT_REG_ADD) = 1;
   
      // Write init data to FPGA
      *((unsigned int*)(FPGA_PCOM1_DSP_INITIALIZE_COMMAND_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = 1;
      *((unsigned int*)(FPGA_PCOM1_DSP_INITIALIZE_COMMAND_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = 0;

      //Enable feature on FPGA
      *((unsigned int*)(FPGA_PCOM1_EVENT_ENABLE_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = 1;
   }//end of else - All tests passed successfully.  Start writing values to FPGA.

   u16_homing_failed = 0;

   //update status data
   if(ret_value == PCOM_NO_ERROR)
   {
      *p_u16_stat &= PCOM_CLEAR_ERRORS_MASK;// clear errors
	  
      //signal that PCOM feature is enabled
      *p_u16_stat |= PCOM_STAT_EN_MASK;
   }
   else
   {
      *p_u16_stat &= PCOM_CLEAR_ERRORS_MASK;// clear errors

      //load error to status variable
	  *p_u16_stat |= ((ret_value - PCOM_ERRORS_START_ADDRESS)<< 1);
   }    
 
   return ret_value;
} 

int PcomHoming(int s16_pcom_num)
{
   unsigned long u32_pfb_lo = 0L;
   long s32_pfb_hi = 0L;
   long long s64_pfb1 = 0LL,s64_pfb2 = 0LL;
   unsigned int u16_temp_time = 0;
   int s16_low_index = 0;
   int s16_high_index = 0;
   unsigned int*  p_u16_table_len = 0;
   long long*     p_s64_table = 0;
   long long s64_FPGA_Value = 0LL;
  // unsigned int* p_u16_is_homing_done;
   long long* p_s64_start;
   long long* p_s64_period;
   long long* p_s64_end;
   long long  s64_init_start = 0LL;
   long long  s64_init_end = 0LL;
   long long  s64_point = 0LL;
   long long  s64_N = 0LL;
   PCOM_Cntrl_Word* p_cntrl_word;

   int i = 0;
   // AXIS_OFF;

   //use pointers to handle both PCOM 1 & 2
   if (s16_pcom_num == 1)
   {
      p_u16_table_len = &BGVAR(u16_Pcom_Table_Len1);
	  p_s64_table     = s64_Pcom_Table1_Arr;
      p_s64_start     = &BGVAR(s64_PCom_Start1);
      p_s64_end         = &BGVAR(s64_PCom_End1);
	  p_s64_period    = &BGVAR(s64_PCom_Peirod1);
	  p_cntrl_word = &BGVAR(Pcom_1_Cntrl_Word);
   }
   else
   {
      p_u16_table_len = &BGVAR(u16_Pcom_Table_Len2);
	  p_s64_table     = s64_Pcom_Table2_Arr;
      p_s64_start     = &BGVAR(s64_PCom_Start2);
      p_s64_end         = &BGVAR(s64_PCom_End2);
	  p_s64_period    = &BGVAR(s64_PCom_Peirod2);
	  p_cntrl_word = &BGVAR(Pcom_2_Cntrl_Word);
   }
  
   //Get PFB for the first time
   do
   {
      u16_temp_time = Cntr_3125; // the same RT Interrupt
      u32_pfb_lo = LVAR(AX0_u32_PFB_DF_Lo);
      s32_pfb_hi = LVAR(AX0_s32_PFB_DF_Hi);
   } while (u16_temp_time != Cntr_3125);

   s64_pfb1 =  (long long)s32_pfb_hi;
   s64_pfb1 =  (long long)((s64_pfb1 << 32) & 0xFFFFFFFF00000000);
   s64_pfb1 |= (long long)u32_pfb_lo;

   //Units conversion to MENCRES units to be saved in FPGA.
   s64_FPGA_Value = MultS64ByFixS64ToS64(s64_pfb1,
                                         BGVAR(Unit_Conversion_Table[POSITION_MENCRES_CONVERSION + IS_DUAL_LOOP_ACTIVE]).s64_unit_conversion_to_user_fix,
                                         BGVAR(Unit_Conversion_Table[POSITION_MENCRES_CONVERSION + IS_DUAL_LOOP_ACTIVE]).u16_unit_conversion_to_user_shr);

   //write PFB to FPGA as the "0 point"
   *(long long*)(FPGA_PCOM1_INITIALIZE_POSITION_1_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1)) = s64_FPGA_Value;

   if(p_cntrl_word->u16_type == PCOM_PERIODIC)//periodic mode --> Set init points
   {
      //We use first five table mode registers to save periodic data according to the below:
	  //1st entry - Init start value
	  //2nd entry - Init end value
	  //3rd entry - start value
	  //4th entry - end value
	  //5th entry - N / Gap value   	     
      if(p_cntrl_word->u16_start_cond)//PCOM enabled between PCOMSTART and PCOMEND. PCOMCNTRL bit3 = 1
	  {
	     if(s64_pfb1 <= *p_s64_start) //if PFB < "Start point"
	     {
	        s64_init_start = *p_s64_start;
            s64_init_end   = ((*p_s64_start) + (*p_s64_period));
         }
	     else if(s64_pfb1 >= *p_s64_end) //if PFB > "End point"
	     {
            s64_init_start = ((*p_s64_end) - (*p_s64_period));
            s64_init_end   = *p_s64_end;
	     }
	     else //if "Start point" < PFB < "End point"
	     {
            s64_N = (long long)((s64_pfb1 - *p_s64_start) / *p_s64_period);
		    if(s64_N<0) s64_N = -s64_N; 
            s64_point = ((*p_s64_start) + (s64_N * (*p_s64_period)));

            s64_init_start = s64_point - (*p_s64_period);
            s64_init_end   = s64_point;
	     }      
	  }
	  else//PCOM enabled starting on current psoition -  PCOMCNTRL bit3 = 0
	  {
	     s64_init_start = *(long long*)(FPGA_PCOM1_INITIALIZE_POSITION_1_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1));//init point is current position
		 s64_init_end = (s64_init_start + (*p_s64_period));
	  }

      //write init points in the first and second place in the table (the table is not used in periodic mode)      
      PcomWriteTableEntryToFpga(s64_init_start,0,s16_pcom_num);
      PcomWriteTableEntryToFpga(s64_init_end,1,s16_pcom_num);
   }
   else if(p_cntrl_word->u16_type == PCOM_POSITIONS_TABLE)//table mode
   {
      //write low/high table index to FPGA
      if(*p_u16_table_len == 0)  //0 items
      {
         s16_low_index  = -1;
	     s16_high_index = -1;
      }
      else // 1+ items
      {
         while((p_s64_table[i] < s64_pfb1) && (i < (*p_u16_table_len)))
         {
            i++;
         }
		 if(i == 0)
		 {
		    s16_high_index = 0;
            s16_low_index = -1;
	     }
		 else if (i == *p_u16_table_len)
		 {
		    s16_high_index = *p_u16_table_len;
            s16_low_index = s16_high_index - 1;
		 }
		 else
		 {
            s16_high_index = i;
            s16_low_index = s16_high_index -1;
		 }
      }

      //Set low/high table index value on FPGA
      *((unsigned int*)(FPGA_PCOM1_DSP_SET_TABLE_ADDRESS_LOW_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1)))  = s16_low_index;
      *((unsigned int*)(FPGA_PCOM1_DSP_SET_TABLE_ADDRESS_HIGH_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = s16_high_index;	  	  
   }
   //else// time based
   //{//TBD}
   
   //Get PFB for the second time
   do
   {
      u16_temp_time = Cntr_3125; // the same RT Interrupt
      u32_pfb_lo = LVAR(AX0_u32_PFB_DF_Lo);
      s32_pfb_hi = LVAR(AX0_s32_PFB_DF_Hi);
   } while (u16_temp_time != Cntr_3125);

   s64_pfb2 =  (long long)s32_pfb_hi;
   s64_pfb2 =  (long long)((s64_pfb2 << 32) & 0xFFFFFFFF00000000);
   s64_pfb2 |= (long long)u32_pfb_lo;

   // |s64_pfb1 - s64_pfb2|
   s64_pfb1 -= s64_pfb2;
   if(s64_pfb1 < 0) s64_pfb1 = -s64_pfb1;

   //check if there was a move greater than 0.000075 Rev (MAX VLIM 0f 75rps) 
   if(s64_pfb1 > PCOM_MAX_ALLOWED_MOVEMENT_STANDSTILL)
   {
      return PCOM_MOTOR_MOVE_ON_INIT;
   }

   //if there was a movement that caused the position to get out of the initailzation bounds
   /* Handled on FPGA */
   /*if(p_cntrl_word->u16_type == PCOM_PERIODIC)//table mode
   {
      if(!((s64_pfb2 >= s64_init_start) && (s64_pfb2 <= s64_init_end)))
         return PCOM_MOTOR_MOVE_ON_INIT;
   }
   else if(p_cntrl_word->u16_type == PCOM_POSITIONS_TABLE)//table mode
   {
      if((s16_low_index > 0) && (s16_high_index > 0))
	  {
         if(!((s64_pfb2 >= p_s64_table[s16_low_index]) && (s64_pfb2 <= p_s64_table[s16_high_index])))
		    return PCOM_MOTOR_MOVE_ON_INIT;
	  }
   }
   //else Time based - TBD
   */
   return PCOM_NO_ERROR;
}

int PcomValidateTable(int s16_pcom_num)
{
   unsigned int*  p_u16_table_len = 0;
   long long*     p_s64_table = 0;
   int i = 0;

   //use pointers to handle both PCOM 1 & 2
   if (s16_pcom_num == 1)
   {
      p_u16_table_len = &BGVAR(u16_Pcom_Table_Len1);
	  p_s64_table     = s64_Pcom_Table1_Arr;
   }
   else
   {
      p_u16_table_len = &BGVAR(u16_Pcom_Table_Len2);
	  p_s64_table     = s64_Pcom_Table2_Arr;
   }

   if(((*p_u16_table_len) == 0) || ((*p_u16_table_len) == 1))//if table length is 0 or 1 then table is sorted.
      return 1;

   //go over the table and make sure it is sorted.
   for (i = 0;i < ((*p_u16_table_len) - 1);i++)
   {
      if(p_s64_table[i+1] < p_s64_table[i])
	     break;
   }

   if(i == ((*p_u16_table_len) -1))
      return 1;
   else
      return 0;
}

void PcomDisable(int s16_pcom_num)
{
   unsigned int* p_stat_word;

   if(s16_pcom_num == 1)
   {
	  p_stat_word = &BGVAR(u16_PCom_Stat1);
   }
   else if(s16_pcom_num == 2)
   {
      p_stat_word = &BGVAR(u16_PCom_Stat2);

   }
   else //not supposed to happen
   {
      return;
   }

   //disable in FPGA	        
   *((unsigned int*)(FPGA_PCOM1_EVENT_ENABLE_REG_ADD + ((s16_pcom_num - 1) * FPGA_PCOM2_OFFSET))) = 0;
	                     
   //signal that PCOM feature is disabled
   *p_stat_word &= ~PCOM_STAT_EN_MASK;     
}

void PcomWriteTableEntryToFpga(long long s64_val,int s16_index,int s16_pcom_num)
{
   long long s64_fpga_value = 0LL;
   //Units conversion to MENCRES units to be saved in FPGA.
   s64_fpga_value = MultS64ByFixS64ToS64(s64_val,
                                      BGVAR(Unit_Conversion_Table[POSITION_MENCRES_CONVERSION]).s64_unit_conversion_to_user_fix,
                                      BGVAR(Unit_Conversion_Table[POSITION_MENCRES_CONVERSION]).u16_unit_conversion_to_user_shr);	  
   //Set "Table Value" register
   *((long long*)(FPGA_PCOM1_TABLE_DATA_IN_1_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = s64_fpga_value;
                   
   //Set "Table Address" register
   *((unsigned int*)(FPGA_PCOM1_TABLE_ADDRESS_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = s16_index;
                    
   //Set "Event Write"   register
   *((unsigned int*)(FPGA_PCOM1_TABLE_WRITE_ENABLE_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = 1;
   *((unsigned int*)(FPGA_PCOM1_TABLE_WRITE_ENABLE_REG_ADD + FPGA_PCOM2_OFFSET * (s16_pcom_num-1))) = 0;
}

void UpdatePcomOutputs()
{
   if((u16_Out_Mode[FAST_OUTPUT_1 - 1] == PCOM1_OUT))
      *((unsigned int*)FPGA_DIGITAL_OUTPUT_7_MUX_ADD) = 1; //output 7 is occupied by PCOM1

   if((u16_Out_Mode[FAST_OUTPUT_1 - 1] == PCOM2_OUT))
      *((unsigned int*)FPGA_DIGITAL_OUTPUT_7_MUX_ADD) = 2; //output 7 is occupied by PCOM1

   if((u16_Out_Mode[FAST_OUTPUT_2 - 1] == PCOM1_OUT))
      *((unsigned int*)FPGA_DIGITAL_OUTPUT_8_MUX_ADD) = 1; //output 8 is occupied by PCOM1

   if((u16_Out_Mode[FAST_OUTPUT_2 - 1] == PCOM2_OUT))
      *((unsigned int*)FPGA_DIGITAL_OUTPUT_8_MUX_ADD) = 2; //output 8 is occupied by PCOM1	
}
