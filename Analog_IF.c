#include "DSP2834x_Device.h"
#include "Math.h"

#include "Analog_If.def"
#include "Err_Hndl.def"
#include "FPGA.def"
#include "MultiAxis.def"
#include "Ser_Comm.def"
#include "i2c.def"
#include "design.def"
#include "PtpGenerator.def"

#include "Foldback.var"
#include "Ser_Comm.var"
#include "Extrn_Asm.var"
#include "An_Fltr.var"
#include "Units.var"
#include "Drive.var"
#include "Motor.var"
#include "Init.var"
#include "PtpGenerator.var"
#include "FltCntrl.var"

#include "Prototypes.pro"


void AnalogInputConfig1(int drive)
{   
   float f_sample_time = 0.00003125;

   REFERENCE_TO_DRIVE;
   if (BGVAR(s16_An_Lpf_Hz_1) == 10000)
   {
      WaitForNext32kHzTaskStart();
      VAR(AX0_s16_An_In_1_Alpha) = 0;
      VAR(AX0_s16_An_In_1_Beta)  = 0x7FFF;
      return;
   }

   // Analogue handling will be done on 1msec task in general, if analogue modes are used then it will run on every MTS, so the filter coef should be calculated accordingly
   if (VAR(u16_Analogue_Lo_Sample_Rate)) f_sample_time = 0.001; 

      WaitForNext32kHzTaskStart();

   VAR(AX0_s16_An_In_1_Alpha) = (int)((float)(32768.0 * (float)exp((float)((-6.2831853) * (float)BGVAR(s16_An_Lpf_Hz_1) * f_sample_time))));
   VAR(AX0_s16_An_In_1_Beta) = 0x8000 - VAR(AX0_s16_An_In_1_Alpha);

   if (IS_DDHD_EC2_DRIVE)
   { // Needed because Signals are reversed on Control-PCB; eliminate if Board is edited...
      VAR(AX0_s16_An_In_1_Alpha) *= -1;
      VAR(AX0_s16_An_In_1_Beta) *= -1;
   }
}


void AnalogInputConfig2(int drive)
{
   float f_sample_time = 0.00003125;

   REFERENCE_TO_DRIVE;
   if (BGVAR(s16_An_Lpf_Hz_2) == 10000)
   {
      WaitForNext32kHzTaskStart();

      VAR(AX0_s16_An_In_2_Alpha) = 0;
      VAR(AX0_s16_An_In_2_Beta)  = 0x7FFF;
      return;
   }

   // Analog handling will be done on 1msec task in general, if analog modes are used then it will run on every MTS, so
   if (VAR(u16_Analogue_Lo_Sample_Rate)) f_sample_time = 0.001; // the filter coefficients should be calculated accordingly

      WaitForNext32kHzTaskStart();

   VAR(AX0_s16_An_In_2_Alpha) = (int)((float)(32767.0 * (float)exp((float)((-6.2831853) * (float)BGVAR(s16_An_Lpf_Hz_2) * f_sample_time))));
   VAR(AX0_s16_An_In_2_Beta) = 0x7FFF - VAR(AX0_s16_An_In_2_Alpha);

   if (IS_DDHD_EC2_DRIVE)
   { // Needed because Signals are reversed on Control-PCB; eliminate if Board is edited...
      VAR(AX0_s16_An_In_2_Alpha) *= -1;
      VAR(AX0_s16_An_In_2_Beta) *= -1;
   }
}


int SalAnLpfHz1Command(long long param, int drive)
{
   BGVAR(s16_An_Lpf_Hz_1) = (int)param;
   AnalogInputConfig1(drive);

   return (SAL_SUCCESS);
}


int SalAnLpfHz2Command(long long param, int drive)
{
   BGVAR(s16_An_Lpf_Hz_2) = (int)param;
   AnalogInputConfig2(drive);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalCalcAnZero
// Description:
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalCalcAnZero(int s16_anin, int* p_s16_result)
{
   static unsigned int u16_state = 0, u16_samples_cntr;
   static long s32_anin_avg = 0;

   *p_s16_result = 0;

   switch (u16_state)
   {
      case ANIN_INIT_STATE:   //initilize variables
         s32_anin_avg = (long)s16_anin;
         u16_samples_cntr = 1;
         u16_state = ANIN_AVARAGING_STATE;
      break;

      case ANIN_AVARAGING_STATE:   //samle anin for 128 RT and avarage it
         s32_anin_avg += (long)s16_anin;
         u16_samples_cntr++;

         if (u16_samples_cntr == 128)
            u16_state = ANIN_CONVERTING_STATE;
      break;

      case ANIN_CONVERTING_STATE: //convert the averaged value to mili volts
         s32_anin_avg = (s32_anin_avg + 64 ) >> 7; // divide by 128 + rounding

         if (s32_anin_avg > 32767)          // Saturation for max positive
            s32_anin_avg = 32767;

         if (s32_anin_avg < -32767)         // Saturation for max negative
            s32_anin_avg = -32767;

         *p_s16_result = (int)s32_anin_avg;

         u16_state = ANIN_INIT_STATE;
      break;
   }

   if (u16_state == ANIN_INIT_STATE)
      return SAL_SUCCESS;
   else
      return SAL_NOT_FINISHED;
}


int SalAnzero1(int drive)
{
   int s16_return_value, s16_anin;

   REFERENCE_TO_DRIVE;
   s16_return_value = SalCalcAnZero(VAR(AX0_s16_An_In_1_Anzero), &s16_anin);
   if (s16_return_value != SAL_SUCCESS) return (s16_return_value);

   BGVAR(s16_An_In_1_Offset_User_Setting) = -s16_anin;
   VAR(AX0_s16_An_In_1_Offset) = -s16_anin;

   return SAL_SUCCESS;
}


int SalAnzero2(int drive)
{
   int s16_return_value, s16_anin;

   REFERENCE_TO_DRIVE;
   s16_return_value = SalCalcAnZero(VAR(AX0_s16_An_In_2_Anzero), &s16_anin);
   if (s16_return_value != SAL_SUCCESS) return (s16_return_value);

   BGVAR(s16_An_In_2_Offset_User_Setting) = -s16_anin;
   VAR(AX0_s16_An_In_2_Offset) = -s16_anin;

   return SAL_SUCCESS;
}


void AnalogTorqueConfig1(int drive)
{
   //convert to fix/shift (divide by 2621.44 - internal value of 1V)
   REFERENCE_TO_DRIVE;
   FloatToFix16Shift16(&VAR(AX0_s16_Torque_Scale_Fix_1), &VAR(AX0_u16_Torque_Scale_Shr_1), (float)((float)(BGVAR(s32_Current_Scale_In) / (float)2621.44)));
}


void AnalogTorqueConfig2(int drive)
{
   //convert to fix/shift (divide by 2621.44 - internal value of 1V)
   REFERENCE_TO_DRIVE;
   FloatToFix16Shift16(&VAR(AX0_s16_Torque_Scale_Fix_2), &VAR(AX0_u16_Torque_Scale_Shr_2), (float)((float)(BGVAR(u32_Current_Scale_2) / (float)2621.44)));
}


int SalIScaleInCommand(long long lparam, int drive)
{
   if (lparam > 26214LL) return (VALUE_TOO_HIGH);   //dipeak
   if (lparam < -26214LL) return (VALUE_TOO_LOW);   //dipeak
   if ((lparam > -1LL) && (lparam < 1LL)) return (VALUE_OUT_OF_RANGE); // value not allowed to be close to 0

   BGVAR(s32_Current_Scale_In) = (long)lparam;
   AnalogTorqueConfig1(drive);
   return (SAL_SUCCESS);
}


int SalIScaleCommand2(long long lparam, int drive)
{
   if (lparam > 26214LL) return (VALUE_TOO_HIGH); //dipeak

   if (lparam < 1LL) return (VALUE_TOO_LOW);      // value not allowed to be close to 0

   BGVAR(u32_Current_Scale_2) = (long)lparam;
   AnalogTorqueConfig2(drive);
   return (SAL_SUCCESS);
}


void AnalogVelocityConfig(int drive)
{
   int shr_val;
   long long half_for_rounding = 0LL;
   long s32_vscale_in_user, s32_vscale_in_internal;

   // convert from internal velocity (out of the loop) to user units (out of the loop)
   REFERENCE_TO_DRIVE;
   shr_val = BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr;
   if (shr_val > 0)
      half_for_rounding = 1LL << ((long long)shr_val - 1);

   s32_vscale_in_user = (long)((BGVAR(s32_Velocity_Scale_In) * (long long)BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix + half_for_rounding) >> (long long)shr_val);

   half_for_rounding = 0;

   // convert from user velocity (in the loop) to internal units (in of the loop)
   shr_val = BGVAR(Unit_Conversion_Table[VELOCITY_IN_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr;
   if (shr_val > 0)
      half_for_rounding = 1LL << ((long long)shr_val - 1);

   s32_vscale_in_internal = (long)((s32_vscale_in_user * (long long)BGVAR(Unit_Conversion_Table[VELOCITY_IN_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix + half_for_rounding) >> (long long)shr_val);

   //convert to fix/shift (divide by 2621.44 - internal value of 1V)
   FloatToFix16Shift16(&VAR(AX0_s16_Vel_Scale_Fix), &VAR(AX0_u16_Vel_Scale_Shr), (float)((float)s32_vscale_in_internal / (float)2621.44));
}


int SalVScaleInCommand(long long lparam, int drive)
{
   if (lparam > 2147483112LL) return (VALUE_TOO_HIGH);
   if (lparam < -2147483112LL) return (VALUE_TOO_LOW);

   if ((lparam > -537LL) && (lparam < 537LL)) return (VALUE_OUT_OF_RANGE);
   // value not allowed to be closed to 0

   BGVAR(s32_Velocity_Scale_In) = (long)lparam;
   AnalogVelocityConfig(drive);

   return (SAL_SUCCESS);
}


int SalIScaleOutCommand(long long lparam, int drive)
{
   long s32_Lower_Limit;
   s32_Lower_Limit= (long)(262.14/(float)BGVAR(s32_Drive_I_Peak)); // = 26214*.001/s32_Drive_I_Peak
   if (s32_Lower_Limit == 0L) s32_Lower_Limit = 1L;
   if (lparam > 26214LL) return (VALUE_TOO_HIGH);   //dipeak
   if (lparam < (long long)s32_Lower_Limit) return (VALUE_TOO_LOW);

   BGVAR(s32_Current_Scale_Out) = (long)lparam;
   // calling AnOutMode to recalculate AnOutScale after changing IScale
   return AnOutMode(drive);
}


int SalVScaleOutCommand(long long lparam, int drive)
{
   if (lparam > 2147483112LL) return (VALUE_TOO_HIGH);
   if (lparam < -2147483112LL) return (VALUE_TOO_LOW);
   //if ((lparam > -537LL) && (lparam < 537LL)) return (VALUE_OUT_OF_RANGE);

   BGVAR(s32_Velocity_Scale_Out) = (long)lparam;
   // calling AnOutMode to recalculate AnOutScale after changing VScale
   return AnOutMode(drive);
}


//**********************************************************
// Function Name: SalAnOutModeCommand
// Description:
//          This function is called in response to the ANOUTMODE command.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalAnOutModeCommand(long long lparam, int drive)
{
   if ((!IS_HW_FUNC_ENABLED(ANALOG_OUT_MASK)) && (AN_OUT_NOT_SUPPORTED != lparam)) return NOT_SUPPORTED_ON_HW;

   if ((lparam==7LL) || (lparam==8LL) || (lparam==9LL) || (lparam==10LL))  return (VALUE_OUT_OF_RANGE);

   BGVAR(u8_Analog_Out_Mode) = lparam;

   return AnOutMode(drive);
}


//**********************************************************
// Function Name: AnOutMode
// Description:
//          This function is called to set the Analog Output Source and Scale
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int AnOutMode(int drive)
{
   long s32_temp = 0L;

   REFERENCE_TO_DRIVE;

   // set default scaling of unity
   BGVAR(s16_An_Out_Scale_Fix) = 0x4000;
   BGVAR(u16_An_Out_Scale_Shift) = 14;

   switch (BGVAR(u8_Analog_Out_Mode))
   {
      case USER_COM: // User-Command (ANOUTCMD)
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Analog_Out_Volt_Cmd);
      break;

      case TACHOMETER: // Velocity Feedback
         FloatToFix16Shift16(&BGVAR(s16_An_Out_Scale_Fix), &BGVAR(u16_An_Out_Scale_Shift), ((float)1000 / BGVAR(s32_Velocity_Scale_Out)));
         BGVAR(s32_Analog_Out_Ptr) = &LVAR(AX0_s32_Vel_Var_Fb_0);
      break;

      case I_MONITOR: // Equivalent Current
         FloatToFix16Shift16(&BGVAR(s16_An_Out_Scale_Fix), &BGVAR(u16_An_Out_Scale_Shift), ((float)1000 / BGVAR(s32_Current_Scale_Out)));
         BGVAR(s32_Analog_Out_Ptr) = (long *)&BGVAR(u32_EqCrrnt);
      break;

      case VE_MONITOR: // Velocity Error
         FloatToFix16Shift16(&BGVAR(s16_An_Out_Scale_Fix), &BGVAR(u16_An_Out_Scale_Shift), ((float)1000 / BGVAR(s32_Velocity_Scale_Out)));
         BGVAR(s32_Analog_Out_Ptr) = &LVAR(AX0_s32_Vel_Var_Err);
      break;

      case TC_MONITOR: // Torque Command
         FloatToFix16Shift16(&BGVAR(s16_An_Out_Scale_Fix), &BGVAR(u16_An_Out_Scale_Shift), ((float)1000 / BGVAR(s32_Current_Scale_Out)));
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Current_Cmnd);
      break;

      case TRIANGLE_LOW_FREQ: // Triangular Wave, ~0.041Hz (24 seconds)
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Analog_Out_Triangle_Low_Freq);
      break;

      case IQ_ACT_OUT: // Iq Actual Monitoring
         FloatToFix16Shift16(&BGVAR(s16_An_Out_Scale_Fix), &BGVAR(u16_An_Out_Scale_Shift), ((float)1000 / BGVAR(s32_Current_Scale_Out)));
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Q_Current);
      break;

      case PE_MONITOR: // Position Error
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Analog_Out_Volt_Cmd); // Temporary until TBD
         // TBD
      break;

      case PFB_MONITOR: // Position Feedback
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Analog_Out_Volt_Cmd); // Temporary until TBD
         // TBD
      break;

      case TRIANGLE_10Hz: // Triangular Wave, 10Hz
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Analog_Out_Triangle_10Hz);
      break;

      case SQUARE_10Hz: // Square Wave, 10Hz
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Analog_Out_Square_10Hz);
      break;

      case VELOCITY_COMMAND: // Velocity Feedback
         s32_temp = (long)((float)BGVAR(s32_Velocity_Scale_Out) / (float)BGVAR(s32_V_Lim_Design) * 22750.0); // Translate from out of the loop to in the loop
         FloatToFix16Shift16(&BGVAR(s16_An_Out_Scale_Fix), &BGVAR(u16_An_Out_Scale_Shift), ((float)1000 / (float)s32_temp));
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Vel_Vcmd_For_Anout);
      break;

      case RESERVED_9:
      case RESERVED_10:
      default:
         BGVAR(s32_Analog_Out_Ptr) = &BGVAR(s32_Analog_Out_Volt_Cmd); // Maintain Zero Output
      break;
   }

   if (BGVAR(u8_Analog_Out_Mode) != USER_COM)
   {
      BGVAR(s32_Analog_Out_Volt_Cmd) = 0; // Zero User-Command to avoid surprises
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadAnalogOutCommand
// Description:
//    This function is called in response to the ANOUT command.
//
// Author: A.H.
// Algorithm:
// Revisions:
//**********************************************************
int SalReadAnalogOutCommand(void)
{
   unsigned int u16_temp = 0;

   u16_temp = *(int*)FPGA_DAC_REG_ADD;

   return u16_temp;
}


//**********************************************************
// Function Name: SalAnalogOutCommand
// Description:
//    This function is called in response to the ANOUTCMD command.
//
//
// Author: A.H.
// Algorithm:
// Revisions:
//**********************************************************
int SalAnalogOutCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u8_Analog_Out_Mode) != USER_COM) return INVALID_ANOUT_MODE;

   if (lparam >  (long long)BGVAR(u16_Analog_Out_Volt_Lim))  return VALUE_TOO_HIGH;
   if (lparam < -(long long)BGVAR(u16_Analog_Out_Volt_Lim))  return VALUE_TOO_LOW;

   BGVAR(s32_Analog_Out_Volt_Cmd) = (long)lparam;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalAnalogOutLimCommand
// Description:
//    This function is called in response to the ANOUTLIM command.
//
//
// Author: A.H.
// Algorithm:
// Revisions:
//**********************************************************
int SalAnalogOutLimCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Analog_Out_Volt_Lim) = (int)lparam;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: UpdateLexAnalogOutputScaling
// Description:
//    This function updates the fix-shift values for the analog
//    output scaling of the Lexium.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void UpdateLexAnalogOutputScaling (int drive)
{
   // Min./Max. value check is done in an higher level
   int u16_Max_Output_Voltage = 8000; // 8000[mV] = 8[V]

   int s16_fix = 1;
   unsigned int u16_shift = 0, u16_index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Calculation for analog outputs 1 and 2
   for (u16_index=0; u16_index<2; u16_index++)
   {
      switch ((BGVAR(u16_Analog_Output_Monitor)>>(4*u16_index)) & 0x000F) // Check mode of analog output channel 1 and 2
      {
         case 0: // Actual velocity  = +/- 8[V] = MSPEED --> +/- 8000[mV] / MSPEED[Counts32/125us]
         case 3: // Command velocity = +/- 8[V] = MSPEED --> +/- 8000[mV] / MSPEED[Counts32/125us]
             FloatToFix16Shift16(&s16_fix, &u16_shift, ((float)u16_Max_Output_Voltage / BGVAR(u32_Mspeed)));
         break;

         case 1: // Actual motor torque (derived from IQ).    +/- 8[V] = 3 * MICONT --> +/- 8000[mV] / ((3*MICONT/DIPEAK)*26214) since the current variables used in CalcLexAnalogOutputVaules are in the unit DIPEAK = 26214.
         case 4: // Commanded motor torque (derived from IQ). +/- 8[V] = 3 * MICONT --> +/- 8000[mV] / ((3*MICONT/DIPEAK)*26214) since the current variables used in CalcLexAnalogOutputVaules are in the unit DIPEAK = 26214.
            FloatToFix16Shift16(&s16_fix, &u16_shift, (float)u16_Max_Output_Voltage * (float)BGVAR(s32_Drive_I_Peak) / (3 * (float)BGVAR(s32_Motor_I_Cont) * 26214));
         break;

         case 2: // Pulse train frequency in pps (pulse per second). +/- 8[V] = 4.5[Mpps] --> Since the source-code calculates the delta pulses
                 // every 1[ms] we need to represent 8[V] at 4500[ppms] --> +/- 8000[mV] / 4500[ppms]. The function CalcLexAnalogOutputVaules
                 // furthermore considers additional issues like quadruple analysis for AquadB pulses (the FPGA counts four edges per AquadB pulse) and so on.
            FloatToFix16Shift16(&s16_fix, &u16_shift, (float)u16_Max_Output_Voltage / (float)4500);
         break;

         case 5: // Actual bus voltage = +/- 8000[mV] / 450[V]
            FloatToFix16Shift16(&s16_fix, &u16_shift, (float)u16_Max_Output_Voltage / 450);
         break;

         case 8: // Test signal
         case 9: // Test signal
            FloatToFix16Shift16(&s16_fix, &u16_shift, (float)u16_Max_Output_Voltage / 12000);
         break;

         default:
            // Do nothing, the mode will not be supported by the cyclic background function.
         break;
      }

      if (u16_index == 0) // Calculation of scaling factors for analog output 1
      {
         BGVAR(s16_Analog_Output1_Scale_Fix) = s16_fix;
         BGVAR(u16_Analog_Output1_Scale_Shift) = u16_shift;
      }
      else  // Calculation of scaling factors for analog output 1
      {
         BGVAR(s16_Analog_Output2_Scale_Fix) = s16_fix;
         BGVAR(u16_Analog_Output2_Scale_Shift) = u16_shift;
      }
   }
}


//**********************************************************
// Function Name: CalcLexAnalogOutputVaules
// Description:
//    This function is called continuously from Background
//    and is responsible for calculating the analog output value
//    depending on the analog output mode P0-03 and the additional
//    scaling factors P1-04 and P1-05.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(CalcLexAnalogOutputVaules, "ramfunc_3");
void CalcLexAnalogOutputVaules(int drive)
{
   /*** Here handle the analog output calculations based on parameter P0-03 ***/
   unsigned int u16_index, u16_temp_mode;
   long long s64_temp_result;
   int s16_Output_Scale_Fix, s16_Proportion_Fix;
   unsigned int u16_Output_Scale_Shift, u16_Proportion_Shift;

   REFERENCE_TO_DRIVE;
   /*************************************/
   /*** Handle analog outputs 1 and 2 ***/
   /*************************************/
   for (u16_index=0; u16_index<2; u16_index++)
   {
      if (u16_index == 0) // Output 1 has to be hanlded
      {
         s16_Output_Scale_Fix   = BGVAR(s16_Analog_Output1_Scale_Fix);
         u16_Output_Scale_Shift = BGVAR(u16_Analog_Output1_Scale_Shift);
         s16_Proportion_Fix     = BGVAR(s16_Analog_Output1_Proportion_Fix);
         u16_Proportion_Shift   = BGVAR(u16_Analog_Output1_Proportion_Shift);
      }
      else // Output 2 has to be handled
      {
         s16_Output_Scale_Fix   = BGVAR(s16_Analog_Output2_Scale_Fix);
         u16_Output_Scale_Shift = BGVAR(u16_Analog_Output2_Scale_Shift);
         s16_Proportion_Fix     = BGVAR(s16_Analog_Output2_Proportion_Fix);
         u16_Proportion_Shift   = BGVAR(u16_Analog_Output2_Proportion_Shift);
      }

      // Determine the mode to be handled
      u16_temp_mode = (BGVAR(u16_Analog_Output_Monitor)>>(4*u16_index)) & 0x000F;

      // switch case is not allowed when executing code from RAM
      if (u16_temp_mode == 0)
      {
         // Calculate the actual motor speed in analog output Volts (shift-right is applied later)
         s64_temp_result = (long long)LVAR(AX0_s32_Vel_Var_Fb_0) * (long long)s16_Output_Scale_Fix;
      }
      else if (u16_temp_mode == 1)
      {
         // Calculate the actual motor torque in analog output Volts (shift-right is applied later)
         s64_temp_result = (long long)VAR(AX0_s16_Crrnt_Q_Act_0) * (long long)s16_Output_Scale_Fix;
      }
      else if (u16_temp_mode == 2)
      {
         // Calculate the input pulse-train frequency in analog output Volts (shift-right is applied later)
         if (VAR(AX0_u8_Gear_Mode) <= 2)
         {
            // Use "s32_Encoder_Follower2_Counter_Delta_1ms" loaded by "AX0_s16_Encoder_Follower2_Ptr"
            s64_temp_result = (long long)BGVAR(s32_Encoder_Follower2_Counter_Delta_1ms);
         }
         else
         {
            // Use "s32_Encoder_Follower3_Counter_Delta_1ms" loaded by "AX0_s16_Encoder_Follower3_Ptr"
            s64_temp_result = (long long)BGVAR(s32_Encoder_Follower3_Counter_Delta_1ms);
         }

         // Now multiply the converted value with the output1 scaling fix value.
         s64_temp_result = s64_temp_result * (long long)s16_Output_Scale_Fix;

         // Reduce the value in case that the gearing input interpolation has been activated (see GEARINMODE).
         // Since the input pulse frequency is in reality lower by the factor in the formula below.
         s64_temp_result = s64_temp_result >> (VAR(AX0_u16_Qep_Out_Scale) - VAR(AX0_u16_Qep_In_Scale_Design));

         // If A_quad_B pulses are selected on C2 or C3
         if ((VAR(AX0_u8_Gear_Mode) == 0) || (VAR(AX0_u8_Gear_Mode) == 3))
         {
            // Reduce value by 4 since the FPGA counts edges (4 edges per A_quad_B pulse)
            s64_temp_result = s64_temp_result >> 2;
         }
      }
      else if (u16_temp_mode == 3)
      {
         // Calculate the commanded motor speed in analog output Volts (shift-right is applied later)
         // First convert the the velocity command value from the unit "in the loop" (22750=VLIM)
         // into the unit "out of the loop" in [Counts32/125us].
         s64_temp_result = ((long long)VAR(AX0_s16_Vel_Var_Ref_0) * LVAR(AX0_s32_In_To_Out_Vel_Fix)) >> VAR(AX0_u16_In_To_Out_Vel_Shr);

         // Now multiply the converted value with the output1 scaling fix value.
         s64_temp_result = s64_temp_result * (long long)s16_Output_Scale_Fix;
      }
      else if (u16_temp_mode == 4)
      {
         // Calculate the commanded motor torque in analog output Volts (shift-right is applied later)
         s64_temp_result = (long long)VAR(AX0_s16_Icmd) * (long long)s16_Output_Scale_Fix;
      }
      else if (u16_temp_mode == 5)
      {
         // Calculate the measured bus-voltage in analog output Volts (shift-right is applied later)
         s64_temp_result = (long long)BGVAR(u16_Vbus_Volts) * (long long)s16_Output_Scale_Fix;
      }
      else if (u16_temp_mode == 8)  // test signal triangle 10Hz
      {
         // Calculate the signal in analog output Volts (shift-right is applied later)
         s64_temp_result = (long long)BGVAR(s32_Analog_Out_Triangle_10Hz) * (long long)s16_Output_Scale_Fix;
      }
      else if (u16_temp_mode == 9)  // test signal square 10Hz
      {
         // Calculate the signal in analog output Volts (shift-right is applied later)
         s64_temp_result = (long long)BGVAR(s32_Analog_Out_Square_10Hz) * (long long)s16_Output_Scale_Fix;
      }
      else
      {
         s64_temp_result = 0; // Zero the analog voltage since the mode is not supported
      }

      // Multiply the scaled analog output voltage with P1-04/P1-05 and apply the shift-right here
      // in order to achieve the highest precision.
      s64_temp_result = (s64_temp_result * (long long)s16_Proportion_Fix) >> (u16_Proportion_Shift + u16_Output_Scale_Shift);

      // Limit the 32-bit variable to fit in S16 (see parameter ANOUTLIM)
      if (s64_temp_result > BGVAR(u16_Analog_Out_Volt_Lim))
      {
         s64_temp_result = BGVAR(u16_Analog_Out_Volt_Lim);
      }
      else if (s64_temp_result < -BGVAR(u16_Analog_Out_Volt_Lim))
      {
          s64_temp_result = -BGVAR(u16_Analog_Out_Volt_Lim);
      }

      // Write the result back into the proper variable
      if (u16_index == 0) // Output 1 has to be hanlded
      {
         BGVAR(s16_Scaled_Analog_Output1_Value) = (int)s64_temp_result;
      }
      else // Output 2 has to be hanlded
      {
         BGVAR(s16_Scaled_Analog_Output2_Value) = (int)s64_temp_result;
      }
   }
   /*********************************************/
   /*** End of analog output 1 and 2 handling ***/
   /*********************************************/
}


//**********************************************************
// Function Name: SalWriteAnalogMonitorOutputCommand
// Description:
//    This function sets the P-parameter P0-03 (MON = Analog
//    Monitor Output).
//
// Nibble validity check is now preformed in function 
// IsPParamValueValid as part of "ExecutePParam" function
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteAnalogMonitorOutputCommand(long long lparam, int drive)
{
   BGVAR(u16_Analog_Output_Monitor) = (unsigned int)lparam;

   // Update the fix-shift values depending on the analog output mode
   UpdateLexAnalogOutputScaling(drive);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWriteAnalogMonitorOutputProportion1Command
// Description:
//    This function sets the P-parameter P1-04 (MON1 = Analog
//    Monitor Output Proportion 1).
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteAnalogMonitorOutputProportion1Command(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Min./Max. value check is done in an higher level

   BGVAR(u16_Analog_Output1_Proportion) = (unsigned int)lparam;

   FloatToFix16Shift16(&BGVAR(s16_Analog_Output1_Proportion_Fix), &BGVAR(u16_Analog_Output1_Proportion_Shift), ((float)100 / BGVAR(u16_Analog_Output1_Proportion)));

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWriteAnalogMonitorOutputProportion2Command
// Description:
//    This function sets the P-parameter P1-05 (MON2 = Analog
//    Monitor Output Proportion 2).
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteAnalogMonitorOutputProportion2Command(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Min./Max. value check is done in an higher level

   BGVAR(u16_Analog_Output2_Proportion) = (unsigned int)lparam;

   FloatToFix16Shift16(&BGVAR(s16_Analog_Output2_Proportion_Fix), &BGVAR(u16_Analog_Output2_Proportion_Shift), ((float)100 / BGVAR(u16_Analog_Output2_Proportion)));

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadVCMCommand
// Description:
//    This function gets the P-parameter P1-40 (VCM - Max. Analog Speed Command / Limit).
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadVCMCommand(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   *data = BGVAR(s32_P1_40_VCM);
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalWriteVCMCommand
// Description:
//    This function sets the P-parameter P1-40 (VCM - Max. Analog Speed Command / Limit).
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteVCMCommand(long long param,int drive)
{
   int s16_ret_value;
   long s32_param_after_unit_conversion=0;

   //(param * 100) to convert from speed/10V(Schneider style) to speed/1v(CDHD style) (parser is dividing by 1000)
   s32_param_after_unit_conversion = MultS64ByFixS64ToS64((param*100),
                                     BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                     BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);
   s16_ret_value = SalVScaleInCommand((long long)s32_param_after_unit_conversion,drive);
   if (s16_ret_value == SAL_SUCCESS)
   {// SAVE THE NEW VALUE INTO THE p PARAM
      BGVAR(s32_P1_40_VCM) = (long)(param);
   }
   return s16_ret_value;
}


//**********************************************************
// Function Name: SalWriteTCMCommand
// Description:
//    This function sets the P-parameter P1-41 (TCM - Max. Torque limit 0 - 1000% from MICONT depends if not exceeds MIPEAK).
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteTCMCommand(long long param,int drive)
{
   int s16_ret_value;
   long long s64_param_before_unit_conversion=0, s64_param_after_unit_conversion=0;

   // convert to % from current MICONT
   s64_param_before_unit_conversion = param * BGVAR(s32_Motor_I_Cont);

   // convert into current scale
   s64_param_after_unit_conversion = MultS64ByFixS64ToS64(s64_param_before_unit_conversion,
                                     BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).s64_unit_conversion_to_internal_fix,
                                     BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).u16_unit_conversion_to_internal_shr);

   //Div by 1000 (100 for Precent, 10 because SE are A/10V where CDHD is A/V) , and rounding.
   s64_param_after_unit_conversion = (s64_param_after_unit_conversion + 500) / 1000;

   // update the current scale value in the CDHD way.
   // we using analog 2 as torque analog input
   s16_ret_value = SalIScaleCommand2(s64_param_after_unit_conversion, drive);
   if (s16_ret_value == SAL_SUCCESS)
   {// SAVE THE NEW VALUE INTO THE p PARAM
      BGVAR(u16_P1_41_TCM) = (unsigned int)(param);
   }
   return s16_ret_value;
}


// Allow modifiable wave on the analogue output to validation
// s16_analogue_debug = 1, square wave with amplitude of s32_amplitude[mV] and frequency of s32_frequency[Hz] (0-500Hz)
// s16_analogue_debug = 2, Triangle wave with amplitude of s32_amplitude[mV] and frequency of s32_frequency[Hz] (0-500Hz)
// s16_analogue_debug = 3, Sine wave with amplitude of s32_amplitude[mV] and frequency of s32_frequency[Hz] (0-500Hz)
unsigned int u16_analogue_debug_prev = 0;
long s32_amplitude = 12000L, s32_amplitude_curr = 0L, s32_frequency = 10L, s32_frequency_cntr = 0L;
#define ANALOGUE_FREQUENCY 32000
#pragma CODE_SECTION(WriteAnalogOutputToDACHighSpeed, "ramfunc_3");
void WriteAnalogOutputToDACHighSpeed(void)
{
   if (u16_analogue_debug_prev != u16_Analogue_Debug) // First setting when debug mode changed
   {
      s32_amplitude_curr = 0L;
      s32_frequency_cntr = 0L;
      u16_analogue_debug_prev = u16_Analogue_Debug;   
   }

   s32_frequency_cntr++;
   if (u16_Analogue_Debug == 1) // Square wave
   {
      s32_amplitude_curr = s32_amplitude;
      if (s32_frequency_cntr >= (ANALOGUE_FREQUENCY/2/s32_frequency))
      {
         s32_amplitude_curr = -s32_amplitude_curr;
         if (s32_frequency_cntr >= (ANALOGUE_FREQUENCY/s32_frequency))
            s32_frequency_cntr = 0L;            
      }
   }
   else if (u16_Analogue_Debug == 2) // Triangular wave
   {
      s32_amplitude_curr += 2*s32_amplitude*s32_frequency/ANALOGUE_FREQUENCY;
      if (s32_amplitude_curr > s32_amplitude)
         s32_amplitude_curr = -s32_amplitude; 
         
   }
   else if (u16_Analogue_Debug == 3) // Sine wave
   {
      if (s32_frequency_cntr == (ANALOGUE_FREQUENCY/s32_frequency)) // To avoid overflow on the sin() argument
         s32_frequency_cntr = 0L;

      s32_amplitude_curr = s32_amplitude*sin((float)s32_frequency_cntr/((float)ANALOGUE_FREQUENCY/(float)s32_frequency)*6.28318);
   }

   s32_Analog_Out_Square_10Hz = s32_amplitude_curr;     
}


#pragma CODE_SECTION(WriteAnalogOutputToDAC, "ramfunc_3");
void WriteAnalogOutputToDAC(int drive)
{
   int s16_dac_value;
   long long s64_temp_value;

   REFERENCE_TO_DRIVE;
   // Triangle Wave for Verification ~0.041Hz (24 seconds)
   BGVAR(s32_Analog_Out_Triangle_Low_Freq) += 1;
   if (BGVAR(s32_Analog_Out_Triangle_Low_Freq) > 12000)
      BGVAR(s32_Analog_Out_Triangle_Low_Freq) = -12000;

   // Triangle Wave for Verification 10Hz
   BGVAR(s32_Analog_Out_Triangle_10Hz) += 240;
   if (BGVAR(s32_Analog_Out_Triangle_10Hz) > 12000)
      BGVAR(s32_Analog_Out_Triangle_10Hz) = -12000;

   // Square Wave for Verification 10Hz
   if (!u16_Analogue_Debug)
   {
      if (BGVAR(s32_Analog_Out_Triangle_10Hz) >= 0)
         BGVAR(s32_Analog_Out_Square_10Hz) = 12000;
      else
         BGVAR(s32_Analog_Out_Square_10Hz) = -12000;
   }   

   // Read current command (in case selected by ANOUTMODE).
   BGVAR(s32_Current_Cmnd) = (long)(*(int *)(LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr)));

   // Type cast Iq (in case selected by ANOUTMODE).
   BGVAR(s32_Q_Current) = (long)VAR(AX0_s16_Crrnt_Q_Act_0);

   // For analog output translate 16bit vcmd to 32bit
   BGVAR(s32_Vel_Vcmd_For_Anout) = (long)VAR(AX0_s16_Vel_Var_Ref_0);

   // read the value via pointer, and scale it to volts
   s64_temp_value = ((*(long *)BGVAR(s32_Analog_Out_Ptr)) * (long long)BGVAR(s16_An_Out_Scale_Fix)) >> BGVAR(u16_An_Out_Scale_Shift);

   // Impose Limits:
   if (s64_temp_value > (long long)BGVAR(u16_Analog_Out_Volt_Lim))
      BGVAR(s16_Analog_Out_Volt) = BGVAR(u16_Analog_Out_Volt_Lim);
   else if (s64_temp_value < -(long long)BGVAR(u16_Analog_Out_Volt_Lim))
      BGVAR(s16_Analog_Out_Volt) = -BGVAR(u16_Analog_Out_Volt_Lim);
   else
      BGVAR(s16_Analog_Out_Volt) = (int)s64_temp_value;

   // Conversion of selected Value to DAC Command, 0x8000 for Zero.
   if (IS_CAN2_DRIVE || IS_EC2_DRIVE || IS_DDHD_EC2_DRIVE)
   {
      // Hardware: full_scale = (3.3 - 1.65)volt * 100/(10.5+3.09) = 1.65 * 7.358 = 12.141 volts = 12141 mv
      // Conversion: value[mv]/12141*32768 (the value is limited to 12000, so no overflow is expected)
      //             value[mv] * 0x565e >> 13

      *(int*)FPGA_SIGMA_DELTA_DAC1_REG_ADD = (int)(((long)BGVAR(s16_Analog_Out_Volt) * 0x565e) >> 13) + 0x8000;
      *(int*)FPGA_SIGMA_DELTA_DAC2_REG_ADD = (int)(((long)BGVAR(s16_Analog_Out2_Volt) * 0x565e) >> 13) + 0x8000;
   }
   else if (u16_Product == DDHD)
   {
      // Device: DAC5311 - 8 bit serial interface
      // Hardware: full_scale = (5 - 2.5)volt * 49.9/10 = 2.5 * 4.99 = 12.475 volts = 12475 mv
      // Conversion: value[mv]/12475*32768 (the value is limited to 12000, so no overflow is expected)
      //             value[mv] * 0x540e >> 13

      s16_dac_value = (int)(((long)BGVAR(s16_Analog_Out_Volt) * 0x540e) >> 13) + 0x8000;
      s16_dac_value = (s16_dac_value >> 2) & 0x3fff;                        // The two MSB are PD1,PD0 (power down mode) that should be 0 for normal operation
      *(int*)FPGA_DAC_REG_ADD = (s16_dac_value >> 8) & 0x00ff;              // write the 8 MSB: PD1,PD0, and 6 MSB of the DAC
      *(int*)FPGA_DAC_REG_4_LSB_REG_ADD = (s16_dac_value >> 4) & 0x000f;    // write the next 4 bits: 2 LSB of the DAC and 2 more bits to support
                                                                            // 10bit DAC (don't care for 8 bit DAC)
   }
   else
   {
      // Hardware: full_scale = (5 - 2.5)volt * 49.9/10 = 2.5 * 4.99 = 12.475 volts = 12475 mv
      // Conversion: value[mv]/12475*32768 (the value is limited to 12000, so no overflow is expected)
      //             value[mv] * 0x540e >> 13

      s16_dac_value = (int)(((long)BGVAR(s16_Analog_Out_Volt) * 0x540e) >> 13) + 0x8000;
      *(int*)FPGA_DAC_REG_ADD = (s16_dac_value >> 8) & 0x00ff;              // write the 8 MSB
      *(int*)FPGA_DAC_REG_4_LSB_REG_ADD = (s16_dac_value >> 4) & 0x000f;    // write the next 4 bits to support 12bit DAC
   }
}


//**********************************************************
// Function Name: GetAnalogInput2ValueCorrected
// Description:
//    This function returns the measured voltage on the analog input 2. In case
//    that the voltage correction feature is enabled, it is possible that the measured
//    voltage is corrected by the Frencken algorithm inside of this function. Please note also the
//    Servotronix documentation regarding the voltage correction feature (when using dual-loop).
//    The corrected voltage of the analog input 2 is just for reading.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int GetAnalogInput2ValueCorrected (int drive)
{
   int s16_temp_analog_voltage_2 = VAR(AX0_s16_An_In_2_Filtered);
   unsigned char u8_sector_found = 0;
   char s8_sector_index = 0;
   long long s64_temp_result;

   REFERENCE_TO_DRIVE;
   if (BGVAR(u16_Voltage_Correct_2_Enable) != 0) // If voltage correction feature is active
   {
      do { // Find the right sector within the voltage correction table
         // Increasing values inside of the table
         if (BGVAR(u16_Voltage_Correct_2_Enable) == 1)
         {
            if (s16_temp_analog_voltage_2 < BGVAR(s16_Voltage_Correct_2_Array)[s8_sector_index])
            {
               u8_sector_found = 1;
            }
         }
         else // Decreasing values inside of the table
         {
            if (s16_temp_analog_voltage_2 > BGVAR(s16_Voltage_Correct_2_Array)[s8_sector_index])
            {
               u8_sector_found = 1;
            }
         }
         if (u8_sector_found == 0)         // If we did not found the sector yet
         {
            s8_sector_index++;
         }
      } while((u8_sector_found == 0) && (s8_sector_index <= BGVAR(u16_Voltage_Correct_2_Number_Of_Sections)));

      s8_sector_index--; // Decrement by one since we need to get the correct index for the Fix/Shift values

      BGVAR(s16_Sector_2_Identified) = s8_sector_index;

      // If we are within the voltage correction table (a value of -1 or BGVAR(u16_Voltage_Correct_2_Number_Of_Sections) states that we are outside of the table)
      if ((s8_sector_index >= 0) && (s8_sector_index < BGVAR(u16_Voltage_Correct_2_Number_Of_Sections)))
      {
         // *********************************************************************************************************************
         // At this place the index variable is between "0 ... u16_Voltage_Correct_2_Number_Of_Sections-1",
         // which means we need to apply the voltage correction feature.
         // Apply the following formula:
         //    U_corrected = ((U_measured - U_low) * Fix_x) >> Shift_x + x * U_sector_ideal + U_neg
         //       with: x       = sector we are in between 0...u16_Voltage_Correct_Number_Of_Sections-1
         //             Fix_x   = Fix-value of Gain_x
         //             Shift_x = Shift-value of Gain_x
         //             U_low   = Voltage value out of the table
         //             U_neg   = Lowest voltage measured during calibration procedure stored in BGVAR(s16_Voltage_Correct_Array)[0]
         //
         // Please refer to the documentation about the voltage correction feature on the STX Server.
         // *********************************************************************************************************************
         s64_temp_result = (long long)(s16_temp_analog_voltage_2 - BGVAR(s16_Voltage_Correct_2_Array)[s8_sector_index]) * (long long)BGVAR(s16_Voltage_Correct_2_Fix_Array)[s8_sector_index];
         s64_temp_result = s64_temp_result >> BGVAR(u16_Voltage_Correct_2_Shift_Array)[s8_sector_index];
         s64_temp_result += (long long)s16_Voltage_Correct_2_Ideal * (long long)s8_sector_index;
         s64_temp_result += BGVAR(s16_Voltage_Correct_2_Array)[0];
         // If the result is in the range of 16-bits
         if ((s64_temp_result <= 32767) && (s64_temp_result >= -32768))
         {
            // Now put the result in the variable
            s16_temp_analog_voltage_2 = (int)s64_temp_result;
         }
      }
   }

   return (s16_temp_analog_voltage_2);
}


void DesignAnin1MsqFilter(int drive)
{
// /* Set the input scale to 3 (2^3=8) to increase resolution.
// *  For maximum of 60,000rpm, with an 8-Pole Motor, there are 240,000 Commutation Cycle per Minute.
// *  At full Sampling Rate, the expected Counting Rate is 240,000 * 65,536 / 60 / 32,000 = 8,192 / Tsp
// *  Set Input Scaling of 4 to avoid exceeding re 3,000*1.25 = 3,750 counts/Tsp.
// *  Input scale of 8 gives 3,750*8 = 30,000 bits/Tsp.
// *
// *  AX0_u16_In_Scale  - determines how many bits to shift left the gear input
// *  AX0_u16_Out_Scale - determines how many bits to shift right the gear input after multiplied by GEARIN
// */
   REFERENCE_TO_DRIVE;
   VAR(AX0_u16_Anin1_In_Scale_Design) = 0;
   VAR(AX0_u16_Anin1_Out_Scale) = 0;

// t1 - depth of the filter, expressed in sample time of 31.25us

   VAR(AX0_u16_Anin1_Msq_Fltr_T1) = (unsigned int)((float)BGVAR(u32_Anin1_Msq_Filt_User_T1) / (float)31.25);
   VAR(AX0_u16_Anin1_Msq_Fltr_T2) = (unsigned int)((float)BGVAR(u16_Anin1_Msq_Filt_User_T2) / (float)31.25);

// // Vff = (t1 - user_Vff)/3.125 (in RT it will be divided by 10 to get Vff in units of sample time).

   if (BGVAR(s32_Anin1_Msq_Filt_User_Vff) > BGVAR(u32_Anin1_Msq_Filt_User_T1)) 
      VAR(AX0_s16_Anin1_Msq_Vff_N_Design) = 0;
   else
      VAR(AX0_s16_Anin1_Msq_Vff_N_Design) = (int)((BGVAR(u32_Anin1_Msq_Filt_User_T1) - BGVAR(s32_Anin1_Msq_Filt_User_Vff) ) / 3.125);

// Beta1 = f1 = 1/t1.  Use 15 shifts, so Beta1 = 32768 / t1.
// Alpha1 = "1" - Beta1 = 32768 - Beta1

   VAR(AX0_s16_Anin1_Msq_Fltr_Beta1_Design)  = (int)(((long)32768 + ((long)VAR(AX0_u16_Anin1_Msq_Fltr_T1) >> 1)) / VAR(AX0_u16_Anin1_Msq_Fltr_T1));
   VAR(AX0_s16_Anin1_Msq_Fltr_Alpha1_Design) = (int)((long)32768 - (long)VAR(AX0_s16_Anin1_Msq_Fltr_Beta1_Design));

// Calculate the correction for Sxp in case of rollover in the input.
// Correction = (t1-1)*32768, but since we need to take t1 as back calculated fro Beta1, we get that
// Correction = Alpha1 / Beta1 * 2^23

   LVAR(AX0_s32_Anin1_Msq_Fltr_Rollover_Sxp_Fix_Design) = (long)((((long long)0x00800000 * VAR(AX0_s16_Anin1_Msq_Fltr_Alpha1_Design)) + ((long long)VAR(AX0_s16_Anin1_Msq_Fltr_Beta1_Design) >> 1)) / VAR(AX0_s16_Anin1_Msq_Fltr_Beta1_Design));

// S1 = t1 - 1

   VAR(AX0_s16_Anin1_Msq_Fltr_S1) = VAR(AX0_u16_Anin1_Msq_Fltr_T1) - 1;

// Inv_Delta = 1/(t1*(t1-1)) = Beta1^2 / Alpha1

   FloatToFix16Shift16(&VAR(AX0_s16_Anin1_Msq_Fltr_Inv_Delta_Fix_Design),
                       &VAR(AX0_u16_Anin1_Msq_Fltr_Inv_Delta_Shr_Design),
                       (float)((float)VAR(AX0_s16_Anin1_Msq_Fltr_Beta1_Design) * (float)VAR(AX0_s16_Anin1_Msq_Fltr_Beta1_Design) / (float)VAR(AX0_s16_Anin1_Msq_Fltr_Alpha1_Design)) );

   VAR(AX0_u16_Anin1_Msq_Fltr_Inv_Delta_Shr_Design) -= 1;  // add 15 because Beta in the above calc has scale of 2^15, and sub 16 to increase the resolution of V[n]

// Beta2 = f2 = 1/t2.  Use 15 shifts, so Beta1 = 32768 / t2.
// Alpha2 = "1" - Beta2 = 32768 - Beta2

   if (VAR(AX0_u16_Anin1_Msq_Fltr_T2) > 0)
   {
      VAR(AX0_s16_Anin1_Msq_Fltr_Beta2_Design)  = (int)((long)32768 / VAR(AX0_u16_Anin1_Msq_Fltr_T2));
      VAR(AX0_s16_Anin1_Msq_Fltr_Alpha2_Design) = (int)((long)32768 - (long)VAR(AX0_s16_Anin1_Msq_Fltr_Beta2_Design));
   }
   else
   {
      VAR(AX0_s16_Anin1_Msq_Fltr_Beta2_Design) = 0x8000;//32768;
      VAR(AX0_s16_Anin1_Msq_Fltr_Alpha2_Design) = 0;
   }

// A_Factor = f2/(1-f2) = Beta2 / Alpha2

   FloatToFix16Shift16(&VAR(AX0_s16_Anin1_Msq_Fltr_A_Factor_Fix_Design),
                       &VAR(AX0_u16_Anin1_Msq_Fltr_A_Factor_Shr_Design),
                       (float)((float)VAR(AX0_s16_Anin1_Msq_Fltr_Beta2_Design) / (float)VAR(AX0_s16_Anin1_Msq_Fltr_Alpha2_Design)) );

// Aff_Gain = (t1-1)^2 * User_Aff/1000 = (Alpha1/Beta1)^2 * User_Aff/1000

   FloatToFix16Shift16(&VAR(AX0_s16_Anin1_Msq_Fltr_Aff_Gain_Fix),
                       &VAR(AX0_u16_Anin1_Msq_Fltr_Aff_Gain_Shr),
                       (float)((float)VAR(AX0_s16_Anin1_Msq_Fltr_Alpha1_Design) * (float)VAR(AX0_s16_Anin1_Msq_Fltr_Alpha1_Design) * (float)BGVAR(s16_Anin1_Msq_Filt_User_Aff)
                              / (float)VAR(AX0_s16_Anin1_Msq_Fltr_Beta1_Design) / (float)VAR(AX0_s16_Anin1_Msq_Fltr_Beta1_Design) / 1000.0) );

// Calculate the units conversion factor of velocity and acceleration from filter units to internal units
// factor = 2^(16 - _AX0_u16_Anin1_In_Scale) / (XENCRES*?4?) * GearIn / GearOut
// Consider gear mode for whether XENCRES should be multiplied by 4 or not

   FloatToFixS32Shift16(&LVAR(AX0_s32_Anin1_Msq_Fltr_Units_Conv_Fix_Design),
                        &VAR(AX0_u16_Anin1_Msq_Fltr_Units_Conv_Shr_Design),
                        (float)1 );

   VAR(AX0_u16_Anin1_Msq_Fltr_Units_Conv_Shr_Design) -= (16 - VAR(AX0_u16_Anin1_Out_Scale));

   VAR(AX0_u16_Gear_Skip_Flags) |= COPY_ANIN1_FILT_MASK;
}


int SalAnin1FiltModeCommand(long long param, int drive)
{
   VAR(AX0_u16_Anin1_Msq_Fltr_Mode) = (int)param;

   DesignAnin1MsqFilter(drive);

   return (SAL_SUCCESS);
}


int SalAnin1FiltT1Command(long long param, int drive)
{
   unsigned long u32_temp1;
   unsigned int u16_temp2;

   u32_temp1 = (unsigned long)param;
   u16_temp2 = (unsigned int)(u32_temp1 / 125);
   if ( ((unsigned long)u16_temp2 * 125) == u32_temp1 ) // check if user value is muliple of 125 (0.125 * 1000)
   {
      BGVAR(u32_Anin1_Msq_Filt_User_T1) = u32_temp1;
      DesignAnin1MsqFilter(drive);
      return (SAL_SUCCESS);
   }
   else
      return (ARGUMENT_NOT_125);
}


int SalAnin1FiltT2Command(long long param, int drive)
{
   unsigned int u16_temp1, u16_temp2;

   u16_temp1 = (unsigned int)param;
   u16_temp2 = (unsigned int)(u16_temp1 / 125);
   if ( (u16_temp2 * 125) == u16_temp1 ) // check if user value is muliple of 125 (0.125 * 1000)
   {
      BGVAR(u16_Anin1_Msq_Filt_User_T2) = u16_temp1;
      DesignAnin1MsqFilter(drive);
      return (SAL_SUCCESS);
   }
   else
      return (ARGUMENT_NOT_125);
}


int SalAnin1FiltAffCommand(long long param, int drive)
{
   BGVAR(s16_Anin1_Msq_Filt_User_Aff) = (int)param;
   DesignAnin1MsqFilter(drive);
   return (SAL_SUCCESS);
}


int SalAnin1FiltVffCommand(long long param, int drive)
{
   BGVAR(s32_Anin1_Msq_Filt_User_Vff) = (long)param;
   DesignAnin1MsqFilter(drive);
   return (SAL_SUCCESS);
}

