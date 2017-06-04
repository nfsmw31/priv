#include "MultiAxis.def"
#include "Err_Hndl.def"
#include "PtpGenerator.def"
#include "Design.def"
#include "Velocity.def"

#include "Drive.var"
#include "Extrn_Asm.var"
#include "Motor.var"
#include "PtpGenerator.var"
#include "units.var"

#include "Prototypes.pro"


void DesignGearMsqFilter(int drive)
{
   int s16_temp_value,s16_temp;
   // AXIS_OFF;

/*
*  Set the input scale to 3 (2^3=8) to increase resolution.
*  For max QEP input freq of 12,000,000 counts/sec, at sample rate of 4 kHz, there are 3,000 counts/Tsp.
*  Allow margin of 25% for overshoot, there are 3,000*1.25 = 3,750 counts/Tsp.
*  Input scale of 8 gives 3,750*8 = 30,000 bits/Tsp.
*
*  AX0_u16_Qep_In_Scale  - determines how many bits to shift left the gear input
*  AX0_u16_Qep_Out_Scale - determines how many bits to shift right the gear input after multiplied by GEARIN
*  Their values are set according to the following table:
*
*  GEARMODE  GEARINMODE  GEARFILTMODE  AX0_u16_Qep_In_Scale  AX0_u16_Qep_Out_Scale  Comment
*  --------  ----------  ------------  --------------------  ---------------------  ------------------------------------------------
*     0-2         0            0                  0                   0
*     0-2         0            1                  3                   3              MSQ filter increases the gear resolution by (*8)
*     0-2         1            0                  0                   4              GEARINMODE implements 1/T (*16) on GEARMODE 0-2 by the FPGA
*     0-2         1            1                  0                   4              GEARINMODE implements 1/T (*16) on GEARMODE 0-2 by the FPGA
*     3-4         0            0                  0                   0
*     3-4         0            1                  3                   3              MSQ filter increases the gear resolution by (*8)
*     3-4         1            0                  0                   0
*     3-4         1            1                  3                   3              MSQ filter increases the gear resolution by (*8)
*/
   s16_temp_value = VAR(AX0_u16_Qep_Msq_Fltr_Mode);
   s16_temp_value |= (BGVAR(u16_Gear_In_Mode) << 1);
   if ((VAR(AX0_u8_Gear_Mode) == 3) || (VAR(AX0_u8_Gear_Mode) == 4))
   {
      s16_temp_value |= 4;
   }

   switch (s16_temp_value)
   {
      case 0:
      case 4:
      case 6:
      default:
         VAR(AX0_u16_Qep_In_Scale_Design) = 0;
         VAR(AX0_u16_Qep_Out_Scale_Design) = 0;
         VAR(AX0_s16_Gear_Dead_Band_Design) = 1;
      break;

      case 1:
      case 5:
      case 7:
         VAR(AX0_u16_Qep_In_Scale_Design) = 3;
         VAR(AX0_u16_Qep_Out_Scale_Design) = 3;
         VAR(AX0_s16_Gear_Dead_Band_Design) = 9;       // equivalent to 9/8 original gear pulses
      break;

      case 2:
      case 3:
         VAR(AX0_u16_Qep_In_Scale_Design) = 0;
         VAR(AX0_u16_Qep_Out_Scale_Design) = 4;
         VAR(AX0_s16_Gear_Dead_Band_Design) = 18;      // equivalent to 18/16 original gear pulses
      break;
   }

/*
*  t1 - depth of the filter, expressed in sample time of 250 us
*/
   s16_temp = (int)((float)VAR(AX0_u16_Pos_Loop_Tsp)*0.01);

   VAR(AX0_u16_Qep_Msq_Fltr_T1) = (unsigned int)(BGVAR(u32_Gear_Filt_User_T1) / s16_temp);
   VAR(AX0_u16_Qep_Msq_Fltr_T2) = BGVAR(u16_Gear_Filt_User_T2) / s16_temp;

/*
*  Vff = (t1 - user_Vff)/12.5 (in RT it will be divided by 10 to get Vff in units of sample time).
*/
   if (BGVAR(s32_Gear_Filt_User_Vff) > BGVAR(u32_Gear_Filt_User_T1))
      VAR(AX0_s16_Qep_Msq_Vff_N_Design) = 0;
   else
      VAR(AX0_s16_Qep_Msq_Vff_N_Design) = (int)(((float)(((long)BGVAR(u32_Gear_Filt_User_T1)) - BGVAR(s32_Gear_Filt_User_Vff) )) / 12.5);

/*
*  Beta1 = f1 = 1/t1.  Use 15 shifts, so Beta1 = 32768 / t1.
*  Alpha1 = "1" - Beta1 = 32768 - Beta1
*/
   VAR(AX0_s16_Qep_Msq_Fltr_Beta1_Design)  = (int)(((long)32768 + ((long)VAR(AX0_u16_Qep_Msq_Fltr_T1) >> 1)) / VAR(AX0_u16_Qep_Msq_Fltr_T1));
   VAR(AX0_s16_Qep_Msq_Fltr_Alpha1_Design) = (int)((long)32768 - (long)VAR(AX0_s16_Qep_Msq_Fltr_Beta1_Design));

/*
*  Calculate the correction for Sxp in case of rollover in the input.
*  Correction = (t1-1)*32768, but since we need to take t1 as back calculated fro Beta1, we get that
*  Correction = Alpha1 / Beta1 * 2^23
*/
   LVAR(AX0_s32_Qep_Msq_Fltr_Rollover_Sxp_Fix_Design) = (long)((((long long)0x00800000 * VAR(AX0_s16_Qep_Msq_Fltr_Alpha1_Design)) + ((long long)VAR(AX0_s16_Qep_Msq_Fltr_Beta1_Design) >> 1)) / VAR(AX0_s16_Qep_Msq_Fltr_Beta1_Design));

/*
*  S1 = t1 - 1
*/
   VAR(AX0_s16_Qep_Msq_Fltr_S1) = VAR(AX0_u16_Qep_Msq_Fltr_T1) - 1;

/*
*  Inv_Delta = 1/(t1*(t1-1)) = Beta1^2 / Alpha1
*/
   FloatToFix16Shift16(&VAR(AX0_s16_Qep_Msq_Fltr_Inv_Delta_Fix_Design),
                       &VAR(AX0_u16_Qep_Msq_Fltr_Inv_Delta_Shr_Design),
                       (float)((float)VAR(AX0_s16_Qep_Msq_Fltr_Beta1_Design) * (float)VAR(AX0_s16_Qep_Msq_Fltr_Beta1_Design) / (float)VAR(AX0_s16_Qep_Msq_Fltr_Alpha1_Design)) );

   VAR(AX0_u16_Qep_Msq_Fltr_Inv_Delta_Shr_Design) -= 1;  // add 15 because Beta in the above calc has scale of 2^15, and sub 16 to increase the resolution of V[n]

/*
*  Beta2 = f2 = 1/t2.  Use 15 shifts, so Beta1 = 32768 / t2.
*  Alpha2 = "1" - Beta2 = 32768 - Beta2
*/
   if (VAR(AX0_u16_Qep_Msq_Fltr_T2) > 0)
   {
      VAR(AX0_s16_Qep_Msq_Fltr_Beta2_Design)  = (int)((long)32768 / VAR(AX0_u16_Qep_Msq_Fltr_T2));
      VAR(AX0_s16_Qep_Msq_Fltr_Alpha2_Design) = (int)((long)32768 - (long)VAR(AX0_s16_Qep_Msq_Fltr_Beta2_Design));
   }
   else
   {
      VAR(AX0_s16_Qep_Msq_Fltr_Beta2_Design) = 0x8000;//32768;
      VAR(AX0_s16_Qep_Msq_Fltr_Alpha2_Design) = 0;
   }

/*
*  A_Factor = f2/(1-f2) = Beta2 / Alpha2
*/
   FloatToFix16Shift16(&VAR(AX0_s16_Qep_Msq_Fltr_A_Factor_Fix_Design),
                       &VAR(AX0_u16_Qep_Msq_Fltr_A_Factor_Shr_Design),
                       (float)((float)VAR(AX0_s16_Qep_Msq_Fltr_Beta2_Design) / (float)VAR(AX0_s16_Qep_Msq_Fltr_Alpha2_Design)) );

/*
*  Aff_Gain = (t1-1)^2 * User_Aff/1000 = (Alpha1/Beta1)^2 * User_Aff/1000
*/
   FloatToFix16Shift16(&VAR(AX0_s16_Qep_Msq_Fltr_Aff_Gain_Fix),
                       &VAR(AX0_u16_Qep_Msq_Fltr_Aff_Gain_Shr),
                       (float)((float)VAR(AX0_s16_Qep_Msq_Fltr_Alpha1_Design) * (float)VAR(AX0_s16_Qep_Msq_Fltr_Alpha1_Design) * (float)BGVAR(s16_Gear_Filt_User_Aff)
                              / (float)VAR(AX0_s16_Qep_Msq_Fltr_Beta1_Design) / (float)VAR(AX0_s16_Qep_Msq_Fltr_Beta1_Design) / 1000.0) );

/*
*  Calculate the units conversion factor of velocity and acceleration from filter units to internal units
*  factor = 2^(16 - _AX0_u16_Qep_In_Scale) / (XENCRES*?4?) * GearIn / GearOut
*  Consider gear mode for whether XENCRES should be multiplied by 4 or not
*/

   FloatToFixS32Shift16(&LVAR(AX0_s32_Qep_Msq_Fltr_Units_Conv_Fix_Design),
                        &VAR(AX0_u16_Qep_Msq_Fltr_Units_Conv_Shr_Design),
                        (float)((float)LVAR(AX0_s32_Gearin_Design) / (float)LVAR(AX0_u32_Gearout_Design) / (float)BGVAR(u32_External_Enc_Res) ) );

   if ((VAR(AX0_u8_Gear_Mode) == 0) || (VAR(AX0_u8_Gear_Mode) == 3))   // If the Input is Quadrature
        VAR(AX0_u16_Qep_Msq_Fltr_Units_Conv_Shr_Design) += 2;                 // Add 2 shr to account for 4 Counts per each Encoder Line.

   VAR(AX0_u16_Qep_Msq_Fltr_Units_Conv_Shr_Design) -= (16 - VAR(AX0_u16_Qep_Out_Scale_Design));

   LVAR(AX0_s32_Qep_Msq_Fltr_Acc_Out_Tresh_Design) = (long)(BGVAR(u64_Gear_Acc_Tresh)>>32LL);

   // calc factor for filter update on enable 1/(2^15-Alpha1)
   FloatToFix16Shift16(&VAR(AX0_s16_Qep_Msq_Fltr_Alpha1_Factor_Design),&VAR(AX0_s16_Qep_Msq_Fltr_Alpha1_Factor_Shr_Design),
                        1.0/(float)(32768 - VAR(AX0_s16_Qep_Msq_Fltr_Alpha1_Design)) );

   // in implementation there are 16 shr
   VAR(AX0_s16_Qep_Msq_Fltr_Alpha1_Factor_Shr_Design) -=16;

   // This will set internal XENCRES value
   SetXEncResInternalValue(drive);

   // This must be after XENCRES was updated!
   // This will update the P&D Line Brake filter in the FPGA
   UpdatePulseandDirectionLineBrakeFilterInFpga();
 
   VAR(AX0_u16_Gear_Skip_Flags) |= COPY_GEAR_FILT_MASK;
}

//  Set internal XENCRES value according to GEARMODE & GEARFILTMODE
int SetXEncResInternalValue(int drive)
{
    // AXIS_OFF;
    long s32_temp;
    REFERENCE_TO_DRIVE;

    //  Called after changing the XENCRES parameter
    s32_temp = BGVAR(u32_External_Enc_Res);

    if ((VAR(AX0_u8_Gear_Mode) == 0) || (VAR(AX0_u8_Gear_Mode) == 3))
    {
        //  CDHD refers to the number of AquadB pulses.
        //  Multiply the variable "AX0_u32_Xencres" by 4 due to the fact that the
        //  Drive counts edges and not pulses.
        s32_temp = s32_temp << 2;
    }

    //  Compensate for the increase of input resolution
    s32_temp = s32_temp << VAR(AX0_u16_Qep_Out_Scale_Design);

    if (LVAR(AX0_u32_Xencres_Design) != s32_temp)
    {
        LVAR(AX0_u32_Xencres_Design) = s32_temp;
    }

    return SAL_SUCCESS;
}

int SalGearInModeCommand(long long param,int drive)
{
   BGVAR(u16_Gear_In_Mode) = (int)param;
   DesignGearMsqFilter(drive);
   SetupGear(drive);

   // for LX28/26 recalculate gearing filter.
   if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
   {
      UpdatePulseDirectionFiltersFromPTTParameter(drive);
   }

   return (SAL_SUCCESS);
}

int SalGearFiltModeCommand(long long param,int drive)
{
   // AXIS_OFF;

   VAR(AX0_u16_Qep_Msq_Fltr_Mode) = (int)param;
   DesignGearMsqFilter(drive);

   SetGearingFFPointers(drive);

   return (SAL_SUCCESS);
}

int SalGearFiltAffCommand(long long param,int drive)
{
   BGVAR(s16_Gear_Filt_User_Aff) = (int)param;
   DesignGearMsqFilter(drive);
   return (SAL_SUCCESS);
}


int SalGearFiltT1Command(long long param,int drive)
{
   // AXIS_OFF;
   unsigned long u32_temp1;
   unsigned int u16_temp2;
   int s16_temp = (int)((float)VAR(AX0_u16_Pos_Loop_Tsp)*0.01);

   u32_temp1 = (unsigned long)param;
   u16_temp2 = (unsigned int)(u32_temp1 / s16_temp);
   if ( ((unsigned long)u16_temp2 * s16_temp) == u32_temp1 )          // check if user value is muliplication of 250 (0.25 * 1000)
   {
      BGVAR(u32_Gear_Filt_User_T1) = u32_temp1;
      DesignGearMsqFilter(drive);
      return (SAL_SUCCESS);
   }
   else
   {
      return (ARGUMENT_NOT_250);
   }
}


int SalGearFiltT2Command(long long param,int drive)
{
   // AXIS_OFF;
   unsigned int u16_temp1, u16_temp2;
   int s16_temp = (int)((float)VAR(AX0_u16_Pos_Loop_Tsp)*0.01);

   u16_temp1 = (unsigned int)param;
   u16_temp2 = u16_temp1 / s16_temp;
   if ( (u16_temp2 * s16_temp) == u16_temp1 )          // check if user value is muliplication of 250 (0.25 * 1000)
   {
      BGVAR(u16_Gear_Filt_User_T2) = u16_temp1;
      DesignGearMsqFilter(drive);
      return (SAL_SUCCESS);
   }
   else
   {
      return (ARGUMENT_NOT_250);
   }
}


int SalGearFiltVffCommand(long long param,int drive)
{
   BGVAR(s32_Gear_Filt_User_Vff) = (long)param;
   DesignGearMsqFilter(drive);
   return (SAL_SUCCESS);
}



int SalGearAccTreshCommand(long long lparam,int drive)
{
    //value must be between 0.06 rpm/sec to 1,000,000 rpm/sec
   int ret_val = SAL_SUCCESS;
   ret_val = CheckAccDecLimits(lparam);

   if (ret_val != SAL_SUCCESS)
   {
      return ret_val;
   }

   BGVAR(u64_Gear_Acc_Tresh) = (long long)lparam;
   DesignGearMsqFilter(drive);
   return (ret_val);
}

//**********************************************************
// Function Name: SalGearAccThreshInMsCommand
// Description:
//          This function is called in response to the P8-36 (GEARACCTHRESH) write in Schneider drive mode.
//          this function convert the entered ms to acc/dec rate from 0 to 6000 RPM in ms.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalGearAccThreshInMsCommand(long long lparam,int drive)
{
   int s16_ret_value = SAL_SUCCESS;
   long long tempAcc ;

   if(lparam > MIN_ACC_DEC_IN_MS)
   {
      return (VALUE_TOO_HIGH);
   }

   if(lparam  < MAX_ACC_DEC_IN_MS)
   {
      return (VALUE_TOO_LOW);
   }

   // convert from ms to CDHD acc units and then use the regular CDHD ACC write function
   tempAcc = convertMsToAccDecInternalUnitsForPos(drive,(unsigned int)lparam);

    //value must be between 0.06 rpm/sec to 6,000,000 rpm/sec
   s16_ret_value = CheckAccDecLimits(tempAcc);

   // 1/ACC -> if ms too low acc too high.
   if(s16_ret_value == (VALUE_TOO_HIGH))
   {
      s16_ret_value = (VALUE_TOO_LOW);
   }
   else if(s16_ret_value == (VALUE_TOO_LOW))
   {
      s16_ret_value = (VALUE_TOO_HIGH);
   }

   if(s16_ret_value == SAL_SUCCESS)
   {
      BGVAR(u64_Gear_Acc_Tresh) = (unsigned long long)tempAcc;
      DesignGearMsqFilter(drive);
   }
   return (s16_ret_value);
}


//**********************************************************
// Function Name: SalReadGearAccThreshInMs
// Description:
//          This function is called in response to the P8-36 (GEARACCTHRESH) read in Schneider drive mode.
//          this function convert the current GEARACCTHRESH to ACC/DEC in ms.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadGearAccThreshInMs(long long *data,int drive)
{
   *data =(long long)convertAccDecInternalUnitsForPosToMs(drive,(long long)BGVAR(u64_Gear_Acc_Tresh));
   return (SAL_SUCCESS);
}


int SalHWPextWriteCommand(long long param,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (param == 0LL)
   {
      VAR(AX0_u16_Gear_Skip_Flags) |= ZERO_HWPEXT_MASK;
   }

   return SAL_SUCCESS;
}

