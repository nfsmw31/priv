#include "Analog_If.def"
#include "ModCntrl.def"
#include "FPGA.def"
#include "MultiAxis.def"
#include "Err_Hndl.def"
#include "Design.def"
#include "Exe_IO.def"
#include "i2c.def"
#include "PtpGenerator.def"

#include "Drive.var"
#include "An_fltr.var"
#include "Foldback.var"
#include "Extrn_Asm.var"
#include "Units.var"
#include "Motor.var"
#include "Init.var"
#include "SensorlessBG.var"
#include "i2c.var"
#include "Exe_IO.var"
#include "FltCntrl.var"

#include "Prototypes.pro"

void CalcFoldbackParam(int drive, int foldback_type)
{
   unsigned long u32_fold_d, u32_fold_t, u32_fold_r;
   long s32_imax, s32_icont;
   unsigned int u16_shr, u16_foldback_not_ready;
   long long s64_fix, s64_half_for_rounding;
   float f_n_gain, f_p_gain, f_irail, f_ratio, f_foldr;

   s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s64_half_for_rounding = 0LL;
   if (u16_shr > 0)
   {
      s64_half_for_rounding = 1LL << (u16_shr - 1);
   }

   if (foldback_type == DRIVE_FOLDBACK)
   {
      u32_fold_d = BGVAR(u32_FoldD_Time);
      u32_fold_t = BGVAR(u32_FoldT_Time);
      u32_fold_r = BGVAR(u32_FoldR_Time);

      if ( BGVAR(u16_Foldback_At_Stall) == 1 )
      {
         u32_fold_d = BGVAR(u32_FoldD_Time_Calc);
         u32_fold_t = BGVAR(u32_FoldT_Time_Calc);
         u32_fold_r = BGVAR(u32_FoldR_Time_Calc);
      }

      s32_imax   = 26214;                   // DIPEAK
      s32_icont  = (long)((BGVAR(s32_Drive_I_Cont) * s64_fix + s64_half_for_rounding) >> u16_shr);
   }
   else
   {
      u32_fold_d = BGVAR(u32_Motor_FoldD_Time);
      u32_fold_t = BGVAR(u32_Motor_FoldT_Time);

      s32_imax   = (long)((BGVAR(s32_Motor_I_Peak) * s64_fix + s64_half_for_rounding) >> u16_shr);
      s32_icont  = (long)((BGVAR(s32_Motor_I_Cont) * s64_fix + s64_half_for_rounding) >> u16_shr);
      BGVAR(s32_Motor_I_Cont_Internal) = s32_icont;
      CalcTorqueCmdLimitTable(drive);

      // Calculate MFOLDR
      // FOLDR = (IPEAK^2*FOLDD + 5*ICONT^2*FOLDT + (IPEAK-ICONT)^2*FOLFT/2 + 2*ICONT*(IPEAK-ICONT)*FOLDT) / (ICONT*(MFOLDF/1000))^2 - FOLDD - 5*FOLDT
      //       Ratio = IPEAK / ICONT
      // FOLDR = (Ratio^2*FOLDD + 5*FOLDT + (Ratio-1)^2*FOLFT/2 + 2*(Ratio-1)*FOLDT) / (MFOLDF/1000)^2 - FOLDD - 5*FOLDT

      if ( (BGVAR(s32_Motor_I_Peak) <= BGVAR(s32_Motor_I_Cont)) || (BGVAR(s32_Motor_I_Cont) == 0) )
      {
         f_ratio = 1.0;
      }
      else
      {
         f_ratio = (float)BGVAR(s32_Motor_I_Peak) / (float)BGVAR(s32_Motor_I_Cont);
      }
      f_foldr = (f_ratio * f_ratio * (float)u32_fold_d +
                 5 * (float)u32_fold_t +
                 (f_ratio - 1) * (f_ratio - 1) * (float)u32_fold_t / 2 +
                 2 * (f_ratio - 1) * (float)u32_fold_t) /
                 ( (float)BGVAR(u16_Motor_Fold_Rms_Factor) * (float)BGVAR(u16_Motor_Fold_Rms_Factor) / 1000000.0) -
                (float)u32_fold_d - 5 * (float)u32_fold_t + 0.5;
      if (f_foldr < 1) f_foldr = 1;
      if (f_foldr > 3600000) f_foldr = 3600000;

      u32_fold_r = (unsigned long)f_foldr;
      BGVAR(u32_Motor_FoldR_Time) = u32_fold_r;
   }

   BGVAR(s32_I_Lim_Internal) = (long)((BGVAR(s32_Ilim_Actual) * s64_fix + s64_half_for_rounding) >> u16_shr);

   u16_foldback_not_ready = 0;

  /*  Calculate Fold_N_Gain.
   *  Fold_N_Gain = Inv_Fold_T_Time
   *  but it should be scaled by (1/32) since the Bg timer counts in (1/32)ms units.
   */
   f_n_gain = 1.0/(float)(u32_fold_t * 32);


  /* Calculate I_Rail, which is the initial value and the maximum of s32_I_Fold.
   *  I_Rail = (I_Max - I_Cont) * Fold_D_Time / Fold_T_Time  +  I_Max
   */
   if ( (u32_fold_d > 0) && (u32_fold_t > 0) && (s32_imax > s32_icont) )
   {
      f_irail = (((float)s32_imax - (float)s32_icont) * (float)u32_fold_d / (float)u32_fold_t) +  (float)s32_imax;
   }
   else if (u32_fold_t == 0)
   {
      f_irail = (float)0x7fffffff;
      u16_foldback_not_ready |= 0x0001;
   }
   else
   {
      f_irail = (float)s32_imax;
      if (s32_imax == 0)
      {
         u16_foldback_not_ready |= 0x0002;
      }
   }


  /* Calculate Fold_P_Gain.
   * Fold_P_Gain = (I_Rail - I_Cont) / (Fold_R_Time * I_Cont)
   *  but it should be scaled by (1/32) since the Bg timer counts in (1/32)ms units.
   */
   if ( (s32_icont > 0) && (u32_fold_r > 0) && ((long long)f_irail > s32_icont) )
   {
      f_p_gain =  (f_irail - (float)s32_icont) / (float)(u32_fold_r * 32) / (float)s32_icont;
   }
   else if ( (s32_icont == 0) || (u32_fold_r == 0) )
   {
      f_p_gain = (float)0x7fff;
      u16_foldback_not_ready |= 0x0004;
   }
   else if ((long long)f_irail == s32_icont)
   {
      f_p_gain = 0.000001;
   }
   else
   {
      f_p_gain = 0;
      u16_foldback_not_ready |= 0x0008;
   }


   if (foldback_type == DRIVE_FOLDBACK)
   {
      BGVAR(f_Fold_P_Gain_Regular) = f_p_gain;
      BGVAR(f_Fold_N_Gain_Regular) = f_n_gain;
      BGVAR(f_I_Rail_Regular) = f_irail;

      if ( BGVAR(u16_Foldback_At_Stall) == 0 )
      {
         BGVAR(f_Motion_I_Rail) = f_irail;

         // Update I_Fold when not at stall. Don't update when at stall to keep continuity
/*         if ( (u16_Init_State == 1)                                                    ||   // During Initialization
              (BGVAR(s32_I_Lim_Internal) == 0)                                         ||   // or when ILIM equals 0
              ( (BGVAR(u16_Foldback_Not_Ready) != 0) && (u16_foldback_not_ready == 0) )  )  // or upon transition from not active to active
         {
            // Set s32_I_Fold to highest value (I_Rail) to avoid Foldback Fault indication
            BGVAR(s32_I_Fold) = (long)f_irail;
         }
         else                     // During normal running
         {
            // Set s32_I_Fold to the current saturation value to maintain continuity.
            BGVAR(s32_I_Fold) = BGVAR(s32_I_Lim_Internal) + 1;
         }
*/
         // Init the IFOLD to IRAIL on any change to foldback parameters, to avoid waiting to the value to gradualy increase to the correct value
         // Although it will not keep the value continous, it will avoid false foldback warnings/faults
         BGVAR(s32_I_Fold) = (long)f_irail;

         BGVAR(f_I_Fold) = (float)BGVAR(s32_I_Fold);
         BGVAR(u16_Foldback_Not_Ready) = u16_foldback_not_ready;
      }
      else
      {
         // To maintain continuity (between stall and motion) of I_Fold when low current
         // is applied, stall I_Rail and motion I_Rail should be the same.
         // To keep the same timing, ngain and pgain will be changed in the same ratio ONLY WHEN
         // I_FOLD IS ABOVE DIPEAK.
         //
         // Ratio = (Motion_I_Rail - DIPEAK) / (Stall_I_Rail - DIPEAK)
         // New NGain = Ngain * Ratio
         // New Pgain = Pgain * Ratio

         f_ratio = (BGVAR(f_Motion_I_Rail) - 26214.0) / (f_irail - 26214.0);
         BGVAR(f_I_Rail) = BGVAR(f_Motion_I_Rail);
         BGVAR(f_Fold_P_Gain_High_I_Fold) = f_p_gain * f_ratio;
         BGVAR(f_Fold_N_Gain_High_I_Fold) = f_n_gain * f_ratio;
      }
   }
   else
   {
      BGVAR(f_Motor_Fold_P_Gain) = f_p_gain;
      BGVAR(f_Motor_Fold_N_Gain) = f_n_gain;
      BGVAR(f_Motor_I_Rail) = f_irail;

      /*if ( (u16_Init_State == 1)                                                          ||   // During Initialization
           (BGVAR(s32_I_Lim_Internal) == 0)                                               ||   // or when ILIM equals 0
           ( (BGVAR(u16_Motor_Foldback_Not_Ready) != 0) && (u16_foldback_not_ready == 0) )  )  // or upon transition from not active to active
      {
         // Set s32_Motor_I_Fold to highest value (MotorI_Rail) to avoid Foldback Fault indication
         BGVAR(s32_Motor_I_Fold) = (long)BGVAR(f_Motor_I_Rail);
      }
      else                     // During normal running
      {
         // Set s32_Motor_I_Fold to the current saturation value to maintain continuity.
         //BGVAR(s32_Motor_I_Fold) = BGVAR(s32_I_Lim_Internal) + 1;
         if ( BGVAR(s32_Motor_I_Fold) > (long)BGVAR(f_Motor_I_Rail) )
         {
            BGVAR(s32_Motor_I_Fold) = (long)BGVAR(f_Motor_I_Rail);
         }
      }
      */

      // Init the IFOLD to IRAIL on any change to foldback parameters, to avoid waiting to the value to gradualy increase to the correct value
      // Although it will not keep the value continous, it will avoid false foldback warnings/faults
      BGVAR(s32_Motor_I_Fold) = (long)BGVAR(f_Motor_I_Rail);

      BGVAR(f_Motor_I_Fold) = (float)BGVAR(s32_Motor_I_Fold);
      BGVAR(u16_Motor_Foldback_Not_Ready) = u16_foldback_not_ready;

      //Init for Load-Pct, avilable for =S= vie P0-02 value 12 and 13
      BGVAR(u32_Motor_Load_Pct_Max) = 0;
      //Max Pct is (Irail-Micont)/Micont , and factor is used to display Max as 300
      BGVAR(f_Motor_Load_Pct_Factor) = 300.0 /(BGVAR(f_Motor_I_Rail)- (float)BGVAR(s32_Motor_I_Cont_Internal));
   }

   BGVAR(u32_Stall_Current_Thresh_Internal) = (unsigned long)((BGVAR(u32_Stall_Current_Thresh) * s64_fix + s64_half_for_rounding) >> u16_shr);
   BGVAR(u16_Foldback_At_Stall) = 0;
}


// Evaluate the stall condition, and modify foldback coefficients accordingly
void FoldbackAtStall(int drive)
{
   float f_ratio, f_foldr, f_temp;
   unsigned long u32_prev_fold_d, u32_prev_fold_t, u32_prev_fold_r;
   unsigned int u16_stall_condition_flag;
   int s16_power_temp_deg;
   int drive_backup = drive;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u16_stall_condition_flag = 0;

   u32_prev_fold_d = BGVAR(u32_FoldD_Time_Calc);
   u32_prev_fold_t = BGVAR(u32_FoldT_Time_Calc);
   u32_prev_fold_r = BGVAR(u32_FoldR_Time_Calc);

   // For test purpose, allow overriding the measured power temperature
   if ( BGVAR(s16_Force_Stall_Temp) == 0 )
   {
      drive = 0;
      s16_power_temp_deg = (int)BGVAR(u16_IPM_Temperature);      // use measured power temperature
      drive = drive_backup;
   }
   else
   {
      s16_power_temp_deg = BGVAR(s16_Force_Stall_Temp);    // use debug power temperature
   }

   if (
        ( s16_power_temp_deg >= (int)BGVAR(u16_Stall_Thresh_Temp))                                                &&   // power temp is equal or greater than stall threshold temp
        ( BGVAR(u16_In_Stall_Flag) == 1 )                                                                         &&   // at stall (set at 1ms ISR)
        ( (BGVAR(u32_EqCrrnt_Avg_2) > BGVAR(u32_Stall_Current_Thresh_Internal)) || (BGVAR(u16_Force_Stall_Current) == 1) )      // current is above stall current threshold
      )
   {
      u16_stall_condition_flag = 1;

      // Calc FOLDD
      // FOLDD is a linear interpolation of FOLDD at OT and at stall threshold temperature
      f_temp = (float)BGVAR(u32_FoldD_At_Stall_Thresh_Temp) +
               BGVAR(f_Stall_Foldd_Slope) * ((float)s16_power_temp_deg - (float)BGVAR(u16_Stall_Thresh_Temp)) + 0.5;

      BGVAR(u32_FoldD_Time_Calc) = (unsigned long)f_temp;
      BGVAR(u32_FoldT_Time_Calc) = BGVAR(u32_FoldD_Time_Calc);    // Set FOLDT = FOLDD

      // Calculate FOLDR
      // FOLDR = (IPEAK^2*FOLDD + 5*ICONT^2*FOLDT + (IPEAK-ICONT)^2*FOLFT/2 + 2*ICONT*(IPEAK-ICONT)*FOLDT) / ICONT^2 - FOLDD - 5*FOLDT
      //       Ratio = IPEAK / ICONT
      // FOLDR = Ratio^2*FOLDD + 5*FOLDT + (Ratio-1)^2*FOLFT/2 + 2*(Ratio-1)*FOLDT - FOLDD - 5*FOLDT
      //       = Ratio^2*FOLDD + (Ratio-1)^2*FOLFT/2 + 2*(Ratio-1)*FOLDT - FOLDD

      if ( (BGVAR(s32_Drive_I_Peak) <= BGVAR(s32_Drive_I_Cont)) || (BGVAR(s32_Drive_I_Cont) == 0) )
      {
         f_ratio = 1.0;
      }
      else
      {
         f_ratio = (float)BGVAR(s32_Drive_I_Peak) / (float)BGVAR(s32_Drive_I_Cont);
      }
      f_foldr = f_ratio * f_ratio * (float)BGVAR(u32_FoldD_Time_Calc) +
                (f_ratio - 1) * (f_ratio - 1) * (float)BGVAR(u32_FoldT_Time_Calc) / 2 +
                2 * (f_ratio - 1) * (float)BGVAR(u32_FoldT_Time_Calc) -
                (float)BGVAR(u32_FoldD_Time_Calc) + 0.5;
      if (f_foldr < 1) f_foldr = 1;
      if (f_foldr > 3600000) f_foldr = 3600000;

      BGVAR(u32_FoldR_Time_Calc) = (unsigned long)f_foldr;
   }
   else
   {
      // Load the "regular" foldback param to the "calculation" param set
      BGVAR(u32_FoldD_Time_Calc) = BGVAR(u32_FoldD_Time);
      BGVAR(u32_FoldT_Time_Calc) = BGVAR(u32_FoldT_Time);
      BGVAR(u32_FoldR_Time_Calc) = BGVAR(u32_FoldR_Time);
   }


   if ( (u32_prev_fold_d != BGVAR(u32_FoldD_Time_Calc)) ||
        (u32_prev_fold_t != BGVAR(u32_FoldT_Time_Calc)) ||
        (u32_prev_fold_r != BGVAR(u32_FoldR_Time_Calc))    )
   {
      BGVAR(u16_Foldback_At_Stall) = 1;      // Indicate stall condition
      CalcFoldbackParam(DRIVE_PARAM, DRIVE_FOLDBACK);
   }


   // To maintain continuity (between stall and motion) of I_Fold when low current
   // is applied, stall I_Rail and motion I_Rail should be the same.
   // To keep the same timing, ngain and pgain will be changed in the same ratio ONLY WHEN
   // I_FOLD IS ABOVE DIPEAK.
   //
   // Ratio = (Motion_I_Rail - DIPEAK) / (Stall_I_Rail - DIPEAK)
   // New NGain = Ngain * Ratio
   // New Pgain = Pgain * Ratio
   if ( (BGVAR(f_I_Fold) > 26214.0) && (u16_stall_condition_flag == 1) )
   {                              // Use the special stall values
      BGVAR(f_Fold_P_Gain) = BGVAR(f_Fold_P_Gain_High_I_Fold);
      BGVAR(f_Fold_N_Gain) = BGVAR(f_Fold_N_Gain_High_I_Fold);
   }
   else
   {
      BGVAR(f_Fold_P_Gain) = BGVAR(f_Fold_P_Gain_Regular);
      BGVAR(f_Fold_N_Gain) = BGVAR(f_Fold_N_Gain_Regular);
   }

   BGVAR(f_I_Rail) = BGVAR(f_Motion_I_Rail);

   BGVAR(u32_Fold_P_Gain_For_Record) = (unsigned long)(BGVAR(f_Fold_P_Gain)*1000000*1000000);
   BGVAR(u32_Fold_N_Gain_For_Record) = (unsigned long)(BGVAR(f_Fold_N_Gain)*1000000*1000000);
   BGVAR(u32_I_Rail_For_Record) = (unsigned long)BGVAR(f_I_Rail);
}

void Foldback(int drive)
{
   int Bg_Time;
   long s32_temp_val;
   unsigned int u16_shr;
   long long s64_fix, s64_half_for_rounding;


   //  Perform the foldback algorithm only if at least 10 mS elapsed from last time
   if (PassedTimeMS(10L, BGVAR(s32_FoldbackTimer)) == 0) return;
   Bg_Time = 32*(Cntr_1mS - BGVAR(s32_FoldbackTimer));     // time is measured in Cntr_1mS units
   BGVAR(s32_FoldbackTimer) = Cntr_1mS;

   s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s64_half_for_rounding = 0LL;
   if (u16_shr > 0)
   {
      s64_half_for_rounding = 1LL << (u16_shr - 1);
   }

   //re-calculate s32_I_Lim_Internal according to ILIM actual
   BGVAR(s32_I_Lim_Internal) = (long)((BGVAR(s32_Ilim_Actual) * s64_fix + s64_half_for_rounding) >> u16_shr);

   //re-calculate s32_I_Cont_Internal according to the derated Drive I Cont
   BGVAR(s32_I_Cont_Internal) = (long)((BGVAR(s32_Derated_Drive_I_Cont) * s64_fix + s64_half_for_rounding) >> u16_shr);

   FoldbackAtStall(drive);   // Evaluate the stall condition, and modify foldback coefficients accordingly

  /*
   * Dribe foldback
   *
   * Calc IFold:
   * i>icont => Ifold = Ifold - ngain(i-icont)
   * i<icont => Ifold = Ifold + pgain(icont-i)
   */
   s32_temp_val = (long)BGVAR(u32_EqCrrnt_Avg) - (long)BGVAR(s32_I_Cont_Internal);
   if (s32_temp_val > 0)
   {
      BGVAR(f_I_Fold) -= (float)s32_temp_val * (float)BGVAR(f_Fold_N_Gain) * (float)Bg_Time;
      if (BGVAR(f_I_Fold) < (float)BGVAR(s32_I_Cont_Internal))
      {
         BGVAR(f_I_Fold) = (float)BGVAR(s32_I_Cont_Internal);
      }
   }
   else
   {
      BGVAR(f_I_Fold) -= (float)s32_temp_val * (float)BGVAR(f_Fold_P_Gain) * (float)Bg_Time;
      if (BGVAR(f_I_Fold) > BGVAR(f_I_Rail))
      {
         BGVAR(f_I_Fold) = BGVAR(f_I_Rail);
      }
   }
   BGVAR(s32_I_Fold) = (long)BGVAR(f_I_Fold);

  /*
   * Motor foldback
   *
   * Calc IFold:
   * i>icont => Ifold = Ifold - ngain(i-icont)
   * i<icont => Ifold = Ifold + pgain(icont-i)
   */
   s32_temp_val = (long)BGVAR(u32_EqCrrnt_Avg) - (long)BGVAR(s32_Motor_I_Cont_Internal);
   if (s32_temp_val > 0)
   {
      BGVAR(f_Motor_I_Fold) -= (float)s32_temp_val * (float)BGVAR(f_Motor_Fold_N_Gain) * (float)Bg_Time;
      if (BGVAR(f_Motor_I_Fold) < (float)BGVAR(s32_Motor_I_Cont_Internal))
      {
         BGVAR(f_Motor_I_Fold) = (float)BGVAR(s32_Motor_I_Cont_Internal);
      }
   }
   else
   {
      BGVAR(f_Motor_I_Fold) -= (float)s32_temp_val * (float)BGVAR(f_Motor_Fold_P_Gain) * (float)Bg_Time;
      if (BGVAR(f_Motor_I_Fold) > BGVAR(f_Motor_I_Rail))
      {
         BGVAR(f_Motor_I_Fold) = BGVAR(f_Motor_I_Rail);
      }
   }
   BGVAR(s32_Motor_I_Fold) = (long)BGVAR(f_Motor_I_Fold);

   // Calc Motor Load for =SE= P0-02 value 12 and 13.

   BGVAR(u32_Motor_Load_Pct) = (unsigned long)(0.5+((BGVAR(f_Motor_I_Rail)- BGVAR(f_Motor_I_Fold))*
                                                             BGVAR(f_Motor_Load_Pct_Factor)));

   //Keep Max
   if( BGVAR(u32_Motor_Load_Pct) > BGVAR(u32_Motor_Load_Pct_Max))
      BGVAR(u32_Motor_Load_Pct_Max)= BGVAR(u32_Motor_Load_Pct);

  /*
   * Set FOLD and MFOLD indications
   */
   if ( (BGVAR(s32_I_Lim_Internal) > BGVAR(s32_I_Fold)) && (BGVAR(u16_Foldback_Not_Ready) == 0) )
   {
      BGVAR(u16_Fold) = 1;
      BGVAR(u16_At_Fold) = 1;
   }
   else
   {
      BGVAR(u16_Fold) = 0;
      if ( (BGVAR(s32_At_Fold_Lim) > BGVAR(s32_I_Fold)) && (BGVAR(u16_Foldback_Not_Ready) == 0) ) BGVAR(u16_At_Fold) = 1;
      else BGVAR(u16_At_Fold) = 0;
   }

   if ( (BGVAR(s32_I_Lim_Internal) > BGVAR(s32_Motor_I_Fold)) &&
        (BGVAR(u16_Motor_Foldback_Disable) == 0)              &&
        (BGVAR(u16_Motor_Foldback_Not_Ready) == 0)                )
   {
      BGVAR(u16_Motor_Fold) = 1;
   }
   else
   {
      BGVAR(u16_Motor_Fold) = 0;
   }

   SetCurrentCommandSaturation(drive);

   // Trigger averaging of I
   BGVAR(u32_EqCrrnt_Acc) = 0;
   BGVAR(u16_EqCrrnt_Counter) = 0;
}


#pragma CODE_SECTION(SetCurrentCommandSaturation, "ramfunc_2");
void SetCurrentCommandSaturation(int drive)
{
   // AXIS_OFF;

   long s32_temp_val;
   unsigned int u16_shr;
   long long s64_fix, s64_half_for_rounding;
   unsigned int u16_LS_active_in_opmode_torque = 0; // Flag that indicates an active LS in opmode torque (opmode 2 and 3)
   REFERENCE_TO_DRIVE;

   s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s64_half_for_rounding = 0LL;
   if (u16_shr > 0)
   {
      s64_half_for_rounding = 1LL << (u16_shr - 1);
   }

   //re-calculate s32_I_Lim_Internal according to ILIM actual
   BGVAR(s32_I_Lim_Internal) = (long)((BGVAR(s32_Ilim_Actual) * s64_fix + s64_half_for_rounding) >> u16_shr);

  /*
   * Determine the current command saturation as minimum of Drive IFold, Motor IFold, and Ilim
   */
   s32_temp_val = BGVAR(s32_I_Lim_Internal);
   if (s32_temp_val > BGVAR(s32_I_Fold))
   {
      s32_temp_val = BGVAR(s32_I_Fold);
   }
   if ( (s32_temp_val > BGVAR(s32_Motor_I_Fold)) && (BGVAR(u16_Motor_Foldback_Disable) == 0) )
   {
      s32_temp_val = BGVAR(s32_Motor_I_Fold);
   }

   // Here apply the current limitation from the feature "velocity limit in torque mode"
   if ((long)BGVAR(s16_Torque_Mode_Vel_Limit_Current_Limitation) < s32_temp_val)
   {
      s32_temp_val = (long)BGVAR(s16_Torque_Mode_Vel_Limit_Current_Limitation);
   }

   // If a limit switch HOLD is in process
   if(VAR(AX0_u16_HP_Flags) & HOLD_IN_PROCESS_MASK)
   {
      // In OPMODE torque ensure that no motion will happen. Do not interfere with other OPMODEs,
      // especially in case that the OPMODE has been adjusted during a pending hold. Therefore
      // check the OPMODE also here.
      // check if command pointer is still pointing to one of the current commands to avoid
      // saturation release (motor jump) in case we are in diffrent opmode but the pointer
      // didn't change yet and still pointing to current command.
      // Use also the OPMODE aligned variable to avoid a jump when switching away
      // from analog torque during OPMODE change while HOLD is active.
      if((VAR(AX0_s16_Opmode) == 2)         || (VAR(AX0_s16_Opmode) == 3) ||
         (VAR(AX0_s16_Opmode_Aligned) == 2) || (VAR(AX0_s16_Opmode_Aligned) == 3))
      {
         if (BGVAR(u16_Brake_Release_State) == BRAKE_RELEASE_IDLE)
         {
            // Set saturation to 0
            s32_temp_val = 0;
         }
         
         // Set flag to ensure that s32_temp_val is really applied in the following code lines
         u16_LS_active_in_opmode_torque = 1;
      }
   }

   // Consider current limit feature for Schneider
   if ( (BGVAR(s16_I_Sat_Hi) < (int)s32_temp_val) && (BGVAR(s16_I_Sat_Hi_En)) && (u16_LS_active_in_opmode_torque == 0) )
   {
      VAR(AX0_s16_I_Sat_Hi_Shadow) = BGVAR(s16_I_Sat_Hi);
   }
   else
   {
      VAR(AX0_s16_I_Sat_Hi_Shadow) = (int)s32_temp_val;
   }

   if ( (BGVAR(s16_I_Sat_Lo) < (int)s32_temp_val) && (BGVAR(s16_I_Sat_Lo_En)) && (u16_LS_active_in_opmode_torque == 0) )
   {
      VAR(AX0_s16_I_Sat_Lo_Shadow) = -BGVAR(s16_I_Sat_Lo);
   }
   else
   {
      VAR(AX0_s16_I_Sat_Lo_Shadow) = -(int)s32_temp_val;
   }

   // Here determine the minimum of positive and negative saturation,
   // which is needed for the runaway condition check.
   if (VAR(AX0_s16_I_Sat_Hi) < abs(VAR(AX0_s16_I_Sat_Lo)))
   {
      BGVAR(u16_Runaway_Check_Curr_Thresh) = VAR(AX0_s16_I_Sat_Hi);
   }
   else
   {
      BGVAR(u16_Runaway_Check_Curr_Thresh) = abs(VAR(AX0_s16_I_Sat_Lo));
   }
}


#pragma CODE_SECTION(CalcEqCurrent, "ramfunc_5");
void CalcEqCurrent(int drive)
{
// This function is called from the 1ms ISR
// The execution time of this function from the internal RAM is about 2.2us

/* Ieq = sqrt ( 4/3*(iv*iv + iu*iu + iv*iu) )
 * 4/3 = 0x5555>>14
*/
   // AXIS_OFF;

   int  i, iq, crrnt_u, crrnt_v;
   unsigned long u32_root, u32_value, u32_temp;
   long s32_i_signed;
   REFERENCE_TO_DRIVE;

   i = Cntr_3125;
   crrnt_u = VAR(AX0_s16_Crrnt_Srvo_Cmnd_U_Act_0);
   crrnt_v = VAR(AX0_s16_Crrnt_Srvo_Cmnd_V_Act_0);
   iq = VAR(AX0_s16_Crrnt_Q_Act_0);
   if (Cntr_3125 != i)
   {
      crrnt_u = VAR(AX0_s16_Crrnt_Srvo_Cmnd_U_Act_0);
      crrnt_v = VAR(AX0_s16_Crrnt_Srvo_Cmnd_V_Act_0);
      iq = VAR(AX0_s16_Crrnt_Q_Act_0);
   }

   u32_value = (long)crrnt_v*(long)crrnt_v + (long)crrnt_u*(long)crrnt_u + (long)crrnt_u*(long)crrnt_v;

   u32_value = (unsigned long)(((unsigned long long)u32_value * 0x5555) >> 14);

   /* Check that the squared value does not exceed (0x6666)^2.
    * This is because the result of the sqrt is still to be multiplied
    * by 2.5,and must be (after the multiplication) less than
    * (unsigned) 0xffff.(0x6666)^2 = 0x28F570A4
    */
   if (u32_value < 0x28f570a4)
   {
      u32_temp = 0x00004000;
      u32_root = 0;
      for (i = 0; i < 15; i++)
      {
         u32_root += u32_temp;
         if ((u32_root*u32_root) > u32_value)
         {
            u32_root -= u32_temp;
         }
         u32_temp >>= 1;
      }
   }
   else
   {
      u32_root = 0x6666;
   }

   // Average 2 samples of I
   BGVAR(u32_EqCrrnt_Avg_2) = (BGVAR(u32_EqCrrnt) + u32_root + 1) >> 1;

   // Store I
   BGVAR(u32_EqCrrnt) = u32_root;

   // take I with sign (for canopen object 0x6078)
   s32_i_signed = (long)u32_root;
   if (iq < 0)
   {
      // set sign accordingly.
      BGVAR(s32_EqCrrnt_Signed) = -s32_i_signed;
   }
   else
   {
      BGVAR(s32_EqCrrnt_Signed) = s32_i_signed;
   }
   
   // Average 8 samples of I
   if (BGVAR(u16_EqCrrnt_Counter) < 8)
   {
      BGVAR(u32_EqCrrnt_Acc) += u32_root;
      BGVAR(u16_EqCrrnt_Counter)++;
   }
   if (BGVAR(u16_EqCrrnt_Counter) == 8)
   {
      BGVAR(u32_EqCrrnt_Avg) = (BGVAR(u32_EqCrrnt_Acc) + 4) >> 3;
      BGVAR(u16_EqCrrnt_Counter)++;
   }
}


//**********************************************************
// Function Name: SalMotorFoldTCommand
// Description:
//          This function is called in response to the MFOLDT command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorFoldTCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u32_Motor_FoldT_Time) = (unsigned long)lparam;

   CalcFoldbackParam(DRIVE_PARAM, MOTOR_FOLDBACK);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalMotorFoldDCommand
// Description:
//          This function is called in response to the MFOLDD command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorFoldDCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u32_Motor_FoldD_Time) = (unsigned long)lparam;

   CalcFoldbackParam(DRIVE_PARAM, MOTOR_FOLDBACK);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalMotorFoldRmsFactorCommand
// Description:
//          This function is called in response to the MFOLDF command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorFoldRmsFactorCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Motor_Fold_Rms_Factor) = (unsigned int)lparam;

   CalcFoldbackParam(DRIVE_PARAM, MOTOR_FOLDBACK);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalMotorFoldRCommand
// Description:
//          This function is called in response to the MFOLDR command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
//int SalMotorFoldRCommand(long long lparam,int drive)
//{
//   BGVAR(u32_Motor_FoldR_Time) = (unsigned long)lparam;
//
//   CalcFoldbackParam(DRIVE_PARAM, MOTOR_FOLDBACK);
//
//   return (SAL_SUCCESS);
//}


//**********************************************************
// Function Name: SalFoldFltThreshCommand
// Description:
//          This function is called in response to the IFOLDFTHRESH command.
//
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalFoldFltThreshCommand(long long lparam,int drive)
{
unsigned int u16_shr;
long long s64_fix, s64_half_for_rounding, s64_upper_limit;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL)   return (VALUE_TOO_LOW);

   // check upper limit of 300.000 A.
   // convert to internal units
   s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s64_half_for_rounding = 0LL;
   if (u16_shr > 0)
   {
      s64_half_for_rounding = 1LL << (u16_shr - 1);
   }
   s64_upper_limit  = (long)((300000LL * s64_fix + s64_half_for_rounding) >> u16_shr);

   if (lparam > s64_upper_limit) return (VALUE_TOO_HIGH);

   BGVAR(u32_I_Fold_Fault_Threshold) = (unsigned long)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalFoldWrnThreshCommand
// Description:
//          This function is called in response to the IFOLDWTHRESH command.
//
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalFoldWrnThreshCommand(long long lparam,int drive)
{
unsigned int u16_shr;
long long s64_fix, s64_half_for_rounding, s64_upper_limit;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL)   return (VALUE_TOO_LOW);

   // check upper limit of 300.000 A.
   // convert to internal units
   s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s64_half_for_rounding = 0LL;
   if (u16_shr > 0)
   {
      s64_half_for_rounding = 1LL << (u16_shr - 1);
   }
   s64_upper_limit  = (long)((300000LL * s64_fix + s64_half_for_rounding) >> u16_shr);

   if (lparam > s64_upper_limit) return (VALUE_TOO_HIGH);

   BGVAR(u32_I_Fold_Warning_Threshold) = (unsigned long)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMotorFoldFltThreshCommand
// Description:
//          This function is called in response to the MIFOLDFTHRESH command.
//
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorFoldFltThreshCommand(long long lparam,int drive)
{
unsigned int u16_shr;
long long s64_fix, s64_half_for_rounding, s64_upper_limit;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL)   return (VALUE_TOO_LOW);

   // check upper limit of 300.000 A.
   // convert to internal units
   s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s64_half_for_rounding = 0LL;
   if (u16_shr > 0)
   {
      s64_half_for_rounding = 1LL << (u16_shr - 1);
   }
   s64_upper_limit  = (long)((300000LL * s64_fix + s64_half_for_rounding) >> u16_shr);

   if (lparam > s64_upper_limit) return (VALUE_TOO_HIGH);

   BGVAR(u32_Motor_I_Fold_Fault_Threshold) = (unsigned long)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMotorFoldWrnThreshCommand
// Description:
//          This function is called in response to the MIFOLDWTHRESH command.
//
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorFoldWrnThreshCommand(long long lparam,int drive)
{
   unsigned int u16_shr;
   long long s64_fix, s64_half_for_rounding, s64_upper_limit;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL)   return (VALUE_TOO_LOW);

   // check upper limit of 300.000 A.
   // convert to internal units
   s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s64_half_for_rounding = 0LL;
   if (u16_shr > 0)
   {
      s64_half_for_rounding = 1LL << (u16_shr - 1);
   }
   s64_upper_limit  = (long)((300000LL * s64_fix + s64_half_for_rounding) >> u16_shr);

   if (lparam > s64_upper_limit) return (VALUE_TOO_HIGH);

   BGVAR(u32_Motor_I_Fold_Warning_Threshold) = (unsigned long)lparam;

   return (SAL_SUCCESS);
}


//*****************************************************************************************
// Function Name: RunVelLimitPiLoop
// Description: This function is a PI-Loop, which output is meant to be a current
//              limit. In OPMODE torque the PI loop checks the difference between
//              a velocity limit parameter and the actual velocity and produces
//              an output, which is a current limit of the Drive. This function
//              prevents the motor to run in overspeed when commanding an current
//              in OPMODE torque.
//
// Author: APH
// Algorithm:
// Revisions:
//*****************************************************************************************
#pragma CODE_SECTION(RunVelLimitPiLoop, "ramfunc_3");
void RunVelLimitPiLoop (int drive)
{
   // AXIS_OFF; // Define axis offset of 64 * drive due to the location of rt variables in pages.
             // Needed since RT variables are used in this function.

   // Local variable which will hold the result of the PI loop
   signed long s32_result = 0x7FFF; // Maximum value
   unsigned int u16_temp1, u16_temp2;

   // need to check here if the opmode and the current command pointer are aligned
   // in order to avoid condition when opmode changed to torque but the current command pointer
   // stil pointing to the velocity current command.
   // IPR 1429: Motor phase monitor activate,but no response when motor UVW lost and in standstill status.
   // go in only if not during motor phase disconnect test (on enabling drive).
   // BZ#5444 - "No current or no fake commutation (v=0) when moving to burnin": avoid calculating velocity limitation in burning mode 
   if ( (BGVAR(u16_Brake_Release_State) == BRAKE_RELEASE_IDLE) &&
        (BGVAR(u16_Run_Torque_Mode_Vel_Limit_Pi_Loop) || (VAR(AX0_s16_Opmode) == 2) || (VAR(AX0_s16_Opmode) == 3)) &&
         (BGVAR(s8_BurninParam) == 0))
   // If the speed-limit in torque mode is being activated or in torque mode. This is determined
   // by the "ProcessVelocityCommand" function.
   {

      // Calculate the difference between the absolute values of the command and the actual velocity value
      int s16_Control_Loop_Deviation = abs(BGVAR(s16_Torque_Mode_Vel_Limit_Velocity_Thresh)) - abs(VAR(AX0_s16_Vel_Var_Fb_0));

      if ((s16_Control_Loop_Deviation > 0) && (BGVAR(u16_Pi_Active_Latch) == 0))
      {
         // The actual velocity is below velocity limit.
         // Do not apply current limit. Load the integrator with the minimum of ICMD and IQ (abs values) to maintain continuity when activating the PI loop.
         u16_temp1 = abs(VAR(AX0_s16_Crrnt_Q_Act_0));
         u16_temp2 = abs(VAR(AX0_s16_Icmd));
         if (u16_temp2 < u16_temp1)
         {
            u16_temp1 = u16_temp2;
         }

         // Here load the integrator with a value that would lead to a pure integrator result (without proportional part) of u16_temp. Therefore multiply
         // the torque value with the integrator-time (because the integrator-gain in the PI-loop is "1/integrator-time") and add also the proportional
         // gain-fix to the result.
         // After the instruction the pure integrator result would be:
         //   "(BGVAR(s64_Torque_Mode_Vel_Limit_Pi_Integral_Part) * BGVAR(s16_Torque_Mode_Vel_Limit_Integral_Time_Inverse_Fix)) >> (BGVAR(u16_Torque_Mode_Vel_Limit_Prop_Gain_Shift) + BGVAR(u16_Torque_Mode_Vel_Limit_Integral_Time_Inverse_Shift))" = u16_temp
         BGVAR(s64_Torque_Mode_Vel_Limit_Pi_Integral_Part) = ((long long)((long)u16_temp1 * BGVAR(u16_Integral_Time_Ms))) << BGVAR(u16_Torque_Mode_Vel_Limit_Prop_Gain_Shift);

         s32_result = ((long)abs(BGVAR(s16_Torque_Mode_Vel_Limit_Velocity_Thresh)) * BGVAR(s16_Torque_Mode_Vel_Limit_Current_Limit_Factor_Fix))
                       >> BGVAR(u16_Torque_Mode_Vel_Limit_Current_Limit_Factor_Shr);
         if (s32_result > 32767L)
         {
            s32_result = 32767L;
         }
      }
      else
      {
         // Multiply the input with Kp, which is the proportional part. First without considering the variable
         // u16_Torque_Mode_Vel_Limit_Prop_Gain_Shift in order to gain accuracy in the integrator if the input value is very small
         // (avoid truncation after bitshift to the right).
         BGVAR(s32_Torque_Mode_Vel_Limit_Pi_Proportional_Part) = (signed long)s16_Control_Loop_Deviation * BGVAR(s16_Torque_Mode_Vel_Limit_Prop_Gain_Fix);

         // Perform the integral part without involving u16_Torque_Mode_Vel_Limit_Prop_Gain_Shift
         // and u16_Torque_Mode_Vel_Limit_Integral_Time_Inverse_Shift in order to gain accuracy if the input value is very small
         // (avoid truncation after bitshift to the right).
         BGVAR(s64_Torque_Mode_Vel_Limit_Pi_Integral_Part) += (long long)BGVAR(s32_Torque_Mode_Vel_Limit_Pi_Proportional_Part);

         if (BGVAR(s64_Torque_Mode_Vel_Limit_Pi_Integral_Part) < 0)
         {
            BGVAR(s64_Torque_Mode_Vel_Limit_Pi_Integral_Part) = 0;
         }

         // Calculate result = proportional_part + integral_part, this time considering the shift-factors.
         s32_result = (BGVAR(s32_Torque_Mode_Vel_Limit_Pi_Proportional_Part) >> BGVAR(u16_Torque_Mode_Vel_Limit_Prop_Gain_Shift)) +
                      (signed long)(((signed long long)BGVAR(s64_Torque_Mode_Vel_Limit_Pi_Integral_Part) * BGVAR(s16_Torque_Mode_Vel_Limit_Integral_Time_Inverse_Fix)) >> (BGVAR(u16_Torque_Mode_Vel_Limit_Prop_Gain_Shift) + BGVAR(u16_Torque_Mode_Vel_Limit_Integral_Time_Inverse_Shift)));

         if (s32_result < 0)
         {
            s32_result = 0;
         }
         if (s32_result > 32767L)
         {
            s32_result = 32767L;
         }

         // check when to stop using the PI loop for current limit.
         // Set a counter for the first lime the PI loop is used.
         // Each sample time that the current command is below the PI current limit decrement the counter (not below 0), otherwise increment
         // the counter (not above 10).
         if (BGVAR(u16_Pi_Active_Latch) == 0)
         {
            BGVAR(u16_Pi_Active_Latch) = 10;   // arbitrary value
         }

         u16_temp1 = abs(*(LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr)) >> 1);  // 50% of the current command
         if (u16_temp1 < (int)s32_result)
         {
            if (BGVAR(u16_Pi_Active_Latch) > 0)
            {
               BGVAR(u16_Pi_Active_Latch)--;
            }
         }
         else
         {
            if (BGVAR(u16_Pi_Active_Latch) < 10)
            {
               BGVAR(u16_Pi_Active_Latch)++;
            }
         }

      }

   } // End of OPMODE 2

   // Write the result into the current limit variable
   BGVAR(s16_Torque_Mode_Vel_Limit_Current_Limitation) = (int)s32_result;
   SetCurrentCommandSaturation(drive);
}




