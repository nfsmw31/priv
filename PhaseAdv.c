#include "ModCntrl.def"
#include "MultiAxis.def"
#include "Err_Hndl.def"
#include "Design.def"
#include "FltCntrl.def"
#include "PhaseAdv.def"

#include "Motor.var"
#include "Drive.var"
#include "ModCntrl.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Units.var"
#include "SensorlessBG.var"
#include "PhaseAdv.var"
#include "User_Var.var"
#include "AutoTune.var"

#include "Prototypes.pro"


void TorquePhaseAdvDesign(int drive)
{
 /*
  * Description:
  *  MTANGLC  is the phase advance in electrical degrees
  * at MICONT.
  *  MTANGLC  is the phase advance in electrical degrees
  * at MIPEAK.
  *
  *              |
  *   MTANGLP -|               /x
  *            |           /
  *            |       /
  *   MTANGLC -|    /x
  *            |  /
  *            |/
  *            ---------------------------> ICMD
  *                  |          |
  *               MICONT      MIPEAK
  *
  *       For ICMD >=0 to ICMD<= MICONT (slope 1) the torque phase advance is
  *                                  =  (ICMD  / MICONT) * MTANGLC
  *     we need to calculate fix & shift for RT
  *     torque phase advance(slope 1) =   ICMD * Slope_1_Fix >> Slope_1_Shr
  *     Slope_1_Fix >> Slope_1_Shr = MTANGLC / MICONT

  *     For ICMD > MICONT (slope 2) the torque phase advance is
  *     torque phase advance = MTANGLC + ((ICMD - MICONT) / (MIPEAK - MICONT)) * (MTANGLP - MTANGLC)
  *     we need to calculate fix & shift for RT
  *     torque phase advance(slope 2) =   Slope_2_Offset + ICMD * Slope_1_Fix >> Slope_1_Shr
  *     Slope_2_Offset = MTANGLC - MICONT*(MTANGLP - MTANGLC)/(MIPEAK - MICONT)
  *     Slope_2_Fix >> Slope_2_Shr = (MTANGLP - MTANGLC)/(MIPEAK - MICONT)
  *
  */

   // AXIS_OFF;
   float f_mcont_phase_advance = BGVAR(s16_Mcont_T_Adv) * 65536L / 360.0;  //convert from degree to bits
   float f_mpeak_phase_advance = BGVAR(s16_Mpeak_T_Adv) * 65536L / 360.0;  //convert from degree to bits
   float f_temp;
   long  s32_temp;

   //convert micont and mipeak to internal units
   unsigned long u32_micont = (BGVAR(s32_Motor_I_Cont) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
                        >> (long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   unsigned long u32_mipeak = (BGVAR(s32_Motor_I_Peak) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
                        >> (long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;

   REFERENCE_TO_DRIVE;
   //Slope_1_Fix >> Slope_1_Shr = MTANGLC / MICONT
   f_temp = f_mcont_phase_advance / (float)u32_micont;
   FloatToFix16Shift16(&VAR(AX0_s16_Crrnt_Phase_Adv_Slope_1_Fix), (unsigned int *)&VAR(AX0_s16_Crrnt_Phase_Adv_Slope_1_Shr), f_temp);

   if (u32_mipeak == u32_micont)
   {
      VAR(AX0_s16_Crrnt_Phase_Adv_Slope_2_Offset) = (int)f_mcont_phase_advance;
      VAR(AX0_s16_Crrnt_Phase_Adv_Slope_2_Fix) = 0;
      VAR(AX0_s16_Crrnt_Phase_Adv_Slope_2_Shr) = 0;
   }
   else
   {
      //Slope_2_Offset = MTANGLC - MICONT*(MTANGLP - MTANGLC)/(MIPEAK - MICONT)
      s32_temp  = (long)f_mcont_phase_advance - u32_micont*((long)f_mpeak_phase_advance - (long)f_mcont_phase_advance)/(u32_mipeak - u32_micont);
      VAR(AX0_s16_Crrnt_Phase_Adv_Slope_2_Offset) = (int)s32_temp;

      //Slope_2_Fix >> Slope_2_Shr = (MTANGLP - MTANGLC)/(MIPEAK - MICONT)
      f_temp =  (f_mpeak_phase_advance - f_mcont_phase_advance)/(float)(u32_mipeak - u32_micont);
      FloatToFix16Shift16(&VAR(AX0_s16_Crrnt_Phase_Adv_Slope_2_Fix), (unsigned int *)&VAR(AX0_s16_Crrnt_Phase_Adv_Slope_2_Shr), f_temp);
   }

   if (u32_micont > 32767L) u32_micont = 32767L;
   VAR(AX0_s16_Crrnt_Phase_Adv_Tresh) = u32_micont; // Threshold = MICONT
}


void SpeedPhaseAdvDesign(int drive)
{
   /*
   * Description:
   * MVANGLH is the phase advance in electrical degrees
   * at half of MSPEED.
   * MVANGLF is the phase advance in electrical degrees at
   * MSPEED.
   *              |
   *   MVANGLF -|               /x
   *            |           /
   *            |       /
   *   MVANGLH -|    /x
   *            |  /
   *            |/
   *            ---------------------------> V
   *                  |          |
   *               MSPEED/2    MSPEED
   *
   *
   *       For V>=0 to V<=MSPEED/2 (slope 1) the phase advance is
   *     speed phase advance(degrees) =  (V_rpm  / (MSPEED_rpm/2)) * MVANGLH =
   *     (convert to velocity units)  =  (V_bits / (MSPEED_rpm/2 / VLIM_rpm * 22750)) * MVANGLH
   *                                  =  (V_bits * MVANGLH * VLIM_rpm) / (MSPEED_rpm *11375)
   *
   *
   *     we need to calculate fix & shift for RT
   *     speed phase advance(slope 1) =   V_bits * Slope_1_Fix >> Slope_1_Shr
   *
   *     Slope_1_Fix >> Slope_1_Shr = MVANGLH*VLIM_rpm / MSPEED_rpm *11375
   *
   *     For V>MSPEED/2 (slope 2) the phase advance is
   *     speed phase advance = MVANGLH + ((V - MSPEED_rpm/2) / (MSPEED_rpm/2)) * (MVANGLF - MVANGLH)
   *                         = MVANGLH + (V/(MSPEED_rpm/2) - 1) * (MVANGLF - MVANGLH)
   *                         = MVANGLH + (V/(MSPEED_rpm/2) * (MVANGLF - MVANGLH) - MVANGLF + MVANGLH
   *                           = (V_rpm/(MSPEED_rpm/2) * (MVANGLF - MVANGLH) + 2*MVANGLH - MVANGLF
   *
   *     we need to calculate fix & shift for RT
   *     speed phase advance(slope 2) =   V_bits * Slope_2_Fix >> Slope_2_Shr  + Slope_2_Offset
   *
   *     Slope_2_Offset = 2*MVANGLH - MVANGLF
   *     Slope_2_Fix >> Slope_2_Shr  = (MVANGLF - MVANGLH) / MSPEED_rpm/2
   *     (convert to velocity units) = (MVANGLF - MVANGLH) / (MSPEED_rpm/2 * 22750/VLIM_rpm)
   *                                 = (MVANGLF - MVANGLH)*VLIM_rpm  /  (MSPEED_rpm * 11375)
   *
   */

   // AXIS_OFF;
   float f_temp,f_mspeed_phase_advance,f_half_mspeed_phase_advance,f_mspeed_internal_phase = 0.0;

   REFERENCE_TO_DRIVE;
   //check that VLIM & MSPEED greater zero
   if ((BGVAR(s32_V_Lim_Design) == 0) || (BGVAR(u32_Mspeed) == 0))
   {
      VAR(AX0_s16_Speed_Phase_Adv_Slope_1_Fix) = 0;
      VAR(AX0_u16_Speed_Phase_Adv_Slope_1_Shr) = 0;
      VAR(AX0_s16_Speed_Phase_Adv_Slope_2_Offset) = 0;
      VAR(AX0_s16_Speed_Phase_Adv_Slope_2_Fix) = 0;
      VAR(AX0_u16_Speed_Phase_Adv_Slope_2_Shr) = 0;
      VAR(AX0_u16_Speed_Phase_Adv_Threshold) = 0;

      VAR(AX0_s16_Sl_Speed_Phase_Adv_Slope_1_Fix) = 0;
      VAR(AX0_u16_Sl_Speed_Phase_Adv_Slope_1_Shr) = 0;
      VAR(AX0_s16_Sl_Speed_Phase_Adv_Slope_2_Offset) = 0;
      VAR(AX0_s16_Sl_Speed_Phase_Adv_Slope_2_Fix) = 0;
      VAR(AX0_u16_Sl_Speed_Phase_Adv_Slope_2_Shr) = 0;

      return;
   }

   // calculate internal phase advance due to drive digital implementation using V and 1.5 Tsc
   // Phase[65536=360] = 1.5*Tsc/(1/(V[fbc32/Tsv]/2^32/Tsv))*65536
   //                  = 65536*1.5*Tsc*V[fbc32/Tsv]/Tsv/2^32
   if (!BGVAR(u8_Sl_Mode)) // On SL dont add the internal phsae advance
      f_mspeed_internal_phase = 0.0000057220458984375 * (float)BGVAR(u32_Mspeed);
   f_mspeed_phase_advance      = f_mspeed_internal_phase + (float)BGVAR(s16_Mspeed_Speed_Advance) * 65536.0 / 360.0;  //convert from degree to bits
   f_half_mspeed_phase_advance = f_mspeed_internal_phase/2.0 + (float)BGVAR(s16_Half_Mspeed_Speed_Advance) * 65536.0 / 360.0; //convert from degree to bits

   // Slope_1_Fix >> Slope_1_Shr = MVANGLH*VLIM_rpm / (MSPEED_rpm *11375)
   f_temp =  f_half_mspeed_phase_advance * BGVAR(s32_V_Lim_Design) / BGVAR(u32_Mspeed) / 11375;
   FloatToFix16Shift16(&VAR(AX0_s16_Speed_Phase_Adv_Slope_1_Fix), (unsigned int *)&VAR(AX0_u16_Speed_Phase_Adv_Slope_1_Shr), f_temp);

   // Slope_2_Offset = 2*MVANGLH - MVANGLF
   VAR(AX0_s16_Speed_Phase_Adv_Slope_2_Offset) = 2*f_half_mspeed_phase_advance - f_mspeed_phase_advance;

   // Slope_2_Fix >> Slope_2_Shr  = (MVANGLF - MVANGLH)*VLIM_rpm  /  (MSPEED_rpm * 11375)
   f_temp =  (f_mspeed_phase_advance - f_half_mspeed_phase_advance) * BGVAR(s32_V_Lim_Design) / BGVAR(u32_Mspeed) / 11375;
   FloatToFix16Shift16(&VAR(AX0_s16_Speed_Phase_Adv_Slope_2_Fix), (unsigned int *)&VAR(AX0_u16_Speed_Phase_Adv_Slope_2_Shr), f_temp);

   // Handle Sensorless VANGLes
   f_mspeed_phase_advance      = (float)BGVAR(s16_Sl_Mspeed_Speed_Advance) * 65536.0 / 360.0;  //convert from degree to bits
   f_half_mspeed_phase_advance = (float)BGVAR(s16_Sl_Half_Mspeed_Speed_Advance) * 65536.0 / 360.0; //convert from degree to bits

   //Slope_1_Fix >> Slope_1_Shr = SLVANGLH*VLIM_rpm / (MSPEED_rpm *11375)
   f_temp =  f_half_mspeed_phase_advance * BGVAR(s32_V_Lim_Design) / BGVAR(u32_Mspeed) / 11375;
   FloatToFix16Shift16(&VAR(AX0_s16_Sl_Speed_Phase_Adv_Slope_1_Fix), (unsigned int *)&VAR(AX0_u16_Sl_Speed_Phase_Adv_Slope_1_Shr), f_temp);

   // Slope_2_Offset = 2*SLVANGLH - SLVANGLF
   VAR(AX0_s16_Sl_Speed_Phase_Adv_Slope_2_Offset) = 2*f_half_mspeed_phase_advance - f_mspeed_phase_advance;

   //Slope_2_Fix >> Slope_2_Shr  = (SLVANGLF - SLVANGLH)*VLIM_rpm  /  (MSPEED_rpm * 11375)
   f_temp =  (f_mspeed_phase_advance - f_half_mspeed_phase_advance) * BGVAR(s32_V_Lim_Design) / BGVAR(u32_Mspeed) / 11375;
   FloatToFix16Shift16(&VAR(AX0_s16_Sl_Speed_Phase_Adv_Slope_2_Fix), (unsigned int *)&VAR(AX0_u16_Sl_Speed_Phase_Adv_Slope_2_Shr), f_temp);

   // Convert MSPEED/2 to internal units
   f_temp =  (float)BGVAR(u32_Mspeed) / ((float)BGVAR(s32_V_Lim_Design));
   f_temp *= 11375.0;

   if (f_temp > 32767.0) f_temp = 32767.0;
   VAR(AX0_u16_Speed_Phase_Adv_Threshold) =  (unsigned int)f_temp;  //threshold = MSPEED/2
}


//**********************************************************
// Function Name: SalMPeakTAdvCommand
// Description:
//          This function is called in response to the MTANGLP command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMPeakTAdvCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_ControllerType) ==  VOLTAGE_CONTROL) BGVAR(s16_Mpeak_T_Adv) = 0;
   else BGVAR(s16_Mpeak_T_Adv) = (int)lparam;

   TorquePhaseAdvDesign(DRIVE_PARAM);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMContTAdvCommand
// Description:
//          This function is called in response to the MTANGLC command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMContTAdvCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_ControllerType) ==  VOLTAGE_CONTROL) BGVAR(s16_Mcont_T_Adv) = 0L;
   else BGVAR(s16_Mcont_T_Adv) = (int)lparam;

   TorquePhaseAdvDesign(DRIVE_PARAM);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMspeedSpeedAdvCommand
// Description:
//          This function is called in response to the MVANGLF command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMspeedSpeedAdvCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(BGVAR(s16_Mspeed_Speed_Advance) != (int)lparam)
   {
    BGVAR(s16_Mspeed_Speed_Advance) = (int)lparam;
    BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
   }

   SpeedPhaseAdvDesign(DRIVE_PARAM);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalHalfMspeedSpeedAdvCommand
// Description:
//          This function is called in response to the MVANGLH command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalHalfMspeedSpeedAdvCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(BGVAR(s16_Half_Mspeed_Speed_Advance) != (int)lparam)
   {
      BGVAR(s16_Half_Mspeed_Speed_Advance) = (int)lparam;
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
   }
   SpeedPhaseAdvDesign(DRIVE_PARAM);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMphaseCommand
// Description:
//          This function is called in response to the MPHASE command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************/
int SalMphaseCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   /* Convert the value to internal units by multiplying by 65536 and
   * dividing by 360 =  multiplying by 0x5B06 >> 7    */
   Math32 = ((long)lparam * 0x5B06) >> 7;

   if (BGVAR(u16_MTP_Mode))   // Dont allow to set MPHASE if MTP is used (MTPMODE > 0)
   {
      if (BGVAR(u16_Electrical_Phase_Offset_User) != (int)Math32) return (MTP_USED);
   }

   BGVAR(u16_Electrical_Phase_Offset_User) = VAR(AX0_s16_Electrical_Phase_Offset) = (int)Math32;

   return (SAL_SUCCESS);
}


int SalReadMphaseCommand(long long* param, int drive)
{
   unsigned int tmp;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   tmp = BGVAR(u16_Electrical_Phase_Offset_User);
   Math32 = (((long)tmp * 0x5A00) + 0x200000) >> 22L;

   if (Math32 >= 360) Math32 -= 360;

   *param = (long long)Math32;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: ACommand
// Description:
//          This function is called in response to the A command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int ACommand(int drive)
{
 // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   VAR(AX0_s16_Electrical_Phase_Offset) += 182;
   BGVAR(u16_Electrical_Phase_Offset_User) += 182;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: LCommand
// Description:
//          This function is called in response to the L command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int LCommand(int drive)
{
 // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   VAR(AX0_s16_Electrical_Phase_Offset) -= 182;
   BGVAR(u16_Electrical_Phase_Offset_User) -= 182;
   return (SAL_SUCCESS);
}


int SalPhaseAdvCalibrateCommand(long long param,int drive)
{
   if ((param == 1)                               &&
       (BGVAR(s16_Phase_Adv_Calibrate) != 1)        )
   {
      BGVAR(s16_Phase_Adv_Calibrate) = 1;
      BGVAR(s16_PA_Cal_State) = PA_CAL_USER_EN_STATE;
   }
   else if ((param == 0)                          &&
            (BGVAR(s16_Phase_Adv_Calibrate) == 1)   )
   {
      BGVAR(s16_PA_Cal_State) = PA_CAL_IDLE_STATE;
      BGVAR(s16_Phase_Adv_Calibrate) = 0;
      BGVAR(s16_Mspeed_Speed_Advance) = BGVAR(s16_Half_Mspeed_Speed_Advance) = 0;
      SpeedPhaseAdvDesign(drive);
   }
   return (SAL_SUCCESS);
}


