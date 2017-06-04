#include "DSP2834x_Device.h"

#include "CommFdbk.def"
#include "Current.def"
#include "Design.def"
#include "Drive_Table.def"
#include "Endat.def"
#include "Err_Hndl.def"
#include "FltCntrl.def"
#include "FPGA.def"
#include "Homing.def"
#include "i2c.def"
#include "ModCntrl.def"
#include "Modbus_Comm.def"
#include "MultiAxis.def"
#include "ResCnfg.def"
#include "Ser_Comm.def"
#include "Velocity.def"
#include "Exe_IO.def"
#include "PtpGenerator.def"
#include "BiSS_C.def"

#include "An_Fltr.var"
#include "AutoTune.var"
#include "CommFdbk.var"
#include "Drive.var"
#include "Endat.var"
#include "Exe_Hndl.var"
#include "Extrn_Asm.var"
#include "FlashHandle.var"
#include "FltCntrl.var"
#include "Foldback.var"
#include "Hiface.var"
#include "Homing.var"
#include "Init.var"
#include "ModCntrl.var"
#include "Motor.var"
#include "MotorSetup.var"
#include "Modbus_Comm.var"
#include "PhaseFind.var"
#include "Position.var"
#include "PtpGenerator.var"
#include "SensorlessBG.var"
#include "Ser_Comm.var"
#include "Units.var"
#include "User_Var.var"
#include "Velocity.var"
#include "Zeroing.var"
#include "Exe_IO.var"
#include "Display.var"
#include "Lxm_profile.var"
#include "ExFbVar.var"
#include "BiSS_C.var"

#include "MotorSetup.pro"
#include "Prototypes.pro"
#include "BiSS_C.pro"

#include "cal_conf.h"
#include "co_usr.h"


extern int Enc_Hall_Switch_Tbl_4to5to1to3to2to6;
extern int Enc_Hall_Switch_Tbl_4to6to2to3to1to5;
extern int Enc_Hall_Comm_Tbl_4to5to1to3to2to6;
extern int Enc_Hall_Comm_Tbl_4to6to2to3to1to5;

extern UNSIGNED8 lNodeId;

extern void EnablePwmW(void);
extern void DisablePwmW(void);

void CalcInternalParams(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   CalcImax(DRIVE_PARAM);
   CalcILim(DRIVE_PARAM);
   CalcFoldbackParam(DRIVE_PARAM, DRIVE_FOLDBACK);
   CalcFoldbackParam(DRIVE_PARAM, MOTOR_FOLDBACK);
   CalcFbFilterGain(DRIVE_PARAM);
   CalcSwr2dCoef(DRIVE_PARAM);
}

int DesignRoutine(unsigned int u16_feedback_design_needed, int drive)
{
   // AXIS_OFF;
   int res;

   // In case there's a change at the motorcommtype the GPIO pin functionality will be changed
   if (u16_Prev_Motor_Comm_Type != VAR(AX0_u16_Motor_Comm_Type))
   {
      if (VAR(AX0_u16_Motor_Comm_Type) != BRUSHLESS_MOTOR) DisablePwmW();
      else EnablePwmW();
   }

   u16_Prev_Motor_Comm_Type = VAR(AX0_u16_Motor_Comm_Type);

   CalcPolesRatio(DRIVE_PARAM);

   if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
   {
      res = TestLinearLimits(DRIVE_PARAM);
      if (res != SUCCESS) return (res);
   }

   MotorFeedbackConfig(DRIVE_PARAM);

   SetMotorEncIntrpolation(drive); // This is called twice to run the encoder state machine
                                   // with the correct PFB32 scaling
   SetLoadEncIntrpolation(drive); // This is called twice to run the encoder state machine
                                   // with the correct PFB32 scaling

   // Update fix-shift values for analog output scaling (Lexium) in case that
   // the motor continuous current or the maximum motor speed has changed (a config
   // command is required by the user).
   UpdateLexAnalogOutputScaling(drive);

   if (u16_feedback_design_needed)
   {
      if ( (!FEEDBACK_BISSC) && (!FEEDBACK_WITH_ENCODER) && (!SL_FEEDBACK)   &&
           (!FEEDBACK_RESOLVER) && (BGVAR(u16_FdbkType) != FORCED_COMM_FDBK) &&
           (!FEEDBACK_YASKAWA)                                                 )
            //  for Yaskawa this work is done internally by FPGA module
      { // Set Comm. Feedback Parameters only if applicable
         if (BGVAR(u16_Comm_Fdbk_Init_Ran) == 0)                 // fix to bugzilla 4597
            VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

         res = CommFdbkDefaultValues(drive, 0, 0);
         if (res != SUCCESS) return (res);
      }

      res = FeedbackConfig(DRIVE_PARAM, BGVAR(u16_Fdbk_Source));
      if (res != SUCCESS) return (res);

      res = UpdatePulseandDirectionLineBrakeFilterInFpga();
      if (res != SUCCESS) return (res);

      //  Raise the Feedback Process Active flag
      BGVAR(u16_Feedback_Config_Process_Active) = TRUE;

      if(IS_SECONDARY_FEEDBACK_ENABLED)
      {
         if ( (!SFB_BISSC) && (!SFB_WITH_ENCODER) && (!SFB_YASKAWA)                       &&
              (BGVAR(u16_SFBType) != NO_SEC_FDBK) && (BGVAR(u16_SFBType) != RESOLVER_FDBK)  )
         { // Set Comm. Feedback Parameters only if applicable

            res = CommFdbkDefaultValues(drive, 1, 1);
            if (res != SUCCESS)
               return (res);
         }
         //      BGVAR(AX0_u16_Crc_Table_Select) = 1; // Zero RT Table Selector...
         res = SFBConfig(DRIVE_PARAM, (1 - BGVAR(u16_Fdbk_Source)));
         if (res != SUCCESS) return (res);
      }
   }

   res = UpdateVelocityLimits(DRIVE_PARAM);
   if (res != SUCCESS) return (res);

   SetMotorEncIntrpolation(drive);

   SetLoadEncIntrpolation(drive);

   res = CrrntConfig(DRIVE_PARAM);
   if (res != SUCCESS) return (res);

   res = EncoderSimulationConfig(DRIVE_PARAM);   // Check if encoder simulation freq is too high
   if (res != SUCCESS) return (res);

   res = VelocityConfig(DRIVE_PARAM);  // execute velocity loop design, will not execute if no comp exists
                                      // (UpdateVelocityLimits calls it as well)

   if ((res == SUCCESS) && (BGVAR(u8_Sl_Mode) != 0))
   {
      res = CalcInternalSlpeeds(drive);
      SensorlessDesign(DRIVE_PARAM);
   }

   // Do not allow using PHASEFINDMODE = 4 and KCMODE < 6 due to potential bad behavior of the combination,
   // bug #4170
   if (((BGVAR(u16_PhaseFind_Mode)==4)||(BGVAR(u16_PhaseFind_Mode)==5)) && (VAR(AX0_S16_Kc_Mode)<6))
      res = CONFIG_FAIL_PHASEFIND4_KCMODE;

      
   //DualLoop verifications
   //this restriction is temporary removed to allow all VELCONTROLMODE values
 /*  if (IS_DUAL_LOOP_ACTIVE && (BGVAR(u8_CompMode) != 7) && (BGVAR(u8_CompMode) != 5) && (BGVAR(u8_CompMode) != 6))
      res = CONFIG_FAIL_DL_VELCONTROLMODE;*/
      
   /*********Priour Dual Feedback Verifications***************/   
   if (IS_DUAL_LOOP_ACTIVE && (VAR(AX0_u16_Pos_Control_Mode) != NON_LINEAR_POSITION_5))
      res = CONFIG_FAIL_DL_POSCONTROLMODE;
   else if (IS_DUAL_LOOP_ACTIVE && (1 == u8_EncoutMode))
      res = CONFIG_FAIL_DL_ENCOUTMODE; 
   else if (IS_SECONDARY_FEEDBACK_ENABLED && (0 == u16_SFBType))
      res = INVALID_DF_SFBTYPE;  
   /**********************************************************/
  
   if (res != SUCCESS)
   {
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
      return (res);
   }

   PositionConfig(drive,0);
   KalmanFilterDesign(drive);
   if (!BGVAR(u16_MTP_Mode)) DisableCommand(drive); // reset sw en request if exsists

   BGVAR(u8_PhaseFind_State) = PHASE_FIND_IDLE;



   // Clear the error flag when the end of the function has been reached.
   BGVAR(s64_SysNotOk) &= ~NO_COMP_FLT_MASK;
   return (SUCCESS);
}


void CalcImax(int drive)
{
   // AXIS_OFF;

   unsigned int u16_shr;
   long long s64_fix, s64_half_for_rounding;
   float f_temp = 0.0;
   REFERENCE_TO_DRIVE;
   BGVAR(s32_Imax) = BGVAR(s32_Drive_I_Peak);   //Set IMAX as min of DIPEAK and MIPEK

   if (u16_Product == SHNDR_HW)
   {  //For SE, to allow setting ILIM without motor connected.
      if ((BGVAR(s32_Motor_I_Peak) > 0) && (BGVAR(s32_Motor_I_Peak) < BGVAR(s32_Drive_I_Peak)) )
         BGVAR(s32_Imax) = BGVAR(s32_Motor_I_Peak);
   }
   else
   {
      if ((BGVAR(s32_Motor_I_Peak) < BGVAR(s32_Drive_I_Peak)))
         BGVAR(s32_Imax) = BGVAR(s32_Motor_I_Peak);
   }

   s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s64_half_for_rounding = 0LL;
   if (u16_shr > 0)
      s64_half_for_rounding = 1LL << (u16_shr - 1);

   // This is usedfor PHASEFIND as a temp sat value during the procedure (IMAX will be used instead of ILIM)
   VAR(AX0_s16_I_Imax_Sat_Hi) = (int)((BGVAR(s32_Imax) * s64_fix + s64_half_for_rounding) >> u16_shr);

   // Calculate (DIPEAK/MICONT)^2 for the VBL (NLTUNE) calculation
   if (BGVAR(s32_Motor_I_Cont) == 0)
   {
      VAR(AX0_s16_Drive_Peak_Motor_Cont_2_Fix) = 1;
      VAR(AX0_u16_Drive_Peak_Motor_Cont_2_Shr) = 0;
   }
   else
   {
      f_temp = (float)BGVAR(s32_Drive_I_Peak)/(float)BGVAR(s32_Motor_I_Cont);
      FloatToFix16Shift16(&VAR(AX0_s16_Drive_Peak_Motor_Cont_2_Fix),
                          &VAR(AX0_u16_Drive_Peak_Motor_Cont_2_Shr),
                          f_temp*f_temp);
   }
}


void CalcILim(int drive)
{
   // AXIS_OFF;

   //  Make sure that I_Lim is not greater than I_Max.
   if (BGVAR(s32_Ilim_User) > BGVAR(s32_Imax)) BGVAR(s32_Ilim_User) = BGVAR(s32_Imax);

   //  Make sure that I_Lim_Act is not greater than I_Max.
   if (BGVAR(s32_Ilim_Actual) > BGVAR(s32_Imax)) BGVAR(s32_Ilim_Actual) = BGVAR(s32_Imax);

   // Create Ilim_User + 20% of DIPEAK in internal units for future using in RT
   // value = (Ilim_User + dipeak * 0.2 )* 26214 / dipeak
   VAR(AX0_s16_120P_Ilim_User) = (int)((BGVAR(s32_Ilim_User) + (long)((float)BGVAR(s32_Drive_I_Peak) * 0.2)) * 26214.0/ (float)BGVAR(s32_Drive_I_Peak));

   if(u16_Init_State == 1) // execute only in init state
   {
      CalcFoldbackParam(DRIVE_PARAM, DRIVE_FOLDBACK);
      CalcFoldbackParam(DRIVE_PARAM, MOTOR_FOLDBACK);
   }
}


void ResetAllParam(int drive)
{
   // AXIS_OFF;
   int s16_i, s16_j;

   DEFAULT_MOTOR_VALUES;

   if (VAR(AX0_u16_Pos_Control_Mode) >= NON_LINEAR_POSITION_5)
   {
      VAR(AX0_u16_Pos_Loop_Tsp) = TSP_125_USEC;
      BGVAR(u64_CAN_Max_Acc) = MAX_ACC_DEC_TSP125;
      BGVAR(u64_CAN_Max_Dec) = MAX_ACC_DEC_TSP125;
   }
   else
      VAR(AX0_u16_Pos_Loop_Tsp) = TSP_250_USEC;

   CalcImax(DRIVE_PARAM);

   // Reset index found
   VAR(AX0_s16_Crrnt_Run_Code) &= ~(LOOK_FOR_INDEX_MASK | INDEX_CAPTURE_DONE_MASK);
   VAR(AX0_s16_Index_Elect_Pos) = 0;

   // Above this line it was ResetMotorParam()
   // ===================================================
   // Below this line it was ResetOperatinalParam()

   ResetCommandsAndStates(DRIVE_PARAM);

   // This is a unique CDHD init, as these are read-only parameters for the CDHD serial, hence not initialized by the "CDHD -Commands.xls" macro
   if (!((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)))
   {
      BGVAR(u16_Icmd_Slope_Use) = 0;
      BGVAR(u32_Icmd_Slp) = 100000L;
   }
   DEFAULT_OPR_VALUES;
   // Change ENCOUTMODE to 0 for drives not supporting EEO
   if (!IS_HW_FUNC_ENABLED(EEO_MASK))
      BGVAR(u8_EncoutMode) = 0;

   CanCopyFlashImgToPortLib();

   DualLoopInit(drive);
   ConversionAninToUserUnitsPreperation(drive);

   BGVAR(s32_PhaseFind_Current) = BGVAR(s32_Drive_I_Cont) / 10;
   BGVAR(u16_UV_Threshold) = BGVAR(u16_UV_Threshold_Default);

   if (u16_Product == DDHD)
   {
      VAR(AX0_u8_Gear_Mode) = 0;   // CDHD default for GEARMODE is 3, but this mode is not supported by DDHD HW
   }

   BGVAR(s16_Regen_Resistor_Resistance) = BGVAR(s16_Internal_Regen_Res_Ohm);
   BGVAR(s16_Regen_Resistor_Power) = BGVAR(s16_Internal_Regen_Power_Watt);
   RecalcMaxRegenOnTime(drive);

   BGVAR(s32_V_Lim_Design) = 8947849L; // = 1000 RPM - /**********SHOULD BE 10 RPM - CHANGED TEMPORARY TO AVOID CURRENT LOOP ISSUE ********/
//   BGVAR(u32_Home_Switch_Speed) = 894784L; // Default for HOMESPEED1 100 RPM
//   BGVAR(u32_Home_Zero_Speed) = 178956L; // Default for HOMESPEED2 20 RPM
   BGVAR(s32_Ilim_User) = BGVAR(s32_Imax);

   SetOpmode(drive, 2);

   SalPfbOffCommand(0LL, drive); // set PFBOffset to Zero, cannot assign Default
                                 // Value to these two Real-Time Variables.

//   SalSfbOffCommand(0LL, drive); // set PFBOffset to Zero, cannot assign Default
                                 // Value to these two Real-Time Variables.

   SalMotorAccCommand(0xAEC33E1F670D, drive); // Default values for ACC, DEC, and DECSTOP
   SalMotorDecCommand(0xAEC33E1F670D, drive);
   SalSfbAccCommand(0xAEC33E1F670D, drive); // Default values for ACC, DEC, and DECSTOP
   SalSfbDecCommand(0xAEC33E1F670D, drive);
   SalHomeAccCommand(0x45E7B272F605, drive); // Default for HOMEACC 4,000 RPM/Sec
   SalDecStopCommand(0x1B4E81B4E81B4, drive); // Default for DECSTOP 100,000 RPM/Sec
   SalDecStopTimeCommand(0LL, drive);
   SalGearAccTreshCommand(0x0011111111111111, drive);


   VAR(AX0_s16_Icmd_Harmonic_Phase_1) = VAR(AX0_s16_Icmd_Harmonic_Num_1) = VAR(AX0_s16_Icmd_Harmonic_Amp_1) = 0;
   VAR(AX0_s16_Icmd_Harmonic_Phase_2) = VAR(AX0_s16_Icmd_Harmonic_Num_2) = VAR(AX0_s16_Icmd_Harmonic_Amp_2) = 0;

   for (s16_i = 0; s16_i < 12; s16_i++) BGVAR(s16_Vh)[s16_i] = 0;
   for (s16_i = 0; s16_i < 7; s16_i++) BGVAR(s16_Vf)[s16_i] = 0;
   BGVAR(s16_Vf)[B0] = 1;
   for (s16_i = 0; s16_i < 7; s16_i++) BGVAR(s16_Vfi)[s16_i] = 0;
   BGVAR(s16_Vfi)[B0] = 1;
   for (s16_i = 0; s16_i < 10; s16_i++) BGVAR(s16_Vr)[s16_i] = 0;
   for (s16_i = 0; s16_i < 8; s16_i++) BGVAR(s16_Vd)[s16_i] = 0;

   for (s16_j = 0; s16_j < 32; s16_j++)
   {
      BGVAR(u16_Script)[s16_j * 128] = ('~' | ('~' << 8));
      BGVAR(u16_Script)[s16_j * 128 + 1] = 0;
   }

   //SalLimdisCommand(0L, drive);

   AnalogInputConfig1(drive);   // reset analog inputs variables
   AnalogInputConfig2(drive);
   AnalogTorqueConfig1(drive);
   AnalogTorqueConfig2(drive);
   AnalogVelocityConfig(drive);

   SalOutOfRangeCommand((long long)BGVAR(u16_Out_of_Range_Percent), drive);

   PtpFilterDesign(drive);

   if ((BGVAR(u16_MotorType) == LINEAR_MOTOR) || (LOAD_TYPE == LINEAR_MOTOR))// linear motor/load
   {
      SalUnitsVelLinearCommand((long long)BGVAR(u16_Units_Vel_Linear), drive);
      SalUnitsAccDecLinearCommand((long long)BGVAR(u16_Units_Acc_Dec_Linear), drive);
      SalUnitsPosLinearCommand((long long)BGVAR(u16_Units_Pos_Linear), drive);
   }
   else// rotary motor or any other motor
   {
      SalUnitsVelRotaryCommand((long long)BGVAR(u16_Units_Vel_Rotary), drive);
      SalUnitsAccDecRotaryCommand((long long)BGVAR(u16_Units_Acc_Dec_Rotary), drive);
      SalUnitsPosRotaryCommand((long long)BGVAR(u16_Units_Pos_Rotary), drive);
   }

//   BGVAR(u16_Modulo_Mode) = 0;
   BGVAR(s64_Position_Modulo)[0] = 0LL;
   BGVAR(s64_Position_Modulo)[1] = 0LL;
   PositionModuloConfig(drive);

   ConvertAmpToInternal(drive);
   ConvertInternalToAmp1000(drive);
   ConvertAnalogIOToInternal(drive);
   ConvertInternalToAnalogIO1000(drive);
   ConvertInternalPWMCmdToVolts1000(drive);

   UnitsVelRotLinRPMCommand(drive);
   UnitsVelRotLinRPSCommand(drive);
   UnitsPosRotLinCommand(drive);
   UnitsRotLinKPPCommand(drive);
   UnitsRotLinENCRESCommand(drive);
   UnitsAmpVelRotLinCommand(drive);
   UnitsMechAngleRotLinCommand(drive);

   BGVAR(s32_Current_Scale_In) = 263; // 0.01 of dipeak
   BGVAR(u32_Current_Scale_2)  = 263; // 0.01 of dipeak
   BGVAR(s32_Velocity_Scale_In) = 537;
   BGVAR(s32_Current_Scale_Out) = 263; // 0.01 of dipeak

   BGVAR(u32_I_Fold_Fault_Threshold) = (unsigned long)(((long long)BGVAR(s32_Drive_I_Cont)*26214) / BGVAR(s32_Drive_I_Peak));
   BGVAR(u32_I_Fold_Warning_Threshold) = 26214L;
   BGVAR(u32_Motor_I_Fold_Fault_Threshold) = 0L;
   BGVAR(u32_Motor_I_Fold_Warning_Threshold) = 0L;

   BGVAR(u32_Stop_Current) = BGVAR(s32_Drive_I_Peak);

   InitDigIOs(drive); // Init digital IOs states

   // Init probe filter defaults
   STORE_EXECUTION_PARAMS_0_1
   s64_Execution_Parameter[0] = 1; // TP 1
   s64_Execution_Parameter[1] = 5; // Filter depth 5
   SalProbeLevelFltWriteCommand(drive);
   s64_Execution_Parameter[0] = 2; // TP 2 same default filter depth as TP 1
   //s64_Execution_Parameter[1] = 5;
   SalProbeLevelFltWriteCommand(drive);
   RESTORE_EXECUTION_PARAMS_0_1


   //set default commode according EEPROM CANOpen/EtherCAT assembly bit
   if(IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK) || IS_HW_FUNC_ENABLED(ETHERNET_BASED_FIELDBUS_MASK))
      BGVAR(u8_Comm_Mode) = 1;

   s16_Fieldbus_Flags |= (IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK) | IS_HW_FUNC_ENABLED(ETHERNET_BASED_FIELDBUS_MASK));

   BGVAR(s32_P_Minus_Init)[0] = 1000;
   BGVAR(s32_P_Minus_Init)[1] = 1000;
   BGVAR(s32_P_Minus_Init)[2] = 100000;
   BGVAR(s32_P_Minus_Init)[3] = 10000;
   BGVAR(s32_Sl_EKF_Q)[0] = 900000000;
   BGVAR(s32_Sl_EKF_Q)[1] = 900000000;
   BGVAR(s32_Sl_EKF_Q)[2] = 3.9e6 * 1000e-6;//1000e-6=tkalman
   BGVAR(s32_Sl_EKF_Q)[3] = 150;

   BGVAR(s32_Sl_EKF_R)[0] = 10;
   BGVAR(s32_Sl_EKF_R)[1] = 10;

   //SalProbe1LvlPrdWriteCommand((long long)5, drive);
   //SalProbe2LvlPrdWriteCommand((long long)5, drive);

//   BGVAR(u32_Runaway_Check_Vel_Thresh) = 536870; // 60[rpm] = 1[rps] = 536870[Counts32/125us]

   // if there any abnormal value reset all the paths data to defaults it's still not in the Schneider's user units
   // for now we are using the CDHD's user units <-> internal units.
   for (s16_i = 0; s16_i < NUM_OF_PATHS; s16_i++)
   {
      (BGVAR(s64_Path_Position)[s16_i]) = 0;

      (BGVAR(u64_Path_Acceleration)[s16_i]) = 0xA7C5AC471B4; // 600 rpm/sec internal units
      (BGVAR(u64_Path_Deceleration)[s16_i]) = 0xA7C5AC471B4; // 600 rpm/sec internal units
      (BGVAR(u32_Path_Speed)[s16_i]) = 0x1F0FB3; // 200 rpm

      (BGVAR(u16_Path_Delay)[s16_i]) = 0;
      (BGVAR(u16_Path_Control)[s16_i]) = 0;
   }

   BGVAR(s16_Sl_States)[0] = BGVAR(s16_Sl_States)[1] = BGVAR(s16_Sl_States)[2] = 1;
   BGVAR(s16_Sl_Speeds)[0] = BGVAR(s16_Sl_Speeds)[1] = BGVAR(s16_Sl_Speeds)[2] = BGVAR(s16_Sl_Speeds)[3] = 0;
   if(BGVAR(u16_Vbus_Scale) == 0x0200)// VScale = (2*256)=512 -> VBus = 540 V
   {// HIGH VOLTAGE BOARD
      BGVAR(s16_Vbus) = 540;
   }
   else // VBus = 320 V
   {// MEDIUM VOLTAGE BOARD
      BGVAR(s16_Vbus) = 320;
   }
//   for (s16_i = 0; s16_i < NUM_OF_GT; s16_i++) KnlGTBackup(drive, s16_i);
   // init controller varaibles to user variables defaults - gain table support
   BGVAR(u32_Nl_Kpd) = BGVAR(u32_Nl_Kpd_User);
   BGVAR(u32_Nl_Kpi) = BGVAR(u32_Nl_Kpi_User);
   BGVAR(u32_Nl_Kpiv) = BGVAR(u32_Nl_Kpiv_User);
   BGVAR(u32_Nl_Kpp) = BGVAR(u32_Nl_Kpp_User);
   BGVAR(u32_Nl_Kpgf) = BGVAR(u32_Nl_Kpgf_User);
   BGVAR(s16_Nl_Out_Filter_1) = BGVAR(s16_Nl_Out_Filter_1_User);
   BGVAR(s16_Nl_Out_Filter_2) = BGVAR(s16_Nl_Out_Filter_2_User);
   BGVAR(u32_Nl_K_Anti_Vibration) = BGVAR(u32_Nl_K_Anti_Vibration_User);
   BGVAR(s32_Pe_Filt_Gain) = BGVAR(s32_Pe_Filt_Gain_User);
   BGVAR(u32_Nl_K_Anti_Resonance_Fcenter) = BGVAR(u32_Nl_K_Anti_Resonance_Fcenter_User);
   BGVAR(u32_Pe_Filt_Fcenter) = BGVAR(u32_Pe_Filt_Fcenter_User);
   BGVAR(u16_Nl_K_Anti_Resonance_Sharpness) = BGVAR(u16_Nl_K_Anti_Resonance_Sharp_User);
   BGVAR(u16_Pe_Filt_Sharpness) = BGVAR(u16_Pe_Filt_Sharpness_User);
   BGVAR(s16_Kbff_Spring_LPF) = BGVAR(s16_Kbff_Spring_LPF_User);
   BGVAR(u32_Nl_KffSpring) = BGVAR(u32_Nl_KffSpring_User);
   BGVAR(u32_LMJR) = BGVAR(u32_LMJR_User);

   UpdateEventDecelerations(drive);// set deceleration events default

   // Restore the baud-rate after restoring the NV variable for the baud-rate setting. This call
   // is needed since CPU registers have to be set in case that the baud-rate variable has changed.
   SalSerialBaudRateCommand((long long)BGVAR(u32_Serial_Baud_Rate), drive);

   // load default values from MTP.
   BGVAR(s16_Load_Defaults_From_MTP) = 1;

   u16_Reset_All_Param_Executed = 1;
}


void ResetCommandsAndStates(int drive)
{
   // AXIS_OFF;

   DisableCommand(drive);
   BGVAR(s16_DisableInputs) |= HW_EN_MASK;
   BGVAR(s64_SysNotOk) = 0;
   BGVAR(s64_SysNotOk_2) = 0;

   BGVAR(u16_Sto_Indication_Filter) = 0;
   BGVAR(s16_UvFilter) = 0;
   s16_OvFilter = 0;
   s16_OtFilter = 0;
   s16_Plus15vFilter = 0;
   s16_Minus15vFilter = 0;
   u16_VbusMeasureFilter = 0;
   u16_DriveDigOTFilter = 0;
   u16_DrivePowOTFilter = 0;
   u16_DriveIPMOTFilter = 0;
   u16_UnderVoltageFilter = 0;
   u16_last_state_sto_fault = 0;
   BGVAR(s16_VospdFilter) = 0;
   BGVAR(u16_ABFilter) = 0;
   BGVAR(u16_OutOfRangeFilter) = 0;
   BGVAR(s16_Index_Filter) = 0;
   BGVAR(u16_Motor_Ot_Filter) = 0;
   BGVAR(u16_HallsFilter) = 0;
   BGVAR(u16_Diff_Halls_Filter) = 0;
   BGVAR(u16_IndexFilter) = 0;
   BGVAR(u16_Encoder5Vfilter) = 0;
   BGVAR(u16_SecEncoder5Vfilter) = 0;
   BGVAR(u16_SecIndexFilter) = 0;
   BGVAR(u16_SecLineFilter) = 0;
   BGVAR(u16_PD_LineFilter) = 0;
   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

   // init sine gain to unity
   VAR(AX0_s16_Sine_Offset) = 0;
   VAR(AX0_s16_Cosine_Offset) = 0;
   VAR(AX0_s16_Sine_Gain_Fix) = 0x4000;
   VAR(AX0_s16_Sine_Gain_Shr) = 14;
   VAR(AX0_s16_Swr2d_Fix) = 0x4000;
   VAR(AX0_s16_Swr2d_Shr) = 14;
   BGVAR(s8_BurninParam) = 0;

   // Initialize Communication options - this line causes wd if eeprom chksum happens
   // BGVAR(u8_Comms_Options) = (ECHO_CONTROL_MASK | PROMPT_MASK | ERR_MSG_MASK);

   VAR(AX0_s16_Serial_Vel_Cmnd) = 0;
   VAR(AX0_s16_Serial_Crrnt_Cmnd) = 0;
}


int TestLinearLimits(int drive)
{
/* int i;

   // test u16_Mspeed range
   i = ConvertToRotary(drive,BGVAR(u16_Mspeed));
   if ((i < MSPEED_MIN_RANGE) || (i > MSPEED_MAX_RANGE)) return (CONFIG_FAIL_MSPEED);

   // test Mvanglf range
   i = ConvertToRotary(drive,BGVAR(s16_Mspeed_Speed_Advance));
   if ((i < SPEED_ADV_MIN_RANGE) || (i > SPEED_ADV_MAX_RANGE)) return (CONFIG_FAIL_MVANGLF);

   // test Mvanglh range
   i = ConvertToRotary(drive,BGVAR(s16_Half_Mspeed_Speed_Advance));
   if ((i < SPEED_ADV_MIN_RANGE) || (i > SPEED_ADV_MAX_RANGE)) return (CONFIG_FAIL_MVANGLH);

   // calc vmax and test vlim is in range
   i = BGVAR(s32_V_Lim_Design); // keep value before callling vmax
   UpdateVelocityLimits(DRIVE_PARAM);
   if ( (i!= BGVAR(s32_V_Lim_Design))                   ||
      (ConvertToRotary(drive,BGVAR(s32_V_Lim_Design)) < 10)   ) return (CONFIG_FAIL_VLIM);
*/
   drive++;   // just to defeat the compilation remark
   return (SUCCESS);
}


void CalcSwr2dCoef(int drive)
{
   /*
   *  Data was taken from "swr2d.xls" spreadsheet.
   *
   *  When the algorithm is executed at 125uS rate, use:
   *  AX0_s16_Swr2d_W_Sqr = 0.025735927 * u16_ResBw^2
   *  AX0_s16_Swr2d_Bw_Inv_W = 57041.1316 / u16_ResBw
   *  (AX0_s16_Swr2d_Bw_Inv_W includes extra 5 shl, that will be compensated in real-time)
   *
   *  When the algorithm is executed at 62.5uS rate, use:
   *  AX0_s16_Swr2d_W_Sqr = 0.006433982 * u16_ResBw^2
   *  AX0_s16_Swr2d_Bw_Inv_W = 114082.2632 / u16_ResBw
   *  (AX0_s16_Swr2d_Bw_Inv_W includes extra 5 shl, that will be compensated in real-time)
   *
   *  0.025735927 = 0x696A >> 20
   *  57041.1316 = 0x6F6890D8 >> 15
   *
   *  0.006433982 = 0x34B5 >> 21
   *  114082.2632 = 0xDED121B0 >> 15
   */
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
//   VAR(AX0_s16_Swr2d_W_Sqr) = (int)(((float)BGVAR(u16_ResBw)) * ((float)BGVAR(u16_ResBw)) * 0.025735927);
   VAR(AX0_s16_Swr2d_W_Sqr) = (int)(((float)BGVAR(u16_ResBw)) * ((float)BGVAR(u16_ResBw)) * 0.006433982);

//   VAR(AX0_s16_Swr2d_Bw_Inv_W) = (int)(57041.131591796875 / ((float)BGVAR(u16_ResBw)));
   VAR(AX0_s16_Swr2d_Bw_Inv_W) = (int)(114082.2632 / ((float)BGVAR(u16_ResBw)));
}


int SalFdbkBrateCommand(long long lparam, int drive)
{
   // AXIS_OFF;                 // fix to bugzilla 4597

   REFERENCE_TO_DRIVE;
   if (BGVAR(u16_Fdbk_Brate_Khz) == (unsigned int)lparam)
   {
      return SAL_SUCCESS;
   }
   else
   {
    //   return NOT_AVAILABLE; //require special FPGA version , uncomment 3 below lines
         BGVAR(u16_Fdbk_Brate_Khz) = (unsigned int)lparam;
         VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK; // fix to bugzilla 4597
         BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
      return SAL_SUCCESS;
   }
}


int CommFdbkDefaultValues(int drive, int enc_source, int fdbk_dest)
{
   unsigned int i = 0;

   // AXIS_OFF;
   FDBK_OFF;
   REFERENCE_TO_DRIVE;
   do {
      if ( ( (enc_source == 0) && (Comm_Parameters[i].fdbk_type == BGVAR(u16_FdbkType)) ) ||
           ( (enc_source == 1) && (Comm_Parameters[i].fdbk_type == BGVAR(u16_SFBType)) )    )
      {
         if (BGVAR(u16_Fdbk_Brate_Khz) == 0) // Generate Comm. Bit Interval from Default Baud-Rate
            BGVAR(u16_Fdbk_Clk_Interval) = (int)((float)60000 / (float)Comm_Parameters[i].dflt_baud_rate_khz + 0.5) - 1;
         else                                // Generate Comm. Bit Interval from User Baud-Rate
            BGVAR(u16_Fdbk_Clk_Interval) = (int)((float)60000 / (float)BGVAR(u16_Fdbk_Brate_Khz) + 0.5) - 1;
         BGVAR(u16_Xmt_Length) = Comm_Parameters[i].dflt_xmt_length;
         BGVAR(u16_Rcv_Length) = Comm_Parameters[i].dflt_rcv_length;
         BGVAR(u16_Wait_Time_Clks) = Comm_Parameters[i].dflt_wait_time_clks;
         BGVAR(u16_Field_Length) = Comm_Parameters[i].dflt_field_length;
         FDBKVAR(VAR(AX0_u16_Comm_Fdbk_Flags_A_1)) = (FDBKVAR(VAR(AX0_u16_Comm_Fdbk_Flags_A_1)) & 0xFFF8) | Comm_Parameters[i].deflt_sync_symbol_bits;
         // Set Sync Symbol in lower three Bits of AX0_u16_Comm_Fdbk_Flags_A_1.
         return SAL_SUCCESS;
      }
      else
         i++;
   } while (i < NUMBER_OF_FDBK_TYPES);

   return COMM_FDBK_DFLT_UNDEFINED;
}


//**********************************************************
// Function Name: SalFdbkCommand
// Description:
//          This function is called in response to the FEEDBACKTYPE command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalFdbkCommand(long long lparam, int drive)
{
//   int s16_original_fdbktype = BGVAR(u16_FdbkType); //, return_value = SAL_SUCCESS;
//   // AXIS_OFF;

   //if (BGVAR(u16_FdbkType) == (int)lparam) return (SAL_SUCCESS);

   if ( ( (u16_Product == SHNDR) || (u16_Product == SHNDR_HW) ) &&
        ((int)lparam == AUTO_DETECTION_FDBK)                      )  return NOT_AVAILABLE;

   if ( (u16_Product == DDHD) || (u16_Product == GBOT10) || (u16_Product == SHNDR_HW) )
   {  // These products do not support feedback with analog feedback
      if (((int)lparam == RESOLVER_FDBK) || ((int)lparam == SW_SINE_FDBK)) return NOT_AVAILABLE;
   }

   // Yaskawa/Biss_C is supported for EC version only as this is the only the FPGA accomodates the library
   if (!IS_EC_DRIVE)
   {
      if ( ((int)lparam == YASKAWA_ABS_COMM_FDBK) ||
           ((int)lparam == YASKAWA_INC_COMM_FDBK)   )
         return NOT_SUPPORTED_ON_HW;
   }

   if ((int)lparam == SL_FDBK) return NOT_AVAILABLE;   // Sensorless is not supported

   BGVAR(u16_FdbkType_User) = (unsigned int)lparam;

   if (BGVAR(u16_FdbkType_User) != AUTO_DETECTION_FDBK)
      return (FdbkTypeCommand(BGVAR(u16_FdbkType_User), drive));
   else
      return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalSFBTypeCommand
// Description:
//          This function is called in response to the SFBTYPE command.
//
//
// Author: A.H.
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBTypeCommand(long long lparam, int drive)
{
   int ret_val = SAL_SUCCESS;
   // AXIS_OFF;

   if ((int)lparam == SL_FDBK) return NOT_AVAILABLE;   // Sensorless is not supported

   if (BGVAR(u16_SFBType_User) == (unsigned int)lparam) return SAL_SUCCESS;
   //BGVAR(u16_SFBType_User) = (unsigned int)lparam;

   ret_val = SFBTypeCommand(((unsigned int)lparam & 0xFFFF), drive);

   if (SAL_SUCCESS == ret_val)
   {
      SfbUnitsStringsUpdate();
      SalUnitsPosLinearCommand((long long)u16_Units_Pos_Linear, drive);//update sfb units string
      if (IS_DUAL_LOOP_ACTIVE)//only if the DF is active:
      {
         VAR(AX0_u16_Home_Ind) = 0; //reset 'homed' flag.
         LLVAR(AX0_u32_Home_Offset_Lo) = 0;//reset calculated Home Offset
      }
   }

   return ret_val;
}


int PriorFeedbackConfig (int enc_source, int drive)
{
   int u16_offset = 0, u16_fdbk_type = 0;
   AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (!enc_source) //MFB
   {
      u16_fdbk_type = BGVAR(u16_FdbkType);

      // This Involves a new HW design in CDHD2, and is required to avoid false Line-Break indication
      //performed only for MFB
      if (RESOLVER_FDBK == u16_fdbk_type)
         *(int *)(FPGA_RESOLVER_SHORT_REG_ADD) = 0x0000;
      else
         *(int *)(FPGA_RESOLVER_SHORT_REG_ADD) = 0x0001;
   }
   else //SFB
   {
      u16_fdbk_type = BGVAR(u16_SFBType);
   }

   /* This Involves a new FPGA design for DF. The 'idle' buffer should be initialized according to the feedbacktype.
      Previously, 2 dedicated buffers where held for BISS-C and Endat Comm-only feedbacks. This is changed because DF
      feature (now each of these 2 buffers is dedicated to Motor/Load, and thus we need to initialize the 'idle' properly)  */
   if ( (ENDAT2X_COMM_FDBK == u16_fdbk_type) || (BISSC_COMM_FDBK   == u16_fdbk_type)                ||
        ((0 == enc_source) && (SW_SINE_FDBK == u16_fdbk_type) && (9 == VAR(AX0_s16_Motor_Enc_Type)))  )
   //Clear 'Idle' registers if either: 1. Endat comm-only Feedback. 2. BISS-C Feedback. 3. Endat with sine-signals via C4
   {
      for (u16_offset = 0; u16_offset < 64; u16_offset++)
         *(int *)(FPGA_MFB_TX_BUFFER_ADD + (enc_source * 0x80) + u16_offset) = 0x0000;
   }
   //Set 'Idle' registers for all other options
   else
   {
      for (u16_offset = 0; u16_offset < 64; u16_offset++)
         *(int *)(FPGA_MFB_TX_BUFFER_ADD + (enc_source * 0x80) + u16_offset) = 0xFFFF;
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: FdbkTypeCommand
// Description:
//          This function is called by the SalFdbkCommand(), and performs the actual Parameter Setting
//          and Status Manipulation; this separation is required to allow manipulation of Feedback Type
//          internally without affecting the User Setting.
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int FdbkTypeCommand(unsigned int lparam, int drive)
{
   int s16_original_fdbktype = BGVAR(u16_FdbkType); //, return_value = SAL_SUCCESS;
   // AXIS_OFF;

   BGVAR(u16_FdbkType) = (int)lparam;
   VAR(AX0_u16_Abs_Fdbk_Device) &= ~0x0001; // Use Bit 0 for Main Feedback, Bit 1 for
                                            // Secondary Feedback Source.

   // set dummy out of range limits to avoid out of range while parameter download
   // actual value is set after config

   // Disable RT communication, clear all bits except for Sync Symbol
   VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= 0x00007;
   VAR(AX0_s16_Out_Of_Range_High) = 0x7FFF;
   VAR(AX0_s16_Out_Of_Range_Low) = 0;

   if(!FEEDBACK_SERVOSENSE)
      BGVAR(u16_Mencres_Shr) = 0;                 // not relevant for anything else than SRVSNS

   // Init the delay from MTS that FPGA will send the pos read command to communication feedback to 0 (normal condition)
   *((int*)FPGA_TRANSMIT_ENABLE_COUNTER_REG_ADD) = 0;

//         *(int *)FPGA_YASKAWA_EEOI_EN_REG_ADD = 0;

/*   if ( !FEEDBACK_WITH_ENCODER && !SL_FEEDBACK && !FEEDBACK_RESOLVER &&
        (BGVAR(u16_FdbkType) != FORCED_COMM_FDBK) && (!FEEDBACK_YASKAWA))
   { // Set Comm. Feedback Parameters only if applicable, for Yaskawa this work is done internally by FPGA module
      return_value = CommFdbkDefaultValues(drive);
      if (return_value != SAL_SUCCESS)  return return_value;
   } */

   // Initialize the feedback type mismatch warning
   BGVAR(u64_Sys_Warnings) &= ~FDBK_TYPE_MISMATCH_WRN_MASK;

   PriorFeedbackConfig(0,drive);

   switch (BGVAR(u16_FdbkType))
   {
      case RESOLVER_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &SW_RESOLVER_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
         DetectFeedbackFaults(drive, RESOLVER_FAULTS_MASK, 0LL);
         BGVAR(s16_Adjust_Reference_State) = ADJ_REF_INIT; // Restart Resolver reference algorithm
      break;

      case INC_ENC_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &INC_ENCODER_FEEDBACK;
         DetectFeedbackFaults(drive, INC_ENC_FAULTS_MASK, 0LL);
         if (u16_Product == SHNDR_HW) BGVAR(s64_Faults_Mask) &= ~LINE_BRK_FLT_MASK;
         VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

         if (VAR(AX0_s16_Motor_Enc_Type) == 5)
         {
            BGVAR(s64_Faults_Mask) |= (ILLEGAL_HALLS_MASK | MOTOR_OT_FLT_MASK);
            BGVAR(s64_Faults_Mask_2) |= DIFF_HALLS_LINE_BRK_MASK;
            LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Halls_Comm_Pos);
         }
      break;

      case SW_SINE_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &SW_SINE_ENC_FEEDBACK;
         DetectFeedbackFaults(drive, SW_SINE_ENC_FAULTS_MASK, 0LL);
         VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

         if (VAR(AX0_s16_Motor_Enc_Type) == 9)
            BGVAR(s64_Faults_Mask) |= ENDAT2X_FDBK_FAULTS_MASK;
      break;

      case NK_COMM_FDBK: //Communication (Nikon)
         LVAR(AX0_s32_Feedback_Ptr) = &NK_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
         DetectFeedbackFaults(drive, NK_COMM_FDBK_FAULTS_MASK, 0LL);
         BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = (unsigned int)NIKON_ENCDR_MAX_NUM_TURNS;
         VAR(AX0_u16_Abs_Fdbk_Device) |= 0x01;
      break;

/*      case SL_FDBK: // Sensorless
         LVAR(AX0_s32_Feedback_Ptr) = &SENSORLESS_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
         DetectFeedbackFaults(drive, 0x00LL, 0LL); // Do not detect feedback faults
      break; */

      case TAMAGAWA_COMM_MULTI_TURN_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &TM_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
         DetectFeedbackFaults(drive, TM_MULT_COMM_FDBK_FAULTS_MASK, 0LL);
         // If TM Battery-Fault indication then Absolute Position not valid...
         VAR(AX0_u16_Abs_Fdbk_Device) |= (!(VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x04000));
      break;

      case TAMAGAWA_COMM_SINGLE_TURN_FDBK:
      case TAMAGAWA_CID0_SINGLE_TURN:
         LVAR(AX0_s32_Feedback_Ptr) = &TM_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
         DetectFeedbackFaults(drive, TM_SINGLE_COMM_FDBK_FAULTS_MASK, 0LL);
      break;

      case PS_P_G_COMM_FDBK: // Communication Panasonic Incremental
         LVAR(AX0_s32_Feedback_Ptr) = &PS_P_G_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;         // Hall-Effect Initialization.
         DetectFeedbackFaults(drive, PS_P_G_COMM_FDBK_FAULTS_MASK, 0LL);
      break;

      case FANUC_COMM_FDBK: // Communication (Fanuc) Multi-Turn
         LVAR(AX0_s32_Feedback_Ptr) = &FANUC_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&PHASE_FIND_STATE_0 & 0xffff); // Initialize
         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;  // Commutation in case Absolute
         DetectFeedbackFaults(drive, FANUC_COMM_FDBK_FAULTS_MASK, 0LL);
         // If FA Battery-Fault indication then Absolute Position not valid...
         VAR(AX0_u16_Abs_Fdbk_Device) |= (!(VAR(AX0_u16_Abs_Enc_Data_0Frame) & 0x0020));
      break;

      case ENDAT2X_COMM_FDBK: // Communication (EnDat) Multi-Turn
         LVAR(AX0_s32_Feedback_Ptr) = &ENDAT2X_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
         DetectFeedbackFaults(drive, ENDAT2X_COMM_FDBK_FAULTS_MASK, ENDAT2X_COMM_FDBK_FAULTS_2_MASK);
      break;

      case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
      case SERVOSENSE_MULTI_TURN_COMM_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &SRVSNS_COMMUNICATION_FEEDBACK;
         BGVAR(u16_Mencres_Shr) = 8;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
         DetectFeedbackFaults(drive, SRVSNS_SINGLE_COMM_FDBK_FAULTS_MASK_1, SRVSNS_SINGLE_COMM_FDBK_FAULTS_MASK_2);
         // Set the delay from MTS that FPGA will send the pos read command to ServoSense
         *((int*)FPGA_TRANSMIT_ENABLE_COUNTER_REG_ADD) = 510;   // 8.5 us @ 60 MHz (8.5 us is based on actual time measurement on EtherCAT drive plus 2 us margin)
      break;

      case PS_S_COMM_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &PS_S_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
         DetectFeedbackFaults(drive, PS_S_COMM_FDBK_FAULTS_MASK, PS_S_COMM_FDBK_FAULTS_2_MASK);
         // If PS A5 Battery-Fault indication then Absolute Position not valid...
         VAR(AX0_u16_Abs_Fdbk_Device) |= ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x04000) == 0);
      break;

      case SANKYO_COMM_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &SK_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
         DetectFeedbackFaults(drive, SK_MULT_COMM_FDBK_FAULTS_MASK_1, SK_MULT_COMM_FDBK_FAULTS_MASK_2);
         // If TM Battery-Fault indication then Absolute Position not valid...
         VAR(AX0_u16_Abs_Fdbk_Device) |= (!(VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x04000));
      break;

      case YASKAWA_ABS_COMM_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &YS_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;         // Hall-Effect Initialization.
         DetectFeedbackFaults(drive, FANUC_COMM_FDBK_FAULTS_MASK, 0LL);
         VAR(AX0_u16_Abs_Fdbk_Device) |= 0x01;
      break;

      case BISSC_COMM_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &BISSC_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
         DetectFeedbackFaults(drive, COMM_FDBK_FLT_MASK, 0LL);
      break;

      case YASKAWA_INC_COMM_FDBK:
         LVAR(AX0_s32_Feedback_Ptr) = &YS_COMMUNICATION_FEEDBACK;
         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;         // Hall-Effect Initialization.
         DetectFeedbackFaults(drive, FANUC_COMM_FDBK_FAULTS_MASK, 0LL);
      break;

      default:
         DetectFeedbackFaults(drive, 0x00LL, 0LL); // Do not detect feedback faults
         LVAR(AX0_s32_Feedback_Ptr) = &NO_FEEDBACK;
      break;
   }

   if(s16_original_fdbktype != BGVAR(u16_FdbkType))
   {
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
      VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;
   }

   // This updates the operational homing offset with the user non-volatile variable
   // according to the feedback multi turn absolute
   UpdateRefOffsetValue(drive);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SFBTypeCommand
// Description:
//   This function is called by the SalSFBTypeCommand(), and performs the actual Parameter
//   Setting and Status Manipulation; this separation is required to allow manipulation of
//   Feedback Type internally without affecting the User Setting.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SFBTypeCommand(unsigned int lparam, int drive)
{
   
   int s16_original_sfbtype = BGVAR(u16_SFBType_User); //, return_value = SAL_SUCCESS;
   if ((0 != (lparam & 0xFF)) && (1 != (lparam & 0xFF)) && (2 != (lparam & 0xFF)) && (3 != (lparam & 0xFF)) && (4 != (lparam & 0xFF)) &&
       (6 != (lparam & 0xFF)) && (10 != (lparam & 0xFF)) && (11 != (lparam & 0xFF)) && (15 != (lparam & 0xFF)) && (16 != (lparam & 0xFF))) 
      return NOT_AVAILABLE;
   if ((0 != (lparam & 0xFF00)) && (0x100 != (lparam & 0xFF00)))
      return NOT_AVAILABLE;
   // AXIS_OFF;   
   BGVAR(u16_SFBType_User) = (unsigned int)lparam;

   BGVAR(u16_SFBType) = (unsigned int)lparam & 0x00FF;
   VAR(AX0_u16_Abs_Fdbk_Device) &= ~0x0002;
   // set dummy out of range limits to avoid out of range while parameter download
   // actual value is set after config
//   VAR(AX0_s16_Out_Of_Range_High) = 0x7FFF;
//   VAR(AX0_s16_Out_Of_Range_Low) = 0;
   // Disable RT communication, clear all bits except for Sync Symbol
   VAR(AX0_u16_Comm_Fdbk_Flags_A_2) &= 0x00007;

   PriorFeedbackConfig(1, drive);

   if (!IS_SECONDARY_FEEDBACK_ENABLED)
   {
       LVAR(AX0_s32_Feedback_Ptr_2) = &NO_FEEDBACK;
       LLVAR(AX0_u32_SFB_Pos_Fdbk_Lo) = 0LL;
   }
   else
   {
      switch (BGVAR(u16_SFBType))
      {
         case INC_ENC_FDBK:
            LVAR(AX0_s32_Feedback_Ptr_2) = &INC_ENCODER_FEEDBACK;
   //         DetectFeedbackFaults(drive, INC_ENC_FAULTS_MASK, 0LL);
            if (u16_Product == SHNDR_HW) BGVAR(s64_Faults_Mask) &= ~LINE_BRK_FLT_MASK;
            VAR(AX0_s16_Skip_Flags) |= SEC_ENC_CONFIG_NEEDED_MASK;

   //         if (VAR(AX0_s16_Motor_Enc_Type) == 5)
   //         {
   //            BGVAR(s64_Faults_Mask) |= (ILLEGAL_HALLS_MASK | MOTOR_OT_FLT_MASK);
   //            BGVAR(s64_Faults_Mask_2) |= DIFF_HALLS_LINE_BRK_MASK;
   //            LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Halls_Comm_Pos);
   //         }
         break;

         case NK_COMM_FDBK: //Communication (Nikon)
            LVAR(AX0_s32_Feedback_Ptr_2) = &NK_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
   //         DetectFeedbackFaults(drive, NK_COMM_FDBK_FAULTS_MASK, 0LL);
   //         BGVAR(u16_Abs_Feedback_Max_Num_Of_Turns) = (unsigned int)NIKON_ENCDR_MAX_NUM_TURNS;
            VAR(AX0_u16_Abs_Fdbk_Device) |= 0x02;
         break;

         case TAMAGAWA_COMM_MULTI_TURN_FDBK:
            LVAR(AX0_s32_Feedback_Ptr_2) = &TM_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
   //         DetectFeedbackFaults(drive, TM_MULT_COMM_FDBK_FAULTS_MASK, 0LL);
            // If TM Battery-Fault indication then Absolute Position not valid...
            VAR(AX0_u16_Abs_Fdbk_Device) |= 0x02 *(!(VAR(AX0_u16_Abs_Enc_Data_3Frame_2) & 0x04000));
         break;

         case TAMAGAWA_COMM_SINGLE_TURN_FDBK:
         case TAMAGAWA_CID0_SINGLE_TURN:
            LVAR(AX0_s32_Feedback_Ptr_2) = &TM_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
   //         DetectFeedbackFaults(drive, TM_SINGLE_COMM_FDBK_FAULTS_MASK, 0LL);
         break;

         case PS_P_G_COMM_FDBK: // Communication Panasonic Incremental
            LVAR(AX0_s32_Feedback_Ptr_2) = &PS_P_G_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
   //         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;         // Hall-Effect Initialization.
   //         DetectFeedbackFaults(drive, PS_P_G_COMM_FDBK_FAULTS_MASK, 0LL);
         break;

         case FANUC_COMM_FDBK: // Communication (Fanuc) Multi-Turn
            LVAR(AX0_s32_Feedback_Ptr_2) = &FANUC_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&PHASE_FIND_STATE_0 & 0xffff); // Initialize
   //         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;  // Commutation in case Absolute
   //         DetectFeedbackFaults(drive, FANUC_COMM_FDBK_FAULTS_MASK, 0LL);
            // If FA Battery-Fault indication then Absolute Position not valid...
            VAR(AX0_u16_Abs_Fdbk_Device) |= 0x02 * (!(VAR(AX0_u16_Abs_Enc_Data_0Frame_2) & 0x0020));
         break;

         case ENDAT2X_COMM_FDBK: // Communication (EnDat) Multi-Turn
            LVAR(AX0_s32_Feedback_Ptr_2) = &ENDAT2X_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
   //         DetectFeedbackFaults(drive, ENDAT2X_COMM_FDBK_FAULTS_MASK, ENDAT2X_COMM_FDBK_FAULTS_2_MASK);
         break;

         case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
         case SERVOSENSE_MULTI_TURN_COMM_FDBK:
            LVAR(AX0_s32_Feedback_Ptr_2) = &SRVSNS_COMMUNICATION_FEEDBACK;
   //         BGVAR(u16_Mencres_Shr) = 8;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
   //         DetectFeedbackFaults(drive, SRVSNS_SINGLE_COMM_FDBK_FAULTS_MASK_1, SRVSNS_SINGLE_COMM_FDBK_FAULTS_MASK_2);
         break;

         case PS_S_COMM_FDBK:
            LVAR(AX0_s32_Feedback_Ptr_2) = &PS_S_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
   //         DetectFeedbackFaults(drive, PS_S_COMM_FDBK_FAULTS_MASK, PS_S_COMM_FDBK_FAULTS_2_MASK);
            // If PS A5 Battery-Fault indication then Absolute Position not valid...
            VAR(AX0_u16_Abs_Fdbk_Device) |= 0x02 * ((VAR(AX0_u16_Abs_Enc_Data_3Frame_2) & 0x04000) == 0);
         break;

         case SANKYO_COMM_FDBK:
            LVAR(AX0_s32_Feedback_Ptr_2) = &SK_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
   //         DetectFeedbackFaults(drive, SK_MULT_COMM_FDBK_FAULTS_MASK_1, SK_MULT_COMM_FDBK_FAULTS_MASK_2);
            // If TM Battery-Fault indication then Absolute Position not valid...
            VAR(AX0_u16_Abs_Fdbk_Device) |= 0x02 * (!(VAR(AX0_u16_Abs_Enc_Data_3Frame_2) & 0x04000));
         break;

         case YASKAWA_ABS_COMM_FDBK:
            LVAR(AX0_s32_Feedback_Ptr_2) = &YS_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
   //         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;         // Hall-Effect Initialization.
   //         DetectFeedbackFaults(drive, FANUC_COMM_FDBK_FAULTS_MASK, 0LL);
            VAR(AX0_u16_Abs_Fdbk_Device) |= 0x02;
         break;

         case BISSC_COMM_FDBK:
            LVAR(AX0_s32_Feedback_Ptr_2) = &BISSC_COMMUNICATION_FEEDBACK;
            VAR(AX0_s16_Skip_Flags) |= SEC_ENC_CONFIG_NEEDED_MASK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
   //         DetectFeedbackFaults(drive, COMM_FDBK_FLT_MASK, 0LL);
         break;

         case YASKAWA_INC_COMM_FDBK:
            LVAR(AX0_s32_Feedback_Ptr_2) = &YS_COMMUNICATION_FEEDBACK;
   //         VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);
   //         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;         // Hall-Effect Initialization.
   //         DetectFeedbackFaults(drive, FANUC_COMM_FDBK_FAULTS_MASK, 0LL);
         break;

         case NO_SEC_FDBK:
   //         DetectFeedbackFaults(drive, 0x00LL, 0LL); // Do not detect feedback faults
            LVAR(AX0_s32_Feedback_Ptr_2) = &NO_FEEDBACK;
         break;

         default:
   //         DetectFeedbackFaults(drive, 0x00LL, 0LL); // Do not detect feedback faults
            LVAR(AX0_s32_Feedback_Ptr_2) = &NO_FEEDBACK;
         break;
      }
   }
   
   if (LOAD_TYPE == LINEAR_MOTOR)
   {
      SalUnitsVelLinearCommand((long long)BGVAR(u16_Units_Vel_Linear), drive);
      SalUnitsAccDecLinearCommand((long long)BGVAR(u16_Units_Acc_Dec_Linear), drive);
      SalUnitsPosLinearCommand((long long)BGVAR(u16_Units_Pos_Linear), drive);
   }
   else  
   {
      SalUnitsVelRotaryCommand((long long)BGVAR(u16_Units_Vel_Rotary), drive);
      SalUnitsAccDecRotaryCommand((long long)BGVAR(u16_Units_Acc_Dec_Rotary), drive);
      SalUnitsPosRotaryCommand((long long)BGVAR(u16_Units_Pos_Rotary), drive);
   }
   
   if (s16_original_sfbtype != BGVAR(u16_SFBType_User))
   {
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
      VAR(AX0_s16_Skip_Flags) |= SEC_ENC_CONFIG_NEEDED_MASK;
   }

   return SAL_SUCCESS;
}


void MotorFeedbackConfig(int drive)
{
   // AXIS_OFF;

   // Swap U-V
   REFERENCE_TO_DRIVE;
   if (BGVAR(u16_Motor_Feedback_Direction) & 0x0001)
      VAR(AX0_s16_Skip_Flags)  |= SWAP_UV_COMM_MASK;
   else
      VAR(AX0_s16_Skip_Flags)  &= ~SWAP_UV_COMM_MASK;

   // Inverse hall sequence direction
   if (BGVAR(u16_Motor_Feedback_Direction) & 0x0002)
   {
      VAR(AX0_Enc_Hall_Switch_Comm_Tbl_Ptr) = (int)((long)&Enc_Hall_Switch_Tbl_4to6to2to3to1to5 & 0xffff); //Changed to avoid Remark
      VAR(AX0_Enc_Hall_Comm_Tbl_Ptr) = (int)((long)&Enc_Hall_Comm_Tbl_4to6to2to3to1to5 & 0xffff); //Changed to avoid Remark
   }
   else
   {
      VAR(AX0_Enc_Hall_Switch_Comm_Tbl_Ptr) = (int)((long)&Enc_Hall_Switch_Tbl_4to5to1to3to2to6 & 0xffff); //Changed to avoid Remark
      VAR(AX0_Enc_Hall_Comm_Tbl_Ptr) = (int)((long)&Enc_Hall_Comm_Tbl_4to5to1to3to2to6 & 0xffff); //Changed to avoid Remark
   }

   // Inverse Index polarity
   if (BGVAR(u16_Motor_Feedback_Direction) & 0x0004)  EQep1Regs.QDECCTL.bit.QIP = 1;
   else                                               EQep1Regs.QDECCTL.bit.QIP = 0;

   // Reverse Sine-Cosine Direction
   if (BGVAR(u16_Motor_Feedback_Direction) & 0x0008)
      VAR(AX0_s16_Sine_Enc_Bits) |= REVERSE_SIN_COS_MASK;
   else
      VAR(AX0_s16_Sine_Enc_Bits) &= ~REVERSE_SIN_COS_MASK;

   VAR(AX0_s16_Skip_Flags) &= ~SPD_ADV_INV_DIRECTION_MASK;
   if (BGVAR(s16_Direction) == 1)           VAR(AX0_s16_Skip_Flags) |= INV_DIRECTION_MASK;
   else if (BGVAR(s16_Direction) == 0)      VAR(AX0_s16_Skip_Flags) &= ~INV_DIRECTION_MASK;
   else if (BGVAR(s16_Direction) == (-1))
   {  // Same as DIR = 1, but without negation of speed phase advance
      VAR(AX0_s16_Skip_Flags) |= INV_DIRECTION_MASK;
      VAR(AX0_s16_Skip_Flags) |= SPD_ADV_INV_DIRECTION_MASK;
   }

   if ( (BGVAR(u16_MTP_Mode) == 3) && (BGVAR(s16_Direction) != BGVAR(u16_MTP_Dir)) )
   // Condition internal reversal of MPHASE on MTP Mode 3, if User reversed DIR from MTP Database
      VAR(AX0_s16_Electrical_Phase_Offset) = BGVAR(u16_Electrical_Phase_Offset_User) + 0x8000;
   else
      VAR(AX0_s16_Electrical_Phase_Offset) = BGVAR(u16_Electrical_Phase_Offset_User);
}


// Gets as a parameter the CRC polynomial and creats a 256 look up table to be used
// in the CRC calculation in RT
void FillCRCTable(unsigned int u16_crc_polynom, int drive, int enc_source)
{
   unsigned int *crc_array_ptr = u16_Crc_Table[enc_source];
   unsigned long i, bit, u32_crc_value, u32_crc_mask = 0x0000FFFF, u32_effective_bits = 1L;
   unsigned long u32_top_bit = 0x000000008000, u32_crc_polynom_shifted = 0L;
   REFERENCE_TO_DRIVE;
   for (i = 0; i < 16; i++)
   {
      if (u16_crc_polynom & u32_top_bit) break;
      u32_effective_bits++;
      u32_top_bit >>= 1L;
      u32_crc_mask >>= 1L;
   }
   u32_crc_mask >>= 1L;
   u32_effective_bits = 16L - u32_effective_bits;
   u32_top_bit = 1L << (7L + u32_effective_bits);
   u32_crc_polynom_shifted = ((unsigned long)u16_crc_polynom & 0x00000000FFFFFFFF) << 8L;

   VAR(AX0_u16_Abs_Crc_Shl) = 8 - (int)u32_effective_bits;

   for (i = 0L; i < 256L; i++)
   {
      u32_crc_value = i << u32_effective_bits;
      for (bit = 8L; bit > 0L; bit--)
      {
         if (u32_crc_value & u32_top_bit)
            u32_crc_value = (u32_crc_value << 1L) ^ u32_crc_polynom_shifted;
         else
            u32_crc_value = (u32_crc_value << 1L);
      }
      *(crc_array_ptr++) = (unsigned int)((u32_crc_value >> 8) & u32_crc_mask);
   }
}


/*
SalMotorEncResShrCommand  (MENCRESSHROVR):

   If user requests specific value - it will be assigned to the mencresshr value
   otherwise,
   Calc MENCRESSHR according to the formula:

      2^MENCRESSHR =    ((2 ^ 16) * MPOLES/2) /(2 ^ 14)  (*in Halls Only feedback*)    *   Factor_DDR
                        or
                        (2 ^ MSININT)* MENCRES/(2 ^ 14)  (*in sine encoders*)          *   Factor_DDR
                        or
                        4*MENCRES             /(2 ^ 14)  (*in all the rest*)           *   Factor_DDR

   if user requests specific value - set it. otherwise (-1) calculate according to the formula as explained above
*/

int SalMotorEncResShrCommand(long long param,int drive)
{
   int index = 0;
   long long s64_Mencresshr_calculated_temp_val = 0LL,  i = 0LL, s64_temp_Mspeed = 0LL, s64_two_pow_Mencresshr_temp_val = 0LL;
   float f_Mencresshr_calculated_temp_val = 0.0, f_low_speed_factor = 0.0, f_multi_poles_factor = 0.0, f_ddr_mencresshr_factor = 0.0;

   BGVAR(s16_User_Motor_Enc_Res_Shr) = (int)param;

   if (-1 != BGVAR(s16_User_Motor_Enc_Res_Shr)) //set MENCRESSHR according to user's param
   {
      s64_Mencresshr_calculated_temp_val = (long long)BGVAR(s16_User_Motor_Enc_Res_Shr);
   }

   else  //calculate MENCRESSHR with the formula descrived above
   {
      if (FEEDBACK_SERVOSENSE) //SRVSNS will always recieve MENCRESSHR 8
      {
         s64_Mencresshr_calculated_temp_val =  8LL;
      }
      else     //calculate the MENCRESSHR value
      {  // 1st step:  Calc   counts/rev according to FeedbackType
         if (FEEDBACK_HALLS_ONLY) //Halls Only Feedback
         {
            s64_Mencresshr_calculated_temp_val = ((0x10000LL * ((long long)(VAR(AX0_s16_Num_Of_Poles_Ratio))))); //((2 ^ 16) * MPOLES/2)
         }
         else if (SW_SINE_FDBK == BGVAR(u16_FdbkType))   ////sine encoder feedback
         {
            s64_Mencresshr_calculated_temp_val = ((1LL<< BGVAR(u16_Sine_Interpolation)) * (long long)BGVAR(u32_User_Motor_Enc_Res)); //(2 ^ MSININT)* MENCRES
         }
         else  //all other feedbacks
         {
            s64_Mencresshr_calculated_temp_val = (((long long)BGVAR(u32_User_Motor_Enc_Res)) << 2); //4*MENCRES
         }

         /* 2nd step: devide by 2^14  (if smaller than 2^14 - assign 0)*/
         if (0x4000LL > s64_Mencresshr_calculated_temp_val)
         {
            s64_Mencresshr_calculated_temp_val = 0;
         }
         else
         {
            s64_Mencresshr_calculated_temp_val >>= 14; //devide by 2^14
         }

         if (0LL != s64_Mencresshr_calculated_temp_val)
         {
            /* 3rd step: calculate the DDR_mencresshr_factor */
            index = UnitsConversionIndex(&s8_Units_Vel_Out_Loop_Rot_Lin_RPM,drive);
            s64_temp_Mspeed = MultS64ByFixS64ToS64((long long)BGVAR(u32_Mspeed),
            BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_user_fix,
            BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_user_shr);
            if  (BGVAR(u16_MotorType) == LINEAR_MOTOR)
            {                                                  /*60:  60(formula) * 1000 (mpitch is decimal) / 1000 (mspeed is decimal)  */
               f_low_speed_factor = ((((float)(BGVAR(s64_temp_Mspeed))) * 60.0) / (float)BGVAR(u32_Mpitch))   / 4000.0;//LowSpeedFactor=[(linear Motor Speed (mm) *60)/pitch (mm)]/normal_speed;
               f_multi_poles_factor  = 1.0; //multiPolesFactor  = 1;
            }
            else
            {                                          /*4000000:  4000(formula) * 1000 (mspeed is decimal)  */
               f_low_speed_factor = (((float)BGVAR(s64_temp_Mspeed)) / 4000000.0); //    lowSpeedFactor =mspeed/4000;
               f_multi_poles_factor  = 10.0 / (float)VAR(AX0_s16_Num_Of_Poles_Ratio); //multiPolesFactor  = 5/mpoles;
            }

            if (f_multi_poles_factor < f_low_speed_factor)
               f_ddr_mencresshr_factor = f_multi_poles_factor;
            else
               f_ddr_mencresshr_factor = f_low_speed_factor;

            /* 4rd step: multiply  calculated mencresshr with calculated value to get 2^MENCRESSHR = Mencresshr_calculated_temp_val  */
            f_Mencresshr_calculated_temp_val = (float)s64_Mencresshr_calculated_temp_val * f_ddr_mencresshr_factor;
            s64_Mencresshr_calculated_temp_val = ((long long)f_Mencresshr_calculated_temp_val) << 10; //this is to move from decimal to LongLong, but we multiply by 1024 instead of 1000 for better performance
            /* 5rd step: find MENCRESSHR floored value -
            since the formula is 2^MENCRESSHR = Mencresshr_calculated_temp_val
            and we dont want to calculate Log - we find the first i value where 2^i <  Mencresshr_calculated_temp_val */
            s64_two_pow_Mencresshr_temp_val = (1LL<<26); // 2^16 * 1024 (1024 - to be compared to s64_Mencresshr_calculated_temp_val)

            for (i = 26; ((s64_two_pow_Mencresshr_temp_val > s64_Mencresshr_calculated_temp_val) && (i > 9LL)) ; i--)
            {
               s64_two_pow_Mencresshr_temp_val = (1LL<<i);//recalc 2^i * 1024
            }
            i -= 9; //we've added additional 10 by multplying in 1024, we should take back 9 (1 was added by the FOR loop)
            s64_Mencresshr_calculated_temp_val = (long long)i;
         }
      }
   }

   SalMencResShrCommand (s64_Mencresshr_calculated_temp_val, drive);
   return SAL_SUCCESS;
}


int FeedbackConfig(int drive, int enc_source)
{
   int res = SUCCESS;
   unsigned int u16_enc_type = 0;

   // test if enc config needed
   if ((VAR(AX0_s16_Skip_Flags) & ENC_CONFIG_NEEDED_MASK) == 0)  return res;

   VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= 0x0007;
   VAR(AX0_s16_Skip_Flags) &= ~(HALLS_ONLY_MASK | SL_COMMUTATION_MASK);
   BGVAR(s16_DisableInputs) &= ~(ENDAT_DIS_MASK | HIFACE_DIS_MASK | BISSC_DIS_MASK);
   // zero phase advance in case of motor type 4
   if (BGVAR(u16_MotorType) == DC_BRUSH_MOTOR)
      VAR(AX0_s16_Skip_Flags) |= ZERO_PHASE_ADV_MASK;
   else
      VAR(AX0_s16_Skip_Flags) &= ~ZERO_PHASE_ADV_MASK;

   BGVAR(u8_Sl_Mode) = 0;

   if (BGVAR(u16_FdbkType) != SW_SINE_FDBK) // Revert to No-Swap in every case except for
      EQep1Regs.QDECCTL.bit.SWAP = 0; // Sine-Feedback, avoiding toggle that causes incorrect
                                      // Quad Counting (leading to Sine-Quad Mismatch Error)

   if ( (BGVAR(u16_FdbkType) == RESOLVER_FDBK) || (BGVAR(u16_FdbkType) == PS_P_G_COMM_FDBK)          || // Panasonic Incremental
        (FEEDBACK_SERVOSENSE_SINGLE_TURN) || (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK) ||
        ( (BGVAR(u16_FdbkType) == SW_SINE_FDBK) && (VAR(AX0_s16_Motor_Enc_Type) == 10) &&
          (BGVAR(u16_Stegmann_Single_Turn) == 1)                                         )             ) // Hiperface single turn
      BGVAR(u16_Abs_Feedback_Single_Turn) = 1;
   else
      BGVAR(u16_Abs_Feedback_Single_Turn) = 0;

   BGVAR(u16_CommFdbk_Init_Select) = enc_source;

   switch (BGVAR(u16_FdbkType))
   {
      case INC_ENC_FDBK:
         if ( (VAR(AX0_s16_Motor_Enc_Type) != 0) && (VAR(AX0_s16_Motor_Enc_Type) != 1) &&
              (VAR(AX0_s16_Motor_Enc_Type) != 2) && (VAR(AX0_s16_Motor_Enc_Type) != 3) &&
              (VAR(AX0_s16_Motor_Enc_Type) != 4) && (VAR(AX0_s16_Motor_Enc_Type) != 5) &&
              (VAR(AX0_s16_Motor_Enc_Type) != 6) && (VAR(AX0_s16_Motor_Enc_Type) != 11)&&
              (VAR(AX0_s16_Motor_Enc_Type) != 12) )
            return MENCTYPE_MISMATCH;

         // Do not allow usage of index for linear motors (because it can cause damage in MOTORSETUP,
         // and there are no known app which uses it)
         if ( (VAR(AX0_s16_Motor_Enc_Type) >= 0) && (VAR(AX0_s16_Motor_Enc_Type) <= 2) &&
              (BGVAR(u16_MotorType) == LINEAR_MOTOR)                                     )
            return MENCTYPE_LINEAR_MISMATCH;

         if (VAR(AX0_s16_Motor_Enc_Type) == 5)
            VAR(AX0_s16_Skip_Flags) |= (HALLS_ONLY_MASK);

         u16_enc_type = 0x00; // All Quad Encoders require FPGA routing A/B Signals to QEP1,
         if (VAR(AX0_s16_Motor_Enc_Type) == 11) u16_enc_type = 0x10; // except Tamagawa
         else if (BGVAR(u16_AqB_Filter) == 0)  u16_enc_type = 0x03; // Route A/B Signals to QEP1 without FPGA filters
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = 8; // Set A / B Qualifier Value
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         EPwm6Regs.TBCTL.bit.CTRMODE = 3; // Disable resovler excitation

         res = EncoderConfig(DRIVE_PARAM);
      break;

      case SW_SINE_FDBK:
         if ( (VAR(AX0_s16_Motor_Enc_Type) != 0) && (VAR(AX0_s16_Motor_Enc_Type) != 1)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 2) && (VAR(AX0_s16_Motor_Enc_Type) != 3)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 4) && (VAR(AX0_s16_Motor_Enc_Type) != 6)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 7) && (VAR(AX0_s16_Motor_Enc_Type) != 8)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 9) && (VAR(AX0_s16_Motor_Enc_Type) != 10) &&
              (VAR(AX0_s16_Motor_Enc_Type) != 12)                                         )
            return MENCTYPE_MISMATCH;

         //Idle other feedbacks SM to avoid irelevant fails
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_IDLE_STATE;

         // This is to cause the QEP to count the same direction as the Hiperface absolute position
         EQep1Regs.QDECCTL.bit.SWAP = 1;

         // Inform FPGA which feedback is connected
         // Bit0:Sine, bit1:Endat, bit2:Hiperface, bit3:Biss, bit4:Tamagawa, bit5:Sanyo Denky
         switch (VAR(AX0_s16_Motor_Enc_Type))
         { // QEP1 Source
            case 0: case 1: case 2: case 3: case 4:
            case 6: case 7: case 8: u16_enc_type = 0x01; break; // Squared Sine/Cosine
            case 9 : u16_enc_type = 0x02; break; // EnDat 2.x Communication with Sine Signals
            case 10: u16_enc_type = 0x04; break; //.........
            case 11: u16_enc_type = 0x10; break; //.........
         }
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = 8; // Set A / B Qualifier Value
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         EPwm6Regs.TBCTL.bit.CTRMODE = 3; // Disable resovler excitation

         res = EncoderConfig(DRIVE_PARAM);
         //VAR(AX0_s16_Out_Of_Range_High) = BGVAR(s16_Out_Of_Range_Hi)[OUT_OF_RANGE_SINE_ENC];
         //VAR(AX0_s16_Out_Of_Range_Low) = BGVAR(s16_Out_Of_Range_Lo)[OUT_OF_RANGE_SINE_ENC];
      break;

      case RESOLVER_FDBK:
         EPwm6Regs.TBCTL.bit.CTRMODE = 2; // Enable Resolver Excitation
         CalcSwr2dCoef(DRIVE_PARAM);
         VAR(AX0_s16_Crrnt_Run_Code) |= RESET_SWR2D_MASK;

         SalOutOfRangeCommand((long long)BGVAR(u16_Out_of_Range_Percent), drive);
         u16_enc_type = 0;
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         // Signal R.T. to start resolver excitation, ensuring it always start with the

// DEBUG - to activate SL with Resolver
//BGVAR(u8_Sl_Mode) = 1; // This will initiate the sensorless state machine
//VAR(AX0_s16_Skip_Flags) |= SL_COMMUTATION_MASK;
      break;

      case SL_FDBK:
         BGVAR(u8_Sl_Mode) = 1; // This will initiate the sensorless state machine
         VAR(AX0_s16_Skip_Flags) |= SL_COMMUTATION_MASK;
      break;

      case NK_COMM_FDBK:
         FillCRCTable(0x11D, drive, 0); // CRC polynom x^8 + x^4 + x^3 + x^2 + 1
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + 0x0540 * enc_source) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x20;
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case TAMAGAWA_COMM_MULTI_TURN_FDBK:
      case TAMAGAWA_COMM_SINGLE_TURN_FDBK:
      case TAMAGAWA_CID0_SINGLE_TURN:
      case PS_S_COMM_FDBK:
      case SANKYO_COMM_FDBK:
         *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) = 1;
         FillCRCTable(0x101, drive, 0); // CRC polynomial X^8+1
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + 0x0540 * enc_source) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x21;
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         if (u16_background_ran) //if background was ever reached
            BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
         else // Avoid Fault-Resetting for Tamagawa, Panasonic Aabsolute, and Sankyo
              // to preserve Battery-Fault if occurred
         { // Initialize 1mSec Counter which is NOT initialized in Comm-Feedback
           // Handler State Machine COMM_FDBK_CLEAR_FAULT_DONE State.
            BGVAR(s16_DisableInputs) |= COMM_FDBK_DIS_MASK;
            BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
            BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
         }
      break;

      case PS_P_G_COMM_FDBK:
         FillCRCTable(0x101, drive, 0); // CRC polynomial X^8 + 1
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + 0x0540 * enc_source) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x21;
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case FANUC_COMM_FDBK:
         FillCRCTable(0x158, drive, 0); // CRC polynomial X^6 + X^4 + X^3
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + 0x0540 * enc_source) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x22;
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case ENDAT2X_COMM_FDBK:
         FillCRCTable(0x158, drive, 0); // CRC polynomial X^6 + X^4 + X^3
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + 0x0540 * enc_source) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x23;
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         BGVAR(s16_Endat_Init_State) = ENDAT_REVERSAL_RESET_STATE;
      break;

      case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
      case SERVOSENSE_MULTI_TURN_COMM_FDBK:
         FillCRCTable(0x89, drive, 0); // CRC polynomial X^7 + X^3 + 1
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + 0x0540 * enc_source) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x24;
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case YASKAWA_ABS_COMM_FDBK:
      case YASKAWA_INC_COMM_FDBK:
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
         u16_enc_type = 0x27;
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = u16_enc_type;
         if ( (BGVAR(u32_User_Motor_Enc_Res) != 262144) &&
              (BGVAR(u32_User_Motor_Enc_Res) != 32768)    )
            res = CONFIG_FAIL_MENCRES;
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case BISSC_COMM_FDBK:
         FillCRCTable(0x43, drive, 0); // CRC polynomial X^6 + X^1 + 1
         BiSSC_Init(drive, 0);
      break;

      default:
         res = FEEDBACK_NOT_DEFINED;
      break;
   }

   if ((res == SUCCESS) && (BGVAR(u8_EncoutMode)))
      EncoderSimSetup(drive);

   SalMotorEncResShrCommand((long long)s16_User_Motor_Enc_Res_Shr, drive);

   VAR(AX0_s16_Skip_Flags) &= ~ENC_CONFIG_NEEDED_MASK;
   return (res);
}


int SFBConfig(int drive, int enc_source)
{
   // AXIS_OFF;
   int res = SUCCESS;
   unsigned int u16_enc_type = 0;

   // test if enc config needed
   if ((VAR(AX0_s16_Skip_Flags) & SEC_ENC_CONFIG_NEEDED_MASK) == 0)  return res;
   if (!IS_SECONDARY_FEEDBACK_ENABLED) return res;
   
   VAR(AX0_u16_Comm_Fdbk_Flags_A_2) &= 0x0007;
//   VAR(AX0_s16_Skip_Flags) &= ~(HALLS_ONLY_MASK | SL_COMMUTATION_MASK);
//   BGVAR(s16_DisableInputs) &= ~(ENDAT_DIS_MASK | HIFACE_DIS_MASK | BISSC_DIS_MASK);
   // zero phase advance in case of motor type 4
/*   if (BGVAR(u16_MotorType) == DC_BRUSH_MOTOR)
      VAR(AX0_s16_Skip_Flags) |= ZERO_PHASE_ADV_MASK;
   else
      VAR(AX0_s16_Skip_Flags) &= ~ZERO_PHASE_ADV_MASK; */

/*   BGVAR(u8_Sl_Mode) = 0;

   if (BGVAR(u16_FdbkType) != SW_SINE_FDBK) // Revert to No-Swap in every case except for
      EQep1Regs.QDECCTL.bit.SWAP = 0; // Sine-Feedback, avoiding toggle that causes incorrect
                                      // Quad Counting (leading to Sine-Quad Mismatch Error)

   if ( (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK) ||
        (FEEDBACK_SERVOSENSE)                                   ||
        (BGVAR(u16_FdbkType) == PS_P_G_COMM_FDBK)                 ) // Hiperface single turn
      BGVAR(u16_Abs_Feedback_Single_Turn) = 1;
   else
      BGVAR(u16_Abs_Feedback_Single_Turn) = 0;
*/
   if (BGVAR(s16_SFB_Direction) == 1)           VAR(AX0_u16_Comm_Fdbk_Flags_B_2) |= INV_DIRECTION_MASK;
   else if (BGVAR(s16_SFB_Direction) == 0)      VAR(AX0_u16_Comm_Fdbk_Flags_B_2) &= ~INV_DIRECTION_MASK;

   switch (BGVAR(u16_SFBType))
   {
      case INC_ENC_FDBK:
         if ( (VAR(AX0_s16_Motor_Enc_Type) != 0) && (VAR(AX0_s16_Motor_Enc_Type) != 1)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 2) && (VAR(AX0_s16_Motor_Enc_Type) != 3)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 4) && (VAR(AX0_s16_Motor_Enc_Type) != 5)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 6) && (VAR(AX0_s16_Motor_Enc_Type) != 11) &&
              (VAR(AX0_s16_Motor_Enc_Type) != 12)                                         )
            return MENCTYPE_MISMATCH;

         // Do not allow usage of index for linear motors (because it can cause damage in MOTORSETUP,
         // and there are no known app which uses it)
         if ( (VAR(AX0_s16_Motor_Enc_Type) >= 0) && (VAR(AX0_s16_Motor_Enc_Type) <= 2) &&
              (BGVAR(u16_MotorType) == LINEAR_MOTOR)                                     )
            return MENCTYPE_LINEAR_MISMATCH;

//         if (VAR(AX0_s16_Motor_Enc_Type) == 5)
//            VAR(AX0_s16_Skip_Flags) |= (HALLS_ONLY_MASK);

         u16_enc_type = 0x00; // All Quad Encoders require FPGA routing A/B Signals to QEP1,
/*         if (VAR(AX0_s16_Motor_Enc_Type) == 11) u16_enc_type = 0x10; // except Tamagawa
         else
         {
            if (BGVAR(u16_AqB_Filter) == 0)  u16_enc_type = 0x03; // Route A/B Signals to QEP1 without FPGA filters
         } */
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + enc_source * 0x0540) = 8; // Set A / B Qualifier Value
         *(int*)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         EPwm6Regs.TBCTL.bit.CTRMODE = 3; // Disable resovler excitation

         res = EncoderConfig(DRIVE_PARAM);
      break;
/*
      case SW_SINE_FDBK:
         if ( (VAR(AX0_s16_Motor_Enc_Type) != 0) && (VAR(AX0_s16_Motor_Enc_Type) != 1)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 2) && (VAR(AX0_s16_Motor_Enc_Type) != 3)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 4) && (VAR(AX0_s16_Motor_Enc_Type) != 6)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 7) && (VAR(AX0_s16_Motor_Enc_Type) != 8)  &&
              (VAR(AX0_s16_Motor_Enc_Type) != 9) && (VAR(AX0_s16_Motor_Enc_Type) != 10) &&
              (VAR(AX0_s16_Motor_Enc_Type) != 12)                                         )
            return MENCTYPE_MISMATCH;

         // This is to cause the QEP to count the same direction as the Hiperface absolute position
         EQep1Regs.QDECCTL.bit.SWAP = 1;

         // Inform FPGA which feedback is connected
         // Bit0:Sine bit1:Endat bit2:Hiperface bit3:Biss bit4:Tamagawa bit5:Sanyo Denky
         switch (VAR(AX0_s16_Motor_Enc_Type))
         { // QEP1 Source
            case 0: case 1: case 2: case 3: case 4:
            case 6: case 7: case 8: u16_enc_type = 0x01; break; // Squared Sine/Cosine
            case 9 : u16_enc_type = 0x02; break; // EnDat 2.x Communication with Sine Signals
            case 10: u16_enc_type = 0x04; break; //.........
            case 11: u16_enc_type = 0x10; break; //.........
         }
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + enc_source * 0x0540) = 8; // Set A / B Qualifier Value
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         EPwm6Regs.TBCTL.bit.CTRMODE = 3; // Disable resovler excitation

         res = EncoderConfig(DRIVE_PARAM);
         //VAR(AX0_s16_Out_Of_Range_High) = BGVAR(s16_Out_Of_Range_Hi)[OUT_OF_RANGE_SINE_ENC];
         //VAR(AX0_s16_Out_Of_Range_Low) = BGVAR(s16_Out_Of_Range_Lo)[OUT_OF_RANGE_SINE_ENC];
      break;

      case RESOLVER_FDBK:
         EPwm6Regs.TBCTL.bit.CTRMODE = 2; // Enable Resolver Excitation
         CalcSwr2dCoef(DRIVE_PARAM);
         VAR(AX0_s16_Crrnt_Run_Code) |= RESET_SWR2D_MASK;

         SalOutOfRangeCommand((long long)BGVAR(u16_Out_of_Range_Percent), drive);
         u16_enc_type = 0;
         *(int*)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         // Signal R.T. to start resolver excitation, ensuring it always start with the

// DEBUG - to activate SL with Resolver
//BGVAR(u8_Sl_Mode) = 1; // This will initiate the sensorless state machine
//VAR(AX0_s16_Skip_Flags) |= SL_COMMUTATION_MASK;
      break;

      case SL_FDBK:
         BGVAR(u8_Sl_Mode) = 1; // This will initiate the sensorless state machine
         VAR(AX0_s16_Skip_Flags) |= SL_COMMUTATION_MASK;
      break; */

      case NK_COMM_FDBK:
         FillCRCTable(0x11D, drive, 1); // CRC polynom x^8 + x^4 + x^3 + x^2 + 1
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length_2) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length_2) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + enc_source * 0x0540) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x20;
         *(int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case TAMAGAWA_COMM_MULTI_TURN_FDBK:
      case TAMAGAWA_COMM_SINGLE_TURN_FDBK:
      case TAMAGAWA_CID0_SINGLE_TURN:
      case PS_S_COMM_FDBK:
      case SANKYO_COMM_FDBK:
         *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + enc_source * 0x0540) = 1;
         FillCRCTable(0x101, drive, 1); // CRC polynomial X^8+1
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length_2) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length_2) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + enc_source * 0x0540) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x21;
         *(int*)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         if (u16_background_ran) //if background was ever reached
            BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_START_INIT;
         else // Avoid Fault-Resetting for Tamagawa, Panasonic Aabsolute, and Sankyo
              // preserving Battery-Fault if occurred
         { // Initialize 1mSec Counter which is NOT initialized in Comm-Feedback
           // Handler State Machine COMM_FDBK_CLEAR_FAULT_DONE State.
            BGVAR(s16_DisableInputs) |= COMM_SEC_FDBK_DIS_MASK;
            BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
            BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
         }
      break;

      case PS_P_G_COMM_FDBK:
         FillCRCTable(0x101, drive, 1); // CRC polynomial X^8 + 1
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length_2) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length_2) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + enc_source * 0x0540) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x21;
         *(int*)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case FANUC_COMM_FDBK:
         FillCRCTable(0x158, drive, 1); // CRC polynomial X^6 + X^4 + X^3
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length_2) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length_2) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + enc_source * 0x0540) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x22;
         *(int*)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case ENDAT2X_COMM_FDBK:
         FillCRCTable(0x158, drive, 1); // CRC polynomial X^6 + X^4 + X^3
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length_2) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length_2) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + enc_source * 0x0540) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x23;
         *(int*)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         BGVAR(s16_Endat_Init_State) = ENDAT_REVERSAL_RESET_STATE;
      break;

      case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
      case SERVOSENSE_MULTI_TURN_COMM_FDBK:
         FillCRCTable(0x89, drive, 1); // CRC polynomial X^7 + X^3 + 1
         *(int *)(FPGA_SD_CLOCK_DIV_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length_2) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + enc_source * 0x0540) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length_2) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + enc_source * 0x0540) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + enc_source * 0x0540) = BGVAR(u16_Field_Length);
         u16_enc_type = 0x24;
         *(int*)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case YASKAWA_ABS_COMM_FDBK:
      case YASKAWA_INC_COMM_FDBK:
         u16_enc_type = 0x27;
         *(int*)(FPGA_MFB_ENCODER_SEL_REG_ADD + enc_source * 0x0540) = u16_enc_type;
         if ( (BGVAR(u32_User_Motor_Enc_Res) != 262144) &&
              (BGVAR(u32_User_Motor_Enc_Res) != 32768)    )
            res = CONFIG_FAIL_MENCRES;
         BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      break;

      case BISSC_COMM_FDBK:
         FillCRCTable(0x43, drive, 1); // CRC polynomial X^6 + X^1 + 1
         BiSSC_Init(drive, 1);
      break;

      case NO_SEC_FDBK:
         // TBD
      break;

      default:
         res = FEEDBACK_NOT_DEFINED;
      break;
   }

/*   if ((res == SUCCESS) && (BGVAR(u8_EncoutMode)))
      EncoderSimSetup(drive);

   SetMencresshr(drive); */

   VAR(AX0_s16_Skip_Flags) &= ~SEC_ENC_CONFIG_NEEDED_MASK;
   return (res);
}


void SetMotorEncIntrpolation(int drive)
{
   // AXIS_OFF;
   long long s64_temp_value;
   unsigned int u16_prev_motor_enc_interpolation = BGVAR(u16_Motor_Enc_Interpolation);

   if ( (BGVAR(u16_FdbkType) != INC_ENC_FDBK) || (BGVAR(u16_Motor_Enc_Interpolation_Mode) == 0) || (u16_Product == SHNDR_HW) )
      BGVAR(u16_Motor_Enc_Interpolation) = 1;
   else if ( BGVAR(u16_Motor_Enc_Interpolation_Override) == 1 )
      BGVAR(u16_Motor_Enc_Interpolation) = BGVAR(u16_Motor_Enc_Interpolation_Override_Value);
   else
   {
      // Calculate and set the maximum interpolation
      // The interpolated AqB frequency is limited to 6,000,000 lines/sec (FPGA limit)
      // FPGA limit was observed by actual test to 9.178 MHz. Cosider 1.2 over speed over VLIM and 80% margin,
      // the FPGA limit is 6.118 MHz. For calculation take 6 MHz.
      // The limit is:
      // (VLIM internal) * MENCRES * interpolation * 8000 / 2^32 <= 6,000,000
      // Comments:
      // 1. The same calculation is applied to rotary and linear motor.
      // 2. interpulation range is 1 to 16
      // 3. Find the highest integer value for interpolation that satisfies the above equation
      //
      // interpolation = int( 750 * 2^32 / (VLIM int * MENCRES) )

      s64_temp_value = 0x000002ee00000000LL / ((long long)BGVAR(u32_User_Motor_Enc_Res) * BGVAR(s32_V_Lim_Design));

      if (s64_temp_value < 1) s64_temp_value = 1;
      if (s64_temp_value > 16) s64_temp_value = 16;

      // MFBMODE option to avoid calculation of encoder interpolation in case 90 degrees does not keeped between A and B pulses.
      if ( BGVAR(u16_Motor_Enc_Interpolation_Mode) != 2 )
      {
        // in the field found that encoder interpolation with high resolution encoders causes to pulse loss
        // The reason of pulse loss is that 90 degrees not always keeped between A and B pulses.
        // from the field expirience:
        //                  Disable interpolation in cases when resolution is higher 2um for linear motors
        //               and higher than 17bit for rotary.

        if (s64_temp_value > 1)
        {
          if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
          {
            if ( ((float)BGVAR(u32_Mpitch) / (float)BGVAR(u32_User_Motor_Enc_Res)/4.0) <= 2.0) s64_temp_value =1; //mpitch changed to decimal
          }
          else //rotary motors
          {
            if (BGVAR(u32_User_Motor_Enc_Res) >= 32768) s64_temp_value =1;
          }
        }
      }
      BGVAR(u16_Motor_Enc_Interpolation) = (unsigned int)s64_temp_value;
   }

   // Set FPGA to the 1/T interpolation
   *(int*)FPGA_OVT_FACTOR_REG_ADD = BGVAR(u16_Motor_Enc_Interpolation);
   if (BGVAR(u16_Motor_Enc_Interpolation) > 1)
      *(int*)FPGA_ONE_OVER_T_EN_REG_ADD = 1;
   else
      *(int*)FPGA_ONE_OVER_T_EN_REG_ADD = 0;

   UpdateVelocityLimits(DRIVE_PARAM);

   //update position conversion that uses mencres
   ConvertInternalToPos1000(drive);
   ConvertPosToInternal(drive);
   ConvertCountsToInternal(drive);

   // If MFBINT changed, zero position to avoid commutation issues during the change
   if (u16_prev_motor_enc_interpolation != BGVAR(u16_Motor_Enc_Interpolation))
   {
      BGVAR(s16_Motor_Enc_Interpolation_Delta) = (int)BGVAR(u16_Motor_Enc_Interpolation) - (int)u16_prev_motor_enc_interpolation;
     
      VAR(AX0_s16_DF_Run_Code) |= DUAL_LOOP_CHANGE_PFB_RES_MASK;
   }
}

void SetLoadEncIntrpolation(int drive)
{
   long long s64_temp_value;
   unsigned int u16_prev_load_enc_interpolation = BGVAR(u16_Load_Enc_Interpolation);
   // AXIS_OFF;
   if (!IS_SECONDARY_FEEDBACK_ENABLED) return;
   if ( (BGVAR(u16_SFBType) != INC_ENC_FDBK) || (BGVAR(u16_Load_Enc_Interpolation_Mode) == 0) || (u16_Product == SHNDR_HW) )
      BGVAR(u16_Load_Enc_Interpolation) = 1;
   else if ( BGVAR(u16_Load_Enc_Interpolation_Override) == 1 )
      BGVAR(u16_Load_Enc_Interpolation) = BGVAR(u16_Load_Enc_Interpolation_Override_Value);
   else
   {
      // Calculate and set the maximum interpolation
      // The interpolated AqB frequency is limited to 6,000,000 lines/sec (FPGA limit)
      // FPGA limit was observed by actual test to 9.178 MHz. Cosider 1.2 over speed over VLIM and 80% margin,
      // the FPGA limit is 6.118 MHz. For calculation take 6 MHz.
      // The limit is:
      // (VLIM internal) * SFBRES * interpolation * 8000 / 2^32 <= 6,000,000
      // Comments:
      // 1. The same calculation is applied to rotary and linear motor.
      // 2. interpulation range is 1 to 16
      // 3. Find the highest integer value for interpolation that satisfies the above equation
      //
      // interpolation = int( 750 * 2^32 / (VLIM int * SFBCRES) )

      s64_temp_value = 0x000002ee00000000LL / ((long long)BGVAR(u32_User_Sec_Enc_Res) * BGVAR(s32_V_Lim_Design));

      if (s64_temp_value < 1) s64_temp_value = 1;
      if (s64_temp_value > 16) s64_temp_value = 16;

      // MFBMODE option to avoid calculation of encoder interpolation in case 90 degrees does not keeped between A and B pulses.

        // in the field found that encoder interpolation with high resolution encoders causes to pulse loss
        // The reason of pulse loss is that 90 degrees not always keeped between A and B pulses.
        // from the field expirience:
        //                  Disable interpolation in cases when resolution is higher 2um for linear motors
        //               and higher than 17bit for rotary.

        if (s64_temp_value > 1)
        {
          if (LOAD_TYPE == LINEAR_MOTOR) //LOAD_TYPE holds a value paralel to motortype
          {
            if ( (1000.0 / (float)BGVAR(u32_User_Sec_Enc_Res)/4.0) <= 2.0) s64_temp_value =1;
          }
          else //rotary motors
          {
            if (BGVAR(u32_User_Sec_Enc_Res) >= 32768) s64_temp_value =1;
          }
        }

      BGVAR(u16_Load_Enc_Interpolation) = (unsigned int)s64_temp_value;
   }

   // Set FPGA to the 1/T interpolation
   *(int*)(FPGA_SFB_INTERFACE_ADD | 0x00000013) = BGVAR(u16_Load_Enc_Interpolation); //(FPGA_OVT_FACTOR_REG_ADD)
   if (BGVAR(u16_Load_Enc_Interpolation) > 1)
      *(int*)(FPGA_SFB_INTERFACE_ADD | 0x00000014) = 1;  //(ONE_OVER_T)
   else
      *(int*)(FPGA_SFB_INTERFACE_ADD | 0x00000014) = 0;  //(ONE_OVER_T)

   UpdateVelocityLimits(DRIVE_PARAM);

   //update position conversion that uses mencres
   ConvertInternalToPos1000(drive);
   ConvertPosToInternal(drive);
   ConvertCountsToInternal(drive);

   // If SFBINT changed, zero position to avoid commutation issues during the change
   if (u16_prev_load_enc_interpolation != BGVAR(u16_Load_Enc_Interpolation))
      VAR(AX0_s16_DF_Run_Code_2) |= DUAL_LOOP_CHANGE_PFB_RES_MASK;
}

void WaitForNext32kHzTaskStart(void)
{
   long s32_timer_capture_1ms;
   unsigned int  u16_timer_capture_3125;

   if (u16_background_ran) //if background was ever reached
   {
      s32_timer_capture_1ms = Cntr_1mS;      //wait for next start of 1kHz task
      while (s32_timer_capture_1ms == Cntr_1mS){}

      u16_timer_capture_3125 = Cntr_3125;      //wait for next start of 32kHz task
      while (u16_timer_capture_3125 == Cntr_3125){}
   }
}


int SalLockedRotor(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (param == 1LL)
   {
      VAR(AX0_s16_Skip_Flags) |=  MOTOR_SETUP_MASK;
      BGVAR(u16_Force_Regular_Pwm_Freq) = 1;
   }
   else
   {
      BGVAR(u16_Force_Regular_Pwm_Freq) = 0;
      VAR(AX0_s16_Skip_Flags) &=  ~MOTOR_SETUP_MASK;
   }
   return (SAL_SUCCESS);
}


int SalReadLockedRotor(long long *data, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   *data = 0LL;
   if ((VAR(AX0_s16_Skip_Flags) &  MOTOR_SETUP_MASK) != 0) *data = 1LL;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalHcIcmd1AmpCommand
// Description:
//          This function is called in response to the HCICMD1AMP command.
//
//
// Author: S.S
// Algorithm:
// Revisions:
//**********************************************************
int SalHcIcmd1AmpCommand(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (param > (long long)BGVAR(s32_Drive_I_Peak)) return (VALUE_TOO_HIGH);
   BGVAR(s32_Icmd_Harmonic_Amp_1) = (long)param;

   VAR(AX0_s16_Icmd_Harmonic_Amp_1) = (int)(((long)param * 26214L) / BGVAR(s32_Drive_I_Peak));

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalHcIcmd2AmpCommand
// Description:
//          This function is called in response to the HCICMD2AMP command.
//
//
// Author: S.S
// Algorithm:
// Revisions:
//**********************************************************
int SalHcIcmd2AmpCommand(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (param > (long long)BGVAR(s32_Drive_I_Peak)) return (VALUE_TOO_HIGH);
   BGVAR(s32_Icmd_Harmonic_Amp_2) = (long)param;

   VAR(AX0_s16_Icmd_Harmonic_Amp_2) = (int)(((long)param * 26214L) / BGVAR(s32_Drive_I_Peak));

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalHcIcmd1PhaseCommand
// Description:
//          This function is called in response to the HCICMD1PHASE command.
//
//
// Author: S.S
// Algorithm:
// Revisions:
//**********************************************************
int SalHcIcmd1PhaseCommand(long long param,int drive)
{
   // AXIS_OFF;
   // Convert to internal units by multiplying by 65536 and dividing by 360 = multiplying by 0x5B06 >> 7
   REFERENCE_TO_DRIVE;
   Math32 = ((long)param * 0x5B06) >> 7;
   VAR(AX0_s16_Icmd_Harmonic_Phase_1) = (int)Math32;
   return (SAL_SUCCESS);
}


int SalReadHcIcmd1PhaseCommand(long long* param,int drive)
{
   // AXIS_OFF;
   unsigned int tmp;
    // Convert the value to external units by multiplying by 360 and dividing by 65536
   REFERENCE_TO_DRIVE;
   tmp = (unsigned int)VAR(AX0_s16_Icmd_Harmonic_Phase_1);
   Math32 = ((long)tmp * 0x5A00)+0x200000;
   Math32 = Math32 >> 22;

   if (Math32 >= 360) Math32-=360;

   *param = (long long)Math32;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalHcIcmd2PhaseCommand
// Description:
//          This function is called in response to the HCICMD2PHASE command.
//
//
// Author: S.S
// Algorithm:
// Revisions:
//**********************************************************
int SalHcIcmd2PhaseCommand(long long param,int drive)
{
   // AXIS_OFF;
   // Convert to internal units by multiplying by 65536 and dividing by 360 = multiplying by 0x5B06 >> 7
   REFERENCE_TO_DRIVE;
   Math32 = ((long)param * 0x5B06) >> 7;
   VAR(AX0_s16_Icmd_Harmonic_Phase_2) = (int)Math32;
   return (SAL_SUCCESS);
}


int SalReadHcIcmd2PhaseCommand(long long* param,int drive)
{
   // AXIS_OFF;

   unsigned int tmp;
    // Convert the value to external units by multiplying by 360 and dividing by 65536
   REFERENCE_TO_DRIVE;
   tmp = (unsigned int)VAR(AX0_s16_Icmd_Harmonic_Phase_2);
   Math32 = ((long)tmp * 0x5A00)+0x200000;
   Math32 = Math32 >> 22;

   if (Math32 >= 360) Math32-=360;

   *param = (long long)Math32;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalHcFb1PhaseCommand
// Description:
//          This function is called in response to the HCFB1PHASE command.
//
//
// Author: S.S
// Algorithm:
// Revisions:
//**********************************************************
int SalHcFb1PhaseCommand(long long param,int drive)
{
   // AXIS_OFF;
   // Convert to internal units by multiplying by 65536 and dividing by 360 = multiplying by 0x5B06 >> 7
   REFERENCE_TO_DRIVE;
   Math32 = ((long)param * 0x5B06) >> 7;
   VAR(AX0_s16_Fb_Harmonic_Phase_1) = (int)Math32;
   return (SAL_SUCCESS);
}


int SalReadHcFb1PhaseCommand(long long* param,int drive)
{
   // AXIS_OFF;

   unsigned int tmp;
   // Convert the value to external units by multiplying by 360 and dividing by 65536
   REFERENCE_TO_DRIVE;
   tmp = (unsigned int)VAR(AX0_s16_Fb_Harmonic_Phase_1);
   Math32 = ((long)tmp * 0x5A00) + 0x200000;
   Math32 = Math32 >> 22;

   if (Math32 >= 360) Math32-=360;

   *param = (long long)Math32;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalHcFb2PhaseCommand
// Description:
//          This function is called in response to the HCFB2PHASE command.
//
//
// Author: S.S
// Algorithm:
// Revisions:
//**********************************************************
int SalHcFb2PhaseCommand(long long param,int drive)
{
   // AXIS_OFF;
   // Convert to internal units by multiplying by 65536 and dividing by 360 = multiplying by 0x5B06 >> 7
   REFERENCE_TO_DRIVE;
   Math32 = ((long)param * 0x5B06) >> 7;
   VAR(AX0_s16_Fb_Harmonic_Phase_2) = (int)Math32;
   return (SAL_SUCCESS);
}


int SalReadHcFb2PhaseCommand(long long* param,int drive)
{
   // AXIS_OFF;

   unsigned int tmp;
   // Convert the value to external units by multiplying by 360 and dividing by 65536
   REFERENCE_TO_DRIVE;
   tmp = (unsigned int)VAR(AX0_s16_Fb_Harmonic_Phase_2);
   Math32 = ((long)tmp * 0x5A00)+0x200000;
   Math32 = Math32 >> 22;

   if (Math32 >= 360) Math32-=360;

   *param = (long long)Math32;
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalReadHcFb1AmpCommand
// Description:
//    This function returns HCFB1AMP value
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalReadHcFb1AmpCommand(int drive)
{
   // AXIS_OFF;
   long long s64_temp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error
   // convert internal to external position units
   s64_temp = MultS64ByFixS64ToS64((long long)LVAR(AX0_s32_Fb_Harmonic_Amp_1),
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);
   PrintSignedLongLong(s64_temp);
   PrintChar(SPACE);
   PrintChar('[');
   PrintString((char *)s8_Units_Pos,0);
   PrintChar(']');
   PrintCrLf();

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalHcFb1AmpCommand
// Description:
//    This function sets HCFB1AMP value
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalHcFb1AmpCommand(int drive)
{
   // AXIS_OFF;
   int index = 0;
   //Convert external to internal position units
   index = UnitsConversionIndex(&s8_Units_Pos, drive);
   LVAR(AX0_s32_Fb_Harmonic_Amp_1) = (long) MultS64ByFixS64ToS64(s64_Execution_Parameter[0],
                           BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                           BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadHcFb1AmpCommand
// Description:
//    This function returns HCFB1AMP value
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalReadHcFb2AmpCommand(int drive)
{
   // AXIS_OFF;
   long long s64_temp;
   REFERENCE_TO_DRIVE;
   // convert internal to external position units
   s64_temp = MultS64ByFixS64ToS64((long long)LVAR(AX0_s32_Fb_Harmonic_Amp_2),
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);
   PrintSignedLongLong(s64_temp);
   PrintChar(SPACE);
   PrintChar('[');
   PrintString((char *)s8_Units_Pos,0);
   PrintChar(']');
   PrintCrLf();

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalHcFb1AmpCommand
// Description:
//    This function sets HCFB1AMP value
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalHcFb2AmpCommand(int drive)
{
   // AXIS_OFF;
   int index = 0;
   //Convert external to internal position units
   index = UnitsConversionIndex(&s8_Units_Pos, drive);
   LVAR(AX0_s32_Fb_Harmonic_Amp_2) = (long) MultS64ByFixS64ToS64(s64_Execution_Parameter[0],
                           BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                           BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);

   return SAL_SUCCESS;
}


void DesignHallsMsqFilter(int drive)
{
//   int s16_temp_value;
   // AXIS_OFF;

/* Set the input scale to 3 (2^3 = 8) to increase resolution.
*  For maximum of 60,000rpm, with an 8-Pole Motor, there are 240,000 Commutation Cycle per Minute.
*  At full Sampling Rate, the expected Counting Rate is 240,000 * 65,536 / 60 / 32,000 = 8,192 / Tsp
*  Set Input Scaling of 4 to avoid exceeding re 3,000 * 1.25 = 3,750 counts/Tsp.
*  Input scale of 8 gives 3,750*8 = 30,000 bits/Tsp.
*
*  AX0_u16_In_Scale  - determines how many bits to shift left the gear input
*  AX0_u16_Out_Scale - determines how many bits to shift right the gear input after multiplied by GEARIN */
   REFERENCE_TO_DRIVE;
   VAR(AX0_u16_In_Scale_Design) = 0;
   VAR(AX0_u16_Out_Scale) = 0;

// t1 - depth of the filter, expressed in sample time of 125us

   VAR(AX0_u16_Msq_Fltr_T1) = (unsigned int)(BGVAR(u32_Halls_Msq_Filt_User_T1) / 125);
   VAR(AX0_u16_Msq_Fltr_T2) = (unsigned int)(BGVAR(u16_Halls_Msq_Filt_User_T2) / 125);

// Vff = (t1 - user_Vff)/25 (in RT it will be divided by 10 to get Vff in units of sample time).
   if (BGVAR(s32_Halls_Msq_Filt_User_Vff) > BGVAR(s32_Anin1_Msq_Filt_User_Vff))
      VAR(AX0_s16_Msq_Vff_N) = 0;
   else
      VAR(AX0_s16_Msq_Vff_N) = (int)((BGVAR(u32_Halls_Msq_Filt_User_T1) - BGVAR(s32_Halls_Msq_Filt_User_Vff) ) / 25);

// Beta1 = f1 = 1/t1.  Use 15 shifts, so Beta1 = 32768 / t1.
// Alpha1 = "1" - Beta1 = 32768 - Beta1

   VAR(AX0_s16_Msq_Fltr_Beta1_Design)  = (int)(((long)32768 + ((long)VAR(AX0_u16_Msq_Fltr_T1) >> 1)) / VAR(AX0_u16_Msq_Fltr_T1));
   VAR(AX0_s16_Msq_Fltr_Alpha1_Design) = (int)((long)32768 - (long)VAR(AX0_s16_Msq_Fltr_Beta1_Design));

// Calculate the correction for Sxp in case of rollover in the input.
// Correction = (t1-1)*32768, but since we need to take t1 as back calculated fro Beta1, we get that
// Correction = Alpha1 / Beta1 * 2^23

   LVAR(AX0_s32_Msq_Fltr_Rollover_Sxp_Fix_Design) = (long)((((long long)0x00800000 * VAR(AX0_s16_Msq_Fltr_Alpha1_Design)) + ((long long)VAR(AX0_s16_Msq_Fltr_Beta1_Design) >> 1)) / VAR(AX0_s16_Msq_Fltr_Beta1_Design));

// S1 = t1 - 1

   VAR(AX0_s16_Msq_Fltr_S1) = VAR(AX0_u16_Msq_Fltr_T1) - 1;

// Inv_Delta = 1/(t1*(t1-1)) = Beta1^2 / Alpha1

   FloatToFix16Shift16(&VAR(AX0_s16_Msq_Fltr_Inv_Delta_Fix_Design),
                       &VAR(AX0_u16_Msq_Fltr_Inv_Delta_Shr_Design),
                       (float)((float)VAR(AX0_s16_Msq_Fltr_Beta1_Design) * (float)VAR(AX0_s16_Msq_Fltr_Beta1_Design) / (float)VAR(AX0_s16_Msq_Fltr_Alpha1_Design)) );

   VAR(AX0_u16_Msq_Fltr_Inv_Delta_Shr_Design) -= 1;  // add 15 because Beta in the above calc has scale of 2^15, and sub 16 to increase the resolution of V[n]

// Beta2 = f2 = 1/t2.  Use 15 shifts, so Beta1 = 32768 / t2.
// Alpha2 = "1" - Beta2 = 32768 - Beta2

   if (VAR(AX0_u16_Msq_Fltr_T2) > 0)
   {
      VAR(AX0_s16_Msq_Fltr_Beta2_Design)  = (int)((long)32768 / VAR(AX0_u16_Msq_Fltr_T2));
      VAR(AX0_s16_Msq_Fltr_Alpha2_Design) = (int)((long)32768 - (long)VAR(AX0_s16_Msq_Fltr_Beta2_Design));
   }
   else
   {
      VAR(AX0_s16_Msq_Fltr_Beta2_Design) = 0x8000;//32768;
      VAR(AX0_s16_Msq_Fltr_Alpha2_Design) = 0;
   }

// A_Factor = f2/(1-f2) = Beta2 / Alpha2

   FloatToFix16Shift16(&VAR(AX0_s16_Msq_Fltr_A_Factor_Fix_Design),
                       &VAR(AX0_u16_Msq_Fltr_A_Factor_Shr_Design),
                       (float)((float)VAR(AX0_s16_Msq_Fltr_Beta2_Design) / (float)VAR(AX0_s16_Msq_Fltr_Alpha2_Design)) );

// Aff_Gain = (t1-1)^2 * User_Aff/1000 = (Alpha1/Beta1)^2 * User_Aff/1000

   FloatToFix16Shift16(&VAR(AX0_s16_Msq_Fltr_Aff_Gain_Fix),
                       &VAR(AX0_u16_Msq_Fltr_Aff_Gain_Shr),
                       (float)((float)VAR(AX0_s16_Msq_Fltr_Alpha1_Design) * (float)VAR(AX0_s16_Msq_Fltr_Alpha1_Design) * (float)BGVAR(s16_Halls_Msq_Filt_User_Aff)
                              / (float)VAR(AX0_s16_Msq_Fltr_Beta1_Design) / (float)VAR(AX0_s16_Msq_Fltr_Beta1_Design) / 1000.0) );

// Calculate the units conversion factor of velocity and acceleration from filter units to internal units
// factor = 2^(16 - _AX0_u16_In_Scale) / (XENCRES*?4?) * GearIn / GearOut
// Consider gear mode for whether XENCRES should be multiplied by 4 or not

   FloatToFixS32Shift16(&LVAR(AX0_s32_Msq_Fltr_Units_Conv_Fix_Design),
                        &VAR(AX0_u16_Msq_Fltr_Units_Conv_Shr_Design),
                        (float)1 );

   VAR(AX0_u16_Msq_Fltr_Units_Conv_Shr_Design) -= (16 - VAR(AX0_u16_Out_Scale));

   VAR(AX0_u16_Gear_Skip_Flags) |= COPY_HALLS_FILT_MASK;
}


