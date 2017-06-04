#include "DSP2834x_Device.h"
#include "math.h"

#include "design.def"
#include "Endat.def"
#include "Err_Hndl.def"
#include "Exe_IO.def"
#include "Flash.def"
#include "FltCntrl.def"
#include "FPGA.def"
#include "i2c.def"
#include "Init.def"
#include "ModCntrl.def"
#include "MultiAxis.def"
#include "PhaseFind.def"
#include "Position.def"
#include "PtpGenerator.def"
#include "SensorlessBG.def"
#include "Ser_Comm.def"
#include "Modbus_Comm.def"
#include "SysInit.def"
#include "Velocity.def"
#include "AutoTune.def"
#include "402fsa.def"
#include "ExFbVar.def"

#include "Drive.var"
#include "Exe_IO.var"
#include "Extrn_Asm.var"
#include "Flash.var"
#include "FltCntrl.var"
#include "i2c.var"
#include "ModCntrl.var"
#include "Motor.var"
#include "MotorSetup.var"
#include "PhaseFind.var"
#include "Position.var"
#include "PtpGenerator.var"
#include "SensorlessBG.var"
#include "Units.var"
#include "Velocity.var"
#include "User_Var.var"
#include "Ser_Comm.var"
#include "Modbus_Comm.var"
#include "AutoTune.var"
#include "ExFbVar.var"
#include "Lxm_profile.var"
#include "MotorParamsEst.var"


#include "Prototypes.pro"

extern unsigned int u16_CAN_commands_register, u16_EC_commands_register;
extern int p402_modes_of_operation, p402_modes_of_operation_display;
extern unsigned int* p_u16_tx_nmt_state;

int ProcedureRunning(int drive)
{
   if (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_IDLE) return PROC_MOTOR_SETUP;

   if (BGVAR(s8_BurninParam) != 0) return PROC_BURNIN;

   /**** old motor parameers estimation ***/
   /*
   if ((BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_IDLE) &&
       (BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_DONE) &&
       (BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_FAULT)  )
      return PROC_ML_EST_AUTOTUNE;
   */

   if (BGVAR(u16_Est_Motor_Params))  return PROC_EST_MOTOR_PARAMS;

/*   if ( IsHomingProcessRunning(drive) )
      return PROC_HOMING; */

   if ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_SERVICE_MASK) != 0) return PROC_POSTUNE;
   if (BGVAR(s16_LMJR_Mode) != 0) return PROC_ONLINE_LMJR;

   if (BGVAR(u16_Voltage_Correct_State) != VOLTAGE_CORRECT_STATE_IDLE) return PROC_VOLT_CALIBRATION;

   ++drive;

   return PROC_NONE;
}


#pragma CODE_SECTION(Enabled, "ramfunc_2");
int Enabled(int drive)
{
   REFERENCE_TO_DRIVE;
   // AXIS_OFF;
   return (VAR(AX0_s16_Active_Indication));
}


void EnableDriveHW(int drive)
{
   int s16_open_brake;
   
   // AXIS_OFF;
      
   REFERENCE_TO_DRIVE;
   // IPR 1429: Motor phase monitor activate,but no response when motor UVW lost and in standstill status.
   // on lxm26/28 perform motor phase disconnection test when enabling the dive (if brake is defined).
   if (((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)) &&
       (BGVAR(u16_Brake_Output_Is_Defined) && (VAR(AX0_u16_BrakeOn) == 1)))
   {
      // init state machine for motr phase disconnect test on drive enable
      BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_CHANGE_OPMODE;
      BGVAR(u16_Allow_Motion) = 0;
      s16_open_brake = 0;
   }
   else
   {
      s16_open_brake = 1;
   }
   
   VAR(AX0_s16_Transition_To_Enable) |= ASK_HW_ENABLE_MASK;  // Signal the real-time to enable the PWM

   if (s16_open_brake == 1)
   {
      // On MENCTYPES 1-4 the PHASEFIND state machine will dictate the brake release
      // Otherwise release the brake
      // Valid for Phasefindmodes 0 and 3 (Voltage pulse)
      if ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_REQUESTED_MASK) && ( (BGVAR(u16_PhaseFind_Mode) == 0) || (BGVAR(u16_PhaseFind_Mode) == 3) ))
      {
         if (BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_SUCCESS_MASK)  VAR(AX0_u16_BrakeOn) = 0;
      }
      else
         VAR(AX0_u16_BrakeOn) = 0;
   }   

   BGVAR(u16_AD_State) = AD_IDLE;
   BGVAR(u16_AD_In_Process) = 0;
}


void DisableControl(int drive)
{
   // AXIS_OFF;
   int nltune_disable = 0;
   long long s64_temp;
   float f_temp;
   unsigned int u16_temp;

   // On SE "DISTIME" is composed from P8-43+uiBrakeCouplingTime (read from the MTP) on CDHD it will be equal to DISTIME (as u16_Motor_Brake_Engage_Time==0)
   long s32_disable_time = (long)BGVAR(u16_DisTime) + (long)BGVAR(u16_Motor_Brake_Engage_Time);

   if ( ((BGVAR(s64_SysNotOk)   & BGVAR(u64_Fault_Action_Disable)) != 0x00LL)  ||
        ((BGVAR(s64_SysNotOk_2) & BGVAR(u64_Fault_Action_Disable_2)) != 0x00LL)  ) // Check for Faults
      BGVAR(s16_DisableInputs) |= FLT_EXISTS_MASK;
   else
      BGVAR(s16_DisableInputs)  &= ~FLT_EXISTS_MASK;

   if (Security_Released <= 0) BGVAR(s16_DisableInputs) |= FLT_EXISTS_MASK;

   // Set ENC_STATE disable inputs bit if encoder init not done
   if ( (FEEDBACK_WITH_ENCODER) && (!BGVAR(u16_Motor_Setup_Allow_Enable))                            &&
        ( (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_0_SW_SINE_ENC_CD & 0xffff))       || //Changed to avoid Remark
//          (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_0_SW_SINE_ENC_ENDAT & 0xffff))    ||  //Changed to avoid Remark
//          (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_0_SW_SINE_ENC_STEGMANN & 0xffff)) ||  //Changed to avoid Remark
//        Removed testing for incomplete EnDat or HiperFace processes
//        to avoid redundant Drive Disable causes.
          (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_0_INC_ENC & 0xffff))              ||  //Changed to avoid Remark
          ( (VAR(AX0_s16_Motor_Enc_Type) == 11) && (VAR(AX0_u16_Tamagawa_Timer) != 0xFFFF) )        )  )
      BGVAR(s16_DisableInputs) |= ENC_STATE_DIS_MASK;
   else
      BGVAR(s16_DisableInputs) &= ~ENC_STATE_DIS_MASK;

   BGVAR(s16_DisableInputs)  &= ~RESOLVER_FDBK_DIS_MASK;
   if (LVAR(AX0_s32_Feedback_Ptr) == &SW_RESOLVER_FEEDBACK)
   {
      if (BGVAR(s16_Adjust_Reference_State) != ADJ_REF_DONE)
      BGVAR(s16_DisableInputs)  |= RESOLVER_FDBK_DIS_MASK;
   }

   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command)) BGVAR(s16_DisableInputs) &= ~(ENC_STATE_DIS_MASK | RESOLVER_FDBK_DIS_MASK);

   ReadRemoteCommand(&s64_temp, drive);   // Check for Remote (HW Enable)
   BGVAR(u16_Remote_Indication) = (unsigned int)s64_temp;
   if (s64_temp == 0LL)
      BGVAR(s16_DisableInputs) |= HW_EN_MASK;
   else
      BGVAR(s16_DisableInputs) &= ~HW_EN_MASK;

   ReadEmergencyStopCommand(&s64_temp, drive);   // Check for Emergency Stop
   BGVAR(u16_EStop_Indication) = (unsigned int)s64_temp;
   if (s64_temp == 1LL)
   {
      BGVAR(s16_DisableInputs) |= EMERGENCY_STOP_MASK;
      if ( (BGVAR(u16_Pfb_Backup_Mode) != 0)             &&
           (BGVAR(s16_PfbBackupWR) == PFB_BACKUP_WR_DONE)  )
         BGVAR(s16_PfbBackupWR) = PFB_BACKUP_WR_CALC_ADDR;
   }
   else
      BGVAR(s16_DisableInputs) &= ~EMERGENCY_STOP_MASK;

   // Test valid feedback
   if ( (BGVAR(u16_FdbkType) == 0)                                      &&
        (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) != &VAR(AX0_s16_Burnin_Command))  )
      BGVAR(s16_DisableInputs) |= INVALID_FDBK_MASK;
   else BGVAR(s16_DisableInputs) &= ~INVALID_FDBK_MASK;

   // Test if 5V_FLT is recieved when axis is not moving and enabled - if so maintain
   // latest commutation angle and current loop input. This is to handle the robot drop in
   // FC. Handling is now done in RT at HANDLE_5V_FAIL
   if (VAR(AX0_u16_5v_Fault_On_Standstill) && (!Enabled(drive)))
   {
      VAR(AX0_u16_Lock_Position_Flag) = 0;
      SalLockedRotor(0LL, drive);
      VAR(AX0_u16_5v_Fault_On_Standstill) = 0;
   }

   // Check dis->en transition, Occures when s16_DisableInputs==0 and drive is disabled
   // If drive disabled by NLTUNE procedure dont enable the drive for 1 BG
   BGVAR(s16_Nltune_Status) = VAR(AX0_u16_NL_Tune_Status);
   if ( ((BGVAR(s16_Nltune_Status_Prev) == 1) && (BGVAR(s16_Nltune_Status) == 2)) ||
        (VAR(AX0_s16_Transition_To_Disable) == 2)                                   )
      nltune_disable = 1;

   if ((BGVAR(s16_Nltune_Status_Prev) != 6) && (BGVAR(s16_Nltune_Status) == 6))
      u16_Vbl_Exceeded = 1;

   if ( (BGVAR(s16_DisableInputs) == 0) && (!Enabled(DRIVE_PARAM)) &&
        (!nltune_disable) && (0 == BGVAR(s64_RT_Faults))             )
      // Added conditioning on Over-Voltage not in effect to avoid Race Contidions...
   {
      if ( (BGVAR(u8_Sl_Mode) == 0)                               ||
           (ProcedureRunning(DRIVE_PARAM) == PROC_ML_EST_AUTOTUNE)  )
         EnableDriveHW(drive);
      else if (BGVAR(s16_Sensorless_Handler_State) == SL_STATE_IDLE)
         BGVAR(s16_Sensorless_Handler_State) = SL_STATE_ENABLE_REQUEST;
    // Capture velocity at enabling
    f_temp = (float)VAR(AX0_s16_Vel_Var_Fb_0);
    // Abs(Vel)
    if (f_temp < 0.0) f_temp = -f_temp;
    // Calculate Bemf voltage = Vel*Bemf Constant
    f_temp = (float)((unsigned long)(f_temp*(float)VAR(AX0_s16_Crrnt_Coef_Bemf_Comp_Fix))>>VAR(AX0_s16_Crrnt_Coef_Bemf_Comp_Shr));
    // Convert voltage from internal to external units
    u16_temp = (unsigned int)(f_temp*(float)BGVAR(s16_Vbus_Design)/(float)VAR(AX0_s16_Pwm_Half_Period));
    // Calc 5% from Vbusreadout
    f_temp = (float)BGVAR(u16_Vbus_Volts)*0.05;
    // Use calibration only if Vbemf is less than 5% of Vbusreadout, what indicates low velocity
    // This will help to avoid current spikes at enabling
    // Note: don't use calibration with 4 kHz drives
    // Added support for motor_params_est
    if ((u16_temp <= (unsigned int)f_temp) && (BGVAR(u32_Pwm_Freq) != 4000L) && (VAR(AX0_u16_Motor_Comm_Type) == BRUSHLESS_MOTOR) && (VAR(AX0_u16_Motor_Params_Detection_Process) == 0))
        VAR(AX0_s16_Transition_To_Enable) |= CORR_CAL_REQUIRED_MASK;
    else VAR(AX0_s16_Transition_To_Enable) &= ~CORR_CAL_REQUIRED_MASK;
   }
   // This will cause the brake to lock when NLTUNE terminates
   if (nltune_disable)
   {
      BGVAR(s16_DisableInputs) |= SW_EN_MASK;
      if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
         BGVAR(s16_DisableInputs) |= FB_EN_MASK;
      BGVAR(u16_AD_In_Process) = 1;
      BGVAR(u16_AD_State) = AD_DISABLE_DRIVE;
   }

   // This will disable the drive W/O AD if VBL is too high (NLTUNE==6)
   if (u16_Vbl_Exceeded == 1)
   {
      u16_Vbl_Exceeded = 2;
      BGVAR(s16_DisableInputs) |= SW_EN_MASK;
      if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
         BGVAR(s16_DisableInputs) |= FB_EN_MASK;
      BGVAR(u16_AD_In_Process) = 1;
      BGVAR(u16_AD_State) = AD_START_DISTIME_COUNTER;

      // Divide the knlusergain
      u32_Orig_Knlusergain = BGVAR(u32_Nl_Kpgf);
      SalNlKpgfCommand(((long long)u32_Orig_Knlusergain) / s64_Div_Factor, drive);
   }

   BGVAR(s16_Nltune_Status_Prev) = BGVAR(s16_Nltune_Status);

   // When s16_DisableInputs != 0 and drive is Enabled it means either immediate disable or AD
   if ( ((BGVAR(s16_DisableInputs) != 0) && Enabled(DRIVE_PARAM))                         ||
        (BGVAR(u16_AD_In_Process))                                                        ||
        //if a fault exists while brake is disengaged - enter SM to engage it
        (((BGVAR(s64_SysNotOk) > 0) || (BGVAR(s64_SysNotOk_2) > 0)) && (0 == VAR(AX0_u16_BrakeOn))) ||
        //This is to show external world that drive is still be disabled but internally the drive enabled to perform current calibration
        ((BGVAR(s16_DisableInputs) != 0)                                               &&
         (!Enabled(DRIVE_PARAM))                                                       &&
         (VAR(AX0_s16_Controller_Ptr) == (int)((long)&CURRENT_CONTROLLER_EN & 0xffff)) &&
         ((VAR(AX0_s16_Transition_To_Enable) & CORR_CAL_REQUIRED_MASK) != 0)             )   )
   {
      // Check if AD is needed
      if ( (BGVAR(s16_DisableInputs) & FLT_EXISTS_MASK)                                                      &&
           ( ((BGVAR(s64_SysNotOk)   & BGVAR(u64_Fault_Action_Active_Disable))   != BGVAR(s64_SysNotOk))  ||
             ((BGVAR(s64_SysNotOk_2) & BGVAR(u64_Fault_Action_Active_Disable_2)) != BGVAR(s64_SysNotOk_2))  )  )
      {
         if (VAR(AX0_u16_5v_Fault_On_Standstill))
         {
            if (VAR(AX0_u16_5v_Fault_On_Standstill) == 1)
               BGVAR(u16_AD_State) = AD_START_DISTIME_COUNTER;
            VAR(AX0_u16_5v_Fault_On_Standstill) = 2;
         }
         else
            BGVAR(u16_AD_State) = AD_DISABLE_DRIVE;
      }
      else if (BGVAR(u16_AD_State) == AD_IDLE)
      {
         // Start AD when DISMODE>2 or EMERGENCY_STOP is on
         if ( (BGVAR(u16_Disable_Mode) <= 2)                         &&
              ((BGVAR(s16_DisableInputs) & EMERGENCY_STOP_MASK) == 0)  )
            BGVAR(u16_AD_State) = AD_START_DISTIME_COUNTER_REGULAR;
         else
         {
            BGVAR(u16_AD_State) = AD_START_RAMP_DOWN;
         }
      }
      else // Check if Emergency Stop is issued while AD is in process
      {
         if ( (BGVAR(s16_DisableInputs) & EMERGENCY_STOP_MASK)            &&
              ((BGVAR(s16_Prev_DisableInputs) & EMERGENCY_STOP_MASK) == 0)  )
         {
            VAR(AX0_u16_BrakeOn) = 1;

            if (BGVAR(s32_AD_Overall_Timeout) > s32_disable_time)
               BGVAR(s32_AD_Overall_Timeout) = s32_disable_time;

            BGVAR(s32_AD_Timer) = BGVAR(s32_AD_Start_Time);
         }
      }
      ActiveDisableHandler(drive);
   }

   BGVAR(s16_Prev_DisableInputs) = BGVAR(s16_DisableInputs);

   if (!Enabled(DRIVE_PARAM))
   {
      BGVAR(u16_AD_State) = AD_IDLE;
      BGVAR(u16_AD_In_Process) = 0;

      //if fieldbus changed Dismode (objects 0x605B,0x605C,0x605E) then restore Dismode
      if (BGVAR(u16_Fieldbus_Changed_Dismode))
      {
         BGVAR(u16_Fieldbus_Changed_Dismode) = 0;
         BGVAR(u16_Disable_Mode) = BGVAR(u16_Disable_Mode_User);
      }
   }
}


// On power up, ignore DISMODE to calibrate sensors, then set DISMODE as requested
void CrrntSensorAdjust(int drive)
{
   // AXIS_OFF;
   long s32_actual_velocity = 0;

   s32_actual_velocity = LVAR(AX0_s32_Vel_Var_Fb_0);
   if (s32_actual_velocity < 0L) s32_actual_velocity = -s32_actual_velocity;

   if (!BGVAR(u16_CrrntSensor_Calibrated_Once))
   {
      if (BGVAR(u16_Store_Dismode) != BGVAR(u16_Disable_Mode))
      {
         BGVAR(u16_CrrntSensor_Calibrated_Once) = 1;
         BGVAR(u16_Store_Dismode) = BGVAR(u16_Disable_Mode);
         SalDisableModeCommand(0LL, drive);
      }
      else BGVAR(u16_CrrntSensor_Calibrated_Once) = 2;
   }

   // Suspend adjust if enabled, moving, or on Tamagawa and did not finish
   // Encoder Initialization.  Ignore Encoder Type if Feedback Type other than Encoder.
    if ( (Enabled(DRIVE_PARAM) != 0) || 
        ((VAR(AX0_u16_Motor_Stopped) == 0)  && (0LL == LLVAR(AX0_u32_Inpos_Threshold_Lo)))  ||
        ( (VAR(AX0_s16_Motor_Enc_Type) == 11)                             &&
          (VAR(AX0_u16_Tamagawa_Timer) != 0xFFFF) && FEEDBACK_WITH_ENCODER  )         ||
        ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)                         ||
        ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0)                       )
   {
      BGVAR(u8_Sensor_Adjust_State) = ADJ_WAIT;
      BGVAR(s16_Sensor_Cntr) = 0;
   }

   /* Dont perfrom if adc not ready */
   /* put relevant code here, if needed */

   switch (BGVAR(u8_Sensor_Adjust_State))
   {
      case ADJ_WAIT:  // wait 150 bg before starting
         BGVAR(s16_Sensor_Cntr)++;
         if (BGVAR(s16_Sensor_Cntr) == 150)
         {
            BGVAR(u8_Sensor_Adjust_State) = ADJ_START;
            BGVAR(s16_Sensor_Cntr) = 0;
            BGVAR(s32_Sensor_U) = 0;
            BGVAR(s32_Sensor_V) = 0;
         }
      break;

      case ADJ_START:
         // Collect 256 samples of each current
         if (BGVAR(s16_Sensor_Cntr) == 256 ) BGVAR(u8_Sensor_Adjust_State) = ADJ_CALC;
         else
         {
            BGVAR(s32_Sensor_U) += (VAR(AX0_s16_Crrnt_Srvo_Cmnd_U_Act_0) - VAR(AX0_s16_Crrnt_U_Ofst));
            BGVAR(s32_Sensor_V) += (VAR(AX0_s16_Crrnt_Srvo_Cmnd_V_Act_0) - VAR(AX0_s16_Crrnt_V_Ofst));
         }
         BGVAR(s16_Sensor_Cntr)++;
      break;

      case ADJ_CALC:
         VAR(AX0_s16_Crrnt_U_Ofst) = -(int)(BGVAR(s32_Sensor_U) >> 8);
         VAR(AX0_s16_Crrnt_V_Ofst) = -(int)(BGVAR(s32_Sensor_V) >> 8);
         BGVAR(s16_Sensor_Cntr) = 151;
         BGVAR(u8_Sensor_Adjust_State) = ADJ_WAIT; // Restart calculation
         BGVAR(s16_DisableInputs) &= ~SENSOR_ADJ_DIS_MASK;

         if (BGVAR(u16_CrrntSensor_Calibrated_Once) == 1)
         {
            BGVAR(u16_CrrntSensor_Calibrated_Once) = 2;
            SalDisableModeCommand((long long)BGVAR(u16_Store_Dismode), drive);
         }
      break;

      default:
         BGVAR(u8_Sensor_Adjust_State) = ADJ_WAIT;
      break;
   }
}


void ModeControl(int drive)
{
   AdjustResolverReference(DRIVE_PARAM);
   FaultControl(DRIVE_PARAM);
// this function was disabled because of it was sent cyclic
// and the new design use the CANFaultControl when required
//   CANFaultControl(DRIVE_PARAM);
   DisableControl(DRIVE_PARAM);
   CrrntSensorAdjust(DRIVE_PARAM);
   SineZeroControl(DRIVE_PARAM);
   SineCDInitHandler(DRIVE_PARAM);
   HallsOnlyCommSource(DRIVE_PARAM);

   // Not sure this should be done here
   ScanDigitalOutputs();
   ScanDigitalInputs(drive);
   OpmodeChangeBackgroundHandler(drive);
}


// must be in RAM because used in flash write/erase routines
#pragma CODE_SECTION(PassedTimeMS, "ramfunc_2");
char PassedTimeMS(long i, long capture)
{
   long j;
   
   // fix BZ 6513: FOE - POSLIMMODE update value while downloading txt parameters file in bootstrap state via Twincat
   // this bug is not only for FOE, but caused due to the implementation of PassedTimeMS().
   // if timestamp is taken and right after that PassedTimeMS(1, timestamp) is called,
   // it is possible that if a 1ms interrupt happens in between, the PassedTimeMS() will return true immediately.
   // as a result 1ms will not elapse.
   // see SalPosLimModeCommand() for example, where the while loop will exit immediately.
   if (i == 1L) i++;
   
   j = Cntr_1mS - capture;
   if (j < 0L) j = -j;
   if (j >= i) return 1;
   else return 0;
}


void SineZeroControl(int drive)
{
   // AXIS_OFF;

   int vel, drive_backup = drive, s16_val_0, s16_val_1;
   static int fault_track = 0;
   long long status;
   float f_temp, vel_limit;

   // code relevant for sine encoder and resolver only
   if ( (BGVAR(u16_FdbkType) != RESOLVER_FDBK) && (BGVAR(u16_FdbkType) != SW_SINE_FDBK) ) return;

   if (BGVAR(s16_SinZeroSave) == SAL_NOT_FINISHED)
   { // if during save - call save handler
      BGVAR(s16_SinZeroSave) = SaveSineZeroParams(drive);
      return;
   }
   /* verfiy velocity is not too high for sine zero
   * Max frequncy is 250Hz
   * For Resolver:
   * max_vel = 60*250*1/4=3750 (4 peaks of sine/cosine cycle per rev)
   *
   * For sine encoder
   * max_vel = 60*250/mencres. */

   SalReadSinInitStatusCommand(&status, drive);

   vel = ConvertVelocityToRpm(drive, VAR(AX0_s16_Vel_Var_Fb_0)); // SalReadVelocityCommand
   if (vel < 0) vel = -vel;

   if (BGVAR(u16_FdbkType) == RESOLVER_FDBK) vel_limit = 3750;
   else vel_limit = 60 * 250 / 4 / BGVAR(u32_User_Motor_Enc_Res);

   // if the velocity is ok , sine zero ospd is set - restart process
   if ( ((VAR(AX0_s16_Sine_Enc_Bits) & SINE_ZERO_VEL_TOO_HIGH_MASK) != 0) && (vel < vel_limit) )
   {
      if (status != 0) SininitCommand(drive);
      VAR(AX0_s16_Sine_Enc_Bits) &= ~SINE_ZERO_VEL_TOO_HIGH_MASK;
      return;
   }
   else if (vel > vel_limit)
   {
      VAR(AX0_s16_Sine_Enc_Bits) |= SINE_ZERO_VEL_TOO_HIGH_MASK;
      return;
   }

   if ( !fault_track && ((BGVAR(s64_SysNotOk) != 0)||(BGVAR(s64_SysNotOk_2) != 0)) ) // Restart SinInit after resetting any Fault
      fault_track = 1;
   if ( fault_track && (BGVAR(s64_SysNotOk) == 0) && (BGVAR(s64_SysNotOk_2) == 0) )
   {
      if (status != 0) SininitCommand(drive);
      fault_track = 0;
      return;
   }

   if ((VAR(AX0_s16_Sine_Enc_Bits) & SINE_ZERO_DONE_MASK) == 0) return; // test if process terminated

   VAR(AX0_s16_Sine_Enc_Bits) &= ~SINE_ZERO_DONE_MASK;   // reset done bit

   // calc sine and cosine offsets
   Math32 = (LVAR(AX0_s32_Sin_Neg_Acc) + LVAR(AX0_s32_Sin_Pos_Acc)) >> 8;
   VAR(AX0_s16_Sine_Offset) = -(int)Math32;

   Math32 = (LVAR(AX0_s32_Cos_Neg_Acc) + LVAR(AX0_s32_Cos_Pos_Acc)) >> 8;
   VAR(AX0_s16_Cosine_Offset) = -(int)Math32;

//   f_temp = (((float)(LVAR(AX0_s32_Sin_Pos_Acc)) - (float)(LVAR(AX0_s32_Sin_Neg_Acc)))) / (((float)(LVAR(AX0_s32_Cos_Pos_Acc)) - (float)(LVAR(AX0_s32_Cos_Neg_Acc))));
   f_temp = (((float)(LVAR(AX0_s32_Cos_Pos_Acc)) - (float)(LVAR(AX0_s32_Cos_Neg_Acc)))) / (((float)(LVAR(AX0_s32_Sin_Pos_Acc)) - (float)(LVAR(AX0_s32_Sin_Neg_Acc))));
   FloatToFix16Shift16(&VAR(AX0_s16_Sine_Gain_Fix),(unsigned int *) &VAR(AX0_s16_Sine_Gain_Shr), f_temp);

   /* for sw resolver calculate the gain corection (fix and shr) to normalize
   * the sampled sine and cosine with the sin(swr2d comm) and cos(swr2d comm)
   * this is needed for the tracking filter calculations.
   * since only the sine is multiplied by the fix and shift values, matching the gain
   * can be done by matching the cos peak value (with offset) with 7FFF */

   f_temp = 30720.0 / ((float)(LVAR(AX0_s32_Cos_Pos_Acc) >> 7) + (float)(VAR(AX0_s16_Cosine_Offset)));
   FloatToFix16Shift16(&VAR(AX0_s16_Swr2d_Fix), (unsigned int *)&VAR(AX0_s16_Swr2d_Shr), f_temp);

   drive = 0;
   s16_val_0 = BGVAR(s16_SineZeroSaveState);
   drive = 1;
   s16_val_1 = BGVAR(s16_SineZeroSaveState);
   drive = drive_backup;
   if ( (s16_val_0 == 0) && (s16_val_1 == 0) )
      BGVAR(s16_SinZeroSave) = SaveSineZeroParams(drive);
}


//**********************************************************
// Function Name: SininitCommand
// Description:
//          This function is called in response to the SININIT command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SininitCommand(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if ( (LVAR(AX0_s32_Feedback_Ptr) != &SW_RESOLVER_FEEDBACK) &&
        (LVAR(AX0_s32_Feedback_Ptr) != &SW_SINE_ENC_FEEDBACK)   )
      return NOT_AVAILABLE;

   VAR(AX0_s16_Sine_Offset) = 0;
   VAR(AX0_s16_Cosine_Offset) = 0;
   VAR(AX0_s16_Sine_Gain_Fix) = 0x4000;
   VAR(AX0_s16_Sine_Gain_Shr) = 14;
   VAR(AX0_s16_Swr2d_Fix) = 0x4000;
   VAR(AX0_s16_Swr2d_Shr) = 14;
   VAR(AX0_s16_Sine_Enc_Bits) |= RESET_SINE_ZERO_MASK; // enable sine zero process
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadSinInitStatus
// Description:
//          This function is called in response to the SININITST command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalReadSinInitStatusCommand(long long* param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if ((VAR(AX0_s16_Sine_Enc_Bits) & SINE_ZERO_RUNNING_MASK) != 0)
   {
      *param = 1LL; // running
      if ((VAR(AX0_s16_Sine_Enc_Bits) & SINE_ZERO_VEL_TOO_HIGH_MASK) != 0)
         *param = 2LL; // speed too high
   }
   
   else if (0 < (BGVAR(s64_SysNotOk) & SININIT_INVALID_MASK))
      *param = 3LL; //sininit failed
      
   else *param = 0LL;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SinparamCommand
// Description:
//          This function is called in response to the SINPARAM command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SinparamCommand(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Sine_Offset), 4);
   PrintChar(SPACE);
   PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Cosine_Offset), 4);
   PrintChar(SPACE);
   PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Sine_Gain_Fix), 4);
   PrintChar(SPACE);
   PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Sine_Gain_Shr), 4);
   if (LVAR(AX0_s32_Feedback_Ptr) == &SW_RESOLVER_FEEDBACK)
   {
      PrintChar(SPACE);
      PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Swr2d_Fix), 4);
      PrintChar(SPACE);
      PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Swr2d_Shr), 4);
   }
   PrintCrLf();
   return (SAL_SUCCESS);
}


void SetOpmodePosition(int drive)
{
   // AXIS_OFF;

   // saturation block input - irrelevant. saturation is done in vel loop
   // set crrnt controller input
   if (IS_DUAL_LOOP_ACTIVE)
   {
      //Set Current Pointer
      SetCurrLoopPtr(drive,VAR(AX0_s16_Opmode),0);
      // set vel loop input source as the acc_dec block output
      VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Dual_Loop_LinearVel_Vcmd) & 0xffff);
   }  
  
   else if (VAR(AX0_u16_Pos_Control_Mode) != LINEAR_POSITION)
   {
      //Set Current Pointer
      SetCurrLoopPtr(drive,VAR(AX0_s16_Opmode),0);
   }
   else
   {
      //Set Current Pointer
      SetCurrLoopPtr(drive,VAR(AX0_s16_Opmode),0);
      // set vel loop input source as the acc_dec block output
      VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Pos_Vcmd) & 0xffff);
   }

   VAR(AX0_s16_Weight_Comp_Source_Ptr_Design) = (int)((long)&VAR(AX0_s16_Delta_PCMD) & 0xffff);

   SetupGear(drive);

   POS_LOOP_VCMD_PTR_ASSIGN((int)((long)&LVAR(AX0_s32_Pos_Vcmd) & 0xffff));
   //VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Pos_Vcmd) & 0xffff);
   VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Delta_Delta_Pos_Cmd) & 0xffff);
   VAR(AX0_s16_Hold_Pos_Acccmnd_Ptr_Design) = VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr);
   VAR(AX0_s16_Pre_Pos_Input_Ptr_Design) = (int)((long)&LVAR(AX0_u32_Pos_Cmd_Ptp_Lo) & 0xffff);

   //BGVAR(s64_Faults_Mask) &= ~SEC_LINE_BRK_FLT_MASK;  // Disable detection of 2nd Encoder Broken Wires.

   // If DS402 modes of operation in synchronous position mode AND if COMMODE set to 1
   // nitsan - fix bugzilla 4400: p402_modes_of_operation_display is being updated only after opmode change is completed.
   //          so use p402_modes_of_operation here.
//   if ((p402_modes_of_operation == CYCLIC_SYNCHRONOUS_POSITION_MODE) && BGVAR(u8_Comm_Mode))
   if ((IS_CANOPEN_SYNC_POS_MODES) && BGVAR(u8_Comm_Mode))
   {
      // fix bug IPR 396: Uncontrolled movement of the axis after switching the op-mode from "6 = homimg" to "8 = cyclic synchronous position"
      // in case of changing opmodes while hold is active, need to update the back up copy of target pos (fieldbus or internal PTP)
      // e.g. while homing in progress, limit-switch occurs, next, master changes opmode to 8 (sync position) and then clears limit-switch alarm.
      // in this case the tagrget positon pointer should be updated back to fieldbus only after limit-switch is cleared
      if (VAR(u16_Hold) == 0)
      {
         VAR(AX0_s16_Pre_Pos_Input_Ptr_Design) = (int)((long)&LVAR(AX0_u32_Pos_Cmd_Ptp_Lo) & 0xffff);
      }
      else
      {
         VAR(AX0_s16_Pre_Pos_Input_Ptr_Store) = (int)((long)&LVAR(AX0_u32_Pos_Cmd_Ptp_Lo) & 0xffff);
      }
   }

   return;
}

void SetPosLoopMainControllerVcmdPtr (int drive, int opmode)
{
   AXIS_OFF;
   REFERENCE_TO_DRIVE;

   switch (opmode)
   {
      case 0: // Serial Velocity
      case 1: // Analog Velocity
         if (((BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7)))
            VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design) = (int)((long)&LVAR(AX0_s32_NL_Vel_Loop_Cmd_Src) & 0xffff);
         else
            VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Pos_Vcmd) & 0xffff);
      break;
      
      case 4:
      case 8:
         if (IS_DUAL_LOOP_ACTIVE)
            VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design) = (int)((long)&VAR(AX0_s32_Dual_Loop_Vcmd) & 0xFFFF);
         else 
            VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design) = VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr_Design);
      default:
      break;
   }
}

void SetVelocityLoopPtr(int drive, int opmode)
{
   AXIS_OFF;
   REFERENCE_TO_DRIVE;

   switch (opmode)
   {
      case 0: // Serial Velocity
      case 1: // Analog Velocity
         VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Vel_Acc_Dec_Cmnd_Filtered) & 0xffff);
      break;
      
      case 4:
      case 8:
         if (IS_DUAL_LOOP_ACTIVE)
            VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Dual_Loop_LinearVel_Vcmd) & 0xffff);
         else 
            VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Pos_Vcmd) & 0xffff);
      default:
      break;
   }
}  

// Call this function to set current loop pointers only
// if you call this function outside of SetOpmode, then set ontheflybit to 1, otherwise to 0
void SetCurrLoopPtr(int drive, int opmode, int ontheflybit)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   switch (opmode)
   {
      case 0: // Serial Velocity
      case 1: // Analog Velocity
         // saturation block input - irrelevant. saturation is done in vel loop
         // set crrnt controller input
         if ((BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7))
         {
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design) = &VAR(AX0_s16_Nct_Icmd);
         }
         else LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design) = &VAR(AX0_s16_Crrnt_Lp_Inp);
      break;

      case 2: // Serial Torque
         // set crrnt controller input
         if ((p402_modes_of_operation == CYCLIC_SYNCHRONOUS_TORQUE_MODE) && BGVAR(u8_Comm_Mode))
         {
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design) = (int*)((long)&LVAR(AX0_u32_FieldBus_Int_Lo));
         }
         else
         {
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design) = &VAR(AX0_s16_Serial_Crrnt_Cmnd);
         }
      break;

      case 3: // Analog Torque
         LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design) = &VAR(AX0_s16_Analog_Crrnt_Cmnd);
      break;

      case 4:
      case 8:
         if (IS_DUAL_LOOP_ACTIVE)
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design) = &VAR(AX0_s16_Crrnt_Lp_Inp);
         else if (VAR(AX0_u16_Pos_Control_Mode) != LINEAR_POSITION)
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design) = &VAR(AX0_s16_Nct_Icmd);
         else
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design) = &VAR(AX0_s16_Crrnt_Lp_Inp);
      break;

      default:
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design)     = LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr);
      break;
   }
      if (1 == ontheflybit)
      {
           LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr)            = LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design);
      }

}

void SetOpmode(int drive, int opmode)
{
   // AXIS_OFF;
   int s16_prev_opmode = VAR(AX0_s16_Opmode);
   long s32_timeout_cntr;
   unsigned int u16_copy_position_design = 0;

   VAR(AX0_s16_Opmode) = opmode;
   
   UpdateAccDec(drive,ACC_UPDATE);
   UpdateAccDec(drive,DEC_UPDATE);

   RestoreAccDecInternalVal(drive);

   LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design)     = LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr);
   VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr_Design)    = VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr);
   VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design)     = VAR(AX0_s16_Vel_Loop_Cmnd_Ptr);
   VAR(AX0_s16_Acc_Dec_Cmnd_Ptr_Design)      = VAR(AX0_s16_Acc_Dec_Cmnd_Ptr);
   VAR(AX0_s16_Weight_Comp_Source_Ptr_Design)= VAR(AX0_s16_Weight_Comp_Source_Ptr);
   VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr_Design)  = VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr);
   VAR(AX0_s16_Hold_Pos_Acccmnd_Ptr_Design)  = VAR(AX0_s16_Hold_Pos_Acccmnd_Ptr);
   VAR(AX0_s16_Pre_Pos_Input_Ptr_Design)     = VAR(AX0_s16_Pre_Pos_Input_Ptr);
   VAR(AX0_s16_KPAFR_Source_Ptr_Design)      = VAR(AX0_s16_KPAFR_Source_Ptr);
   VAR(AX0_s16_KPVFR_Source_Ptr_Design)      = VAR(AX0_s16_KPVFR_Source_Ptr);
   VAR(AX0_s16_HP_Pos_Loop_Vcmnd_Ptr_Store_Design) = VAR(AX0_s16_HP_Pos_Loop_Vcmnd_Ptr_Store);
   VAR(AX0_s16_Pre_Pos_Input_Ptr_Store_Design) = VAR(AX0_s16_Pre_Pos_Input_Ptr_Store);

   // As the analogue hanling sample rate depends on the opmode, need to recall the design functions to setup the filter coefs
   if ((VAR(AX0_s16_Opmode)==1) || (VAR(AX0_s16_Opmode)==3)) VAR(u16_Analogue_Lo_Sample_Rate) = 0;
   else VAR(u16_Analogue_Lo_Sample_Rate) = 1;
   AnalogInputConfig1(drive);
   AnalogInputConfig2(drive);
   SetPosLoopMainControllerVcmdPtr(drive, opmode);

   switch (opmode)
   {
      case 0: // Serial Velocity
      case 1: // Analog Velocity
         // saturation block input - irrelevant. saturation is done in vel loop
         // set crrnt controller input
         // ATTENTION!!! If you're going to change any condition in the "IF" below, do the same change in SetCurrLoopPtr function
         if ((BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7))
         {
            if (opmode == 0) SetOpmodePosition(drive);
            //Set Current Pointer
            SetCurrLoopPtr(drive,opmode,0);
            //VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr_Design) = (int)((long)&LVAR(AX0_s32_NL_Vel_Loop_Cmd_Src) & 0xffff);
			POS_LOOP_VCMD_PTR_ASSIGN((int)((long)&LVAR(AX0_s32_NL_Vel_Loop_Cmd_Src) & 0xffff));
            // If changed from position to velocity call the position desing again to use the velocity parameters
            if ((s16_prev_opmode == 8) || (s16_prev_opmode == 4))
            {
               PositionConfig(drive, 2);
               u16_copy_position_design = 1;
            }
         }
         else
         {
            //Set Current Pointer
            SetCurrLoopPtr(drive,opmode,0);
         }

         // set vel loop input source as the acc_dec block output
         VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Vel_Acc_Dec_Cmnd_Filtered) & 0xffff);

         if (opmode == 0) // Set acc_dec block input as the serial command
         {
            if ((p402_modes_of_operation == CYCLIC_SYNCHRONOUS_VELOCITY_MODE) && BGVAR(u8_Comm_Mode))
            {
               VAR(AX0_s16_Acc_Dec_Cmnd_Ptr_Design) = (int)((long)&LVAR(AX0_u32_FieldBus_Int_Lo) & 0x0000ffff);
            }
            else
            {
               VAR(AX0_s16_Acc_Dec_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Serial_Vel_Cmnd)  & 0xffff);
            }
         }
         else // Set acc_dec block input as the analog command
         {
            VAR(AX0_s16_Acc_Dec_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Analog_Vel_Cmnd) & 0xffff);
         }

         VAR(AX0_s16_Weight_Comp_Source_Ptr_Design) = (int)((long)&VAR(AX0_s16_Vel_Var_Ref_0) & 0xffff);

         //BGVAR(s64_Faults_Mask) &= ~SEC_LINE_BRK_FLT_MASK;  // Disable detection of 2nd Encoder Broken Wires.
      break;

      case 2: // Serial Torque
         // set crrnt controller input
         //Set Current Pointer
         SetCurrLoopPtr(drive,opmode,0);
         //BGVAR(s64_Faults_Mask) &= ~SEC_LINE_BRK_FLT_MASK;  // Disable detection of 2nd Encoder Broken Wires.
      break;

      case 3: // Analog Torque
         //Set Current Pointer
         SetCurrLoopPtr(drive,opmode,0);
         //BGVAR(s64_Faults_Mask) &= ~SEC_LINE_BRK_FLT_MASK;  // Disable detection of 2nd Encoder Broken Wires.
      break;

      case 4: // Gearing
         //Set Current Pointer
         SetCurrLoopPtr(drive,opmode,0);

         // set vel loop input source as the acc_dec block output
         if (IS_DUAL_LOOP_ACTIVE)
         {
            //VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design) = (int)((long)&VAR(AX0_s32_Dual_Loop_Vcmd) & 0xffff);
            VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Dual_Loop_LinearVel_Vcmd) & 0xffff);     
         }    
         else
         {
            //VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Pos_Vcmd) & 0xffff);
            VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design) = (int)((long)&VAR(AX0_s16_Pos_Vcmd) & 0xffff);
         }
         SetupGear(drive);

         VAR(AX0_s16_Weight_Comp_Source_Ptr_Design) = (int)((long)&VAR(AX0_s16_Delta_PCMD) & 0xffff);

         //VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Pos_Vcmd) & 0xffff);
		 POS_LOOP_VCMD_PTR_ASSIGN((int)((long)&LVAR(AX0_s32_Pos_Vcmd) & 0xffff));
         // When using movesmooth, MSQ ACC cannot be used as the smoothing will change it
         if ( VAR(AX0_u16_Qep_Msq_Fltr_Mode) && ((VAR(AX0_u16_Move_Smooth_Mode)==0) || ((VAR(AX0_u16_Move_Smooth_Mode)==1) && (BGVAR(s16_Move_Smooth_Lpf_Hz)==5000)) || ((VAR(AX0_u16_Move_Smooth_Source) & 0x02)==0)))
            VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Qep_Msq_Fltr_Acc_Out) & 0xffff);
         else
            VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Delta_Delta_Pos_Cmd) & 0xffff);

         VAR(AX0_s16_Hold_Pos_Acccmnd_Ptr_Design) = VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr);

         // Switch the pointer of the position command variable to the output of the PTP hunting generator,
         // since the PTP hunter is now running after the electronic gearing function block.
         VAR(AX0_s16_Pre_Pos_Input_Ptr_Design) = (int)((long)&LVAR(AX0_u32_Pos_Cmd_Ptp_Lo) & 0xffff);

         // If changed from velocity to to position (NCT) reconfigure position with position params
         if (((s16_prev_opmode == 0) || (s16_prev_opmode == 1)) &&
            ((BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7)) &&
            (VAR(AX0_u16_Pos_Control_Mode) != LINEAR_POSITION))
         {
               PositionConfig(drive, 2);
               u16_copy_position_design = 1;
         }
      break;

      case 8: // Serial Position
         SetOpmodePosition(drive);

         // If changed from velocity to to position (NCT) reconfigure position with position params
         if (((s16_prev_opmode == 0) || (s16_prev_opmode == 1)) &&
            ((BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7)) &&
            (VAR(AX0_u16_Pos_Control_Mode) != LINEAR_POSITION))
         {
            PositionConfig(drive, 2);
            u16_copy_position_design = 1;
            if (!IS_DUAL_LOOP_ACTIVE)
            {
               VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design) = VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr_Design);
            }
            else
            {
               VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design) = (int)((long)&VAR(AX0_s32_Dual_Loop_Vcmd) & 0xFFFF);
            }            
         }
      break;

      case 12: // ? set opmode to open loop
         //BGVAR(s64_Faults_Mask) &= ~SEC_LINE_BRK_FLT_MASK;  // Disable detection of 2nd Encoder Broken Wires.
      break;
   }

   DetermineDetectionOfPDBrokenWire(drive);

   if ((VAR(AX0_s16_Opmode) == 4) && VAR(AX0_u16_Qep_Msq_Fltr_Mode) && (VAR(AX0_u16_Move_Smooth_Mode)==0))
   {
      VAR(AX0_s16_KPAFR_Source_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Gear_Acc_Out) & 0xffff);
      VAR(AX0_s16_KPVFR_Source_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Gear_Vel_Out) & 0xffff);
   }
   else
   {
      VAR(AX0_s16_KPAFR_Source_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Delta_Delta_Pos_Cmd) & 0xffff);
      VAR(AX0_s16_KPVFR_Source_Ptr_Design) = (int)((long)&LVAR(AX0_s32_Delta_Pos_Cmd) & 0xffff);
   }

   IsLinearVelocityNeeded(drive);
   IsPositionLoopNeeded(drive);

   // Handle the case where opmode is changed while HOLD = 1
   VAR(AX0_s16_HP_Pos_Loop_Vcmnd_Ptr_Store_Design) = VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr_Design);
   VAR(AX0_s16_Pre_Pos_Input_Ptr_Store_Design) = VAR(AX0_s16_Pre_Pos_Input_Ptr_Design);
   // Mangaging the use of the torque slope
   IcmdSlopeEn(drive);

   // Check in which mode the PTP hunter is supposed to run. Variable update needs
   // to be done within the OPMODE change RT function (function argument = 0). This
   // function call must be done prior to the OPMODE change trigger in RT.
   CheckPtpHunterMode(drive, 0);

   if (u16_copy_position_design) // If positon config was called, raise a RT bit to indicate copy of design->operative vars
      VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_ISSUED_MASK | OPMODE_CHAGE_POS_DESIGN_COPY_MASK;
   else
      VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_ISSUED_MASK;

   // Wait for the RT to copy the Design pointers to operational ones (+ timeout)
   if (u16_background_ran) // As SetOpmode is called from the init as well avoid this as the RT is not running yet
   {
      s32_timeout_cntr = Cntr_1mS;
      while ((VAR(AX0_s16_Opmode_Change_Flags) & OPMODE_CHANGE_ISSUED_MASK) && (!PassedTimeMS(3L,s32_timeout_cntr)));

      if (u16_copy_position_design)
         while ((VAR(AX0_s16_Crrnt_Run_Code) & COPY_NCT_COEF_MASK) && (!PassedTimeMS(3L,s32_timeout_cntr)));
   }
   else // On init copy the design to operational wo the RT
   {
      LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr)            = LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design);
      VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr)           = VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr_Design);
      VAR(AX0_s16_Vel_Loop_Cmnd_Ptr)            = VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design);
      VAR(AX0_s16_Acc_Dec_Cmnd_Ptr)             = VAR(AX0_s16_Acc_Dec_Cmnd_Ptr_Design);
      VAR(AX0_s16_Weight_Comp_Source_Ptr)       = VAR(AX0_s16_Weight_Comp_Source_Ptr_Design);
      VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr)         = VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr_Design);
      VAR(AX0_s16_Hold_Pos_Acccmnd_Ptr)         = VAR(AX0_s16_Hold_Pos_Acccmnd_Ptr_Design);
      VAR(AX0_s16_Pre_Pos_Input_Ptr)            = VAR(AX0_s16_Pre_Pos_Input_Ptr_Design);
      VAR(AX0_s16_KPAFR_Source_Ptr)             = VAR(AX0_s16_KPAFR_Source_Ptr_Design);
      VAR(AX0_s16_KPVFR_Source_Ptr)             = VAR(AX0_s16_KPVFR_Source_Ptr_Design);
      VAR(AX0_s16_HP_Pos_Loop_Vcmnd_Ptr_Store)  = VAR(AX0_s16_HP_Pos_Loop_Vcmnd_Ptr_Store_Design);
      VAR(AX0_s16_Pre_Pos_Input_Ptr_Store)      = VAR(AX0_s16_Pre_Pos_Input_Ptr_Store_Design);
      VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr) = VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design);
   }

   // When the drive is at hold state and the operating mode has changed, but not
   // caused by the HoldScan function itself (feature velocity controlled ramp-down
   // in OPMODE torque), then the ActivateHold function will be reactivated .
   // This code is needed specifically when being in OPMODE position because a
   // restore of position-control pointers is triggered upon the HOLD termination
   // and for this we need to initialize the shadow pointers one time with the
   // proper pointers (see DIVERT_POS_LOOP_VCMD_PTR & RESTORE_POS_LOOP_VCMD_PTR).
   // Whether the rotor is in movement or not, the original decelarations values
   // will be restored here..
   if ((BGVAR(u16_Hold) != 0) && (IsVelControlledRampDownInOpmodeTrqActive(drive) == 0))
   {  ActivateHold(drive);  }
}

//******************************************************************************
// Function Name: IsOpmodeChangeInProgress
// Description:
//          This function is needs to be called in order to identify if an OPMODE
//          change process is currently running. This function has been added
//          based on requirements for a Lexium OPMODE change.
//
// Author: APH
// Algorithm:
// Revisions:
//******************************************************************************
int IsOpmodeChangeInProgress(int drive)
{
   int s16_return_value = 0; // No OPMODE change pending

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If someone tries to initiate an OPMODE change process (via the new method)
   // or if the OPMODE change state machine is currently running.
   if ((BGVAR(s16_Start_Opmode_Change) != -1) || (BGVAR(u16_Opmode_Change_State) != OPMODE_CHANGE_IDLE))
   {
      s16_return_value = 1; // OPMODE change pending
   }
   // If the fieldbus will trigger an OPMODE change process in an upcoming BG cycle
   else if (BGVAR(s16_CAN_Flags) & CANOPEN_OPMODE_CHANGED_MASK)
   {
      s16_return_value = 1; // OPMODE change pending (here via fieldbus)
   }

   return (s16_return_value);
}

//******************************************************************************
// Function Name: OpmodeChangeBackgroundHandler
// Description:
//          This function is called continuously from Background and handles
//          the different OPMODE change modes. The modes are called:
//          a) Standstill - First come to a stop, then change the OPMODE.
//          b) On the fly - Smooth transition during the OPMODE change while the
//                          motor is moving.
//
// Author: APH
// Algorithm:
// Revisions:
//******************************************************************************
void OpmodeChangeBackgroundHandler(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If someone raises an CDHD OPMODE change request
   if (BGVAR(s16_Start_Opmode_Change) != -1)
   {
      // If we are in OPMODE change idle state -> Accept the request
      if (BGVAR(u16_Opmode_Change_State) == OPMODE_CHANGE_IDLE)
      {
         // Do nothing if CDHD OPMODE request via the Sal-function is the same.
         if ( ((BGVAR(s16_Start_Opmode_Change) & 0xFF00) == OPMODE_CHANGE_SOURCE_SAL_FCT) &&
             ((BGVAR(s16_Start_Opmode_Change) & 0x00FF) == VAR(AX0_s16_Opmode)) )
         {
            // Do nothing
         }
         // Do nothing if the HoldScan function temporarily switched to OPMODE 0
         // in order to ramp down to zero velocity.
         else if (BGVAR(u16_Block_Opmode_Change) & BLOCK_OPMODE_CHANGE_BY_HOLD)
         {
            // Do nothing
         }
         else
         {
            // Apply the request
            BGVAR(s16_Start_Opmode_Change_Capture) = BGVAR(s16_Start_Opmode_Change);
         }
      }
      BGVAR(s16_Start_Opmode_Change) = -1; // Clear the OPMODE change request / do not accept it
   }

   switch (BGVAR(u16_Opmode_Change_State))
   {
      case OPMODE_CHANGE_IDLE:
         if (BGVAR(s16_Start_Opmode_Change_Capture) != -1)
         {
            if (BGVAR(u16_Opmode_Change_Mode) == 0) // If we want to perform an OPMODE change in standstill mode
            {
               BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_START_STANDSTILL_PROCESS;
            }
            else // Perform an OPMODE change on the fly if allowed
            {
               if ( (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)                                   &&
                   ((BGVAR(s16_Start_Opmode_Change_Capture) & 0xFF00) == OPMODE_CHANGE_SOURCE_CAN_ECAT)        &&
                   ( (BGVAR(s16_CAN_Opmode_Temp) == PROFILE_VELOCITY_MODE)                               ||
                     (BGVAR(s16_CAN_Opmode_Temp) == PROFILE_TORQUE_MODE)                                 ||
                     (BGVAR(s16_CAN_Opmode_Temp) == CYCLIC_SYNCHRONOUS_VELOCITY_MODE)                    ||
                     (BGVAR(s16_CAN_Opmode_Temp) == CYCLIC_SYNCHRONOUS_TORQUE_MODE)                      ||
                     (BGVAR(s16_CAN_Opmode_Temp) == PROFILE_POSITION_MODE)                               ||
                     (BGVAR(s16_CAN_Opmode_Temp) == CYCLIC_SYNCHRONOUS_POSITION_MODE)                    ||
                     (BGVAR(s16_CAN_Opmode_Temp) == INTRPOLATED_POSITION_MODE)                           ||
                     (BGVAR(s16_CAN_Opmode_Temp) == ANALOG_TORQUE_MODE)                                  ||
                     (BGVAR(s16_CAN_Opmode_Temp) == JOG_MODE)                                            ||
                     (BGVAR(s16_CAN_Opmode_Temp) == ANALOG_VEL_MODE)
                   )
                 )
               {
                  /*******************************************************************************************************************/
                  /*** At this place the CAN or EtherCAT fieldbus triggered an OPMODE change on-the-fly to one of the above modes. ***/
                  /*******************************************************************************************************************/

                  VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_ON_THE_FLY_MASK;         // Indicate to the RT task that we want an OPMODE change on the fly
                  BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_SWITCH_OPMODE_AFTER_VEL0;   // Call directly the required "SetOpmode" function
               }
               else if ( (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)                              &&
                        ((BGVAR(s16_Start_Opmode_Change_Capture) & 0xFF00) == OPMODE_CHANGE_SOURCE_CAN_ECAT)   &&
                        ((BGVAR(s16_CAN_Opmode_Temp) == GEAR_MODE) && (VAR(AX0_u8_Gear_Limits_Mode) & 0x0C)) // In speed-gear mode or if PTP hunter applies ACC/DEC/VLIM
                      )
               {
                  // Indicate to the RT task that we want an OPMODE change on the fly
                  VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_ON_THE_FLY_MASK;
                  // Indicate that we want to overwrite PTPVCMD while the move-smooth filters are in the initialization process.
                  // This is only allowed for speed-gear mode.
                  if (VAR(AX0_u8_Gear_Limits_Mode) & 0x08)
                  {
                     // Overwrite PTPVCMD during filter-init in speed-gear mode
                     VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_OVERWRITE_PTPVCMD_DURING_FILTER_INIT_MASK;
                  }
                  BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_SWITCH_OPMODE_AFTER_VEL0;   // Call directly the required "SetOpmode" function
               }
               else if ( ((BGVAR(s16_Start_Opmode_Change_Capture) & 0xFF00) == OPMODE_CHANGE_SOURCE_DUAL_MODE_IO) &&
                        ( ((BGVAR(s16_Start_Opmode_Change_Capture) & 0x00FF) < 4) ||
                          (((BGVAR(s16_Start_Opmode_Change_Capture) & 0x00FF) == 4) && (VAR(AX0_u8_Gear_Limits_Mode) & 0x0C)) // In speed-gear mode or if PTP hunter applies ACC/DEC/VLIM
                        )
                      )
               {
                  VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_ON_THE_FLY_MASK;         // Indicate to the RT task that we want an OPMODE change on the fly

                  // Indicate that we want to overwrite PTPVCMD in OPMODE 4 while the move-smooth filters
                  // are in the initialization process. This is only allowed for speed-gear mode.
                  if ((VAR(AX0_u8_Gear_Limits_Mode) & 0x08) && ((BGVAR(s16_Start_Opmode_Change_Capture) & 0x00FF) == 4))
                  {
                     // Overwrite PTPVCMD during filter-init in speed-gear mode
                     VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_OVERWRITE_PTPVCMD_DURING_FILTER_INIT_MASK;
                  }

                  BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_SWITCH_OPMODE_AFTER_VEL0;   // Call directly the required "SetOpmode" function
               }
               else if ( ((BGVAR(s16_Start_Opmode_Change_Capture) & 0xFF00) == OPMODE_CHANGE_SOURCE_OPMODE_SWITCH_INP) &&
                        ( ((BGVAR(s16_Start_Opmode_Change_Capture) & 0x00FF) < 4) ||
                          (((BGVAR(s16_Start_Opmode_Change_Capture) & 0x00FF) == 4) && (VAR(AX0_u8_Gear_Limits_Mode) & 0x0C)) // In speed-gear mode or if PTP hunter applies ACC/DEC/VLIM
                        )
                      )
               {
                  /***************************************************************************************************************/
                  /*** At this place the OPMODE switch digital input function triggered an OPMODE change on-the-fly in to mode ***/
                  /***  0, 1, 2, 3 and 4 (gearing when PTP-hunter applies ACC/DEC limits or if gearing in speed-gear mode.     ***/
                  /***************************************************************************************************************/

                  // Indicate to the RT task that we want an OPMODE change on the fly
                  VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_ON_THE_FLY_MASK;

                  // Indicate that we want to overwrite PTPVCMD in OPMODE 4 while the move-smooth filters
                  // are in the initialization process. This is only allowed for speed-gear mode.
                  if ((VAR(AX0_u8_Gear_Limits_Mode) & 0x08) && ((BGVAR(s16_Start_Opmode_Change_Capture) & 0x00FF) == 4))
                  {
                     // Overwrite PTPVCMD during filter-init in speed-gear mode
                     VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_OVERWRITE_PTPVCMD_DURING_FILTER_INIT_MASK;
                  }

                  BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_SWITCH_OPMODE_AFTER_VEL0;   // Call directly the required "SetOpmode" function
               }
               else // No OPMODE change on the fly is allowed
               {
                  BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_START_STANDSTILL_PROCESS;   // Perform an OPMODE change in standstill
               }
            }
         }
      break;

      case OPMODE_CHANGE_START_STANDSTILL_PROCESS:
         BGVAR(u16_Deceleration_Event_ID) = EVENT_DEC;     // Define to use DEC as a ramp
         BGVAR(u16_Hold_Bits) |= HOLD_OPMODE_CHANGE_MASK;  // Initiate the HOLD
         BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_CALC_DEC_TIME;
      break;

      case OPMODE_CHANGE_CALC_DEC_TIME:
         // calc deceleration time based on "BGVAR(u64_DecRate)", since in the state before we set the
         // variable "BGVAR(u16_Deceleration_Event_ID)" to "EVENT_DEC". Furthermore add 0.5 seconds
         // overhead for safety.
         BGVAR(s32_Opmode_Change_Timeout) = CalcDecelerationTimeout(drive, BGVAR(u64_DecRate), 0, 10000, 500);
         BGVAR(s32_Opmode_Change_TO_Timer) = Cntr_1mS;     // Cature the current value if the 1ms Counter
         BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_WAIT_FOR_VEL_0;
         break;

      case OPMODE_CHANGE_WAIT_FOR_VEL_0:
         // wait for motor to stop
         if (VAR(AX0_u16_Motor_Stopped) == 2)
         {
            BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_SWITCH_OPMODE_AFTER_VEL0;
         }
         else if (!Enabled(drive)) // If the Drive is in disable state, then switch anyhow the OPMODE
         {
            BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_SWITCH_OPMODE_AFTER_VEL0;
         }
         else if (PassedTimeMS(BGVAR(s32_Opmode_Change_Timeout), BGVAR(s32_Opmode_Change_TO_Timer))) // Perform also an OPMODE switch if timeout elapsed
         {
            BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_SWITCH_OPMODE_AFTER_VEL0;
         }
         break;

      case OPMODE_CHANGE_SWITCH_OPMODE_AFTER_VEL0:
         if ((BGVAR(s16_Start_Opmode_Change_Capture) & 0xFF00) == OPMODE_CHANGE_SOURCE_CAN_ECAT) // If CAN/ECAT triggered the OPMODE change...
         {
            // Call the function that determines which CDHD OPMODE to be set based
            // on the requested CANopen DS402 OPMODE value.
            SetCanOpmode(drive);
         }
         else if ( ((BGVAR(s16_Start_Opmode_Change_Capture) & 0xFF00) == OPMODE_CHANGE_SOURCE_SAL_FCT)            ||
                   ((BGVAR(s16_Start_Opmode_Change_Capture) & 0xFF00) == OPMODE_CHANGE_SOURCE_DUAL_MODE_IO)       ||
                   ((BGVAR(s16_Start_Opmode_Change_Capture) & 0xFF00) == OPMODE_CHANGE_SOURCE_OPMODE_SWITCH_INP)
                 )
         {
            // Set the required CDHD OPMODE (hold in the low byte of the variable)
            SetOpmode(drive, BGVAR(s16_Start_Opmode_Change_Capture) & 0x00FF);
         }
         BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_FINISH_STANDSTILL_PROCESS;
      break;

      case OPMODE_CHANGE_FINISH_STANDSTILL_PROCESS:
         BGVAR(s16_Start_Opmode_Change_Capture) = -1;                            // Clear the OPMODE change request
         BGVAR(u16_Hold_Bits) &= ~HOLD_OPMODE_CHANGE_MASK;                       // Clear the HOLD
         VAR(AX0_s16_Opmode_Change_Flags) &= ~OPMODE_CHANGE_ON_THE_FLY_MASK;     // Clear the "on the fly" bit in case that the OPMODE change has been rejected
                                                                                 // and the bit has not been cleared in real-time.
         BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_IDLE;
      break;

      default: // Undefined state which is not supposed to happen
         BGVAR(u16_Opmode_Change_State) = OPMODE_CHANGE_IDLE;   // Reset the state machine to go to idle state
         BGVAR(s16_Start_Opmode_Change_Capture) = -1;           // Reset OPMODE change request
      break;
   }
}


//**********************************************************
// Function Name: SalSwEnModeCommand
// Description:
//          This function is called in response to the POWERONMODE command.
//
// Author:  Alon Harpaz
// Algorithm:
// Revisions:
//**********************************************************
int SalSwEnModeCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u8_SwEnMode) = (char)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalSinInitModeCommand
// Description:
//          This function is called in response to the SININITMODE command.
//
// Author:  Alon Harpaz
// Algorithm:
// Revisions:
//**********************************************************
int SalSinInitModeCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u8_SinInitMode) = (char)lparam;

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: EnableCommand
// Description:
//          This function is called in response to the EN command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
// EN command
int EnableCommand(int drive)
{
    //  Defeat compilation error
    REFERENCE_TO_DRIVE;
    //Do not enable if Active Disable, or Config Transfer (=S= only)  is in process

    if (BGVAR(u16_AD_In_Process))
    {
        return (AD_IN_PROCCESS);
    }
    if (u16_Config_Lock == CONFIG_LOCK_WRITE)
    {
        return (VALUE_IS_NOT_ALLOWED);
    }

    //  if COMMODE = 1 and the EN command did not come from Master (but from ASCII) --> reject the command
    if ((IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1) && (BGVAR(s16_DisableInputs) & FB_EN_MASK))
    {
        return (NOT_ALLOWED_ENABLE_ON_COMMODE1);
    }

    //  Validate Feedback Configuration is in Process and if it is we want all these Inputs to be finished
    if ((BGVAR(u16_Feedback_Config_Process_Active)) &&
        (BGVAR(s16_DisableInputs) & (COMM_FDBK_DIS_MASK | BISSC_DIS_MASK | RESOLVER_FDBK_DIS_MASK | HIFACE_DIS_MASK | INVALID_FDBK_MASK | ENDAT_DIS_MASK | ENC_STATE_DIS_MASK)) == 0)
    {
        //  Reset Feedback Configuration in Process
        BGVAR(u16_Feedback_Config_Process_Active) = FALSE;
        //  Initialize the Modulo Mode State Machine
        InitializeModuloModeStateMachine(drive);
    }

    BGVAR(s16_DisableInputs) &= ~SW_EN_MASK;

    //  enable done reset rising edge flag to idle.
    BGVAR(u16_Son_Rising_Edge_detected) = 0;

    return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalDisableCommand
// Description:
//          This function is called in response to the K command with parameter.
//          This command returns the prompt when active is 0, or when timeout expires (if not 0).
//          The timeout is the parameter in milli-seconds.
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
int SalDisableCommand(long long s64_timeout_ms, int drive)
{
   if (BGVAR(u16_Disable_State) == 0)
   {
      DisableCommand(drive);
      BGVAR(s32_Disable_Timer) = Cntr_1mS;
      BGVAR(u16_Disable_State) = 1;
   }

   if ( (!Enabled(drive)) || (PassedTimeMS((long)s64_timeout_ms, BGVAR(s32_Disable_Timer))) )
      BGVAR(u16_Disable_State) = 0;

   if (BGVAR(u16_Disable_State) == 0)
      return (SAL_SUCCESS);
   else
      return (SAL_NOT_FINISHED);
}


//**********************************************************
// Function Name: DisableCommand
// Description:
//          This function is called in response to the K command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
// K,DIS commands
int DisableCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // 2 "K" commands will cause the AD process to cancle and go to immediate disable
   if ((BGVAR(s16_DisableInputs) & SW_EN_MASK) && BGVAR(u16_AD_In_Process))
   {
      if ( (BGVAR(u16_AD_State) == AD_START_RAMP_DOWN)         ||
           (BGVAR(u16_AD_State) == AD_WAIT_FOR_FIRST_DISSPEED) ||
           (BGVAR(u16_AD_State) == AD_WAIT_FOR_DISSPEED_QUAL)    )
         BGVAR(u16_AD_State) = AD_START_DISTIME_COUNTER;
      else
         BGVAR(u16_AD_State) = AD_DISABLE_DRIVE;
   }

   BGVAR(s16_DisableInputs) |= SW_EN_MASK;
   if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
            BGVAR(s16_DisableInputs) |= FB_EN_MASK;
   return (SAL_SUCCESS);
}


int StopCommand(int drive) // STOP command
{
   // AXIS_OFF;
   int ret_val = SAL_SUCCESS;

   if ((VAR(AX0_s16_Opmode) != 8) && (VAR(AX0_s16_Opmode) != 0) && (VAR(AX0_s16_Opmode) != 2))
      return (INVALID_OPMODE);

   switch (VAR(AX0_s16_Opmode))
   {
      case 2:
         ret_val = InternalTorqueCommand(0LL, drive);
      break;

      case 0:
         BGVAR(StepHandlerState) = STEP_IDLE;
         ret_val = JogCommand(0LL, drive);
      break;

      // case 4: Needs to be supported
      // break;

      case 8: // Stop the PTP generator
         PtpAbort(drive);
         BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_IDLE;
         MoveSineAbort(drive);
      break;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SalReadSwenCommand
// Description:
//          This function is called in response to the SWEN command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalReadSwenCommand(long long* res, int drive)
{
   int func;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   func = BGVAR(s16_DisableInputs) & SW_EN_MASK;
   if (func != 0) *res = 0;
   else  *res = 1;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: ReadRemoteCommand
// Description:
//          This function is called to read the value of the Remote-Enable Input
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int ReadRemoteCommand(long long* param, int drive)
{
   unsigned int u16_remote_in = 0;

   // If INXMODE = REMOTE_ENABLE_INP set REMOTE accordingly otherwire REMOTE=1
   if (u16_remote_in = IsInFunctionalityConfigured(REMOTE_ENABLE_INP, drive))
   {
      if (InxValue(u16_remote_in, drive))  *param = 1LL;
      else                                 *param = 0LL;
   }
   else *param = 1LL;

   return (SAL_SUCCESS);
}


int ReadEmergencyStopCommand(long long* param, int drive)
{
   // AXIS_OFF;
   unsigned int u16_emergency_stop_in = 0, i;

   *param = 0LL;

   // If INXMODE = EMERGENCY_STOP_INP set EMERGENCY_STOP accordingly otherwire EMERGENCY_STOP=0
   // Here more than 1 input may be configured as EMERGENCY_STOP_INP
   if (u16_emergency_stop_in = IsInFunctionalityConfigured(EMERGENCY_STOP_INP, drive))
   {
      for (i = u16_emergency_stop_in; i <= (unsigned int)s16_Num_Of_Inputs; i++)
      {
         if ((VAR(AX0_u16_Input_Mode_Arr[i]) == EMERGENCY_STOP_INP) && (u16_Supported_Dig_Inputs_Mask & (1 << (i-1))))
         {
            if (InxValue(i, drive))  *param = 1LL;
         }
      }
   }

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadReadyCommand
// Description:
//          This function is called in response to the READY command.
//
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
int SalReadReadyCommand(long long* ready, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // READY = 1 if all bits of s16_DisableInputs are 0, except for HW_EN
   if ( (BGVAR(s16_DisableInputs) & ~HW_EN_MASK) == 0) *ready = 1;
   else *ready = 0;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: OpmodeCommand
// Description:
//          This function is called in response to the Opmode command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalOpmodeCommand(long long lparam, int drive)
{
   int s16_canopen_mode;

   if ( (lparam != 0L) && (lparam != 1L) && (lparam != 2L) && (lparam != 3L) &&
        (lparam != 4L) && (lparam != 8L))
      return (VALUE_OUT_OF_RANGE);
   if ((4L == lparam) && (!IS_HW_FUNC_ENABLED(SECONDARY_ENC_MASK)) && (!IS_HW_FUNC_ENABLED(PULSE_DIRECTION_INPUT_MASK)))
      return NOT_SUPPORTED_ON_HW;

   if ((lparam == 2) && (BGVAR(s16_ControllerType) == VOLTAGE_CONTROL)) lparam = 0L;

   SetOpmode(drive, (int)lparam);
   if (lparam == 0)
      s16_canopen_mode = PROFILE_VELOCITY_MODE;
   else if (lparam == 2)
      s16_canopen_mode = PROFILE_TORQUE_MODE;
   else if (lparam == 8)
      s16_canopen_mode = PROFILE_POSITION_MODE;
   else if (lparam == 4)
      s16_canopen_mode = GEAR_MODE;
   else if (lparam == 1)
      s16_canopen_mode = ANALOG_VEL_MODE;
   else if (lparam == 3)
      s16_canopen_mode = ANALOG_TORQUE_MODE;
   else
      s16_canopen_mode = 0;

   // use s16_canopen_mode to avoid situation where RPDO overwrites p402_modes_of_operation
   if (s16_canopen_mode)
   {
       BGVAR(s16_CAN_Opmode) = s16_canopen_mode;
       p402_modes_of_operation = s16_canopen_mode;
       p402_modes_of_operation_display = s16_canopen_mode;
   }

   return (SAL_SUCCESS);
}


int CollectSinCosSamples(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   switch (BGVAR(s16_Sin_Cos_Avg_State))
   {
      case SIN_COS_AVG_INIT:
         BGVAR(s32_Avg_Sin) = 0;
         BGVAR(s32_Avg_Cos) = 0;
         BGVAR(s16_Avg_Cntr) = 0;
         BGVAR(s16_Sin_Cos_Avg_State) = SIN_COS_AVG_COLLECT_SAMPLES;
      break;

      case SIN_COS_AVG_COLLECT_SAMPLES:
         // Collect 32 samples
         BGVAR(s32_Avg_Sin) += abs(VAR(AX0_s16_Sine_2nd));
         BGVAR(s32_Avg_Cos) += abs(VAR(AX0_s16_Cosine_2nd));
         BGVAR(s16_Avg_Cntr)++;
         if (BGVAR(s16_Avg_Cntr) == 32)
         {
            BGVAR(s32_Avg_Sin) = BGVAR(s32_Avg_Sin) >> 5;
            BGVAR(s32_Avg_Cos) = BGVAR(s32_Avg_Cos) >> 5;
            BGVAR(s16_Sin_Cos_Avg_State) = SIN_COS_AVG_DONE;
         }
      break;

      case  SIN_COS_AVG_DONE:
      break;
   }

   if ( BGVAR(s16_Sin_Cos_Avg_State) ==  SIN_COS_AVG_DONE)
   {
      BGVAR(s16_Sin_Cos_Avg_State) = SIN_COS_AVG_INIT;
      return 1;
   }

   return 0;
}


int ExcitationOffestOutOFRange(int drive) // Check that s16_Excitation_Offset is in bounds
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( (BGVAR(s16_Excitation_Offset) < -(MAX_EXC_DELAY >> 1))                 ||
        (BGVAR(s16_Excitation_Offset) > (MAX_EXC_DELAY + (MAX_EXC_DELAY >> 1)))  )
      return 1;
    else
      return 0;
}


/**********************************************************
This function determines the sampling location of the resolver sin.cos signals

Bring ADC sample to sin/cos peak, make sre it is below threshold to avoid saturation

This Algorithm centralizes ADC sample to sin/cos peak and sets sin/cos amplitude to X% of full scale
   1) Select swr2d gain = minimum
   2) Move reference pos and than neg till peak is found
   3) Change gain so that sin^2+cos^2 is large enough (45 degrees) but not too large
   4) Find peak point. if gain exceeds limit - adj gain re-do
   5) After gain is set repeat (goto step 2, without changing the initial gain value)

**********************************************************/
int AdjustResolverReference(int drive)
{
   // AXIS_OFF;
//   int temp;
   long temp; // Switched verification of Amplitude Threshold to Sum-of-Square, hence 32-Bit Variable.
   int return_value;
   static int u16_gain_completed = 0, u16_first_iteration = 1;

   if (LVAR(AX0_s32_Feedback_Ptr) != &SW_RESOLVER_FEEDBACK) return SAL_SUCCESS;

   *(unsigned int *)(FPGA_RESOLVER_ENABLE_REG_ADD) = 1;

   // This will cause the state machine to delay for the sampling point and gain change to stabilize
   if (BGVAR(s16_Adjust_Reference_Delay) > 0)
   {
      if (u16_Desired_Resolver_Exc_Gain == u16_Actual_Resolver_Exc_Gain)
         BGVAR(s16_Adjust_Reference_Delay)--;

      if (BGVAR(s16_Adjust_Reference_Delay) > 0)
         return SAL_NOT_FINISHED;
   }

   switch (BGVAR(s16_Adjust_Reference_State))
   {
      case ADJ_REF_INIT:      // Init state
         // Inhibit PFB uppdate till procedure is done
         VAR(AX0_s16_Skip_Flags) &= ~UPDATE_RESOLVER_PFB_MASK;

//         BGVAR(u16_Max_Sin_Cos) = 0;
         BGVAR(s32_Sin2_Cos2) = 0;
         // Init offset to the middle of the range to allow change in both directions
         BGVAR(s16_Excitation_Offset) = (unsigned int)MAX_EXC_DELAY >> 1;
         u16_Desired_Resolver_Exc_Gain = 0x30;         // Init gain closer to high-end, to accommodate Resolvers
         BGVAR(s16_Adjust_Reference_Delay) = 1;        // with Tranmission Ratio lower than 0.4

         u16_gain_completed = 0;
         u16_first_iteration = 1;

         BGVAR(s16_Adjust_Reference_State) = ADJ_REF_WAIT_SAMPLES;
      break;

      case ADJ_REF_WAIT_SAMPLES:      // Read MAX(abs(SIN),abs(COS))
         if (CollectSinCosSamples(drive))
         {
//            BGVAR(u16_Max_Sin_Cos) = (unsigned int)MAX(BGVAR(s32_Avg_Sin), BGVAR(s32_Avg_Cos));
            BGVAR(s32_Sin2_Cos2) = BGVAR(s32_Avg_Sin) * BGVAR(s32_Avg_Sin) + BGVAR(s32_Avg_Cos) * BGVAR(s32_Avg_Cos);
            // SIN(90) is a max, COS(90) is 0, SIN(0) is 0, COS(0) is a max
            // if SIN at 45 COS at 45 --> this is a case when MAX(abs(SIN),abs(COS)) is minimal
            // let say amplitude of sin/cos is 2V pk-pk, at 45 degrees, sin (45) = 0.7 yields 0.7V with tolerance 0.5V
            // MAX(abs(SIN),abs(COS)) < 0.5V (=0x2AAA) it means either resolver feedback is not connected or ADC samples the signals around 0
            // move the resolver reference by 45 degrees to get out from 0 and only then start peak detection.
//            if (BGVAR(u16_Max_Sin_Cos) < 0x2AAA)
            if (BGVAR(s32_Sin2_Cos2) < 0x71C38E4)
            {
               BGVAR(s16_Excitation_Offset) += (unsigned int)MAX_EXC_DELAY >> 2;
               if (ExcitationOffestOutOFRange(drive))
               {
                  BGVAR(s16_Adjust_Reference_State) = ADJ_REF_FAILED;
                  break;
               }
               BGVAR(s16_Adjust_Reference_Delay) = 1;
               break;
            }
            BGVAR(s16_Excitation_Offset) += ADC_SAMPLE_POS_CORRECTION;
            if (ExcitationOffestOutOFRange(drive))
            {
               BGVAR(s16_Adjust_Reference_State) = ADJ_REF_FAILED;
               break;
            }
            BGVAR(s16_Adjust_Reference_Delay) = 1;
            BGVAR(s16_Adjust_Reference_State) = ADJ_REF_POSITIVE_CORR;
         }
      break;

      // Positive reference correction, move sampling point till sampled value starts to drop
      // then change direction
      case ADJ_REF_POSITIVE_CORR:
         if (CollectSinCosSamples(drive))
         {
//            temp = (unsigned int)MAX(BGVAR(s32_Avg_Sin), BGVAR(s32_Avg_Cos));
            temp = BGVAR(s32_Avg_Sin) * BGVAR(s32_Avg_Sin) + BGVAR(s32_Avg_Cos) * BGVAR(s32_Avg_Cos);
//            if (temp < BGVAR(u16_Max_Sin_Cos))
            if (temp < BGVAR(s32_Sin2_Cos2))
            {
               BGVAR(s16_Excitation_Offset) += ADC_SAMPLE_NEG_CORRECTION;
               BGVAR(s16_Adjust_Reference_State) = ADJ_REF_NEGATIVE_CORR;
            }
            else
               BGVAR(s16_Excitation_Offset) += ADC_SAMPLE_POS_CORRECTION;

            if (ExcitationOffestOutOFRange(drive))
            {
               BGVAR(s16_Adjust_Reference_State) = ADJ_REF_FAILED;
               break;
            }
            BGVAR(s16_Adjust_Reference_Delay) = 1;
//            BGVAR(u16_Max_Sin_Cos) = temp;
            BGVAR(s32_Sin2_Cos2) = temp;
         }
      break;

      // Negative reference correction, move sampling point till sampled value reduces again
      // This is the max value (assuming we are not at saturation)
      case ADJ_REF_NEGATIVE_CORR:
         if (CollectSinCosSamples(drive))
         {
//            temp = (unsigned int)MAX(BGVAR(s32_Avg_Sin), BGVAR(s32_Avg_Cos));
            temp = BGVAR(s32_Avg_Sin) * BGVAR(s32_Avg_Sin) + BGVAR(s32_Avg_Cos) * BGVAR(s32_Avg_Cos);
            if (temp <= BGVAR(s32_Sin2_Cos2))
               BGVAR(s16_Adjust_Reference_State) = ADJ_REF_THRESH_ADJUST;
            else
               BGVAR(s16_Excitation_Offset) += ADC_SAMPLE_NEG_CORRECTION;

            if (ExcitationOffestOutOFRange(drive))
            {
               BGVAR(s16_Adjust_Reference_State) = ADJ_REF_FAILED;
               break;
            }
            BGVAR(s16_Adjust_Reference_Delay) = 1;
//            BGVAR(u16_Max_Sin_Cos) = temp;
            BGVAR(s32_Sin2_Cos2) = temp;
         }
      break;

      // Limit SIN,COS to 80% of full range (to allow inaccuracies) which is 0.8*0x7FFF = 0x6666
      // Change gain till value of sin^2+cos^2 is just below (0x6666^2)
      case ADJ_REF_THRESH_ADJUST:
         if (CollectSinCosSamples(drive))
         {
            BGVAR(s32_Sin2_Cos2) = BGVAR(s32_Avg_Sin) * BGVAR(s32_Avg_Sin) + BGVAR(s32_Avg_Cos) * BGVAR(s32_Avg_Cos);

            if (BGVAR(s32_Sin2_Cos2) < 0x28F570A4)
            {  // Increase or decrease gain till sin^2+cos^2 <= 0x28F570A4 = 0x6666^2
               u16_gain_completed |= 0x01;
               if (u16_Desired_Resolver_Exc_Gain > 0)
                  u16_Desired_Resolver_Exc_Gain--;
               else
                  BGVAR(s16_Adjust_Reference_State) = ADJ_REF_FAILED;
            }
            else
            {
               u16_gain_completed |= 0x02;
               if (u16_Desired_Resolver_Exc_Gain < 0x7F)
                  u16_Desired_Resolver_Exc_Gain++;
               else
                  BGVAR(s16_Adjust_Reference_State) = ADJ_REF_FAILED;
            }

            // If gain value is found, repeat the search with better gain value
            if ( ((u16_gain_completed & 0x03) == 0x03)                &&
                 (BGVAR(s16_Adjust_Reference_State) != ADJ_REF_FAILED)  )
            {
               if (u16_first_iteration)               // Retry with adjusted gain
               {
//                  BGVAR(u16_Max_Sin_Cos) = 0;
                  BGVAR(s32_Sin2_Cos2) = 0;
                  BGVAR(s16_Excitation_Offset) = (unsigned int)MAX_EXC_DELAY >> 1;
                  u16_gain_completed = 0;
                  u16_first_iteration = 0;
                  BGVAR(s16_Adjust_Reference_State) = ADJ_REF_WAIT_SAMPLES;
               }
               else // Second time - start updating the PFB and finish
               {  // Start updating PFB
                  VAR(AX0_s16_Skip_Flags) |= (UPDATE_RESOLVER_PFB_MASK | INIT_RESOLVER_PFB_MASK);
                  BGVAR(s16_Adjust_Reference_State) = ADJ_REF_DONE;
               }
            }

            BGVAR(s16_Adjust_Reference_Delay) = 1;
         }
      break;

      case  ADJ_REF_DONE:
      break;

      case  ADJ_REF_FAILED:
      break;
/*
      case ADJ_REF_SCAN_INIT:
         BGVAR(s16_Excitation_Offset) = 0;
         BGVAR(s16_Adjust_Reference_State) = ADJ_REF_SCAN;
         return SAL_NOT_FINISHED;
      break;

      case ADJ_REF_SCAN:
         if (CollectSinCosSamples(drive))
         {
            *(int*)(FPGA_RESOLVER_DELAY_REG_ADD) = BGVAR(s16_Excitation_Offset);
            BGVAR(s32_Sin2_Cos2) = BGVAR(s32_Avg_Sin) * BGVAR(s32_Avg_Sin) + BGVAR(s32_Avg_Cos) * BGVAR(s32_Avg_Cos);
            if (BGVAR(s16_Excitation_Offset) >= MAX_EXC_DELAY)
               BGVAR(s16_Adjust_Reference_State) = ADJ_REF_FAILED;
            BGVAR(s16_Excitation_Offset) += 10;
            return SAL_NOT_FINISHED;
         }
      break; */
   }

   if (BGVAR(s16_Adjust_Reference_State) == ADJ_REF_DONE)         return_value = SAL_SUCCESS;
   else if (BGVAR(s16_Adjust_Reference_State) == ADJ_REF_FAILED)  return_value = -1;
   else                                                           return_value = SAL_NOT_FINISHED;

   // Write requested delay to FPGA
   if ( (BGVAR(s16_Adjust_Reference_State) != ADJ_REF_DONE)  &&
        (BGVAR(s16_Adjust_Reference_State) != ADJ_REF_FAILED)  )
   {
      temp = BGVAR(s16_Excitation_Offset);
      if (temp < 0) temp += MAX_EXC_DELAY;
      else if (temp > MAX_EXC_DELAY) temp -= MAX_EXC_DELAY;

      if (temp != EPwm6Regs.TBPHS.half.TBPHS)
      {
         EPwm6Regs.TBPHS.half.TBPHS = temp;
         // Signal R.T. to update Resolver Excitation Phase to ensure
         // uniform reference to the "same" MTS cycle
         VAR(AX0_s16_Crrnt_Run_Code) &= ~RES_EXC_INITIALIZED;
      }
   }

   return return_value;
}


void InrushHandler()
{
   unsigned int u16_i, u16_vbus_avg;
   unsigned long u32_acc = 0L;
   static unsigned int b_inrush_relay_in_work_mode = 0;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

    if (0 != (u16_Output_Handler_State & (1 << OUTPUTST_INRUSH)))
      return;


   if (1 == u16_STO_Inrush_Inhibit_Enable)
   {
      BGVAR(s16_DisableInputs) |= INRUSH_INHIBIT_MASK;
      u16_STO_Inrush_Inhibit_Enable = 0;
   }

   // Do not close Inrush relay if power EEPROM was not read correctly since Vbus scale might be wrong (ignore invalid production area in EEPROM)
   if ( (1 == u16_Fast_STO_En_Turn_Inrush_On) && ((BGVAR(u16_Power_Board_Eeprom_Fault) & (~(PRODUCTION_STAMP_FAULT_MASK | PRODUCTION_CHECKSUM_FAULT_MASK))) == 0) )
   {
      InrushRelayOn();
   }
   else if ( (!u16_STO_Flt_Indication_Actual) && (!u16_STO_Just_Inserted) &&      // mask inrush handler while STO is off
             ((BGVAR(u16_Power_Board_Eeprom_Fault) & (~(PRODUCTION_STAMP_FAULT_MASK | PRODUCTION_CHECKSUM_FAULT_MASK))) == 0) )
   {
      u16_Vbus_Array[u16_Vbus_Array_Index] = BGVAR(u16_Vbus_Volts);      // Fill the vbus array
      u16_Vbus_Array_Index++;
      if (u16_Vbus_Array_Index >= VBUS_BUFFER_SIZE) u16_Vbus_Array_Index = 0;

      if (PassedTimeMS(50L, s32_Vbus_Timer))      // Run the algorithm every 50ms
      {
         s32_Vbus_Timer = Cntr_1mS;

         // Calculate averate bus voltage
         for (u16_i = 0; u16_i < VBUS_BUFFER_SIZE; u16_i++) u32_acc += u16_Vbus_Array[u16_i];
         u16_vbus_avg = (unsigned int)(u32_acc / VBUS_BUFFER_SIZE);

         // Check if inrush relay needs to close
         if ((u16_vbus_avg >= u16_Inrush_On_Value) && (u16_vbus_avg > 20))  // the condition > 20 is to protect against u16_Inrush_On_Value that was set to low value (0) by mistake
                                                                            // changed from 30V to 20V to allow working with 24V LV
         {
            if (!b_inrush_relay_in_work_mode)
            {
               if (u16_Inrush_Qual_On_Counter < 3) u16_Inrush_Qual_On_Counter++;
               else // Run the "ON" check after 3 times the VBUS is above the on_threshold
               {
                  if (abs(u16_vbus_avg - u16_Vbus_Avg_Prev) <= 3) u16_Inrush_Qual_Settled_Counter++;
                  else if (!IsInrushRelayOn()) u16_Inrush_Qual_Settled_Counter = 0;

                  if (u16_Inrush_Qual_Settled_Counter >= 6) InrushRelayOn(); // Inrush to working mode;
                  if (u16_Inrush_Qual_Settled_Counter >= 8)
                  {
                     BGVAR(s16_DisableInputs) &= ~INRUSH_INHIBIT_MASK; // Allow to enable the drive
                     drive = 1;
                     BGVAR(s16_DisableInputs) &= ~INRUSH_INHIBIT_MASK; // Allow to enable the drive
                     drive = 0;

                     u16_Regen_Algorithm_Activate = 1;
                     b_inrush_relay_in_work_mode = 1;
                  }
               }
            }
         }
         else // Check if inrush relay needs to open
         {
            u16_Inrush_Qual_On_Counter      = 0;
            u16_Inrush_Qual_Settled_Counter = 0;

            if (u16_vbus_avg <= u16_Inrush_Off_Value)
            {
               if (IsInrushRelayOn())
               {
                  if (u16_Inrush_Qual_Off_Counter < 3) u16_Inrush_Qual_Off_Counter++;
                  else // Run the "OFF" check after 3 times the VBUS is below the off_threshold
                  {
                     InrushRelayOff();
                     b_inrush_relay_in_work_mode = 0;
                     u16_Regen_Algorithm_Activate = 0;
                     BGVAR(s16_DisableInputs) |= INRUSH_INHIBIT_MASK; // Inhibit enable
                     drive = 1;
                     BGVAR(s16_DisableInputs) |= INRUSH_INHIBIT_MASK; // Inhibit enable
                     drive = 0;
                     u16_Inrush_Qual_Off_Counter = 0;
                  }
               }
            }
            else
               u16_Inrush_Qual_Off_Counter = 0;
         }
         u16_Vbus_Avg_Prev = u16_vbus_avg;
      }
   }
   else // (!u16_STO_Flt_Indication_Actual)
   {  // open the relay and init vbus buffer and inrush counters
      for(u16_Vbus_Array_Index = 0; u16_Vbus_Array_Index < VBUS_BUFFER_SIZE; u16_Vbus_Array_Index++)
         u16_Vbus_Array[u16_Vbus_Array_Index] = 0;

      // IPR 763 (watchdog): zero index to prevent setting value outside of array bounderies
      u16_Vbus_Array_Index = 0;

      InrushRelayOff();
      b_inrush_relay_in_work_mode = 0;
      u16_Regen_Algorithm_Activate = 0;
      u16_Inrush_Qual_Off_Counter = 0;
      u16_Inrush_Qual_On_Counter = 0;
      u16_Inrush_Qual_Settled_Counter = 0;
   }

   if (DC_DRIVE) // On LV drive Regen is not relevant
   {
      u16_Regen_Algorithm_Activate = 0;
   }

}


void InrushRelayOff(void)
{
   if ( (u16_Product == DDHD) || (u16_Product == SHNDR_HW) || (IS_CAN2_DRIVE) )
   {
      EALLOW;
      GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;    // High Z (off)
      EDIS;
   }
   else
   {
      INRUSH_RELAY_OFF;                      // FPGA inverts the control bit
   }
}

void InrushRelayOn(void)
{
   if ( (u16_Product == DDHD) || (u16_Product == SHNDR_HW) || (IS_CAN2_DRIVE))
   {
      GpioDataRegs.GPADAT.bit.GPIO26 = 0;
      EALLOW;
      GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;    // active low (on)
      EDIS;
   }
   else
   {
      INRUSH_RELAY_ON;                       // FPGA inverts the control bit
   }
}


int IsInrushRelayOn(void)
{
   if ( (u16_Product == DDHD) || (u16_Product == SHNDR_HW) || (IS_CAN2_DRIVE))
   {
      if (GpioCtrlRegs.GPADIR.bit.GPIO26 == 1)
      {
         return(1);
      }
      else
      {
         if (GpioDataRegs.GPADAT.bit.GPIO26 == 0)
            return(1);
         else
            return(0);
      }
   }
   else
   {
      return(IS_INRUSH_RELAY_ON);            // FPGA inverts the control bit
   }
}


//**********************************************************
// Function Name: SalChangeOutputState
// Description:
//     This function is called in response to the OUTPUTST command
//     Currently supports 2 digital Outputs: Regen (0), Inrush (1)
//           READ METHOD:
//             input arg  : desired Output
//             output args:  1. desired Output's Handler overwrite state
//                           2. desired Output's current state
//           WRITE METHOD:
//             Change an OutPut's State
//             input args  : 1. desired Output.
//                           2. desired Output's Handler overwrite state
//                           3. desired Output's state change
// Author: Lior
//**********************************************************
int SalChangeOutputState(int drive)
{
   int u16_selected_output = 0, u16_handler_overwrite = 0, u16_output_state = 0, u16_handler_overwrite_state = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   u16_selected_output = s64_Execution_Parameter[0];

   /* Read Method */
   if ((1 == s16_Number_Of_Parameters))
   {
      if ((OUTPUTST_REGEN <= s64_Execution_Parameter[0]) && (OUTPUTST_INRUSH >= s64_Execution_Parameter[0]))
      {
         u16_handler_overwrite_state = (u16_Output_Handler_State >> u16_selected_output) & 0x1;
         // first output argument --> handler's state
         PrintSignedInteger(u16_handler_overwrite_state);
         PrintChar(' ');
         switch (u16_selected_output)
         {
            // second output argument --> current output's state
            case OUTPUTST_REGEN:
               PrintSignedInteger(IS_REGEN_RELAY_ON);
               PrintCrLf();
            break;
            case OUTPUTST_INRUSH:
               PrintSignedInteger(IS_INRUSH_RELAY_ON);
               PrintCrLf();
            break;
         }
         return SAL_SUCCESS;
        }
        else
         return VALUE_OUT_OF_RANGE;
   }
   /* Write Method */
   else if ( ((OUTPUTST_REGEN > s64_Execution_Parameter[0]) || (OUTPUTST_INRUSH < s64_Execution_Parameter[0])) ||  //selected output
             ((0 != s64_Execution_Parameter[1]) && (1 != s64_Execution_Parameter[1])) ||     //handler overwrite
             ((0 != s64_Execution_Parameter[2]) && (1 != s64_Execution_Parameter[2]))   )    //selected output state
      return VALUE_IS_NOT_ALLOWED;

   u16_handler_overwrite = s64_Execution_Parameter[1];
   u16_output_state = s64_Execution_Parameter[2];

   if (u16_handler_overwrite)
   {
      u16_Output_Handler_State |= (0x1 << u16_selected_output);
      switch (u16_selected_output)
      {
         case OUTPUTST_REGEN:
            if (u16_output_state)
               REGEN_RELAY_ON;
            else
               REGEN_RELAY_OFF;
         break;

         case OUTPUTST_INRUSH:
            if (u16_output_state)
               InrushRelayOn();
            else
               InrushRelayOff();
         break;

         default:
            return VALUE_IS_NOT_ALLOWED;
      }
   }
   else //handlers off
   {
      u16_Output_Handler_State &= ~(0x1 << u16_selected_output);
   }
   return SAL_SUCCESS;
}


// Implement bang-bang algorithm, when obove Regen_On value close the relay,
// when below Regen_Off value open the relay.
// This function must be called out of the 1[ms] RT task.
#pragma CODE_SECTION(RegenHandler, "ramfunc_2");
void RegenHandler(int drive)
{
   unsigned int u16_capture_vbus_value = BGVAR(u16_Vbus_Volts);

   REFERENCE_TO_DRIVE;     // Defeat compilation error
   if (0 != (u16_Output_Handler_State & (1 << OUTPUTST_REGEN)))
      return;

   if ( (u16_Regen_Algorithm_Activate)            && (!u16_Regen_Overload_Protection_Active)          &&
        (!u16_Regen_Max_On_Time_Protection_Active) && ((BGVAR(s64_SysNotOk) & REGEN_OC_FLT_MASK) == 0)  )
   {
      if (u16_capture_vbus_value >= u16_Regen_Actual_On_Value)
      {
         REGEN_RELAY_ON;
         // For bus sharing: if the regen was activated, increase the on threshold by 1 volt, up to EEPROM value plus 5 volts, and reset timer
         if (u16_Regen_Actual_On_Value < (u16_Regen_On_Value + 5))
         {
            u16_Regen_Actual_On_Value++;
         }
         u16_Regen_Timer = 0;
      }
      else if (u16_capture_vbus_value <= u16_Regen_Off_Value)
      {
         REGEN_RELAY_OFF;
      }
   }
   else
   {
      if (DC_DRIVE)
         REGEN_RELAY_ON; // On LV - in order to use the Regen_OC signal, the regen should be activated (FPGA will not monitor the signal otherwise)
      else
         REGEN_RELAY_OFF;
   }

   // For the regen overload algorithm and for the ON time monitoring, check here the state of the regen relay
   // and not above where the regen relay is turned on.  The reason is that due to the hysteresis, when
   // the bus voltage drops below the ON value but still above the OFF value, the relay remains ON without
   // the need to turn it ON.
   // The ON time monitoring limits the continuous time the regen is allowed to be ON.  When the regen ON time
   // gets to this limit, the regen should remain OFF for 9 times this limit to maintain duty cycle of 10%.
   // Since the regen can be ON and OFF alternately, this algorithm weights the ON time 9 times compared to
   // the OFF time, again, to maintain duty cycle of 10%.
   // Each time the regen is ON, a counter is incremented by 9, and each time the regen is OFF, this counter
   // is deccremented by 1.  The counter can not go below 0.  When the counter reaches 9 times the max allowed
   // ON time, a flag is set to force OFF time till the counter gets back to 0 (which equivalent to OFF time
   // of 9 times the max allowed ON time).
   if (IS_REGEN_RELAY_ON)
   {
      // Increment values for regeneration overload protection
      u16_Regen_On_Time_Last_500ms++;
      u16_Regen_On_Time_Last_5s++;

      u16_Regen_On_Time_Monitor += 9;
      if (u16_Regen_On_Time_Monitor >= (BGVAR(u16_Regen_Max_On_Time_ms) * 9))
      {
         u16_Regen_Max_On_Time_Protection_Active = 1;
      }
   }
   else
   {
      if (u16_Regen_On_Time_Monitor > 0)
      {
         u16_Regen_On_Time_Monitor--;
      }

      if (u16_Regen_On_Time_Monitor == 0)
      {
         u16_Regen_Max_On_Time_Protection_Active = 0;
      }
   }

   // For bus sharing: gradually reduce the on threshold to the EEPROM value. Each 2.5 seconds reduce 1 volt
   if (u16_Regen_Actual_On_Value > u16_Regen_On_Value)
   {
      u16_Regen_Timer++;
      if (u16_Regen_Timer > u16_Regen_Timer_Threshold)
      {
         u16_Regen_Timer = 0;
         u16_Regen_Actual_On_Value--;
      }
   }

   // ****************************************************************
   // Here run the regeneration resistor overload protection algorithm
   // ****************************************************************
   // If the regen-resistor overload-protection algorithm is active at all
   if (u16_Max_Regen_On_Time != 0)
   {
      // If we have reached 50% of a 500[ms] sub-cycle. This instruction is done in
      // order to average the variable "u16_Regen_On_Time_Last_5s". So when this variable
      // is subtracted by the oldest value in the buffer, we subtract from a total value
      // of the last 5.25[s] the time of the oldest 0.5[s] cycle and therefore the variable
      // is always in a time-range of the last 4.75[s]...5.25[s].
      if (u16_Regen_Cycle_Counter == (REGEN_CYCLE_IN_MS>>1))
      {
         // Subtract the oldest entry in the array (value of "u16_Regen_On_Time_Last_500ms" 5[s] ago)
         // from the total 5[s] period on-time.
         u16_Regen_On_Time_Last_5s -= u16_Regen_On_Time_Array[u16_Regen_Array_Index];
      }

      // If one full regen overload protection sub-cycle (500ms) is over.
      if (++u16_Regen_Cycle_Counter >= REGEN_CYCLE_IN_MS)
      {
         // Start a new cycle
         u16_Regen_Cycle_Counter = 0;
         // Store the on-time of the last 500[ms] --> of the current cycle
         u16_Regen_On_Time_Array[u16_Regen_Array_Index] = u16_Regen_On_Time_Last_500ms;
         // Clear the regen on-time counter for the next 500ms cycle
         u16_Regen_On_Time_Last_500ms = 0;
         // Increment index for the array
         if (++u16_Regen_Array_Index >= REGEN_ON_TIME_ARR_SIZE)
         {
            u16_Regen_Array_Index = 0;
         }
      }
      // Check if the regeneration resistor is loaded by 100%. If Yes then
      // deactivate the algorithm temporarily.
      if (u16_Regen_On_Time_Last_5s > u16_Max_Regen_On_Time)
      {
         // Disable regen-algorithm until the on-time during
         // the last 5[s] falls under the limit
         u16_Regen_Overload_Protection_Active = 1;
         // Indicate regen overload condition
         u16_Regen_Overload_Condition = 5000;      // Latch the regen overload condition for 5 sec
      }
      else
      {
         // Re-enable the regeneration-resistor algorithm
         u16_Regen_Overload_Protection_Active = 0;
         // Remove regen overload condition
         if (u16_Regen_Overload_Condition > 0)
            u16_Regen_Overload_Condition--;
      }
   }
   else // get here if REGENPOW=-1 and/or REGENGES=-1.
   {
      // Activate the regen to maintain backwards compatibility to versions before considering REGENPOW and REGENGES (e.g. 1.4.6).
      u16_Regen_Overload_Protection_Active = 0;

      u16_Regen_Max_On_Time_Protection_Active = 0;  // overide prev value to avoid deactivation of regen (backwards compatibility)


      // Remove regen overload condition
      if (u16_Regen_Overload_Condition > 0)
         u16_Regen_Overload_Condition--;
   }
}


// Implement low pass filter with 10Hz cutoff frequency for motor temp. signal coming from ADC
// #pragma CODE_SECTION(MotorTempFilter, "ramfunc_4");
#pragma CODE_SECTION(MotorTempFilter, "ramfunc_3");
void MotorTempFilter(int drive)
{
    // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   //low pass filter on AX0_s16_Adc_Plus_15v_Readout
   BGVAR(s16_Adc_Mfb_Motor_Temp_Filtered) = (int)(((long)((long)BGVAR(s16_Adc_Mfb_Motor_Temp_Filtered) * LPF_10_HZ_ALPHA_FIX) + // alpha factor
                              (long)((long)VAR(AX0_s16_Adc_Mfb_Motor_Temp) * LPF_10_HZ_BETA_FIX) +         // beta factor
                              LPF_10_HZ_ROUNDING) >> LPF_10_HZ_SHR); // round and shift
}


// Implement low pass filter with 10Hz cutoff frequency for P. S. signal coming from ADC
#pragma CODE_SECTION(PowerSupplyFilter, "ramfunc_4");
void PowerSupplyFilter(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   // The ADC provides new value every 6.25 ms.
   // Execute the filters at 10 ms rate, only after the ADC provided the first readout.

   u16_Power_Supply_Filter_Counter++;
   if (u16_Power_Supply_Filter_Counter >= 10) u16_Power_Supply_Filter_Counter = 0;

   if ((VAR(AX0_u16_Power_Supply_Adc_Flags) == 3) && (u16_Power_Supply_Filter_Counter == 0))
   {
      //low pass filter on AX0_s16_Adc_Plus_15v_Readout
      s16_Adc_Plus_15v_Readout_Filtered = (int)(((long)((long)s16_Adc_Plus_15v_Readout_Filtered * LPF_05_HZ_10mS_ALPHA_FIX) + // alpha factor
                                 (long)((long)VAR(AX0_s16_Adc_Plus_15v_Readout) * LPF_05_HZ_10mS_BETA_FIX) +         // beta factor
                                 LPF_05_HZ_10mS_ROUNDING) >> LPF_05_HZ_10mS_SHR); // round and shift


      //low pass filter on AX0_s16_Adc_Minus_15v_Readout
      s16_Adc_Minus_15v_Readout_Filtered = (int)(((long)((long)s16_Adc_Minus_15v_Readout_Filtered * LPF_05_HZ_10mS_ALPHA_FIX) + // alpha factor
                                  (long)((long)VAR(AX0_s16_Adc_Minus_15v_Readout) * LPF_05_HZ_10mS_BETA_FIX) +          // beta factor
                                  LPF_05_HZ_10mS_ROUNDING) >> LPF_05_HZ_10mS_SHR); // round and shift
   }
}


// Determine FAN speed according to the temp threshold defined on the EE
void FanHandler(void)
{
   // AXIS_OFF;

   int s16_power_temp = 0, u16_ipm_temp;

   s16_power_temp = BGVAR(s16_Power_Temperature_Deg);
   u16_ipm_temp = BGVAR(u16_IPM_Temperature);

   if ( (s16_power_temp > s16_Fan_Hi_Spd_Threshold) ||
        ((u16_ipm_temp > BGVAR(u16_IPM_Temp_Fan_Thresh)) && (VAR(AX0_u16_IPM_OT_DC_Enable) == 1)) )
   {
      FanFullSpeed();
      u16_Fan_High_Speed = 1;
   }
   else if (u16_Fan_High_Speed && (s16_power_temp <= (s16_Fan_Hi_Spd_Threshold - 10)) &&
            ((u16_ipm_temp <= (BGVAR(u16_IPM_Temp_Fan_Thresh) - 10)) || (VAR(AX0_u16_IPM_OT_DC_Enable) == 0))   )
   {
      FanHalfSpeed();
      u16_Fan_High_Speed = 0;
   }
}


void FanFullSpeed(void)
{
   if ( ( (u16_Product == DDHD) && !IS_DDHD_EC2_DRIVE ) || (u16_Product == SHNDR_HW) || (IS_CAN2_DRIVE) )
   {
      if ((BGVAR(u16_Power_Hw_Features) & 0x0080) == 0)
      {
         // no polarity inversion of fan control
      EALLOW;
      GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;    // High Z (off)
      EDIS;
   }
   else
   {
         // inverted polarity of fan control
         GpioDataRegs.GPADAT.bit.GPIO25 = 0;
         EALLOW;
         GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;    // active low (on)
         EDIS;
      }
   }
   else
   {
      if ((BGVAR(u16_Power_Hw_Features) & 0x0080) == 0)
      {
         // no polarity inversion of fan control
         FAN_FULL_SPEED;                        // FPGA inverts the control bit
      }
      else
      {
         // inverted polarity of fan control
         FAN_HALF_SPEED;                        // FPGA inverts the control bit
      }
   }
}


void FanHalfSpeed(void)
{
   if ( ( (u16_Product == DDHD) && !IS_DDHD_EC2_DRIVE ) || (u16_Product == SHNDR_HW) || (IS_CAN2_DRIVE))
   {
      if ((BGVAR(u16_Power_Hw_Features) & 0x0080) == 0)
      {
         GpioDataRegs.GPADAT.bit.GPIO25 = 0;    // no polarity inversion of fan control
         EALLOW;
         GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;    // active low (on)
         EDIS;
      }
      else
      {
         EALLOW;         // inverted polarity of fan control
         GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;    // High Z (off)
         EDIS;
      }
   }
   else
   {
      if ((BGVAR(u16_Power_Hw_Features) & 0x0080) == 0)
      {
         // no polarity inversion of fan control
      FAN_HALF_SPEED;                        // FPGA inverts the control bit
   }
      else
      {
         // inverted polarity of fan control
         FAN_FULL_SPEED;                        // FPGA inverts the control bit
      }
   }
}


//**********************************************************
// Function Name: SalGearActiveCommand
// Description:
//          This function is called in response to the GEAR command.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalGearActiveCommand(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   VAR(AX0_u8_Gear_Active) = param;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalGearLimitsModeCommand
// Description:
//          This function is called in response to the GEARLIMITSMODE command.
//
// Author: Alon Harpaz
// Algorithm:
// Revisions:
//**********************************************************
int SalGearLimitsModeCommand(long long param, int drive)
{
   // AXIS_OFF;

   /* If continue counting master pulses even if the Drive is disabled AND */
   /* if we do not apply ACC/DEC/VLIM for the electronic gearing.          */
   if ((param & 0x0001) && (!(param & 0x0004)))
   {
      // Do not accept this combination since the PTP-hunter may produce a motion
      // without limitations according to the maxium allowed accel/decel/velocity.
      return (VALUE_OUT_OF_RANGE);
   }

   /* If we discard in each sample the position lag between the input and output of the PTP hunter due to the limits AND  */
   /* if we do not put the PTP hunter in mode 1 (reach moving target position).                                           */
   else if (((param & 0x000C) == 0x000C) && (!(param & 0x0010)))
   {
      // Do not accept this combination since the so-called speed-gear mode (bit 3 = true)
      // would not work properly. The PTP-hunter would not produce a ramp up to the speed 
      // of the gearing master pulses since Vsaf (_AX0_s32_Vsaf) lowers the maximum allowed
      // PTP-hunter speed due to the fact that it wants to keep a deceleration distance to the
      // moving target position. But keep in mind that due to bit 3 = true, the gearing position 
      // is being continuously set to the PTP-hunter output, so we eliminate all the time the distance
      // to decelerate into target, but still the PTP-hunter in mode 0 wants to re-establish that distance
      // to the target position. That means that bit 3 of GEARLIMITSMODE and PTP-hunter in mode 0
      // (adjusted by bit 4 of GEARLIMITSMODE) work against each other and we would never reach the speed
      // of the gearing pulses.
      //
      // But if we put the PTP-hunter into the mode 1 (run synchronously with the target position),
      // then the PTP-hunter does NOT keep a deceleration distance to the moving target position and 
      // therefore the speed-gear mode is able to accelerate up to the velocity of the gearing pulses 
      // (which is the d/dt of the gearing position).
       return (VALUE_OUT_OF_RANGE);
   }

   VAR(AX0_u8_Gear_Limits_Mode) = (unsigned int)param;

   // Check in which mode the PTP hunter is supposed to run. Update the neccessary
   // variables straight away (function argument = 1).
   CheckPtpHunterMode(drive, 1);

   // Update Acc and Dec after changing Gear Limits Mode
   UpdateAccDec(drive,ACC_UPDATE);
   UpdateAccDec(drive,DEC_UPDATE);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalReadLxmGearingMode
// Description:
//    This function converts the settings of the gearing papameters into the Lexium
//    object description 0x301B Sub 0x12 (see "LXM32M_CANopen_Manual_V105_EN.pdf" on
//    page 65, GEARreference). Note that the Write and the Read function always need
//    to be aligned to each other.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadLxmGearingMode(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;

   *data = BGVAR(u16_P8_31_Lex_Gearmode);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalWriteLxmGearingMode
// Description:
//    This function converts a setting of the P-Parameter P8-31, which corresponds
//    to the Lexium object description 0x301B Sub 0x12 (see "LXM32M_CANopen_Manual_V105_EN.pdf"
//    page 65) into the proper CDHD gearing parameters setting.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteLxmGearingMode(long long param,int drive)
{
   // AXIS_OFF;
   int s16_return_value;
   long long s64_temp_gear_limits_mode;

   BGVAR(u16_P8_31_Lex_Gearmode) = param;

   // Set the CDHD commands in a proper manner. The meaning of param is according to
   // the object GEARreference in the document "LXM32M_CANopen_Manual_V105_EN.pdf"
   // on page 65.

   switch(param)
   {
      case (0):
         s64_temp_gear_limits_mode = 4; // This setting does not matter, since gearing will be anyhow deactivated by "BGVAR(u16_P8_31_Lex_Gearmode)"
      break;
      case (1):
         s64_temp_gear_limits_mode = 4; // Activate ACC/DEC/VLIM without compensation move
      break;
      case (2):
         s64_temp_gear_limits_mode = 5; // Activate ACC/DEC/VLIM with compensation move
      break;
      case (3):
         s64_temp_gear_limits_mode = 28;  // Activate ACC/DEC/VLIM as velocity synchronization and put the PTP hunter in mode 1 (reach target and allow overshoot)
      break;
      default:
         s64_temp_gear_limits_mode = -1; // Set temporary gear limits mode to an invalid value
      break;
   }

   if (s64_temp_gear_limits_mode != -1)
   {
      s64_temp_gear_limits_mode |= (long long)(VAR(AX0_u8_Gear_Limits_Mode) & 0x0002LL);   // preserve bit 1 of GEARLIMITSMODE (see P2-65)

      // Set the gear-limits-mode parameter
      s16_return_value = SalGearLimitsModeCommand(s64_temp_gear_limits_mode, drive);
   }
   else
   {
      s16_return_value = VALUE_OUT_OF_RANGE;
   }

   return (s16_return_value);
}

//**********************************************************
// Function Name: SalGearInCommand
// Description:
//          This function is called in response to the GEARIN command.
//
// Author: Dimitry
// Algorithm:
// Revisions: Nitsan 25/11/2014: consider p1-00 nibble C (ext input polarity) when setting gearin
//          : Avishay 4/10/2015: handling p1-00 nibble C was moved to SetupGear(), to be at one place.
//**********************************************************
int SalGearInCommand(long long param, int drive)
{
   // AXIS_OFF;

   LVAR(AX0_s32_Gearin_Design) = param;

   DesignGearMsqFilter(drive);
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalGearModeCommand
// Description:
//          This function is called in response to the GEARMODE command.
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalGearModeCommand(long long param, int drive)
{
   if (param == 5LL) return (VALUE_TOO_HIGH);
   if ((u16_Product == DDHD) && (param > 2LL)) return (VALUE_TOO_HIGH);
//   if ((u16_Product == SHNDR_HW) && (param > 2LL)) return (VALUE_TOO_HIGH);
   if ((IS_SECONDARY_FEEDBACK_ENABLED) && (param > 2LL)) return (INVALID_DF_GEARMODE);

   VAR(AX0_u8_Gear_Mode) = param;

   PositionConfig(drive,0); // Called to update Ext. Enc. Fix and Shift-Left variables
   DesignGearMsqFilter(drive);

   SetupGear(drive);

   return (SAL_SUCCESS);
}

unsigned int u16_PD_Mux_Debug = 0;
// Handle FPGA Multiplex to select between controller I/O and fast I/O P&D gearing inputs
void PulseandDirectionFpgaMUX(int drive)
{
   unsigned int u16_input_pulse = IsInFunctionalityConfigured(HIGH_SPD_PUSLE_INP, drive);
   unsigned int u16_input_dir = IsInFunctionalityConfigured(HIGH_SPD_DIR_INP, drive);

   if (u16_Product == DDHD)
   { /*
      if ((1 == u16_input_pulse) && (2 == u16_input_dir))
         *(int*)FPGA_PULSE_DIRECTION_INPUT_REG_ADD = 3;
      else if ((3 == u16_input_pulse) && (4 == u16_input_dir))
         *(int*)FPGA_PULSE_DIRECTION_INPUT_REG_ADD = 4;
      else
         *(int*)FPGA_PULSE_DIRECTION_INPUT_REG_ADD = 0; */
      if ((5 == u16_input_pulse) && (6 == u16_input_dir))
      // Backward Compatibility, set Bit 0 for Pulse at Input 5 and Bit 1 for
      {
         u16_PD_Mux_Debug = 3;
         *(int*)FPGA_PULSE_DIRECTION_INPUT_REG_ADD = u16_PD_Mux_Debug; // Direction at Input 6.
      }
      else if ( ((0 < u16_input_pulse) && (5 > u16_input_pulse)) &&
                ((0 < u16_input_dir) && (5 > u16_input_dir))       )
      // Direction Source at upper Digit and Pulse Source at next Digit.
      {
         u16_PD_Mux_Debug = (u16_input_pulse << 8) | (u16_input_dir << 12);
         *(int*)FPGA_PULSE_DIRECTION_INPUT_REG_ADD = u16_PD_Mux_Debug;
      }
      else // Discrete Input Configuration not valid for P&D, select P&D Dedicated Inputs.
      {
         u16_PD_Mux_Debug = 0;
         *(int*)FPGA_PULSE_DIRECTION_INPUT_REG_ADD = u16_PD_Mux_Debug;
      }
   }
   else
   {
       if ((5 == u16_input_pulse) && (6 == u16_input_dir))
         *(int*)FPGA_PULSE_DIRECTION_INPUT_REG_ADD = 3;
       else
         *(int*)FPGA_PULSE_DIRECTION_INPUT_REG_ADD = 0;
   }

   DetermineDetectionOfPDBrokenWire(drive);
}


unsigned long int u32_ValueToSendToFPGA = 0L;

int UpdatePulseandDirectionLineBrakeFilterInFpga()
{
    //                                                                                                           *in RPS*
    //  Calculate the maximum allowed rate at which we need to receive Pulse and Direction indication (MARGIN * (VLIM / 60) * (GEARIN / GEAROUT) * XENCRES)
    u32_ValueToSendToFPGA = ((1200 / 1000) * BGVAR(s32_V_Lim_Design) * LVAR(AX0_s32_Gearin_Design) * BGVAR(u32_External_Enc_Res)) / (60 * LVAR(AX0_u32_Gearout_Design));
    //  This should be altered to the result of the write to the FPGA
    return SAL_SUCCESS;
}

void SetupGear(int drive)
{
   REFERENCE_TO_DRIVE;

   PulseandDirectionFpgaMUX(drive);
// (Machine and controller encoders counters are continuously stored at 32 bit registers
//  _AX0_s32_Encoder_Follower2_Counter
//  _AX0_s32_Encoder_Follower3_Counter
   VAR(AX0_s16_Encoder_Follower3_Ptr) = (int)((long)&EQep3Regs.QPOSCNT & 0xffff);
   if (BGVAR(u16_Gear_In_Mode) == 0)
   {
       VAR(AX0_s16_Encoder_Follower2_Ptr) = FPGA_PULSE_DIRECTION_REG_ADD;
   }
   else
   {
       VAR(AX0_s16_Encoder_Follower2_Ptr) = FPGA_PULSE_DIRECTION_OVT_REG_ADD;
   }

   switch (VAR(AX0_u8_Gear_Mode))
   {
      case 0: // Encoder Follower at Stepper (Controller Interface J2) Input
         EQep3Regs.QDECCTL.bit.QSRC = 0; // Quadrature Count Mode for Sec. Enc. Input.
         EQep3Regs.QDECCTL.bit.XCR = 0; // counting each Edge
         if (BGVAR(u16_Gear_In_Mode) == 0)
            VAR(AX0_s16_2ndQep_Ptr) = FPGA_PULSE_DIRECTION_REG_ADD;     // P&D Input
         else
            VAR(AX0_s16_2ndQep_Ptr) = FPGA_PULSE_DIRECTION_OVT_REG_ADD; // P&D Input with 1/T (*16)
         *(int*)FPGA_PULSE_DIRECTION_MODE_REG_ADD = 0x02; // Quadrature Mode
         if (!IS_SECONDARY_FEEDBACK_ENABLED)
            BGVAR(s64_Faults_Mask) &= ~SEC_LINE_BRK_FLT_MASK; // Disable detection of 2nd Encoder A&B Inputs Broken Wires.
      break;

      case 1: // Step-and-Direction Follower at Stepper (Controller Interface J2) Input
         EQep3Regs.QDECCTL.bit.QSRC = 0; // Quadrature Count Mode for Sec. Enc. Input.
         EQep3Regs.QDECCTL.bit.XCR = 0; // counting each Edge
         if (BGVAR(u16_Gear_In_Mode) == 0)
            VAR(AX0_s16_2ndQep_Ptr) = FPGA_PULSE_DIRECTION_REG_ADD;     // P&D Input
         else
            VAR(AX0_s16_2ndQep_Ptr) = FPGA_PULSE_DIRECTION_OVT_REG_ADD; // P&D Input with 1/T (*16)
         *(int*)FPGA_PULSE_DIRECTION_MODE_REG_ADD = 0x00; // Pulse&Dir Mode
         if (!IS_SECONDARY_FEEDBACK_ENABLED)
            BGVAR(s64_Faults_Mask) &= ~SEC_LINE_BRK_FLT_MASK; // Disable detection of 2nd Encoder A&B Inputs Broken Wires.
      break;

      case 2: // Up/Down Follower at Stepper (Controller Interface J2) Input
         if (BGVAR(u16_Gear_In_Mode) == 0)
            VAR(AX0_s16_2ndQep_Ptr) = FPGA_PULSE_DIRECTION_REG_ADD;     // P&D Input
         else
            VAR(AX0_s16_2ndQep_Ptr) = FPGA_PULSE_DIRECTION_OVT_REG_ADD; // P&D Input with 1/T (*16)
         *(int*)FPGA_PULSE_DIRECTION_MODE_REG_ADD = 0x01; // Up&Down Mode
         if (!IS_SECONDARY_FEEDBACK_ENABLED)
            BGVAR(s64_Faults_Mask) &= ~SEC_LINE_BRK_FLT_MASK; // Disable detection of 2nd Encoder A&B Inputs Broken Wires.
      break;

      case 3: // Encoder Follower at QEP3 Input
         VAR(AX0_s16_2ndQep_Ptr) = (int)((long)&EQep3Regs.QPOSCNT & 0xffff);
         EQep3Regs.QDECCTL.bit.QSRC = 0; // Quadrature Count Mode for Sec. Enc. Input.
         EQep3Regs.QDECCTL.bit.XCR = 0; // counting each Edge
         BGVAR(s64_Faults_Mask) |= SEC_LINE_BRK_FLT_MASK; // Enable detection of 2nd Encoder A&B Inputs Broken Wires.
      break;

      case 4: // Pulse-and-Direction at QEP3 Input
         VAR(AX0_s16_2ndQep_Ptr) = (int)((long)&EQep3Regs.QPOSCNT & 0xffff);
         EQep3Regs.QDECCTL.bit.QSRC = 1; // Pulse-and-Direction Mode for Sec. Enc. Input.
         EQep3Regs.QDECCTL.bit.XCR = 1;  // counting Rising Edge only
         BGVAR(s64_Faults_Mask) |= SEC_LINE_BRK_FLT_MASK; // Enable detection of 2nd Encoder A&B Inputs Broken Wires.
      break;

      case 5: //  Up/Down Follower at QEP3 Input
         // Add Configuration and Setup Code
         //BGVAR(s64_Faults_Mask) |= SEC_LINE_BRK_FLT_MASK; // Enable detection of 2nd Encoder A&B Inputs Broken Wires.
      break;

      default:
      break;
   }

   DetermineDetectionOfPDBrokenWire(drive);
}


//**********************************************************
// Function Name: DetermineDetectionOfPDBrokenWire
// Description:
//          Determine the detection of the P&D Input Broken Wire.
//          Enable the detection if OPMODE=4, GEARMODE=0,1,2, and the signals don't come from digital inputs.
//          Otherwise the detection is disabled.
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
void DetermineDetectionOfPDBrokenWire(int drive)
{
   // AXIS_OFF;

    int u16_input_pulse, u16_input_dir;
    u16_input_pulse =  IsInFunctionalityConfigured(HIGH_SPD_PUSLE_INP, drive);
    u16_input_dir   =  IsInFunctionalityConfigured(HIGH_SPD_DIR_INP, drive);

   if ( (VAR(AX0_s16_Opmode) == 4)
              &&
        (VAR(AX0_u8_Gear_Mode) <= 2)
              &&
        (((u16_Product != DDHD) &&
           ((u16_input_pulse != 5) || ( u16_input_dir   != 6) ))
                         ||
        ((u16_Product == DDHD) && /* In case of DDHD, both digital inputs 1&2 or 3&4 should be configured to P&D to disable detection*/
           ((((u16_input_pulse != 1) || (u16_input_dir != 2))  &&  ((u16_input_pulse != 3) || (u16_input_dir != 4))     ))   ))
      )
      BGVAR(s64_Faults_Mask) |= PD_LINE_BRK_FLT_MASK;  // Enable detection of P&D Input Broken Wire.
   else
      BGVAR(s64_Faults_Mask) &= ~PD_LINE_BRK_FLT_MASK; // Disable detection of P&D Input Broken Wire.

}


//**********************************************************
// Function Name: SalGearOutCommand
// Description:
//          This function is called in response to the GEAROUT command.
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalGearOutCommand(long long param, int drive)
{
   // AXIS_OFF;
   LVAR(AX0_u32_Gearout_Design) = param;
   DesignGearMsqFilter(drive);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalWriteGearOutDenominatorCommand
// Description:
//          This function is called in response to the P1-45 command.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteGearOutDenominatorCommand(long long param,int drive)
{
   int s16_retVal=0;

   // 22.5.14 - as discussed with Avishay & Joachim, due to PUU effect on internal values of parameters (e.g. SW limit)
   // it was decided to prevent writing P1-44 and P1-45 via CANOpen channel.
   // 5.10.14 - Udi: Allow set gearing when Downloading a parameter file.
   //           This is indicated by "Config_Lock == Write"

   if ((((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_CANOPEN)      ||
        ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)       ||
        ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)  ||
        ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)        )   &&
       (u16_Config_Lock != CONFIG_LOCK_WRITE)                                          )
      return (INVALID_OPMODE);

   s16_retVal = SalGearOutCommand(param, drive);

   if (s16_retVal != SAL_SUCCESS)
   {
      return s16_retVal;
   }

   // update the new gear ratio and PUU unit conversion fix shift for Schneider
   SchneiderCalcPuuUnits(drive);
   return s16_retVal;
}


// DISSPEED
int SalDisSpeedCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL) return (VALUE_TOO_LOW);
   if (lparam > (long long)2147483647) return (VALUE_TOO_HIGH);

   BGVAR(u32_DisSpeed) = (long)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: CalcDecelerationTimeout
// Description:
//          This function calculates the timeout interval for decelaration from the current V until standstill.
//          Returned value is in ms.
//          It is possible to add overhead to the total timeout value.
//          in parameters:
//          u64_dec: decelaration value (internal value, the unit must be [Count64/Tsp^2], which means 2^64[Counts per rev] and Tsp = position loop sample rate (125[us] in POSCONTROLMODE>=5 or 250[us] if POSCONTROLMODE < 5)
//          u16_min_dec_time: minimum deceleration time (in ms). If this function argument is 0, then it will not be considered in the code.
//          u16_torque_mode_timeout_ms: timeout for torque modes (OPMODE 2 = serial and 3 = analog), since no controlled decelration is used.
//          u16_overhead_ms: total timeout is 1.2 multiplied the calculated value or minimum of this u16_overhead_ms.
//
// Author: Nitsan
// Algorithm: The code of this function is originally taken from ActiveDisableHandler() function.
// Revisions:
//**********************************************************
unsigned long CalcDecelerationTimeout(int drive, unsigned long long u64_dec, unsigned int u16_min_dec_time_ms, unsigned int u16_torque_mode_timeout_ms, unsigned int u16_overhead_ms)
{
   unsigned long u32_over_timeout;
   float f_abs_vel = 0.0f;

   // AXIS_OFF;

   // Calculate TimeOut according to velocity and DECSTOP
   if (((VAR(AX0_s16_Opmode) == 2) || (VAR(AX0_s16_Opmode) == 3)) && !IsVelControlledRampDownInOpmodeTrqActive(drive))
   {
      // In OPMODE torque with a deactivated feature "vel controlled ramp-down
      // in mode torque" we use 5[s] for coasting to a stop (which means just
      // clearing the torque command value)
      u32_over_timeout = u16_torque_mode_timeout_ms;
   }
   else // Calculate TimeOut according to velocity and DECSTOP
   {
      // Calculate TimeOut according to V and u64_dec (or u16_min_dec_time_ms (e.g. DECSTOPTIME))
      // since V = VCMD_internal * s32_V_Lim_Design * 0.0000049124
      // and DEC = DECSTOP_internal * 0.0000000520417042793042
      // V/DEC [sec] = s32_V_Lim_Design * 94.393 (the same ratio remains with other units as the calculation is done in internal units)
      f_abs_vel = (float)VAR(AX0_s16_Vel_Var_Ref_0); // Read VCMD

      // On position HDC use actual velocity not commanded one as the command V is not updated
      if (((VAR(AX0_s16_Opmode)==4) || (VAR(AX0_s16_Opmode)==8)) && (VAR(AX0_u16_Pos_Control_Mode) != LINEAR_POSITION))
      {
         f_abs_vel = (float)VAR(AX0_s16_Vel_Var_Fb_0); // Read V
      }
      else if (IsVelControlledRampDownInOpmodeTrqActive(drive))
      {
         // Also in this case use actual velocity because in previous OPMODE
         // torque the command value V is also not updated.
         f_abs_vel = (float)VAR(AX0_s16_Vel_Var_Fb_0); // Read V
      }

      if (f_abs_vel < 0) f_abs_vel = -f_abs_vel;

      // The following formula calculates the deceleration time in [ms].
      //
      // 1) "BGVAR(s32_V_Lim_Design)" is always in unit [Counts32/125us].
      //    "f_abs_vel" is always in unit [22750 = VLIM]
      //    velocity[rev/s] = (f_abs_vel / 22750) * BGVAR(s32_V_Lim_Design) * (8000 / 2^32)
      // 2) "u64_dec" is in unit [Counts64/Tsp^2]. That means:
      //      a) If POSCONTROLMODE < 5 (Tsp = 250us):  deceleration[rev/s^2] = u64_dec[Counts64/(250us)^2] * (4000^2 / 2^64)
      //      b) If POSCONTROLMODE >= 5 (Tsp = 125us): deceleration[rev/s^2] = u64_dec[Counts64/(125us)^2] * (8000^2 / 2^64)
      //
      // The time it takes for decelerating to 0 can be calculated as follows: dec_time[s] = velocity[rev/s] / deceleration[rev/s^2]
      // For POSCONTROMODE<5:
      //   dec_time[s] = (f_abs_vel / 22750) * BGVAR(s32_V_Lim_Design) * (8000 / 2^32) / (u64_dec[Counts64/125us] * (4000^2 / 2^64))
      //               = f_abs_vel * BGVAR(s32_V_Lim_Design) * (8000*2^64/(22750*2^32*4000^2) / u64_dec
      //               = f_abs_vel * BGVAR(s32_V_Lim_Design) * 94.394 / u64_dec
      // For POSCONTROMODE>=5:
      //   dec_time[s] = (f_abs_vel / 22750) * BGVAR(s32_V_Lim_Design) * (8000 / 2^32) / (u64_dec[Counts64/125us] * (8000^2 / 2^64))
      //               = f_abs_vel * BGVAR(s32_V_Lim_Design) * (8000*2^64/(22750*2^32*8000^2) / u64_dec
      //               = f_abs_vel * BGVAR(s32_V_Lim_Design) * 23.598 / u64_dec
      //
      // Calculate the timeout in [ms] (factor multiplied by 1000):
      u32_over_timeout = (long)(f_abs_vel * (float)BGVAR(s32_V_Lim_Design) * 94393.0 / (float)u64_dec);
      if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)
      {
         u32_over_timeout = u32_over_timeout / 4;  // Correct the expected deceleration time, divide by 4 since sample time
                                                   // contributes in a power of 2 for calculating the deceleration time.
      }
      if (u16_min_dec_time_ms)
      {
         if (u32_over_timeout < (long)u16_min_dec_time_ms) u32_over_timeout = (long)u16_min_dec_time_ms;
      }

      //Add 20% for overhead or u16_overhead_ms whichever is larger
      if ((0.2 * (float)u32_over_timeout) > (float)u16_overhead_ms)
         u32_over_timeout = (long)(1.2 * (float)u32_over_timeout);
      else
         u32_over_timeout += (unsigned long)u16_overhead_ms;
   }

   return u32_over_timeout;
}


// The description for this procedure can be found at "CDHD DisableMode Spec Rev x_0.doc"
// Same procedure will be activate on regular disable (which is a sub-set of AD)
void ActiveDisableHandler(int drive)
{
   // AXIS_OFF;
   unsigned long long u64_local_decstop = BGVAR(u64_DecStop);
   long s32_abs_vel = 0L;
//   float f_abs_vel = 0.0;

   // On SE "DISTIME" is composed from P8-43+uiBrakeCouplingTime (read from the MTP) on CDHD it will be equal to DISTIME (as u16_Motor_Brake_Engage_Time==0)
   long s32_disable_time = (long)BGVAR(u16_DisTime) + (long)BGVAR(u16_Motor_Brake_Engage_Time);

   switch (BGVAR(u16_AD_State))
   {
      case AD_IDLE:
         // Restore user gain reduced when NLTUNE=6
         if (u16_Vbl_Exceeded)
         {
            u16_Vbl_Exceeded = 0;
            SalNlKpgfCommand((long long)u32_Orig_Knlusergain, drive);
         }
      break;

      case AD_START_RAMP_DOWN:
         // For FC, On Gearing with E-STOP (or AC OFF) use special Ramp-Down profile
         if (VAR(AX0_s16_Opmode) == 4)
         {
//            VAR(AX0_u16_HP_Flags) |= EMERGENCY_STOP_ISSUED_MASK;

            if (BGVAR(u16_EStop_Indication))
               VAR(AX0_u16_BrakeOn) = 1;    // Lock brake immediately on EStop
         }

         // set the required deceleration
         BGVAR(u16_Deceleration_Event_ID) = EVENT_DECSTOP;

         // special for LXM node guard fault event change "DECSTOP" to "u64_Ser_Comm_Time_Out_Dec_Rate" (P5-21) deceleration
         // only for the "RAMP DOWN" disable time calculation.
         // FIX BUG#4056
         if ((u16_Modbus_NodeGuard_Flt & MODBUS_NODEGUARD_AD_INDC_MASK) && ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)))
         {// if LXM drive and NODEGuard event executed use nodeguard event deceleration

            BGVAR(u16_Deceleration_Event_ID) = EVENT_COMM_TIMED_OUT_DEC;

            // update local decstop to nodegaurd event deceleration
            u64_local_decstop = BGVAR(u64_Ser_Comm_Time_Out_Dec_Rate);

            // reset the bit that was rised by the "FLTCTRL" to prevent multi calls
            u16_Modbus_NodeGuard_Flt &= ~MODBUS_NODEGUARD_AD_INDC_MASK;
         }

         // set the AD hold bit
         BGVAR(u16_Hold_Bits) |= HOLD_AD_MASK;
         // call the hold scan without waiting for BG. This HoldScan function call
         // is required since the state machine in hold scan jumps directly to a state
         // where the feature "velocity controlled ramp-down in OPMODE torque is being
         // applied. This is required for the upcoming "IsVelControlledRampDownInOpmodeTrqActive(drive)"
         // function calls.
         HoldScan(drive);


         BGVAR(s32_AD_Overall_Timeout) = CalcDecelerationTimeout(drive, u64_local_decstop, BGVAR(u16_DecStopTime), 5000, 300);


         // On Emergency Stop limit the overall procedure to DISTIME

         BGVAR(s32_Ilim_Active_Disable) = BGVAR(s32_Ilim_User);
         if (BGVAR(s16_Ilim_Active_Disable_Factor) < 1000)
         {
            BGVAR(s32_Ilim_Active_Disable) = ((long long)BGVAR(s32_Ilim_User) * BGVAR(s16_Ilim_Active_Disable_Factor)) / 1000;
         }

         BGVAR(s32_AD_Timer) = Cntr_1mS;
         BGVAR(s32_AD_Start_Time) = BGVAR(s32_AD_Timer);

         BGVAR(u16_AD_In_Process) = 1;
         BGVAR(StepHandlerState) = STEP_IDLE;

         BGVAR(s32_AD_Overall_Timeout_Timer) = Cntr_1mS;
         BGVAR(u16_AD_State) = AD_WAIT_FOR_FIRST_DISSPEED;
      break;

      case AD_WAIT_FOR_FIRST_DISSPEED: // Wait till actual velocity drops first time below DISSPEED
         if (PassedTimeMS(BGVAR(s32_AD_Overall_Timeout), BGVAR(s32_AD_Overall_Timeout_Timer)))
         {
            if (VAR(AX0_u16_BrakeOn))
               BGVAR(u16_AD_State) = AD_DISABLE_DRIVE;
            else
            BGVAR(u16_AD_State) = AD_START_DISTIME_COUNTER;
         }
         else
         {
            s32_abs_vel = LVAR(AX0_s32_Vel_Var_Fb_0);
            if (s32_abs_vel < 0L) s32_abs_vel = -s32_abs_vel;

            if (s32_abs_vel <= BGVAR(u32_DisSpeed))
            {
               BGVAR(s32_AD_Disspeed_Qual_Timer) = Cntr_1mS;
               BGVAR(u16_AD_State) = AD_WAIT_FOR_DISSPEED_QUAL;
            }
         }
      break;

      // Wait till actual velocity is below DISSPEED 50ms (or overall timeout expired)
      case AD_WAIT_FOR_DISSPEED_QUAL:
         if (PassedTimeMS(BGVAR(s32_AD_Overall_Timeout), BGVAR(s32_AD_Overall_Timeout_Timer)))
         {
            if (VAR(AX0_u16_BrakeOn))
               BGVAR(u16_AD_State) = AD_DISABLE_DRIVE;
            else
            BGVAR(u16_AD_State) = AD_START_DISTIME_COUNTER;
         }
         else
         {
            s32_abs_vel = LVAR(AX0_s32_Vel_Var_Fb_0);
            if (s32_abs_vel < 0L) s32_abs_vel = -s32_abs_vel;

            if (s32_abs_vel > BGVAR(u32_DisSpeed))
               BGVAR(u16_AD_State) = AD_WAIT_FOR_FIRST_DISSPEED;

            if (PassedTimeMS(50L, BGVAR(s32_AD_Disspeed_Qual_Timer)))
                  BGVAR(u16_AD_State) = AD_START_DISTIME_COUNTER;
            }
      break;

      case AD_START_DISTIME_COUNTER: // Lock brake
      case AD_START_DISTIME_COUNTER_REGULAR:
         BGVAR(u16_AD_In_Process) = 1;
         BGVAR(StepHandlerState) = STEP_IDLE;

         VAR(AX0_u16_BrakeOn) = 1;            // Close Brake
         BGVAR(s32_AD_Timer) = Cntr_1mS;

         if (BGVAR(u16_AD_State) == AD_START_DISTIME_COUNTER_REGULAR)
            BGVAR(u16_AD_State) = AD_WAIT_DISTIME_EXPIRE_REGULAR;
         else
            BGVAR(u16_AD_State) = AD_WAIT_DISTIME_EXPIRE;
      break;

      case AD_WAIT_DISTIME_EXPIRE: // Wait DISTIME before disabling the drive
      case AD_WAIT_DISTIME_EXPIRE_REGULAR:
         if (PassedTimeMS(s32_disable_time, BGVAR(s32_AD_Timer)))
         {
            if (BGVAR(u16_AD_State) == AD_WAIT_DISTIME_EXPIRE_REGULAR)
               BGVAR(u16_AD_State) = AD_DISABLE_DRIVE_REGULAR;
            else
               BGVAR(u16_AD_State) = AD_DISABLE_DRIVE;
         }
      break;

      case AD_DISABLE_DRIVE:
      case AD_DISABLE_DRIVE_REGULAR:
         SignalRTToDisable(drive);

         VAR(AX0_s16_Serial_Crrnt_Cmnd) = 0;
         BGVAR(s32_Serial_Torque_Cmd) = 0L;

       // reset all the saved torque (T command) data.
         resetTorqueParamsToDefault(drive);

         VAR(AX0_s16_Serial_Vel_Cmnd) = 0;

         // *************************************************************************************
         // The following lines are important for the generation of the STOPPED
         // signal (see AX0_s16_Stopped) in OPMODE 0 (if the HD controller is active).
         // If we would releaset he HOLD_AD_MASK before these variables come to 0, then
         // the STOPPED indication would jump from -1 (movement interrupted) to 0 (motor moving),
         // which is handled in the RT code and which can interrupt the BG code.
         VAR(AX0_s16_Vel_Acc_Dec_Cmnd) = 0;
         // Load the init counter with the buffer-depth, which starts the initialization process.
         // This ensures that the filter memory does output straight away a 0 signal.
         VAR(AX0_s16_Vel_Moving_Average_Init_Cntr) = VAR(AX0_u16_Vel_Moving_Average_Buffer_Depth);
         VAR(AX0_s16_Vel_Acc_Dec_Cmnd_Filtered) = 0;
         LVAR(AX0_s32_NL_Vel_Loop_Cmd_Src) = 0;
         // End of code that ensures a proper value for the STOPPED (AX0_s16_Stopped) indication.
         // *************************************************************************************

         VAR(AX0_s16_Crrnt_Lp_Prev) = 0;
         BGVAR(s32_Jog) = 0;

         // release the AD hold bit and call the hold scan without waiting for BG.
         BGVAR(u16_Hold_Bits) &= ~HOLD_AD_MASK;
         HoldScan(drive);

         BGVAR(u8_Sensor_Adjust_State) = ADJ_WAIT;         // Enable ia,ic offset calibration

         BGVAR(s32_Ilim_Active_Disable) = BGVAR(s32_Ilim_User);

         BGVAR(u16_AD_State) = AD_IDLE;            // Reset AD states
         BGVAR(u16_AD_In_Process) = 0;
         VAR(AX0_u16_BrakeOn) = 1;

         VAR(AX0_u16_HP_Flags) &= ~EMERGENCY_STOP_ISSUED_MASK;
         // Restore user gain reduced when NLTUNE=6
         if (u16_Vbl_Exceeded)
         {
            u16_Vbl_Exceeded = 0;
            SalNlKpgfCommand((long long)u32_Orig_Knlusergain,drive);
         }

       //clear the error in position command from Fieldbus notification
       VAR(AX0_s16_Fieldbus_Postion_Cmd_Err_Flag) = 0;


       if (BGVAR(u16_Son_Rising_Edge_detected) == 3)
       {// reset flag to "0" - idle state.
         BGVAR(u16_Son_Rising_Edge_detected) = 0;
       }
       else if (BGVAR(u16_Son_Rising_Edge_detected) == 1)
       {// change the rising edge flag from 1 (enable pending) to 2 (preform enable automatic in next BG)
         BGVAR(u16_Son_Rising_Edge_detected) = 2;
       }
      break;
   }
}


void SignalRTToDisable(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   // Select between dynamic-brake and regular disable
   if ( (BGVAR(u16_Disable_Mode) == 2) || (BGVAR(u16_Disable_Mode) == 5)                   ||
        ( (BGVAR(u16_Disable_Mode) == 1) && (BGVAR(s16_DisableInputs) & FLT_EXISTS_MASK) ) ||
        ( (BGVAR(u16_Disable_Mode) == 4) && (BGVAR(s16_DisableInputs) & FLT_EXISTS_MASK) )   )
   {
      if (BGVAR(s64_SysNotOk) & OVER_CRRNT_FLT_MASK)  // over current fault exists
         VAR(AX0_s16_Transition_To_Disable) = 1;      // Signal the real-time to disable the PWM
      else
         VAR(AX0_s16_Transition_To_Disable) = 2;      // Signal the real-time to start DB
   }
   else
      VAR(AX0_s16_Transition_To_Disable) = 1;   // Signal the real-time to disable the PWM
}


int SalSetTorqueSlopeEnableCommand(long long param, int drive)
{
   BGVAR(u16_Icmd_Slope_Use)= (int)param;
   IcmdSlopeEn(drive);
   return (SAL_SUCCESS);
}


int SalDisableModeCommand(long long param, int drive)
{
   if (BGVAR(u16_Disable_Mode) == (int)param) return (SAL_SUCCESS);

   BGVAR(u16_Disable_Mode) = (int)param;
   BGVAR(u16_Disable_Mode_User) = (int)param;

   if (!Enabled(drive))
      SignalRTToDisable(drive);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: PllHandler
// Description:
//          This function is PLL synchroniztion handler
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void PllHandler(int drive)
{
   // AXIS_OFF;

   int u16_i = 0, s16_diff_pll_correction;
//   float f_interpolation_time;
//   int s16_temp;
   static int sync_signal_counter = 0;

   if (BGVAR(u8_Sync_Mode) == 0)
   {
      u16_Pll_Flags = 0;
      u16_Pll_State = PLL_IDLE;
   }

   if (IS_CAN_DRIVE_AND_COMMODE_1)
   {
      // On CAN, When Sync signal is alive, automaticaly change SYNCSOURCE to 6 to allow pll lock
      if ((BGVAR(u8_Sync_Mode) != 6) && u16_CAN_commands_register)
         SyncModeCommand(6LL,drive);

      // Try to re-sync automatically
      if (u16_Pll_State == PLL_UNLOCKED) u16_Pll_State = PLL_IDLE;
   }
   else if (IS_EC_DRIVE_AND_COMMODE_1)
   {
      // On Ethercat, When Sync signal is alive, automaticaly change SYNCSOURCE to 5 to allow pll lock
      if ((BGVAR(u8_Sync_Mode) != 5) && (u16_EC_commands_register & EC_CWORD_SYNC_SIGNAL_ALIVE_MASK))
         SyncModeCommand(5LL,drive);

      // Try to re-sync automatically
      if (u16_Pll_State == PLL_UNLOCKED) u16_Pll_State = PLL_IDLE;
   }
   else if (u16_Product == DDHD)
   {
      if (BGVAR(u8_Sync_Mode) == 0)
         SyncModeCommand(7LL,drive);

      // Try to re-sync automatically
      if (u16_Pll_State == PLL_UNLOCKED) u16_Pll_State = PLL_IDLE;
   }
   else if (VAR(AX0_u16_Gantry_Mode) == 2) // If the Drive is working in Gantry mode and operates as axis Y2
   {
      if (BGVAR(u8_Sync_Mode) != 8)
      {
         SyncModeCommand(8LL,drive);
      }

      // Try to re-sync automatically
      if (u16_Pll_State == PLL_UNLOCKED) u16_Pll_State = PLL_IDLE;
   }

   switch (u16_Pll_State)
   {
      case PLL_IDLE: // check if one fast inputs assigned for synq signal
         s16_Pll_Correction = 0;
         s16_Prev_Pll_Correction = s16_Pll_Correction;
         if (BGVAR(u8_Sync_Mode) == 6)     u16_Pll_State = PLL_CONFIG;   // CAN PLL uses different mechanism (DSP Timer0)
         else if (BGVAR(u8_Sync_Mode) > 0) u16_Pll_State = PLL_INIT;
      break;

      case PLL_INIT:
         // On ethercat lock to MTS 5 otherwise lock to 0 (CAN PLL doesn't get to this state)
         if (BGVAR(u8_Sync_Mode) == 5)
         {
            u16_Pll_Lock_Mts_Slot = 5;
         }
         else if (BGVAR(u8_Sync_Mode) == 8)
         {
            // In Gantry mode the Y1 sync telegram is triggered in MTS_7 but the
            // y2 sync signal is generated at MTS_0, so Y2 needs to get synchronized
            // with MTS_0.
            u16_Pll_Lock_Mts_Slot = 0;
         }
         else
         {
            u16_Pll_Lock_Mts_Slot = 0;
         }

         if (BGVAR(u8_Sync_Mode) == 8) // If Gantry mode
         {
            // For Gantry mode adjust the FPGA register to 5 in order to allow
            // the sync signal to be handled properly.
            *(int*)FPGA_FAST_IN_MUX_REG_ADD     = 5;
         }
         else
         {
            *(int*)FPGA_FAST_IN_MUX_REG_ADD     = BGVAR(u8_Sync_Mode) - 1;
         }

         *(int*)FPGA_SYNC_SIGNAL_EN_REG_ADD  = 1; // Enable syncronization in FPGA
         *(int*)FPGA_SYNC_SIGNAL_FREQ_DIVIDE_REG_ADD  = 0;
         u32_Controller_Sync_Period = 0L;
         sync_signal_counter = 0;
         u16_Pll_State = PLL_WAIT_SYNC_SIGNAL;
      break;

      case PLL_WAIT_SYNC_SIGNAL:  //wait for sync signal stable
         if (u32_Controller_Sync_Period > 0L) // If the FPGA measured a time between 2 sync signals
         {
            if (sync_signal_counter == 10)
            {
               if (u32_Controller_Sync_Period > 30000000) u32_Controller_Sync_Period = 30000000;

               // The division by 1875 represents a unit conversion from 60[MHz] (this is the frequency
               // of the FPGA counter that is loaded to u32_Controller_Sync_Period) into 32[kHz], since
               // 60[MHz] / 32[kHz] = 1875. The value 937 is used to prevent truncation issues due to
               // inaccuracy in the equidistance of the arriving sync-signal.
               u32_Controller_Sync_Period_in_32Khz = (u32_Controller_Sync_Period + 937L) / 1875L;  // max result is 16000
            }

            u32_Controller_Sync_Period = 0L;
            sync_signal_counter++;
         }
         if (sync_signal_counter > 10) u16_Pll_State = PLL_CONFIG;
      break;

      case PLL_CONFIG:
         if (BGVAR(u8_Sync_Mode) == 6) // CAN Drive
         {
            // calc number of sync periods to average.
            // average every about 200ms, thus:
            // number of sync periods to average = 200ms/sync_period + round = 800/sync_period_250us + round
            // min value is 2 and max value is 100
            // the above numbers (average period, min, max) are arbitrary choise based on tests with GT controller
            u16_CAN_Sync_Periods_To_Average = (unsigned int)((800L + (u32_CAN_Sync_250us_Length >> 1)) / u32_CAN_Sync_250us_Length);
            if (u16_CAN_Sync_Periods_To_Average < 2) u16_CAN_Sync_Periods_To_Average = 2;
            if (u16_CAN_Sync_Periods_To_Average > 100) u16_CAN_Sync_Periods_To_Average = 100;

            u32_Controller_Sync_Period_in_32Khz = (u32_CAN_Sync_250us_Length << 3) * u16_CAN_Sync_Periods_To_Average;
            u32_Pll_Max_Delta = 192L * u32_Controller_Sync_Period_in_32Khz;
            u16_Num_Of_Mts_In_Sync_Period = (unsigned int)(u32_CAN_Sync_250us_Length << 3);
            u16_Pll_Flags |= PLL_ENABLED_MASK;
            u16_Pll_State = PLL_WAIT_FOR_LOCK;
         }
         else
         {
            //find common denumerator between drive frequency and controller frequency
            if ((BGVAR(u8_Sync_Mode) == 5) ||      // ECAT Drive
                (BGVAR(u8_Sync_Mode) == 8))        // Gantry Drive (uses the same method as ECAT)
            {
               u16_i = 1;
               // This is to support 125usec period, or N*250usec
               if (u32_Controller_Sync_Period_in_32Khz == 4)
               {
                  u32_Controller_Sync_Period_in_32Khz = 8;
                  u16_i = 2;
               }
               else if ( (((u32_Controller_Sync_Period_in_32Khz / 8) << 3) != u32_Controller_Sync_Period_in_32Khz) ||   // If not a multiple of 250[us]
                          (u32_Controller_Sync_Period_in_32Khz == 0)                                                 )
                  u16_i = 0;

               while (u32_Controller_Sync_Period_in_32Khz < 32)    // do the pll calculations not faster than 1 ms
               {
                  // Here set variables in a way that the PLL runs at 1[ms] or slower. For this an FPGA sync divider register will be set
                  // which ensures that only every x sync-signals that sync-event is shown to the DSP.
                  u32_Controller_Sync_Period_in_32Khz <<= 1;   // Multiply the variable "u32_Controller_Sync_Period_in_32Khz" by 2
                  u16_i <<= 1;                                 // Multiply also u16_i by 2, which will be written to the sync divider register
               }
            }
            else
            {
               if ((u32_Controller_Sync_Period > 59970L) && (u32_Controller_Sync_Period < 60030L))  u16_i = 1; //divide freq by 1
               if ((u32_Controller_Sync_Period > 14970L) && (u32_Controller_Sync_Period < 15030L))  u16_i = 4; //divide freq by 4
               if ((u32_Controller_Sync_Period > 11970L) && (u32_Controller_Sync_Period < 12030L))  u16_i = 5; //divide freq by 5
            }

            if (u16_i == 0)
               u16_Num_Of_Mts_In_Sync_Period = (unsigned int)u32_Controller_Sync_Period_in_32Khz;
            else
               u16_Num_Of_Mts_In_Sync_Period = u32_Controller_Sync_Period_in_32Khz / u16_i;  // no fraction is expected

            u32_Pll_Max_Delta = 192L * u32_Controller_Sync_Period_in_32Khz;                  // max result is 3,072,000

            // The way the FPGA mechanism works, max measurement is aprox +/-37500 (+/-125us in DSP clock of 3.333 ns)
            // Limit the sat to 65535 to allow a 16 div by 16 in RT
            if (u32_Pll_Max_Delta > 65535L) u32_Pll_Max_Delta = 65535L;

            if (u16_i > 0)
            {
               *(int*)FPGA_SYNC_SIGNAL_FREQ_DIVIDE_REG_ADD  = u16_i - 1; // Set sync signal freq divider in FPGA
               u16_Pll_Flags |= PLL_ENABLED_MASK;
               u16_Pll_State = PLL_WAIT_FOR_LOCK;
            }
            else
            {
               sync_signal_counter = 0;
               u16_Pll_State = PLL_WAIT_SYNC_SIGNAL; // Go back in the state machine
            }
         }
      break;

      case PLL_WAIT_FOR_LOCK: // Wait PLL locked
         if (BGVAR(u8_Sync_Mode) == 6)
         {
            if (u32_Pll_Mts_Counter_Latch < (128 + (128 * (u32_Controller_Sync_Period_in_32Khz / 32))))
            {
               if (u16_CAN_Sync_Reload_Counter != u16_CAN_Sync_Prev_Reload_Counter)
               {
                  u16_CAN_Sync_Prev_Reload_Counter = u16_CAN_Sync_Reload_Counter;

                  s16_diff_pll_correction = s16_Pll_Correction - s16_Pll_Correction_Prev;
                  s16_Pll_Correction_Prev = s16_Pll_Correction;
                  if (s16_diff_pll_correction < 0) s16_diff_pll_correction = -s16_diff_pll_correction;

                  if ( (u16_Pll_Rt_Flags & PLL_LOCKED_MASK) && (s16_diff_pll_correction <= 6) )
                     u16_Pll_Locked_Counter++;
                  else
                     u16_Pll_Locked_Counter = 0;

                  // Itai - For Buzilla 5349 (EtherCAT initialization fault ("b1")) changed qualifier value from 20 to 40
                  if (u16_Pll_Locked_Counter >= 40)    // wait for 40 consecutive locked indication. Itai - Buzilla 5349 - enlarged from 20 to 40
                     u16_Pll_State = PLL_LOCKED;       // 40 is an arbitrary choice.
               }
            }
         }
         else
         {
            if (u16_Pll_Rt_Flags & PLL_LOCKED_MASK)
            {
               u16_Pll_Locked_Counter++;
            }
            else
            {
               u16_Pll_Locked_Counter = 0;
            }

            if (BGVAR(u8_Sync_Mode) == 8)           // If Gantry mode 2 is adjusted
            {
               // Wait for 1000 consecutive locked indication. Tests during power-up
               // the Drive Y2 have been proven, that even after 600 successful
               // consecutive locked indications, the synchronization could fail
               // and a new attempt to get synchronized had to be done.
               // The test was:
               //   1) Power-up axis Y1 until fully booted, afterwards power-up Y2.
               //   2) Power-up axis Y2 until fully booted, afterwards power-up Y1.
               //
               // Therefore we wait for so many consecutive cycles since we don't want to get
               // errors inside the Gantry status variable "_AX0_u16_Gantry_Status_Variable"
               // straight away after power up because the Gantry RX timeouts (see
               // _AX0_u16_Gantry_Rx_Timeout) are chosen as small as
               // possible and require a very stable synchronization.
               if (u16_Pll_Locked_Counter >= 1000)
               {
                  u16_Pll_State = PLL_LOCKED;
               }
            }
            // Itai - For Buzilla 5349 (EtherCAT initialization fault ("b1")) changed qualifier value from 20 to 40
            else if (u16_Pll_Locked_Counter >= 40) // wait for 40 consecutive locked indication. Itai - Buzilla 5349 - enlarged from 20 to 40
            {                                      // 40 is an arbitrary choice.
               u16_Pll_State = PLL_LOCKED;
            }
         }
      break;

      case PLL_LOCKED: //PLL locked
         // Comment from APH:
         // -----------------
         // Here we check if we have lost the synchronization. The bit real-time function "PLL_SUPPORT" can only detect a time-shift in the DC sync 
         // signal of less than +/- 125[us] and therefore the bit "PLL_LOCKED_MASK" is only set or cleared correctly in case that a DC signal occurred 
         // and if the measured time is within a window of +/- 125[us]. But if the DC signal is not generated at all, then the "PLL_SUPPORT" function
         // does not update the "PLL_LOCKED_MASK" bit. Therefore we have a second check with the "u32_Pll_Mts_Counter". That counter counts in a perfectly
         // synchronized Drive from 0...(u32_Controller_Sync_Period_in_32Khz-1). If that counter should exceed a certain value, then we know that DC
         // sync signals are missing at all. But the formula for the threshold "(128 + (128 * (u32_Controller_Sync_Period_in_32Khz / 32)))" cannot be
         // explained, yet.
         if (((u16_Pll_Rt_Flags & PLL_LOCKED_MASK) == 0) || (u32_Pll_Mts_Counter > (128 + (128 * (u32_Controller_Sync_Period_in_32Khz / 32)))))
         {
            u16_Pll_Locked_Counter = 0;
            u16_Pll_Flags = 0;

            SET_DPRAM_FPGA_MUX_REGISTER(0);

            if ( (BGVAR(u8_Sync_Mode) == 5) && (*(unsigned int*)p_u16_tx_nmt_state != EC_NMTSTATE_OP) )
            {
               // On EtherCAT, before (EC NMT state == OP), don't indicate unlocked PLL.
               u16_Pll_State = PLL_IDLE;
            }
            else
            {
               u16_Pll_State = PLL_UNLOCKED;
            }
         }
      break;

      case PLL_UNLOCKED: // PLL unlocked
      default:
      break;
   }
}


//**********************************************************
// Function Name: SalSyncModeCommand
// Description:
//          This function is define sync signal source
//          0 - disabled
//          1 - input 5
//          2 - input 6
//          3 - "pulse" input Controller I/F
//          4 - "pulse" input Machine I/F
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int SalSyncModeCommand(long long lparam, int drive)
{
   BGVAR(u8_Sync_Mode_User) = (char)lparam;

   SyncModeCommand(lparam, drive);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SyncModeCommand
// Description:
//          This function is define sync signal source
// Set by user command (SYNCSOURCE):
//          0 - disabled
//          1 - input 5
//          2 - input 6
//          3 - "pulse" input Controller I/F
//          4 - "pulse" input Machine I/F
// Set internally:
//          5 - Ehtercat
//          6 - CAN sync message
//          7 - DDHD 1kHz generated by FPGA
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SyncModeCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u8_Sync_Mode) = (char)lparam;

   if (BGVAR(u8_Sync_Mode) == 0) u16_Pll_Flags = 0;

   u16_Pll_State = PLL_IDLE;//reinitialize state machine

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: WritePFB2Flash
// Description:
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void WritePFB2Flash(int drive)
{
   // SST uses Sector (2KWord) Arrangement
   long s32_sector_block_start_addr, s32_pfb_lo,s32_pfb_hi;
   unsigned int u16_temp, u16_pfb_backup_flash_ptr;
   unsigned int* p_u16_pfb_off_flash;
   int s16_checksum;
   // AXIS_OFF;

   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return;  // axis 1 is not supported on Spansion

   if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
   {
      s32_sector_block_start_addr = (long)(FLASH_TYPE_1_PFB_OFF_ADDR);
   }
   else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
   { // Spansion uses Block (32KWord) Arrangement
      return;  // not supported on Spansion 16Mb or 32Mb
   }
   else
   { // SST uses Sector (2KWord) Arrangement
      s32_sector_block_start_addr = (long)(AX0_PFB_OFF_SECTOR + (6 * drive)) * FLASH_SECTOR_SIZE;
   }

   WaitForNext32kHzTaskStart();
   // read pfb
   do {
      u16_temp = Cntr_3125;
      s32_pfb_lo = (long)LVAR(AX0_u32_Pos_Fdbk_User_Lo);
      s32_pfb_hi = (long)LVAR(AX0_s32_Pos_Fdbk_User_Hi);
   } while (u16_temp != Cntr_3125);

   // calc checksum
   s16_checksum = (int)s32_pfb_lo + (int)(s32_pfb_lo >> 16) + (int)s32_pfb_hi + (int)(s32_pfb_hi >> 16);
   p_u16_pfb_off_flash = &Flash[s32_sector_block_start_addr];

   u16_temp = PfbOffCalcAddr(drive);
   if (u16_temp == 0)
      u16_pfb_backup_flash_ptr = 0;
   else
      u16_pfb_backup_flash_ptr = 5 * NumberOfZeroBits(p_u16_pfb_off_flash[u16_temp + PFB_OFF_PTR_START_ADDR - 1]);

   // write data to flash
   FlashWriteWord(s16_checksum, s32_sector_block_start_addr, u16_pfb_backup_flash_ptr);
   if (s16_Flash_Fault_Flag == 1)
   {   //try again
      s16_Flash_Fault_Flag = 0;
      FlashWriteWord(s16_checksum, s32_sector_block_start_addr, u16_pfb_backup_flash_ptr);
   }

   FlashWriteWord((int)s32_pfb_lo, s32_sector_block_start_addr, u16_pfb_backup_flash_ptr + 1);
   if (s16_Flash_Fault_Flag == 1)
   {   //try again
      s16_Flash_Fault_Flag = 0;
      FlashWriteWord((int)s32_pfb_lo, s32_sector_block_start_addr, u16_pfb_backup_flash_ptr + 1);
   }

   FlashWriteWord((int)(s32_pfb_lo >> 16), s32_sector_block_start_addr, u16_pfb_backup_flash_ptr + 2);
   if (s16_Flash_Fault_Flag == 1)
   {   //try again
      s16_Flash_Fault_Flag = 0;
      FlashWriteWord((int)(s32_pfb_lo >> 16), s32_sector_block_start_addr, u16_pfb_backup_flash_ptr + 2);
   }
   FlashWriteWord((int)s32_pfb_hi, s32_sector_block_start_addr, u16_pfb_backup_flash_ptr + 3);
   if (s16_Flash_Fault_Flag == 1)
   {   //try again
      s16_Flash_Fault_Flag = 0;
      FlashWriteWord((int)s32_pfb_hi, s32_sector_block_start_addr, u16_pfb_backup_flash_ptr + 3);
   }
   FlashWriteWord((int)(s32_pfb_hi >> 16), s32_sector_block_start_addr, u16_pfb_backup_flash_ptr + 4);
   if (s16_Flash_Fault_Flag == 1)
   {   //try again
      s16_Flash_Fault_Flag = 0;
      FlashWriteWord((int)(s32_pfb_hi >> 16), s32_sector_block_start_addr, u16_pfb_backup_flash_ptr + 4);
   }

   if (u16_temp == 0)   // update ptr bits - zero one more bit in the PFB_OFF_PTR_START_ADDR
   {
      FlashWriteWord(0x7FFF, s32_sector_block_start_addr, PFB_OFF_PTR_START_ADDR);
      if (s16_Flash_Fault_Flag == 1)
      {   //try again
         s16_Flash_Fault_Flag = 0;
         FlashWriteWord(0x7FFF, s32_sector_block_start_addr, PFB_OFF_PTR_START_ADDR);
      }
   }
   else
   {
      u16_pfb_backup_flash_ptr =  NumberOfZeroBits(p_u16_pfb_off_flash[u16_temp + PFB_OFF_PTR_START_ADDR - 1]);
      if (u16_pfb_backup_flash_ptr == 16)
      {
         FlashWriteWord(0x7FFF, s32_sector_block_start_addr, u16_temp + PFB_OFF_PTR_START_ADDR);
         if (s16_Flash_Fault_Flag == 1)
         {   //try again
            s16_Flash_Fault_Flag = 0;
            FlashWriteWord(0x7FFF, s32_sector_block_start_addr, u16_temp + PFB_OFF_PTR_START_ADDR);
         }
      }
      else
      {
         FlashWriteWord((p_u16_pfb_off_flash[u16_temp + PFB_OFF_PTR_START_ADDR - 1] >> 1), s32_sector_block_start_addr, u16_temp + PFB_OFF_PTR_START_ADDR - 1);
         if (s16_Flash_Fault_Flag == 1)
         {   //try again
            s16_Flash_Fault_Flag = 0;
            FlashWriteWord((p_u16_pfb_off_flash[u16_temp + PFB_OFF_PTR_START_ADDR - 1] >> 1), s32_sector_block_start_addr, u16_temp + PFB_OFF_PTR_START_ADDR - 1);
         }
      }
   }
}


unsigned int PfbOffCalcAddr(int drive)
{
   unsigned int u16_temp = 0;
   unsigned int* p_u16_pfb_off_flash;
   long s32_sector_block_start_addr;

   if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
   {
      s32_sector_block_start_addr = (long)(FLASH_TYPE_1_PFB_OFF_ADDR);
   }
   else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
   { // Spansion uses Block (32KWord) Arrangement
      return (u16_temp);  // not supported on Spansion 16Mb or 32Mb
   }
   else
   { // SST uses Sector (2KWord) Arrangement
      s32_sector_block_start_addr = (long)(AX0_PFB_OFF_SECTOR + (6 * drive)) * FLASH_SECTOR_SIZE;
   }

   s32_sector_block_start_addr+=PFB_OFF_PTR_START_ADDR;

   p_u16_pfb_off_flash = &Flash[s32_sector_block_start_addr];
   while ((p_u16_pfb_off_flash[u16_temp] != 0xFFFF) && (u16_temp < 26)) ++u16_temp;
   return (u16_temp);
}


void PfbBackupFeatureWriteSupport(int drive)
{

   if ( ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) || (BGVAR(s16_PfbBackupRD) == 0) ) return;

   switch (BGVAR(s16_PfbBackupWR))
   {
      case PFB_BACKUP_WR_CALC_ADDR:
         if (Enabled(drive)) return;

         if (PfbOffCalcAddr(drive) == 26)
            BGVAR(s16_PfbBackupWR) = PFB_BACKUP_WR_ERASE_FLASH;
         else
            BGVAR(s16_PfbBackupWR) = PFB_BACKUP_WR_WRITE_FLASH;
      break;

      case PFB_BACKUP_WR_ERASE_FLASH:
         if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
         {
            if (EraseBlock(AX0_FLASH_TYPE_1_PFB_OFF_BLOCK) == FLASH_DONE) BGVAR(s16_PfbBackupWR) = PFB_BACKUP_WR_WRITE_FLASH;
         }
         else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            return;  // not supported on Spansion 16Mb or 32Mb
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            if (EraseSector(AX0_PFB_OFF_SECTOR + (6 * drive)) == FLASH_DONE) BGVAR(s16_PfbBackupWR) = PFB_BACKUP_WR_WRITE_FLASH;
         }
      break;

      case PFB_BACKUP_WR_WRITE_FLASH:
         WritePFB2Flash(drive);
         BGVAR(s16_PfbBackupWR) = PFB_BACKUP_WAIT_FOR_ESTOP_CLR;
      break;

      case PFB_BACKUP_WAIT_FOR_ESTOP_CLR:
         if ((BGVAR(s16_DisableInputs) & EMERGENCY_STOP_MASK) == 0) BGVAR(s16_PfbBackupWR) = PFB_BACKUP_WR_DONE;
      break;

      case PFB_BACKUP_WR_DONE:
      break;
   }
}


void PfbBackupFeatureReadSupport(int drive)
{
   // AXIS_OFF;

   long long s64_temp;
   long s32_sector_block_start_addr, s32_flash_pfb_hi;
   unsigned long u32_flash_pfb_lo, u32_pfb_lo;

   unsigned int u16_temp, u16_pfb_backup_flash_ptr;
   unsigned int* p_u16_pfb_off_flash;
   int s16_checksum, s16_calc_checksum, s16_temp1, s16_temp2;

   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return;  // axis 1 is not supported on Spansion

   if ((BGVAR(s16_PfbBackupRD)==0) && (BGVAR(u16_Pfb_Backup_Mode) == 1)) BGVAR(u64_Sys_Warnings) |= PFBBACKUP_NOT_READ;
   BGVAR(u64_Sys_Warnings) &=~PFBBACKUP_NOT_READ;

   if (BGVAR(s16_PfbBackupRD)==1) return;


   if (BGVAR(u16_Pfb_Backup_Mode) == 0)
   {
      BGVAR(s16_PfbBackupRD) = 1;
      return;
   }

   if ( ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)                                       ||
        ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0)                                   ||
        ((BGVAR(s16_DisableInputs) & (ENDAT_DIS_MASK | HIFACE_DIS_MASK | COMM_FDBK_DIS_MASK | BISSC_DIS_MASK)) != 0) ||
        ((VAR(AX0_s16_Skip_Flags) & ENC_CONFIG_NEEDED_MASK) != 0)                                     )
   {
      BGVAR(s32_PFBBackup_Timer) = Cntr_1mS;
      return;
   }

   if (!PassedTimeMS(500L, BGVAR(s32_PFBBackup_Timer))) return;

   // read data from flash
   if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
   {
      s32_sector_block_start_addr = (long)(FLASH_TYPE_1_PFB_OFF_ADDR);
   }
   else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
   { // Spansion uses Block (32KWord) Arrangement
      return;  // not supported on Spansion 16Mb or 32Mb
   }
   else
   { // SST uses Sector (2KWord) Arrangement
      s32_sector_block_start_addr = (long)(AX0_PFB_OFF_SECTOR + (6 * drive)) * FLASH_SECTOR_SIZE;
   }

   p_u16_pfb_off_flash = &Flash[s32_sector_block_start_addr];

   u16_temp = PfbOffCalcAddr(drive);
   if (u16_temp == 0)
   {
      HandleFault(drive, PFB_OFF_NO_DATA_MASK, 0);
      BGVAR(s16_PfbBackupRD) = 1;
      return;
   }
   else
      u16_pfb_backup_flash_ptr = 5 * (NumberOfZeroBits(p_u16_pfb_off_flash[u16_temp + PFB_OFF_PTR_START_ADDR - 1]) - 1);

   s16_checksum =  p_u16_pfb_off_flash[u16_pfb_backup_flash_ptr];
   u32_flash_pfb_lo =  (long)(p_u16_pfb_off_flash[u16_pfb_backup_flash_ptr + 1]) | ((long)(p_u16_pfb_off_flash[u16_pfb_backup_flash_ptr + 2]) << 16);
   s32_flash_pfb_hi =  (long)(p_u16_pfb_off_flash[u16_pfb_backup_flash_ptr + 3]) | ((long)(p_u16_pfb_off_flash[u16_pfb_backup_flash_ptr + 4]) << 16);

   BGVAR(s64_Pfb_Backup) = ((long long)s32_flash_pfb_hi << 32) | (long long)u32_flash_pfb_lo;
   BGVAR(s64_Pfb_RiseUp) = ((long long)AX0_s32_Pos_Fdbk_User_Hi << 32) | (long long)AX0_u32_Pos_Fdbk_User_Lo;

   s16_calc_checksum = (int)u32_flash_pfb_lo + (int)(u32_flash_pfb_lo >> 16) + (int)s32_flash_pfb_hi + (int)(s32_flash_pfb_hi >> 16);

   if (s16_calc_checksum != s16_checksum)
   {
      HandleFault(drive, PFB_OFF_CHKSUM_MASK, 0);
      BGVAR(s16_PfbBackupRD)=1;
      return;
   }

   // compare pfb lo
   WaitForNext32kHzTaskStart();
   u32_pfb_lo = (unsigned long)LVAR(AX0_u32_Pos_Fdbk_User_Lo);     // read pfb

   // verify delta does not exceed 5 degrees = 5/360*65536 =910
   s16_temp1 = (int)(u32_pfb_lo>>16);
   s16_temp2 = (int)(u32_flash_pfb_lo>>16);

   s16_temp1 = s16_temp1-s16_temp2;
   if (s16_temp1<0) s16_temp1=-s16_temp1;
   if (s16_temp1 > 910)
   {
      HandleFault(drive, PFB_OFF_DATA_MISMATCH_MASK, 0);
      BGVAR(s16_PfbBackupRD)=1;
      return;
   }

   // update pfb high with value read from flash. correct according to pfb lo
   if ((u32_pfb_lo < 0x40000000) && (u32_flash_pfb_lo > 0x40000000)) ++s32_flash_pfb_hi;
   else if ((u32_pfb_lo > 0x40000000) && (u32_flash_pfb_lo < 0x40000000)) --s32_flash_pfb_hi;

   s64_temp = (((long long)s32_flash_pfb_hi << 32) | (long long)u32_pfb_lo) -  *(long long*)&LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) - *(long long*)&LVAR(AX0_u32_Home_Offset_Lo);

   if ((VAR(AX0_s16_Skip_Flags) & INV_DIRECTION_MASK) != 0) s64_temp=-s64_temp;
   WaitForNext32kHzTaskStart();
   *(long long*)&LVAR(AX0_u32_Pos_Fdbk_Lo) = s64_temp;

   BGVAR(s16_PfbBackupRD)=1;
}


// Function Name: SalWriteStopModeCommand
// Description:
//    Dismode is called stop mode in schneider drive.
//    This function writes the param P1-32 and converts the value to the dismode option.
//    also modify disspeed and distime
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteStopModeCommand(long long param,int drive)
{
   int retVal;

   if ((param != 0LL) && (param != 0x10LL) && (param != 0x20LL))
       return (VALUE_OUT_OF_RANGE);

   retVal = SetSchneiderDismode(param, drive);
   if (retVal == SAL_SUCCESS)
      BGVAR(s16_P1_32_Stop_Mode) = param;

   return retVal;
}


// Function Name: SetSchneiderDismode
// Description:
//    This function converts the shcnider dismode value to cdhd dismode and updates necessary params:
//          schneiderDismode = 0x00: active disable (AD)
//          schneiderDismode = 0x10: no AD and no dynamic brake (DB)
//          schneiderDismode = 0x20: AD until vel<P1-38 and then disable.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SetSchneiderDismode(int schneiderDismode, int drive)
{
   int retVal;
   long long mode;

   switch (schneiderDismode)
   {
       case 0x00:        // scheinder dynamic brake (equals to cdhd active disable)
       // use disspeed (P1-43)     BGVAR(u16_DisTime) = 0;
            retVal = SalDisSpeedCommand(2 * 53687LL, drive);   // 2 * 6 rpm, recording shows that servosense velocity in standstill reaches 12 rpm :)
            if (retVal != SAL_SUCCESS)
                 return retVal;

            mode = 3;
            break;
       case 0x10:        // coast to stop (no AC and no DB)
            mode = 0;
            break;
       case 0x20:        // scheinder dynamic brake until v<P1-38_ZSPD and then coast to stop
       // use disspeed (P1-43)    BGVAR(u16_DisTime) = 0;
            retVal = SalDisSpeedCommand(BGVAR(s32_P1_38_Zero_Speed_Window_Out_Loop), drive);
            if (retVal != SAL_SUCCESS)
                 return retVal;

            mode = 3;
            break;
       default:
            return VALUE_OUT_OF_RANGE;
   }

   return SalDisableModeCommand(mode, drive);
}




//**********************************************************
// Function Name: SalReadGearInNumeratorCommand
// Description: Return the value of the gear ratio numerator index in the numerators array.
//              The array consists of 4 elements as follows:
//                              P param | array index
//                              --------|------------
//                              P1-44   | 0
//                              P2-60   | 1
//                              P2-61   | 2
//                              P2-62   | 3
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalReadGearInNumeratorCommand(long long *data,int drive)
{
   unsigned int u16_path_index = (unsigned int)s64_Execution_Parameter[0];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((u16_path_index >= 4 )) return (VALUE_OUT_OF_RANGE);

   *data = (long long)BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[u16_path_index];
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalWriteGearInNumeratorCommand
// Description: Return the value of the gear ratio numerator index in the numerators array.
//              The array consists of 4 elements as follows:
//                              P param | array index
//                              --------|------------
//                              P1-44   | 0
//                              P2-60   | 1
//                              P2-61   | 2
//                              P2-62   | 3
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteGearInNumeratorCommand(int drive)
{
   unsigned int u16_index = (unsigned int)s64_Execution_Parameter[0];
   long s32_value = (long)s64_Execution_Parameter[1];

   // 22.5.14 - as discussed with Avishay & Joachim, due to PUU effect on internal values of parameters (e.g. SW limit)
   // it was decided to prevent writing P1-44 and P1-45 via CANOpen channel.
   // 10.8.14 - Udi: Allow set gearing when Downloading a parameter file.
   //           This is indicated by "Config_Lock == Write"
   if ((u16_index == 0) &&
       (((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_CANOPEN)      ||
        ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)       ||
        ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)  ||
        ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)        )  &&
       (u16_Config_Lock != CONFIG_LOCK_WRITE)                                           )
       return (INVALID_OPMODE);

   if ((u16_index >= 4 )) return (VALUE_OUT_OF_RANGE);
   if (s32_value > 536870911) return (VALUE_TOO_HIGH);
   if (s32_value < 1) return (VALUE_TOO_LOW) ;

   // spec shows some limitation on P1-44 but not on P2-60 to P2-62
   if (u16_index == 0)    // index == 0 changing P1-44
   {
      // cannot modify value if enabled and drive not in Pt mode (gearing).
      // fix IPR 1469: In PT mode ,when enable axis via P2-30.P1-44 can not be modify.
      // check according to lxm opmode instead of internal opmode
      if (Enabled(drive) && (BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PT)) return (DRIVE_ACTIVE);
   }

   BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[u16_index] = s32_value;

   if (u16_index == 0)
   {// update the PUU only when P1-44 is changing
      // update the new gear ratio and PUU unit conversion fix shift for Schneider
      SchneiderCalcPuuUnits(drive);
   }

   // nitsan: set flag to update gear value via schnieder dig input selectors (gnum).
   BGVAR(s16_Gear_Changed_Flag) = 1;
   return (SAL_SUCCESS);
}



//**********************************************************
// Function Name: SalWriteExtPulseInputTypeCommand
// Description:
//          This function is called in response to the P-Parameter P1-00 write command.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteExtPulseInputTypeCommand(long long param,int drive)
{
   unsigned int u16_temp_value = (unsigned int)param;
   int s16_gearMode = 0;

   // AXIS_OFF;

   if (Enabled(drive)) return DRIVE_ACTIVE;

   // check range for each byte
   if ((u16_temp_value & 0x000f) > 0x0002) return VALUE_OUT_OF_RANGE;
   if ((u16_temp_value & 0x00f0) > 0x0030) return VALUE_OUT_OF_RANGE;
   if ((u16_temp_value & 0x0f00) > 0x0100) return VALUE_OUT_OF_RANGE;
   if ((u16_temp_value & 0xf000) > 0x1000) return VALUE_OUT_OF_RANGE;

   BGVAR(u16_P1_00_Ext_Pulse_Input_Type) = u16_temp_value;
   if (u16_Product == SHNDR_HW)
   {
       switch (u16_temp_value & 0xf)
       {
            case 0:
                 s16_gearMode = 0;   // AB phase pulse (4x) (Quadrature Input)
                 break;
            case 1:
                 s16_gearMode = 2;   // up/down (CW or CCW in schnieder terminology).
                 break;
            case 2:
                 s16_gearMode = 1;   // pulse + direction
                 break;
       }

       //check if PTCMS input is not in use set the HIGH/LOW channel P1-00 D field value else dont change
       if (IsInFunctionalityConfigured(PTCMS_INP,drive) == 0)
       {
            BGVAR(u16_Schneider_Input_Pulse_Speed) = ((u16_temp_value & 0xf000)>>12);
       }
   }
   else
   {
       switch (u16_temp_value & 0xf)
       {
            case 0:
                 s16_gearMode = (u16_temp_value & 0xf000) ? 0 : 3;   // AB phase pulse (4x) (Quadrature Input)
                 break;
            case 1:
                 s16_gearMode = (u16_temp_value & 0xf000) ? 2 : 2;   // up/down (CW or CCW in schnieder terminology).
                 break;
            case 2:
                 s16_gearMode = (u16_temp_value & 0xf000) ? 1 : 4;   // pulse + direction
                 break;
       }
   }
   SalGearModeCommand(s16_gearMode, drive);

   UpdatePulseDirectionFiltersFromPTTParameter(drive);

   // update gear with polarity
   // send absolute value, since poalrity is multiplied inside SalGearInCommand()
   SalGearInCommand(labs(LVAR(AX0_s32_Gearin_Design)), drive);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: UpdatePulseDirectionFiltersFromPTTParameter
// Description:
//          This function is called in response to the P-Parameter P1-00 write command
//          and handle the pulse and direction filter
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void UpdatePulseDirectionFiltersFromPTTParameter(int drive)
{
   // BUG# 3724 fix
   unsigned int u16_filter_multiplier = 0;
   unsigned int u16_input_pulse_filter_index = ((BGVAR(u16_P1_00_Ext_Pulse_Input_Type) & 0x00f0) >> 4);

   // filter multiplier calculation:
   // multiplier = (( 1 / (10 * freq)) / 16.6666666 10^-9 ) // to give 20% more then pulse freq
   //
   // the pulse & direction filter base time is 16.6666 nSec (FPGA filter)


   if (BGVAR(u16_Schneider_Input_Pulse_Speed) == 1)
   {// high speed pulse input
      switch(u16_input_pulse_filter_index)
      {
         case 0: // 8 Mpps filter -> set Alarm to 4 Mpps + 10%
            u16_filter_multiplier = 3;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 4400L; // 4.4 MPPS
         break;

         case 1: // 4 Mpps filter -> set Alarm to 2 Mpps + 10%
            u16_filter_multiplier = 7;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 2200L; // 2.2 MPPS
         break;

         case 2: // 2 Mpps filter -> set Alarm to 1 Mpps + 10%
            u16_filter_multiplier = 15;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 1100L; // 1.1 MPPS
         break;

         case 3: // 1 Mpps filter -> set Alarm to 500 Kpps + 10%
            u16_filter_multiplier = 30;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 550L; // 550 Kpps
         break;

         default:
            u16_filter_multiplier = 1;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 4400L; // 4.4 MPPS
         break;
      }
   }
   else if (BGVAR(u16_Schneider_Input_Pulse_Speed) == 0)
   {// low speed pulse input
      switch(u16_input_pulse_filter_index)
      {
         case 0: // 1 Mpps filter -> set Alarm to 500 Kpps + 10%
            u16_filter_multiplier = 30;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 550L; // 550 Kpps
         break;

         case 1: // 400 Kpps filter -> set Alarm to 200 Kpps + 10%
            u16_filter_multiplier = 75;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 220L; // 220 KPPS
         break;

         case 2: // 200 Kpps filter -> set Alarm to 100 Kpps + 10%
            u16_filter_multiplier = 150;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 110L; // 110 KPPS
         break;

         case 3: // 100 Kpps filter -> set Alarm to 50 Kpps + 10%
            u16_filter_multiplier = 300;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 55L; // 55 Kpps
         break;

         default:
            u16_filter_multiplier = 1;
            BGVAR(u32_Max_Pulse_Train_Frequancy) = 550L; // 550 Kpps
         break;
      }
   }

   if (BGVAR(u16_Gear_In_Mode))
   {// if P8-30 (MULTIPLIER) is on multiply the max freq by 16
      BGVAR(u32_Max_Pulse_Train_Frequancy) *=16;
   }

   if ((BGVAR(u16_P1_00_Ext_Pulse_Input_Type) & 0xf) == 0)
   {// if P1-00 nibble "A" == 0 -> A quad B so multiply the max freq by 4
      BGVAR(u32_Max_Pulse_Train_Frequancy) *=4;
   }

   // if calculated frequency exceeds the internal representation:
   // the HW counter is 16 bit, means +/- 32767 bits. taking 80% marging gives +/- 26214
   // since this counter is sampled at 8 Khz rate the max value is +/- 26214 * 8 = +/- 209712 bits per milisecond.
   if (BGVAR(u32_Max_Pulse_Train_Frequancy) > MAX_ALLOWED_EXTERNAL_FREQUENCY)
   {
      BGVAR(u32_Max_Pulse_Train_Frequancy) = MAX_ALLOWED_EXTERNAL_FREQUENCY;
   }

   // update the filter with the new multiplier
   WritePtPulseBounceFilterCommand(u16_filter_multiplier,drive);
   WritePtDirectionBounceFilterCommand(u16_filter_multiplier,drive);
}

//************************************************************
// Function Name: IcmdSlopeEn
// Description:
//          This function controls the use of the Icmd Slope
//
// Author: Gil
// Algorithm:
// Revisions:
//************************************************************
void IcmdSlopeEn(drive)
{
   // RT will use the following var as permission to use torque slope.
   // 1) only when the drive is disabled
   // 2) The user wants to enable the feature
   // 3) the used opmode is toque mode.
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (((VAR(AX0_s16_Opmode) == 2) || (VAR(AX0_s16_Opmode) == 3))&& BGVAR(u16_Icmd_Slope_Use))
   {
       VAR(AX0_u16_Icmd_Slope_Use) = 1;
   }
   else
   {
       VAR(AX0_u16_Icmd_Slope_Use) = 0;
   }
}

//**********************************************************
// Function Name: SalSetTorqueSlopeCommand
// Description:
//          Set the torque slope according to canopen units.
//          Code segment was taken from FalTorqueSlopeCommandBg() and formed into function
//          in order to allow usage in lexium drive profile
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalSetTorqueSlopeCommand(long long param,int drive)
{
   long long s64_tmp;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (param < 1) {return VALUE_TOO_LOW;}
   if (param > 30000000LL) {return VALUE_TOO_HIGH;}

   BGVAR(u32_Icmd_Slp) = (unsigned long)param;

   s64_tmp = MultS64ByFixS64ToS64((long long)BGVAR(u32_Icmd_Slp),
   BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_internal_fix,
   BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_internal_shr);

   //Saturation of the Torque slope according to 32767
   //The Icmd and Icmd_prev have values in the range of -26414:26414
   //therefore ,in case of Delta 26214- (-26214)= 52428
   //a saturation of 32767 will be in use.
   //pay attention that in order to have an integer and remainder representation.
   //the integer part will be positioned in the upper 16 bits
   //0x7FFF0000; //Internal limit of the current slope
   LVAR(AX0_s32_Icmd_Slp)= ( (s64_tmp > 0x7FFF0000LL)?0x7FFF0000:(long)s64_tmp );

   return SAL_SUCCESS;
}


//************************************************************
// Function Name: RecalcMaxRegenOnTime
// Description:
//          This function calculates the maximum allowed regeneration-resistor
//          on-time for a 5[s] time period based on the resistance and the power
//          settings.
//
// Author: APH
// Algorithm:
// Revisions:
//************************************************************
void RecalcMaxRegenOnTime(int drive)
{
   // The maximum on-time is averaged for a 5[s] time period. The formula for
   // the on-time is as follows:
   // P_regen = V_trip^2 / REGENRES          // Power when regen-resistor is active (assuming bus-voltage = V_trip).
   // P_regen_1ms = P_regen * 1 / 5000       // Averaged power when regen-resistor is on for 1[ms] during a 5[s] period.
   // t_on_max = REGENPOW / P_regen_1ms      // Time in [ms] how long the regen resistor can be switched on during a 5[s] cycle.
   double d_temp;
   float f_temp_result;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If a -1 (or 0 which is forbidden) is in the variable "s16_Regen_Resistor_Resistance" or "s16_Regen_Resistor_Power",
   // which means --> NO REGEN RESISTOR AVAILABLE
   if ((BGVAR(s16_Regen_Resistor_Resistance) <= 0) || (BGVAR(s16_Regen_Resistor_Power) <= 0))
   {
      // De-activate the regen overload feature
      u16_Max_Regen_On_Time = 0;
      s16_Max_Regen_Power = -1;
      return;
   }

   // Calculate P_regen
   f_temp_result = (double)u16_Regen_On_Value * (double)u16_Regen_On_Value / (double)BGVAR(s16_Regen_Resistor_Resistance);

   // Calculate P_regen_1ms
   f_temp_result /= 5000.0;

   // Calculate maximum on-time
   f_temp_result = (double)BGVAR(s16_Regen_Resistor_Power) / f_temp_result;

   // If the temporary result is larger than 500[ms] OR
   // if the result is negative (which is never suppose to happen)
   if ((f_temp_result > 500) || (f_temp_result <= 0))
   {
      u16_Max_Regen_On_Time = 500; // Set to 500[ms] which corresponds to a duty-cycle of 10%
      d_temp = (((double)u16_Regen_On_Value * (double)u16_Regen_On_Value) / (double)(10.0 * (double)BGVAR(s16_Regen_Resistor_Resistance)));
      if (d_temp > 32767.0)
         d_temp = 32767.0;
      s16_Max_Regen_Power = (int)d_temp;
   }
   else // result is OK, therefore apply the value
   {
      u16_Max_Regen_On_Time = (unsigned int)f_temp_result;
      s16_Max_Regen_Power   = BGVAR(s16_Regen_Resistor_Power);
   }

   // When 0<f_temp_result<1, u16_Max_Regen_On_Time=0.
   // In this case set it to 1, to keep the overload protection active.
   if (u16_Max_Regen_On_Time == 0)
   {
      u16_Max_Regen_On_Time = 1;
   }
}

//************************************************************
// Function Name: SalSetRegenResistorResistance
// Description:
//          Set resistance of the regeneration resistor.
//
// Author: APH
// Algorithm:
// Revisions:
//************************************************************
int SalSetRegenResistorResistance(long long param,int drive)
{
   if (param == 0)
   {
      return (VALUE_OUT_OF_RANGE);
   }

   BGVAR(s16_Regen_Resistor_Resistance) = (unsigned int)param;
   RecalcMaxRegenOnTime(drive);
   return (SAL_SUCCESS);
}
//************************************************************
// Function Name: SalSetRegenResistorPower
// Description:
//          Set power of the regeneration resistor.
//
// Author: APH
// Algorithm:
// Revisions:
//************************************************************
int SalSetRegenResistorPower(long long param,int drive)
{
   if (param == 0)
   {
      return (VALUE_OUT_OF_RANGE);
   }

   BGVAR(s16_Regen_Resistor_Power) = (unsigned int)param;
   RecalcMaxRegenOnTime(drive);
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: RestoreAccDecInternalVal
// Description:
//    This function restores the internal variables regarding acc,dec
//    with their original values (during operation,these values are
//    loaded by other values (like in position hunting)
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
void RestoreAccDecInternalVal(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   // This will allow to run the phantom hunting according to the ACC and DEC settings
   LVAR(AX0_s32_Ptp_Acc_Rounded) = LVAR(AX0_s32_Ptp_Acc_Rounded_Bg_Shadow) = (long)((BGVAR(u64_AccRate) + (unsigned long long)0x080000000) >> 32);
   LVAR(AX0_s32_Ptp_Dec_Rounded) = LVAR(AX0_s32_Ptp_Dec_Rounded_Bg_Shadow) = (long)((BGVAR(u64_DecRate) + (unsigned long long)0x080000000) >> 32);
}


//**********************************************************
// Function Name: SalWriteControlModeCommand
// Description:
//    This function write p param P1-01 (Control Mode and Output Direction) CTL.
//    each nibble has its own min/max, so check it here
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteControlModeCommand(long long param,int drive)
{
   unsigned int val = (unsigned int)param;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // nibble A,B
   if (u16_Flash_Type == 1)   // 2 * 64Mb flash
   {//the drive is LXM28E: 0x20 - SERCOS, 0x30 - EthernetIP, 0x40 - EtherCAT
      if ( (((val & 0x00ff) > 0x000A) && ((val & 0x00ff) < 0x0020))    ||      // temporary, till fieldbus is stabilized, allow I/O modes
           (((val & 0x00ff) > 0x0020) && ((val & 0x00ff) < 0x0030))    ||
           (((val & 0x00ff) > 0x0030) && ((val & 0x00ff) < 0x0040))    ||
           (((val & 0x00ff) > 0x0040)                             )       )
      {
         return INVALID_OPMODE;
      }
   }
   else if (IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK))
   {//the drive is (LXM28A CANOpen version)
      if ((val & 0x00ff) > 0x000B) return INVALID_OPMODE;
   }
   else
   {//the drive is (LXM28A I/O version) dont allow set the CANOpen opmode.
      if ((val & 0x00ff) > 0x000A) return INVALID_OPMODE;
   }

   // nibble C
   if (((val >> 8) & 0x000f) > 0x0001) return VALUE_OUT_OF_RANGE;

   // nibble D
   if (((val >> 12) & 0x000f) > 0x0001) return VALUE_OUT_OF_RANGE;

   BGVAR(u16_P1_01_CTL) = param;

   //For Verification team request - this will skip the need for power cycle (password protected)
   if ((1 == BGVAR(u16_Enable_OpmodeSE_Skip_PowerCycle)) && ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)))
   {
      BGVAR(u16_P1_01_CTL_Current) = BGVAR(u16_P1_01_CTL);
      SetSchneiderOpmode(drive , BGVAR(u16_P1_01_CTL_Current));
      InitSchneiderDigitalIODefaults(0);
   }

   return SAL_SUCCESS;
}
