/*
 * LXM_Profile.c
 * This file handles the Schneider CANopen Profile: Drive Profile Lexium
 * In this profile there are 2 PDOs:
 * RPDO1:
 * dmCtrl (2 bytes), refA16 (2 bytes), refB32 (4 bytes)
 *
 * TPDO1:
 * driveStat (2 bytes), mfStat(2 bytes), motionSt (2 bytes), driveInput (2 bytes)
 *
 * full description in document: Bai_Yang_DES04_Drive_Profile_LXM.doc
*/



#include <stdio.h>
#include <limits.h>
#include "cal_conf.h"
#include <co_usr.h>
#include <co_acces.h>
#include <co_pdo.h>
#include <co_sdo.h>
#include <co_drv.h>
#include <co_flag.h>

#include "objects.h"
#include "402fsa.h"

#include "402fsa.def"
#include "Homing.def"
#include "ExFbVar.def"
#include "Err_Hndl.def"
#include "PtpGenerator.def"
#include "Position.def"
#include "Init.def"
#include "Drive.var"
#include "Extrn_Asm.var"
#include "Units.var"
#include "Homing.var"
#include "Lxm_profile.var"
#include "ExFbVar.var"
#include "Foldback.var"
#include "PtpGenerator.var"
#include "ModCntrl.var"

#include "Lxm_profile.pro"
#include "Prototypes.pro"


/*
debug

int s16_debug_change_opmode[]={0,0,0,0,0,0,0,0};
int s16_debug_moveabs[]={0,0,0,0};
int s16_debug_lxm_home[6]={0,0,0,0,0,0};
int s16_debug_change_opmode[10]=0;
long long s64_vlim_can_units;

int s16_debug1=0;
int s16_debug2=0;
int s16_debug3=0;
int s16_debug4=0;
int s16_debug5=0;

int s16_nitsan_lxm_1=0;
int s16_nitsan_lxm_2=0;
int s16_nitsan_qs1=0;
int s16_nitsan_mt_check=0;
*/

//int s16_on_the_fly_position_opmode = 0;

// RT handler to Lexium Profile Drive RPDO.
//#pragma CODE_SECTION(Lxm_Rpdo_Handler, "ramcan");
//void Lxm_Rpdo_Handler()
//{
/*   int drive = 0;
   unsigned int u16_temp_time;
   // AXIS_OFF;


   // check drive profile lexium only if in canopen mode and profile is enabled.
   if ((BGVAR(s16_P3_10_PlcEn) == 1) && ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN))
   {
      do
      {
         // capture position (required for position opmode)
         u16_temp_time = Cntr_3125; // the same RT Interrupt
         s64_Lxm_Pfb_Capture = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
      } while (u16_temp_time != Cntr_3125);

      u16_Lxm_DmCtrl.all = manu_spec_u16_P3_12_driveModeCtrl;
      s16_Lxm_RefA16 = manu_spec_s16_P3_13_refA16;
      s32_Lxm_RefB32 = manu_spec_s32_P3_14_refB32;
      s16_Lxm_New_Data_Arrived = 1; // signal to background that new pdos arrived
   }

   // to support SDO values.
   BGVAR(u16_P3_12_Lxm_Ctrl_Can_Obj) = manu_spec_u16_P3_12_driveModeCtrl;
   BGVAR(s16_P3_13_Lxm_RefA_Can_Obj) = manu_spec_s16_P3_13_refA16;
   BGVAR(s32_P3_14_Lxm_RefB_Can_Obj) = manu_spec_s32_P3_14_refB32;*/
//}

// Background handler to Lexium Profile Drive.
// run in background since it manipulates control word which is handled in function UpdateStateMachine()
// which is run in background (402fsa.c)
//void Lxm_Profile_Handler(int drive)
//{
/*   static Lxm_DmCtrl u16_lxm_dmCtrl_shadow = {0};
   static int s16_lxm_refA16_shadow = 0;
   static long s32_lxm_refB32_shadow = 0L;

   // check drive profile lexium only if in canopen mode and profile is enabled.
   if ((BGVAR(s16_P3_10_PlcEn) == 0) || ((BGVAR(u16_P1_01_CTL_Current) & 0xff) != SE_OPMODE_CANOPEN))
   {
       return;
   }

   // make sure data is not changed in case that a new RPDO arrives while copying
   // bugs 3277, 3278
   while (s16_Lxm_New_Data_Arrived == 1)
   {
      s16_Lxm_New_Data_Arrived = 0;
      u16_lxm_dmCtrl_shadow.all = u16_Lxm_DmCtrl.all;
      s16_lxm_refA16_shadow = s16_Lxm_RefA16;
      s32_lxm_refB32_shadow = s32_Lxm_RefB32;
   }
   
   // Check if pcmd is beyond CANopen 32bit position limits
   // (IPR 1395: Drive Profile LXM - RF Bit not reset if LXM28 overtravel the maximum position range)
   if (!Lxm_Is_Can_Position_Referenced(drive))
   {
      // movement is beyond canopen 32bit position limits, so homing is lost
      // dont use HomeCommand() with argument zero for cancelling homing because it will also stop ptp movement.
      BGVAR(u8_Homing_State) = HOMING_IDLE;
      BGVAR(u8_Homing_Not_Started) = 0;
   }


   // handle driveCtrl bits
   Lxm_Handle_Drive_Ctrl(drive, u16_lxm_dmCtrl_shadow);

   // handle opmode
   Lxm_Handle_Opmode(drive, u16_lxm_dmCtrl_shadow, s16_lxm_refA16_shadow, s32_lxm_refB32_shadow);

   u16_Lxm_DmCtrl_Prev.all = u16_lxm_dmCtrl_shadow.all;*/
//}


//void Lxm_Handle_Drive_Ctrl(int drive, Lxm_DmCtrl u16_lxm_dmCtrl_shadow)
//{
/*   int s16_need_clamp_position = 0;
   unsigned int statusWord = p402_statusword;
   unsigned int u16_CtrlBits = 0;    // 1 if rising edge detected. 0: disable bit
                                     //                            1: enable bit
                                     //                            2: quick stop bit
                                     //                            3: fault reset bit

   // AXIS_OFF;

   drive += 0;



     // disable has priority over enable
   // or special case in which when setting value from !=0 to 0, disable drive
   if (((u16_lxm_dmCtrl_shadow.bit.DS) && (u16_lxm_dmCtrl_shadow.bit.DS != u16_Lxm_DmCtrl_Prev.bit.DS)) ||
       ((u16_lxm_dmCtrl_shadow.all == 0) && (u16_Lxm_DmCtrl_Prev.all != 0)))
   {
       u16_CtrlBits |= 1;  // disable
   }

   if (((u16_lxm_dmCtrl_shadow.bit.EN) && (u16_lxm_dmCtrl_shadow.bit.EN != u16_Lxm_DmCtrl_Prev.bit.EN)))
   {
       u16_CtrlBits |= 2;  // enable
   }

   if ((u16_lxm_dmCtrl_shadow.bit.QS) && (u16_lxm_dmCtrl_shadow.bit.QS != u16_Lxm_DmCtrl_Prev.bit.QS))
   {
       u16_CtrlBits |= 4;  // quick stop
   }

   if ((u16_lxm_dmCtrl_shadow.bit.FR) && (u16_lxm_dmCtrl_shadow.bit.FR != u16_Lxm_DmCtrl_Prev.bit.FR))
   {
       u16_CtrlBits |= 8;  // fault reset
   }
   else
   {
      // reset fault-reset bit in case it was set on prev cycle
      if (p402_controlword & FAULT_RESET)
      {
         p402_controlword &= ~FAULT_RESET;
      }
   }


   // IPR 1154: LXM Profile: Wrong operation mode after the execution of MC_TorqueControl and power off / on ... leads to "no torque"
   // when drive is disabled, make sure it is in position mode. this will force it to keep position when enabling it.
   // same for when entering quick stop and hold process has finished
   // same for when drive is in operation_enabled state and halt is requested (and hold is not in progress)
   if (VAR(AX0_s16_Opmode) != 8)    // if not in position mode
   {
      if (!Enabled(drive))
      {
         s16_need_clamp_position = 1;
      }
      else if ((statusWord & STATUSWORD_STATE) == OPERATION_ENABLED)
      {
         // if halt bit is raised and hold is not in process
         if ((s16_HaltBit_Cleared == 0) && (BGVAR(u16_Hold_Stop_In_Process) == 0))
         {
            s16_need_clamp_position = 1;
         }
      }
      else if ((statusWord & STATUSWORD_STATE) == QUICK_STOP_ACTIVE)
      {
         if (BGVAR(u16_Hold_Stop_In_Process) == 0) // wait for hold to finish
         {
            s16_need_clamp_position = 1;
         }
      }

      if (s16_need_clamp_position)
      {
         // change the opmode
         BGVAR(s16_CAN_Opmode_Temp) = PROFILE_POSITION_MODE;
         BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
         CheckCanOpmode(drive);
      }
   }
   

   switch (statusWord & STATUSWORD_STATE)
   {
       case SWITCH_ON_DISABLED:
            BGVAR(u16_Lexium_Jog_Move_Activate) = 0;
            s16_Active_Opmode = 0;
            s16_Lxm_Reference_Val = 0;

            if (u16_CtrlBits & 1)
            {
                // if disbale is on, and automatic transition to ready to switch on state
                 if (BGVAR(u16_P3_32_SwOnDis_To_RdyToSwOn) == 0)
                 {
                      // transition 2
                      s16_Lxm_Enable_In_Progress = 0;
                      p402_controlword &= ~COMMAND_MASK;     // reset enable bit
                      p402_controlword |= SHUTDOWN;          // per =s= request, if disable, set canopen state machine to ready-to-switch-on state (transition #8)
                 }
            }
            else if (u16_CtrlBits & 2)
            {
                 // if enable is on, do transition 2
                 // transition 2,6,8 - shutdown
                 s16_Lxm_Enable_In_Progress = 1;
                 p402_controlword &= ~COMMAND_MASK;     // reset enable bit
                 p402_controlword |= SHUTDOWN;          // per =s= request, if disable, set canopen state machine to ready-to-switch-on state (transition #8)
            }
            else if (u16_CtrlBits & 8)
            {
                 // if fault reset is on, do fault reset
                 p402_controlword |= FAULT_RESET;
            }
            // if automatic transition from SWITCH_ON_DISABLED to READY_TO_SWITCH_ON
            // need to change control word to prevent toggeling between the 2 states
            else if (BGVAR(u16_P3_32_SwOnDis_To_RdyToSwOn) == 0)
            {
                 // transition 2
                 p402_controlword &= ~COMMAND_MASK;     // reset enable bit
                 p402_controlword |= SHUTDOWN;          // per =s= request, if disable, set canopen state machine to ready-to-switch-on state (transition #8)
            }

            break;

       case READY_TO_SWITCH_ON:
            s16_Active_Opmode = 0;
            s16_Lxm_Reference_Val = 0;
            BGVAR(u16_Lexium_Jog_Move_Activate) = 0;

            if (u16_CtrlBits & 1)
            {
               s16_Lxm_Enable_In_Progress = 0;
            }
            // if disbale is off and enable is on, do transition 3
            else if (((u16_CtrlBits & 2)) || s16_Lxm_Enable_In_Progress)
            {
                 // transition 3 - switch on
                 s16_Lxm_Enable_In_Progress = 1;
                 p402_controlword &= ~0x18;
                 p402_controlword |= SWITCH_ON;
            }
            else if (u16_CtrlBits & 8)
            {
                 // if fault reset is on, do fault reset
                 p402_controlword |= FAULT_RESET;
            }
            // if control word holds DISABLE_VOLTAGE command and automatic transition is on
            // then set command to SHUTDOWN to prevent toggling between states: READY_TO_SWITCH_ON and SWITCH_ON_DISABLED
            else if (((p402_controlword & 0x82) == DISABLE_VOLTAGE) && (BGVAR(u16_P3_32_SwOnDis_To_RdyToSwOn) == 0))
            {
                 // transition 2
                 s16_Lxm_Enable_In_Progress = 0;
                 p402_controlword &= ~COMMAND_MASK;     // reset enable bit
                 p402_controlword |= SHUTDOWN;          // per =s= request, if disable, set canopen state machine to ready-to-switch-on state (transition #8)
            }

            break;

       case SWITCHED_ON:
            s16_Active_Opmode = 0;
            s16_Lxm_Reference_Val = 0;
            BGVAR(u16_Lexium_Jog_Move_Activate) = 0;

            // if disable, do disable
            if (u16_CtrlBits & 1)
            {
                 // transition 6
                 p402_controlword &= ~COMMAND_MASK;     // reset enable bit
                 p402_controlword |= SHUTDOWN;          // per =s= request, if disable, set canopen state machine to ready-to-switch-on state (transition #8)
                 s16_Lxm_Enable_In_Progress = 0;        // reset flag just in case we are during enable.
            }
            // if disbale is off and enable is on, do transition 3
            else if ((u16_CtrlBits & 2) || s16_Lxm_Enable_In_Progress)
            {
                 // raise halt bit to prevent start of motion when enable
                 p402_controlword |= HALT_MOVE;
                 s16_HaltBit_Cleared = 1;

                 p402_controlword &= ~0x10;
                 p402_controlword |= ENABLE_OPERATION;
                 s16_Lxm_Enable_In_Progress = 0;

                 // reset opmode process var when entering operation enabled
                 s16_Opmode_Change = 0;
                 s16_on_the_fly_position_opmode = 0;
            }

            break;

       case OPERATION_ENABLED:

            // fix for hold (hold will not finish if VAR(AX0_s16_Stopped) == -1)
            // so set it to 2 (last movement finished)
            if (VAR(AX0_s16_Stopped) == -1)
            {
                 VAR(AX0_s16_Stopped) = 2;
            }

            // if disbale, do transition 8
            if (u16_CtrlBits & 1)
            {
                 // transition 8 - shutdown
                 p402_controlword &= ~COMMAND_MASK;     // reset enable bit
                 p402_controlword |= SHUTDOWN;          // per =s= request, if disable, set canopen state machine to ready-to-switch-on state (transition #8)
            }
            else if (u16_CtrlBits & 4)
            {
                 s16_Active_Opmode = 0;
                 s16_Lxm_Reference_Val = 0;
                 // if quick stop, do transition 11
//                 p402_controlword &= ~COMMAND_MASK;

                 // reset bit 2,7 and set quick stop bit (quick stop code according to ds402 standard).
                 p402_controlword &= ~0x04;
                 p402_controlword &= ~FAULT_RESET;
                 p402_controlword |= QUICK_STOP;
            }

            break;

       case QUICK_STOP_ACTIVE:
            s16_Active_Opmode = 0;
            s16_Lxm_Reference_Val = 0;

            // set contol word to enable to prevent going to disable automatically in ds402 state machine
            p402_controlword &= ~0x10;
            p402_controlword |= ENABLE_OPERATION;

            // if disbale, do transition 12
            if (u16_CtrlBits & 1)
            {
               // transition 12 - disable voltage
               p402_controlword &= ~COMMAND_MASK;     // reset enable bit and quick stop bit
               p402_controlword |= DISABLE_VOLTAGE;
            }
            else if (u16_CtrlBits & 8)
            {
               // fault reset, do transition 16
               // raise halt bit to prevent start of motion when enable
               p402_controlword |= HALT_MOVE;
               s16_HaltBit_Cleared = 1;

               // stay with enable bits  in order to move from quick-stop to operation-enabled and not to disable
               p402_controlword |= FAULT_RESET;
            }
            break;

       case FAULT:
            s16_Active_Opmode = 0;
            s16_Lxm_Reference_Val = 0;
            BGVAR(u16_Lexium_Jog_Move_Activate) = 0;

            // if enable, set data error
            if (u16_CtrlBits & 2)
            {
                 BGVAR(u16_Lxm_DataError_Can_Obj) = FAULTS_INHIBITION;
                 BGVAR(u16_Lxm_DataErrorInfo_Can_Obj) = 1;
            }
            else
            {
                 // allow reset faults only if enable bit is off
                 if (u16_CtrlBits & 8)
                 {
                      // make sure control word is not set to enable and set faults reset
                      p402_controlword &= ~COMMAND_MASK;     // reset enable bit
                      p402_controlword |= SHUTDOWN;
                      p402_controlword |= FAULT_RESET;
                 }
            }
            break;
       case FAULT_REACTION_ACTIVE:
            BGVAR(u16_Lexium_Jog_Move_Activate) = 0;

            // if enable, set data error
            if (u16_CtrlBits & 2)
            {
                 BGVAR(u16_Lxm_DataError_Can_Obj) = FAULTS_INHIBITION;
                 BGVAR(u16_Lxm_DataErrorInfo_Can_Obj) = 1;
            }
            break;
   }



   // set halt has priority over clear halt
   if (u16_lxm_dmCtrl_shadow.bit.SH && u16_lxm_dmCtrl_shadow.bit.SH != u16_Lxm_DmCtrl_Prev.bit.SH)
   {
       // set halt
       s16_Lxm_Reference_Val = 0;
       p402_controlword |= HALT_MOVE;
       s16_HaltBit_Cleared = 0;
   }
   else if (u16_lxm_dmCtrl_shadow.bit.CH && u16_lxm_dmCtrl_shadow.bit.CH != u16_Lxm_DmCtrl_Prev.bit.CH)
   {
       // clear halt
       s16_HaltBit_Cleared = 1;
   }

   // continue bit
   s16_ContinueBit = 0;
   if (u16_lxm_dmCtrl_shadow.bit.CU && u16_lxm_dmCtrl_shadow.bit.CU != u16_Lxm_DmCtrl_Prev.bit.CU)
   {
       int temp_return = CheckLexiumAccessRights(drive, LEX_AR_FCT_CONTINUE, LEX_CH_FIELDBUS);
       if (temp_return != SAL_SUCCESS)
       {
            BGVAR(u16_Lxm_DataError_Can_Obj) = temp_return;
            BGVAR(u16_Lxm_DataErrorInfo_Can_Obj) = 1;
       }
       else
       {
            // handle continue opmode bit
            s16_ContinueBit = 1;
       }
   }*/
//}


//void Lxm_Handle_Opmode(int drive, Lxm_DmCtrl u16_lxm_dmCtrl_shadow, int s16_lxm_refA16_shadow, long s32_lxm_refB32_shadow)
//{
/*   int s16_opmode_request = u16_lxm_dmCtrl_shadow.bit.OpMode;
   int s16_opmode_action = u16_lxm_dmCtrl_shadow.bit.Action;
   unsigned int u16_temp_time, statusWord = p402_statusword;
   int retval;
   int s16_opmode_start_trigger = 0;    // 0: mt toggled, 1: continue bit is set
   long long s64_pcmd, s64_pcmd_addition;
   long s32_vcmd;
   
   
   // AXIS_OFF;

   // need to reset home start bit when finished
   if (s16_Lxm_Reset_Home_Start_Bit)
   {
     if ( (BGVAR(u8_Homing_State) == HOMING_IDLE)         ||
        (BGVAR(u8_Homing_State) == HOMING_TARGET_REACHED) ||
        (BGVAR(u8_Homing_State) == HOMING_FAILED) )
        {
            // reset start home bit
            p402_controlword &= ~HM_OP_START;
            s16_Lxm_Reset_Home_Start_Bit = 0;
        }
   }


   // special case: if byte is zero, reset MT and ME and dont process data
   if (s16_opmode_request == 0 &&
       u16_lxm_dmCtrl_shadow.bit.MT == 0 &&
       u16_lxm_dmCtrl_shadow.bit.Action == 0 &&
       s16_ContinueBit == 0)
   {
       // reset MT and ME bits
       u16_Lxm_MfStat.bit.ME = 0;
       u16_Lxm_MfStat.bit.MT = 0;
       return;
   }

   // if we are during opmode change, need to proceed to finish the change and start the new opmode
   if (s16_Opmode_Change == 0)
   {
       // if mode toggle is not toggled, do nothing)
       if (u16_lxm_dmCtrl_shadow.bit.MT == u16_Lxm_DmCtrl_Prev.bit.MT)
       {
//s16_nitsan_mt_check++;
            // if continue bit is set, need to continue last known opmode
            if (!s16_ContinueBit) return;
            s16_opmode_start_trigger = 1;
       }
   }

   // only if operation enabled
   if ((statusWord & STATUSWORD_STATE) != OPERATION_ENABLED)
   {
       // set mode error bit
       BGVAR(u16_Lxm_ModeError_Can_Obj) = DRIVE_INACTIVE;
       BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
       u16_Lxm_MfStat.bit.ME = 1;
       u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;    // lxm drive profile support
       return;
   }

//s16_debug_change_opmode[2]++;

   // check if need to change opmode
   if (s16_opmode_start_trigger == 0)
   {
       // in canopen JOG_MODE is 0xffff while in lexium drive profile it is 0x001f, so fix it here
       // in canopen GEAR_MODE is 0xfffe while in lexium drive profile it is 0x001e, so fix it here
       if (s16_opmode_request == 0x1f) s16_opmode_request = JOG_MODE;
       else if (s16_opmode_request == 0x1e) s16_opmode_request = GEAR_MODE;

       // check if request is for serial or analog command
       if (s16_opmode_request == PROFILE_VELOCITY_MODE)
       {
            // set opmode to analog vel
            s16_opmode_request = (s16_opmode_action == 0) ? ANALOG_VEL_MODE : PROFILE_VELOCITY_MODE;
       }
       else  if (s16_opmode_request == PROFILE_TORQUE_MODE)
       {
            // set opmode to analog torque
            s16_opmode_request = (s16_opmode_action == 0) ? ANALOG_TORQUE_MODE : PROFILE_TORQUE_MODE;
       }

       // opmode is 8 bit in CANopen
       if (s16_opmode_request == p402_modes_of_operation_display)
       {
            s16_Opmode_Change = -1; // signal that no opmdoe change is required
       }
       else
       {
            // need to stop motion and then change opmode
            switch (s16_Opmode_Change)
            {
                 case 0:
                     // change the opmode
                     BGVAR(s16_CAN_Opmode_Temp) = s16_opmode_request;
                     BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
                      
                      // fix IPR 1309
                     if (s16_opmode_request == PROFILE_POSITION_MODE)
                     {
                        retval = Lxm_Profile_Position_Opmode_Parse_Cmd(drive, s16_opmode_start_trigger, u16_lxm_dmCtrl_shadow, 
                                                     s16_lxm_refA16_shadow, s32_lxm_refB32_shadow);
                         
                        if (retval != SAL_SUCCESS)
                        {
                           // new opmode arguments are invalid, so cancel opmode change.
                           s16_Opmode_Change = 0;
                           return;
                        }
                         
                        s16_on_the_fly_position_opmode = 1;
                     }
                     else if (s16_opmode_request == JOG_MODE)
                     {
                        unsigned int u16_controlword_backup = p402_controlword;
                        retval = Lxm_Jog_Opmode(drive, s16_opmode_start_trigger, u16_lxm_dmCtrl_shadow, s16_lxm_refA16_shadow);
                        p402_controlword = u16_controlword_backup;
                        
                        if (retval != SAL_SUCCESS)
                        {
                           // new opmode arguments are invalid, so cancel opmode change.
                           s16_Opmode_Change = 0;
                           return;
                        }
                         
                        s16_on_the_fly_position_opmode = 1;
                         
                        // if opmode change on the fly (not via standstill)
                        if (BGVAR(u16_Opmode_Change_Mode))
                        {
                           // need to evaluate the jog command params (mode, speed, direction...)
                           // and extract the target position and target velocity commands
                           BGVAR(u16_Lexium_Jog_Move_Activate) = s16_lxm_refA16_shadow & 0x7;
                           
                           // find speed (bit 2)
                           if (BGVAR(u16_Lexium_Jog_Move_Activate) & 4)
                           {
                              // fast
                              s32_vcmd = BGVAR(s32_P4_80_Jog_SpeedFast_Out_Loop);
                           }
                           else
                           {
                              // slow
                              s32_vcmd = BGVAR(s32_P4_84_Jog_SpeedSlow_Out_Loop);
                           }
                           
                           // check jog mode
                           if(BGVAR(u16_P4_83_Jog_Method) == 0)
                           {
                              // only jog command
                              s64_pcmd_addition = 0x0100000000000000LL;
                           }
                           else
                           {
                              // first move ptp, then delay and jog
                              s64_pcmd_addition = BGVAR(s64_Lexium_Jog_Step);
                           }
                           
                           // Take the current position command as the starting point. Try to make it thread-safe.
                           do
                           {
                              u16_temp_time = Cntr_3125;
                              s64_pcmd = ((long long)LVAR(AX0_s32_Pos_Cmd_User_Hi) << 32) | ((long long)LVAR(AX0_u32_Pos_Cmd_User_Lo) & 0x00000000FFFFFFFFLL);
                           }
                           while (u16_temp_time != Cntr_3125);

                           
                           // find direction
                           if ((BGVAR(u16_Lexium_Jog_Move_Activate) & 3) == 1)
                           {
                              // positive direction
                              s64_pcmd = s64_pcmd + s64_pcmd_addition;
                           }
                           else 
                           {  
                              // negative direction
                              s64_pcmd = s64_pcmd - s64_pcmd_addition;
                           }
                           
                           // set pos and vel for transition to position mode on the fly
                           LLVAR(AX0_u32_Target_Position_Lo_Bg_Shadow) = s64_pcmd;
                           LVAR(AX0_s32_Vlimit_Bg_Shadow) = s32_vcmd;
                        }
                     }
                     else
                     {
                        s16_on_the_fly_position_opmode = 0;
                     }
                      
                     s16_Opmode_Change = 99;
                     break;
                      
                 case 99:
                      if (!IsOpmodeChangeInProgress(drive))
                      {
                        s16_Opmode_Change = 100;
                      }
                   
                      break;
              
                 case 100:
                      break;

                 default:
                      if (s16_Opmode_Change < 100)
                           s16_Opmode_Change++;
                      break;
            }

//s16_debug_change_opmode[6]++;
       }
   }
   else
   {
       // opmode starts since continue bit is set, so stay in same opmode
       s16_opmode_request = p402_modes_of_operation_display;
   }


   // continue to opmode only after changing opmode finished
   // or if continue bit is set (i.e. s16_Opmode_Change==0)
   if (s16_Opmode_Change != 100 && s16_Opmode_Change != 0 && s16_Opmode_Change != -1)
   {
       return;
   }
   
   

   // execute command according to opmode
   switch (s16_opmode_request)
   {
       case PROFILE_POSITION_MODE:
            // we got here without opmode change, so need to run the parse function again
            if (s16_Opmode_Change == -1)
            {
               retval = Lxm_Profile_Position_Opmode_Parse_Cmd(drive, s16_opmode_start_trigger, u16_lxm_dmCtrl_shadow, 
                                            s16_lxm_refA16_shadow, s32_lxm_refB32_shadow);
                
               if (retval != SAL_SUCCESS)
               {
                  // new opmode arguments are invalid, so cancel opmode change.
                  s16_Opmode_Change = 0;
                  return;
               }
            }
            
            Lxm_Profile_Position_Opmode_Execute(drive);
            
            break;

       case PROFILE_VELOCITY_MODE:
       case ANALOG_VEL_MODE:
            Lxm_Profile_Velocity_Opmode(drive, s16_opmode_start_trigger, u16_lxm_dmCtrl_shadow, s16_lxm_refA16_shadow, s32_lxm_refB32_shadow);
            break;

       case PROFILE_TORQUE_MODE:
       case ANALOG_TORQUE_MODE:
            Lxm_Profile_Torque_Opmode(drive, s16_opmode_start_trigger, u16_lxm_dmCtrl_shadow, s16_lxm_refA16_shadow, s32_lxm_refB32_shadow);
            break;

       case HOMING_MODE:
            Lxm_Homing_Opmode(drive, s16_opmode_start_trigger, u16_lxm_dmCtrl_shadow, s16_lxm_refA16_shadow, s32_lxm_refB32_shadow);
            break;

       case JOG_MODE:
            Lxm_Jog_Opmode(drive, s16_opmode_start_trigger, u16_lxm_dmCtrl_shadow, s16_lxm_refA16_shadow);
            break;

       case GEAR_MODE:
            Lxm_Gear_Opmode(drive, s16_opmode_start_trigger, u16_lxm_dmCtrl_shadow, s16_lxm_refA16_shadow, s32_lxm_refB32_shadow);
            break;

       default:
            // set mode error bit
            BGVAR(u16_Lxm_ModeError_Can_Obj) = INVALID_OPMODE;
            BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
            u16_Lxm_MfStat.bit.ME = 1;
            u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;    // lxm drive profile support
//s16_debug_change_opmode[1]++;
            break;
   }
   
   s16_Opmode_Change = 0;
   
   s16_on_the_fly_position_opmode = 0;*/
//}


// return error code or sal_success 
//int Lxm_Profile_Position_Opmode_Parse_Cmd(int drive, int start_trigger, Lxm_DmCtrl u16_lxm_dmCtrl_shadow, int s16_lxm_refA16_shadow, long s32_lxm_refB32_shadow)
//{
/*   static long long s64_pcmd_internal_units = 0;
   static long s32_vcmd = 0;
   unsigned int controlWord = p402_controlword;
   int s16_mode_err = 0;

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Lxm_ModeError_Can_Obj) = 0;
   BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 0;

   switch (u16_lxm_dmCtrl_shadow.bit.Action)
   {
       case 0:
            // allow absolute psoition only if drive is homed
            if (u16_Lxm_DriveStat.bit.RF == 0)
            {
                 s16_mode_err = 1;
                 BGVAR(u16_Lxm_ModeError_Can_Obj) = DRIVE_NOT_HOMED;
                 BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
            }
            else
            {
                 // absolut move (bit 6==0), immediate (bit 5==1), bit 4 should be reset and then set to signal new set-point
                 controlWord &= ~PPM_ABS_REL;
                 controlWord |= PPM_CHANGE_SET_IMMEDIATELY;
            }

            break;
       case 1:
            // relative to pcmd move (bit 6==1), immediate (bit 5==1), bit 4 should be reset and then set to signal new set-point
            controlWord |= (PPM_CHANGE_SET_IMMEDIATELY | PPM_ABS_REL);
            break;
       case 2:
            // relative to pfb move (bit 6==1), immediate (bit 5==1), bit 4 should be reset and then set to signal new set-point
            // will use absolute command with adding captured pfb
            controlWord &= ~PPM_ABS_REL;
            controlWord |= PPM_CHANGE_SET_IMMEDIATELY;
            break;

       default:
            // set mode error bit
            s16_mode_err = 1;
            BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_OUT_OF_RANGE;
            BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
            break;
   }

   if (s16_mode_err == 0)
   {
       if (start_trigger == 0) // trigger is mt toggle bit, so evaluate refA and refB
       {
            // set speed (do scaling if required).
            // need to do convertion into internal units
            s32_vcmd = (long)s16_lxm_refA16_shadow;
            s32_vcmd = s32_vcmd << BGVAR(u16_Lxm_Shift_RefA_Can_Obj);
       }

       if (s32_vcmd <= 0)
       {
            // velocity should be above zero
            s16_mode_err = 1;
            BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_TOO_LOW;
            BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 2;
       }
       else
       {
            if (start_trigger == 0) // trigger is mt toggle bit, so evaluate refA and refB
            {
                 // convert position into internal units (using canopen scaling)
                 s64_pcmd_internal_units = MultS64ByFixS64ToS64(s32_lxm_refB32_shadow,
                           BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
                           BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

                 // Fix IPR 1381: [CANopen] MC_ReadActPos_LXM28 with PosType=2 : "Target position", is not working
                 // update the 0x607A object with the target position value
                 // in order to allow the PLC read target position in the MC_ReadActPos_LXM28 function block
                 p402_target_position = s32_lxm_refB32_shadow;
                 
                 if (u16_lxm_dmCtrl_shadow.bit.Action == 2)
                 {
                      // relative to pfb
                      s64_pcmd_internal_units += s64_Lxm_Pfb_Capture;
                 }
            }

            BGVAR(s64_Fb_Target_Position) = s64_pcmd_internal_units;

            BGVAR(s32_Fb_Target_Velocity_For_Pos) = (long)MultS64ByFixS64ToS64((long long)(s32_vcmd),
                      BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

                      
             if (BGVAR(u16_Opmode_Change_Mode) == 1)
             {
               // set pos and vel for transition to position mode on the fly
               LLVAR(AX0_u32_Target_Position_Lo_Bg_Shadow) = BGVAR(s64_Fb_Target_Position);
               LVAR(AX0_s32_Vlimit_Bg_Shadow) = BGVAR(s32_Fb_Target_Velocity_For_Pos);
               s16_Active_Opmode = 1;
               s16_Lxm_MT_Request = 2;
             }
       }
   }
   
   u16_Lxm_MfStat.bit.ME = s16_mode_err;
   
   if (s16_mode_err)
   {
      // failure
      if (BGVAR(u16_Opmode_Change_Mode) == 1)
      {
         // yo update toggle bit
         s16_Lxm_MT_Request = 2;
      }
   }
   else
   {
      // success - update control word
      p402_controlword = controlWord;
      return SAL_SUCCESS;
   }
   
   return BGVAR(u16_Lxm_ModeError_Can_Obj);*/
//}


//void Lxm_Profile_Position_Opmode_Execute(int drive)
//{
/*   unsigned int controlWord = p402_controlword;
   
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   
   // issue move command
   if (s16_HaltBit_Cleared)
   {
        controlWord &= ~HALT_MOVE;
    }
  
   // clear new set point bit (in control word) in order to be ready for next position to arrive
   p402_position_option_code = 0x20;
   controlWord |= PPM_NEW_SET_POINT;
   s16_Lxm_Quickstop_Event = 0; // new motion is entered, so clear quick stop event (if was set)
   

   if (BGVAR(u16_Opmode_Change_Mode) == 0)
   {
       s16_Active_Opmode = 1;
       s16_Lxm_MT_Request = 1;
   }

   p402_controlword = controlWord;*/
//}


//void Lxm_Profile_Velocity_Opmode(int drive, int start_trigger, Lxm_DmCtrl u16_lxm_dmCtrl_shadow, int s16_lxm_refA16_shadow, long s32_lxm_refB32_shadow)
//{
//   static long s32_vcmd = 0;
//   long s32_vcmd_tmp = 0;
//   unsigned int controlWord = p402_controlword;
//   int s16_mode_err = 0;
//   int s16_vel_param = 0; // if velocity is transfered in refA or refB
//   long long s64_half_for_rounding;

//   // AXIS_OFF;

/*
   BGVAR(u16_Lxm_ModeError_Can_Obj) = 0;
   BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 0;

   // if start opmode becuse of MT toggle bit (==0), need to evaluate refA and refB,
   // if  start opmode because of 'continue' bit is set (==1), stay with old refA, refB
   if (start_trigger == 0)
   {
       switch (u16_lxm_dmCtrl_shadow.bit.Action)
       {
            case 0:
                 // analog command for velocity, do nothing
                 if (s16_HaltBit_Cleared)
                      controlWord &= ~HALT_MOVE;

                 p402_controlword = controlWord;
                 u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;
                 return;
                 //break;
            case 1:
                 // velocity is in RefA, need scaling
                 s32_vcmd = s16_lxm_refA16_shadow;
                 s32_vcmd = s32_vcmd << BGVAR(u16_Lxm_Shift_RefA_Can_Obj);
                 s16_vel_param = 1;
                 break;
            case 2:
                 // velocity is in RefB
                 s32_vcmd = s32_lxm_refB32_shadow;
                 s16_vel_param = 2;
                 break;
            default:
                 // set mode error bit
                 s16_mode_err = 1;
                 BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_OUT_OF_RANGE;
                 BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
                 break;
       }
   }

   if (s16_mode_err == 0)
   {
      // convert velocity into internal units (using canopen scaling)
      s32_vcmd_tmp = (long)MultS64ByFixS64ToS64((long long)(s32_vcmd),
                 BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                 BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      // convert from internal velocity (out of the loop) to internal units (in the loop)
      if (VAR(AX0_u16_Out_To_In_Vel_Shr) > 0)
         s64_half_for_rounding = 1LL << ((long long)VAR(AX0_u16_Out_To_In_Vel_Shr) - 1);

      // for usage in X_Add1 status bit
      s16_Vcmd_Inloop_Request = (int)((((long long)s32_vcmd_tmp * (long long)LVAR(AX0_s32_Out_To_In_Vel_Fix)) + s64_half_for_rounding) >> (long long)VAR(AX0_u16_Out_To_In_Vel_Shr));



      // if velocity is invalid
      if ((s32_vcmd_tmp > (long long)BGVAR(s32_V_Lim_Design)) || (s32_vcmd_tmp < -(long long)BGVAR(s32_V_Lim_Design)))
      {
         s16_mode_err = 1;
         BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_OUT_OF_RANGE;
         BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = s16_vel_param+1;
      }
      else
      {
         // here vcmd is valid
         BGVAR(s32_Fb_Target_Velocity_For_Vel) = s32_vcmd_tmp;
         // issue move command, if halt bit is cleared.
         if (s16_HaltBit_Cleared)
         {
              controlWord &= ~HALT_MOVE;
         }

         // mark that drive has reference value
         s16_Lxm_Reference_Val = 1;
      }
   }

   u16_Lxm_MfStat.bit.ME = s16_mode_err;
   if (s16_mode_err)
   {
       // if error, me bit is set, so set status toggle bit
       u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;
   }
   else
   {
       s16_Active_Opmode = 1;
       s16_Lxm_MT_Request = 1;
   }

   p402_controlword = controlWord;*/
//}

//void Lxm_Profile_Torque_Opmode(int drive, int start_trigger, Lxm_DmCtrl u16_lxm_dmCtrl_shadow, int s16_lxm_refA16_shadow, long s32_lxm_refB32_shadow)
/*{
   static int s16_tcmd = 0;
   static long s32_t_ramp = 0;
   int ret_val, s16_ilim_internal, s16_tcmd_internal;
   unsigned int controlWord = p402_controlword;
   int s16_mode_err = 0;
   long long s64_tmp;

   BGVAR(u16_Lxm_ModeError_Can_Obj) = 0;
   BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 0;

   // if start opmode becuse of MT tobble bit (==0), need to evaluate refA and refB,
   // if  start opmode because of 'continue' bit is set (==1), stay with old refA, refB
   if (start_trigger == 0)
   {
       switch (u16_lxm_dmCtrl_shadow.bit.Action)
       {
            case 0:
                 // analog command for torque, do nothing
                 if (s16_HaltBit_Cleared)
                      controlWord &= ~HALT_MOVE;

                 p402_controlword = controlWord;
                 u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;
                 return;
                 //break;

            case 1:
                 // torque is in 0.1% of micont ???
                 s16_tcmd = s16_lxm_refA16_shadow;

                 // handle torque ramp
                 if (s32_lxm_refB32_shadow)
                 {
                     s32_t_ramp = s32_lxm_refB32_shadow;

                     // get value in internal units (convert from CAN units to internal units)
                     s64_tmp = MultS64ByFixS64ToS64((long long)s32_t_ramp,
                              BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_internal_fix,
                              BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_internal_shr);

                     // get value in cdhd user units (convert from internal units to user units)
                     s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
                              BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_user_fix,
                              BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_user_shr);


                      // SalSetTorqueSlopeCommand takes torque ramp in canopen units
                      ret_val = SalSetTorqueSlopeCommand(s64_tmp,drive);
                      if (ret_val != SAL_SUCCESS)
                      {
                           // set mode error bit
                           s16_mode_err = 1;
                           BGVAR(u16_Lxm_ModeError_Can_Obj) = ret_val;
                           BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 3;
                      }
                 }
                 break;
            default:
                 // set mode error bit
                 s16_mode_err = 1;
                 BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_OUT_OF_RANGE;
                 BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
                 break;
       }
   }

   if (s16_mode_err == 0)
   {
      // convert velocity into internal units (using canopen scaling)
      s16_ilim_internal = (int)MultS64ByFixS64ToS64((long long)BGVAR(s32_Ilim_User),
                               BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                               BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);
*/
      /* ITAI - changed that from FB_CAN_TORQUE_CONVERSION to FB_CAN_CURRENT_CONVERSION */
      /* In order to avoid MKT usage is units conversion */
//      s16_tcmd_internal = (int)MultS64ByFixS64ToS64((long long)s16_tcmd,
//                          BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
//                          BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);
/*
      // for use in X_Add1 status bit
      s16_Tcmd_Internal_Request = s16_tcmd_internal;

      //make sure we do not exceed ILIM (for positive and negative torque command)
      if (s16_tcmd_internal > s16_ilim_internal)
      {
         s16_tcmd_internal = s16_ilim_internal;
      }
      else if (s16_tcmd_internal < -s16_ilim_internal)
      {
         BGVAR(s16_Fb_Target_Torque) = -s16_ilim_internal;
      }
      else
      {
         BGVAR(s16_Fb_Target_Torque) = s16_tcmd_internal;
      }

      // to allow reading target torque via canopen (Bugzilla 3553)
      p402_target_torque = s16_tcmd;

      // issue move command, if halt bit is cleared.
      if (s16_HaltBit_Cleared)
      {
         controlWord &= ~HALT_MOVE;
      }

      // mark that drive has reference value
      s16_Lxm_Reference_Val = 1;
   }

   u16_Lxm_MfStat.bit.ME = s16_mode_err;
   if (s16_mode_err)
   {
       // if error, me bit is set, so set status toggle bit
       u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;
   }
   else
   {
       s16_Active_Opmode = 1;
       s16_Lxm_MT_Request = 1;
   }

   p402_controlword = controlWord;*/
//}



//void Lxm_Homing_Opmode(int drive, int start_trigger, Lxm_DmCtrl u16_lxm_dmCtrl_shadow, int s16_lxm_refA16_shadow, long s32_lxm_refB32_shadow)
//{
/*   static int s16_home_type = 0;
   unsigned int controlWord = p402_controlword;
   int retVal, s16_mode_err = 0;

   BGVAR(u16_Lxm_ModeError_Can_Obj) = 0;
   BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 0;

   // if start opmode becuse of MT tobble bit (==0), need to evaluate refA and refB,
   // if  start opmode because of 'continue' bit is set (==1), stay with old refA, refB
   if (start_trigger == 0)
   {
       switch (u16_lxm_dmCtrl_shadow.bit.Action)
       {
            case 0:
                 // homing on current position with home offset
                 s16_home_type = 35;
                 BGVAR(s64_Home_Offset) = MultS64ByFixS64ToS64((long long)s32_lxm_refB32_shadow,
                         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
                         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

                 break;
            case 1:
                 // homing according to canopen type
                 s16_home_type = s16_lxm_refA16_shadow;
                 break;
            default:
                 // set mode error bit
                 s16_mode_err = 1;
                 BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_OUT_OF_RANGE;
                 BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
                 break;
       }
   }

   if (s16_mode_err == 0)
   {
       // start homing
       retVal = SalHomeTypeCommand(s16_home_type, drive);
       s16_mode_err = (retVal != SAL_SUCCESS);
       if (s16_mode_err)   // if error
       {
            BGVAR(u16_Lxm_ModeError_Can_Obj) = retVal;
            BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = (retVal == INVALID_HOMING_TYPE) ? 2 : 1;
       }

       controlWord |= HM_OP_START;
       if (s16_HaltBit_Cleared)
            controlWord &= ~HALT_MOVE;

       s16_Lxm_Reset_Home_Start_Bit = 1;
   }

   u16_Lxm_MfStat.bit.ME = s16_mode_err;
   if (s16_mode_err)
   {
       // if error, me bit is set, so set status toggle bit
       u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;
   }
   else
   {
       s16_Active_Opmode = 1;
       s16_Lxm_MT_Request = 1;
   }

   p402_controlword = controlWord;*/
//}

// schneider jog mode (opmode -1 in canopen)
//int Lxm_Jog_Opmode(int drive, int start_trigger, Lxm_DmCtrl u16_lxm_dmCtrl_shadow, int s16_lxm_refA16_shadow)
//{
/*   int s16_mode_err = 0;
   unsigned int controlWord = p402_controlword;
   int retVal;

   BGVAR(u16_Lxm_ModeError_Can_Obj) = 0;
   BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 0;

   // check access rights
   retVal = CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_JOG_MODE, LEX_CH_FIELDBUS);
   if (retVal != SAL_SUCCESS)
   {
       BGVAR(u16_Lxm_DataError_Can_Obj) = retVal;
       BGVAR(u16_Lxm_DataErrorInfo_Can_Obj) = 1;
       u16_Lxm_MfStat.bit.DE = 1;

       // if error, me bit is set, so set status toggle bit
       u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;
       return BGVAR(u16_Lxm_DataError_Can_Obj);
   }


   // if start opmode becuse of MT tobble bit (==0), need to evaluate refA and refB,
   // if  start opmode because of 'continue' bit is set (==1), stay with old refA, refB
   if (start_trigger == 0)
   {
       switch (u16_lxm_dmCtrl_shadow.bit.Action)
       {
            case 0:
                 // set jog activate word
                 if (s16_lxm_refA16_shadow >= 0 && s16_lxm_refA16_shadow <= 7)
                 {
                      // in s16_lxm_refA16_shadow: Bit 0 = jog positive; Bit 1 = jog negative; Bit 2 = jog slow/fast
                      // in controlword: Bit 4: Positive direction of movement, Bit 5: Negative direction of movement, Bit 6: 0=slow 1=fast
                      controlWord &= ~0x70;  // reset bits 4,5,6 beforr setting accoring to refA
                      controlWord |= (s16_lxm_refA16_shadow & 0x7) << 4;
                 }
                 else
                 {
                      s16_mode_err = 1;
                      BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_OUT_OF_RANGE;
                      BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 2;
                 }
                 break;

            default:
                 // set mode error bit
                 s16_mode_err = 1;
                 BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_OUT_OF_RANGE;
                 BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
                 break;
       }
   }

   if (s16_mode_err == 0)
   {
       // clear halt bit if needed
       if (s16_HaltBit_Cleared)
            controlWord &= ~HALT_MOVE;
   }

   u16_Lxm_MfStat.bit.ME = s16_mode_err;
   if (s16_mode_err)
   {
       // if error, me bit is set, so set status toggle bit
       u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;
   }
   else
   {
       s16_Active_Opmode = 1;
       u16_JogMode_MT_Request_Count = 3;  // wait 3 background cycles until setting MT bit
   }

   p402_controlword = controlWord;
   if (BGVAR(u16_Lxm_ModeError_Can_Obj) == 0)
      return SAL_SUCCESS;
   return BGVAR(u16_Lxm_ModeError_Can_Obj);*/
//}


// schneider gear mode (opmode -2 in canopen)
//void Lxm_Gear_Opmode(int drive, int start_trigger, Lxm_DmCtrl u16_lxm_dmCtrl_shadow, int s16_lxm_refA16_shadow, long s32_lxm_refB32_shadow)
//{
/*   // AXIS_OFF;

   int s16_mode_err = 0;
   int retVal;
   unsigned int controlWord = p402_controlword;
   int s16_gearLimitMode = 0;

   BGVAR(u16_Lxm_ModeError_Can_Obj) = 0;
   BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 0;

   // if start opmode becuse of MT tobble bit (==0), need to evaluate refA and refB,
   // if  start opmode because of 'continue' bit is set (==1), stay with old refA, refB
   if (start_trigger == 0)
   {
       switch (u16_lxm_dmCtrl_shadow.bit.Action)
       {
            case 0:
            case 1:
            case 2:
                 // set denominator (gear out) if not zero
                 if (s16_lxm_refA16_shadow)
                 {
                      BGVAR(s32_Fb_Gear_Out) = (int)s16_lxm_refA16_shadow;
                      retVal = SalGearOutCommand((long long)s16_lxm_refA16_shadow, drive);
                      if (retVal != SAL_SUCCESS)
                      {
                           // set mode error bit
                           s16_mode_err = 1;
                           BGVAR(u16_Lxm_ModeError_Can_Obj) = retVal;
                           BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 2;
                           break;
                      }
                 }

                 // set numerator (gear in) only if 16 bit (fail if outside the 16 bit boundaries)
                 if (s32_lxm_refB32_shadow > (long)0x1FFFFFFF || s32_lxm_refB32_shadow < (long)1)
                 {
                      // set mode error bit
                      s16_mode_err = 1;
                      BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_OUT_OF_RANGE;
                      BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 3;
                      break;
                 }

                 BGVAR(s32_Fb_Gear_In) = s32_lxm_refB32_shadow;
                 retVal = SalGearInCommand((long long)s32_lxm_refB32_shadow, drive);
                 if (retVal != SAL_SUCCESS)
                 {
                      // set mode error bit
                      s16_mode_err = 1;
                      BGVAR(u16_Lxm_ModeError_Can_Obj) = retVal;
                      BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 3;
                      break;
                 }

                 switch (u16_lxm_dmCtrl_shadow.bit.Action)
                 {
                      case 0:   // Position: Immediate synchronisation
                           s16_gearLimitMode = 4;
                           break;
                      case 1:   // Position: Compensation movement
                           s16_gearLimitMode = 5;
                           break;
                      case 2:   // Velocity: speed gear mode
                           s16_gearLimitMode = 28;
                           break;
                 }

                 break;

            default:
                 // set mode error bit
                 s16_mode_err = 1;
                 BGVAR(u16_Lxm_ModeError_Can_Obj) = VALUE_OUT_OF_RANGE;
                 BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
                 break;
       }
   }

   if (s16_mode_err == 0)
   {
       s16_gearLimitMode |= (VAR(AX0_u8_Gear_Limits_Mode) & 0x0002);   // preserve bit 1 of GEARLIMITSMODE (see P2-65)
       retVal = SalGearLimitsModeCommand((long long)s16_gearLimitMode, drive);
       if (retVal != SAL_SUCCESS)
       {
            // set mode error bit
            s16_mode_err = 1;
            BGVAR(u16_Lxm_ModeError_Can_Obj) = retVal;
            BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = 1;
       }
       else
       {
            // clear halt bit if needed
            if (s16_HaltBit_Cleared)
                 controlWord &= ~HALT_MOVE;

            s16_Lxm_Quickstop_Event = 0; // new motion is entered, so clear quick stop event (if was set)

            s16_Active_Opmode = 1;

            // fix IPR 856: Drive_Profile_LXM: X_end Bit = TRUE during GearIn is active AND "no external master pulses"
            // mark that drive has reference value
            s16_Lxm_Reference_Val = 1;
       }
   }

   u16_Lxm_MfStat.bit.ME = s16_mode_err;

   // in gear mode always set MT bit at this point.
   u16_Lxm_MfStat.bit.MT = u16_lxm_dmCtrl_shadow.bit.MT;

   p402_controlword = controlWord;*/
//}


//int s16_nitsan_debug_toggle_input=0;
//void Lxm_Update_Status(int drive)
//{
//   unsigned int statusWord = p402_statusword;
//   unsigned int u16_motor_stopped_shadow;
//   // AXIS_OFF;

/*
   // take shadow values of profile generator RT vars
   int s16_shadow_vel_var_ref_0 = VAR(AX0_s16_Vel_Var_Ref_0); // vcmd
   long s32_shadow_delta_delta_pos_cmd = LVAR(AX0_s32_Delta_Delta_Pos_Cmd);
   long s32_shadow_Pos_vcmd = LVAR(AX0_s32_Pos_Vcmd);    // ptpvcmd
   int s16_min_no_acc_val = 32;
*/
/*   int s16_vfb_shadow;

   // Drive status (driveStat)
   u16_Lxm_DriveStat.bit.State = Lxm_GetDriveState(statusWord);
   u16_Lxm_DriveStat.bit.Fault = ((statusWord & FAULT) != 0);
   u16_Lxm_DriveStat.bit.Warn = ((statusWord & WARNING_MASK) != 0);
   u16_Lxm_DriveStat.bit.Halt = (p402_controlword & HALT_MOVE) && (s16_HaltBit_Cleared == 0);
   u16_Lxm_DriveStat.bit.RF = (BGVAR(u8_Homing_State) == HOMING_TARGET_REACHED);
   u16_Lxm_DriveStat.bit.QS = u16_Lxm_DmCtrl.bit.QS;     // reflect the command


   // x_err == 0 when ds402 state is OPERATION_ENABLED, x_err == 1 otherwise
   u16_Lxm_DriveStat.bit.X_err = ((statusWord & STATUSWORD_STATE) != OPERATION_ENABLED);

   u16_Lxm_DriveStat.bit.X_end = LXM_Get_X_End(drive, s16_Active_Opmode, u16_Lxm_DriveStat, s16_Lxm_Reference_Val);
   u16_Lxm_DriveStat.bit.X_add1 = LXM_Get_X_Add1(drive, statusWord, u16_Lxm_DriveStat);

   //u16_Lxm_DriveStat.bit.X_err = (statusWord & STATUSWORD_STATE) != OPERATION_ENABLED;

   // Mode and Function Status (mfStat)
   u16_Lxm_MfStat.bit.DE = (BGVAR(u16_Lxm_DataError_Can_Obj) != 0);

   if (p402_modes_of_operation_display == ANALOG_VEL_MODE)
   {
      u16_Lxm_MfStat.bit.OpMode = 3;
   }
   else if (p402_modes_of_operation_display == ANALOG_TORQUE_MODE)
   {
      u16_Lxm_MfStat.bit.OpMode = 4;
   }
   else
   {
      u16_Lxm_MfStat.bit.OpMode = p402_modes_of_operation_display;
   }

   if ((u16_JogMode_MT_Request_Count != 0xffff) && (u16_JogMode_MT_Request_Count > 0))
   {
      u16_JogMode_MT_Request_Count--;
   }
   else if (u16_JogMode_MT_Request_Count == 0)
   {
      s16_Lxm_MT_Request = 2;
      u16_JogMode_MT_Request_Count = 0xffff; // signal to disable the counter
   }

   // update MT after mode is evaluated and handled (DE or ME are set at this point).
   if (s16_Lxm_MT_Request == 2)
   {
       s16_Lxm_MT_Request = 0;
       u16_Lxm_MfStat.bit.MT = u16_Lxm_DmCtrl.bit.MT;
   }

   u16_Lxm_MfStat.bit.Cap1 = VAR(AX0_u16_TProbe_Cntr_Arr[0]) + VAR(AX0_u16_TProbe_Cntr_Arr[1]);
   u16_Lxm_MfStat.bit.Cap2 = VAR(AX0_u16_TProbe_Cntr_Arr[2]) + VAR(AX0_u16_TProbe_Cntr_Arr[3]);

   // motion status (motionStat)
   // 10rpm = 89478 out_loop; 6rpm = 53687
   u16_Lxm_MotionStat.bit.MotZ = 0;
   u16_Lxm_MotionStat.bit.MotP = 0;
   u16_Lxm_MotionStat.bit.MotN = 0;

   s16_vfb_shadow = BGVAR(s16_Motor_Moving);
   u16_motor_stopped_shadow = VAR(AX0_u16_Motor_Stopped);
   if (u16_motor_stopped_shadow == 2)     // means standing in position
       u16_Lxm_MotionStat.bit.MotZ = 1;
   else if (s16_vfb_shadow > 0)           // if vfb positive (and outside 6rpm window), positive movement
       u16_Lxm_MotionStat.bit.MotP = 1;
   else if (s16_vfb_shadow < 0)           // if vfb negative (and outside 6rpm window), neggative movement
       u16_Lxm_MotionStat.bit.MotN = 1;
   else
       u16_Lxm_MotionStat.bit.MotZ = 1;
*/

   /*
   if (s16_vfb_shadow == 0)               // if vfb inside 6rpm window, no movement
       u16_Lxm_MotionStat.bit.MotZ = 1;
   else if (s16_vfb_shadow > 0)           // if vfb positive (and outside 6rpm window), positive movement
       u16_Lxm_MotionStat.bit.MotP = 1;
   else                                   // if vfb negative (and outside 6rpm window), neggative movement
       u16_Lxm_MotionStat.bit.MotN = 1;
   */
/*
   u16_Lxm_MotionStat.bit.LimP = ((VAR(AX0_u16_CW_LS) & 0x2) == 0x2);
   u16_Lxm_MotionStat.bit.LimN = ((VAR(AX0_u16_CCW_LS) & 0x2) == 0x2);

   // as defined in for param DS402intLim in LXM32M manual V107
   u16_Lxm_MotionStat.bit.Info_0 = Lxm_InfoBit(drive, BGVAR(u16_Lxm_DplIntLim_Can_Obj));
   u16_Lxm_MotionStat.bit.Info_1 = Lxm_InfoBit(drive, BGVAR(u16_Lxm_DS402IntLim_Can_Obj));

   // digital inputs:
   // forced value if exists or input value.
   u16_Lxm_InputStat.all = VAR(AX0_u16_P4_07_Read) & 0xFF;   // if have 8 inputs in lxm28
   u16_Lxm_InputStat.bit.Reserved = 0;
*/

   // toggle bit for debug only - to remove
   /* ******************************************* */
   //u16_Lxm_InputStat.bit.DI1=s16_nitsan_debug_toggle_input;
   //s16_nitsan_debug_toggle_input=!s16_nitsan_debug_toggle_input;
   /* ******************************************* */

/*
   // update canopen objects for PDO access
   manu_spec_u1_P3_15_driveStat = u16_Lxm_DriveStat.all;
   manu_spec_u1_P3_16_mfStat = u16_Lxm_MfStat.all;
   manu_spec_u1_P3_17_motionStat = u16_Lxm_MotionStat.all;
   manu_spec_u16_P3_11_driveInput = u16_Lxm_InputStat.all;

   // update canopen objects for SDO access
   BGVAR(u16_P3_15_Lxm_DriveStat_Can_Obj) = u16_Lxm_DriveStat.all;
   BGVAR(u16_P3_16_Lxm_MfStat_Can_Obj) = u16_Lxm_MfStat.all;
   BGVAR(u16_P3_17_Lxm_MotionStat_Can_Obj) = u16_Lxm_MotionStat.all;
   BGVAR(u16_P3_11_Lxm_DriveInput_Can_Obj) = u16_Lxm_InputStat.all;*/
//}


//**********************************************************
// Function Name: Lxm_GetDriveState
// Description:
//          This function returns the drive state according to lexium drive profile.
//          states values are specified in diagram in file: Bai_Yang_DES04_Drive_Profile_LXM.doc, page 30
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
//unsigned int Lxm_GetDriveState(unsigned int u16_statusWord)
//{
/*   int lxm_state = 0;

   switch (u16_statusWord & STATUSWORD_STATE)
   {
        case NOT_READY_TO_SWITCH_ON:
            lxm_state = 2;
            break;
         case SWITCH_ON_DISABLED:
            lxm_state = 3;
            break;
         case READY_TO_SWITCH_ON:
            lxm_state = 4;
            break;
         case SWITCHED_ON:
            lxm_state = 5;
            break;
         case OPERATION_ENABLED:
            lxm_state = 6;
            break;
         case QUICK_STOP_ACTIVE:
            lxm_state = 7;
            break;
         case FAULT_REACTION_ACTIVE:
            lxm_state = 8;
            break;
         case FAULT:
            lxm_state = 9;
            break;

   }

   return lxm_state;*/
//}


// calc the required shift left value for velocity which is sent as 16 bit (in refA16)
//void Lxm_SetScaler(int drive)
//{
/*   int i = 0;
   long long max_16_bit = 1LL << 15;

   //long long tmp = ((long long)BGVAR(s32_V_Lim_Design) * (long long)BGVAR(u32_Scaling_Pnumerator)) / (long long)BGVAR(u32_Scaling_Pdenominator);

   long long tmp =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(s32_V_Lim_Design)),
              BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
              BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

//   s64_vlim_can_units = tmp;

   while (tmp > max_16_bit)
   {
       i++;
       tmp = tmp >> 1;
   }

   BGVAR(u16_Lxm_Shift_RefA_Can_Obj) = i;*/
//}


//int Lxm_InfoBit(int drive, int s16_infoBit)
//{
/*   int value = 0;
   // AXIS_OFF;

   // as defined in for param DS402intLim in LXM32M manual V107
   switch (s16_infoBit)
   {
       case 1:        // current below threshold (current internl units)
            value = BGVAR(u32_EqCrrnt) >= BGVAR(u32_Lxm_Current_Threshold_Can_Obj);
            break;
       case 2:        // velocity below threshold (vel in-loop units)
            //value = abs(VAR(AX0_s16_Vel_Var_Err)) < BGVAR(u16_Lxm_Vel_Threshold_In_Loop);
            
            // units are internal velocity (out of loop).
            value = labs(LVAR(AX0_s32_Vel_Var_Fb_0)) < BGVAR(u32_Lxm_Vel_Threshold_Can_Obj);
            break;

       // alexander badura (=s=) said that 3 and 11 are the same !!!
       case 3:        // in position deviation window
       case 11:       // position window
            value = (VAR(AX0_u16_Inpos) == 1);
            break;
       case 4:        // in velocity deviation window (vel in-loop units)
            value = abs(VAR(AX0_s16_Vel_Var_Err)) > BGVAR(u16_Lxm_Vel_Threshold_In_Loop);
            break;
       case 9:        // hardware limit switch (bit 0 is for sw limit, bit 1 is for hw limit)
            value = (VAR(AX0_u16_CW_LS) & 0x02) || (VAR(AX0_u16_CCW_LS) & 0x02);
            break;
   }

   return value;*/
//}


// currently not for profile position and homing
//int LXM_Get_X_End(int drive, int s16_active_opmode, Lxm_DriveStat lxm_drvStat, int s16_reference_val)
//{
/*   // AXIS_OFF;

   // if drive disabled, motion has ended
   if (Enabled(drive) == 0)
   {
      return 1;
   }

   // drive is not in NMT operational, quick stop, active fault, motion has ended
   if ((lxm_drvStat.bit.State < 6) || (lxm_drvStat.bit.State > 8))
   {
      return 1;
   }

   // drive is changing opmode.
   // nitsan: 21/5/14 at integration with =s= (with Johachim Eichner) it was decided to return 1 when opmode is changing instead
   // of zero in order to allow the PLC to know when opmode is active

   if (s16_Opmode_Change)
   {
      return 1;
   }

   // check if there is an active opmode (e.g on transition from disabel to enable there is no active opmode)
   if (!s16_active_opmode)
   {
       return 1;
   }

   if (lxm_drvStat.bit.QS || lxm_drvStat.bit.Halt)
   {
       // return 1 if motor is standing, 0 if motor is moving
       //return !BGVAR(s16_Motor_Moving);
       return (VAR(AX0_u16_Motor_Stopped) == 2);
   }

   if (p402_modes_of_operation_display == PROFILE_POSITION_MODE)
   {
       // return 1 if motor is standing, 0 if motor is moving
       //return !BGVAR(s16_Motor_Moving);
       return (VAR(AX0_u16_Motor_Stopped) == 2);
   }
   else if (p402_modes_of_operation_display == HOMING_MODE)
   {
      // zero when homing is active, 1 when finished (success or error) or not started
      if (!IsHomingProcessRunning(drive))
      {
         return 0;
      }
      else
      {
         return 1;
      }
   }
   else if (p402_modes_of_operation_display == JOG_MODE)
   {
       // if jog activate word has no motion value
       if (((BGVAR(u16_Lexium_Jog_Move_Activate) & 0x3) == 0) || ((BGVAR(u16_Lexium_Jog_Move_Activate) & 0x3) == 3))
       {
            // return 1 if motor is standing, 0 if motor is moving
            //return !BGVAR(s16_Motor_Moving);
            return (VAR(AX0_u16_Motor_Stopped) == 2);
       }

       return 0;
   }
   else if ((p402_modes_of_operation_display == INTRPOLATED_POSITION_MODE)        ||
            (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_POSITION_MODE) ||
            (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_VELOCITY_MODE) ||
            (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_TORQUE_MODE))
   {
      // IPR 855: CANmotion: DTM access possible if Bit14 - X_end - is not set
      // for CANopen sync modes:
      // X_end=0 if state is OPERATION_ENABLED
      // X_end=1 if drive is in any other state
      return (lxm_drvStat.bit.State == 6) ? 0 : 1;
   }
   else
   {
       // if we have reference value, motion has not ended.
       return (!s16_reference_val);
   }*/
//}

//int LXM_Get_X_Add1(int drive, unsigned int statusWord, Lxm_DriveStat lxm_drvStat)
//{
/*   long long s64_pcmd;
   int s16_temp_time;
   // AXIS_OFF;

   switch (p402_modes_of_operation_display)
   {
       case PROFILE_POSITION_MODE:
            if (lxm_drvStat.bit.State == 6)
            {
                 //if (lxm_drvStat.bit.Halt && BGVAR(s16_Motor_Moving) == 0)
                 if (lxm_drvStat.bit.Halt && (VAR(AX0_u16_Motor_Stopped) == 2))
                      return 1;

                 // compare target position with pcmd (return 1 if equal)
                 do {
                      s16_temp_time = Cntr_3125;
                      s64_pcmd = LLVAR(AX0_u32_Pcmd_Raw_Lo);
                 } while (s16_temp_time != Cntr_3125);


                 if (s64_pcmd == BGVAR(s64_Gen_Target_Position))
                      return 1;
            }
            //else if (lxm_drvStat.bit.State == 7 && BGVAR(s16_Motor_Moving) == 0)
            else if (lxm_drvStat.bit.State == 7 && (VAR(AX0_u16_Motor_Stopped) == 2))
            {
                 // set flag for quick stop occured
                 if (lxm_drvStat.bit.State == 7) s16_Lxm_Quickstop_Event = 1;
                 return 1;
            }

            if (s16_Lxm_Quickstop_Event) return 1;

            break;
       case PROFILE_VELOCITY_MODE:
       case PROFILE_TORQUE_MODE:
            // if state is quick stop active (7)
            if (lxm_drvStat.bit.State == 7)
            {
                 //if (BGVAR(s16_Motor_Moving) == 0)
                 if (VAR(AX0_u16_Motor_Stopped) == 2);   // not moving
                      return 1;
            }
            else if (lxm_drvStat.bit.State == 6)    // if operation enabled
            {
                 if (p402_controlword & HALT_MOVE)
                 {
                      // if drive in standstill
                      //if (BGVAR(s16_Motor_Moving) == 0)
                      if (VAR(AX0_u16_Motor_Stopped) == 2);   // not moving
                           return 1;
                 }
                 else
                 {
                      if (p402_modes_of_operation_display == PROFILE_TORQUE_MODE)
                      {
                           // if torque window is zero, then torque window feature is inactive
                           if (manu_spec_Torque_Window_Command == 0L)
                           {
                                // if target torque equal icmd
                                if (s16_Tcmd_Internal_Request == VAR(AX0_s16_Icmd))
                                     return 1;
                           }
                           else
                           {
                                // torque window is active
                                if (statusWord & PPM_TARGET_REACHED)
                                     return 1;
                           }
                      }
                      else
                      {
                           // vel mode
                           // if vel window is zero, then speed window feature is inactive
                           if (BGVAR(s32_Velocity_Window) == 0L)
                           {
                                // if target vel equal vcmd
                                if (s16_Vcmd_Inloop_Request == VAR(AX0_s16_Vel_Var_Ref_0))
                                     return 1;
                           }
                           else
                           {
                                // speed window is active
                                if (statusWord & PVM_STAT_TARGET_REACHED)
                                     return 1;
                           }
                      }
                 }
            }

            break;

       case HOMING_MODE:
            // if state is quick stop active (7) and drive not moving
            if ((lxm_drvStat.bit.X_err == 0 && lxm_drvStat.bit.X_end == 1) ||
                 //(lxm_drvStat.bit.State == 7 && BGVAR(s16_Motor_Moving) == 0))
                 (lxm_drvStat.bit.State == 7 && (VAR(AX0_u16_Motor_Stopped) == 2)))
                 return 1;

            break;

       case GEAR_MODE:
            // quick stop and motor in standstill
            if ((lxm_drvStat.bit.State == 7) && (VAR(AX0_u16_Motor_Stopped) == 2))
            {
               s16_Lxm_Quickstop_Event = 1;
               return 1;
            }
            else if ((lxm_drvStat.bit.State == 6) && (lxm_drvStat.bit.Halt == 1) && (VAR(AX0_u16_Motor_Stopped) == 2))
            {
               // state operational and halt is requested and motor in standstill
               return 1;
            }
            else if (lxm_drvStat.bit.State == 6)    // operation enabled: check vel window
            {
               // difference between ptpvcmd and vfb (if window is active)
               if (BGVAR(s32_Velocity_Window) > 0)
               {
                  // fix IPR 705: Canopen: FB MC_GearIn is clearing flag "IN Gear" when gearing is working
                  // ptpvcmd (AX0_s32_Pos_Vcmd) is in ptp_vel units (fbc32/Tsp)
                  // v (AX0_s32_Vel_Var_Fb_0) and s32_Velocity_Window are in out_loop vel units
                  // devide ptpvcmd by 2 in order to use same units (ptpvcmd runs in pos loop which runs half the times of vel loop).
                  if(  ((labs(LVAR(AX0_s32_Pos_Vcmd) - LVAR(AX0_s32_Vel_Var_Fb_0)) < BGVAR(s32_Velocity_Window))  && (VAR(AX0_u16_Pos_Loop_Tsp) != TSP_250_USEC))      ||
                       ((labs((LVAR(AX0_s32_Pos_Vcmd) >> 1) - LVAR(AX0_s32_Vel_Var_Fb_0)) < BGVAR(s32_Velocity_Window)) && (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC))  )
                  {
                     return 1;
                  }
               }
            }

            if (s16_Lxm_Quickstop_Event)
                 return 1;

            break;

       case JOG_MODE:
            // always return zero.
            break;
   }

   return 0;*/
//}



//#pragma CODE_SECTION(LXM_MotionGeneratorState, "ramfunc_3");
//void LXM_MotionGeneratorState(int drive)
//{
/*   unsigned int cntr_rt;
   long s32_shadow_delta_delta_pos_cmd;
//   long s32_shadow_pos_vcmd;
   long s32_vcmd = 0;
   static int s16_cnst_cntr = 0;
   int s16_cntr_max = 10;


   // AXIS_OFF;

   // find profile generator state for position and velocity (velControlMode=5 or 6)
   if ((BGVAR(s16_P3_10_PlcEn) == 1) && ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN))
   {
       // if serial torque or analog torque
       if (VAR(AX0_s16_Opmode) == 2 || VAR(AX0_s16_Opmode) == 3)
       {
            if (VAR(AX0_s16_Icmd) == 0)
            {
                 u16_Lxm_MotionStat.bit.MotionGenerator = 1;  // tar_0 (standstill)
            }
            else
            {
                 if (VAR(AX0_s16_Slope_Acc_Dec) > 0)
                      u16_Lxm_MotionStat.bit.MotionGenerator = 4;  // acc
                 else if (VAR(AX0_s16_Slope_Acc_Dec) < 0)
                      u16_Lxm_MotionStat.bit.MotionGenerator = 2;  // dec
                 else
                      u16_Lxm_MotionStat.bit.MotionGenerator = 8;  // cnst
            }
       }
       else
       {
            do
            {
               cntr_rt = Cntr_3125;
               s32_shadow_delta_delta_pos_cmd = LVAR(AX0_s32_Delta_Delta_Pos_Cmd);
               s32_vcmd = (long)(*(long*)(VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr) & 0xffff));
            }
            while (cntr_rt != Cntr_3125);

            if (s32_shadow_delta_delta_pos_cmd > 0)
                      s32_shadow_delta_delta_pos_cmd &= 0xFFFFffe0;     // zero last 5 bits (LSB)
            else
            {
                 s32_shadow_delta_delta_pos_cmd = -s32_shadow_delta_delta_pos_cmd;
                 s32_shadow_delta_delta_pos_cmd &= 0xFFFFffe0;     // zero last 5 bits (LSB)
                 s32_shadow_delta_delta_pos_cmd = -s32_shadow_delta_delta_pos_cmd;
            }

            if ((s32_vcmd == 0))
            {
                 u16_Lxm_MotionStat.bit.MotionGenerator = 1;  // tar_0 (standstill)
            }
            else
            {
                 if (s32_shadow_delta_delta_pos_cmd == 0)
                 {
                      s16_cnst_cntr++;
                      if (s16_cnst_cntr > s16_cntr_max)
                      {
                           s16_cnst_cntr = s16_cntr_max;
                           u16_Lxm_MotionStat.bit.MotionGenerator = 8;  // cnst
                      }
                 }
                 else
                 {
                      s16_cnst_cntr = 0;
                 }

                 if (s16_cnst_cntr != s16_cntr_max)
                 {
                      if ((long long)s32_vcmd * (long long)s32_shadow_delta_delta_pos_cmd > 0)  // both are positive or negative (same signs)
                           u16_Lxm_MotionStat.bit.MotionGenerator = 4;  // acc
                      else if ((long long)s32_vcmd * (long long)s32_shadow_delta_delta_pos_cmd < 0)  // one is positive and other is negative (different signs)
                           u16_Lxm_MotionStat.bit.MotionGenerator = 2;  // dec
                 }
            }
       }
   }*/
//}


//**********************************************************
// Function Name: Lxm_Is_Can_Position_Referenced
// Description:
//          This function is called for lexium proofile handler and calle devery background cycle.
//          If drive is homed and pcmd overflows the maximum canopen position 
//          avaliable by 32bit presentation, it will return false.
//          If drive is not homed or homed but position is inside canopen 32bit position presentation, it will return true;
//
//          After homing process is finished, the functin will wait one BG cycle to avoid false value if big home offset is used.
//          If home state is canceled, the one_bg_cycle var will be set again.
//
//          Function is added as result of IPR 1395: Drive Profile LXM - RF Bit not reset if LXM28 overtravel the maximum position range.
//          
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
//int Lxm_Is_Can_Position_Referenced(int drive)
//{
/*   static long long s64_pcmd_prev = 0; // in internal units (dont use canopen units here to avoid the need to update this variable if units scaling is changed).
   static int s16_num_wait_homing_finished_bg_cycles = 1;
   int s16_temp_time;
   long long s64_pcmd;
   long s32_pcmd_canopen, s32_pcmd_prev_canopen;
   
   // AXIS_OFF;
   
   // take pcmd without RT interfering 
   do 
   {
      s16_temp_time = Cntr_3125;
      s64_pcmd = LLVAR(AX0_u32_Pcmd_Raw_Lo);
   } while (s16_temp_time != Cntr_3125);
   
   
   // if need to wait for one cycle after homing
   if (s16_num_wait_homing_finished_bg_cycles > 0)
   {
      if (BGVAR(u8_Homing_State) == HOMING_TARGET_REACHED)
      {
         s16_num_wait_homing_finished_bg_cycles--;
      } 
   
      // update prev pcmd with the current one, and return 1 to avoid loosing is_homed state
      s64_pcmd_prev = s64_pcmd;
      return 1;
   }
   
   
   // if not homed
   if (BGVAR(u8_Homing_State) != HOMING_TARGET_REACHED)
   {
      // update num of cycles to wait for next homing, update pcmd and return 1 to avoid loosing is_homed state
      s16_num_wait_homing_finished_bg_cycles = 1;
      s64_pcmd_prev = s64_pcmd;
      return 1;
   }
   
   // convert to canopen units and check if target position is overflowing  
   s32_pcmd_canopen = (long)MultS64ByFixS64ToS64(s64_pcmd,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

   s32_pcmd_prev_canopen = (long)MultS64ByFixS64ToS64(s64_pcmd_prev,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);
                
   // update prev pcmd with the current one.
   s64_pcmd_prev = s64_pcmd;
                   
   // check if there is overflow for target position
   
   // if the variables have opposite signs (one is neg and other is pos)
   if ((s32_pcmd_canopen ^ s32_pcmd_prev_canopen) & 0x80000000)
   {
      if (labs(s32_pcmd_canopen) > (LONG_MAX >> 1))
      {
         // diff is too large
         // overflow
         return 0;
      }
   }
   
   return 1;*/
//}
