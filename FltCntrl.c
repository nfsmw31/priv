//###########################################################################
//
// FILE:        FltCntr.c
//
// TITLE:       Fault control functions
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 17 Jun 2002 | Y.D. | Creation
//
//##################################################################

#include "DSP2834x_Device.h"
#include "utility.h"
#include "nmt.h"
#include "pdo.h"

#include "FltCntrl.def"
#include "Err_Hndl.def"
#include "ModCntrl.def"
#include "Endat.def"
#include "Hiface.def"
#include "design.def"
#include "MultiAxis.def"
#include "ser_comm.def"
#include "FPGA.def"
#include "CommFdbk.def"
#include "i2c.def"
#include "PhaseFind.def"
#include "Current.def"
#include "SensorlessBG.def"
#include "Exe_IO.def"
#include "402fsa.def"
#include "ExFbVar.def"
#include "Init.def"
#include "Modbus_comm.def"
#include "Flash.def"
#include "PtpGenerator.def"
#include "BiSS_C.def"

#include "ModCntrl.var"
#include "FltCntrl.var"
#include "Foldback.var"
#include "Ser_Comm.var"
#include "Motor.var"
#include "Drive.var"
#include "User_Var.var"
#include "Extrn_Asm.var"
#include "Endat.var"
#include "Hiface.var"
#include "Init.var"
#include "Runtime.var"
#include "FlashHandle.var"
#include "Flash.var"
#include "CommFdbk.var"
#include "Display.var"
#include "i2c.var"
#include "PhaseFind.var"
#include "MotorSetup.var"
#include "Position.var"
#include "Units.var"
#include "SensorlessBG.var"
#include "Exe_IO.var"
#include "Velocity.var"
#include "ExFbVar.var"
#include "an_fltr.var"
#include "Modbus_Comm.var"
#include "PtpGenerator.var"
#include "BiSS_C.var"

#include <cal_conf.h>
#include <co_emcy.h>
#include <objects.h>

#include "Prototypes.pro"
#include "BiSS_C.pro"


extern PDO_COMM_PAR3_T	p301_n1_rpdo_para;
extern PDO_COMM_PAR3_T	p301_n2_rpdo_para;
extern PDO_COMM_PAR3_T	p301_n3_rpdo_para;
extern PDO_COMM_PAR3_T	p301_n4_rpdo_para;


void FaultsHandler(int drive, int mode)
{
   NvramFault(drive, mode);

   FieldBusTargetPositionVelFault(drive, mode);

   //Decided to remove Fieldbus ACC/DEC detection
   //FieldBusTargetPositionAccDecFault(drive, mode);

   FieldBusEtherCATCableDisconnectedFault(drive, mode);

   FieldBusEtherCATPacketsLostFault(drive, mode);

   FieldBusActiveNotOpFault(drive, mode);

   FieldBusTargetCommandLostFault(drive, mode);

   NvramChecksumFault(drive, mode);

   FeedbackFailure(drive, mode);

   STOFault(drive, mode); // must be called before 1)VbusMeasureFault 2)UnderVoltageFault 3)OverVoltageFault 4)DriveOTFault

   VbusMeasureFault(drive, mode);

   UnderVoltageFault(drive, mode);

   OverVoltageFault(drive, mode);

   FoldbackFault(drive, mode);

   DriveOTFaults(drive, mode);

   VospdFault(drive, mode);

   PeMaxFault(drive, mode);

   DriveNumericalPeFault(drive,mode);

   DriveRTOverloadFault(drive,mode);

   NotConfiguredFault(drive);

   OverCurrentFault(drive, mode);
   
   DigitalOutputOCFault(drive, mode);

   UnStableCurrLoopFault(drive, mode);
   
   HighIqFault(drive, mode);

   FPGAFault(drive);

   if (IS_EC_DRIVE_AND_COMMODE_1)
   {
      UBLAZEVersionMismatchFault(drive);
      ESIVersionMismatchFault(drive);
      ESIVendorMismatchFault(drive);
   }

   SecurityFault();

   RegenOCFault(drive, mode);

   RegenOverloadFault(drive, mode);

   CANSupplyFault(drive, mode);

   BrakeFault(drive, mode);

   SelfTestFault(drive, mode);

   EEpromFaults(drive);

   Plus15VFault(drive, mode);

   Minus15VFault(drive, mode);

   Drive5VFault(drive, mode);

   LogicACSupplyFault(drive, mode);

   BusACSupplyFault(drive, mode);

   CrrntSensorsOffsetFault(drive, mode);

   MotorSetupFault(drive, mode);

   PllFault(drive, mode);

   MotorPhaseDisconnect(drive, mode);

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))   // If not Lexium product
   {
      LimitSwitchWarn(drive);
   }
   else // If Lexium product
   {
      LexiumLimitSwitchWarn(drive);
   }

   SensorLessFaults(drive , mode);

   EmergencyStop(drive , mode);

   StallFault(drive , mode);

   CanOpenShortPdoFault(drive , mode);

   CanOpenHeartBeatLostFault(drive , mode);

   if (mode == FLT_CLR)
   {
      CANFaultControl(drive, FLT_CLR, 0, 0);
      BGVAR(s64_SysNotOk) &=~(PFB_OFF_DATA_MISMATCH_MASK|PFB_OFF_CHKSUM_MASK|PFB_OFF_NO_DATA_MASK);
      BGVAR(s64_SysNotOk_2) &=~INVALID_GAIN_TABLE_DATA_MASK;
   }

   VeMaxFault(drive,mode);

   RunawayDetectionFault(drive, mode);

//   SchneiderFault(drive, mode);

   InternalErrorFault(drive,mode);

   MTPFault(drive, mode);

   DriveMotorMismatchFault(drive, mode);

   CanBusOffFault(drive, mode);

   CyclePowerNeededFault(drive, mode);

   SFBModeWarning(drive);

   SFBPositionDeviationFault(drive,mode);

   DefaultConfigurationWarning(drive);

   I2CTempSensReadWarning(drive);

   FdbkAutoDetectFault(drive, mode);

   MotorNameMTPMismatchFault(drive, mode);

   FanCircuitWarning(drive);

   //ExcessiveElectNoiseWarning(drive);

   MenczposMismatchFault(drive, mode);
   
   FbSyncFault(drive, mode);
}


void NotConfiguredFault(int drive)
{
   if (MaskedFault(drive, NO_COMP_FLT_MASK, 0)) return;

   // No Comp Fault
   if ((BGVAR(s64_SysNotOk) & NO_COMP_FLT_MASK) != 0)
   {
      if (HandleFault(drive, NO_COMP_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_NO_COMP;
         CANFaultControl(drive, 0, NO_COMP_FLT_MASK, 0);
      }
   }
}


void STOFault(int drive, int mode)
{
   static long u32_time_out = 0;
   unsigned int u16_FPGA_flt_indication = u16_STO_Flt_Indication;  // Capture the idication from the 1ms ISR
// if (MaskedFault(drive, STO_FAULT_MASK, 0))  return;

   // the below state machine is needed since STO circuit affect other circuits (like VBUS measure)
   // we do not want to change STO state from error to ok until other circuits are stabled.
   
   // ***************************************************************************************************************************
   // An exeption exists for the "fast STO enable" feature:
   //    This feature enables to clear the STO flt and enable the drive after 100-150ms instead of 800ms on drives with an AC loss detection mechanism.
   //    The state machine is handled as follows - first re-hook of STO will be handled normally. But from second re-hook:
   //       1. wait for OC indication from H/W to be cleared (around 10 ms)
   //       2. wait additional 20 ms and close inrush relay.
   //       3. wait additional 80ms and allow the drive to clr the STO fault and enable, but ignore vbus and IGBT tests.        
   //       4. after total of 800ms allow detection of vbus and IGBT.
   //
   //    u16_STO_Just_Inserted - when set, enables the sto fault CLR, but still denies inrush handler activity and vbus + igbt faults detections (for 700 msec)
   // ***************************************************************************************************************************

   if ((u16_FPGA_flt_indication) && (u16_Sto_state > STO_FAULT_STATE))
      u16_Sto_state = NO_STO_FAULT_STATE;  //at any point of the state machine, sto may be extracted, so we go back to fault state. this will make sure transition to fault state is done properly. 

   switch(u16_Sto_state)
   {
      case NO_STO_FAULT_STATE: // No STO fault state - check if fpga indication shows sto extract 
         if (u16_FPGA_flt_indication)
         {
            u16_STO_Just_Inserted = 0;
            u16_STO_Flt_Indication_Actual = 1;
            u16_Sto_state = STO_FAULT_STATE;
            u16_Fast_STO_En_Turn_Inrush_On = 0;
            BGVAR(s16_DisableInputs) |= INRUSH_INHIBIT_MASK;  // IPR 1563: When STO is not connected, there is no supply voltage to the inrush relay anyway,
                                                              // so indicate "inrush inhibit" in the disble inputs. This will clear SRDY.
         }
      break;

      case STO_FAULT_STATE: // STO fault state - check if fpga indication shows sto insert
         if (!u16_FPGA_flt_indication)
         {
            s32_Sto_Timer = Cntr_1mS;
            // if fast sto enable required and sto was not out during init
            if ((FAST_STO_ENABLE_ACTIVE) && (!u16_STO_Out_On_Init))
            {
               u16_Sto_state = STO_IN_WAIT_FOR_OC_CLR;
            }
            //if fast sto is not supported or this is the first sto insert after extract
            else  
            {
               u32_time_out = 800UL;
               u16_Sto_state = STO_IN_CLR_STO_FLT;
            }
         }
      break;

      case STO_IN_WAIT_FOR_OC_CLR:
         //verify fpga indication for OC is cleared, and wait additional 20ms
         if (0 < ((*(unsigned int *)(FPGA_FAULT_MANAGEMENT_STATUS_REG_ADD)) & PWR_AX0OCn))
         {
            s32_Sto_Timer = Cntr_1mS;
            u32_time_out = 20UL;
            u16_Sto_state = STO_IN_CLOSE_RELAY;
         }
      break;

      case STO_IN_CLOSE_RELAY:
         //if fpga indication shows again sto extract - raise flt
         if (PassedTimeMS(u32_time_out, s32_Sto_Timer))
         {
            u16_Fast_STO_En_Turn_Inrush_On = 1;
            u32_time_out = 80UL;
            s32_Sto_Timer = Cntr_1mS;
            u16_Sto_state = STO_IN_ALLOW_FAST_ENABLE;
         }
      break;

      case STO_IN_ALLOW_FAST_ENABLE:
         //if 80ms passed, allow fast sto enable by seting u16_STO_just_returned
         //and wait additional 700 ms before detection of related flts (vbus, IGBT, inrush..)
         if (PassedTimeMS(u32_time_out, s32_Sto_Timer))
         {
            u16_STO_Just_Inserted = 1;
            u32_time_out = 700;
            s32_Sto_Timer = Cntr_1mS;
            u16_Sto_state = STO_IN_CLR_STO_FLT;
         }
      break;

      case STO_IN_CLR_STO_FLT:
         //if time_out passed, allow flt clr and detection of ALL related flts (vbus, IGBT, inrush..)
         if (PassedTimeMS(u32_time_out, s32_Sto_Timer))
         {
            u16_Fast_STO_En_Turn_Inrush_On = 0;
            u16_STO_Just_Inserted = 0;
            u16_STO_Flt_Indication_Actual = 0;
            u16_Sto_state = NO_STO_FAULT_STATE;
         }
      break;
   }
   // Allow flt clr if no indication to sto extract from fpga AND u16_STO_just_returned is not set
   if (u16_STO_Flt_Indication_Actual && (!u16_STO_Just_Inserted))
   {
      u16_last_state_sto_fault = u16_STO_Flt_Indication_Actual;

      if ( (Enabled(DRIVE_PARAM)) ||
           (BGVAR(s16_DisableInputs) == INRUSH_INHIBIT_MASK) )  // if the disable is only due to inrush then generate STO fault, otherwise the
      {                                                         // drive will be enabled immediatly when connecting the STO (fix to bug 3285)
         BGVAR(u64_Sys_Warnings) &= ~STO_WRN_MASK;

         if (HandleFault(drive, STO_FAULT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_STO;
            CANFaultControl(drive, 0, STO_FAULT_MASK, 0);
         }
      }
      else if (!(BGVAR(s64_SysNotOk) & STO_FAULT_MASK))
      {
         BGVAR(u64_Sys_Warnings) |= STO_WRN_MASK;
      }
   }
   else
   {
      if (mode == FLT_CLR)
      {
         u16_STO_Out_On_Init = 0; //next time sto is inserted, fast enable feature will be enabled.
         BGVAR(s64_SysNotOk) &= ~STO_FAULT_MASK;
      }

      BGVAR(u64_Sys_Warnings) &= ~STO_WRN_MASK;

      if (u16_last_state_sto_fault == 1)
      {  //call inrush relay inhibition in order to not allow inrush relay close before FW handles it
         
         if (FAST_STO_ENABLE_ACTIVE)//if drive supports fast sto enable - raise only if BUS_AC flt raised
         {
            if (BGVAR(s64_SysNotOk_2) & BUS_AC_LOSS_MASK) u16_STO_Inrush_Inhibit_Enable = 1; // Inhibit enable
         }
         else
            u16_STO_Inrush_Inhibit_Enable = 1; // Inhibit enable
            
         u16_last_state_sto_fault = 0;
      }
   }
}


void SelfTestFault(int drive, int mode)
{
   if (MaskedFault(drive, SELF_TEST_FLT_MASK, 0)) return;

   if (mode == DETECT)
   {
      // Check if micro Blaze load failed
      // Or RAM test failed (if u32_Ram_Test_Result != 8)
         // RAM test error code when u32_Ram_Test_Result ==
         // if (0 <= u32_Ram_Test_Result <= 1) STACK area fault
         // if (2 <= u32_Ram_Test_Result <= 3) L0-L7 area fault
         // if (4 <= u32_Ram_Test_Result <= 5) H0-H1 area fault
         // if (6 <= u32_Ram_Test_Result <= 7) H2-H5 area fault available only for TMS320C28346 chip.
      // or setting ADC_Busy signal failed
      if ( ((u16_Load_Micro_Blaze_Exit_Code > 1) && (u16_Load_Micro_Blaze_Exit_Code < 90)) ||
           u16_Bad_Rotary_Switch                                                           ||
           (BGVAR(u32_Ram_Test_Result) != 0x08)                                            ||
           (u16_ADC_Stby_Counter >= 1000)                                                         )
      {
         if (HandleFault(drive, SELF_TEST_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_SELF_TEST;
            CANFaultControl(drive, 0, SELF_TEST_FLT_MASK, 0);
         }
      }
   }
}


void InternalErrorFault(int drive, int mode)
{
   if (MaskedFault(drive, INTERNAL_ERROR_FLT_MASK, 1)) return;

   if (mode == DETECT)
   {
      if (u16_Internal_OverFlow)
      {
         if (HandleFault(drive, INTERNAL_ERROR_FLT_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_INTERNAL_ERROR;
            CANFaultControl(drive, DETECT, INTERNAL_ERROR_FLT_MASK, 1);
         }
      }
   }
}

void EEpromFaults(int drive)
{
   if (!MaskedFault(drive, EEPROM_DIG_FLT_MASK, 0))
   {
      if (u16_Control_Board_Eeprom_Fault)
      {
         if (HandleFault(drive, EEPROM_DIG_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_EEPROM_DIG;
            CANFaultControl(drive, 0, EEPROM_DIG_FLT_MASK, 0);
         }
      }
   }

   if (!MaskedFault(drive, EEPROM_POW_FLT_MASK, 0))
   {
      if (BGVAR(u16_Power_Board_Eeprom_Fault))
      {

         if (HandleFault(drive, EEPROM_POW_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_EPROM_POW;
            CANFaultControl(drive, 0, EEPROM_POW_FLT_MASK, 0);
         }
      }
   }
}

// Handles faults detected by the FPGA
void GenericFPGADetectedFault(int drive, int mode, unsigned long long u64_flt, unsigned int u16_fpga_bit)
{
   unsigned int u16_FPGA_flt_indication = 0;

   if (MaskedFault(drive, u64_flt, 0))  return;

   u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
   u16_FPGA_flt_indication &= u16_fpga_bit;

   // Clear the fault on FPGA
   *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_flt_indication;

   // In case the power stage does not support regen OC (another functionality is assigned to this signal), clear the fault indication
   if ( ((BGVAR(u16_Power_Hw_Features) & 0x0020) != 0) && (u64_flt == REGEN_OC_FLT_MASK) )
   {
      u16_FPGA_flt_indication = 0;
   }

   u16_FPGA_flt_indication = (u16_FPGA_flt_indication > 0); // Translate to 0/1

   // In case of DDHD, get the CAN supply fault indication from GPIO11
   if ((u16_Product == DDHD) && (u64_flt == CAN_SUPPLY_FLT_MASK))
   {
      u16_FPGA_flt_indication = 0;
      if (GpioDataRegs.GPADAT.bit.GPIO11 == 1)
         u16_FPGA_flt_indication = 1;
   }

   if (u16_FPGA_flt_indication)
   {
      if (HandleFault(drive, u64_flt, 0))
      {
         switch (u64_flt)
         {
            case CAN_SUPPLY_FLT_MASK:
               p402_error_code = ERRCODE_CAN_CAN_SUPPLY;
               break;
            case REGEN_OC_FLT_MASK:
               p402_error_code = ERRCODE_CAN_REGEN_OC;
               break;
            case DRIVE_5V_FLT_MASK:
               p402_error_code = ERRCODE_CAN_DRIVE_5V;
               break;
         }
         CANFaultControl(drive, 0, u64_flt, 0);
      }

   }
   else if (mode == FLT_CLR) BGVAR(s64_SysNotOk) &= ~u64_flt;
}


void BrakeFault(int drive, int mode)
{
   static long s32_delay_timer = 0;
   static unsigned int u16_delay_state = 0, u16_prev_brake_on = 0;
   // AXIS_OFF;

   if (u16_Power_Brake_Exist) // if power switch exists
   {
      if ((BGVAR(u16_Power_Hw_Features) & 0x0010) == 0)            // 3A, 6A drives
      {
         // handle the faults normally
         PWRBrakeFault(mode, PWR_BRAKE_FAULT_MASK, drive);
      }
      else if ((BGVAR(u16_Power_Hw_Features) & 0x0010) > 0)      // 12A, 30A drives
      {
         switch (u16_delay_state)
         {
            case 0:
               if ((VAR(AX0_u16_BrakeOn) == 0) && u16_prev_brake_on) // if the brake has been just released (on enable)
               {  // take the delay timer and increment the state
                  s32_delay_timer = Cntr_1mS;
                  u16_delay_state++;
               }
               else// otherwise handle the fault normally
               {
                  PWRBrakeFault(mode, PWR_BRAKE_FAULT_MASK, drive);
               }
               break;

            case 1:
               // validate that 150ms passed since the brake has been released
               if (PassedTimeMS(150L, s32_delay_timer))
               {
                  PWRBrakeFault(mode, PWR_BRAKE_FAULT_MASK, drive);      // handle the fault

                  // initiallize variables
                  u16_delay_state = 0;
                  s32_delay_timer = 0;
               }
               else
               {
                  // for a large brake load, we might get a faulsy status bit indication
                  // clear the faulsy fault indication
                  if (*(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) & PWR_BRK_STAT)
                     *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) |= PWR_BRK_STAT;
               }
               break;

            default:
               break;
         }

         u16_prev_brake_on = VAR(AX0_u16_BrakeOn);         // take the previous brake engagement state
      }
   }
}


void PWRBrakeFault(int mode, unsigned long long u64_flt, int drive)
{
   int s16_det_flg = 0, s16_fault_set = 0;
   unsigned int u16_status_bit = 0;
   // AXIS_OFF;

   if (u64_flt & BGVAR(s64_Faults_Mask_2))   // identify faults set
   {
      s16_fault_set = 1;
   }

   if ( (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command)) ||// if burning command
        (u16_Out_Mode[PWR_BRAKE_OUT_NUM - 1] != BRAKE_OUT)               ||// if brake output is not configured
         ((BGVAR(u16_Ignore_Power_Brake_Fault) & 0x0001) == 0x0001)         )// if the fault indication is ignored (Bit#0 in IGNOREPWRBRK)
   {
      if (s16_fault_set == 1)
         BGVAR(s64_SysNotOk_2) &= ~(u64_flt);
      else
         BGVAR(s64_SysNotOk) &= ~(u64_flt);

      return;
   }

   if (MaskedFault(drive, u64_flt, s16_fault_set))
      return;

   if ((u64_flt & (PWR_BRK_FAULTS_MASK | PWR_BRAKE_FAULT_MASK)) == 0)
      return;

   u16_status_bit = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD); // Get the power switch status signal

   u16_status_bit &= PWR_BRK_STAT;

   // Clear the fault on FPGA
   *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) |= PWR_BRK_STAT;

   u16_status_bit = (u16_status_bit > 0); // Translate to 0/1

   switch (u64_flt)
   {
      case PWR_BRAKE_FAULT_MASK:
         if (u16_status_bit == 1)// status bit is HIGH --> generic brake fault
         {
            s16_det_flg = 1;
         }
         break;

      default:
         break;
   }

   if (s16_det_flg == 1)
   {
      if (u64_flt & PWR_BRAKE_FAULT_MASK)
      {
         p402_error_code = ERRCODE_CAN_PWR_BRK_FAULT;
      }
      else
      {
         (u64_flt & PWR_BRK_OPEN_LOAD_FLT_MASK) ? (p402_error_code = ERRCODE_CAN_PWR_BRK_OPEN_LOAD) : (p402_error_code = ERRCODE_CAN_PWR_BRK_SHORT);
      }

      if (HandleFault(drive, u64_flt, s16_fault_set))
      {
         CANFaultControl(drive, mode, u64_flt, s16_fault_set);
      }
   }
   else if (mode == FLT_CLR)
   {
      if (s16_fault_set == 1)
         BGVAR(s64_SysNotOk_2) &= ~(PWR_BRAKE_FAULT_MASK);
      else
         BGVAR(s64_SysNotOk) &= ~(u64_flt);

      CANFaultControl(drive, mode, u64_flt, s16_fault_set);
   }

   return;
}

void CANSupplyFault(int drive, int mode)
{
   // AXIS_OFF;

   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~CAN_SUPPLY_FLT_MASK;
      return;
   }

   if (IS_HW_FUNC_ENABLED(CAN_TRANSCEIVER_MASK))
      GenericFPGADetectedFault(drive, mode, CAN_SUPPLY_FLT_MASK, CAN_V_FAULT);
}


void RegenOCFault(int drive, int mode)
{
   // On LV drive (or any other DC drive) this fault is not relevant
   if (!DC_DRIVE)
      GenericFPGADetectedFault(drive, mode, REGEN_OC_FLT_MASK, PWR_RGN_FAULTn);
}


void RegenOverloadFault(int drive, int mode)
{
   if (MaskedFault(drive, REGEN_OVER_LOAD_FLT_MASK, 1)) return;

   if (mode == DETECT)
   {
      if ( (u16_Regen_Overload_Condition > 0) && (BGVAR(u16_Regen_Flt_Mode) == 1) )
      {
         if (HandleFault(drive, REGEN_OVER_LOAD_FLT_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_REGEN_OVER_LOAD;
            CANFaultControl(drive, DETECT, REGEN_OVER_LOAD_FLT_MASK, 1);
         }
      }
   }
   else
   {
      if ( (u16_Regen_Overload_Condition == 0) || (BGVAR(u16_Regen_Flt_Mode) == 0) )
      {
         BGVAR(s64_SysNotOk_2) &= ~REGEN_OVER_LOAD_FLT_MASK;
      }
   }

   // Handle regen overload warning.
   if ( (u16_Regen_Overload_Condition > 0) && (BGVAR(u16_Regen_Flt_Mode) == 0) && (!(BGVAR(s64_SysNotOk_2) & REGEN_OVER_LOAD_FLT_MASK)) )
   {
      // Generate a warning
      BGVAR(u64_Sys_Warnings) |= REGEN_OVER_LOAD_WRN_MASK;
   }
   else
   {
      // Clear the warning
      BGVAR(u64_Sys_Warnings) &= ~REGEN_OVER_LOAD_WRN_MASK;
   }


}

void FPGAFault(int drive)
{
   if (!u8_FPGA_ok)
   {
      if (HandleFault(drive, FPGA_CONFIG_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_FPGA_CONFIG;
         CANFaultControl(drive, 0, FPGA_CONFIG_FLT_MASK, 0);
      }
   }

   if (u16_FPGA_ver_mismatch)
   {
      if (HandleFault(drive, FPGA_VER_MISMATCH_FLT_MASK, 0))
     {
         p402_error_code = ERRCODE_CAN_FPGA_VER_MISMATCH;
         CANFaultControl(drive, 0, FPGA_VER_MISMATCH_FLT_MASK, 0);
     }
   }
}

void UBLAZEVersionMismatchFault(int drive)
{
   if (u16_UBLAZE_ver_mismatch)
   {
     if (HandleFault(drive, UBLAZE_VER_MISMATCH_FLT_MASK, 1))
     {
         p402_error_code = ERRCODE_CAN_UBLAZE_VER_MISMATCH;
         CANFaultControl(drive, 0, UBLAZE_VER_MISMATCH_FLT_MASK, 1);
     }
   }
}

void ESIVersionMismatchFault(int drive)
{
   if (u16_ESI_ver_mismatch)
   {
     if (HandleFault(drive, ESI_VER_MISMATCH_FLT_MASK, 1))
     {
         p402_error_code = ERRCODE_CAN_ESI_VER_MISMATCH;
         CANFaultControl(drive, 0, ESI_VER_MISMATCH_FLT_MASK, 1);
     }
   }
}


void ESIVendorMismatchFault(int drive)
{
   if (u16_ESI_vendor_mismatch)
   {
     if (HandleFault(drive, ESI_VENDOR_MISMATCH_FLT_MASK, 1))
     {
         p402_error_code = ERRCODE_CAN_ESI_VENDOR_MISMATCH;
         CANFaultControl(drive, 0, ESI_VENDOR_MISMATCH_FLT_MASK, 1);
     }
   }
}


// Special treatment - allow 3 times to clear the fault with delay of 100ms between tries,
// after 3 tries wait for 1 minute
void OverCurrentFault(int drive, int mode)
{
   static unsigned int clr_flt_try = 0, clr_flt_allow = 0;
   static long clr_flt_timer = 0L, clr_flt_delay_time = 100L, flt_filtering_timer = 0L;
   unsigned int u16_FPGA_latched_flt_indication = 0;
   unsigned int u16_FPGA_momentary_flt_indication = 0;

   if (MaskedFault(drive, OVER_CRRNT_FLT_MASK, 0))  return;

   u16_FPGA_latched_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
   u16_FPGA_momentary_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_STATUS_REG_ADD);
   u16_FPGA_latched_flt_indication &= PWR_AX0OCn;
   u16_FPGA_momentary_flt_indication &= PWR_AX0OCn;

   // Clear the fault on FPGA
   if ((mode == FLT_CLR) && clr_flt_allow && (!u16_FPGA_momentary_flt_indication))
   {
      clr_flt_try++;
      if (clr_flt_try >= 3)
      {
         clr_flt_try = 0;
         clr_flt_delay_time = 60000L;
      }
      BGVAR(s64_SysNotOk) &= ~OVER_CRRNT_FLT_MASK;
   }

   if (!Enabled(drive)) // try to clear FPGA latched indication only if the drive is disabled
      *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_latched_flt_indication;

   u16_FPGA_latched_flt_indication = (u16_FPGA_latched_flt_indication > 0); // Translate to 0/1

   // ignore the over current indication when there is no STO
   // OC indication (not real OC) may occur up to approx. 1.5 ms before STO changed to not exists (STO warning or fault).
   if (mode == DETECT)
   {
      if ( (u16_FPGA_latched_flt_indication)           &&
           (u16_STO_Over_Current_Flag == 0)            &&
           (!(BGVAR(u64_Sys_Warnings) & STO_WRN_MASK)) &&
           (!(BGVAR(s64_SysNotOk) & STO_FAULT_MASK))     )
      {
         u16_STO_Over_Current_Flag = 1;
         flt_filtering_timer = (unsigned long)Cntr_1mS;
      }

      if ( (u16_STO_Over_Current_Flag == 1)        && // Check STO state 16mSec after over-
           (PassedTimeMS(16L, flt_filtering_timer))  ) // -current indication was received
      {
         if ( (!(BGVAR(u64_Sys_Warnings) & STO_WRN_MASK)) &&
              (!(BGVAR(s64_SysNotOk) & STO_FAULT_MASK))     )
         { // STO OK, means real OC fault
            clr_flt_timer = Cntr_1mS;
            clr_flt_allow = 0;
            if (HandleFault(drive, OVER_CRRNT_FLT_MASK, 0))
            {
               p402_error_code = ERRCODE_CAN_OVER_CRRNT;
               CANFaultControl(drive, 0, OVER_CRRNT_FLT_MASK, 0);
            }
         }  // ignore the over current indication when there is no STO - end

         u16_STO_Over_Current_Flag = 0;
      }
   }

   if (PassedTimeMS(clr_flt_delay_time, clr_flt_timer)) clr_flt_allow = 1;
   if (PassedTimeMS(60000L, clr_flt_timer))
   {
      clr_flt_try = 0;
      clr_flt_delay_time = 100L;
   }
}


void UnStableCurrLoopFault(int drive, int mode)
{
   // AXIS_OFF;

   if (mode == DETECT)
   {
      if (VAR(AX0_u16_Unstable_CL_Cntr) == 10)
      {
         if (HandleFault(drive, UNSTABLE_CL_FLT_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_UNSTABLE_CL;
            CANFaultControl(drive, DETECT, UNSTABLE_CL_FLT_MASK, 1);
         }
      }
   }
   else if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~UNSTABLE_CL_FLT_MASK;
      // Clean counter
      VAR(AX0_u16_Unstable_CL_Cntr) = 0;
   }

}


void HighIqFault(int drive, int mode)
{
   // AXIS_OFF;

   if (MaskedFault(drive, HIGH_IQ_FLT_MASK, 1))  
       return;   
   
   if (mode == DETECT)
   {
      // Counter become equal to 4 if Iq is greater than 120% of Ilim more than 1 mS
      // Skip fault if 120% of ILIM is less than 20% of DIPEAK -> DIPEAK=26214 -> 26214*0.2 = 5243
      if (VAR(AX0_s16_High_Iq_Counter) == 4)
      {
         HandleFault(drive, HIGH_IQ_FLT_MASK, 1);
      }
   }
   else if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~HIGH_IQ_FLT_MASK;
      // Clean counter
      VAR(AX0_s16_High_Iq_Counter) = 0;
   }
}


//**********************************************************
// Function Name: ResetFaults
// Description:
//      Reset Faults is the mechanism by which faults are reset.
//      In order to reset faults, a transition from Disable to Enable must be
//      detected. This function enables this transition to occur on either the
//      Remote Enable line or on the Software (User) Enable signal.
//      if sysnotok cleared without dis/en (no-comp) - reset state machine
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
void ResetFaults(int drive)
{
   /* if no faults - no need to reset */
   if ((BGVAR(s16_DisableInputs) & FLT_EXISTS_MASK) == 0)
   {
      BGVAR(s16_HwEnableResetFaultsFlag) = 0;
      BGVAR(s16_SwEnableResetFaultsFlag) = 0;
      return;
   }

   /* if HW enables is off than get ready to clr faults */
   if ( ((BGVAR(s16_DisableInputs) & HW_EN_MASK) != 0) &&
        (BGVAR(s16_HwEnableResetFaultsFlag) == 0)        )
   {
      BGVAR(s16_HwEnableResetFaultsFlag) = 1;
      return;
   }

   /* if SW enables is off than get ready to clr faults */
   if ( ((BGVAR(s16_DisableInputs) & SW_EN_MASK) != 0) &&
        (BGVAR(s16_SwEnableResetFaultsFlag) == 0)        )
   {
      BGVAR(s16_SwEnableResetFaultsFlag) = 1;
      return;
   }

   if ( ((BGVAR(s16_DisableInputs) & HW_EN_MASK) == 0) &&
        (BGVAR(s16_HwEnableResetFaultsFlag) == 1)        )
   {
      FaultsHandler(drive, FLT_CLR);
      BGVAR(s16_HwEnableResetFaultsFlag) = 0;
      return;
   }

   if ( ((BGVAR(s16_DisableInputs) & SW_EN_MASK) == 0) &&
        (BGVAR(s16_SwEnableResetFaultsFlag) == 1)        )
   {
      FaultsHandler(drive, FLT_CLR);
      BGVAR(s16_SwEnableResetFaultsFlag) = 0;
      return;
   }
}


//**********************************************************
// Function Name: FaultControl
// Description:
//   Fault control
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
void FaultControl(int drive)
{
   FaultsHandler(drive, DETECT);
   WarningsHandler(drive);
   RemarksHandler(drive);
   WriteFaultLogToNvram(DRIVE_PARAM);
   ResetFaults(DRIVE_PARAM);
}


//**********************************************************
// Function Name: CANFaultControl
// Description:
//   CAN Fault control
//
// Author: Itai
// Algorithm:
// Revisions: Nitsan: add function in parameters u64_fault_mask and s16_fault_set to send fault string in EMCY for LXM28 drive.
//**********************************************************
void CANFaultControl(int drive, int mode, unsigned long long u64_fault_mask, int s16_fault_set)
{
   unsigned char u16_tmp_error_reg = 0;
   UNSIGNED8 manuFltCode[] = {0,0,0,0,0};

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( (mode == FLT_CLR) || ((BGVAR(s64_SysNotOk) == 0) && (BGVAR(s64_SysNotOk_2) == 0)) )
   {
      u16_tmp_error_reg= p301_error_register & 0xfffe;
      if (!(BGVAR(s64_SysNotOk) & CURRENT_FAULTS_MASK))   //current fault
         u16_tmp_error_reg &= ~0x0002;
      if (!(BGVAR(s64_SysNotOk) & VOLTAGE_FAULTS_MASK))       //voltage fault
         u16_tmp_error_reg &= ~0x0004;
      if ( (!(BGVAR(s64_SysNotOk) & TEMPERATURE_FAULTS_MASK)) && (!(BGVAR(s64_SysNotOk_2) & TEMPERATURE_HW_FLT_MASK)) )  //temperature fault
         u16_tmp_error_reg &= ~0x0008;
      if (!(BGVAR(s64_SysNotOk) & COMMUNICATION_FAULTS_MASK) && !(BGVAR(s64_SysNotOk_2) & COMMUNICATION_FAULTS_MASK_2))   //communication fault
         u16_tmp_error_reg &= ~0x0010;
      if (!(BGVAR(s64_SysNotOk) & DEVICE_SPECIFIC_FAULTS_MASK))   // device faults
         u16_tmp_error_reg &= ~0x0020;
         // reserved 0x0040
      if (!(BGVAR(s64_SysNotOk) & MANUFACTURER_SPECIFIC_FAULTS_MASK))   // manufacturer fault
         u16_tmp_error_reg &= ~0x0080;
      u16_tmp_error_reg |= (u16_tmp_error_reg)?0x1:0x0;
      p301_error_register = u16_tmp_error_reg;
      return;
   }
   u16_tmp_error_reg |= 0x0001;

   if (BGVAR(s64_SysNotOk) & CURRENT_FAULTS_MASK)   //current fault
      u16_tmp_error_reg |= 0x0002;
   if (BGVAR(s64_SysNotOk) & VOLTAGE_FAULTS_MASK)   //voltage fault
      u16_tmp_error_reg |= 0x0004;

   if ( (BGVAR(s64_SysNotOk) & TEMPERATURE_FAULTS_MASK) || (BGVAR(s64_SysNotOk_2) & TEMPERATURE_HW_FLT_MASK) )  //temperature fault
      u16_tmp_error_reg |= 0x0008;

   if ((BGVAR(s64_SysNotOk) & COMMUNICATION_FAULTS_MASK) || (BGVAR(s64_SysNotOk_2) & COMMUNICATION_FAULTS_MASK_2))   //communication fault
      u16_tmp_error_reg |= 0x0010;

   if (BGVAR(s64_SysNotOk) & DEVICE_SPECIFIC_FAULTS_MASK)   // device faults
      u16_tmp_error_reg |= 0x0020;

   // reserved 0x0040

   if (BGVAR(s64_SysNotOk) & MANUFACTURER_SPECIFIC_FAULTS_MASK)   // manufacturer fault
      u16_tmp_error_reg |= 0x0080;

   p301_error_register = u16_tmp_error_reg;

   if (IS_CAN_DRIVE_AND_COMMODE_1)
   {
      if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      {
         GetEmcyManufcturerCode(drive, u64_fault_mask, s16_fault_set, manuFltCode);
      }

      writeEmcyReq(BACKGROUND_CONTEXT, p402_error_code, manuFltCode CO_COMMA_LINE_PARA);
   }
   else if (IS_EC_DRIVE_AND_COMMODE_1 && BGVAR(u16_Emcy_Mode))
   {
      //If we send EMCY message in a lower NMT state than SAFEOP (PREOP) we get uBLAZE crush by losing its interrupts.
      //We do not know why uBLAZE crush - In the future we need understand why it crush.
      SET_DPRAM_FPGA_MUX_REGISTER(2);

      //pass error code to uB
      *(int*)p_u16_bg_tx_emcy_register_index = p402_error_code;
      SET_DPRAM_FPGA_MUX_REGISTER(0);
   }
}


//**********************************************************
// Function Name: HandleFault
// Description:
//   This function handles fault
//   note:
//   s16_fault_set=0 refers to a fault bit in s64_SysNotOk
//   s16_fault_set=1 refers to a fault bit in s64_SysNotOk_2
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int HandleFault(int drive, long long s64_fault_mask, int s16_fault_set)
{
   int return_value = 0;

   if (s16_fault_set == 0)
   {
      if ( (BGVAR(s64_SysNotOk) & s64_fault_mask) == 0)
      {
         BGVAR(s64_SysNotOk) |= s64_fault_mask;

         // 5V fault happens on every power off. Log it after delay, since if the drive is still running the fault is real
         if (s64_fault_mask == DRIVE_5V_FLT_MASK)
         {
            BGVAR(u16_Delayed_5V_Log_Fault_Flag) = 1;
            BGVAR(s32_Delayed_5V_Log_Fault_Timer) = Cntr_1mS;
         }
         else
         {
            WriteToFaultLog(drive, GetFaultId((unsigned long long)s64_fault_mask, s16_fault_set));
         }
         return_value = 1;
      }
   }
   else
   {
      if ( (BGVAR(s64_SysNotOk_2) & s64_fault_mask) == 0)
      {
         BGVAR(s64_SysNotOk_2) |= s64_fault_mask;

         // AC loss fault may happen on every power off. Log it after delay, since if the drive is still running the fault is real
         if (s64_fault_mask == BUS_AC_LOSS_MASK)
         {
            BGVAR(u16_Delayed_Line_Loss_Log_Fault_Flag) = 1;
            BGVAR(s32_Delayed_Line_Loss_Log_Fault_Timer) = Cntr_1mS;
         }
         else if (s64_fault_mask == LOGIC_AC_LOSS_MASK)
         {
            BGVAR(u16_Delayed_Logic_AC_Loss_Log_Fault_Flag) = 1;
            BGVAR(s32_Delayed_Logic_AC_Loss_Log_Fault_Timer) = Cntr_1mS;
         }
         else
         {
            WriteToFaultLog(drive, GetFaultId((unsigned long long)s64_fault_mask, s16_fault_set));
         }

         return_value = 1;
      }
   }

   return return_value;
}


//**********************************************************
// Function Name: WriteToFaultLog
// Description:
//   This function add fault and updates new fault status
//
// Author: Yuval & Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void WriteToFaultLog(int drive, unsigned int fault_id)
{
   /* update most recent fault and raise u8_New_Fault flag so it will be reported   */

   BGVAR(s16_Most_Recent_Fault) = fault_id;
   BGVAR(u8_New_Fault) = 0;

   BGVAR(u16_Fault_Log_Ptr)++;   // Increment cyclic pointer
   if (BGVAR(u16_Fault_Log_Ptr) >= FAULT_LOG_LEN) BGVAR(u16_Fault_Log_Ptr) = 0;

   LogFault(fault_id, BGVAR(u16_Fault_Log_Ptr), drive);   // Write fault to the LOG
}


//**********************************************************
// Function Name: LogFault
// Description:
//   This function add fault to fault log
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void LogFault(int id, int pos, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s_Fault_Log_Image)[pos].fault_id = id;
   BGVAR(s_Fault_Log_Image)[pos].hours = u16_RunTimeHours;
   BGVAR(s_Fault_Log_Image)[pos].minutes = u16_RunTimeMins;
   BGVAR(s_Fault_Log_Image)[pos].seconds = u16_RunTimeSecs;

   //Power up marking
   if (BGVAR(u8_First_Fault_After_Power_Up))
   {
       BGVAR(s_Fault_Log_Image)[pos].first_fault_after_power_up = 1;
       BGVAR(u8_First_Fault_After_Power_Up) = 0;
   }
}


//**********************************************************
// Function Name: WriteFaultLogToNvram
// Description:
//              Writes fault log from the RAM to the EEPROM
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void WriteFaultLogToNvram(int drive)
{
   int status;

   //Check if the fault log is updated
   if (BGVAR(u16_Flash_Fault) != BGVAR(u16_Fault_Log_Ptr))
   {
      BGVAR(u16_Flash_Fault)++;      //Increment ptr
      if (BGVAR(u16_Flash_Fault) > FAULT_LOG_LEN - 1) BGVAR(u16_Flash_Fault) = 0;

      status = FlashWriteFault(drive, BGVAR(u16_Flash_Fault));      //Write fault to Flash
      if (status != SAL_SUCCESS)
      {
         if (BGVAR(u16_Flash_Fault) == 0)
            BGVAR(u16_Flash_Fault) = FAULT_LOG_LEN - 1;
         else
            BGVAR(u16_Flash_Fault)--;
      }
   }
}


char *GetFaultMsgPtr(int fault_id)
{
   if (fault_id > 127) return (char*)u16_Unknown_Fault_Message;
   // return fault general message
   return (char*)u16_Fault_Message[fault_id];
}


//**********************************************************
// Function Name: ReadMostRecentFault
// Description:
//   This function reads most reasent fault
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
char* ReadMostRecentFault(int drive, int *fault_code)
{
   char *fault_string;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   fault_string = GetFaultMsgPtr(BGVAR(s16_Most_Recent_Fault));

   *fault_code = BGVAR(s16_Most_Recent_Fault);

   return( fault_string );
}


//**********************************************************
// Function Name: GetFaultId
// Description:
//   This function converts fault mask to fault id , by calculating the position
//   of "1" in fault mask.
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
unsigned int GetFaultId(unsigned long long u64_fault_mask, int s16_fault_set)
{
   unsigned int i = 0;

   while (u64_fault_mask != 0)
   {
      u64_fault_mask >>= 1;
      i++;
   }

   if (s16_fault_set == 1) i += 64;

   return (i);
}


//**********************************************************
// Function Name: InitFaultLog
// Description:
//   This function initialize fault log and his variables
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void InitFaultLog(int drive)
{
   BGVAR(s16_Most_Recent_Fault) = 0;
   BGVAR(u8_New_Fault) = 0;
   LoadFaultLogFromFlash(drive);
}


//**********************************************************
// Function Name: PrintFaultLog
// Description:
//   This function prints fault log to serail port
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int PrintFaultLog(int drive)
{
   static int ind = 0, print_state = 0, flt_cnt;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // As we removed FLTHISTCLR command, this hidden feature will allow us to clear the fault log on produciton line
   // Use password and then FLTHIST 1234 to clear the fault history
   if (s16_Number_Of_Parameters == 1)
   {
       if (s64_Execution_Parameter[0] == 1234)
       {
         if (!s16_Password_Flag) return (PASSWORD_PROTECTED);
         return ClearFlashFaultLog(drive);
       }
       else return NOT_PROGRAMMABLE;
   }
   
   if (u8_Output_Buffer_Free_Space < 60) return SAL_NOT_FINISHED;

   switch(print_state)
   {
      case 0:
         ind = BGVAR(u16_Fault_Log_Ptr);

         if (BGVAR(s_Fault_Log_Image)[ind].fault_id == 0xFF)
         { // No faults
            PrintString("No Fault History", 0);
            PrintCrLf();
            return SAL_SUCCESS;
         }

         print_state++;
         flt_cnt = 0;
      break;

      case 1:
         if ((flt_cnt < FAULT_LOG_LEN)                                  &&
             ((BGVAR(s_Fault_Log_Image)[ind].fault_id & 0x00FF) != 0xFF)  )
         {
            PrintUnsignedInteger(BGVAR(s_Fault_Log_Image)[ind].hours);
            PrintChar(':');
            if (BGVAR(s_Fault_Log_Image)[ind].minutes < 10) PrintChar('0');
            PrintUnsignedInteger(BGVAR(s_Fault_Log_Image)[ind].minutes);
            PrintChar(':');
            if (BGVAR(s_Fault_Log_Image)[ind].seconds < 10) PrintChar('0');
            PrintUnsignedInteger(BGVAR(s_Fault_Log_Image)[ind].seconds);
            PrintChar(' ');

            PrintString("FLT ", 0);
            PrintUnsignedInteger(BGVAR(s_Fault_Log_Image)[ind].fault_id);
            PrintString("  ", 0);

            PrintString(GetFaultMsgPtr(BGVAR(s_Fault_Log_Image)[ind].fault_id), 0);

            //Bug#5248: [CDHD & SE]: Fault history - Power up marking
            if (BGVAR(s_Fault_Log_Image)[ind].first_fault_after_power_up)
                 PrintChar('*');

            PrintCrLf();

            flt_cnt++;
            ind--;
            if (ind<0) ind = FAULT_LOG_LEN - 1;
         }
         else
         {
            print_state = 0;
            return SAL_SUCCESS;
         }
      break;
   }
   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: UnderVoltageTest
// Description:
//   Look for under voltage fault. Set the fault accordingly
//   UVMODE,UVTIME parameters
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int UnderVoltageTest(int drive)
{
   int uv_fault = 0, return_value = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( (BGVAR(u16_Vbus_Volts) < BGVAR(u16_UV_Threshold)) && (!(BGVAR(u64_Sys_Warnings) & STO_WRN_MASK)) &&
        (!(BGVAR(s64_SysNotOk) & STO_FAULT_MASK))                                                          )
   {
      if (BGVAR(s16_UvFilter) < 10) ++BGVAR(s16_UvFilter);
      else if (BGVAR(s16_UvFilter) == 10) uv_fault = 1;
   }
   else
   {
      BGVAR(s16_UvFilter) = 0;
      uv_fault = 0;
      BGVAR(s32_Uv_Mode2_Timer) = Cntr_1mS;
   }

   if (uv_fault == 0) return return_value;

   switch( BGVAR(u16_Uv_Mode) )
   {
      case 0:  // latch fault immediatly
         return_value = 1;
      break;

      case 1:  // show warning only
         if ( Enabled(DRIVE_PARAM) ) // when disable, do not dispaly warning
            return_value = 2;
      break;

      case 2:  // show warning, after UVTIME latch fault
         if (Enabled(DRIVE_PARAM))   // when disable, do not fault or dispaly warning
         {
            if (PassedTimeMS((long)BGVAR(u16_Uv_Time) * 1000L, BGVAR(s32_Uv_Mode2_Timer)))
               return_value = 1;
            else
               return_value = 2;
         }
      break;

      case 3: // fault only if UV exsists + drive is enabled, Warning if Disabled
         if (Enabled(DRIVE_PARAM)) return_value = 1;
         else return_value = 2;
      break;
   }
   return( return_value );
}



//**********************************************************
// Function Name: FeedbackFailure
// Description:
//   Detects Feedback faults
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void FeedbackFailure(int drive, int mode)
{
   // AXIS_OFF;

   // do not test feedback faults when in burnin or motor w/o feedback
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command) ||
      (VAR(AX0_u16_Motor_Comm_Type) ==  DC_VCM_MOTOR_WO_FDBK)           )
   {
      BGVAR(s64_SysNotOk) &= ~FEEDBACK_LOSS_FLT_MASK; // Avoid all Feedback-related Faults
      BGVAR(s64_SysNotOk_2) &= ~FEEDBACK_LOSS_FLT_2_MASK;
      BGVAR(u64_Sys_Warnings) &= ~PHASE_FIND_REQ_WRN_MASK;
      return;
   }

   BGVAR(u16_Encoder_Off) = ( Encoder5VFault(drive, mode) || !GetFeedback5VoltStatus() );
   // For all Feedback Devices, ignore Faults if 5VDC to Encoder is not On or in Fault.

   BGVAR(u16_Tm_Enc_Status) = (VAR(AX0_u16_Tamagawa_Timer) == 0xFFFF) || (VAR(AX0_s16_Motor_Enc_Type) != 11) ||
                              (BGVAR(u16_FdbkType) != INC_ENC_FDBK);
   // On Tamagawa - ignore Index break, illegal halls and Line break till ABI is transmitted
   // by the feedback.  Detect errors if Feedback Type is not Incremental Encoder

   HallsFault(drive, mode);         // HALLS sensors

   DiffHallsFault(drive, mode);     // Differential HALLS sensors

   LineBreakFault(drive, mode);     // Line break

   IndexBreakFault(drive, mode);    // Index break

   TamagawaInitFault(drive, mode);  // Tamagawa Init Fault

   MotorOverTempFault(drive, mode); // Motor over temperature

   EncoderPhaseError(drive, mode);  // Incremental encoder phase error

   CommutationFault(drive, mode);  // Commutation fault
/*
   if ((!BGVAR(u16_Encoder_Off)) || (mode == FLT_CLR))   // Check next faults only if encoder is on
   {
      if (BGVAR(u16_Tm_Enc_Status))
      {
      }
      PhaseFindFault(drive, mode);
   } */
   PhaseFindFault(drive, mode);
   if ( (VAR(AX0_u8_Gear_Mode) == 0) || (VAR(AX0_u8_Gear_Mode) == 1) || (VAR(AX0_u8_Gear_Mode) == 2) )
      BGVAR(u16_2nd_Encoder_Src) = 1; // Gear Source at Pulse&Dir Contacts of Control Connector C2
   else if ( (VAR(AX0_u8_Gear_Mode) == 3) || (VAR(AX0_u8_Gear_Mode) == 4) )
      BGVAR(u16_2nd_Encoder_Src) = 2; // Gear Source at 2nd Encoder Contacts of Machine Connector C3
   else BGVAR(u16_2nd_Encoder_Src) = 0; // ...to avoid spurious Faults.

   PulseAndDirLineBreakFault(drive, mode); // Pulse&Dir Line-Break Detection

   SecondaryIndexBreakFault(drive, mode); // Index break   
   
   SecondaryEncoder5VFault(drive, mode);

   SecondaryLineBreakFault(drive, mode); // Line break
/*
   if (VAR(AX0_s16_Opmode) == 4 || mode == FLT_CLR)   // Check next faults only if Gearing Input is active
   {
      // If Gearing is derived from Pulse&Dir Input at Controller I/F
      if ( (BGVAR(u16_2nd_Encoder_Src) == 1) || (mode == FLT_CLR) ) {};
      // Reach P&DLineBreakFlt only if Gearing is from P&D Input, or during FltClr

      // If Gearing is derived from Secondary Encoder Input at Machine I/F
      if ( (BGVAR(u16_2nd_Encoder_Src) == 2) || (mode == FLT_CLR) )
      { // Detect 2ndEnc5V only if Gearing is from 2ndEnc, or during FltClr
         BGVAR(u16_2nd_Encoder_Off) = SecondaryEncoder5VFault(drive, mode);

         if ( (!BGVAR(u16_2nd_Encoder_Off)) || (mode == FLT_CLR) )
         { // Detect 2ndEncLineBreakFlt only if 2ndEnc5V is OK
         }
      }
   } */

   AbOutOfRangeFault(drive, mode);   //A/B out of range

   if ( (VAR(AX0_s16_Motor_Enc_Type) == 9) || (mode == FLT_CLR) ||
        (BGVAR(u16_FdbkType) == ENDAT2X_COMM_FDBK)                )   // Endat fault
      EndatFault(drive, mode);

   if ((VAR(AX0_s16_Motor_Enc_Type) == 10) || (mode == FLT_CLR))   // Hiperface fault
      HiperfaceFault(drive, mode);

   CommFdbkFault(drive, mode);         // Communication position fdbck fault handler

   SanyoEncoderFault(drive, mode);     // Sanyo Denky encoder fault handler

   AbsEncBattLowFault(drive, mode);    // Tamagawa17Bit Low-Battery fault handler

   TamagawaAbsEncoderFault(drive, mode);  // Tamagawa17Bit Encoder fault handler

   PanasonicS_EncoderFault(drive, mode);  // Panasonic Absolute Encoder Fault Handler

   InvalidSininitFault(drive, mode);   // Sine Amplitude Setting...

   ResolverInitFault(drive, mode);     // Resolver Adjust fault

   SwSineQuadFault(drive, mode);       // Sw sine quad mismatch

   SrvSnsFault(drive, mode);           // ServoSense Fault

   BiSSC_Fault(drive, mode, 0);           // BiSS-C Fault
}


//**********************************************************
// Function Name: HallsFault
// Description:
//   Detects HALLs faults
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void HallsFault(int drive, int mode)
{
   // AXIS_OFF;
   int temp_var = 0, temp_filter_max = 10;

   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if (MaskedFault(drive, ILLEGAL_HALLS_MASK, 0))  return;
   if ( (BGVAR(u16_Encoder_Off) || !BGVAR(u16_Tm_Enc_Status) ) && (mode != FLT_CLR) )  return;

   if (DETECT == mode)
   {
      // On Panasonic detect bad halls only after init is done
      if (FEEDBACK_PANASONIC_P_G && (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK)) return;

      // Check if Power_Up_Timer decremented to Zero, indicating inconsistent Halls
      if (FEEDBACK_WITH_HALLS && (VAR(AX0_s16_Power_Up_Timer) == 0))
      {
         temp_var = 1;
         temp_filter_max = 30;
      }

      if (20 <= BGVAR(u16_Halls_Invalid_Value))
      {
         temp_var = 1;
         temp_filter_max = 0;
      }
      else
        temp_var = 0;
   }

   // In case of Tamagawa and FLT_CLR, clear the illegal halls fault bit to allow reset of tamagawa state machine
   if (mode == FLT_CLR)
   {
      temp_var = 0;
      BGVAR(u16_Halls_Invalid_Value) = 0;
   }
   FeedbackFault(drive, mode, ILLEGAL_HALLS_MASK, (unsigned int *)(&u16_HallsFilter + drive), (unsigned int)u32_bg_interval_time, temp_filter_max, temp_var, 1, 0);
}


//**********************************************************
// Function Name: DiffHallsFault
// Description:
//   Detects differential HALLs faults
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
void DiffHallsFault(int drive, int mode)
{
   int temp_var;
   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if (MaskedFault(drive, DIFF_HALLS_LINE_BRK_MASK, 1)) return;

   if ( (BGVAR(u16_Encoder_Off) || !BGVAR(u16_Tm_Enc_Status) ) && (mode != FLT_CLR) )  return;

   // Check line break fault on differential halls
   temp_var = 0;
   if (BGVAR(u16_Halls_Type) == 3)
   {
      if (IS_DDHD_EC2_DRIVE)
      {
         temp_var = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
         temp_var &= RESOLVER_FAIL; // For DDHD2-EC only, route Differential-Halls Fault through Fault
                                    // Register at Bit-0 (usually Resolver-Fault).
         *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= temp_var;   // Clear the fault on FPGA
         temp_var = (temp_var != 0);
      }
      else
      {
         if (GpioDataRegs.GPADAT.bit.GPIO10 == 0)
            temp_var = 1;
      }
   }
   FeedbackFault(drive, mode, DIFF_HALLS_LINE_BRK_MASK, (unsigned int *)(&u16_Diff_Halls_Filter + drive), (unsigned int)u32_bg_interval_time, 30, temp_var, 1, 1);
}


void TamagawaInitFault(int drive, int mode)
{
   // AXIS_OFF;
   int temp_var = 0;

   if (MaskedFault(drive, TAMAGAWA_FDBK_FLT_MASK, 0)) return;

   if ( (BGVAR(u16_Encoder_Off) || !BGVAR(u16_Tm_Enc_Status) ) && (mode != FLT_CLR) )  return;

   if (mode == FLT_CLR) VAR(AX0_u16_Tamagawa_TO_Cntr) = 0;

   if (AX0_u16_Tamagawa_TO_Cntr > 1) temp_var = 1;

   FeedbackFault(drive, mode, TAMAGAWA_FDBK_FLT_MASK, NULL, 0, 0, temp_var, 1, 0);
}


//**********************************************************
// Function Name: Encoder5VFault
// Description:
//   Detects Main Encoder 5V fault
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
unsigned int Encoder5VFault(int drive, int mode)
{
   unsigned int u16_FPGA_latched_flt_indication = 0, u16_FPGA_momentary_flt_indication = 0;
   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;
   
   // AXIS_OFF;
   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   u16_FPGA_latched_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
   u16_FPGA_momentary_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_STATUS_REG_ADD);
   u16_FPGA_latched_flt_indication &= MFB_5V_OCn;
   u16_FPGA_momentary_flt_indication &= MFB_5V_OCn;

   *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_latched_flt_indication;//Clear latched

   u16_FPGA_latched_flt_indication = (u16_FPGA_latched_flt_indication > 0); // Translate to 0/1

   if (DETECT == mode)
   {
      if (!MaskedFault(drive, ENCODER_5V_OC_MASK, 0))//If fault is not masked, raise fault and ShutDown 5V Power to encoder
      {
         FeedbackFault(drive, mode, ENCODER_5V_OC_MASK, (unsigned int *)(&u16_Encoder5Vfilter + drive), (unsigned int)u32_bg_interval_time, 30, u16_FPGA_latched_flt_indication, 1, 0);
         if (0 != (BGVAR(s64_SysNotOk) & ENCODER_5V_OC_MASK))
         {  //Mechanism to Allow 3 first faults with minimal 100msec between CLR_FLTs --> 4th fault will result 60000msec break before CLR_FLT is enabled.
            if (u16_FPGA_latched_flt_indication && (!BGVAR(u16_Flt_5v_Was_Diagnosed)))
            {
               BGVAR(s32_Clr_5v_Flt_Timer) = Cntr_1mS;
               BGVAR(u16_Clr_5v_Flt_Allow) = 0;
               BGVAR(u16_Clr_5v_Flt_Try)++;
               BGVAR(u16_Flt_5v_Was_Diagnosed) = 1;
            }
            RemoveFeedback5Volt(); //ShutDown 5V Power to encoder
            VAR(AX0_u16_Encoder5v_Flt) = 1;
         }
         else
         {
            VAR(AX0_u16_Encoder5v_Flt) = 0;
         }
      }
      else //In Case Of No 5v Encoder Support (fault is masked), ShutDown Power but dont raise fault
      {
         if (u16_FPGA_latched_flt_indication && (10 > BGVAR(u16_Encoder5Vfilter))) //Handle Filters
            ++BGVAR(u16_Encoder5Vfilter);
         if ((!u16_FPGA_latched_flt_indication) && (0 < BGVAR(u16_Encoder5Vfilter)))
            --BGVAR(u16_Encoder5Vfilter);
         if (10 == BGVAR(u16_Encoder5Vfilter))
         {
            RemoveFeedback5Volt(); // ShutDown 5V Power to encoder
         }
      }
   }
   else //FLT_CLR mode
   {
      if (BGVAR(u16_Clr_5v_Flt_Allow) && (!u16_FPGA_momentary_flt_indication))
      {
         if (BGVAR(u16_Clr_5v_Flt_Try) >= 3)
         { // if fault occurs the 4th time in 1 minute - enable next CLR_FLT after 60000msec only
            BGVAR(u16_Clr_5v_Flt_Try) = 0;
            BGVAR(s32_5V_Fault_Cleared_Timer) = 60000L;
         }
         if (!MaskedFault(drive, ENCODER_5V_OC_MASK, 0))
         { // dont turn power back on if fault is masked (since this means no support required)
            FeedbackFault(drive, mode, ENCODER_5V_OC_MASK, (unsigned int *)(&u16_Encoder5Vfilter + drive), (unsigned int)u32_bg_interval_time, 30, u16_FPGA_latched_flt_indication, 1, 0);
            SetFeedback5Volt(); // turn back on 5V Power to encoder
         }
         BGVAR(u16_Flt_5v_Was_Diagnosed) = 0;
      }
   }

   if (PassedTimeMS(BGVAR(s32_5V_Fault_Cleared_Timer), BGVAR(s32_Clr_5v_Flt_Timer)))
      BGVAR(u16_Clr_5v_Flt_Allow) = 1; // Flt_timer is either 100 msec or 60000msec
   if (PassedTimeMS(60000L, BGVAR(s32_Clr_5v_Flt_Timer)))
   {
      BGVAR(u16_Clr_5v_Flt_Try) = 0;
      BGVAR(s32_5V_Fault_Cleared_Timer) = 100L;
   }

   return u16_FPGA_latched_flt_indication;
}


//**********************************************************
// Function Name: SecondaryEncoder5VFault
// Description:
//   Detects Secondary Encoder 5V fault
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
unsigned int SecondaryEncoder5VFault(int drive, int mode)
{
   unsigned int u16_FPGA_latched_flt_indication = 0, u16_mode = 0;
   unsigned int u16_FPGA_momentary_flt_indication = 0;
   unsigned long u32_bg_interval_time;
   static unsigned int u16_reset_cntr = 0;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if (MaskedFault(drive, SEC_ENCODER_5V_OC_MASK, 0))  return 0;
   u16_mode = mode;
   u16_FPGA_latched_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
   u16_FPGA_momentary_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_STATUS_REG_ADD);
   u16_FPGA_latched_flt_indication &= SEC_ENC_5V_OCn;
   u16_FPGA_momentary_flt_indication &= SEC_ENC_5V_OCn;

   *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_latched_flt_indication;//Clear latched

   u16_FPGA_latched_flt_indication = (u16_FPGA_latched_flt_indication > 0); // Translate to 0/1
   
   if ((u16_FPGA_latched_flt_indication) && (u16_reset_cntr < 10))
   {      
      *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD &= ~0x02;
      *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD |= 0x02;
      *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD &= ~0x02;
      *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD |= 0x02;
      *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD &= ~0x02;
      *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD |= 0x02;
      *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD &= ~0x02;
      *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD |= 0x02;
      
      u16_FPGA_latched_flt_indication = 0;
      u16_reset_cntr++;
      u16_FPGA_latched_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
      u16_FPGA_momentary_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_STATUS_REG_ADD);
      u16_FPGA_latched_flt_indication &= SEC_ENC_5V_OCn;
      u16_FPGA_momentary_flt_indication &= SEC_ENC_5V_OCn;

      *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_latched_flt_indication;//Clear latched

      u16_FPGA_latched_flt_indication = (u16_FPGA_latched_flt_indication > 0); // Translate to 0/1 
      if (!u16_FPGA_latched_flt_indication)  
      {
          u16_reset_cntr = 0;
          u16_mode = FLT_CLR;          
          //clr the secondary line break flt since it is due to momentary OC. 
          // set to 5 to let the FPGA line break indication stabelize.        
          s16_Clear_Sec_Line_Break_Flt_Flag = 50; 
      }
   }

   if (DETECT == u16_mode)
   {
     FeedbackFault(drive, u16_mode, SEC_ENCODER_5V_OC_MASK, (unsigned int *)(&u16_SecEncoder5Vfilter + drive), (unsigned int)u32_bg_interval_time, 30, u16_FPGA_latched_flt_indication, 0, 0);
     //Mechanism to Allow 3 first faults with minimal 100msec between CLR_FLTs --> 4th fault will result 60000msec break before CLR_FLT is enabled.
     if (0 != (BGVAR(s64_SysNotOk) & SEC_ENCODER_5V_OC_MASK))
     {
         if ((DETECT == u16_mode) && u16_FPGA_latched_flt_indication && (!BGVAR(u16_Flt_Sec5v_Was_Diagnosed)))
         {
            BGVAR(s32_Clr_Sec5v_Flt_Timer) = Cntr_1mS;
            BGVAR(u16_Clr_Sec5v_Flt_Allow) = 0;
            BGVAR(u16_Clr_Sec5v_Flt_Try)++;
            BGVAR(u16_Flt_Sec5v_Was_Diagnosed) = 1;
         }
         RemoveFeedbackSecondary5Volt();//ShutDown 5V Power to Secondary Encoder
     }
   }
   else //FLT_CLR mode
   {
      if (BGVAR(u16_Clr_Sec5v_Flt_Allow) && (!u16_FPGA_momentary_flt_indication))
      {
         if (BGVAR(u16_Clr_Sec5v_Flt_Try) >= 3)//if fault occurs the 4th time in 1 minute - enable next CLR_FLT after 60000msec only
         {
            BGVAR(u16_Clr_Sec5v_Flt_Try) = 0;
            BGVAR(s32_clr_Sec5v_Flt_Delay_Time) = 60000L;
         }
         FeedbackFault(drive, u16_mode, SEC_ENCODER_5V_OC_MASK, (unsigned int *)(&u16_SecEncoder5Vfilter + drive), (unsigned int)u32_bg_interval_time, 30, u16_FPGA_latched_flt_indication, 0, 0);
         SetFeedbackSecondary5Volt();  //turn back on 5V Power to Secondary Encoder
         BGVAR(u16_Flt_Sec5v_Was_Diagnosed) = 0;          
            
      }
   }

   if (PassedTimeMS(BGVAR(s32_clr_Sec5v_Flt_Delay_Time), BGVAR(s32_Clr_Sec5v_Flt_Timer)))//Flt_timer is either 100 msec or 60000msec
      BGVAR(u16_Clr_Sec5v_Flt_Allow) = 1;
   if (PassedTimeMS(60000L, BGVAR(s32_Clr_Sec5v_Flt_Timer)))
   {
      BGVAR(u16_Clr_Sec5v_Flt_Try) = 0;
      BGVAR(s32_clr_Sec5v_Flt_Delay_Time) = 100L;
   }
   return u16_FPGA_latched_flt_indication;
}


//**********************************************************
// Function Name: IndexBreakFault
// Description:
//   Detects Index break faults
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void IndexBreakFault(int drive, int mode)
{
   // AXIS_OFF;
   unsigned int u16_FPGA_flt_indication = 0;
   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if (MaskedFault(drive, INDEX_BRK_FLT_MASK, 0))  return;

   if ( (BGVAR(u16_Encoder_Off) || !BGVAR(u16_Tm_Enc_Status) ) && (mode != FLT_CLR) )  return;

   u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
   u16_FPGA_flt_indication &= MFB_AX0IFAIL;

   // Clear the fault on FPGA
   *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_flt_indication;

   u16_FPGA_flt_indication = (u16_FPGA_flt_indication > 0); // Translate to 0/1
   
   // In case of new CDHD, get the IFAIL indication from GPIO7
   if (IS_CAN2_DRIVE)
   {
      u16_FPGA_flt_indication = 0;
      if (GpioDataRegs.GPADAT.bit.GPIO7 == 0)
      {
         u16_FPGA_flt_indication = 1;
      }
   }

   // In case of DDHD, get the IFAIL indication from GPIO7
   if ( (u16_Product == DDHD) && (!IS_DDHD_EC2_DRIVE) )
   {
      u16_FPGA_flt_indication = 0;
      if (GpioDataRegs.GPADAT.bit.GPIO7 == 0)
         u16_FPGA_flt_indication = 1;
   }

   // Ignore index line break if IGNOREINDEXLB = 1 and correct feedback is used
   if (BGVAR(s16_Ignore_Index_Line_Break_Fault) == 1)
   {
      if ( (BGVAR(u16_FdbkType) == SW_SINE_FDBK)                                       &&
           ( (VAR(AX0_s16_Motor_Enc_Type) == 1) || (VAR(AX0_s16_Motor_Enc_Type) == 2) )  )
         u16_FPGA_flt_indication = 0;
   }

   FeedbackFault(drive, mode, INDEX_BRK_FLT_MASK, (unsigned int *)(&u16_IndexFilter + drive), (unsigned int)u32_bg_interval_time, 30, u16_FPGA_flt_indication, 1, 0);
}


void SecondaryIndexBreakFault(int drive, int mode)
{
   // AXIS_OFF;
   unsigned int u16_FPGA_flt_indication = 0;
   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if (MaskedFault(drive, SEC_INDEX_BRK_FLT_MASK, 0))  return;

   if ( ( (VAR(AX0_s16_Opmode) != 4) || (BGVAR(u16_2nd_Encoder_Src) != 2) || (BGVAR(u16_2nd_Encoder_Off)) ) &&
        (mode != FLT_CLR)                                                                                     )
      return; // Process Fault only if Gearing is On and from 2ndEnc and 5V is On, or during FltClr

   u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
   u16_FPGA_flt_indication &= UIF_SEC_ENC_IFAIL;

   // Clear the fault on FPGA
   *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_flt_indication;

   u16_FPGA_flt_indication = (u16_FPGA_flt_indication > 0); // Translate to 0/1

   if (IS_CAN2_DRIVE)
   {
      u16_FPGA_flt_indication = 0;
      if (GpioDataRegs.GPBDAT.bit.GPIO60 == 0)
      {
         u16_FPGA_flt_indication = 1;
      }
   }

   FeedbackFault(drive, mode, SEC_INDEX_BRK_FLT_MASK, (unsigned int *)(&u16_SecIndexFilter + drive), (unsigned int)u32_bg_interval_time, 30, u16_FPGA_flt_indication, 0, 0);
}


//**********************************************************
// Function Name: LineBreakFault
// Description:
//   Detects A/B line break
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void LineBreakFault(int drive, int mode)
{
   unsigned int u16_FPGA_flt_indication = 0;
   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;
   static unsigned int u16_flt_cntr = 0;
   // AXIS_OFF;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if ( MaskedFault(drive, LINE_BRK_FLT_MASK, 0)                  ||
       ( (BGVAR(u16_FdbkType) == RESOLVER_FDBK)                &&
         (BGVAR(s16_Adjust_Reference_State) != ADJ_REF_EN_PFB) &&
         (BGVAR(s16_Adjust_Reference_State) != ADJ_REF_DONE)     )  )
      return;

   if ( (BGVAR(u16_Encoder_Off) || !BGVAR(u16_Tm_Enc_Status) ) && (mode != FLT_CLR) )  return;

   u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);

   if ( ( (!IS_CAN2_DRIVE) && (u16_Product != DDHD) ) || IS_DDHD_EC2_DRIVE )
   {
      if (LVAR(AX0_s32_Feedback_Ptr) == &SW_SINE_ENC_FEEDBACK)
         u16_FPGA_flt_indication &= SIN_ENC_WB;
      else if (LVAR(AX0_s32_Feedback_Ptr) == &SW_RESOLVER_FEEDBACK)
         u16_FPGA_flt_indication &= RESOLVER_FAIL;
      else if (LVAR(AX0_s32_Feedback_Ptr) == &INC_ENCODER_FEEDBACK)
         u16_FPGA_flt_indication &= MFB_AX0ABFAIL;
   }
   else if (IS_CAN2_DRIVE)// In case of CAN2, get the ABFAIL indication from GPIO6, and the SIN_WB/Resolver_Fail the usuall way from FPGA
   {
      u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
      if (LVAR(AX0_s32_Feedback_Ptr) == &SW_SINE_ENC_FEEDBACK)
         u16_FPGA_flt_indication &= SIN_ENC_WB;
      else if (LVAR(AX0_s32_Feedback_Ptr) == &SW_RESOLVER_FEEDBACK)
         u16_FPGA_flt_indication &= RESOLVER_FAIL;
      else if (LVAR(AX0_s32_Feedback_Ptr) == &INC_ENCODER_FEEDBACK)
         u16_FPGA_flt_indication = (GpioDataRegs.GPADAT.bit.GPIO6 == 0);
   }
   else if (u16_Product == DDHD)     // In case of DDHD, get the ABFAIL indication from GPIO6
   {
      if (LVAR(AX0_s32_Feedback_Ptr) == &INC_ENCODER_FEEDBACK)
         u16_FPGA_flt_indication = (GpioDataRegs.GPADAT.bit.GPIO6 == 0);
   }

   *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_flt_indication;   // Clear the fault on FPGA

   u16_FPGA_flt_indication = (u16_FPGA_flt_indication > 0);   // Translate to 0/1

   if (u16_FPGA_flt_indication)
   {
      if (FEEDBACK_STEGMANN && (BGVAR(u16_Hiface_Init_State) != HIFACE_IDLE_STATE) && (BGVAR(u16_Hiface_Init_State) != HIFACE_INIT_ERROR_STATE))
         u16_FPGA_flt_indication = 0;   // Ignore this fault during HiPerFace Initialization Process.
      if (FEEDBACK_ENDAT && (BGVAR(s16_Endat_Init_State) != ENDAT_IDLE_STATE) && (BGVAR(s16_Endat_Init_State) != ENDAT_ERR_IDLE_STATE))
         u16_FPGA_flt_indication = 0;   // Ignore this fault during EnDat with Sine Initialization Process.
   }
   
   if (u16_FPGA_flt_indication)
   {//here we ignore LB indication fot the first 500 BG cycles, unless a good indication arrives, which means that the feedback is alive. 
      if (u16_flt_cntr < 500)       {
         BGVAR(s16_DisableInputs) |= INVALID_FDBK_MASK;
         u16_FPGA_flt_indication = 0;
         u16_flt_cntr++;
      }
      else
         BGVAR(s16_DisableInputs) &= ~INVALID_FDBK_MASK; 
   }
   else if (u16_flt_cntr < 500)
   {
      BGVAR(s16_DisableInputs) &= ~INVALID_FDBK_MASK;
      u16_flt_cntr = 500;
   }
   FeedbackFault(drive, mode, LINE_BRK_FLT_MASK, (unsigned int *)(&u16_ABFilter + drive), (unsigned int)u32_bg_interval_time, 15, u16_FPGA_flt_indication, 1, 0);
}

void SecondaryLineBreakFault(int drive, int mode)
{
   unsigned int u16_FPGA_flt_indication = 0, u16_clr_the_fault_flag = 0;
   unsigned long u32_bg_interval_time;
   unsigned int u16_mode = 0;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;
   
   if (MaskedFault(drive, SEC_LINE_BRK_FLT_MASK, 0))  return;

   u16_mode = mode;
   if ((!IS_SECONDARY_FEEDBACK_ENABLED) && (4 != VAR(AX0_s16_Opmode)))
   {
      if (((BGVAR(s64_SysNotOk) & SEC_LINE_BRK_FLT_MASK) == 0))
         return;
      else
         u16_clr_the_fault_flag = 1;
      // Process Fault only if Gearing from 2ndEnc is On or if secondary encoder is monitored. 
      // otherwise, if the fault already exists, clear it on clrflt 
   }
       

   u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
   u16_FPGA_flt_indication &= UIF_SEC_ENC_ABFAIL;

   // Clear the fault on FPGA
   *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_flt_indication;

   u16_FPGA_flt_indication = (u16_FPGA_flt_indication > 0); // Translate to 0/1
   
   if (IS_CAN2_DRIVE)
   {
      u16_FPGA_flt_indication = 0;
      if (GpioDataRegs.GPBDAT.bit.GPIO58 == 0)
      {
         u16_FPGA_flt_indication = 1;
      }
   }
   if (u16_clr_the_fault_flag)
      u16_FPGA_flt_indication = 0;
   
   /* if there's a secondary encoder OC fault - dont raise this flt or clear it (since the OC mechanism removes 5V) */   
   if (((u16_FPGA_flt_indication) || ((BGVAR(s64_SysNotOk) & SEC_LINE_BRK_FLT_MASK) != 0)) &&
         ((BGVAR(s64_SysNotOk) & SEC_ENCODER_5V_OC_MASK) != 0))
   {
      u16_FPGA_flt_indication = 0;
      u16_mode = FLT_CLR;
   }
   
   if (s16_Clear_Sec_Line_Break_Flt_Flag > 0) 
   {
      s16_Clear_Sec_Line_Break_Flt_Flag--;
      u16_FPGA_flt_indication = 0;
      u16_mode = FLT_CLR;
   }
   
   FeedbackFault(drive, u16_mode, SEC_LINE_BRK_FLT_MASK, (unsigned int *)(&u16_SecLineFilter + drive), (unsigned int)u32_bg_interval_time, 30, u16_FPGA_flt_indication, 0, 0);
}


// Special treatment - allow 3 times to clear the fault with delay of 100ms between tries,
// after 3 tries wait for 1 minute
void DigitalOutputOCFault (int drive, int mode)
{
   static unsigned int clr_flt_try = 0;
   static long clr_flt_timer = 0;
   static unsigned int clr_flt_allow = 1;
   static long clr_flt_delay_time = DIGITAL_OUTPUT_OC_CLEAR_FAULT_DELAY;
   unsigned int u16_FPGA_flt_indication = 0, u16_reset_mask = 0, bit = 0;
   
   if (!IS_HW_FUNC_ENABLED(DIG_OUTPUTS_OC_FAILURE_MASK))
       return;
   if (MaskedFault(drive, DIGITAL_OUTPUT_OC_FLT_MASK, 1))
       return;
   
   UpdateDigitalOutputsOvercurrent(drive);
   u16_FPGA_flt_indication = BGVAR(u16_Digital_Outputs_Overcurrent);
   
   // BGVAR(u16_Digital_Output_Clear_Allow) - set by the tester to ignore 1 minute delay
   if ((mode == FLT_CLR) && (clr_flt_allow || BGVAR(u16_Digital_Output_Clear_Allow)) )
   {
      for (bit = 0; ((bit < s16_Num_Of_Outputs) && u16_FPGA_flt_indication); bit++) //first reset specific Digital Output
      {
         if ((3 == bit) && (IS_EC_LITE_HV_DRIVE)) bit = 6;
         if (u16_FPGA_flt_indication & 0x1)
         {
            u16_reset_mask |= (1 << bit);                                        //build mask for reset            
            *(unsigned int *)(FPGA_OC_DIG_OUT_RESET_REG_ADD) &= ~u16_reset_mask; //low edge of specific DO 
            *(unsigned int *)(FPGA_OC_DIG_OUT_RESET_REG_ADD) |=  u16_reset_mask; //rise edge of specific DO to reset
            *(unsigned int *)(FPGA_OC_DIG_OUT_RESET_REG_ADD) &= ~u16_reset_mask; //low edge of specific DO 

            u16_reset_mask = 0;
            clr_flt_timer = Cntr_1mS;
         }
         u16_FPGA_flt_indication >>= 1;
      }

      if (BGVAR(u16_Digital_Outputs_Overcurrent)) //next clear the fault (if there was), maximum 3 times per 60 seconds
      {
         clr_flt_try++;
         if (clr_flt_try >= 3)
         {
            clr_flt_try = 0;
            clr_flt_delay_time = DIGITAL_OUTPUT_OC_CLEAR_FAULT_TIMEOUT;
         }
         BGVAR(s64_SysNotOk_2) &= ~DIGITAL_OUTPUT_OC_FLT_MASK;
      }
   
   }
   else if (DETECT == mode)
   {
      //if fault exist, handle fault
      if (BGVAR(u16_Digital_Outputs_Overcurrent))
      {
         clr_flt_allow = 0;
         if (HandleFault(drive, DIGITAL_OUTPUT_OC_FLT_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_OUTPUT_OVERCURRENT;
            CANFaultControl(drive, 0, DIGITAL_OUTPUT_OC_FLT_MASK, 1);
         }
      }
   }
   
   if (PassedTimeMS(clr_flt_delay_time, clr_flt_timer)) clr_flt_allow = 1;
   if (PassedTimeMS(DIGITAL_OUTPUT_OC_CLEAR_FAULT_TIMEOUT, clr_flt_timer))
   {
      clr_flt_try = 0;
      clr_flt_delay_time = DIGITAL_OUTPUT_OC_CLEAR_FAULT_DELAY;
   }  
}


// For FC - dont check this fault when emergency stopped is pressed and 1.5 seconds after emergency stop
// is released to comply with their safety chain
void PulseAndDirLineBreakFault(int drive, int mode)
{
    // AXIS_OFF;

    unsigned int u16_FPGA_flt_indication = 0;
    int s16_emergency_stop = 0;
    unsigned long u32_bg_interval_time;
    static unsigned long u32_cntr_1ms_capture;

    u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
    u32_cntr_1ms_capture = Cntr_1mS;

    // Process Fault only if Gearing is On and from Pulse&Dir, or during FltClr
    if (((VAR(AX0_s16_Opmode) != 4) || (BGVAR(u16_2nd_Encoder_Src) != 1)) && (mode != FLT_CLR))
    {
        return;
    }

    // Checking Condition "Not DDHD2-EC" to allow detection at FPGA Registers
    if ((u16_Product == DDHD) && (!IS_DDHD_EC2_DRIVE))
    {
        // In case of DDHD, get the PULSE_DIRECTION_FAIL indication from GPIO13
        if (GpioDataRegs.GPADAT.bit.GPIO13 == 0)
        {
            u16_FPGA_flt_indication = 1;
        }
    }
    // CDHD
    else
    {
        // As there is noise on the P&D lines filter was added here, hence the flt is not indicated by
        // the FPGA's latched register but the momentary one
        u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_STATUS_REG_ADD);
        u16_FPGA_flt_indication &= CIF_PUL_AND_DIR_FAIL;
        // Clear the fault on FPGA
        *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_flt_indication;
        // Translate to 0/1
        u16_FPGA_flt_indication = (u16_FPGA_flt_indication > 0);
    }

    // Clear the fault indication if the fault is masked.
    // Don't do it via MaskedFault() because it clears the fault and might enable the drive immediately
    if ((BGVAR(s64_Faults_Mask) & PD_LINE_BRK_FLT_MASK) == 0)
    {
        u16_FPGA_flt_indication = 0;
    }

    // check status of IGNOREPDLB
    // 0 - detect fault always
    // 1 - detect fault only when drive enabled
    // 2 - ignore fault completely
    if (((BGVAR(s16_Ignore_Pulse_Line_Break_Fault) == 1) && (!Enabled(drive))) ||
        (BGVAR(s16_Ignore_Pulse_Line_Break_Fault) == 2))
    {
        u16_FPGA_flt_indication = 0;
    }

    s16_emergency_stop = BGVAR(s16_DisableInputs) & EMERGENCY_STOP_MASK;

    if (s16_emergency_stop)
    {
        BGVAR(s16_PD_Flt_State) = 1;
    }

    switch (BGVAR(s16_PD_Flt_State))
    {
        case 0:
        {
            break;
        }
        case 1:
        {
            if (!s16_emergency_stop)
            {
                BGVAR(s32_PD_Flt_Timer) = Cntr_1mS;
                BGVAR(s16_PD_Flt_State) = 2;
            }
            // Ignore the flt during emergency stop
            u16_FPGA_flt_indication = 0;
            break;
        }
        case 2: // Ignore the flt 1.5 Sec after emergency stop is released
        {
            if (PassedTimeMS(1500L, BGVAR(s32_PD_Flt_Timer)))
            BGVAR(s16_PD_Flt_State) = 0;
            u16_FPGA_flt_indication = 0;
            break;
        }
        default:
        {
            break;
        }
    }

    FeedbackFault(drive, mode, PD_LINE_BRK_FLT_MASK, (unsigned int *)(&u16_PD_LineFilter + drive), (unsigned int)u32_bg_interval_time, 30, u16_FPGA_flt_indication, 0, 0);
}


// Becuase of a HW issue the 5V fault will occur when STO is connected (bug #3827), till this is resovled in HW, the faults will be masked
// while the STO is dis-connected
void Drive5VFault(int drive, int mode)
{
   if (!u16_STO_Flt_Indication_Actual) // This "if" needs to be removed after the HW fix mentioned above
   GenericFPGADetectedFault(drive, mode, DRIVE_5V_FLT_MASK, DRV_5V_FAIL);
   else // This will clear the FPGA latch indicaton, can be removed after the HW fix as well
   {
      *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= DRV_5V_FAIL;
   }

   // 5V fault happens on every power off. Log it after delay, since if the drive is still running the fault is real
   if (BGVAR(u16_Delayed_5V_Log_Fault_Flag) == 1)
   {
      if (PassedTimeMS(5000L, BGVAR(s32_Delayed_5V_Log_Fault_Timer)))
      {
         BGVAR(u16_Delayed_5V_Log_Fault_Flag) = 0;
         WriteToFaultLog(drive, GetFaultId((unsigned long long)DRIVE_5V_FLT_MASK, (int)0));
      }
   }
}

// Therm value [ohm] = 3240 * Vadc/(5000-Vadc) or degs in ServoSense
int ReadThermCommand(long long *s64_val, int drive)
{
   int s16_adc_mv = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (FEEDBACK_SERVOSENSE)
   {
      *s64_val = (long long)BGVAR(s16_SrvSns_Temperature);
      return SAL_SUCCESS;
   }

   if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)) return NOT_AVAILABLE;

   s16_adc_mv = BGVAR(s16_Adc_Mfb_Motor_Temp_Filtered);

   s16_adc_mv = (int)(((float)s16_adc_mv / 32767.0) * 10000.0);

   if (s16_adc_mv < 0) s16_adc_mv = 0;

   if (s16_adc_mv >= 4983) *s64_val = 1000000LL;
   else *s64_val = (long long)(3240.0 * ((float)s16_adc_mv / (float)(5000 - s16_adc_mv)));

   return SAL_SUCCESS;
}


int SalReadThermCommand(int drive)
{
   long long s64_temperature = 0;
   int u16_ret_val = SAL_SUCCESS;

   u16_ret_val = ReadThermCommand(&s64_temperature, drive);

   if (u16_ret_val == SAL_SUCCESS)
   {  // On ServoSense temperature is in deg otherwise Ohm
      PrintSignedInt64(s64_temperature);
      if (FEEDBACK_SERVOSENSE)
         PrintString(" [deg]", 0);
      else
         PrintString(" [Ohm]", 0);

      PrintCrLf();
   }

   return u16_ret_val;
}

// THERMODE
int SalThermModeCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((param == 1LL) || (param == 2LL)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u16_Therm_Mode) = (unsigned int)param;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: MotorOverTempFault
// Description:
//   Detects Motor over temperature
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void MotorOverTempFault(int drive, int mode)
{
   long long s64_temp_var = 0LL;
   int s16_servosense_ot_thresh;
   // AXIS_OFF;

   if (MaskedFault(drive, MOTOR_OT_FLT_MASK, 0))  return;

   if ( (BGVAR(u16_Encoder_Off) || !BGVAR(u16_Tm_Enc_Status) ) && (mode != FLT_CLR) )  return;

   // Calc the OT threshold in case of ServoSense:
   // The threshold is min(120, MTP_Winding_Temp - Offset). Currently the offset is set fixed to 40 degC.
   // "MTP_Winding_Temp - Offset" comes from MPC. "120" is to protect the encoder.
   // See email from Wolfgang "Minutes: MTP and Motor Protection" from 30/7/2014

   s16_servosense_ot_thresh = BGVAR(s16_ServoSense_Therm_Trip_Level) - 40;

   // On Pro2 motors, since the temperature threshold on old MTP files (where u16_MTP_Data_Revision<0x20XX) was set incorrectly to a low value, it will be ignored and 115deg value will be used instead.
   // Once the MTP value will be fixed this can be revised   
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW) && ((BGVAR(u16_MTP_Data_Revision) & 0xFF00) < 0x2000)) // On SE stay with the MTP threshold as it is correct
      s16_servosense_ot_thresh = 115;

   if (s16_servosense_ot_thresh > 120)
      s16_servosense_ot_thresh = 120;

   ReadThermCommand(&s64_temp_var, drive);

   if (BGVAR(u16_Therm_Value) == 0)
   {
      if ( (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_MULTI_TURN_FDBK) ||
           (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK)  )
      {  // Don't check fault in case of CRC error, the data might be corrupted
         if ( ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_INVALID_MASK) == 0) &&
              ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) != 0)       &&
              // Ignore TM Over-Temp. Indication when Encoder is re-initialized
              ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x01000) == 0x01000)                    )
            BGVAR(u16_Therm_Value) = 1;
      }
      else
      {  // On ServoSense compare the temperature to a different trip level as its units are in C and not Ohms
         if (FEEDBACK_SERVOSENSE)
         {
            if ((int)s64_temp_var >= s16_servosense_ot_thresh)
               BGVAR(u16_Therm_Value) = 1;
         }
         else if (u16_Product != SHNDR_HW)
         {
            if ( ( (BGVAR(u16_Therm_Type) == 0)                                &&
                   ((unsigned long)s64_temp_var >= BGVAR(u32_Therm_Trip_Level))  ) ||
                 ( (BGVAR(u16_Therm_Type) == 1)                                &&
                   ((unsigned long)s64_temp_var <= BGVAR(u32_Therm_Trip_Level))  )   )
               BGVAR(u16_Therm_Value) = 1;
         }
      }
   }
   else if (BGVAR(u16_Therm_Value) == 1)
   {
      if ( (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_MULTI_TURN_FDBK) ||
           (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK)  )
      {
         if ( ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x01000) != 0x01000)            ||
              ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0) ||
              // Ignore TM Over-Temp. Indication when Encoder is re-initialized
              (mode == FLT_CLR)                                                       )
            BGVAR(u16_Therm_Value) = 0;
      }
      else
      {
         if (FEEDBACK_SERVOSENSE)
         {
            if ((int)s64_temp_var < (s16_servosense_ot_thresh - 20))
               BGVAR(u16_Therm_Value) = 0;
         }
         else if (u16_Product != SHNDR_HW)
         {
            if ( ( (BGVAR(u16_Therm_Type) == 0)                                &&
                   ((unsigned long)s64_temp_var < BGVAR(u32_Therm_Clear_Level))  ) ||
                 ( (BGVAR(u16_Therm_Type) == 1)                                &&
                   ((unsigned long)s64_temp_var > BGVAR(u32_Therm_Clear_Level))  )   )
               BGVAR(u16_Therm_Value) = 0;
         }
      }
   }

   if (BGVAR(u16_Therm_Mode) == 3) BGVAR(u16_Therm_Value) = 0;

   if (BGVAR(u16_Therm_Value))
   {
      switch (BGVAR(u16_Therm_Mode))
      {
         case 0: // Disable drive and open fault relay
            if ( (mode == DETECT) && (!(BGVAR(u64_Sys_Warnings) & MOTOR_OT_WRN_MASK)) &&
                 (!(BGVAR(s64_SysNotOk) & MOTOR_OT_FLT_MASK))                           )
            {
               ++BGVAR(u16_Motor_Ot_Filter);
               if (BGVAR(u16_Motor_Ot_Filter) == 10)
               {
                  if (HandleFault(drive, MOTOR_OT_FLT_MASK, 0))
                  {
                     p402_error_code = ERRCODE_CAN_MOTOR_OT;
                     CANFaultControl(drive, 0, MOTOR_OT_FLT_MASK, 0);
                  }
               }
            }
         break;

         case 3: // Ignore thermostat input, handled before
         break;

         case 4: // Issue warning; no other action
            BGVAR(u64_Sys_Warnings) |= MOTOR_OT_WRN_MASK;
         break;

         case 5: // Issue warning, fault after THERMTIME elapses
            if ( ((BGVAR(u64_Sys_Warnings) & MOTOR_OT_WRN_MASK) == 0) &&
                 ((BGVAR(s64_SysNotOk) & MOTOR_OT_FLT_MASK) == 0)       ) // First time here
            {
               BGVAR(u16_MotorOTTimer) = Cntr_1mS;
               BGVAR(u64_Sys_Warnings) |= MOTOR_OT_WRN_MASK;
            }

            if (PassedTimeMS((long)1000L * BGVAR(u16_Therm_Time), (long)BGVAR(u16_MotorOTTimer)))
            {
               if (HandleFault(drive, MOTOR_OT_FLT_MASK, 0))
               {
                  p402_error_code = ERRCODE_CAN_MOTOR_OT;
                  CANFaultControl(drive, 0, MOTOR_OT_FLT_MASK, 0);
               }
               BGVAR(u64_Sys_Warnings) &= ~MOTOR_OT_WRN_MASK;
            }
         break;
      }

      BGVAR(u16_Therm_Reset_State) = THERM_RESET_IDLE; // Idle the Resetting State Machine

   }
   else
   {
      if ( (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_MULTI_TURN_FDBK) ||
           (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK)  )
      {
         if ( (mode == FLT_CLR) && (BGVAR(s64_SysNotOk) & MOTOR_OT_FLT_MASK) &&
              (BGVAR(u16_Therm_Reset_State) == THERM_RESET_IDLE)               )
            BGVAR(u16_Therm_Reset_State) = THERM_RESET_INIT;

         if ( (mode == FLT_CLR) || (BGVAR(u16_Therm_Reset_State) != THERM_RESET_IDLE) )
         {
            switch (BGVAR(u16_Therm_Reset_State))
            {
               case THERM_RESET_IDLE:
               break;

               case THERM_RESET_INIT:
                  BGVAR(u16_MotorOTTimer) = Cntr_1mS; // Preparing for 0.1 Second Delay...
                  BGVAR(u16_Therm_Reset_State) = THERM_RESET_WAIT_ENC_OFF;
               break;

               case THERM_RESET_WAIT_ENC_OFF: // Wait until TM Encoder Resetting has started...
                  if ( (PassedTimeMS(100L, (long)BGVAR(u16_MotorOTTimer))) ||
                       (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) != 0)                )
                       // If Encoder Restart, or 0.1 Second Delay (if Restart is not required)
                     BGVAR(u16_Therm_Reset_State) = THERM_RESET_WAIT_ENC_ON;
               break;

               case THERM_RESET_WAIT_ENC_ON: // Wait until TM Encoder Resetting is complete,
                  if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) != 0)
                  {
                     BGVAR(u16_MotorOTTimer) = Cntr_1mS;
                     BGVAR(u16_Therm_Reset_State) = THERM_RESET_TIME_DELAY;
                  }
               break;

               case THERM_RESET_TIME_DELAY: // Allow additional delay after TM Encoder is reset...
                  if (PassedTimeMS(100L, (long)BGVAR(u16_MotorOTTimer)))
                  {
                     if ( ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_INVALID_MASK) == 0) &&
                          // Accept TM Over-Temp. No-Indication only if Encoder reading is valid (correct CRC)
                          ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x01000) == 0x00000)                    )
                        BGVAR(s64_SysNotOk) &= ~MOTOR_OT_FLT_MASK;
                     BGVAR(u16_Therm_Reset_State) = THERM_RESET_IDLE;
                  }
               break;

               default:
                  BGVAR(u16_Therm_Reset_State) = THERM_RESET_IDLE;
               break;
            }
         }

      }
      else if (mode == FLT_CLR) // Not TM17 Encoder, Resistive or Switch Temp. Sensor
         BGVAR(s64_SysNotOk) &= ~MOTOR_OT_FLT_MASK;

      BGVAR(u64_Sys_Warnings) &= ~MOTOR_OT_WRN_MASK;

      BGVAR(u16_Motor_Ot_Filter) = 0; // reset the qualifier
   }
}


//**********************************************************
// Function Name: EncoderPhaseError
// Description:
//   In normal operating conditions, quadrature inputs QEPA and QEPB will be 90
//   degrees out of phase. The phase error is set when edge transition is detected
//   simultaneously on the QEPA and QEPB signals.
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
void EncoderPhaseError(int drive, int mode)
{
   // AXIS_OFF;
   unsigned int u16_flt_indication;
   static int s16_prev_no_comp_state = 0;

   if (MaskedFault(drive, ENCODER_PHASE_ERROR_MASK, 1))
   {
      EQep1Regs.QCLR.bit.PHE = 1;  // Clear the fault bit on DSP peripheral
      return;
   }

   if ( (BGVAR(u16_Encoder_Off) || !BGVAR(u16_Tm_Enc_Status) ) && (mode != FLT_CLR) )  return;

   if (BGVAR(u16_FdbkType) != INC_ENC_FDBK)
   {
      EQep1Regs.QCLR.bit.PHE = 1;  // Clear the fault bit on DSP peripheral
      return;
   }

   if ( (VAR(AX0_s16_Motor_Enc_Type) != 0) && (VAR(AX0_s16_Motor_Enc_Type) != 1) &&
        (VAR(AX0_s16_Motor_Enc_Type) != 2) && (VAR(AX0_s16_Motor_Enc_Type) != 3) &&
        (VAR(AX0_s16_Motor_Enc_Type) != 4) && (VAR(AX0_s16_Motor_Enc_Type) != 6)   )
   {
      EQep1Regs.QCLR.bit.PHE = 1;  // Clear the fault bit on DSP peripheral
      return;
   }

   u16_flt_indication = 0;

   if (EQep1Regs.QFLG.bit.PHE)
   {
      // Illegal A/B state transition may occur when enabling/disabling micro-interpolation
      // in the FPGA so next "if" is used to ignore such a false fault during the config
      if ( (s16_prev_no_comp_state == (BGVAR(s64_SysNotOk) & NO_COMP_FLT_MASK))    &&
           ( (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_2 & 0xffff)) ||
             (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_3 & 0xffff))   )  )
         u16_flt_indication = 1;
      EQep1Regs.QCLR.bit.PHE = 1;  // Clear the fault on DSP peripheral
   }
   s16_prev_no_comp_state = BGVAR(s64_SysNotOk) & NO_COMP_FLT_MASK;

   FeedbackFault(drive, mode, ENCODER_PHASE_ERROR_MASK, NULL, 0, 0, u16_flt_indication, 1, 1);
}

long s32_Delta, s32_Index_Pos, s32_Modulo;
//**********************************************************
// Function Name: CommutationFault
// Description:
//         Detect AqB counting/commutation error based on index signal reference
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void CommutationFault(int drive, int mode)
{
   // AXIS_OFF;
   int u16_flt_indication = 0;
   long  s32_counts_in_rev, /* s32_delta, s32_index_pos, */ s32_max_error; //, s32_modulo;

   if (MaskedFault(drive, COMMUTATION_LOSS_FAULT_MASK, 1)) return;

   if ( (BGVAR(u16_Encoder_Off) || !BGVAR(u16_Tm_Enc_Status) ) && (mode != FLT_CLR) )  return;
   if (VAR(AX0_s16_Enc_State_Ptr) != (int)((long)&ENC_STATE_3 & 0xffff))  return;
   if (BGVAR(u16_Init_Commutation_Fault_Flag))//raised after change in counts_per_rev
   {
      BGVAR(s16_Commutation_Fault_State) = 0;
      BGVAR(s16_Commutation_Flt_Filter) = 0; 
      BGVAR(s32_Aqb_Flt_Enc_Index_Capture) = 0L;
      BGVAR(s32_AqB_Flt_Acc_Error) = 0L; 
      BGVAR(u16_Init_Commutation_Fault_Flag) = 0;
   }

   if (mode == FLT_CLR)   //ignore fault if threshold 0
   {
      BGVAR(s64_SysNotOk_2) &= ~COMMUTATION_LOSS_FAULT_MASK;
      BGVAR(s16_Commutation_Fault_State) = 0;
   }

   if ( (BGVAR(u16_Comm_Flt_Threshold) == 0)              || // Fault is disabled by setting Threshold to 0...
        (BGVAR(u16_Motor_Setup_State) |= MOTOR_SETUP_IDLE)  ) // Ignore Fault while Motor-Setup is running...
      return;

   //reset state machine in case of no-comp
   if ( ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)     ||
        ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0) ||
        ((VAR(AX0_s16_Skip_Flags) & ENC_CONFIG_NEEDED_MASK) != 0)   )
   {
      BGVAR(s16_Commutation_Fault_State) = 0;
   }

   switch (BGVAR(s16_Commutation_Fault_State))
   {
      case 0:
         EQep1Regs.QCLR.bit.IEL = 1; // Clear Bit 10, Index Event Latch
         BGVAR(s16_Commutation_Fault_State) = 1; //wait index capture clear
      break;

      case 1:
         if (EQep1Regs.QFLG.bit.IEL == 0)
         {
            EQep1Regs.QEPCTL.bit.IEL = 1; // Allow Index Capture...
            BGVAR(s16_Commutation_Fault_State) = 2; //wait index capture clear
         }
      break;

      case 2:   //capture first index position
         if (EQep1Regs.QFLG.bit.IEL == 1) // wait capture done
         {
            BGVAR(s32_Aqb_Flt_Enc_Index_Capture) = EQep1Regs.QPOSILAT;
            BGVAR(s32_AqB_Flt_Acc_Error) = 0L;
            BGVAR(s16_Commutation_Flt_Filter) = 0;
            BGVAR(s16_Commutation_Fault_State) = 3;
         }
      break;

      case 3:  //check that no drift on encoder with reference to index
         s32_Index_Pos = EQep1Regs.QPOSILAT;
         if ( (BGVAR(u16_Motor_Enc_Interpolation) > 1) && (BGVAR(u16_Motor_Enc_Interpolation_Mode) == 1) )
            s32_counts_in_rev = LVAR(AX0_s32_Counts_Per_Rev);
         else s32_counts_in_rev = LVAR(AX0_s32_Counts_Per_Rev) * BGVAR(u16_Motor_Enc_Interpolation);
         s32_max_error =  s32_counts_in_rev / 360 * BGVAR(u16_Comm_Flt_Threshold); //calculate max error
         if (s32_max_error == 0L) s32_max_error = 1L;

         //calculate modulo 1 rev between index captures.
         //ideally if index alligned with AqB signals modulo shall be 0.
         s32_Delta = BGVAR(s32_Aqb_Flt_Enc_Index_Capture) - s32_Index_Pos; //difference between first cuptulred index and last capture
         s32_Modulo = s32_Delta % s32_counts_in_rev;

         //module is either around 0 +/-several counts or around s32_counts_in_rev , +/- several counts
         //reduce s32_counts_in_rev if module arround s32_counts_in_rev
         if (s32_Modulo > (s32_counts_in_rev >> 2)) s32_Modulo = s32_Modulo - s32_counts_in_rev;
         else if (s32_Modulo < -(s32_counts_in_rev >> 2)) s32_Modulo = s32_Modulo + s32_counts_in_rev;

         //filter and accumulate error
         if (abs(s32_Modulo) > s32_max_error) BGVAR(s16_Commutation_Flt_Filter)++;
         else BGVAR(s16_Commutation_Flt_Filter)--;
         if (BGVAR(s16_Commutation_Flt_Filter) < 0) BGVAR(s16_Commutation_Flt_Filter) = 0;

         if ((BGVAR(s16_Commutation_Flt_Filter) > 3) || (abs(s32_Modulo) < s32_max_error))
         {
            BGVAR(s32_AqB_Flt_Acc_Error) = BGVAR(s32_AqB_Flt_Acc_Error) + s32_Modulo;
            BGVAR(s32_Aqb_Flt_Enc_Index_Capture) = s32_Index_Pos;
         }

         u16_flt_indication = abs(BGVAR(s32_AqB_Flt_Acc_Error)) > s32_max_error;
         if (u16_flt_indication)
         {
            BGVAR(s16_Commutation_Fault_State) = 4;
         }
      break;

      case 4: //fault
         if ( (BGVAR(s64_SysNotOk_2) & COMMUTATION_LOSS_FAULT_MASK ) == 0 ) BGVAR(s16_Commutation_Fault_State) = 0;
      break;
   }

   FeedbackFault(drive, mode, COMMUTATION_LOSS_FAULT_MASK, NULL, 0, 0, u16_flt_indication, 1, 1);
}


//**********************************************************
// Function Name: AbOutOfRangeFault
// Description:
//   Detects A/B out of range
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void AbOutOfRangeFault(int drive, int mode)
{
   // AXIS_OFF;
   int temp_var = 0;
   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if (MaskedFault(drive, AB_OUT_OF_RANGE_FLT_MASK, 0)) return;

   // Test AB_OUT_OF_RANGE only when resolver ref algorithm is done
   if ( (BGVAR(u16_FdbkType) == RESOLVER_FDBK)                    &&
        ( (BGVAR(s16_Adjust_Reference_State) == ADJ_REF_DONE)  ||
          (BGVAR(s16_Adjust_Reference_State) == ADJ_REF_FAILED)  )  )
   {
      if ( ((VAR(AX0_s16_Out_Of_Range_Bits) & (AB_OUT_OF_RANGE_RT_HI | AB_OUT_OF_RANGE_RT_LO)) != 0) &&
           ((BGVAR(s64_SysNotOk) & AB_OUT_OF_RANGE_FLT_MASK) == 0)                                     )
         temp_var = 1; // Condition Error Detection on 1st Time to avoid repeated Detection and
                       // allow Resetting by FeedbackFault()
   }

   temp_var = FeedbackFault(drive, mode, AB_OUT_OF_RANGE_FLT_MASK, (unsigned int *)(&u16_OutOfRangeFilter + drive), (unsigned int)u32_bg_interval_time, 30, temp_var, 1, 0);
}


//**********************************************************
// Function Name: EndatFault
// Description:
//   Detects endat comm fault
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void EndatFault(int drive, int mode)
{
   // AXIS_OFF;
   unsigned int temp_var = 0, u16_endat_alarm, u16_endat_crc_ok, temp_time;

   if (MaskedFault(drive, ENDAT2X_FDBK_FAULTS_MASK, 0))  return;

   // copy Real-Time Variables locally for consistency, check RT Timer to ensure copying within
   temp_time = Cntr_3125; // the same RT Interrupt
   u16_endat_crc_ok = ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_INVALID_MASK) == 0);
   u16_endat_alarm = VAR(AX0_u16_Abs_Enc_Info_Frame);
   if (temp_time != Cntr_3125)
   { // re-copy if RT Interrupt occurred
      u16_endat_crc_ok = ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_INVALID_MASK) == 0);
      u16_endat_alarm = VAR(AX0_u16_Abs_Enc_Info_Frame);
   }
// Condition Alarm-Bit(s) Sensing on valid CRC at time of reading;
// Test Alarm only after Initialization ended (ENDAT_IDLE_STATE of State-Machine)
   temp_var = ( (BGVAR(s16_Endat_Init_State) == ENDAT_ERR_IDLE_STATE)   || // Initialization Error
                ( (BGVAR(s16_Endat_Init_State) == ENDAT_IDLE_STATE) &&
                  ((BGVAR(u16_EnDat_22_Supported) & 0x03) == 0x01)  &&     // EnDat 2.2, Alarm Bits
                  ((u16_endat_alarm & 0x03) != 2) && u16_endat_crc_ok     ) || // Should be "10"
                ( (BGVAR(s16_Endat_Init_State) == ENDAT_IDLE_STATE) &&
                  ((BGVAR(u16_EnDat_22_Supported) & 0x03) != 0x01)  &&     // EnDat 2.1, Alarm Bit
                  ((u16_endat_alarm & 0x01) != 0) && u16_endat_crc_ok     )   ); // should be zero
   if (mode == FLT_CLR)
   {
      VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~(COMM_FDBK_TIME_OUT_ERROR_MASK | COMM_FDBK_CRC_ERROR_MASK);
      temp_var = 0; // force endat state machine reset to try and clr fault
   }
   FeedbackFault(drive, mode, ENDAT2X_FDBK_FAULTS_MASK, NULL, 0, 0, temp_var, 1, 0);
}


//**********************************************************
// Function Name: HiperfaceFault
// Description:
//   Detects Hiperface comm fault
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void HiperfaceFault(int drive, int mode)
{
   int temp_var = 0;

   if (MaskedFault(drive, SIN_COMM_FLT_MASK, 0)) return;

   if (mode == DETECT)
      temp_var = ((BGVAR(u16_Hiface_Init_State) == HIFACE_INIT_ERROR_STATE));
// Removed conditioning of Hiperface Error Resetting on Hiperface Comm. Error, to avoid
// race condition of no resetting.
   FeedbackFault(drive, mode, SIN_COMM_FLT_MASK, NULL, 0, 0, temp_var, 1, 0);

   if (mode == FLT_CLR)
      u16_Hiperface_StampRead_Retry = 0;

   temp_var &= (5 < u16_Hiperface_StampRead_Retry);
      FeedbackFault(drive, mode, HIFACE_ILLEGAL_STAMP_FLT_MASK, 0, NULL, 0, temp_var, 1, 1);
}


//**********************************************************
// Function Name: CommFdbkFault
// Description:
//   Detects Communication Feedback  fault
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void CommFdbkFault(int drive, int mode)
{
   // AXIS_OFF;

   int temp_var = 0;

   if (MaskedFault(drive, COMM_FDBK_FLT_MASK, 0)) return;

   if (mode == FLT_CLR)
   {
      // Init the feeedback state machine if there is a fault
      if ( ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & (COMM_FDBK_TIME_OUT_ERROR_MASK | COMM_FDBK_CRC_ERROR_MASK)) != 0) ||
           ( ( (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_MULTI_TURN_FDBK) || // Tamagawa Encoder Reset is needed if it caused
               (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK)  ) && // Overtemperature Indication.
               ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x01000) == 0x01000)  )                                            )
      {
         VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~COMM_FDBK_READY_MASK;
         BGVAR(s16_DisableInputs) |= COMM_FDBK_DIS_MASK;          // Set disable bit
         // Set Comm.-Feedback Disable Bit to prevent Enable before Comm.-Feedback
         // Initialization Process starts, to prevent Enabling before Initializtion.
         if (FEEDBACK_BISSC)
            BiSSC_Restart(drive);
         else
         {
            BGVAR(u16_CommFdbk_Init_Select) = 0;
            BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      }
      }

      VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~(COMM_FDBK_TIME_OUT_ERROR_MASK | COMM_FDBK_CRC_ERROR_MASK);
      temp_var = 0;
      // If system uses MTP and the user MTP mode has been assigned (MTP has bee loaded)
      // then signal to comm feedback state machine, that MTP load trigger is needed.
      if (BGVAR(u16_MTP_Mode) && (BGVAR(u16_MTP_Mode_User) == BGVAR(u16_MTP_Mode)))
         BGVAR(u16_MTP_Load_Done) = 0;

      // Raise the reset required flag to ensure power reset on comm fdbk stm init for sensAR multi turn
      if(FEEDBACK_SERVOSENSE)
         VAR(AX0_u16_SrvSns_FWStatus) |= SRVSNS_FW_STATUS_PWR_RESET_REQ_MASK;
   }
   else
      temp_var = (int)((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & (COMM_FDBK_TIME_OUT_ERROR_MASK | COMM_FDBK_CRC_ERROR_MASK)) != 0);

   FeedbackFault(drive, mode, COMM_FDBK_FLT_MASK, NULL, 0, 0, temp_var, 1, 0);
}


void SanyoEncoderFault(int drive, int mode)
{
   // AXIS_OFF;

   int temp_var = 0;

   if (MaskedFault(drive, SANYO_FDBK_FLT_MASK, 0)) return;

   if ((mode == FLT_CLR) && (BGVAR(s64_SysNotOk) & SANYO_FDBK_FLT_MASK))
   {
      BGVAR(u16_CommFdbk_Init_Select) = 0;
      BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
   }

   temp_var = VAR(AX0_u16_Abs_Enc_Info_Frame) & 0x5000;
   FeedbackFault(drive, mode, SANYO_FDBK_FLT_MASK, NULL, 0, 0, temp_var, 0, 0);
}


void AbsEncBattLowFault(int drive, int mode)
{
   // AXIS_OFF;

   int temp_var = 0, s16_mask; // Check Battery Low-Voltage Fault in Alarms Field
   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if (MaskedFault(drive, ABS_ENC_BATT_LOW_MASK, 0))
   {
      BGVAR(u64_Sys_Warnings) &= ~TM_BATT_LOW_VOLTAGE_WRN;
      return;
   }
   
   if (u16_Ignore_Abs_Fdbk_Batt_Faults) //On a PRESTO request - an ability to mask the fault
   { 
      BGVAR(u64_Sys_Warnings) &= ~TM_BATT_LOW_VOLTAGE_WRN;
      BGVAR(s64_SysNotOk) &= ~ABS_ENC_BATT_LOW_MASK;
      return;
   }

   if ( (LVAR(AX0_s32_Feedback_Ptr) == &TM_COMMUNICATION_FEEDBACK)  ||
        (LVAR(AX0_s32_Feedback_Ptr) == &PS_S_COMMUNICATION_FEEDBACK)  )
      s16_mask = 0x04000; // Check Tamagawa or Panasonic Absolute Battery Low-Voltage Fault in Status Field
   else if (LVAR(AX0_s32_Feedback_Ptr) == &FANUC_COMMUNICATION_FEEDBACK)
      s16_mask = 0x0020; // Check Fanuc Battery Low-Voltage Fault in Command Data Field
   else if (LVAR(AX0_s32_Feedback_Ptr) == &SK_COMMUNICATION_FEEDBACK)
      s16_mask = 0x01000; // Check Sankyo Battery Low-Voltage Fault in Command Data Field
   else if (LVAR(AX0_s32_Feedback_Ptr) == &YS_COMMUNICATION_FEEDBACK)
      s16_mask = 0x0004; // Check Yaskawa Battery Low-Voltage Fault in Status Field

   // For Tamagawa and Panasonic Absolute Encoders, sensing Battery Low-Voltage Warning by
   // Bit 7 of ALMC Data Field.  Issue Warning only if CRC OK, Initialization is finished,
   // and no Battery Low-Voltage Fault.
   if ( ( (LVAR(AX0_s32_Feedback_Ptr) == &PS_S_COMMUNICATION_FEEDBACK) ||
          (LVAR(AX0_s32_Feedback_Ptr) == &TM_COMMUNICATION_FEEDBACK)   ||
          (LVAR(AX0_s32_Feedback_Ptr) == &SK_COMMUNICATION_FEEDBACK)     )          &&
        ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_INVALID_MASK) == 0) &&
        ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) != 0)       &&
        ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x08000) == 0x08000)                  &&
        ((BGVAR(s64_SysNotOk) & ABS_ENC_BATT_LOW_MASK) == 0)                          )
      BGVAR(u64_Sys_Warnings) |= TM_BATT_LOW_VOLTAGE_WRN;
   else
      BGVAR(u64_Sys_Warnings) &= ~TM_BATT_LOW_VOLTAGE_WRN;

   if ( (mode == FLT_CLR)                                           &&
        ((BGVAR(s64_SysNotOk) & ABS_ENC_BATT_LOW_MASK) != 0)        &&
        ((BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) & 0x8000) != 0x8000)       )
   { // Allow Fault Reset only if Flag indicates that TMTURNRESET ran AFTER the Fault was set.
      if ( ( ( (LVAR(AX0_s32_Feedback_Ptr) == &TM_COMMUNICATION_FEEDBACK)  ||
               (LVAR(AX0_s32_Feedback_Ptr) == &PS_S_COMMUNICATION_FEEDBACK)  ) &&
             ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x08000) == 0x0000)         ) ||
           ( (LVAR(AX0_s32_Feedback_Ptr) == &NK_COMMUNICATION_FEEDBACK) &&
             ((VAR(AX0_u16_Abs_Enc_Info_Frame) & 0xC000) == 0x0000)        )          )
      {
         VAR(AX0_u16_Abs_Enc_Data_3Frame) = 0;
         VAR(AX0_u16_Abs_Enc_Info_Frame) = 0;
         BGVAR(u16_CommFdbk_Init_Select) = 0;
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
      }
   }

   if ( ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_INVALID_MASK) == 0) /* &&
        ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) != 0)       */   )
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   { // If no CRC or Time-Out Error and Comm. Feedback Initialization Complete check Battery Fault
      if ( (LVAR(AX0_s32_Feedback_Ptr) == &TM_COMMUNICATION_FEEDBACK)   ||
           (LVAR(AX0_s32_Feedback_Ptr) == &PS_S_COMMUNICATION_FEEDBACK)   )
      {
         temp_var = ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & s16_mask) != 0);
         if ( (temp_var != 0) && (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 1) )
         // If TMTURNRESET is in progress do not raise the Flag requiring it again...
            BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) |= 0x8000;
      }
      else if (LVAR(AX0_s32_Feedback_Ptr) == &NK_COMMUNICATION_FEEDBACK)
      {
         temp_var = ((VAR(AX0_u16_Abs_Enc_Info_Frame) & 0xE000) != 0);
         if ( (temp_var != 0) && (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 1) )
         // If TMTURNRESET is in progress do not raise the Flag requiring it again...
            BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) |= 0x8000;
      }
      else if (LVAR(AX0_s32_Feedback_Ptr) == &FANUC_COMMUNICATION_FEEDBACK)
         temp_var = ((VAR(AX0_u16_Abs_Enc_Data_0Frame) & s16_mask) != 0);
      else if (LVAR(AX0_s32_Feedback_Ptr) == &SK_COMMUNICATION_FEEDBACK)
      {
         temp_var = ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & s16_mask) != 0);
         if ( (temp_var != 0) ) // && (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 1) )
         // If TMTURNRESET is in progress do not raise the Flag requiring it again...
            BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) |= 0x8000;
      }
      else if (LVAR(AX0_s32_Feedback_Ptr) == &YS_COMMUNICATION_FEEDBACK)
         temp_var = VAR(AX0_u16_Abs_Enc_Info_Frame) & s16_mask;
   }
   else
      temp_var = 0;

   if (u16_Ignore_Abs_Fdbk_Batt_Faults) //On a PRESTO request - an ability to mask the fault
   {
      temp_var = 0; 
      BGVAR(u64_Sys_Warnings) &= ~TM_BATT_LOW_VOLTAGE_WRN;
      BGVAR(s64_SysNotOk) &= ~ABS_ENC_BATT_LOW_MASK;
   }
   
   if ((mode != FLT_CLR) || ((BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) & 0x8000) == 0))
   // CLEARFAULT or K-EN Action may have cleared the Battery-Fault Indication in the
   // Encoder; prevent clearing the CDHD Battery-Fault Indication until Multi-Turn Reset
   // is performed.
      FeedbackFault(drive, mode, ABS_ENC_BATT_LOW_MASK, (unsigned int *)(&s16_AbsEnc_BattFilter + drive), (unsigned int)u32_bg_interval_time, 30, temp_var, 0, 0);
}


void TamagawaAbsEncoderFault(int drive, int mode)
{
   // AXIS_OFF;

   int temp_var = 0, s16_mask = 0x02400; // Check Counting Error and Multi-Turn Error in Status Field
   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if (MaskedFault(drive, TAMAGAWA_ABS_ENC_FLT_MASK, 0)) return;

   if ((mode == FLT_CLR) && (BGVAR(s64_SysNotOk) & TAMAGAWA_ABS_ENC_FLT_MASK))
   {
      VAR(AX0_u16_Abs_Enc_Data_3Frame) = 0;
      BGVAR(u16_CommFdbk_Init_Select) = 0;
      BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
   }

   // Don't check fault in case of CRC error, the data might be corrupted
   if (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_INVALID_MASK) temp_var = 0;
   else temp_var = VAR(AX0_u16_Abs_Enc_Data_3Frame) & s16_mask;

   // Dont check fault during encoder init
   if (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) temp_var = 0;

   FeedbackFault(drive, mode, TAMAGAWA_ABS_ENC_FLT_MASK, (unsigned int *)(&u16_TamagawaAbsFilter + drive), (unsigned int)u32_bg_interval_time, 30, temp_var, 0, 0);
}


void PanasonicS_EncoderFault(int drive, int mode)
{
   // AXIS_OFF;

   int temp_var = 0, s16_mask = 0x02400; // Check Counting Error and Multi-Turn Error in Status Field
   unsigned long u32_bg_interval_time;
   static unsigned long u32_cntr_1ms_capture;

   u32_bg_interval_time = Cntr_1mS - u32_cntr_1ms_capture;
   u32_cntr_1ms_capture = Cntr_1mS;

   if (MaskedFault(drive, PANASONIC_S_ENC_FLT_MASK, 1)) return;

   if ((mode == FLT_CLR) && (BGVAR(s64_SysNotOk_2) & PANASONIC_S_ENC_FLT_MASK))
   {
      VAR(AX0_u16_Abs_Enc_Data_3Frame) = 0;
      BGVAR(u16_CommFdbk_Init_Select) = 0;
      BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
   }

   // Don't check fault in case of CRC error, the data might be corrupted
   if (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_INVALID_MASK) temp_var = 0;
   else temp_var = VAR(AX0_u16_Abs_Enc_Data_3Frame) & s16_mask;

   // Don't check fault during encoder init
   if (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) temp_var = 0;

   FeedbackFault(drive, mode, PANASONIC_S_ENC_FLT_MASK, (unsigned int *)(&u16_TamagawaAbsFilter + drive), (unsigned int)u32_bg_interval_time, 30, temp_var, 0, 1);
}


//**********************************************************
// Function Name: SwSineQuadFault
// Description:
//   Detects sine quad mismatch fault
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void SwSineQuadFault(int drive, int mode)
{
   // AXIS_OFF;
   int temp_var = 0;

   if (MaskedFault(drive, SW_SINE_QUAD_FLT_MASK, 0)) return;

   if ( (mode == FLT_CLR)                                                     ||
        ((BGVAR(s16_DisableInputs) & (ENDAT_DIS_MASK | HIFACE_DIS_MASK)) != 0)  )
      VAR(AX0_s16_Sine_Enc_Bits) &= ~SW_SINE_QUAD_MISMATCH_MASK;

   if ((VAR(AX0_s16_Sine_Enc_Bits) & SW_SINE_QUAD_MISMATCH_MASK) != 0 && Enabled(drive))
      temp_var = 1;
   FeedbackFault(drive, mode, SW_SINE_QUAD_FLT_MASK, NULL, 0, 0, temp_var, 1, 0);
}


//**********************************************************
// Function Name: SrvSnsFault
// Description:
//   Detects the ServoSense fault
//
// Author: A.O.
// Algorithm:
// Revisions:
//**********************************************************
void SrvSnsFault(int drive, int mode)
{
   static unsigned int u16_flt_read_state = 0;
   static long s32_ss_temp_time_capture = 0;
   static unsigned long u32_owner = 0;
   static unsigned long u32_srvSns_active_faults_shadow = 0;
   static unsigned long u32_srvSns_active_wrn_shadow = 0;
   static unsigned int u16_flt_bit = 1;
   static unsigned int u16_wrn_bit = 1;
   unsigned int u16_flt_found, u16_can_error_code;
   unsigned long long u64_dsp_flt_bit;  
   int u16_ret_val = SAL_NOT_FINISHED;
   long s32_srvsns_response = 0L;
   int temp_var = 0;
   int fdbk_source = 0;
   
   // AXIS_OFF;

   if (!FEEDBACK_SERVOSENSE)   /* Do not continue if ServoSense is not in use */
   {
      BGVAR(u64_Sys_Warnings) &= ~SRVSNS_ENCODER_WRN_MASK;
      BGVAR(s64_SysNotOk_2) &= ~SRVSNS_ENCODER_FLT_MASK;
      return;
   }
   
   if (SERVOSENSE_SINGLE_TURN_COMM_FDBK == BGVAR (u16_FdbkType))
      fdbk_source = 0;   
   else if (SERVOSENSE_SINGLE_TURN_COMM_FDBK == BGVAR (u16_SFBType))
      fdbk_source = 1;


   if (SrvSns_GetStatus(drive) & SRVSNS_STATUS_WARNING_MASK)   // Check ServoSense warning status
   {
      BGVAR(u64_Sys_Warnings) |= SRVSNS_ENCODER_WRN_MASK;
   }
   else
   {
      // remove generic warning and unique warnings for servo sense
      BGVAR(u64_Sys_Warnings) &= ~(SRVSNS_ENCODER_WRN_MASK | SRVSNS_DSP_UNIQUE_WRN_MASK);
            
      BGVAR(u32_SrvSns_Active_Warnings) = 0;
   }

   if (MaskedFault(drive, SRVSNS_ENCODER_FLT_MASK, 1)) return;

   if ((mode == FLT_CLR) && (BGVAR(s64_SysNotOk_2) & SRVSNS_ENCODER_FLT_MASK))
   {
      BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;

      SRVSNS_RTMBX_POST_INIT_STATE(VAR(AX0_u16_SrvSns_MBX_Header));      /* Set init state */

      VAR(AX0_u16_SrvSns_Status_Field) &= ~SRVSNS_STATUS_FAULT_MASK;

      // Raise the reset required flag to ensure power reset on comm fdbk stm init.
      VAR(AX0_u16_SrvSns_FWStatus) |= SRVSNS_FW_STATUS_PWR_RESET_REQ_MASK;
	  
	  BGVAR(u32_SrvSns_Active_Faults) = 0;
     // clear he unique srvsns faults. they will be set again if recaptured on next cycle
     BGVAR(s64_SysNotOk_2) &= ~SRVSNS_DSP_UNIQUE_FLT_MASK;
   }

   if (mode == DETECT)
   {
      if (SrvSns_GetStatus(drive) & SRVSNS_STATUS_FAULT_MASK)  // On ServoSense feedback test for "fault" bit indication
   {
      temp_var = 1;
         // init the state machine to read SrvSns faults.
         // only init once for each fault to prevent reading faults from SrvSns constantly. This may
         // block the access to SrvSns service channel
         if ((u16_flt_read_state == 0) && (BGVAR(u32_SrvSns_Active_Faults) == 0))
         {
            u16_flt_read_state++; 
            // in lxm28, dont show general code AL567 on display until we finish to read excat flt code from srvsns
            // then display the unique code or the general one
            /*if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
            {
               u16_Lxm_Display_Flt_Hide = 1;
            }*/
         }
      }
      else
      {
         if (SrvSns_GetStatus(drive) & SRVSNS_STATUS_WARNING_MASK)   // Check ServoSense warning status
         {
            // init the state machine to read SrvSns warnings.
            // only init once for each warning indication to prevent reading warnings from SrvSns constantly. This may
            // block the access to SrvSns service channel
            if ((u16_flt_read_state == 0) && (BGVAR(u32_SrvSns_Active_Warnings) == 0))
            {
               u16_flt_read_state++; 
               // in lxm28, dont show general code AL567 on display until we finish to read excat flt code from srvsns
               // then display the unique code or the general one
               /*if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
               {
                  u16_Lxm_Display_Flt_Hide = 1;
               }*/
            }
         }
      }
   }


   FeedbackFault(drive, mode, SRVSNS_ENCODER_FLT_MASK, NULL, 0, 0, temp_var, 1, 1);


   // this state machine reads the servo-sense faults and warnings into a p-params
   switch (u16_flt_read_state)
   {
      case 0:
         // do nothing
         break;
         
      case 1:
         u16_flt_bit = 1;
         u16_wrn_bit = 1;
         s32_ss_temp_time_capture = Cntr_1mS;
         u16_flt_read_state++;
         break;

      case 2: // Acquire ServoSense
         // Acquire ServoSense device
         u16_ret_val = SrvSns_Acquire(&u32_owner, drive);
         if(u16_ret_val == SAL_SUCCESS)
         {
            u16_flt_read_state++;
            break;
         }
         
         if (PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, s32_ss_temp_time_capture))
         {
            // error, cannot take semaphore
            u16_flt_read_state = 0;
            //u16_Lxm_Display_Flt_Hide = 0; // show generic fault/warning
         }
         
         break;
         
      case 3:  // read faults from srvsns
         u16_ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_ADDR_CMD_GET_FLTS, &s32_srvsns_response, drive, fdbk_source);
         if (u16_ret_val == SAL_SUCCESS)
         {
            BGVAR(u32_SrvSns_Active_Faults) = (unsigned long)s32_srvsns_response;
            u16_flt_read_state++; // to go to next step of reading warnings
            
            // init for next state
            u32_srvSns_active_faults_shadow = BGVAR(u32_SrvSns_Active_Faults);
         }
         else if (u16_ret_val != SAL_NOT_FINISHED)
         {
            // if error
            SrvSns_Release(&u32_owner, drive);         // Release ServoSense device

            // if only error
            //u16_Lxm_Display_Flt_Hide = 0;
            u16_flt_read_state = 0;
         }
         
         break;
         
      case 4:  // read warnings from srvsns
         u16_ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_ADDR_CMD_GET_WRNS, &s32_srvsns_response, drive, fdbk_source);
         if (u16_ret_val == SAL_SUCCESS)
         {
            SrvSns_Release(&u32_owner, drive);         // Release ServoSense device
            BGVAR(u32_SrvSns_Active_Warnings) = (unsigned long)s32_srvsns_response;
            u16_flt_read_state++; // to go to next step of reading warnings
            
            // init for next state
            u32_srvSns_active_wrn_shadow = BGVAR(u32_SrvSns_Active_Warnings);
         }
         else if (u16_ret_val != SAL_NOT_FINISHED)
         {
            // if error
            SrvSns_Release(&u32_owner, drive);         // Release ServoSense device
            //u16_Lxm_Display_Flt_Hide = 0;
            u16_flt_read_state = 0;
         }
         
         break;
         
      case 5:   
         // update sys_not_ok with unique faults from srvsns (as read in state 3)
         // in addition to the general fault (SRVSNS_ENCODER_FLT_MASK)
         // another uniqe fault will be set for each bit that is read from the srvsns fault register
         if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
         {
            u16_flt_read_state = 0;
         }
         else
         {
            if ((u32_srvSns_active_faults_shadow & SRVSNS_UNIQUE_FLT_MASK) == 0)
            {
               // no more faults to process. this is end of state
               u16_flt_read_state++;
            }
            else
            {
               u16_flt_found = 1;
               switch (u32_srvSns_active_faults_shadow & SRVSNS_UNIQUE_FLT_MASK & u16_flt_bit)
               {
                  case SRVSNS_FAULT_BAD_POSITION_MASK:
                     u64_dsp_flt_bit = SRVSNS_BAD_POSITION_FLT_MASK;
                     u16_can_error_code = ERRCODE_CAN_SRVSNS_BAD_POSITION;
                     break;
                     
                  case SRVSNS_FAULT_OVER_TEMP_MASK:
                     u64_dsp_flt_bit = SRVSNS_OVER_TEMP_FLT_MASK;
                     u16_can_error_code = ERRCODE_CAN_SRVSNS_OVER_TEMP;
                     break;
                     
                 case SRVSNS_FAULT_POWER_DOWN_MASK:
                     u64_dsp_flt_bit = SRVSNS_POWER_DOWN_FLT_MASK;
                     u16_can_error_code = ERRCODE_CAN_SRVSNS_POWER_DOWN;
                     break;
                     
                 case SRVSNS_FAULT_MT_BATT_DWN_MASK:
                     u64_dsp_flt_bit = SRVSNS_MT_BATT_DWN_FLT_MASK;
                     u16_can_error_code = ERRCODE_CAN_SRVSNS_MT_BATT_DWN;
                     break;
                     
                 case SRVSNS_FAULT_MT_CONFIG_REQ_MASK:
                     u64_dsp_flt_bit = SRVSNS_MT_CONFIG_REQ_FLT_MASK;
                     u16_can_error_code = ERRCODE_CAN_SRVSNS_MT_CONFIG_REQ;
                     break;
                     
                 case SRVSNS_FAULT_MT_OUT_OF_SYNC_MASK:
                     u64_dsp_flt_bit = SRVSNS_MT_OUT_OF_SYNC_FLT_MASK;
                     u16_can_error_code = ERRCODE_CAN_SRVSNS_MT_OUT_OF_SYNC;
                     break;
                         
                 case SRVSNS_FAULT_MT_FAILURE_MASK:
                     u64_dsp_flt_bit = SRVSNS_MT_FAILURE_FLT_MASK;
                     u16_can_error_code = ERRCODE_CAN_SRVSNS_MT_FAILURE;
                     break;
                      
                 case SRVSNS_FAULT_HW_FW_MISMATCH_MASK:
                     u64_dsp_flt_bit = SRVSNS_HW_FW_MISMATCH_FLT_MASK;
                     u16_can_error_code = ERRCODE_CAN_SRVSNS_HW_FW_MISMATCH;
                     break;
                         
                  default:
                     u16_flt_found = 0;
                     break;
               }
               
               if (u16_flt_found)
               {
                  // if faults were found which should be displayed as unique, do it now
                  if (HandleFault(drive, u64_dsp_flt_bit, 1))
                  {
                     p402_error_code = u16_can_error_code;
                     CANFaultControl(drive, DETECT, u64_dsp_flt_bit, 1);
                  }
                  
                  //u16_Lxm_Display_Flt_Hide = 0;
               }                     
               
               // set it off to allow this state to end
               u32_srvSns_active_faults_shadow &= ~u16_flt_bit;
               
               u16_flt_bit = u16_flt_bit << 1;
            }
}

         break;
         
      case 6:
         // check if unique warnings exists.
         // for each unique warning a bit is set in u64_Sys_Warnings in addition to the general bit SRVSNS_ENCODER_WRN_MASK
         if (u32_srvSns_active_wrn_shadow & SRVSNS_UNIQUE_WRN_MASK)
         {
            u16_flt_found = 1;
            switch (u32_srvSns_active_wrn_shadow & SRVSNS_UNIQUE_WRN_MASK & u16_wrn_bit)
            {
               case SRVSNS_WRN_HIGH_TEMP_MASK:
                     BGVAR(u64_Sys_Warnings) |= SRVSNS_HIGH_TEMP_WRN_MASK;
                     break;
                     
                  case SRVSNS_WRN_FLASH_WARN_MASK:
                     BGVAR(u64_Sys_Warnings) |= SRVSNS_FLASH_WRN_MASK;
                     break;

                  case SRVSNS_WRN_OVER_ECC_WARN_MASK:
                     BGVAR(u64_Sys_Warnings) |= SRVSNS_OVER_ECC_WRN_MASK;
                     break;
                                       
               default:
                  u16_flt_found = 0;
                  break;
            }
            
            // set it off to allow this atate to end
            u32_srvSns_active_wrn_shadow &= ~u16_wrn_bit;
            u16_wrn_bit = u16_wrn_bit << 1;
            
            /*if (u16_flt_found)
            {
               u16_Lxm_Display_Flt_Hide = 0;
            }*/
         }
         else  // no active warnings
         {
            // no more warnings to process. this is end of state machine (go to default)
            u16_flt_read_state++;
         }
         
         break;                 
      
      
      default:
         u16_flt_read_state = 0;
         break;
   }
}


//**********************************************************
// Function Name: MTPFault
// Description:
//   Inidcate a fault with the Motor-Type-Plate reading
//
// Author: S.F.
// Algorithm:
// Revisions:
//**********************************************************
void MTPFault(int drive, int mode)
{
   if (MaskedFault(drive, MTP_FLT_MASK, 1)) return;

   if (mode == FLT_CLR)
   {
      if (BGVAR(s16_MTP_Error))
      {
         BGVAR(u16_Init_From_MTP) = 1; // This will initiate MTP read
         BGVAR(s16_DisableInputs) |= MTP_READ_DIS_MASK;
      }
      BGVAR(s16_MTP_Error) = 0;
      BGVAR(s64_SysNotOk_2) &= ~MTP_FLT_MASK;
      BGVAR(u16_MTP_Load_Done) = 0;
   }
   if ((mode == DETECT) && BGVAR(s16_MTP_Error))
   {
      if (HandleFault(drive, MTP_FLT_MASK, 1))
      {
         p402_error_code = ERRCODE_CAN_MTP;
         CANFaultControl(drive, DETECT, MTP_FLT_MASK, 1);
      }
   }
}


//**********************************************************
// Function Name: DriveMotorMismatchFault
// Description:
//   Used for SE to detect if a valid drive/motor bundle is selected
//
// Author: S.F.
// Algorithm:
// Revisions:
//**********************************************************
void DriveMotorMismatchFault(int drive, int mode)
{
   if (MaskedFault(drive, DRIVE_MOTOR_MISMATCH_FLT_MASK, 1)) return;

   if ((BGVAR(u16_MTP_Mode) != 1) || (BGVAR(u16_Bundle_Check) == 0))
   {
      // This is to avoid enabling the drive if bundles check if avoided (MTPMODE=h1234)
      if (BGVAR(s64_SysNotOk_2) & DRIVE_MOTOR_MISMATCH_FLT_MASK)
      {
         BGVAR(s16_DisableInputs) |= SW_EN_MASK;
         if ((IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1) && ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)))
         {
            BGVAR(s16_DisableInputs) |= FB_EN_MASK;
         }
      }

      BGVAR(s64_SysNotOk_2) &= ~DRIVE_MOTOR_MISMATCH_FLT_MASK;
   }
   else
   {
      if (mode == FLT_CLR)
      {
         // To clear the fault the MTP read is re-init to recheck the matching
         if ((BGVAR(u16_MTP_Mode)==1) &&
            ((BGVAR(s64_SysNotOk_2) & DRIVE_MOTOR_MISMATCH_FLT_MASK) != 0))
         {//  read the MTP only if MTPmode == 1 && motor mismatch fault is active
            BGVAR(u16_Init_From_MTP) = 1; // This will initiate MTP read
            BGVAR(s16_DisableInputs) |= MTP_READ_DIS_MASK;
            BGVAR(s16_Motor_Drive_Mismatch) = 0;
         }
         BGVAR(s64_SysNotOk_2) &= ~DRIVE_MOTOR_MISMATCH_FLT_MASK;
      }
      if ((mode == DETECT) && BGVAR(s16_Motor_Drive_Mismatch))
      {
         if (HandleFault(drive, DRIVE_MOTOR_MISMATCH_FLT_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_DRIVE_MOTOR_MISMATCH;
            CANFaultControl(drive, DETECT, DRIVE_MOTOR_MISMATCH_FLT_MASK, 1);
         }
      }
   }
}


//**********************************************************
// Function Name: InvalidSininitFault
// Description:
//   Detects invalid sinparam fault
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void InvalidSininitFault(int drive, int mode)
{
   // AXIS_OFF;
   int temp_var = 0;

   if (MaskedFault(drive, SININIT_INVALID_MASK, 0))
   {
      // Remove Warning
      BGVAR(u64_Sys_Warnings) &= ~SININIT_WARNING_MASK;

      return;
   }

   if ((mode == FLT_CLR) && ((BGVAR(s64_SysNotOk) & SININIT_INVALID_MASK) != 0))
   {  // On Resolver restart the resolver ref adapt procedure
      if (BGVAR(u16_FdbkType) == RESOLVER_FDBK)
         BGVAR(s16_Adjust_Reference_State) = ADJ_REF_INIT;
      //SininitCommand(drive);
   }

   temp_var = VAR(AX0_s16_Sine_Gain_Fix) >> (VAR(AX0_s16_Sine_Gain_Shr) - 3);
   temp_var = ( (temp_var < 7) || (temp_var > 9)   || // Check Gain Deviation +/- 1/8 of Nominal value
                (VAR(AX0_s16_Sine_Offset) > 2000)  || (VAR(AX0_s16_Cosine_Offset) > 2000) ||
                (VAR(AX0_s16_Sine_Offset) < -2000) || (VAR(AX0_s16_Cosine_Offset) < -2000)  );
                // Test for Offsets larger than 1/16 of Full-Scale
   if ((temp_var == 0) && (BGVAR(u16_FdbkType) == RESOLVER_FDBK))
   {
      temp_var = VAR(AX0_s16_Swr2d_Fix) >> (VAR(AX0_s16_Swr2d_Shr) - 1);
      temp_var = ( (temp_var < 2) || (temp_var > 4)                     ||
                   (BGVAR(s16_Adjust_Reference_State) == ADJ_REF_FAILED)  );
   }

   FeedbackFault(drive, mode, SININIT_INVALID_MASK, NULL, 0, 0, temp_var, 0, 0);

   if (temp_var) // Remove Warning when Fault is set
   {
       BGVAR(u64_Sys_Warnings) &= ~SININIT_WARNING_MASK;
       VAR(AX0_s16_Sine_Enc_Bits) &= ~SINE_ZERO_RUNNING_MASK; 
       VAR(AX0_s16_Sine_Enc_Bits) &= ~RESET_SINE_ZERO_MASK;   //terminate the sininit SM
   }
   else
   { // If no Fault check for Warning Conditions
      temp_var = VAR(AX0_s16_Sine_Gain_Fix) >> (VAR(AX0_s16_Sine_Gain_Shr) - 4);
      temp_var = ( (temp_var < 15) || (temp_var > 17) || // Gain Deviation +/- 1/16 of Nominal value
                   (VAR(AX0_s16_Sine_Offset) > 1000)  || (VAR(AX0_s16_Cosine_Offset) > 1000) ||
                   (VAR(AX0_s16_Sine_Offset) < -1000) || (VAR(AX0_s16_Cosine_Offset) < -1000)  );
                // Test for Offsets larger than 1/32 of Full-Scale
      if ((temp_var == 0) && (BGVAR(u16_FdbkType) == RESOLVER_FDBK))
      {
         temp_var = VAR(AX0_s16_Swr2d_Fix) >> (VAR(AX0_s16_Swr2d_Shr) - 3);
         temp_var = ( (temp_var < 7) || (temp_var > 9) );
      }
      if (temp_var) // Remove Warning when Fault is set
         BGVAR(u64_Sys_Warnings) |= SININIT_WARNING_MASK;
      else
         BGVAR(u64_Sys_Warnings) &= ~SININIT_WARNING_MASK;
   }
}


//**********************************************************
// Function Name: ResolverInitFault
// Description:
//   Test if Resolver adjust procedure succeeded
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
void ResolverInitFault(int drive, int mode)
{
   int temp_var = 0;

   if (MaskedFault(drive, RESOLVER_REF_FLT_MASK, 0)) return;

   if ((mode == FLT_CLR) && ((BGVAR(s64_SysNotOk) & RESOLVER_REF_FLT_MASK) != 0))
      BGVAR(s16_Adjust_Reference_State) = ADJ_REF_INIT;

   temp_var = (BGVAR(s16_Adjust_Reference_State) == ADJ_REF_FAILED);

   FeedbackFault(drive, mode, RESOLVER_REF_FLT_MASK, NULL, 0, 0, temp_var, 0, 0);
}


//**********************************************************
// Function Name: NvramFault
// Description:
//   Detects EEPROM sinparam fault
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void NvramFault(int drive, int mode)
{
   if (MaskedFault(drive, NVRAM_FAULT, 0)) return;

   if ( (mode == DETECT) && (s16_Flash_Fault_Flag) )
   {
      if (HandleFault(drive, NVRAM_FAULT, 0))
      {
         p402_error_code = ERRCODE_CAN_NVRAM;
         CANFaultControl(drive, 0, NVRAM_FAULT, 0);
      }
   }
}
//   int s16_prev_state = 0;

//**********************************************************
// Function Name: FieldBusEtherCATCableDisconnectedFault
// Description:
//   Detects EtherCAT cable disconnected Fault
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void FieldBusEtherCATCableDisconnectedFault(int drive, int mode)
{
   static int s16_is_first_cable_connected = 0;
   static int s16_is_reach_safeop = 0;
   if (MaskedFault(drive, ETHERCAT_CABLE_DISCONNECTED_MASK, 1)) return;
   if (mode == FLT_CLR)
      BGVAR(s64_SysNotOk_2) &= ~ETHERCAT_CABLE_DISCONNECTED_MASK;

   if (!IS_EC_DRIVE_AND_COMMODE_1)
      return;

   SET_DPRAM_FPGA_MUX_REGISTER(0);

   //do not issue cable disconnected fault before first identification of a connected cable
   if (!(u16_EC_commands_register & EC_CWORD_CABLE_DISCONNECTED_MASK))
      s16_is_first_cable_connected = 1;

   if (!s16_is_first_cable_connected)
      return;

   // issue FB3 fault only if reach at least SAFEOP mode
   if ((*p_u16_tx_nmt_state >= EC_NMTSTATE_SAFEOP) && ((BGVAR(s64_SysNotOk_2) & ETHERCAT_CABLE_DISCONNECTED_MASK) == 0))
      s16_is_reach_safeop = 1;

   if (!s16_is_reach_safeop)
      return;

   if ((mode == DETECT) && (IS_EC_DRIVE_AND_COMMODE_1) && (u16_EC_commands_register & EC_CWORD_CABLE_DISCONNECTED_MASK))  // EtherCAT - cable disconnected
   {
      if (HandleFault(drive, ETHERCAT_CABLE_DISCONNECTED_MASK, 1))
      {
        s16_is_reach_safeop = 0; //reset flag
         p402_error_code = ERRCODE_CAN_ETHERCAT_CABLE_DISCONNECTED;
         CANFaultControl(drive, 0, ETHERCAT_CABLE_DISCONNECTED_MASK, 1);
      }
   }
}


//**********************************************************
// Function Name: FieldBusEtherCATPacketsLostFault
// Description:
//   Detects EtherCAT Packets Lost Fault
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void FieldBusEtherCATPacketsLostFault(int drive, int mode)
{
   unsigned int u16_EC_watchdog_times;
   // AXIS_OFF;

   if (MaskedFault(drive, ETHERCAT_PACKETS_LOST_MASK, 1)) return;
   if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~ETHERCAT_PACKETS_LOST_MASK;
   }

   if (!IS_EC_DRIVE_AND_COMMODE_1)
      return;

// 5 * sync time * the times that function ExecuteFieldbusRtRxObject runs is one second (8000)
   u16_EC_watchdog_times =
   MultS64ByFixU32ToS64((long long)40000, BGVAR(fbus_cycle_time_in_secs).s32_fbus_cycle_time_fix,BGVAR(fbus_cycle_time_in_secs).u16_fbus_cycle_time_shr);

   if ((mode == DETECT) && (Enabled(0)) && (IS_EC_DRIVE_AND_COMMODE_1)   && (!(BGVAR(u8_Fb_Packet_Loss_Ignore) & FB_PACKET_LOSS_IGNORE))  && (VAR(u16_EtherCat_Watchdog) > u16_EC_watchdog_times))
   {
      VAR(u16_EtherCat_Watchdog) = 0;
      if (HandleFault(drive, ETHERCAT_PACKETS_LOST_MASK, 1))
      {
         p402_error_code = ERRCODE_CAN_PACKETS_LOST_DISCONNECTED;
         CANFaultControl(drive, 0, ETHERCAT_PACKETS_LOST_MASK, 1);
      }
   }
}


//**********************************************************
// Function Name: FieldBusActiveNotOpFault
// Description:
//   Detects EtherCAT/CAN drive is active and not in OP state
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void FieldBusActiveNotOpFault(int drive, int mode)
{
   if (MaskedFault(drive, FIELDBUS_ACTIVE_NOT_OP_MASK, 1)) return;
   if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~FIELDBUS_ACTIVE_NOT_OP_MASK;
     }


   if (((mode == DETECT) && (Enabled(0)) && (IS_EC_DRIVE_AND_COMMODE_1)   && ((BGVAR(s16_Prev_State) == EC_NMTSTATE_OP) && (*(unsigned int*)p_u16_tx_nmt_state != EC_NMTSTATE_OP))) || // EtherCAT - drive is active and NMT state is not OPERATIONAL
      ((mode == DETECT) && (Enabled(0)) && ((IS_CAN_DRIVE_AND_COMMODE_1) && ((BGVAR(s16_Prev_State) == OPERATIONAL)    && (GL_ARRAY(co_Node).eState != OPERATIONAL)) )))                 // CAN      - drive is active and NMT state is not OPERATIONAL
   {
      if (HandleFault(drive, FIELDBUS_ACTIVE_NOT_OP_MASK, 1))
      {
         p402_error_code = ERRCODE_FIELDBUS_ACTIVE_NOT_OP;
         CANFaultControl(drive, 0, FIELDBUS_ACTIVE_NOT_OP_MASK, 1);
      }
   }

   //save previous state
   if (IS_CAN_DRIVE_AND_COMMODE_1)
   {
      BGVAR(s16_Prev_State) = GL_ARRAY(co_Node).eState;
   }
   else if (IS_EC_DRIVE_AND_COMMODE_1)
   {
      SET_DPRAM_FPGA_MUX_REGISTER(0);
      BGVAR(s16_Prev_State) = *(unsigned int*)p_u16_tx_nmt_state;
   }
}


//**********************************************************
// Function Name: RunawayDetectionFault
// Description:
//   Detects a runaway condition most likely caused by a wrong MPHASE  or MPOLES
//   setting.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void RunawayDetectionFault(int drive, int mode)
{
   // AXIS_OFF;

   if (MaskedFault(drive, RUNAWAY_FAULT_DETECTED_MASK, 1)) return;

   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))   // Ignore this fault during Burnin
   {
      BGVAR(s64_SysNotOk_2) &= ~RUNAWAY_FAULT_DETECTED_MASK;
      return;
   }

   if (mode == FLT_CLR)
   {
      BGVAR(u16_Runaway_Error_Detected) = 0;
      BGVAR(s64_SysNotOk_2) &= ~RUNAWAY_FAULT_DETECTED_MASK;
   }
   else if ((mode == DETECT) && (BGVAR(u16_Runaway_Error_Detected)))
   {
      if (HandleFault(drive, RUNAWAY_FAULT_DETECTED_MASK, 1))
      {
         p402_error_code = ERRCODE_CAN_RUNAWAY_FAULT_DETECTED;
         CANFaultControl(drive, DETECT, RUNAWAY_FAULT_DETECTED_MASK, 1);
      }
   }
}


//**********************************************************
// Function Name: FieldBusTargetPositionVelFault
// Description:
//   Detects FieldBus Target Position Fault
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void FieldBusTargetPositionVelFault(int drive, int mode)
{
   // AXIS_OFF;
   if (MaskedFault(drive, FIELDBUS_TARGET_POS_HIGH_VEL_MASK, 0)) return;

   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))   // Ignore this fault during Burnin
   {
      BGVAR(s64_SysNotOk) &= ~FIELDBUS_TARGET_POS_HIGH_VEL_MASK;
      return;
   }

   if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk) &= ~FIELDBUS_TARGET_POS_HIGH_VEL_MASK;
   }

   // Dedicated lower 8-Bits of Flag Variable to counting Excessive Velocity events...
   if ( (mode == DETECT) && ((VAR(AX0_s16_Fieldbus_Target_Postion_Flag) & 0x00FF) > 4) )
   {
      VAR(AX0_s16_Fieldbus_Target_Postion_Flag) &= 0xFF00;
      if (HandleFault(drive, FIELDBUS_TARGET_POS_HIGH_VEL_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_TARGET_POSITION_VEL_TOO_HIGH;
         CANFaultControl(drive, 0, FIELDBUS_TARGET_POS_HIGH_VEL_MASK, 0);
      }
   }
}


//**********************************************************
// Function Name: FieldBusTargetPositionAccDecFault
// Description:
//   Detects FieldBus Target Position ACC/DEC Fault
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void FieldBusTargetPositionAccDecFault(int drive, int mode)
{
   drive+=0;
   mode+=0;
/*   // AXIS_OFF;
   if (MaskedFault(drive, FIELDBUS_TARGET_POS_HIGH_ACC_DEC_MASK, 1)) return;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk_2) &= ~FIELDBUS_TARGET_POS_HIGH_ACC_DEC_MASK;
      return;
   }

   if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~FIELDBUS_TARGET_POS_HIGH_ACC_DEC_MASK;
   }

   // Dedicated upper 8-Bits of Flag Variable to counting Excessive Acc/Dec events...
   if ( (mode == DETECT) && ((VAR(AX0_s16_Fieldbus_Target_Postion_Flag) & 0xFF00) > 0x0400) )
   {
      VAR(AX0_s16_Fieldbus_Target_Postion_Flag) &= 0x00FF;
      if (HandleFault(drive, FIELDBUS_TARGET_POS_HIGH_ACC_DEC_MASK, 1))
      {
         p402_error_code = ERRCODE_CAN_TARGET_POSITION_ACC_DEC_TOO_HIGH;
         CANFaultControl(drive, 0);
      }
   }
*/
}


//**********************************************************
// Function Name: FieldBusTargetCommandLostFault
// Description:
//   Detects FieldBus Target command lost Fault
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void FieldBusTargetCommandLostFault(int drive, int mode)
{
   int s16_max_allowed_missing_target_cmd = 3;
   // take snapshot of object 0x6061 since it can be modified via PDO.
   char s8_canopen_opmode_display_shadow = p402_modes_of_operation_display;

   // AXIS_OFF;

   if (MaskedFault(drive, FIELDBUS_TARGET_COMMAND_LOST_MASK, 1)) return;
   if (BGVAR(u8_Comm_Mode) == 0) return;

   if ( (mode == DETECT) && (VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) <= s16_max_allowed_missing_target_cmd) )
   {
      BGVAR(u64_Sys_Warnings) &= ~WRN_FIELDBUS_TARGET_COMMAND_LOST;
   }

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk_2) &= ~FIELDBUS_TARGET_COMMAND_LOST_MASK;
      return;
   }

   if (mode == FLT_CLR)
   {
      // reset the missing commands counter
      VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) = 0;

      BGVAR(s64_SysNotOk_2) &= ~FIELDBUS_TARGET_COMMAND_LOST_MASK;
   }

   if (BGVAR(u8_Fb_Packet_Loss_Ignore & FB_PACKET_LOSS_IGNORE)) return;

   if (((s8_canopen_opmode_display_shadow != CYCLIC_SYNCHRONOUS_POSITION_MODE) &&
       (s8_canopen_opmode_display_shadow != INTRPOLATED_POSITION_MODE)        &&
       (s8_canopen_opmode_display_shadow != CYCLIC_SYNCHRONOUS_VELOCITY_MODE) &&
       (s8_canopen_opmode_display_shadow != CYCLIC_SYNCHRONOUS_TORQUE_MODE)) || (!Enabled(0)))
   {
      // this fault can be set only for synchronous mode
      // zero counter to avoid fault when switching opmode back into sync mode.
      VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) = 0;
      return;
   }

   if ( (mode == DETECT) && (VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) > s16_max_allowed_missing_target_cmd) )
   {
      // keep the value in fault/warning state
      VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) = s16_max_allowed_missing_target_cmd + 1;

      // in lxm28, set warning if access right is not set to fieldbus (e.g. modbus tool takes access rights).
      if (((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)) &&
          (BGVAR(u16_Lexium_Acces_Rights_State) != ((LEX_CH_FIELDBUS<<8) & 0xFF00)))
      {
         BGVAR(u64_Sys_Warnings) |= WRN_FIELDBUS_TARGET_COMMAND_LOST;
      }
      else
      {
         if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
         {
            // if warning exists, reset missing-commands counter, in order to allow the counter to increment again
            // this is to avoid gettng fault immediatly when changing access rights from modbus back to plc
            if (BGVAR(u64_Sys_Warnings) & WRN_FIELDBUS_TARGET_COMMAND_LOST)
            {
               BGVAR(u64_Sys_Warnings) &= ~WRN_FIELDBUS_TARGET_COMMAND_LOST;
               VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) = 0;
               return;
            }
         }

         if (HandleFault(drive, FIELDBUS_TARGET_COMMAND_LOST_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_FIELDBUS_TARGET_COMMAND_LOST;
            CANFaultControl(drive, 0, FIELDBUS_TARGET_COMMAND_LOST_MASK, 1);
         }
      }
   }
}


//**********************************************************
// Function Name: NvramChecksumFault
// Description:
//   Detects NVRAM checksum fault
//
// Author: D.R
// Algorithm:
// Revisions:
//**********************************************************
void NvramChecksumFault(int drive, int mode)
{
   if (MaskedFault(drive, NVRAM_CHECKSUM_FAULT, 0)) return;

   if ( (mode == DETECT) && (BGVAR(s16_Flash_Checksum_Fault_Flag)) )
   {
      if (HandleFault(drive, NVRAM_CHECKSUM_FAULT, 0))
      {
         p402_error_code = ERRCODE_CAN_NVRAM_CHECKSUM;
         CANFaultControl(drive, 0, NVRAM_CHECKSUM_FAULT, 0);
      }

      BGVAR(s16_Flash_Checksum_Fault_Flag) = 0;
      BGVAR(s64_SysNotOk)     |= NO_COMP_FLT_MASK;
      BGVAR(u8_Comms_Options) = (ECHO_CONTROL_MASK | PROMPT_MASK | ERR_MSG_MASK);
   }

   if ( (mode == FLT_CLR) && (!BGVAR(s16_Flash_Checksum_Fault_Flag)) )
   {
      BGVAR(s64_SysNotOk) &= ~NVRAM_CHECKSUM_FAULT;
   }
}


void VbusMeasureFault(int drive, int mode)
{
   int fault_bit = u16_Vbus_HW_Measure_Flt;
   if (MaskedFault(drive, VBUS_MEASURE_FLT_MASK, 0)) return;

   if ( (mode == DETECT) && (fault_bit)             &&
        (!u16_STO_Flt_Indication_Actual)            &&
        (!(BGVAR(u64_Sys_Warnings) & STO_WRN_MASK)) &&
        (!u16_STO_Flt_Indication)                   &&
        (!(BGVAR(s64_SysNotOk) & STO_FAULT_MASK))   &&
        (!u16_STO_Just_Inserted)                      )
   {
      ++u16_VbusMeasureFilter;
      if (u16_VbusMeasureFilter == 10)
      {
         if (HandleFault(drive, VBUS_MEASURE_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_VBUS_MEASURE;
            CANFaultControl(drive, 0, VBUS_MEASURE_FLT_MASK, 0);
         }
      }
   }

   if ( (mode == FLT_CLR) && (!fault_bit) )
      BGVAR(s64_SysNotOk) &= ~VBUS_MEASURE_FLT_MASK;

   if (!fault_bit) u16_VbusMeasureFilter = 0;
}


void CrrntSensorsOffsetFault(int drive, int mode)
{
   // AXIS_OFF;

   unsigned int u16_threshold = 0;
   int fault_bit = 0;

   u16_threshold = (BGVAR(s32_Drive_I_Peak) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
             >> BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;

   u16_threshold /= 20; // Threshold is 5% of DIPEAK

   fault_bit = ( (abs(VAR(AX0_s16_Crrnt_U_Ofst)) > u16_threshold) ||
                 (abs(VAR(AX0_s16_Crrnt_V_Ofst)) > u16_threshold)   );

   if (MaskedFault(drive, CUR_SENS_OFST_FLT_MASK, 0)) return;

   if ( (mode == DETECT) && (fault_bit) )
   {
      if (HandleFault(drive, CUR_SENS_OFST_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_CUR_SENS_OFST;
         CANFaultControl(drive, 0, CUR_SENS_OFST_FLT_MASK, 0);
      }

   }

   if ( (mode == FLT_CLR) && (!fault_bit) )
      BGVAR(s64_SysNotOk) &= ~CUR_SENS_OFST_FLT_MASK;
}


void UnderVoltageFault(int drive, int mode)
{
   int tmp = UnderVoltageTest(DRIVE_PARAM);

   if (MaskedFault(drive, UNDER_VOLTAGE_FLT_MASK, 0)) return;

   if ( (mode == DETECT) && (tmp == 1)            && (!(BGVAR(u64_Sys_Warnings) & STO_WRN_MASK))     &&
        (!(BGVAR(s64_SysNotOk) & STO_FAULT_MASK)) && (!(BGVAR(s64_SysNotOk) & VBUS_MEASURE_FLT_MASK))  )
   {
      ++u16_UnderVoltageFilter;
      if (u16_UnderVoltageFilter == 10)
      {
         if (HandleFault(drive, UNDER_VOLTAGE_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_UNDER_VOLTAGE;
            CANFaultControl(drive, 0, UNDER_VOLTAGE_FLT_MASK, 0);
         }
      }
   }

   if ((mode == FLT_CLR) && (tmp == 0)) BGVAR(s64_SysNotOk) &= ~UNDER_VOLTAGE_FLT_MASK;

   if (tmp == 0) u16_UnderVoltageFilter = 0;

   // handle warning
   if (tmp == 2) BGVAR(u64_Sys_Warnings) |=  UNDER_VOLTAGE_WRN_MASK; //Set warning mask
   else BGVAR(u64_Sys_Warnings) &= ~UNDER_VOLTAGE_WRN_MASK; // Clear warning mask

   // handle UVRECOVER - consider hysteresis
   if ( (tmp == 0) && (BGVAR(u16_Uv_Recover) == 1)             &&
        (BGVAR(u16_Vbus_Volts) > (BGVAR(u16_UV_Threshold) + 1))  )
      BGVAR(s64_SysNotOk) &= ~UNDER_VOLTAGE_FLT_MASK;
}

unsigned int u16_ov_debug = 0;
void OverVoltageFault(int drive, int mode)
{
   // AXIS_OFF;
   unsigned int u16_fault_thresh = u16_OV_Threshold;

   // On LV drive (or any other DC drive) this fault is not relevant, currently this is done by checking the Vbus_Scale on the EE
   // This should be finalized in a more precise way
   if (DC_DRIVE)
   {
      if (!SIGNAL_62V_ON) u16_fault_thresh = 95;
   }
   u16_ov_debug = u16_fault_thresh;

   // Provide Momentary Indication for PWM Disabling during Dynamic Braking...
   VAR(AX0_u16_OV_Exists) = (BGVAR(u16_Vbus_Volts) > u16_fault_thresh);

   if (MaskedFault(drive, OVER_VOLTAGE_FLT_MASK, 0)) return;

   // Rely on Over-Voltage Condition from 1mSec. Interrupt Code
   if ( (mode == DETECT) && (((BGVAR(s64_RT_Faults) & OVER_VOLTAGE_FLT_MASK) > 0) || VAR(AX0_u16_OV_Exists)) && (!(BGVAR(u64_Sys_Warnings) & STO_WRN_MASK))   &&
        !(BGVAR(s64_SysNotOk) & (STO_FAULT_MASK | VBUS_MEASURE_FLT_MASK | OVER_VOLTAGE_FLT_MASK))         )
   {
      if (HandleFault(drive, OVER_VOLTAGE_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_OVER_VOLTAGE;
         CANFaultControl(drive, 0, OVER_VOLTAGE_FLT_MASK, 0);
      }
   }

   if ( (mode == FLT_CLR) && (0 == (BGVAR(s64_RT_Faults) & OVER_VOLTAGE_FLT_MASK)))  BGVAR(s64_SysNotOk) &= ~OVER_VOLTAGE_FLT_MASK;
}


// OVTHRESH
int SalReadOVThreshold(long long *data)
{
   *data = (long long)u16_OV_Threshold;

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalOVThresholdCommand
// Description:
//   allows user to set the OV threshold only if CHAKRATEC CUSTOMERID is set
//
// Author: S.C.
// Algorithm:
// Revisions:
//**********************************************************

int SalOVThresholdCommand(long long param,int drive)
{
   
   unsigned int  u16_regen_off_new_value = (unsigned int)param - 20; 
   unsigned int  u16_regen_on_new_value = (unsigned int)param - 10;
 
   REFERENCE_TO_DRIVE; // Defeat compilation remark
  
   if ( CHAKRATEC == u16_Customer_Id )//valid CUSTOMERID
   {
      //new OVThreshold value must be bigger than the UV threshold + 10 (due the regen) and not bigger than the original EEPROM OV_thresh value
      if ( (u16_regen_off_new_value >= u16_UV_Threshold) && ( (unsigned int)param <= u16_Power1_Board_Eeprom_Buffer[OV_THRESH_ADDR] ) )
      {
         u16_Regen_Off_Value = u16_regen_off_new_value;
         u16_Regen_On_Value = u16_regen_on_new_value;
         u16_OV_Threshold = (unsigned int)param;
      }
      else
      {
         return VALUE_OUT_OF_RANGE;
      }
   }
   else
   {
      return WRONG_CUSTOMER_ID;
   }
   return SAL_SUCCESS;
}


void FoldbackFault(int drive, int mode)
{
   // Set the foldback fault/warning state. Defeat it if foldback is not ready.
   int drive_foldback_fault   = (int)( (BGVAR(s32_I_Fold) <= BGVAR(u32_I_Fold_Fault_Threshold)) && (BGVAR(u16_Foldback_Not_Ready) == 0) );
   int drive_foldback_warning = (int)( (BGVAR(s32_I_Fold) <= BGVAR(u32_I_Fold_Warning_Threshold)) && (BGVAR(u16_Foldback_Not_Ready) == 0) );
   int motor_foldback_fault   = (int)( (BGVAR(s32_Motor_I_Fold) <= BGVAR(u32_Motor_I_Fold_Fault_Threshold)) && (BGVAR(u16_Motor_Foldback_Not_Ready) == 0) );
   int motor_foldback_warning = (int)( (BGVAR(s32_Motor_I_Fold) <= BGVAR(u32_Motor_I_Fold_Warning_Threshold)) && (BGVAR(u16_Motor_Foldback_Not_Ready) == 0) );

// Handle warnings
   if (drive_foldback_warning)
      BGVAR(u64_Sys_Warnings) |= DRIVE_FOLDBACK_WRN_MASK;
   else
      BGVAR(u64_Sys_Warnings) &= ~DRIVE_FOLDBACK_WRN_MASK;

   if ( (motor_foldback_warning) && (BGVAR(u16_Motor_Foldback_Disable) == 0) )
      BGVAR(u64_Sys_Warnings) |= MOTOR_FOLDBACK_WRN_MASK;
   else
      BGVAR(u64_Sys_Warnings) &= ~ MOTOR_FOLDBACK_WRN_MASK;

// Handle clear faults
   if ( (mode == FLT_CLR) && (!drive_foldback_fault) )
      BGVAR(s64_SysNotOk) &= ~DRIVE_FOLDBACK_FLT_MASK;
   if ( (mode == FLT_CLR)                                                  &&
        (!motor_foldback_fault || (BGVAR(u16_Motor_Foldback_Disable) == 1))  )
      BGVAR(s64_SysNotOk) &= ~MOTOR_FOLDBACK_FLT_MASK;

// Handle detect faults
   if ( (mode == DETECT) && (drive_foldback_fault)    &&
        !(MaskedFault(drive, DRIVE_FOLDBACK_FLT_MASK, 0))  )
   {
      if (HandleFault(drive, DRIVE_FOLDBACK_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_DRIVE_FOLDBACK;
         CANFaultControl(drive, 0, DRIVE_FOLDBACK_FLT_MASK, 0);
      }
   }
   if ( (mode == DETECT) && (motor_foldback_fault)     &&
        !(MaskedFault(drive, MOTOR_FOLDBACK_FLT_MASK, 0)) &&
        (BGVAR(u16_Motor_Foldback_Disable) == 0)         )
   {
      if (HandleFault(drive, MOTOR_FOLDBACK_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_MOTOR_FOLDBACK;
         CANFaultControl(drive, 0, MOTOR_FOLDBACK_FLT_MASK, 0);
      }
   }
}


void DriveOTFaults(int drive, int mode)
{
   if (!MaskedFault(drive, DRIVE_IPM_OT_FLT_MASK, 0))
      DriveIPMOTFault(drive, mode);

   if (!MaskedFault(drive, DRIVE_POW_OT_FLT_MASK, 0))
      DrivePowerOTFault(drive, mode);

   if (!MaskedFault(drive, DRIVE_DIG_OT_FLT_MASK, 0))
      DriveDigitalOTFault(drive, mode);

   if (!MaskedFault(drive, TEMPERATURE_HW_FLT_MASK, 1))
      DriveTempSensorFailure(drive, mode);
}


void DriveIPMOTFault(int drive, int mode)
{
   // AXIS_OFF;
   unsigned int u16_FPGA_flt_indication = 0, u16_filter_max, u16_sto_temp = 1;
   unsigned int u16_ipm_temperature, u16_ipm_temperature_diff, u16_ipm_temperature_sum, u16_index;
   static unsigned int u16_sto_state_machine = NO_STO_FAULT_STATE;
   static long s32_sto_timer = 0L;

   // Wait 2 secs after STO restore to allow the temperature to settle
   switch (u16_sto_state_machine)
   {
      case NO_STO_FAULT_STATE:
         if (u16_STO_Flt_Indication)
         {
            u16_sto_state_machine = STO_FAULT_STATE;
         }
         else u16_sto_temp = 0;
      break;

      case STO_FAULT_STATE:
         if (!u16_STO_Flt_Indication)
         {
            s32_sto_timer = Cntr_1mS;
            u16_sto_state_machine = STO_FAULT_WAIT_STATE;
         }
      break;

      case STO_FAULT_WAIT_STATE:
         if (u16_STO_Flt_Indication) u16_sto_state_machine = STO_FAULT_STATE;
         else
         {
            if (PassedTimeMS(2300L,s32_sto_timer))
               u16_sto_state_machine = NO_STO_FAULT_STATE;
         }
      break;
   }
   u16_STO_Flt_Indication_Actual_And_Delay = u16_sto_temp;

   if (VAR(AX0_u16_IPM_OT_DC_Enable) == 1)
   {
      if ( (BGVAR(u16_IPM_OT_Prev_Counter) != VAR(AX0_u16_IPM_OT_Counter)) && (VAR(AX0_u16_IPM_OT_Counter) >= 2) )
      {
         do
         {
            BGVAR(u16_IPM_OT_Prev_Counter) = VAR(AX0_u16_IPM_OT_Counter);
            if (VAR(AX0_u16_IPM_OT_Cycle_Time) == 0) VAR(AX0_u16_IPM_OT_Cycle_Time) = 1;
            u16_ipm_temperature =
            (unsigned int)((BGVAR(u16_IPM_Temp_A_Coef)*(long)VAR(AX0_u16_IPM_OT_On_Time)) / VAR(AX0_u16_IPM_OT_Cycle_Time)) + BGVAR(u16_IPM_Temp_B_Coef);
         } while (BGVAR(u16_IPM_OT_Prev_Counter) != VAR(AX0_u16_IPM_OT_Counter));  // Re-calc IPM temp in case new cycle ended during calculation

         // Clac the absolute difference between the newly measured temperaure and the last one, in order to reject temperature
         // readout which are too far from the last temperature.
         if (BGVAR(u16_IPM_Temperature) > u16_ipm_temperature)
         {
            u16_ipm_temperature_diff = BGVAR(u16_IPM_Temperature) - u16_ipm_temperature;
         }
         else
         {
            u16_ipm_temperature_diff = u16_ipm_temperature - BGVAR(u16_IPM_Temperature);
         }

         if ( (BGVAR(u16_IPM_Temperature) == 0)                               ||   // First time, or
              (u16_ipm_temperature_diff < u16_IPM_Temperature_Diff_Threshold) ||   // temperature difference is less than pre-defined threshold (ignore temperature readout if change in one sample more than threshold), or
              (BGVAR(u16_IPM_Temperature_Diff_Counter) > IPM_TEMPERATURE_BUFFER_SIZE) ) // Force buffering in case temp diff for too long
         {
            BGVAR(u16_IPM_Temperature_Buffer)[BGVAR(u16_IPM_Temperature_Index)] = u16_ipm_temperature;  // Store the new temperature in a buffer
            BGVAR(u16_IPM_Temperature_Index)++;                                                         // Advance buffer's index
            if (BGVAR(u16_IPM_Temperature_Index) >= IPM_TEMPERATURE_BUFFER_SIZE)                        // Check if index rollover is needed
            {
               BGVAR(u16_IPM_Temperature_Index) = 0;
               BGVAR(u16_IPM_Temperature_Buffer_Flag) = 1;    // Indicate that the buffer was completely filled at least once
            }

            if (BGVAR(u16_IPM_Temperature_Buffer_Flag) == 1)  // Calc average only if the buffer was completely filled at least once
            {
               u16_ipm_temperature_sum = IPM_TEMPERATURE_BUFFER_ROUNDING;
               for (u16_index = 0; u16_index < IPM_TEMPERATURE_BUFFER_SIZE; u16_index++)
               {
                  u16_ipm_temperature_sum += BGVAR(u16_IPM_Temperature_Buffer)[u16_index];
               }
               u16_ipm_temperature = u16_ipm_temperature_sum >> IPM_TEMPERATURE_BUFFER_SHR;
            }

            // Dont update IPM temperature if STO is on (this will inhibit FAN high speed caused by wrong temperature reading),
            // rather init according to power temperature (as long as it is above the IPM min temperature)
            if (!u16_STO_Flt_Indication_Actual_And_Delay)
               BGVAR(u16_IPM_Temperature) = u16_ipm_temperature;
            else
            {
               if ((BGVAR(s16_Power_Temperature_Deg) > BGVAR(u16_IPM_Temp_B_Coef)) && (BGVAR(s16_Power_Temperature_Deg) > 0))
                  BGVAR(u16_IPM_Temperature) = BGVAR(s16_Power_Temperature_Deg);
               else
                  BGVAR(u16_IPM_Temperature) = BGVAR(u16_IPM_Temp_B_Coef);
            }

            if (BGVAR(u16_IPM_Temperature_Diff_Counter) > 0)
            {
               BGVAR(u16_IPM_Temperature_Diff_Counter)--;
            }
         }
         else
         {
            // Increment counter that indicates this situation (temp diff greater than threshold)
            BGVAR(u16_IPM_Temperature_Diff_Counter)++;
            BGVAR(u32_IPM_Temperature_Diff_Counter)++;   // This counter only counts up. It is for debug information.
         }

         if (BGVAR(u16_IPM_Temperature) >= BGVAR(u16_IPM_Temp_OT_Thresh))
         {
            BGVAR(u16_IPM_OT_Fault_Indication) = 1;
         }
         else if (BGVAR(u16_IPM_Temperature) < (BGVAR(u16_IPM_Temp_OT_Thresh) - 5))     // Implement 5 degC hysteresis
         {
            BGVAR(u16_IPM_OT_Fault_Indication) = 0;
         }

         u16_FPGA_flt_indication = BGVAR(u16_IPM_OT_Fault_Indication);
      }

      u16_filter_max = 1;
   }
   else
   {
      u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
      u16_FPGA_flt_indication &= PWR_AX0IPM_OTN;

      // Clear the fault on FPGA
      *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_FPGA_flt_indication;

      if ((BGVAR(u16_Power_Hw_Features) & 0x0040) != 0)
      {
         // polarity of power module OT indication is inverted
         u16_FPGA_flt_indication ^= PWR_AX0IPM_OTN;
      }

      u16_FPGA_flt_indication = (u16_FPGA_flt_indication > 0); // Translate to 0/1

      u16_filter_max = 30;  // backwards compatible
   }

   if ( (mode == DETECT) && (u16_FPGA_flt_indication) &&
        (!u16_STO_Flt_Indication_Actual_And_Delay)    &&
        (!(BGVAR(u64_Sys_Warnings) & STO_WRN_MASK))   &&
        (!(BGVAR(s64_SysNotOk) & STO_FAULT_MASK))       )
   {
      if (u16_DriveIPMOTFilter == 0)
      {
         u16_DriveIPMOTFilter = 1;
         u32_IPMOT_Filtering_Timer = Cntr_1mS;
      }

      if ( (u16_DriveIPMOTFilter == 1)                                    &&
           (PassedTimeMS((long)u16_filter_max, u32_IPMOT_Filtering_Timer))  )
      {
        if (HandleFault(drive, DRIVE_IPM_OT_FLT_MASK, 0))
        {
           p402_error_code = ERRCODE_CAN_DRIVE_IPM_OT;
           CANFaultControl(drive, 0, DRIVE_IPM_OT_FLT_MASK, 0);
        }
      }
   }

   if (u16_FPGA_flt_indication == 0) u16_DriveIPMOTFilter = 0;

   if ((mode == FLT_CLR) && (u16_FPGA_flt_indication == 0))
      BGVAR(s64_SysNotOk) &= ~DRIVE_IPM_OT_FLT_MASK;

   // Handle Warning
   if (VAR(AX0_u16_IPM_OT_DC_Enable) == 1)
   {
      if ((BGVAR(u16_IPM_Temperature) >= (BGVAR(u16_IPM_Temp_OT_Thresh) - 10)) && ((BGVAR(s64_SysNotOk) & DRIVE_IPM_OT_FLT_MASK) == 0) &&
          (!(BGVAR(u64_Sys_Warnings) & STO_WRN_MASK)) && (!(BGVAR(s64_SysNotOk) & STO_FAULT_MASK))                                         )
      {
         BGVAR(u64_Sys_Warnings) |= DRIVE_IPM_OT_WRN_MASK;
      }
      else if ( (BGVAR(u16_IPM_Temperature) <= (BGVAR(u16_IPM_Temp_OT_Thresh) - 15)) ||
                (BGVAR(s64_SysNotOk) & DRIVE_IPM_OT_FLT_MASK)                        ||
                (BGVAR(u64_Sys_Warnings) & STO_WRN_MASK)                             ||
                (BGVAR(s64_SysNotOk) & STO_FAULT_MASK)                                 )
      {
         BGVAR(u64_Sys_Warnings) &= ~DRIVE_IPM_OT_WRN_MASK;
      }
   }
}


void DriveDigitalOTFault(int drive, int mode)
{
   int s16_drive_temp = 0;
   int drive_backup = drive;

   drive = 0;
   s16_drive_temp = BGVAR(s16_Control_Temperature_Deg);
   drive = drive_backup;
   if ((mode == DETECT) && (s16_drive_temp >= 85))
   {
      ++u16_DriveDigOTFilter;
      if (u16_DriveDigOTFilter == 3)
      {
         if (HandleFault(drive, DRIVE_DIG_OT_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_DRIVE_DIG_OT;
            CANFaultControl(drive, 0, DRIVE_DIG_OT_FLT_MASK, 0);
         }
         BGVAR(u64_Sys_Warnings) &= ~DRIVE_DIG_OT_WRN_MASK;
      }
   }

   if ((mode == FLT_CLR) && (s16_drive_temp < 80))
      BGVAR(s64_SysNotOk) &= ~DRIVE_DIG_OT_FLT_MASK;

   if (s16_drive_temp < 80) u16_DriveDigOTFilter = 0;

   // Handle Warning
   if ((s16_drive_temp >= 75) && ((BGVAR(s64_SysNotOk) & DRIVE_DIG_OT_FLT_MASK) == 0))
      BGVAR(u64_Sys_Warnings) |= DRIVE_DIG_OT_WRN_MASK;
   else if ((s16_drive_temp <= 70) || (BGVAR(s64_SysNotOk) & DRIVE_DIG_OT_FLT_MASK))
      BGVAR(u64_Sys_Warnings) &= ~DRIVE_DIG_OT_WRN_MASK;
}


void DriveTempSensorFailure(int drive, int mode)
{
   // AXIS_OFF;
   int s16_drive_temp = 0;
   int drive_backup = drive;
   static int s16_first_time = 1;

   drive = 0;
   s16_drive_temp = BGVAR(s16_Power_Temperature_Deg);
   drive = drive_backup;

   // Relevant to DDHD where the temperature data is not transferred between the axes
   if (((mode == DETECT) && ((u16_FPGA_BG_Buffer_Checksum > 1000) || (u16_FPGA_BG_Buffer_Consecutive_Not_Ready > 1000))) && (!u16_STO_Just_Inserted))
   {
      if (HandleFault(drive, TEMPERATURE_HW_FLT_MASK, 1))
      {
         p402_error_code = ERRCODE_CAN_TEMPERATURE_HW;
         CANFaultControl(drive, DETECT, TEMPERATURE_HW_FLT_MASK, 1);
      }
   }

   // When continuous readout of IPM temperature is available (HW feature), if the power board temperature is greater than the IPM
   // temperature by at least 20 degC, consider it as a HW fault of the IPM temperature sensor
   if (s16_first_time) // Wait for 2 seconds after poweir up to let the various temperature readings to stabilize
   {
      if (PassedTimeMS(2000L,0L)) s16_first_time = 0;
   }
   else
   {
      if (( ((s16_drive_temp - (int)BGVAR(u16_IPM_Temperature)) > 20) && (VAR(AX0_u16_IPM_OT_DC_Enable) == 1) && (!u16_STO_Flt_Indication_Actual_And_Delay)) && (!u16_STO_Just_Inserted))
      {
         if (HandleFault(drive, TEMPERATURE_HW_FLT_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_TEMPERATURE_HW;
            CANFaultControl(drive, DETECT, TEMPERATURE_HW_FLT_MASK, 1);
         }
      }
   }

   if ( ((mode == FLT_CLR) && (u16_FPGA_BG_Buffer_Checksum < 1000) && (u16_FPGA_BG_Buffer_Consecutive_Not_Ready < 1000))    &&
        (((s16_drive_temp - (int)BGVAR(u16_IPM_Temperature)) <= 20) || (VAR(AX0_u16_IPM_OT_DC_Enable) == 0))      )
   {
      BGVAR(s64_SysNotOk_2) &= ~TEMPERATURE_HW_FLT_MASK;
   }
}


void DriveRTOverloadFault(int drive, int mode)
{
//   if ((mode == DETECT) && ((u16_RT_OL_Status_Bits & RT_OVERLOAD_FAULT) != 0)) HandleFault(drive, RT_OVERLOAD_FAULT_MASK, 1);
//   else if ((mode == DETECT) && ((u16_RT_OL_Status_Bits & RT_OVERLOAD_WARNING) != 0))
   if (mode == DETECT)
   {
      if (u16_RT_OL_Status_Bits & RT_OVERLOAD_FAULT)
      {
         if (HandleFault(drive, RT_OVERLOAD_FAULT_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_RT_OVERLOAD;
            CANFaultControl(drive, DETECT, RT_OVERLOAD_FAULT_MASK, 1);
         }
      }
      else if (u16_RT_OL_Status_Bits & RT_OVERLOAD_WARNING)
      {
         BGVAR(u64_Sys_Warnings) |= RT_OVERLOAD_WRN_MASK;
         u16_RT_OL_Status_Bits &= ~RT_OVERLOAD_WARNING;
         BGVAR(s32_RT_OV_Warning_Timer) = Cntr_1mS;
      }
   }
   else if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~RT_OVERLOAD_FAULT_MASK;
      u16_RT_OL_Status_Bits &= ~RT_OVERLOAD_FAULT;
   }
   // wait for 10 sec, if no RT overload condition occured then clear warning
   if (PassedTimeMS(10000L, BGVAR(s32_RT_OV_Warning_Timer))) BGVAR(u64_Sys_Warnings) &= ~RT_OVERLOAD_WRN_MASK;
}

void DriveNumericalPeFault(int drive, int mode)
{
   // AXIS_OFF;
   int s16_dual_loop_fault;

   if ( ((VAR(AX0_s16_Opmode) != 8) && (VAR(AX0_s16_Opmode) != 4)) || 
        (!Enabled(drive))                                          || 
        IS_DUAL_LOOP_ACTIVE                             ) VAR(AX0_s16_Pe_Scale_Factor_Fault) = 0;

   // avoid invalid fault indication due to configuration transitions - 
   // which catches the fault and that fault is tripped when drives becomes active
   // BZ6172 Yuval 24/4/17
   if (!Enabled(drive)) VAR(AX0_u16_Dual_Loop_Control_Bits) &= ~DUAL_LOOP_NUMERICAL_PE_FAULT_MASK; 

   if (((VAR(AX0_u16_Dual_Loop_Control_Bits) & DUAL_LOOP_NUMERICAL_PE_FAULT_MASK) != 0) && 
       (Enabled(drive))                                                                 && 
       IS_DUAL_LOOP_ACTIVE                                                    )
   {
      s16_dual_loop_fault = 1;
      VAR(AX0_u16_Dual_Loop_Control_Bits) &= ~DUAL_LOOP_NUMERICAL_PE_FAULT_MASK;
   }
   else
      s16_dual_loop_fault = 0;


   if ( (mode == DETECT)                                                            && 
        ((VAR(AX0_s16_Pe_Scale_Factor_Fault) != 0) || (s16_dual_loop_fault == 1))   &&
        ((VAR(AX0_s16_Opmode) == 8) || (VAR(AX0_s16_Opmode) == 4))                     )        
   {
      if (HandleFault(drive, NUMERICAL_PE_FLT_MASK, 1))
      {
         p402_error_code = ERRCODE_CAN_NUMERICAL_PE;
         CANFaultControl(drive, DETECT, NUMERICAL_PE_FLT_MASK, 1);
      }
   }

   if (mode == FLT_CLR)
      BGVAR(s64_SysNotOk_2) &= ~NUMERICAL_PE_FLT_MASK;
}


void DrivePowerOTFault(int drive, int mode)
{
   int s16_drive_temp = 0;
   int drive_backup = drive;

   drive = 0;
   s16_drive_temp = BGVAR(s16_Power_Temperature_Deg);
   drive = drive_backup;

   if (mode == DETECT)
   {
      if (s16_drive_temp >= (int)u16_OT_Flt_Threshold)
      {
         if (u16_DrivePowOTFilter < 201)
         {
            ++u16_DrivePowOTFilter;
         }

         if (u16_DrivePowOTFilter == 200)
         {
            HandleFault(drive, DRIVE_POW_OT_FLT_MASK, 0);
            BGVAR(u64_Sys_Warnings) &= ~DRIVE_POW_OT_WRN_MASK;
         }
      }
      else if (u16_DrivePowOTFilter > 0)
      {
         u16_DrivePowOTFilter--;
      }
   }

   if ((mode == FLT_CLR) && (s16_drive_temp < (int)(u16_OT_Flt_Threshold - 5)))
      BGVAR(s64_SysNotOk) &= ~DRIVE_POW_OT_FLT_MASK;

   // Handle Warning
   if ( (s16_drive_temp >= (int)u16_OT_Wrn_Threshold)       &&
        ((BGVAR(s64_SysNotOk) & DRIVE_POW_OT_FLT_MASK) == 0)  )
      BGVAR(u64_Sys_Warnings) |= DRIVE_POW_OT_WRN_MASK;
   else if ( (s16_drive_temp <= (int)(u16_OT_Wrn_Threshold - 5)) ||
             (BGVAR(s64_SysNotOk) & DRIVE_POW_OT_FLT_MASK)         )
      BGVAR(u64_Sys_Warnings) &= ~DRIVE_POW_OT_WRN_MASK;
}


void PeMaxFault(int drive, int mode)
{
   // AXIS_OFF;
   long long pe;
   int temp_time;

   if (MaskedFault(drive, PEMAX_FLT_MASK, 0)) return;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~PEMAX_FLT_MASK;
      return;
   }

   do
   {
      temp_time = Cntr_3125;
      pe = LLVAR(AX0_u32_Pos_Err_Lo);
   } while (temp_time != Cntr_3125);

   if (BGVAR(u64_Pe_Max) > 0LL)
   {
      if (pe < 0LL) pe = -pe;

      if ( (mode == DETECT) && (pe > BGVAR(u64_Pe_Max)) && (Enabled(drive)) &&
           ( (VAR(AX0_s16_Opmode) == 4) || (VAR(AX0_s16_Opmode) == 8) )       )
      {
         if (HandleFault(drive, PEMAX_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_PEMAX;
            CANFaultControl(drive, 0, PEMAX_FLT_MASK, 0);

         //start timer
            s32_following_error_bit_timer = Cntr_1mS;
         }

         return;
      }
   }

   if (PassedTimeMS((long)p402_following_error_time_out, s32_following_error_bit_timer) &&
       (s32_following_error_bit_timer >= 0)                                             &&
       ((VAR(AX0_s16_Opmode) == 4) || (VAR(AX0_s16_Opmode) == 8))                         )
   {
      p402_statusword |= FOLLOWING_ERROR_BIT;
     s32_following_error_bit_timer = -1;
   }

   if ((mode == FLT_CLR) && ((pe <= BGVAR(u64_Pe_Max)) || (!Enabled(drive))))
   {
      BGVAR(s64_SysNotOk) &= ~PEMAX_FLT_MASK;
      p402_statusword &= ~FOLLOWING_ERROR_BIT;
   }
}


void VospdFault(int drive, int mode)
{
   // AXIS_OFF;
   long s32_vel, s32_vel_overspeed;
   int s16_filter_threshold = 5;

   if ( (MaskedFault(drive, VOSPD_FLT_MASK, 0))                                     ||
        ( ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0) &&
          ( (LVAR(AX0_s32_Feedback_Ptr) == &ENDAT2X_COMMUNICATION_FEEDBACK) ||
            (LVAR(AX0_s32_Feedback_Ptr) == &NK_COMMUNICATION_FEEDBACK)      ||
            (LVAR(AX0_s32_Feedback_Ptr) == &TM_COMMUNICATION_FEEDBACK)      ||
            (LVAR(AX0_s32_Feedback_Ptr) == &PS_P_G_COMMUNICATION_FEEDBACK)  ||
            (LVAR(AX0_s32_Feedback_Ptr) == &PS_S_COMMUNICATION_FEEDBACK)    ||
            (LVAR(AX0_s32_Feedback_Ptr) == &FANUC_COMMUNICATION_FEEDBACK)   ||
            (LVAR(AX0_s32_Feedback_Ptr) == &SK_COMMUNICATION_FEEDBACK)        )   ) ||
        (BGVAR(u16_Motor_Setup_State) |= MOTOR_SETUP_IDLE)                            )
        // Ignore Fault while Motor-Setup is running...
      return;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~VOSPD_FLT_MASK;
      return;
   }

   // On Resolver wait till the adjust is done before checking this fault
   if ( (LVAR(AX0_s32_Feedback_Ptr) == &SW_RESOLVER_FEEDBACK) &&
        (BGVAR(s16_Adjust_Reference_State) != ADJ_REF_DONE)     )
      return;

   // set the velocity overspeed fixed to 1.2*vlim
   s32_vel_overspeed = (long)(((long long)BGVAR(s32_V_Lim_Design) * 0x4ccd) >> 14);

   s32_vel = LVAR(AX0_s32_Vel_Var_Fb_0);
   if (s32_vel < 0L) s32_vel = -s32_vel;

   if ( (mode == DETECT) && (s32_vel > s32_vel_overspeed) && ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK)==0) && ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK)==0) )
   {
      ++BGVAR(s16_VospdFilter);

      // IPR 683: on lxm drive, in torque mode, set filter threshold to 500 backgrounds to avoid catching fault if vel overshoot happens.
      // otherwise, use 5 backgrounds
      if ( (u16_Product == SHNDR) || (u16_Product == SHNDR_HW) )
      {
         if ((VAR(AX0_s16_Opmode) == 2) || (VAR(AX0_s16_Opmode) == 3))
         {
            s16_filter_threshold = 300;
         }
      }

      if (BGVAR(s16_VospdFilter) >= s16_filter_threshold)
      {
         if (HandleFault(drive, VOSPD_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_VOSPD;
            CANFaultControl(drive, 0, VOSPD_FLT_MASK, 0);
         }
      }
   }

   if ((mode == FLT_CLR) && (s32_vel < s32_vel_overspeed)) BGVAR(s64_SysNotOk) &= ~VOSPD_FLT_MASK;

   if (s32_vel < s32_vel_overspeed) BGVAR(s16_VospdFilter) = 0;
}


void ResetFeedbackFault(int drive, int reset_feedback)
{
   // AXIS_OFF;
   long long s64_temp_var = 0;

   if (reset_feedback)
   {
      if (LVAR(AX0_s32_Feedback_Ptr) == &SW_RESOLVER_FEEDBACK)
      { // On Resolver Error recovery init the gain search and the Sine Zero Process
         BGVAR(s16_Adjust_Reference_State) = ADJ_REF_INIT;

         SalReadSinInitStatusCommand(&s64_temp_var, drive); // Restart sininit if in progress
         if (s64_temp_var == 1LL) SininitCommand(drive);
      }
      else if ( (LVAR(AX0_s32_Feedback_Ptr) == &SW_SINE_ENC_FEEDBACK) ||
                 (LVAR(AX0_s32_Feedback_Ptr) == &INC_ENCODER_FEEDBACK)  )
         ResetEncState(drive); // Restart Encoder Setup
      else
      {  // On Panasonic or EnDat reset the Encoder Initialization State Machine
         if (LVAR(AX0_s32_Feedback_Ptr) == &PS_P_G_COMMUNICATION_FEEDBACK)
         { // Re-initialize Halls Detection to recover from Power-Up Timer Fault
            VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;         // Hall-Effect Initialization.
            BGVAR(u16_CommFdbk_Init_Select) = 0;
            BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
         }
         else if (LVAR(AX0_s32_Feedback_Ptr) == &ENDAT2X_COMMUNICATION_FEEDBACK)
            BGVAR(s16_Endat_Init_State) = ENDAT_REVERSAL_RESET_STATE;
         ResetEncState(drive);
         VAR(AX0_s16_Crrnt_Run_Code) |= RESET_SWR2D_MASK;
      } // @_s16_Endat_Init_State
   }
}


//**********************************************************
// Function Name: FeedbackFault
// Description:
//   This function handles feedback faults
//   note:
//   s16_fault_set=0 refers to a fault bit in s64_SysNotOk
//   s16_fault_set=1 refers to a fault bit in s64_SysNotOk_2
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int FeedbackFault(int drive, int mode, long long s64_fault_mask, unsigned int *filter, unsigned int time_interval, int filter_max, int fault_bit, int reset_feedback, int s16_fault_set)
{
   /* return value:
   * -1 -> fault not defined
   *  1 -> mode = detect and fault
   *  0 -> mode = flt_clr and no fault and fault defined */

   int ret_val = -1, u8_Result = 0;

   if (mode == DETECT)
   {
      if (fault_bit)
      {
         if (filter == NULL)
            u8_Result = HandleFault(drive, s64_fault_mask, s16_fault_set);
         else
         {
            if (*filter > filter_max)
               u8_Result = HandleFault(drive, s64_fault_mask, s16_fault_set);
            else *filter += time_interval;
         }

         if (u8_Result == 1)
         {
            if (s16_fault_set == 0)
            {
               switch (s64_fault_mask)      // for s64_SysNotOk masks
               {
                  case ILLEGAL_HALLS_MASK:
                     p402_error_code = ERRCODE_CAN_ILLEGAL_HALLS;
                  break;
                  case TAMAGAWA_FDBK_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_TAMAGAWA_FDBK;
                  break;
                  case ENCODER_5V_OC_MASK:
                     p402_error_code = ERRCODE_CAN_ENCODER_5V_OC;
                  break;
                  case SEC_ENCODER_5V_OC_MASK:
                     p402_error_code = ERRCODE_CAN_SEC_ENCODER_5V_OC;
                  break;
                  case INDEX_BRK_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_INDEX_BRK;
                  break;
                  case SEC_INDEX_BRK_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_SEC_INDEX_BRK;
                  break;
                  case LINE_BRK_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_LINE_BRK;
                  break;
                  case SEC_LINE_BRK_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_SEC_LINE_BRK;
                  break;
                  case PD_LINE_BRK_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_PD_LINE_BRK;
                  break;
                  case AB_OUT_OF_RANGE_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_AB_OUT_OF_RANGE;
                  break;
                  case SIN_COMM_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_SIN_COMM;
                  break;
                  case COMM_FDBK_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_COMM_FDBK;
                  break;
                  case SANYO_FDBK_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_SANYO_FDBK;
                  break;
                  case  ABS_ENC_BATT_LOW_MASK:
                     p402_error_code = ERRCODE_CAN_TAMAGAWA_ABS_BATT_LOW;
                  break;
                  case  TAMAGAWA_ABS_ENC_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_TAMAGAWA_ABS_ENC;
                  break;
                  case SW_SINE_QUAD_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_SW_SINE_QUAD;
                  break;
                  case SININIT_INVALID_MASK:
                     p402_error_code = ERRCODE_CAN_SININIT_INVALID;
                  break;
                  case RESOLVER_REF_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_RESOLVER_REF;
                  break;
                  case ENDAT2X_FDBK_FAULTS_MASK:
                     p402_error_code = ERRCODE_CAN_ENDAT2X_FDBK;
                  break;
                  case PFB_OFF_CHKSUM_MASK:
                     p402_error_code = ERRCODE_CAN_PFB_OFF_CHECKSUM;
                  break;
                  case PFB_OFF_DATA_MISMATCH_MASK:
                     p402_error_code = ERRCODE_CAN_PFB_OFF_MISMATCH;
                  break;
                  case PFB_OFF_NO_DATA_MASK:
                     p402_error_code = ERRCODE_CAN_PFB_OFF_NO_DATA;
                  break;
               }
            }
            else
            {
               switch (s64_fault_mask)      // for s64_SysNotOk_2 masks
               {
                  case  PANASONIC_S_ENC_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_PANASONIC_S_ENC;
                  break;
                  case DIFF_HALLS_LINE_BRK_MASK:
                     p402_error_code = ERRCODE_CAN_DIFF_HALLS_LINE_BRK;
                  break;
                  case ENCODER_PHASE_ERROR_MASK:
                     p402_error_code = ERRCODE_CAN_ENCODER_PHASE_ERROR;
                  break;
                  case COMMUTATION_LOSS_FAULT_MASK:
                     p402_error_code = ERRCODE_CAN_COMMUTATION_LOSS;
                  break;
                  case SRVSNS_ENCODER_FLT_MASK:
                     p402_error_code = ERRCODE_CAN_SRVSNS_ENCODER;
                  break;
                  default:
                  break;
               }
            }

            CANFaultControl(drive, 0, s64_fault_mask, s16_fault_set);
         }
      }
      else
      {
        if ((filter != NULL) && (*filter > time_interval))
           *filter -= time_interval;
      }
      ret_val = 1;
   }

   if (s16_fault_set == 0)
   {
      if ( (mode == FLT_CLR) && (!fault_bit)           &&
          ((BGVAR(s64_SysNotOk) & s64_fault_mask) != 0)  )
      {
         BGVAR(s64_SysNotOk) &= ~s64_fault_mask;
         ResetFeedbackFault(drive, reset_feedback);
         ret_val = 0;
      }
   }
   else
   {
      if ( (mode == FLT_CLR) && (!fault_bit)              &&
           ((BGVAR(s64_SysNotOk_2) & s64_fault_mask) != 0)  )
      {
         BGVAR(s64_SysNotOk_2) &= ~s64_fault_mask;
         ResetFeedbackFault(drive, reset_feedback);
         ret_val = 0;
      }
   }

   if ( (!fault_bit) && (filter != NULL) && (mode == FLT_CLR) ) *filter = 0;

   return (ret_val);
}


//**********************************************************
// Function Name: MaskedFault
// Description:
//   This function returns indication whether the fault is
//   masked (won't be detected) or not masked (may be detected).
//   note:
//   s16_fault_set = 0 refers to a fault bit in s64_SysNotOk
//   s16_fault_set = 1 refers to a fault bit in s64_SysNotOk_2
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int MaskedFault(int drive, long long s64_fault_mask, int s16_fault_set)
{
   /*return value :
   *  0 -> fault will be detected
   *  1 -> fault will NOT be detected   */

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (s16_fault_set == 0)
   {
      if (((BGVAR(s64_Faults_Mask) & s64_fault_mask) == 0) || (BGVAR(s64_User_Faults_Mask) & s64_fault_mask))
      {
         BGVAR(s64_SysNotOk) &= ~s64_fault_mask;
         return (1);
      }
   }
   else
   {
      if (((BGVAR(s64_Faults_Mask_2) & s64_fault_mask) == 0)  || (BGVAR(s64_User_Faults_Mask2) & s64_fault_mask))
      {
         BGVAR(s64_SysNotOk_2) &= ~s64_fault_mask;
         return (1);
      }
   }
   return (0);
}


//**********************************************************
// Function Name: DetectFeedbackFaults
// Description:
//   set detectable feedback faults according to mask.
//   all other feedback faults are cleared
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void DetectFeedbackFaults(int drive, long long s64_mask, long long s64_mask_2)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s64_Faults_Mask) &= ~FEEDBACK_LOSS_FLT_MASK; //clear all feedback faults
   BGVAR(s64_Faults_Mask) |= s64_mask;

   BGVAR(s64_Faults_Mask_2) &= ~FEEDBACK_LOSS_FLT_2_MASK; //clear all feedback faults
   BGVAR(s64_Faults_Mask_2) |= s64_mask_2;
}


//**********************************************************
// Function Name: SalUvModeCommand
// Description:
//          This function is called in response to the UVMODE command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalUvModeCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Uv_Mode) = (unsigned int)lparam;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalUVThreshCommand
// Description:
//          This function is called in response to the UVTHRESH command.
//
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
int SalUVThreshCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( (lparam - 5) < (long long)u16_Inrush_Off_Value ) return (VALUE_TOO_LOW);

   if ( (u16_Product == SHNDR) || (u16_Product == SHNDR_HW) )
   {
      // Although SE uses P4-24 for UVTHRESH, check the range here in case UVTHRESH was used.
      if (lparam > 269LL) return (VALUE_TOO_HIGH);  // SE upper limit for UVTHRESH is 190 * sqrt(2) = 269V
      if (lparam < 198LL) return (VALUE_TOO_LOW);   // SE lower limit for UVTHRESH is 140 * sqrt(2) = 198V
   }

   BGVAR(u16_UV_Threshold) = (unsigned int)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalWriteLowVoltageAlaramDetectionLevel
// Description:
//          This function is called in response to the write P4-24 (the CDHD
//          equivalent is the UVTHRESH command). This function multiplies a
//          factor of "sqrt(2)" to the input value since an alarm is supposed
//          to be triggered in case that the DC-bus voltage drops below
//          P4-24 * sqrt(2).
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteLowVoltageAlaramDetectionLevel(long long lparam,int drive)
{
   // Call UVTHRESH Sal-function with a value multiplied by sqrt(2).
   // Add 0.5 = 8192 / 2^14 for the proper data-representation.
   return (SalUVThreshCommand((lparam * 23170 + 8192)>>14, drive));
}


//**********************************************************
// Function Name: SalReadLowVoltageAlaramDetectionLevel
// Description:
//          This function is called in response to the read P4-24 (the CDHD
//          equivalent is the UVTHRESH command). This function considers a
//          factor of "1/sqrt(2)" since the factor "sqrt(2)" is involved when
//          writing the under-voltage threshold parameter.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadLowVoltageAlaramDetectionLevel(long long* data, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Return the value divided by sqrt(2), add 0.5 = 16384 / 2^15 for the proper data-representation.
   *data = ((long long)BGVAR(u16_UV_Threshold) * 23170 + 16384) >> 15;

   return SAL_SUCCESS;
}


void WarningsHandler(int drive)
{
   /* Here Add non fieldbus warnings handling*/

   // check for canopen warnings
   CAN_WarningsHandler(drive);
}


void RemarksHandler(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if ((BGVAR(u16_FdbkType) == INC_ENC_FDBK))
   {
      //Halls switch not found remark
      if ( (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_1 & 0xffff) ) && (FEEDBACK_WITH_HALLS)) //Changed to avoid Remark
         BGVAR(u16_Sys_Remarks) |= HALL_NOT_FOUND_RMK_MASK;
      else
         BGVAR(u16_Sys_Remarks) &= ~HALL_NOT_FOUND_RMK_MASK;

      //Index not found
      if (  (FEEDBACK_WITH_INDEX)                                                    &&
            ( (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_2  & 0xffff)) ||   //Changed to avoid Remark
              (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_1  & 0xffff))   )  )
         BGVAR(u16_Sys_Remarks) |= INDEX_NOT_FOUND_RMK_MASK;
      else
         BGVAR(u16_Sys_Remarks) &= ~INDEX_NOT_FOUND_RMK_MASK;
   }
   else
      BGVAR(u16_Sys_Remarks) &= ~(INDEX_NOT_FOUND_RMK_MASK | HALL_NOT_FOUND_RMK_MASK);
}


//**********************************************************
// Function Name: CAN_WarningsHandler
// Description:
//          This function is handle the warnings generated from the canopen mechanism.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CAN_WarningsHandler(int drive)
{
   int i;

   if (BGVAR(u64_Sys_Warnings_Prev) != BGVAR(u64_Sys_Warnings))
   {
      // find first new bit
      i = 0;
      while (i < 64)
      {
         if ( ((BGVAR(u64_Sys_Warnings_Prev) & (1 << i)) == 0) && (BGVAR(u64_Sys_Warnings) & (1 << i)) )
            break;

         i++;
      }

      if (i < 64)
      {  // update the last warning P0-47
         BGVAR(u16_P0_47_Last_Wrn_Code) = GetLexiumFaultNumberByBit(drive, BGVAR(u64_Sys_Warnings) & (1<<i), 2);
      }

      BGVAR(u64_Sys_Warnings_Prev) = BGVAR(u64_Sys_Warnings);
   }
}


void EncoderSimulationFreqFault(int drive, int mode)
{
   // AXIS_OFF;

   if (!IS_HW_FUNC_ENABLED(EEO_MASK)) return;

   // If the fault is NOT masked and therefore cleared upon the "MaskedFault" function call --> Leave function
   if (MaskedFault(drive, ENC_SIM_FREQ_FLT_MASK, 0)) return;

   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~ENC_SIM_FREQ_FLT_MASK;
      return;
   }

   if (mode == DETECT)
   {
      if ( BGVAR(u8_EncoutMode) && ((BGVAR(u16_P2_65_GBIT) & 0x2000) == 0))
      {
        if (VAR(AX0_u16_Encsim_Freq_Flags) & FREQ_FAULT_MASK)
        {// fault show is controlled by bit13 of P2-65 only for LX28/26 drives
            if (HandleFault(drive, ENC_SIM_FREQ_FLT_MASK, 0))
            {
               p402_error_code = ERRCODE_CAN_ENC_SIM_FREQ;
               CANFaultControl(drive, 0, ENC_SIM_FREQ_FLT_MASK, 0);
            }
         }
      }
      else
      {
         VAR(AX0_u16_Encsim_Freq_Flags) &= ~FREQ_FAULT_MASK;
      }
   }

   if (mode == FLT_CLR)
   {
      VAR(AX0_u16_Encsim_Freq_Flags) &= ~FREQ_FAULT_MASK;
      BGVAR(s64_SysNotOk) &= ~ENC_SIM_FREQ_FLT_MASK;
   }
}


int u16_first_time_here = 1;

void SecurityFault(void)
{
   int i;
   unsigned int u16_dna_0 = 0;
   unsigned int u16_dna_1 = 0;
   unsigned int u16_dna_2 = 0;
   unsigned int u16_dna_3 = 0;
   unsigned long long u56_fpga_code = 0LL;
   unsigned long long u64_xored_code = 0LL;
   unsigned long long u64_xored_code_odd = 0LL;
   unsigned long long u64_xored_code_even = 0LL;
   unsigned long long u56_key1 = 0LL;
   unsigned long long u56_mask = 0x00DF5913CB582AFE;
   unsigned long long u60_stored_value;
   unsigned long long u56_stored_key;
   unsigned long bits31_0;
   unsigned long bits63_32;
   unsigned long bits95_64;

   if (u16_first_time_here) // Check the Security once more...
   {
      u16_first_time_here = 0;

      u16_dna_3 = *(int *)FPGA_DNA_REG_3_REG_ADD;
      u16_dna_2 = *(int *)FPGA_DNA_REG_2_REG_ADD;
      u16_dna_1 = *(int *)FPGA_DNA_REG_1_REG_ADD;
      u16_dna_0 = *(int *)FPGA_DNA_REG_0_REG_ADD;
      u56_fpga_code = (((unsigned long long)u16_dna_3 & 0x00FF) << 48) | ((unsigned long long)u16_dna_2 << 32) | ((unsigned long long)u16_dna_1 << 16) | ((unsigned long long)u16_dna_0);

      // Create a 96bit code from FPGA's 56 bit code
      u64_xored_code = (u56_fpga_code >> 3) ^ (u56_fpga_code >> 13);
      u64_xored_code_odd = 0LL;
      u64_xored_code_even = 0LL;
      for (i = 54; i >= 0; i -= 2)
      {
         if ((u64_xored_code & (1LL << (i + 1))) > 0)
            u64_xored_code_odd |= 0x01;
         if ((u64_xored_code & (1LL << i)) > 0)
            u64_xored_code_even |= 0x01;
         if (i > 0)
         {
            u64_xored_code_odd <<= 1;
            u64_xored_code_even <<= 1;
         }
      }
      bits31_0 = (unsigned long)(u64_xored_code_even & 0xFFFFF) | (((u56_fpga_code & 0xFFF) << 20LL) & 0xFFF00000);
      bits63_32 = (unsigned long)((u56_fpga_code & 0xFFFFFFFF000) >> 12LL);
      bits95_64 = (unsigned long)(((u56_fpga_code & 0xFFF00000000000) >> 44LL) & 0xFFF) | ((u64_xored_code_odd & 0x000FFFFF) << 12LL);

      u56_key1 = u56_fpga_code ^ Rotate(u56_mask, Ones(u56_fpga_code) + ((bits31_0 + bits63_32 + bits95_64) % 7));
      u56_key1 = Inverse(u56_key1, Ones(u56_key1)) & 0x00FFFFFFFFFFFFFF;

      u60_stored_value = u16_Key_Buffer[4];
      u60_stored_value <<= 16LL;
      u60_stored_value |= ((unsigned long long)u16_Key_Buffer[3]) & 0x0000FFFF;
      u60_stored_value <<= 16LL;
      u60_stored_value |= ((unsigned long long)u16_Key_Buffer[2]) & 0x0000FFFF;
      u60_stored_value <<= 16LL;
      u60_stored_value |= ((unsigned long long)u16_Key_Buffer[1]) & 0x0000FFFF;
      // Extract the writing times
      u56_stored_key = (u60_stored_value & 0x3FFF) | ((u60_stored_value & 0x3FFF8000) >> 1LL) | ((u60_stored_value & 0x1FF80000000) >> 2LL) | ((u60_stored_value & 0x1FFC0000000000) >> 3LL) | ((u60_stored_value & 0xFC0000000000000) >> 4LL);

      // Compare the calculated key to the stored key to decide if Security should be released
      if (u56_key1 != u56_stored_key)
      {
         Security_Released2 = 0;
         AX0_Security_Bit = 1;
         if (u8_FPGA_ok) Display_State = 6;
      }
      else Security_Released2 = 1;
   }

   if (Security_Released <= 0)
   {
      if (HandleFault(0, SECURITY_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_SECURITY;
         CANFaultControl(0, 0, SECURITY_FLT_MASK, 0);
      }
   }
   if (Security_Released2 <= 0)
   {
      if (HandleFault(0, SECURITY_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_SECURITY;
         CANFaultControl(0, 0, SECURITY_FLT_MASK, 0);
      }
   }
}


void Plus15VFault(int drive, int mode)
{
   int fault_bit = 0;
   int s16_plus_15v;

   if (MaskedFault(drive, PLUS_LOGIC_15V_FLT_MASK, 0)) return;

   // V = 4 * Vadc / 32767 * 10 * 1000 = Vadc * 1.22074
   s16_plus_15v = (int)((float)s16_Adc_Plus_15v_Readout_Filtered * 1.22074);

   // the thresh is: 12500mv <= V <= 16500mv
   if ((s16_plus_15v > 16500) || (s16_plus_15v < 12500))  fault_bit = 1;

   if ( (mode == DETECT) && (fault_bit) )
   {
      ++s16_Plus15vFilter;
      if (s16_Plus15vFilter == 10)
      {
         if (HandleFault(drive, PLUS_LOGIC_15V_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_PLUS_LOGIC_15V;
            CANFaultControl(drive, 0, PLUS_LOGIC_15V_FLT_MASK, 0);
         }
      }
   }

   if ( (mode == FLT_CLR) && (!fault_bit) )  BGVAR(s64_SysNotOk) &= ~PLUS_LOGIC_15V_FLT_MASK;

   if (!fault_bit) s16_Plus15vFilter = 0;
}


void Minus15VFault(int drive, int mode)
{
   int fault_bit = 0;
   int s16_minus_15v;

   if (MaskedFault(drive, MINUS_LOGIC_15V_FLT_MASK, 0)) return;

   // V = 5 * Vadc / 32767 * 10 * 1000  - 20000 = Vadc * 1.52592 -20000
   s16_minus_15v = (int)((float)s16_Adc_Minus_15v_Readout_Filtered * 1.52592 - 20000.0);

   // the thresh is: -17500mv <= V <= -13000mv
   if ((s16_minus_15v < -17500) || (s16_minus_15v > -13000))
      fault_bit = 1;

   if ( (mode == DETECT) && (fault_bit) )
   {
      ++s16_Minus15vFilter;
      if (s16_Minus15vFilter == 10)
      {
         if (HandleFault(drive, MINUS_LOGIC_15V_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_MINUS_LOGIC_15V;
            CANFaultControl(drive, 0, MINUS_LOGIC_15V_FLT_MASK, 0);
         }
      }
   }

   if ((mode == FLT_CLR) && (!fault_bit))  BGVAR(s64_SysNotOk) &= ~MINUS_LOGIC_15V_FLT_MASK;

   if (!fault_bit) s16_Minus15vFilter = 0;
}


// We have added a filter to this fault, due to noise on the HW indication. The filter is done on the I2C level
void LogicACSupplyFault(int drive, int mode)
{
   static int s16_ac_logic_flt_cntr = 0;
   unsigned int u16_flt_indication = 0, u16_new_read_flag = 0;

   if (MaskedFault(drive, LOGIC_AC_LOSS_MASK, 1)) return;

   if (BGVAR(u16_Power_Hw_Features) & 0x0002)
   {
      if (u16_Product == DDHD)
      {
         u16_flt_indication = *(unsigned int *)(FPGA_L_LOSS_REG_ADD) & 0x0001; // Isolate the Logic AC Line-Loss at Bit 0
      }
      else if ((u16_Power1_Input_Port & 0x00F1) == 0x00F0)  // Indication that data was read at least once, and no Logic AC
      {
         u16_flt_indication = 1;
      }
   }

   // Indicate new read exists since it takes few BG for I2C new readout
   u16_new_read_flag = 1;
   if (BGVAR(u16_Power_Hw_Features) & 0x0002)
   {
      if (u16_Power1_Input_Port_Read_Counter != u16_Power1_Input_Port_Read_Counter_Prev)
      {
         u16_Power1_Input_Port_Read_Counter_Prev = u16_Power1_Input_Port_Read_Counter;
      }
      else
      {
         u16_new_read_flag = 0;
      }
   }

   // Ignore the fault on first time
   if (u16_new_read_flag)
   {
      if (u16_flt_indication)
      {
         if (s16_ac_logic_flt_cntr == 0)   // first occurance
         {
            s16_ac_logic_flt_cntr++;
            u16_flt_indication = 0;        // clear the flt indication on the first occurence
         }
         else
         {
            s16_ac_logic_flt_cntr = 0;     // clear the counter (to be ready for next time), but keep the flt indication
         }
      }
      else
      {
         s16_ac_logic_flt_cntr = 0;        // clear the counter if no flt indication
      }

      // DDHD check AC1AC2_loss signal, or
      // Power with I2C IO port check (bits 4-7: 1 indicates data was read. bit 0: 1-AC OK, 0-AC loss)
      if (mode == DETECT)
      {
         if (u16_flt_indication)
         {
            if ((BGVAR(u16_Line_Loss_Type) != 0) || (u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
            {
               HandleFault(drive, LOGIC_AC_LOSS_MASK, 1);
            }
         }
      }
      else // FLT_CLR
      {
         if ((!u16_flt_indication) || (BGVAR(u16_Line_Loss_Type) == 0))
         {
            BGVAR(s64_SysNotOk_2) &= ~LOGIC_AC_LOSS_MASK;
            s16_ac_logic_flt_cntr = 0;
         }
      }
   }

   // AC loss fault may happen on every power off. Log it after delay, since if the drive is still running the fault is real
   if (BGVAR(u16_Delayed_Logic_AC_Loss_Log_Fault_Flag) == 1)
   {
      if (PassedTimeMS(5000L, BGVAR(s32_Delayed_Logic_AC_Loss_Log_Fault_Timer)))
      {
         BGVAR(u16_Delayed_Logic_AC_Loss_Log_Fault_Flag) = 0;
         WriteToFaultLog(drive, GetFaultId((unsigned long long)LOGIC_AC_LOSS_MASK, (int)1));
      }
   }
}


// We have added a filter to this fault, due to noise on the HW indication. The filter is done on the I2C level
void BusACSupplyFault(int drive, int mode)
{
   unsigned int u16_hw_indication, u16_loss_condition, u16_fault_condition, u16_warning_condition;
   static int s16_ddhd_flt_cntr = 0;

   if (MaskedFault(drive, BUS_AC_LOSS_MASK, 1)) return;

   u16_hw_indication = 3;      // No phase disconnection by default. Valid before the I2C IO port is read for the first time.
   u16_loss_condition = 0;
   u16_fault_condition = 0;
   u16_warning_condition = 0;

   // Read the HW indication for bus AC supply phase loss
   // DDHD check ???, or
   // Power with I2C IO port check (bits 4-7: 1 indicates data was read. bit 1: 1- L1-L2 OK, 0- L1-L2 loss, bit 2: 1- L2-L3 OK, 0- L2-L3 loss)
   if (BGVAR(u16_Power_Hw_Features) & 0x0002)
   {
      if (u16_Product == DDHD)
      {
         u16_hw_indication = ((*(unsigned int *)(FPGA_L_LOSS_REG_ADD) & 0x0003) >> 1);
         // Isolate the two line to line loss indication (L1-L2, L1-L3), expected at Bits 1 nd 2.
         // If 3 phase bus power supply is not supported, remove the indication of L2-L3 OK
         if (BGVAR(u16_Power_Hw_Features) & 0x0004) u16_hw_indication &= ~0x0002;
      }
      else if ((u16_Power1_Input_Port & 0x00F0) == 0x00F0)     // Indication that data was read at least once
      {
         u16_hw_indication = (u16_Power1_Input_Port >> 1) & 0x0003;  // Isolate the two line to line loss indication (L1-L2, L2-L3)

         // If 3 phase bus power supply is not supported, remove the indication of L2-L3 OK
         if (BGVAR(u16_Power_Hw_Features) & 0x0004) u16_hw_indication &= ~0x0002;
      }
      BGVAR(u16_Hw_Ac_Phase_Indication) = u16_hw_indication;
   }

   // Evaluate loss condition according to the detection type
   // 0 - no detection
   // 1 - one phase connection
   // 2 - three phase connection
   if (BGVAR(u16_Line_Loss_Type) == 1)
   {
      if (u16_hw_indication == 0)
      {
         u16_loss_condition = 1;
      }
   }
   else if (BGVAR(u16_Line_Loss_Type) == 2)
   {
      if (u16_hw_indication != 3)
      {
         u16_loss_condition = 1;
      }
   }

   // On DDHD ignore the fault on first time, this is to replace the filtering done on the I2C for other types of drives
   if (u16_Product == DDHD)
   {
      if (u16_loss_condition)
      {
         if (!s16_ddhd_flt_cntr)
         {
            s16_ddhd_flt_cntr = 1;
            u16_loss_condition = 0;
         }
      }
      else
         s16_ddhd_flt_cntr = 0;
   }

   // Determine if fault exists according to the detection mode
   // 0 - fault when drive is enabled or disabled
   // 1 - fault when drive is enabled

   if ( (BGVAR(u16_Line_Loss_Mode) == 0) || ((BGVAR(u16_Line_Loss_Mode) == 1) && Enabled(DRIVE_PARAM)) )
   {
      if (u16_loss_condition == 1) u16_fault_condition = 1;
   }

   if (mode == DETECT)
   {
      if (u16_fault_condition == 1)
      {
         if (HandleFault(drive, BUS_AC_LOSS_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_BUS_AC_LOSS;
            CANFaultControl(drive, 0, BUS_AC_LOSS_MASK, 1);
         }
      }
   }
   else
   {
      if (u16_fault_condition == 0)
      {
         BGVAR(s64_SysNotOk_2) &= ~BUS_AC_LOSS_MASK;
      }
   }

   // Determine if warning exists according to the detection mode
   // 1 - warning when no fault and drive is disabled
   // 2 - warning when no fault and drive is enabled or disabled

   if ( (BGVAR(u16_Line_Loss_Mode) == 2) || ((BGVAR(u16_Line_Loss_Mode) == 1) && !Enabled(DRIVE_PARAM)) )
   {
      if (u16_loss_condition == 1) u16_warning_condition = 1;
   }

   if ( (u16_warning_condition == 1) && ((BGVAR(s64_SysNotOk_2) & BUS_AC_LOSS_MASK) == 0) )
   {
      BGVAR(u64_Sys_Warnings) |=  BUS_AC_LOSS_WRN_MASK; //Set warning mask
   }
   else
   {
      BGVAR(u64_Sys_Warnings) &= ~BUS_AC_LOSS_WRN_MASK; // Clear warning mask
   }


   // handle RECOVER - consider hysteresis
   if ( (u16_loss_condition == 0) && (BGVAR(u16_Line_Loss_Recover) == 1) )
      BGVAR(s64_SysNotOk_2) &= ~BUS_AC_LOSS_MASK;


   // AC loss fault may happen on every power off. Log it after delay, since if the drive is still running the fault is real
   if (BGVAR(u16_Delayed_Line_Loss_Log_Fault_Flag) == 1)
   {
      if (PassedTimeMS(5000L, BGVAR(s32_Delayed_Line_Loss_Log_Fault_Timer)))
      {
         BGVAR(u16_Delayed_Line_Loss_Log_Fault_Flag) = 0;
         WriteToFaultLog(drive, GetFaultId((unsigned long long)BUS_AC_LOSS_MASK, (int)1));
      }
   }
}


void PhaseFindFault(int drive, int mode)
{
   // AXIS_OFF;

   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {   // Ignore this fault during Burnin
      BGVAR(s64_SysNotOk) &= ~PHASE_FIND_FLT_MASK;
      return;
   }

   if ( (mode == FLT_CLR) && ((BGVAR(s64_SysNotOk) & PHASE_FIND_FLT_MASK) != 0) )
   {
      BGVAR(s64_SysNotOk) &= ~PHASE_FIND_FLT_MASK;
      // Restore PHASEFIND on ENABLE
      if( (VAR(AX0_s16_Motor_Enc_Type) == 2) || (VAR(AX0_s16_Motor_Enc_Type) == 4) )
      {
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_IDLE;
         BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_REQUESTED_MASK;
         BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_SUCCESS_MASK;
      }
   }
   else if (!MaskedFault(drive, PHASE_FIND_FLT_MASK, 0))
   {
      if ( (BGVAR(u8_PhaseFind_State) == PHASE_FIND_FAULT_STATE)              ||
           ( ( (VAR(AX0_s16_Motor_Enc_Type) == 1) ||
               (VAR(AX0_s16_Motor_Enc_Type) == 3)   )                      &&
             (Enabled(drive))                                              &&
             ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_REQUESTED_MASK) == 0)  )  )
      {
         if (HandleFault(drive, PHASE_FIND_FLT_MASK, 0))
         {
            p402_error_code = ERRCODE_CAN_PHASE_FIND;
            CANFaultControl(drive, 0, PHASE_FIND_FLT_MASK, 0);
         }
      }
   }

   if ( ( ( ( (VAR(AX0_s16_Motor_Enc_Type) >= 1) &&
              (VAR(AX0_s16_Motor_Enc_Type) <= 4)   ) ||
            (VAR(AX0_s16_Motor_Enc_Type) == 12)        ) &&
          FEEDBACK_WITH_ENCODER                            )               ||
        ( (BGVAR(u16_FdbkType) == FANUC_COMM_FDBK)                      &&
          ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_REQUESTED_MASK) != 0)  )  )
   { // PhaseFind is required if Motor Enc. Type is 1 through 4 AND Feedbak Type "with Encoder"
      if (((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_SUCCESS_MASK) == 0) &&
          (!(BGVAR(s64_SysNotOk) & PHASE_FIND_FLT_MASK))                 )
      {
         if (VAR(AX0_u16_Motor_Comm_Type) == 0)
            BGVAR(u64_Sys_Warnings) |= PHASE_FIND_REQ_WRN_MASK;
         else
            BGVAR(u64_Sys_Warnings) &= ~PHASE_FIND_REQ_WRN_MASK;
      }
      else
         BGVAR(u64_Sys_Warnings) &= ~PHASE_FIND_REQ_WRN_MASK;
   }
   else BGVAR(u64_Sys_Warnings) &= ~PHASE_FIND_REQ_WRN_MASK;
}


void MotorSetupFault(int drive,int mode)
{
   // AXIS_OFF;
   if (MaskedFault(drive, MOTOR_SETUP_FLT_MASK, 0)) return;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~MOTOR_SETUP_FLT_MASK;
      return;
   }

   if ( (mode == DETECT)                                    &&
        (BGVAR(u16_Motor_Setup_State) == MOTOR_SETUP_IDLE)  &&
        (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_FAILED)  )
   {
      if (HandleFault(drive, MOTOR_SETUP_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_MOTOR_SETUP;
         CANFaultControl(drive, 0, MOTOR_SETUP_FLT_MASK, 0);
      }
   }
   if (mode == FLT_CLR)
   {
      BGVAR(u32_Motor_Setup_Status) &=  ~MOTOR_SETUP_FAILED;
      BGVAR(s64_SysNotOk) &= ~MOTOR_SETUP_FLT_MASK;
   }
}


void PllFault(int drive,int mode)
{
   unsigned int u16_activate_pll_wrn = 0;

   // AXIS_OFF;
   if (MaskedFault(drive, PLL_FLT_MASK, 0)) return;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~PLL_FLT_MASK;
      return;
   }

   // check if need to activate the pll_is_unlocked warning
   if ((u16_Pll_State != PLL_LOCKED) && BGVAR(u8_Sync_Mode)> 0)
   {
      // fix IPR 1313: Communication on DSP402 profile 'Wn709' occures
      // pll unlocked warning is shown only in NMT operational state and
      // In CAN drive: if at least one RPDO is enabled and defiend as sync PDO.
      // if all RPDOs are event triggered (or disabled), the warning is not shown.


      if (!IS_CAN_DRIVE_AND_COMMODE_1)  // for none-CAN drive, set warning
      {
         u16_activate_pll_wrn = 1;
      }
      else if (FalIsInOperationModeAndPdo(0))   // if drive is in operational state (dont consider if rpdo arrived or not)
      {
         // if rpdo is enabled and is event triggered rpdo (check all 4 rpdos)
         if ((((p301_n1_rpdo_para.cobId & 0x80000000) == 0) && (p301_n1_rpdo_para.transType < 253)) ||
             (((p301_n2_rpdo_para.cobId & 0x80000000) == 0) && (p301_n2_rpdo_para.transType < 253)) ||
             (((p301_n3_rpdo_para.cobId & 0x80000000) == 0) && (p301_n3_rpdo_para.transType < 253)) ||
             (((p301_n4_rpdo_para.cobId & 0x80000000) == 0) && (p301_n4_rpdo_para.transType < 253))  )
         {
            u16_activate_pll_wrn = 1;
         }
      }
   }

   if (u16_activate_pll_wrn)
   {
      BGVAR(u64_Sys_Warnings) |= PLL_WRN_MASK;
      if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
      {  // in lxm28, this bit (bit 14) has different meaning (x_end in 402fsa.c)
         p402_statusword &= ~ CAN_PLL_LOCKED_MASK;
      }
   }
   else
   {
      BGVAR(u64_Sys_Warnings) &= ~PLL_WRN_MASK;
      if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
      {  // in lxm28, this bit (bit 14) has different meaning (x_end in 402fsa.c)
         p402_statusword |= CAN_PLL_LOCKED_MASK;
      }
   }

   // The PLL state unlocked can only be reached in case that the Drive was at least
   // one time in the locked state before. Only then the error is being generated.
   if ((mode == DETECT) && (u16_Pll_State == PLL_UNLOCKED))
   {
      if (HandleFault(drive, PLL_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_PLL;
         if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
         {  // in lxm28, this bit (bit 14) has different meaning (x_end in 402fsa.c)
            p402_statusword &= ~ CAN_PLL_LOCKED_MASK;
         }

         CANFaultControl(drive, 0, PLL_FLT_MASK, 0);
      }
   }

   if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk) &= ~PLL_FLT_MASK;
      if (u16_Pll_State == PLL_UNLOCKED)   u16_Pll_State = PLL_IDLE;
   }
}


//**********************************************************
// Function Name: LexiumLimitSwitchWarn
// Description: This function is used by a lexium product in order to generate
//              the limit switch warnings. The lexium limit switch handling is
//              very specific an so is the indication of the limit switch. Lexium
//              distinguish between software and hardware limit switch,
//              it shows AL014 for "reverse hardware limit switch" and AL015 for
//              "forward hardware limit switch error". AL014 and AL015 are shown on the
//              display when raising the "CW_HW_LS_WRN_MASK" and "CCW_HW_LS_WRN_MASK".
//              The software limit switches are handled in the same manner and are
//              displayed via the warning AL283 for clockwise and AL285 for counter-
//              clockwise warning.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LexiumLimitSwitchWarn(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Ask a specific Lexium LS variable what to display
   if (BGVAR(u8_Show_Lex_LS_Warning) & LEX_LS_AL015_WARNING) // If AL015 is supposed to be shown
   {
      BGVAR(u64_Sys_Warnings) |= CW_HW_LS_WRN_MASK;
      if ((BGVAR(u8_Lex_LS_Fault_Log_Only_Once) & 0x1) == 0)
      {
         BGVAR(u8_Lex_LS_Fault_Log_Only_Once) |= 0x1;
         // offset the number by 128 to skip the faults on the start of the table and get to the warnings in the Lexium fault table
         WriteToFaultLog(drive, (GetFaultId((unsigned long long)CW_HW_LS_WRN_MASK, (int)0) + 128));
      }
   }
   else // Clear AL015
   {
      BGVAR(u64_Sys_Warnings) &= ~CW_HW_LS_WRN_MASK;
      BGVAR(u8_Lex_LS_Fault_Log_Only_Once) &= ~0x1;
   }

   if (BGVAR(u8_Show_Lex_LS_Warning) & LEX_LS_AL014_WARNING) // If AL014 is supposed to be shown
   {
      BGVAR(u64_Sys_Warnings) |= CCW_HW_LS_WRN_MASK;
      if ((BGVAR(u8_Lex_LS_Fault_Log_Only_Once) & 0x2) == 0)
      {
         BGVAR(u8_Lex_LS_Fault_Log_Only_Once) |= 0x2;
         // offset the number by 128 to skip the faults on the start of the table and get to the warnings in the Lexium fault table
         WriteToFaultLog(drive, (GetFaultId((unsigned long long)CCW_HW_LS_WRN_MASK, (int)0) + 128));
      }
   }
   else // Clear AL014
   {
      BGVAR(u64_Sys_Warnings) &= ~CCW_HW_LS_WRN_MASK;
      BGVAR(u8_Lex_LS_Fault_Log_Only_Once) &= ~0x2;
   }

   if (BGVAR(u8_Show_Lex_LS_Warning) & LEX_LS_AL283_WARNING) // If AL283 is supposed to be shown
   {
      BGVAR(u64_Sys_Warnings) |= CW_SW_LS_WRN_MASK;
      if ((BGVAR(u8_Lex_LS_Fault_Log_Only_Once) & 0x4) == 0)
      {
         BGVAR(u8_Lex_LS_Fault_Log_Only_Once) |= 0x4;
         // offset the number by 128 to skip the faults on the start of the table and get to the warnings in the Lexium fault table
         WriteToFaultLog(drive, (GetFaultId((unsigned long long)CW_SW_LS_WRN_MASK, (int)0) + 128));
      }
   }
   else // Clear AL283
   {
      BGVAR(u64_Sys_Warnings) &= ~CW_SW_LS_WRN_MASK;
      BGVAR(u8_Lex_LS_Fault_Log_Only_Once) &= ~0x4;
   }

   if (BGVAR(u8_Show_Lex_LS_Warning) & LEX_LS_AL285_WARNING) // If AL285 is supposed to be shown
   {
      BGVAR(u64_Sys_Warnings) |= CCW_SW_LS_WRN_MASK;
      if ((BGVAR(u8_Lex_LS_Fault_Log_Only_Once) & 0x8) == 0)
      {
         BGVAR(u8_Lex_LS_Fault_Log_Only_Once) |= 0x8;
         // offset the number by 128 to skip the faults on the start of the table and get to the warnings in the Lexium fault table
         WriteToFaultLog(drive, (GetFaultId((unsigned long long)CCW_SW_LS_WRN_MASK, (int)0) + 128));
      }
   }
   else // Clear AL285
   {
      BGVAR(u64_Sys_Warnings) &= ~CCW_SW_LS_WRN_MASK;
      BGVAR(u8_Lex_LS_Fault_Log_Only_Once) &= ~0x8;
   }
}


void LimitSwitchWarn(int drive)
{
   // AXIS_OFF;
   unsigned int u16_temp_cw_ls, u16_temp_ccw_ls;
   REFERENCE_TO_DRIVE;

   // For Schneider, or when setting Limits Mode to ignore Transient (Phantom) Bits
   if ( (u16_Product == SHNDR) || (u16_Product == SHNDR_HW) ||
        ((VAR(AX0_u16_SW_Pos_lim) & 0x02) == 0x02)            )
   {  // Do not use the hardware limit-switch phantom bits
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS)  & 0x03;
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS) & 0x03;
   }
   else
   {  // Involve the hardware limit-switch phantom bits
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS);
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS);
   }

   //Activating the Hold in case that at least one of the Limit switches has been activated
   if ( (((VAR(AX0_u16_SW_Pos_lim) & 0x01) == 0) && (VAR(AX0_u16_CW_CCW_Inputs) == 0)) ||
        ((u16_temp_cw_ls == 0) && (u16_temp_ccw_ls == 0))                                )
   {
      BGVAR(u64_Sys_Warnings) &= ~(CW_SW_LS_WRN_MASK | CCW_SW_LS_WRN_MASK | CW_HW_LS_WRN_MASK | CCW_HW_LS_WRN_MASK | CW_CCW_HW_LS_WRN_MASK | CW_CCW_SW_LS_WRN_MASK);
      return;
   }

   if ((VAR(AX0_u16_SW_Pos_lim) & 0x01) == 0)
      BGVAR(u64_Sys_Warnings) &= ~(CW_SW_LS_WRN_MASK | CCW_SW_LS_WRN_MASK | CW_CCW_SW_LS_WRN_MASK);

   if ( VAR(AX0_u16_CW_CCW_Inputs)                                               &&
        (((u16_temp_cw_ls & 0xE) == 0) || ((u16_temp_ccw_ls & 0xE) == 0))  )
   {
      if ((u16_temp_cw_ls & 0xE) == 0)
         BGVAR(u64_Sys_Warnings) &= ~(CW_HW_LS_WRN_MASK | CW_CCW_HW_LS_WRN_MASK);
      if ((u16_temp_ccw_ls & 0xE) == 0)
         BGVAR(u64_Sys_Warnings) &= ~(CCW_HW_LS_WRN_MASK | CW_CCW_HW_LS_WRN_MASK);
   }

   if ((u16_temp_cw_ls & 0xE) != 0)   BGVAR(u64_Sys_Warnings) |= CW_HW_LS_WRN_MASK;

   if ((u16_temp_cw_ls & 0x1) != 0)   BGVAR(u64_Sys_Warnings) |= CW_SW_LS_WRN_MASK;

   if ((u16_temp_ccw_ls & 0xE) != 0)  BGVAR(u64_Sys_Warnings) |= CCW_HW_LS_WRN_MASK;

   if ((u16_temp_ccw_ls & 0x1) != 0)  BGVAR(u64_Sys_Warnings) |= CCW_SW_LS_WRN_MASK;

   if (((u16_temp_cw_ls & 0xE) != 0) && ((u16_temp_ccw_ls & 0xE) != 0))
   {
      BGVAR(u64_Sys_Warnings) &= ~(CCW_HW_LS_WRN_MASK | CW_HW_LS_WRN_MASK);
      BGVAR(u64_Sys_Warnings) |= CW_CCW_HW_LS_WRN_MASK;
   }

   if (((u16_temp_cw_ls & 0x1) != 0) && ((u16_temp_ccw_ls & 0x1) != 0))
   {
      BGVAR(u64_Sys_Warnings) &= ~(CCW_SW_LS_WRN_MASK | CW_SW_LS_WRN_MASK);
      BGVAR(u64_Sys_Warnings) |= CW_CCW_SW_LS_WRN_MASK;
   }

   return;
}


void MotorPhaseDisconnect(int drive,int mode)
{
   // AXIS_OFF;
   
   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~PHASE_DISCNCT_SCN_FLT_MASK;
      return;
   }

   if (mode == FLT_CLR)
   {  // signalling the assembler to clear the UVW flags
     
// AX0_u16_Phase_Disconnect_Sat_Counter are cleared automaticly at "disable" in the currloop.asm

      BGVAR(s64_SysNotOk) &= ~PHASE_DISCNCT_SCN_FLT_MASK;
   }

   if (MaskedFault(drive, PHASE_DISCNCT_SCN_FLT_MASK, 0)) return;
   if ((BGVAR(s64_Faults_Mask) & PHASE_DISCNCT_SCN_FLT_MASK) == 0)
      return;

   if (mode == DETECT)
   {
      if ((VAR(AX0_u16_Phase_Discon_Err_Sigma) > 32000LL)                 ||
         (BGVAR(u16_Brake_Release_State) == BRAKE_RELEASE_REPORT_FAILURE)) // motor phase disconnected on enable command
      {
         if (HandleFault(drive, PHASE_DISCNCT_SCN_FLT_MASK, 0) == 1)
         {
            VAR(AX0_u16_BrakeOn) = 1;
            p402_error_code = ERRCODE_CAN_PHASE_DISCNCT_SCN;
            CANFaultControl(drive, 0, PHASE_DISCNCT_SCN_FLT_MASK, 0);
            BGVAR(u16_Brake_Release_State) = BRAKE_RELEASE_IDLE;
         }
      }
   }

   return;
}


void SensorLessFaults(int drive, int mode)
{
   // AXIS_OFF;

   if (BGVAR(u8_Sl_Mode) == 0)
   {
      BGVAR(s64_SysNotOk) &= ~(SL_INIT_FAILURE | SL_COMMUATATION_LOST);
      BGVAR(u64_Sys_Warnings) &= ~(SL_MODE_EN_MASK | SL_ENABLED_HF_MASK | SL_ENABLED_BEMF_MASK);
      return;
   }

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~SL_INIT_FAILURE;
      return;
   }

   if ((VAR(AX0_s16_Sensorless_Control_Bits) & SL_KALMAN_MASK) != 0) BGVAR(u64_Sys_Warnings) |= SL_ENABLED_KALMAN_MASK;
   else BGVAR(u64_Sys_Warnings) &=~SL_ENABLED_KALMAN_MASK;

   if ((VAR(AX0_s16_Sensorless_Control_Bits) & SL_EN_HF_INJECTION_MASK) != 0) BGVAR(u64_Sys_Warnings) |= SL_ENABLED_HF_MASK;
   else BGVAR(u64_Sys_Warnings) &=~SL_ENABLED_HF_MASK;

   if ((VAR(AX0_s16_Sensorless_Control_Bits) & SL_EN_BEMF_MASK) != 0) BGVAR(u64_Sys_Warnings) |= SL_ENABLED_BEMF_MASK;
   else BGVAR(u64_Sys_Warnings) &=~SL_ENABLED_BEMF_MASK;

   if ( (BGVAR(u64_Sys_Warnings) & (SL_ENABLED_BEMF_MASK|SL_ENABLED_HF_MASK)) == 0) BGVAR(u64_Sys_Warnings) |= SL_MODE_EN_MASK;
   else BGVAR(u64_Sys_Warnings) &= ~SL_MODE_EN_MASK;

   if ( (BGVAR(s16_Sensorless_Handler_State) == SL_FAILED_PROC_RUNNING)             ||
        (BGVAR(s16_Sensorless_Handler_State) == SL_FAILED_PHASE_FIND_COMMAND)       ||
        (BGVAR(s16_Sensorless_Handler_State) == SL_FAILED_PHASE_FIND_PROC_FAILED_0) ||
        (BGVAR(s16_Sensorless_Handler_State) == SL_FAILED_PHASE_FIND_PROC_FAILED_3) ||
        (BGVAR(s16_Sensorless_Handler_State) == SL_FAILED_HF_INIT_PHASE_FAILED)     ||
        (BGVAR(s16_Sensorless_Handler_State) == SL_FAILED_ESTMOTORPARAM_FAILED) )
   {
      if (mode == DETECT)
      {
         if (HandleFault(drive, SL_INIT_FAILURE, 0))
         {
            p402_error_code = ERRCODE_CAN_SL_INIT;
            CANFaultControl(drive, 0, SL_INIT_FAILURE, 0);
         }
      }
      else
      {
         BGVAR(s64_SysNotOk) &= ~SL_INIT_FAILURE;
         BGVAR(s16_Sensorless_Handler_State) = SL_STATE_RESET_MACHINE;
      }
   }
}


void EmergencyStop(int drive, int mode)
{
   // AXIS_OFF;
   int fault_bit = 0;

   if (MaskedFault(drive, EMERGENCY_STOP_FLT_MASK, 0)) return;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~EMERGENCY_STOP_FLT_MASK;
      return;
   }

   fault_bit = BGVAR(u16_EStop_Indication);

   if ((mode == DETECT) && (fault_bit))
   {
      if (HandleFault(drive, EMERGENCY_STOP_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_EMERGENCY_STOP;
         CANFaultControl(drive, 0, EMERGENCY_STOP_FLT_MASK, 0);
      }
   }

   if ( (mode == FLT_CLR) && (!fault_bit) )  BGVAR(s64_SysNotOk) &= ~EMERGENCY_STOP_FLT_MASK;
}


// STALLVEL
int SalStallVelCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL) return (VALUE_TOO_LOW);
   if (lparam > (long long)2147483647) return (VALUE_TOO_HIGH);

   BGVAR(u32_Stall_Vel) = (unsigned long)lparam;

   return (SAL_SUCCESS);
}


void StallFault(int drive, int mode)
{
   // AXIS_OFF;

   int fault_bit = 0;
   int fault_indication = 0;
   long s32_abs_vel_fb;

   if (MaskedFault(drive, STALL_FLT_MASK, 0)) return;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~STALL_FLT_MASK;
      return;
   }

   if ( LVAR(AX0_s32_Vel_Var_Fb_0) >= 0 )
      s32_abs_vel_fb = LVAR(AX0_s32_Vel_Var_Fb_0);
   else
      s32_abs_vel_fb = -LVAR(AX0_s32_Vel_Var_Fb_0);

   if ( ( (long)BGVAR(u32_EqCrrnt) > (long)BGVAR(s32_Motor_I_Cont_Internal) )         &&   // current I is greater than MICONT
        ( (long)BGVAR(u32_EqCrrnt) > (((long)VAR(AX0_s16_I_Sat_Hi) * 0x7333) >> 15) ) &&   // current I is saturated (above 90% of sat value)
        ( s32_abs_vel_fb <= BGVAR(u32_Stall_Vel) )                                    &&   // velocity is less than or equal user threshold STALLVEL
        ( BGVAR(s32_Stall_Time) > 0 )                                                   )  // STALLTIME > 0 (STALLTIME 0 inhibits this fault)
      fault_bit = 1;

   // indicate fault if fault state consists more time than user threshold STALLTIME
   switch (BGVAR(u16_Stall_Fault_State))
   {
      case 0:
         if (fault_bit == 1)
         {
            BGVAR(s32_Stall_Fault_Timer) = Cntr_1mS;
            BGVAR(u16_Stall_Fault_State) = 1;
         }
      break;

      case 1:
         if (fault_bit == 0)
            BGVAR(u16_Stall_Fault_State) = 0;
         else
         {
            if ( PassedTimeMS(BGVAR(s32_Stall_Time), BGVAR(s32_Stall_Fault_Timer)) )
            {
               fault_indication = 1;
               BGVAR(u16_Stall_Fault_State) = 2;
            }
         }
      break;

      case 2:
         if (fault_bit == 0)
            BGVAR(u16_Stall_Fault_State) = 0;
         else
            fault_indication = 1;
      break;

      default:
         BGVAR(u16_Stall_Fault_State) = 0;
      break;
   }

   if ((mode == DETECT) && (fault_indication))
   {
      if (HandleFault(drive, STALL_FLT_MASK, 0))
      {
         p402_error_code = ERRCODE_CAN_STALL;
         CANFaultControl(drive, 0, STALL_FLT_MASK, 0);
      }
   }

   if ( (mode == FLT_CLR) && (!fault_indication) )  BGVAR(s64_SysNotOk) &= ~STALL_FLT_MASK;
}


//**********************************************************
// Function Name: CanOpenShortPdoFault
// Description:
//    This function checks if a PDO arrived whihc is shorter than defined length.
//    PDO length is checked in the CAN interrupt (in port library) and a flag is set as necessary
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CanOpenShortPdoFault(int drive,int mode)
{
   int s16_fault_set = 1;

   if (MaskedFault(drive, CAN_PDO_TOO_SHORT_FLT_MASK, s16_fault_set)) return;

   if (mode == DETECT)
   {
      if (BGVAR(s16_CAN_Flags) & CANOPEN_PDO_TOO_SHORT_MASK)
      {
         if (HandleFault(drive, CAN_PDO_TOO_SHORT_FLT_MASK, s16_fault_set))
         {
            p402_error_code = ERRCODE_BAD_PDOPARA;
            CANFaultControl(drive, 0, CAN_PDO_TOO_SHORT_FLT_MASK, s16_fault_set);
         }

         BGVAR(s16_CAN_Flags) &= ~CANOPEN_PDO_TOO_SHORT_MASK;
      }
   }

   if (mode == FLT_CLR)
      BGVAR(s64_SysNotOk_2) &= ~CAN_PDO_TOO_SHORT_FLT_MASK;
}


//**********************************************************
// Function Name: CanOpenHeartBeatLostFault
// Description:
//    This function checks if hear beat lost flag was rased and issue a fault if needed..
//    the flag is set in mGuardErrorInd(). This is a callback from port library for canopen.
//    Second bit of AX0_s16_CAN_Flags is the flag for the fault.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CanOpenHeartBeatLostFault(int drive,int mode)
{
   LOCAL_NODE_T   *pNode;     /* local pointer to node structure */
   // AXIS_OFF;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk) &= ~CAN_HEARTBEAT_NODEGUARD_FLT_MASK;
      return;
   }

   if (mode == DETECT)
   {
      if (BGVAR(s16_CAN_Flags) & CANOPEN_HB_NG_LOST_MASK)
      {
        pNode = &GL_ARRAY(co_Node);

         //the state transaction depends on SDO 0x1029
         if ((p301_error_behaviour[1] == 0) && (pNode->eState == OPERATIONAL))
         {
            setNodeState(BACKGROUND_CONTEXT, PRE_OPERATIONAL CO_COMMA_LINE_PARA);
         }

         //else if ((p301_error_behaviour[1] == 1) - not needed sine value of 1 means stay in the same state

         else if ((p301_error_behaviour[1] == 2) && (pNode->eState != STOPPED))
         {
            setNodeState(BACKGROUND_CONTEXT, STOPPED CO_COMMA_LINE_PARA);		
         }

         if (HandleFault(drive, CAN_HEARTBEAT_NODEGUARD_FLT_MASK, 0))
       {
            p402_error_code = ERRCODE_CAN_HEARTBEAT_NODEGUARD;
            CANFaultControl(drive, 0, CAN_HEARTBEAT_NODEGUARD_FLT_MASK, 0);
         }
         BGVAR(s16_CAN_Flags) &= ~CANOPEN_HB_NG_LOST_MASK;
      }
   }

   if (mode == FLT_CLR)
      BGVAR(s64_SysNotOk) &= ~CAN_HEARTBEAT_NODEGUARD_FLT_MASK;
}


//**********************************************************
// Function Name: ModbusNodeGuardFault
// Description:
// Author: Udi
//**********************************************************
/*void ModbusNodeGuardFault(int drive,int mode)
{
   // AXIS_OFF;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk_2) &= ~MODBUS_NODEGUARD_FLT_MASK;
      BGVAR(u64_Sys_Warnings) &= ~MODBUS_NODEGUARD_WRN_MASK;
      return;
   }

   if (mode == DETECT)
   {
      if (u16_Modbus_NodeGuard_Flt & MODBUS_NODEGUARD_FLT_INDC_MASK)
      {
         if (HandleFault(drive, MODBUS_NODEGUARD_FLT_MASK, 1))
       {
            //set bit 2 to signal active disable mechanism toi use node guard event deceleration
            u16_Modbus_NodeGuard_Flt |= MODBUS_NODEGUARD_AD_INDC_MASK;

            p402_error_code = ERRCODE_CAN_MODBUS_NODEGUARD_LOST;
            CANFaultControl(drive, 0, MODBUS_NODEGUARD_FLT_MASK, 1);
         }
         // reset node guard fault bit
         u16_Modbus_NodeGuard_Flt &= ~MODBUS_NODEGUARD_FLT_INDC_MASK;

         BGVAR(u64_Sys_Warnings) &= ~MODBUS_NODEGUARD_WRN_MASK;
      }
     else if (u16_Modbus_NodeGuard_Flt & MODBUS_NODEGUARD_WRN_INDC_MASK)
         BGVAR(u64_Sys_Warnings) |= MODBUS_NODEGUARD_WRN_MASK;
      else
         BGVAR(u64_Sys_Warnings) &= ~MODBUS_NODEGUARD_WRN_MASK;

   }

   if (mode == FLT_CLR)
   {
      u16_Modbus_NodeGuard_Flt &= ~MODBUS_NODEGUARD_FLT_INDC_MASK;
      BGVAR(s64_SysNotOk_2) &= ~MODBUS_NODEGUARD_FLT_MASK;
      BGVAR(u16_Hold_Bits) &=  ~HOLD_MODBUS_NODEGUARD_MASK;
   }
}*/


//**********************************************************
// Function Name: ModbusConfigTransferFault
// Description:
// Author: Udi
//**********************************************************
/*void ModbusConfigTransferFault(int drive,int mode)
{

   if (mode == DETECT)
   {
      if (u16_Modbus_Config_Transfer_Flt == 1)
      {
         if (HandleFault(drive, MODBUS_CONFIG_TRNSFER_FLT_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_MODBUS_CONFIG_TRANSFER;
            CANFaultControl(drive, DETECT, MODBUS_CONFIG_TRNSFER_FLT_MASK, 1);
         }
         // reset  fault
         u16_Modbus_Config_Transfer_Flt = 0;
      }
   }

   if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~MODBUS_CONFIG_TRNSFER_FLT_MASK;
   }
}*/



//**********************************************************
// Function Name: SchneiderFault
// Description:
//   This function handles Schneider drive mode faults
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
/*void SchneiderFault(int drive, int mode)
{
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;
   MotorTorqueOvershootFault(drive,mode);
   PulseTrainFreqTooHighFault(drive,mode);
   PositionDeviationAlarm(drive,mode);
   ModbusNodeGuardFault(drive,mode);
   ModbusConfigTransferFault(drive,mode);
   EncoderSimulationFreqFault(drive, mode);
}*/


//**********************************************************
// Function Name: VeMaxFault
// Description:
//   This function handles VE exceeded
//   For Schneider P2-34 (SDEV) overspeed ("S007") fault
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void VeMaxFault(int drive, int mode)
{
   // AXIS_OFF;
   long long s64_half_for_rounding = 0LL;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk_2) &= ~VEMAX_FLT_MASK;
      return;
   }

   if (BGVAR(u32_Ve_Max) == 0L) return;
   if (mode == DETECT)
   {
       // AXIS_OFF;
       if ((VAR(AX0_s16_Opmode) != 0) && (VAR(AX0_s16_Opmode) != 1))  return; // valid only in velocity mode (speed)
       if (VAR(AX0_s16_Vel_Var_Ref_0) == 0) return;
       // active only when there is a velocity command (VCMD >0)

       // convert from internal velocity (out of the loop) to internal units (in the loop)

      if (VAR(AX0_u16_Out_To_In_Vel_Shr) > 0)
         s64_half_for_rounding = 1LL << ((long long)VAR(AX0_u16_Out_To_In_Vel_Shr) - 1);

      s64_half_for_rounding = (long long)((((long long)BGVAR(u32_Ve_Max) * (long long)LVAR(AX0_s32_Out_To_In_Vel_Fix)) + s64_half_for_rounding) >> (long long)VAR(AX0_u16_Out_To_In_Vel_Shr));
      if (s64_half_for_rounding > 0x7fffLL)
      {
         s64_half_for_rounding = 0x7fffLL;
      }

      if ((int)s64_half_for_rounding < abs(VAR(AX0_s16_Vel_Var_Err)))
      {
         if (MaskedFault(drive, VEMAX_FLT_MASK, 1)) return; // if fault is masked dont show it

         if (HandleFault(drive,VEMAX_FLT_MASK , 1))
         {
            p402_error_code = ERRCODE_CAN_VEMAX;
            CANFaultControl(drive, DETECT, VEMAX_FLT_MASK, 1);
         }
      }
   }

   else if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~VEMAX_FLT_MASK;
   }
}


//**********************************************************
// Function Name: MotorTorqueOvershootFault
// Description:
//   This function clears Motor Torque Overshoot Fault
//   For Schneider P1-57(CRSHA) & P1-58(CRSHT) ("P1") fault
//   the detection is in the 1ms interupt because the 1ms interval required in this feature
// Author: Moshe
// Algorithm:
// Revisions: Nitsan: handle fault occurence here instead of in interrupt function.
//**********************************************************
/*void MotorTorqueOvershootFault(int drive, int mode)
{
   int s16_fault_set = 1;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~MOTOR_TORQUE_OVERSHOOT_MASK;
      BGVAR(s16_MOTOR_OVRSHOOT_STATE) = 0;   // set state machine back to idle
   }
   else if ((mode == DETECT) && (BGVAR(s16_MOTOR_OVRSHOOT_STATE) == 2))
   {
      if (MaskedFault(drive,MOTOR_TORQUE_OVERSHOOT_MASK, s16_fault_set)) return;

      if (HandleFault(drive,MOTOR_TORQUE_OVERSHOOT_MASK, s16_fault_set))
      {
         p402_error_code = ERRCODE_CAN_MOTOR_TORQUE_OVERSHOOT;
         CANFaultControl(drive, 0, MOTOR_TORQUE_OVERSHOOT_MASK, s16_fault_set);
      }
   }
}*/


//**********************************************************
// Function Name: PulseTrainFreqTooHighFault
// Description:
//   This function clears pulse train frequency too high  Fault
//   For Schneider P2-65 GBIT bit: 6 ("F2H") fault
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void PulseTrainFreqTooHighFault(int drive, int mode)
{
   // AXIS_OFF;

   // Ignore this fault during Burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
   {
      BGVAR(s64_SysNotOk_2) &= ~PULSE_TRAIN_FREQ_TOO_HIGH_MASK;
      return;
   }

   if (mode == DETECT)
   {
      if (VAR(AX0_s16_Opmode) != 4) return; // valid only in PT (gear) mode 4
      if (BGVAR(u16_P2_65_GBIT) & 0x040) return; // feature is disabled GBIT bit 6

      if (((unsigned long)labs(BGVAR(s32_Encoder_Follower2_Counter_Delta_1ms))) > BGVAR(u32_Max_Pulse_Train_Frequancy))
      {
         if (BGVAR(u32_Max_Pulse_Train_Frequancy) == MAX_ALLOWED_EXTERNAL_FREQUENCY)
         {// In case using max allowed frequency value,
          // we can't have extra 10% so, count 10 times when value is above the maximum frequency and
          // only if it happens 10 or more time constantly, trigger a fault.
            if ( (++BGVAR(u8_Freq_Too_High_Counter)) < 10 ) return;
         }
       //  if (MaskedFault(drive, PULSE_TRAIN_FREQ_TOO_HIGH_MASK, 1)) return; // if fault is masked dont show it
         if (HandleFault(drive, PULSE_TRAIN_FREQ_TOO_HIGH_MASK, 1))             // else set the fault
         {
            p402_error_code = ERRCODE_CAN_PULSE_TRAIN_FREQ_TOO_HIGH;
            CANFaultControl(drive, DETECT, PULSE_TRAIN_FREQ_TOO_HIGH_MASK, 1);
         }
      }
      else
      {// no fault so reset the counter
         if (BGVAR(u8_Freq_Too_High_Counter) > 0)
       {
            BGVAR(u8_Freq_Too_High_Counter)--;
         }
      }
   }
   else if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~PULSE_TRAIN_FREQ_TOO_HIGH_MASK;

      // reset the max freq fault counter
      BGVAR(u8_Freq_Too_High_Counter) = 0;
   }
}


//**********************************************************
// Function Name: PositionDeviationAlarm
// Description:
//   This function handles Position deviation detection for Schneider drive
//   For Schneider  ("AL380") fault
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
/*void PositionDeviationAlarm(int drive, int mode)
{
   // AXIS_OFF;
   if (mode == DETECT)
   {   // check if AL380 is not enabled return and do nothing.
      if (BGVAR(u16_P1_48_MCOK_Output_Selection) & 0x01) return;

      if ((VAR(AX0_u16_Inpos) == 0) && //PEINPOS = 0 (position error) is higher then the treshhold (P1-54)
         (VAR(AX0_s16_Stopped) == 2)) // and position completed OK so
      {
            // trigger Wn380 or AL564 end of motion position deviation warning/fault !!!
            if ((BGVAR(u16_P1_48_MCOK_Output_Selection) & 0xf0) == 0x20)
            {
               if (MaskedFault(drive, POSITION_DEVIATION_FAULT_MASK, 1)) return; // if fault is masked dont show it

               // P1-48 nibble B == 2
               // trigger AL564
               if (HandleFault(drive, POSITION_DEVIATION_FAULT_MASK, 1))
               {
                  p402_error_code = ERRCODE_CAN_POSITION_DEVIATION;
                  CANFaultControl(drive, 0, POSITION_DEVIATION_FAULT_MASK, 1);
               }
            }
            else if ((BGVAR(u16_P1_48_MCOK_Output_Selection) & 0xf0) == 0x10)
            {
               // P1-48 nibble B == 1
               // trigger Wn380
               BGVAR(u64_Sys_Warnings) |= WRN_POSITION_DEVIATION_DIG_OUTPUT_MASK;
            }
      }
   }

   else if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~POSITION_DEVIATION_FAULT_MASK;
      BGVAR(u64_Sys_Warnings) &= ~WRN_POSITION_DEVIATION_DIG_OUTPUT_MASK;
   }
}
*/

//**********************************************************
// Function Name: GetLexiumFaultNumberByBit
// Description:
//    This function returns the Lexium specific fault number according to a give fault bit
//    u64_fault_mask: the bit for the fault which is needed
//    s16_fault_set: 0 for bits in s64_SysNotOk
//                   1 for bits in s64_SysNotOk_2
//                   2 for bits in u64_Sys_Warnings
//    0  = No fault/warning active
//    >0 = ALxyz value for the Lexium
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(GetLexiumFaultNumberByBit, "ramfunc_3")
int GetLexiumFaultNumberByBit(int drive, unsigned long long u64_fault_mask, int s16_fault_set)
{
   int s16_lexium_fault_number = 0;
   int s16_temp_index = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (s16_fault_set == 0)
   {
      s16_lexium_fault_number = 1; // Add 1 since the faults are from 1...64 in "s64_SysNotOk"
   }
   else if (s16_fault_set == 1)
   {
      s16_lexium_fault_number = 65;  // Add 65 since the faults are from 65...128 in "s64_SysNotOk_2"
   }
   else if (s16_fault_set == 2)
   {
      s16_lexium_fault_number = 129;  // Add 129 since the warnings are from 129...192 in "u64_Sys_Warnings"
   }

   if (s16_lexium_fault_number > 0)
   {
      // find the bit number of requested bit
      for (s16_temp_index=0; s16_temp_index<64; s16_temp_index++)
      {
         if (u64_fault_mask == (1LL << s16_temp_index))
         {
            break;
         }
      }

      s16_lexium_fault_number += s16_temp_index;

      if (s16_lexium_fault_number <= LEXIUM_FAULT_NUMBER_ARRAY_SIZE)
      {
         s16_lexium_fault_number = u16_Lex_Fault_Number_Array[s16_lexium_fault_number-1];
      }
   }

   return s16_lexium_fault_number;
}


//**********************************************************
// Function Name: GetLexiumFaultNumber
// Description:
//    This function returns the Lexium specific fault number of the first
//    fault or Lexium-relevant warning, which has been identified by this
//    function in the variables s64_SysNotOk and s64_SysNotOk_2 and u64_Sys_Warnings.
//
//    0  = No fault/warning active
//    >0 = ALxyz value for the Lexium
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int GetLexiumFaultNumber(int drive, long long u64_sys_warnings_mask)
{
   int s16_lexium_fault_number = 0;
   long s32_temp_index = 0;
   long long s64_mask;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // *******************************
   // If no fault and warning exists.
   // *******************************
   if ((BGVAR(s64_SysNotOk) == 0) && (BGVAR(s64_SysNotOk_2) == 0) && (BGVAR(u64_Sys_Warnings) == 0))
   {
      return(s16_lexium_fault_number); // Return fault number, which is 0
   }

   // *******************************************
   // Check "s64_SysNotOk" if a fault is pending.
   // *******************************************
   for (s32_temp_index = 0; s32_temp_index < 64; s32_temp_index++)
   {
      s64_mask = 0x0001LL << s32_temp_index;

      // The next ugly line is to hide Not_Configured fault while MTP is reading
      if (!((s64_mask == NO_COMP_FLT_MASK) && ((BGVAR(u16_Read_Mtp_State) != MTP_INIT_INIT) || (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK))))
      {
         if (BGVAR(s64_SysNotOk) & s64_mask)
         {
            // 5V fault happens on every power off. Dispaly it after delay, since if the drive is still running the fault is real.
            // Other fault display immediately
            if (  (s64_mask != DRIVE_5V_FLT_MASK)                                                  ||
                 ((s64_mask == DRIVE_5V_FLT_MASK) && (BGVAR(u16_Delayed_5V_Log_Fault_Flag) == 0) )    )
            {
               s16_lexium_fault_number = s32_temp_index + 1; // Add 1 since the faults are from 1...64 in "s64_SysNotOk"
               break; // Leave for-loop
            }
         }
      }
   }

   // ************************************************************
   // If no fault found in "s64_SysNotOk", check "s64_SysNotOk_2".
   // ************************************************************
   if (s16_lexium_fault_number == 0)
   {
      for (s32_temp_index = 0; s32_temp_index < 64; s32_temp_index++)
      {
         s64_mask = 0x0001LL << s32_temp_index;

         if (BGVAR(s64_SysNotOk_2) & s64_mask)
         {
            // AC loss faults happen on every power off. Dispaly it after delay, since if the drive is still running the fault is real.
            // Other fault display immediately
            if ( ((s64_mask != BUS_AC_LOSS_MASK) && (s64_mask != LOGIC_AC_LOSS_MASK))                         ||
                 ((s64_mask == BUS_AC_LOSS_MASK)   && (BGVAR(u16_Delayed_Line_Loss_Log_Fault_Flag) == 0) )    ||
                 ((s64_mask == LOGIC_AC_LOSS_MASK) && (BGVAR(u16_Delayed_Logic_AC_Loss_Log_Fault_Flag) == 0) )    )
            {
               s16_lexium_fault_number = s32_temp_index + 65;  // Add 65 since the faults are from 65...128 in "s64_SysNotOk_2"
               break; // Leave for-loop
            }
         }
      }
   }

   // ************************************************************************************
   // If no fault found in "s64_SysNotOk" and in s64_SysNotOk_2, check "u64_Sys_Warnings".
   // ************************************************************************************
   if (s16_lexium_fault_number == 0)
   {
      for (s32_temp_index=0; s32_temp_index<64; s32_temp_index++)
      {
         if (BGVAR(u64_Sys_Warnings) & (0x0001LL << s32_temp_index) & (~u64_sys_warnings_mask))
         {
            s16_lexium_fault_number = s32_temp_index + 129;  // Add 129 since the warnings are from 129...192 in "u64_Sys_Warnings"
            break; // Leave for-loop
         }
      }
   }

   // ************************************************************************************
   // Now convert the CDHD faults and warning value into the desired Lexium "ALxyz" value.
   // ************************************************************************************
   if ((s16_lexium_fault_number > 0) && (s16_lexium_fault_number <= LEXIUM_FAULT_NUMBER_ARRAY_SIZE))
   {
      s16_lexium_fault_number = u16_Lex_Fault_Number_Array[s16_lexium_fault_number-1];
   }

   // Return the fault number
   return (s16_lexium_fault_number);
}


//**********************************************************
// Function Name: SalGetLexiumFaultHistoryNumber
// Description:
//   This function returns the CDHD fault numbers converted into the Lexium AL
//    number out of the fault history buffer.
//      u16_fault_History_Index - Index from 0...9 with:
//                                0 = latest fault and 9 = oldest fault
//
//      Return value - 0 = No fault, >0 = fault number in the ring buffer
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalGetLexiumFaultHistoryNumber(int drive, unsigned int u16_fault_History_Index)
{
   int s16_lexium_fault_number = 0; // Set fault number to 0 = no faults
   int s16_index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // First read the CDHD fault-number out of the fault history buffer
   if (u16_fault_History_Index < FAULT_LOG_LEN)
   {
      // Calculate the index in the fault log array of the requested fault.
      s16_index = BGVAR(u16_Fault_Log_Ptr) - u16_fault_History_Index;
      if (s16_index < 0)
      {
         s16_index += FAULT_LOG_LEN;
      }
      // Check if the ring-buffer holds a fault (a value of 0xFF states that no
      // fault is in the ring-buffer yet)
      if ((BGVAR(s_Fault_Log_Image)[s16_index].fault_id & 0x00FF) != 0xFF)
      {
         s16_lexium_fault_number = BGVAR(s_Fault_Log_Image)[s16_index].fault_id;
      }
   }

   // If a fault has been found in the fault history buffer, convert this fault
   // from a CDHD fault number into a Lexium fault number.
   if ((s16_lexium_fault_number > 0) && (s16_lexium_fault_number <= LEXIUM_FAULT_NUMBER_ARRAY_SIZE))
   {
      s16_lexium_fault_number = u16_Lex_Fault_Number_Array[s16_lexium_fault_number-1];
   }

   return (s16_lexium_fault_number);
}


//**********************************************************
// Function Name: SalRead_P4_00_AlarmLogValue
// Description:
//    This function return latest alarm from the fault history
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalRead_P4_00_AlarmLogValue(long long *data, int drive)
{// this call is for P4-00 only

   *data = SalGetLexiumFaultHistoryNumber(drive, 0);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalRead_P4_01_AlarmLogValue
// Description:
//    This function return alarm number 2 - 5 from the fault history
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalRead_P4_01_AlarmLogValue(long long *data,int drive)
{// this call is for P4-01 to P4-04 and we are using the same function so we need to inc the index by 1 because P4-01 is the second one after P4-00

   long long index = (s64_Execution_Parameter[0] + 1);
   if ((index < 1LL) || (index > 4LL))
       return (VALUE_OUT_OF_RANGE);

   *data = SalGetLexiumFaultHistoryNumber(drive, index);

   return (SAL_SUCCESS);
}


// This is a no clearable fault indicating you will need to cycle power to the drive for
// a change in parameter to have affect
void CyclePowerNeededFault(int drive, int mode)
{
   if (MaskedFault(drive, CYCLE_POWER_NEEDED_FLT_MASK, 1)) return;

//   if ((mode == DETECT) && BGVAR(u16_Power_Cycle_Needed)) HandleFault(drive, CYCLE_POWER_NEEDED_FLT_MASK, 1);
//   else if ((mode == DETECT) && (BGVAR(u16_Power_Cycle_Needed)==0)) BGVAR(s64_SysNotOk_2) &= ~CYCLE_POWER_NEEDED_FLT_MASK;

   if (mode == DETECT)
   {
      if (BGVAR(u16_Power_Cycle_Needed))
      {
         if (HandleFault(drive, CYCLE_POWER_NEEDED_FLT_MASK, 1))
         {
            p402_error_code = ERRCODE_CAN_CYCLE_POWER_NEEDED;
            CANFaultControl(drive, DETECT, CYCLE_POWER_NEEDED_FLT_MASK, 1);
         }
      }
      else
      {
         BGVAR(s64_SysNotOk_2) &= ~CYCLE_POWER_NEEDED_FLT_MASK;
      }
   }
}


//**********************************************************
// Function Name: CanBusOffFault
// Description:
//   CAN bus entered BUSOFF state. so disable CAN
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CanBusOffFault(int drive, int mode)
{
   LOCAL_NODE_T   *pNode;     /* local pointer to node structure */
   unsigned int u16_enable_can_on_fpga = 0;
   
   if (MaskedFault(drive, CAN_BUSOFF_FLT_MASK, 1)) return;

   if ((mode == DETECT) && (BGVAR(u8_Comm_Mode) == 1))
   {
      // if object 0x1029 is 0x80 (manufacturer specific value), ignore can bus off fault/warning
      if (p301_error_behaviour[1] == 0x80)
      {
         // if object 0x1029==0x80, ignore bus off fault
         BGVAR(s64_SysNotOk_2) &= ~CAN_BUSOFF_FLT_MASK;
         BGVAR(u64_Sys_Warnings) &= ~WRN_CAN_BUS_ERROR;
         return;
      }

      if (BGVAR(u16_Disable_Can_Req) > 10)
      {
         // disable recieving CAN packets from fpga
         SET_FPGA_DISABLE_CAN_MODE;
         HandleFault(drive, CAN_BUSOFF_FLT_MASK, 1);

         //set NMT state according to 0x1029
         pNode = &GL_ARRAY(co_Node);
         if ((p301_error_behaviour[1] == 0) && (pNode->eState == OPERATIONAL))
         {
            setNodeState(BACKGROUND_CONTEXT, PRE_OPERATIONAL CO_COMMA_LINE_PARA);
         }
         else if (p301_error_behaviour[1] == 2)
         {
            setNodeState(BACKGROUND_CONTEXT, STOPPED CO_COMMA_LINE_PARA);
         }
      }
      else if (BGVAR(u16_Can_Error_State) != CANFLAG_ACTIVE)
      {
         // IPR 516: SE request via email from Alexander Badura (18/9/2014):
         // if NOT in CANopen mode and acess right is NOT canopen, this is a warning, else trigger a fault
         // for CDHD, always a fault
         BGVAR(u64_Sys_Warnings) |= WRN_CAN_BUS_ERROR;
      }
   }
   else if (mode == FLT_CLR)
   {
      // enable recieving CAN packets from fpga
      BGVAR(u16_Disable_Can_Req) = 0;

      if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      {
         // if node id is valid (above zero and below 128)
         if (BGVAR(u16_P3_05_CMM) != 0x0 && BGVAR(u16_P3_05_CMM) <= 0x007f)
         {
            u16_enable_can_on_fpga = 1;
         }
      }
      else if (BGVAR(u8_Comm_Mode) == 1)
      {
         u16_enable_can_on_fpga = 1;
      }
      
      if (u16_enable_can_on_fpga == 1)
      {
         SET_FPGA_NORMAL_MODE;
      }
      
      BGVAR(s64_SysNotOk_2) &= ~CAN_BUSOFF_FLT_MASK;
   }
}


// Warning if SFBMODE = 1, SFBTYPE = 1 and OPMODE = 1 or 3
void SFBModeWarning(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if ( (BGVAR(s16_SFBMode) == 1) && BGVAR(u16_SFBType)       &&
        ((VAR(AX0_s16_Opmode)==1) || (VAR(AX0_s16_Opmode)==3))  )
      BGVAR(u64_Sys_Warnings) |= SFB_MODE_EXCLUDE_WRN_MASK;
   else
      BGVAR(u64_Sys_Warnings) &= ~SFB_MODE_EXCLUDE_WRN_MASK;
}


void SFBPositionDeviationFault(int drive, int mode)
{
   if (MaskedFault(drive, SFB_POSITION_MISMATCH, 1)) return;

   if ((mode == DETECT) && BGVAR(u16_Sfb_Pe_fault))
   {
      if (HandleFault(drive, SFB_POSITION_MISMATCH, 1))
      {
         p402_error_code = ERRCODE_CAN_SFB_POSITION_MISMATCH;
         CANFaultControl(drive, 0, SFB_POSITION_MISMATCH, 1);
      }
   }
   else if (mode == FLT_CLR)
   {
      BGVAR(u16_Sfb_Pe_fault_Timer) = 0;
      BGVAR(u16_Sfb_Pe_fault) = 0;
      BGVAR(s64_SysNotOk_2) &= ~SFB_POSITION_MISMATCH;
   }
}

//**********************************************************
// Function Name: FdbkAutoDetectFault
// Description:
//   This Function monitors the Feedback Auto-Detection State, if the last (currently Hiperface) Feedback
//   is not detected (signified by the Initialization Process ending in Error State) the Fault is Set.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
void FdbkAutoDetectFault(int drive, int mode)
{
   REFERENCE_TO_DRIVE;

   if (MaskedFault(drive, FEEDBACK_AUTO_DETECT_FAULT, 1)) return;

   if (BGVAR(s16_Auto_FdbkType_Cntr) < 0)
   {
      if (HandleFault(drive, FEEDBACK_AUTO_DETECT_FAULT, 1))
/*    {
         p402_error_code = ERROR_CODE_FEEDBACK_AUTO_DETECT;
         CANFaultControl(drive, 0, FEEDBACK_AUTO_DETECT_FAULT, 1);
      } */;
      BGVAR(s16_Auto_FdbkType_Cntr) = 0; // Reset Auto-Feedback Detection Counter only after Fault is set.
   }
   else if ( (mode == FLT_CLR) && ((BGVAR(s64_SysNotOk_2) & FEEDBACK_AUTO_DETECT_FAULT) != 0) )
   {
      BGVAR(s16_Auto_FdbkType_Cntr) = 0;
      BGVAR(s64_SysNotOk_2) &= ~FEEDBACK_AUTO_DETECT_FAULT;
   }
}


//**********************************************************
// Function Name: MotorNameMTPMismatchFault
// Description:
//   This Function monitors the Motor-Name (in CDHD Memory) Comparison to MTP Motor-Name, and sets a Fault
//   if they are unequal.  Resetting is required to read Motor-Name from MTP again.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
void MotorNameMTPMismatchFault(int drive, int mode)
{
   REFERENCE_TO_DRIVE;

   if (MaskedFault(drive, MOTORNAME_MTP_MISMATCH, 1)) return;

   if (BGVAR(u16_MotorName_Mismatch) == 1)
   {
      if (HandleFault(drive, MOTORNAME_MTP_MISMATCH, 1))
/*    {
         p402_error_code = ERROR_CODE_MOTORNAME_MTP_MISMATCH;
         CANFaultControl(drive, 0, MOTORNAME_MTP_MISMATCH, 1);
      } */;
      BGVAR(u16_MotorName_Mismatch) = 0; // Reset Auto-Feedback Detection Counter only after Fault is set.
   }
   else if ( (mode == FLT_CLR) && ((BGVAR(s64_SysNotOk_2) & MOTORNAME_MTP_MISMATCH) != 0) )
   {
      BGVAR(s16_Auto_FdbkType_Cntr) = 0;
      BGVAR(s64_SysNotOk_2) &= ~MOTORNAME_MTP_MISMATCH;
      BGVAR(s16_DisableInputs) |= MTP_READ_DIS_MASK;
      BGVAR(u16_Init_From_MTP) = 1;
   }
}


//**********************************************************
// Function Name: SalReadSto
// Description:
//   called by read PParam P04-25 . Value returened:
//     Bit 0 represent STO:  0 if STO fault exists. ?
//
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int SalReadSto(long long *data, int drive)
{
   drive += 0;

   if (u16_STO_Flt_Indication_Actual)
       *data = (long long)0;
   else
       *data = (long long)1;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: GetEmcyManufcturerCode
// Description: set array with manufacturer code for CANopen EMCY packet for lexium 28 drive
//              structure is according to chapter 4.8.4 in Bai_Yang_DES04_CANopenImplementationGuide V02_02.pdf
//              manufacturer error code updateed to bytes 6,7 only base on email from alexander badura (11/8/2014)
//
//    u64_fault_mask: the bit for the fault which is needed
//    s16_fault_set: 0 for bits in s64_SysNotOk
//                   1 for bits in s64_SysNotOk_2
//                   2 for bits in u64_Sys_Warnings
//    manuFltCode:  pointer to array of at least 5 chars
//
// IMPORTANT only use in background since it uses const array (u16_Lex_Fault_Number_Array)
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void GetEmcyManufcturerCode(int drive, unsigned long long u64_fault_mask, int s16_fault_set, unsigned char* u8_manuFltCode)
{
   unsigned int u16_lxmFltNumber;
   int index = 0;

   // find lxm28 fault text
   u16_lxmFltNumber = GetLexiumFaultNumberByBit(drive, u64_fault_mask, s16_fault_set);

   // structure redesign due to bug 3399 (BYang00000142:CANOpen: EMCY-Object wrong framing)
   // this array populates bytes for the manufacturer code (bytes 6 to 7 in the EMCY frame)
   u8_manuFltCode[index++] = 0;  // slot ID - always zero, for zero for standard devices
   u8_manuFltCode[index++] = 0;
   u8_manuFltCode[index++] = 0;
   // manufacturer error code updateed to bytes 6,7 only base on email from alexander badura (11/8/2014)
   u8_manuFltCode[index++] = u16_lxmFltNumber & 0xff;       // flt code lower byte
   u8_manuFltCode[index++] = (u16_lxmFltNumber>>8) & 0xff;  // flt code higher byte
}


//**********************************************************
// Function Name: SalSetUserFltsMask
// Description:
//    This function is called in response to the FLTMASK message.
//    Enables faults mask by the user, as set in s64_User_Faults_Mask, s64_User_Faults_Mask2.
//    Password Protected.
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int SalSetUserFltsMask (int drive)
{
   drive += 0;
   s64_User_Faults_Mask = s64_Execution_Parameter[0];
   s64_User_Faults_Mask = ((s64_User_Faults_Mask << 32) | s64_Execution_Parameter[1]);
   s64_User_Faults_Mask2 = s64_Execution_Parameter[2];
   s64_User_Faults_Mask2 = ((s64_User_Faults_Mask2 << 32) | s64_Execution_Parameter[3]);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadUserFltsMask
// Description:
//    This function is called in response to the FLTMASK message.
//    Read user faults-mask as set in s64_User_Faults_Mask, s64_User_Faults_Mask2.
//    Password Protected.
//
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int SalReadUserFltsMask (int drive)
{
   drive += 0;
   PrintDecInAsciiHex((s64_User_Faults_Mask>>32), 8);
   PrintChar(' ');
   PrintDecInAsciiHex((s64_User_Faults_Mask & 0xFFFFFFFF), 8);
   PrintChar(' ');
   PrintDecInAsciiHex((s64_User_Faults_Mask2>>32), 8);
   PrintChar(' ');
   PrintDecInAsciiHex((s64_User_Faults_Mask2 & 0xFFFFFFFF), 8);
   PrintCrLf();
   return SAL_SUCCESS;
}


void DefaultConfigurationWarning(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   if (BGVAR(s16_Drive_Default_Config_Wrn_Flag) == 1)
   {// set Wn737
      BGVAR(u64_Sys_Warnings) |= DRIVE_WITH_DEFAULT_CONFIG;
   }
   else
   {// clear Wn737
      BGVAR(u64_Sys_Warnings) &= ~DRIVE_WITH_DEFAULT_CONFIG;
   }
}


void I2CTempSensReadWarning (int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(u16_PowerTempRead_Fail_Wrn_Flag) + BGVAR(u16_CtrlTempRead_Fail_Wrn_Flag)) > 10)
      BGVAR(u64_Sys_Warnings) |= TEMP_SENS_READ_FAIL_WRN_MASK;
   else
      BGVAR(u64_Sys_Warnings) &= ~TEMP_SENS_READ_FAIL_WRN_MASK;
}


//**********************************************************
// Function Name: HallsValidityCheck
// Description:
//   Called by 1ms_ISR to check the connectivity off each hall
//   by checking proper values. If a hall is disconnected, it will
//   have a stady 0/1 value, and thus we will get at some point a 0/7
//   value from 3 halls combination.  If some adjacent reads show bad values,
//   the flag for invalid values is raised and detected by HallsFault()
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(HallsValidityCheck, "ramfunc_5");
void HallsValidityCheck (int drive)
{
   int temp_var = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (FEEDBACK_TAMAGAWA)
   {
      if (VAR(AX0_u16_Tamagawa_Timer) == 0xFFFF)
         temp_var = VAR(AX0_s16_Tamagawa_Halls) & 0x7; // isolate halls reading
      else
         BGVAR(u16_Halls_Invalid_Value) = 0; // Zero Counter when Initialization not complete
   }
   else
      temp_var = VAR(AX0_s16_Halls) & 0x7 ; // isolate halls reading

   if ((0x7 == temp_var) || (0x0 == temp_var))
   {
      if (20 > BGVAR(u16_Halls_Invalid_Value))
         BGVAR(u16_Halls_Invalid_Value) += 7;
   }
   else if ((0 < BGVAR(u16_Halls_Invalid_Value)) && (20 > BGVAR(u16_Halls_Invalid_Value)))
      BGVAR(u16_Halls_Invalid_Value)--;
}


//**********************************************************
// Function Name: BiSSC_Fault
// Description:
//    This function handles the BiSS-C fault
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
void BiSSC_Fault(int drive, int mode, int fdbk_dest)
{
   int temp_var = 0;
   register unsigned int u16_status;

   // Prevent invalid fault and warning indications
   if ((!FEEDBACK_BISSC) || (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK))
   {
      BGVAR(u64_Sys_Warnings) &= ~BISSC_WRN_MASK;
      BGVAR(s64_SysNotOk_2) &= ~BISSC_FLT_MASK;
      return;
   }

   u16_status = BiSSC_GetStatus(drive, fdbk_dest);   // Latch the status word

   if (u16_status & BISSC_STATUS_WARN_MASK)
      BGVAR(u64_Sys_Warnings) |= BISSC_WRN_MASK;
   else
      BGVAR(u64_Sys_Warnings) &= ~BISSC_WRN_MASK;

   if (MaskedFault(drive, BISSC_FLT_MASK, 1)) return;

   if ((mode == FLT_CLR) && (BGVAR(s64_SysNotOk_2) & BISSC_FLT_MASK))
      BiSSC_Restart(drive);

   if ( (mode == DETECT) && (u16_status & BISSC_STATUS_FAULT_MASK) ) // On BiSS-C feedback test for "fault" bit indication
      temp_var = 1;

   FeedbackFault(drive, mode, BISSC_FLT_MASK, NULL, 0, 0, temp_var, 1, 1);
}

// Excessive electronic noise might cause the FPGA internal PLL to indicate that it is not locked, which means that the input clock is not stable.
// Latch this fault for 2 seconds.
// If a new indication appears during the 2 seconds of the existing warning, restart the 2 second latch period.
/*void ExcessiveElectNoiseWarning(int drive)
{
   unsigned int u16_FPGA_flt_indication = 0;

   drive += 0;     // defeat compiler remark

   //if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
   {

      u16_FPGA_flt_indication = *(unsigned int *)(FPGA_FAULT_MANAGEMENT2_REG_ADD);
      u16_FPGA_flt_indication &= FPGA_PLL_NOT_LOCKED;

      if (u16_FPGA_flt_indication)
      {
         // Clear the fault on FPGA
         *(unsigned int *)(FPGA_FAULT_MANAGEMENT2_REG_ADD) = 0;
         *(unsigned int *)(FPGA_FAULT_MANAGEMENT2_REG_ADD) &= u16_FPGA_flt_indication;
         *(unsigned int *)(FPGA_FAULT_MANAGEMENT2_REG_ADD) = 0;
         *(unsigned int *)(FPGA_FAULT_MANAGEMENT2_REG_ADD) &= u16_FPGA_flt_indication;

         BGVAR(u64_Sys_Warnings) |= EXCESSIVE_ELECT_NOISE_WRN_MASK;
         BGVAR(s32_Excessive_Elect_Noise_Warning_Timer) = Cntr_1mS;
      }
      else if (PassedTimeMS(2000L, BGVAR(s32_Excessive_Elect_Noise_Warning_Timer)))
      {
         BGVAR(u64_Sys_Warnings) &= ~EXCESSIVE_ELECT_NOISE_WRN_MASK;
      }
   }
}*/


void MenczposMismatchFault(int drive, int mode)
{
   // AXIS_OFF;

   if ( (MaskedFault(drive, MENCZPOS_MISMATCH_FLT_MASK, 1))                        ||
        // If Fault is not in effect...
        (BGVAR(u16_FdbkType) != INC_ENC_FDBK) || (VAR(AX0_s16_Motor_Enc_Type) > 2) ||
        // If not Incremental Quadrature Encoder with Index...
        ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ACTIVE) != 0)                  )
        // Motor-Setup may be manipulating MENCZPOS, avoid setting Fault because of interim values...
   {
      BGVAR(s64_SysNotOk_2) &= ~MENCZPOS_MISMATCH_FLT_MASK;
      VAR(AX0_u16_MENCZPOS_Mismatch) &= ~0x0100; // Reset MENCZPOS Mismatch Indication Bit
   }
   else if ( (mode == FLT_CLR) && ((BGVAR(s64_SysNotOk_2) & MENCZPOS_MISMATCH_FLT_MASK) != 0) )
   { // at Fault-Clear, reset Fault, clear RT Variable, and restart Encoder State-Machine
      BGVAR(s64_SysNotOk_2) &= ~MENCZPOS_MISMATCH_FLT_MASK;
      VAR(AX0_u16_MENCZPOS_Mismatch) &= ~0x0100; // Reset MENCZPOS Mismatch Indication Bit
      ResetEncState(drive); // Restart Encoder Setup
   }
   else if (mode == DETECT)
   {
      // Inhibit this fault during BURNIN
      if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command))
      {
         BGVAR(s64_SysNotOk_2) &= ~MENCZPOS_MISMATCH_FLT_MASK;
         VAR(AX0_u16_MENCZPOS_Mismatch) &= ~0x0100; // Reset MENCZPOS Mismatch Indication Bit
      }

      if ((VAR(AX0_u16_MENCZPOS_Mismatch) & 0x0100) != 0)
      {
         if (HandleFault(drive, MENCZPOS_MISMATCH_FLT_MASK, 1))
         {
//            p402_error_code = ERRCODE_CAN_MENCZPOS_MISMATCH;
 //           CANFaultControl(drive, 0, MENCZPOS_MISMATCH_FLT_MASK, 0);
         }
      }
   }
}

void FanCircuitWarning(int drive)
{
   drive += 0;     // defeat compiler remark

   if ( ((BGVAR(u16_Power_Hw_Features) & 0x0020) != 0)                                 &&
        ((*(unsigned int *)(FPGA_FAULT_MANAGEMENT_STATUS_REG_ADD) & PWR_FAN_STAT) != 0)  )
   {
      if (BGVAR(u16_Fan_Warning_Filter) >= 1000)
      {  // tests show fan warning for about 350 BG when switching from rotating fan to stopped fan
         BGVAR(u64_Sys_Warnings) |= FAN_CIRCUIT_WRN_MASK;
      }
      else
      {
         BGVAR(u16_Fan_Warning_Filter)++;
      }
   }
   else
   {
      BGVAR(u64_Sys_Warnings) &= ~FAN_CIRCUIT_WRN_MASK;
      BGVAR(u16_Fan_Warning_Filter) = 0;
   }
}

void FbSyncFault(int drive,int mode)
{
   unsigned int measured_cyc_time = 0;
   unsigned int actual_cyc_time_limit = (unsigned int)(u32_Actual_Cyc_Time/(unsigned int)10);
   drive += 0;
   
   if (MaskedFault(drive, INTERPOLATION_TIME_NOT_SYNC, 1)) return;
   SET_DPRAM_FPGA_MUX_REGISTER(0);
   
   
   if ((mode == DETECT) && 
      ((IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)) &&
      //detect only in EC mode op
      ((*(unsigned int*)p_u16_tx_nmt_state == EC_NMTSTATE_OP) ||
      //detect only in CAN mode op
      ((GL_ARRAY(co_Node).eState) == OPERATIONAL))&& 
      //and NO FB sync time mismatch ignore requested
      !(BGVAR(u8_Fb_Packet_Loss_Ignore) & FB_INTERPOLATION_SYNC_TIME_MISMATCH_IGNORE))
   {
      //set measured_cyc_time for CAN or ECAT
      //if EtherCAT
      if (s16_Fieldbus_Flags == 0x0005)
         measured_cyc_time = (unsigned int)((unsigned int)u32_Controller_Sync_Period_in_32Khz*(unsigned int)100/(unsigned int)32);
      //CAN
      else if (s16_Fieldbus_Flags == 0x0003)
         measured_cyc_time = (unsigned int)((unsigned int)u32_Controller_Sync_Period_in_32Khz/(unsigned int)16);
      //if measured time is either below or above 10% limit of CAN sync time trigger fault. 
      if ((measured_cyc_time > (u32_Actual_Cyc_Time+actual_cyc_time_limit)) || 
         (measured_cyc_time < (u32_Actual_Cyc_Time-actual_cyc_time_limit)))
      {
         HandleFault(drive, INTERPOLATION_TIME_NOT_SYNC, 1);
      }
   }//end if DETECT
   else if (mode == FLT_CLR)
   {
      BGVAR(s64_SysNotOk_2) &= ~INTERPOLATION_TIME_NOT_SYNC;
   } 
} 
  
