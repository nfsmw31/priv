//###########################################################################
//
// FILE: CommFdbk.c
//
// TITLE:   Contains communication position feedback routines
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 14 MAR 2011 | D.R. | Creation
//
//##################################################################

#include "DSP2834x_Device.h"

#include "CommFdbk.def"
#include "Design.def"
#include "Display.def"
#include "Err_Hndl.def"
#include "FltCntrl.def"
#include "FPGA.def"
#include "ModCntrl.def"
#include "MultiAxis.def"
#include "PhaseFind.def"
#include "Ser_Comm.def"
#include "Flash.def"

#include "CommFdbk.var"
#include "Drive.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Motor.var"
#include "MotorSetup.var"
#include "PhaseFind.var"
#include "Ser_Comm.var"
#include "Position.var"
#include "FlashHandle.var"

#include "Prototypes.pro"

#include "co_util.h"
#include "co_type.h"
#include <string.h>


//**********************************************************
// Function Name: CommPosFdbkHandler
// Description:
//    Communication Position Feedback Handler , run each background.  "enc_source" Parameter signifies Encoder
//    Source, '0' for Feedback Connector C4 or '1' for  Machine Interface Connector C3.
//    Parsing Variable Set is fixed, AX0_CRRNT_8_VAR_PAGE for Motor Feedback; reference to Feedback_Ptr only...
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void CommPosFdbkHandler(int drive, int enc_source, int fdbk_dest)
{
   // AXIS_OFF;
   FDBK_OFF;
   int ret_val = 0, u16_tmp_timer, u16_temp_status;
   static int u16_clear_faults_iter = 0;
   static unsigned long u32_srvsns_owner = 0;
   
   fdbk_offset+=0;
   if ( (LVAR(AX0_s32_Feedback_Ptr) != &NK_COMMUNICATION_FEEDBACK)     &&
        (LVAR(AX0_s32_Feedback_Ptr) != &TM_COMMUNICATION_FEEDBACK)     &&
        (LVAR(AX0_s32_Feedback_Ptr) != &PS_P_G_COMMUNICATION_FEEDBACK) &&
        (LVAR(AX0_s32_Feedback_Ptr) != &PS_S_COMMUNICATION_FEEDBACK)   &&
        (LVAR(AX0_s32_Feedback_Ptr) != &SRVSNS_COMMUNICATION_FEEDBACK) &&
        (LVAR(AX0_s32_Feedback_Ptr) != &FANUC_COMMUNICATION_FEEDBACK)  &&
        (LVAR(AX0_s32_Feedback_Ptr) != &YS_COMMUNICATION_FEEDBACK)     &&
        (LVAR(AX0_s32_Feedback_Ptr) != &SK_COMMUNICATION_FEEDBACK)       )
   {
//    SanyoDenkyCommandHack();
      BGVAR(s16_DisableInputs) &= ~COMM_FDBK_DIS_MASK;
      return;
   }

   switch (BGVAR(s16_Comm_Fdbk_Init_State))
   {
      case COMM_FDBK_IDLE_STATE:
      break;

      case COMM_FDBK_START_INIT:
         BGVAR(s16_DisableInputs) |= COMM_FDBK_DIS_MASK;          // Set disable bit

         // Disable RT communication, clear all bits except for Sync Symbol
         VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= 0x00007;

         *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) = 0;

         switch (BGVAR(u16_FdbkType))
         {
            case NK_COMM_FDBK: // Sanyo-Denky
               VAR(AX0_u16_CommEnc_InTurn_Less16_Bits) = GetBitNumber(BGVAR(u32_User_Motor_Enc_Res)) + 1 - 16;
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x20; // Nikon
               *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) = 1;
            break;

            case TAMAGAWA_COMM_MULTI_TURN_FDBK:
            case TAMAGAWA_COMM_SINGLE_TURN_FDBK:
            case TAMAGAWA_CID0_SINGLE_TURN:
            case PS_P_G_COMM_FDBK: // Panasonic
            case PS_S_COMM_FDBK: // Panasonic Absolute
            case SANKYO_COMM_FDBK:
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x21; // Tamagawa or Panasonic
               *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) = 1;   // Use registers in FPGA RAM
            break;

            case FANUC_COMM_FDBK:
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x22; // Fanuc
            break;

            case SERVOSENSE_SINGLE_TURN_COMM_FDBK: // Initialize the ServoSense driver
            case SERVOSENSE_MULTI_TURN_COMM_FDBK:
               SrvSns_Init(drive);
               if (BGVAR(u16_MTP_Mode))
               {
                  BGVAR(s16_DisableInputs) &= ~MTP_READ_DIS_MASK;
                  BGVAR(u16_Init_From_MTP) = 0;
                  BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
               }
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x24; // ServoSense
               *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) = 1;     // Use registers in FPGA RAM
            break;

            case YASKAWA_ABS_COMM_FDBK:
            case YASKAWA_INC_COMM_FDBK:
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x27; // Yaskawa 20 bit in-turn, 12 bit multi-turn serial encoder
            break;
         }

         if (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) != 3)    // long delay big input capacitor workaround for ServoSense
         {
            // ServoSense multi turn requires reset only when clearfaults issued.
            // It is done to prevent bad battery measurement due to short power reset.
            // It also ensures proper battery measurement when the drive's power was down for some long time.
            // The condition is for generic ServoSense feedback, because we might not know what ServoSense is actually connected.
            if (!FEEDBACK_SERVOSENSE)
            {
               RemoveFeedback5Volt();                    // Power down the 5V to the encoder
            }
            else if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PWR_RESET_REQ_MASK)
            {
               VAR(AX0_u16_SrvSns_FWStatus) &= ~SRVSNS_FW_STATUS_PWR_RESET_REQ_MASK;
               RemoveFeedback5Volt();                    // Power down the 5V to the encoder
            }
         }
         CommFdbkDefaultValues(drive, enc_source, fdbk_dest);

         BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
         *(unsigned int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + 0x0540 * enc_source) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Field_Length);
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_POWER_DOWN_STATE;
      break;

      case COMM_FDBK_POWER_DOWN_STATE: // Wait 0.2 sec for the encoder to power down
         if ( ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)       &&
                (PassedTimeMS(1L, BGVAR(s32_Comm_Fdbk_Timer)))  )  ||
              ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) != 3)         &&
                (PassedTimeMS(200L, BGVAR(s32_Comm_Fdbk_Timer)))  )  )
         { // or if Reset-Multi-Turn wait minimal time only
            SetFeedback5Volt(); // Power up the 5V to the encoder

            if (FEEDBACK_SERVOSENSE)
            {
               // Acquire ServoSense device
               ret_val = SrvSns_Acquire(&u32_srvsns_owner, drive);
               if (ret_val != SAL_SUCCESS)
               {
                  if(PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, BGVAR(s32_Comm_Fdbk_Timer)))
                  {
                     SrvSns_SetFWStatus(SRVSNS_FW_STATUS_ACQUIRE_FAILED_MASK, 1, drive);
                     // Set feedback state to error
                     BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_ERROR_STATE;
                  }
                  return;
               }
            }

            BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_POWER_UP_STATE;
            BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
         }
      break;

      case COMM_FDBK_POWER_UP_STATE:
         if ( ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)       &&
                (PassedTimeMS(1L, BGVAR(s32_Comm_Fdbk_Timer)))  )    ||
              ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) != 3)          &&
                (PassedTimeMS(1750L, BGVAR(s32_Comm_Fdbk_Timer)))  )   )
         // Wait 1.75 second for Encoder to stabilize after Power-Up,
         { // or if Reset-Multi-Turn wait minimal time only.
            if (FEEDBACK_SERVOSENSE)
            {
                // Set response to multi-turn
                SrvSns_SetCommMode((SRVSNS_COMM_MODE_OPERATIONAL | SRVSNS_COMM_SUBMODE_MULTITURN), drive);

                // Reset init state
                SRVSNS_RTMBX_CLEAR_INIT_STATE(VAR(AX0_u16_SrvSns_MBX_Header));

                // Enable the request in RT
                VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= (COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK);
                BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
            }
            BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS;
            u16_clear_faults_iter = 0;
         }
      break;

      case COMM_FDBK_CLEAR_FAULTS:
         switch (BGVAR(u16_FdbkType))
         {
            case NK_COMM_FDBK: // Sanyo-Denky
               if (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)
               {
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = NK_RST_TURN_CMD1B;
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 1 + 0x0080 * enc_source) = NK_RST_TURN_CMD2B;
               }
               else
               {
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = NK_CLR_FLT_CMD1B;
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 1 + 0x0080 * enc_source) = NK_CLR_FLT_CMD2B;
               }

               u16_clear_faults_iter++;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;

               if (u16_clear_faults_iter >= 10) // Send the Tamagawa clear faults 10 times
                  BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;

            case TAMAGAWA_COMM_MULTI_TURN_FDBK:
            case TAMAGAWA_COMM_SINGLE_TURN_FDBK:
            case SANKYO_COMM_FDBK:
               if (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(TM_RST_TURN_CMD2); // Dual-Feedback FPGA
               else
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(TM_CLR_FLT_CMD2); // Dual-Feedback FPGA

               u16_clear_faults_iter++;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;

               if (BGVAR(u16_FdbkType) == SANKYO_COMM_FDBK)
               {
                  if (u16_clear_faults_iter >= 20)
                     BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
               }
               else
               {
                  if (u16_clear_faults_iter >= 12) // Send the Tamagawa clear faults 10 times
                     BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
               }
            break;

            case PS_S_COMM_FDBK:
               if (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(PS_S_RST_TURN_CMD2); // Standard FPGA
               else
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(PS_S_CLR_FLT_CMD2); // Standard FPGA

               u16_clear_faults_iter++;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;

               if (u16_clear_faults_iter >= 10) // Send the Tamagawa clear faults 10 times
                  BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;

            case PS_P_G_COMM_FDBK: // Panasonic Incremental
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(PS_P_G_RST_FLT_CMD3); // Standard FPGA
               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG3_ADD + 0x0540 * enc_source) = (unsigned int)((PS_P_G_RST_FLT_CMD1 & 0x07E0) >> 5);
               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG2_ADD + 0x0540 * enc_source) = (unsigned int)((PS_P_G_RST_FLT_CMD1 & 0x001F) << 11);

               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;
               BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;

            case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
            case SERVOSENSE_MULTI_TURN_COMM_FDBK:
               /* If ServoSense didn't respond to operational requests, suspect boot mode */
               if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_TIME_OUT_ERROR_MASK) != 0)
               {
                  /* Release RT communication */
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~(COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK);

                  SRVSNS_FPGA_CONFIG_BOOT_MODE;                  // Try to communicate with boot mode

                  CommFdbkFault(drive, FLT_CLR);                  // Clear fault

                  BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
               }
               else // Otherwise proceed to clear faults
               {
                  if ((ret_val = SrvSns_OnInitProcedure(u32_srvsns_owner, drive, 0)) != SAL_NOT_FINISHED)// on finished
                  {
                     if (ret_val != SAL_SUCCESS)// on error
                     {
                        // Retry SRVSNS_INIT_PROC_RETRIES times
                        if(BGVAR(u16_SrvSns_InitProcRetriesCntr) < SRVSNS_INIT_PROC_RETRIES)
                        {
                           // Clear FW status flags while retrying
                           VAR(AX0_u16_SrvSns_FWStatus) &= ~(SRVSNS_FW_STATUS_PRESENT_MASK | SRVSNS_FW_STATUS_CLRFLTS_FAILED_MASK);
                           BGVAR(u16_SrvSns_InitProcRetriesCntr)++;
                           break; // leave the switch case to try again
                        }
                     }

                     // Pass the error code to handler
                     SrvSns_OnInitProcResultHandler(ret_val, drive);

                     // Install MT faults indications
                     if(BGVAR(u16_SrvSns_ProductType) == SRVSNS_MULTI_TURN)
                        BGVAR(u32_SrvSns_FaultsIndications) |= SRVSNS_MT_FAULTS;

                     // Transition to the next state
                     BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;

                     // Take timer value for the next state
                     BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
                  }
                  else // ret_val == SAL_NOT_FINISHED, not finished, check timeout
                  {
                     if(PassedTimeMS(SRVSNS_INIT_PROC_TIMEOUT_mSEC, BGVAR(s32_Comm_Fdbk_Timer)))
                     {
                        // Release the ServoSense device
                        SrvSns_Release(&u32_srvsns_owner, drive);

                        // Pass the time out code to handler
                        SrvSns_OnInitProcResultHandler(SRVSNS_BUSY_TIMEOUT, drive);

                        // Explicitly initiate a fault to indicate the issue to the user
                        BGVAR(s64_SysNotOk_2) |= SRVSNS_ENCODER_FLT_MASK;

                        // Transition to error state
                        BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_ERROR_STATE;
                     }
                  }
                 if (BGVAR(u16_SrvSns_ProductType) == SRVSNS_MULTI_TURN)
                    BGVAR(u32_SrvSns_FaultsIndications) |= SRVSNS_MT_FAULTS;
               }
            break;

            case YASKAWA_ABS_COMM_FDBK:
            case YASKAWA_INC_COMM_FDBK:
               BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;

            default: // Fanuc, EnDat2.2, etc
               BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;
         }
         if (!FEEDBACK_SERVOSENSE)
            BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
      break;

      case COMM_FDBK_CLEAR_FAULTS_DONE:
         if (FEEDBACK_SERVOSENSE)
         {
            if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PRESENT_MASK)// the encoder comm is valid, check the fault status.
            {
               if ( PassedTimeMS(10L, BGVAR(s32_Comm_Fdbk_Timer)) ) // wait till the faults are cleared at ServoSense
               {
                  // check faults status
                  if ( SrvSns_GetStatus(drive) & SRVSNS_STATUS_FAULT_MASK )
                  {
                     // Let the user monitor the fault by passsing zero to feedback disable bit
                     SrvSns_SetFWStatus(SRVSNS_FW_STATUS_FLTS_NOT_CLEARED_MASK, 0, drive);
                  }
                  // move to position init state and disable the request
                  BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_INIT_POSITION;
                  // release RT communication
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~(COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK);
               }
            }
            else
            {  // the encoder might be in boot mode, check it's response
               ret_val = SrvSns_ValidateBootMode(*(unsigned int *)FPGA_RX_DATA_READY_REG_ADD, drive);
               if (ret_val == SAL_NOT_FINISHED)
               {
                  if (!PassedTimeMS(SRVSNS_BOOT_RESPONSE_TIME_mSEC, BGVAR(s32_Comm_Fdbk_Timer)))
                     break;

                  // the encoder is not connected or neither op FW nor boot are present on the encoder
                  SrvSns_SetFWStatus(SRVSNS_FW_STATUS_NOT_CONNECTED_MASK, 1, drive);

                  // Initiate timeout fault
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_READY_MASK;
               }

               SrvSns_Release(&u32_srvsns_owner, drive);  // Release the ServoSense device

               // Set feedback state to error to prevent further initialization states to execute
               BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_ERROR_STATE;
            }
         }
         else
         {
            // Allow additional Time for Encoder with +5VDC Power to Stabilize if getting to
            // this State of Comm-Feedback Handler (Tamagawa Power-Up).
            if ( ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) != 3)          &&
                   (PassedTimeMS(1600L, BGVAR(s32_Comm_Fdbk_Timer)))  ) ||
                 ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)          &&
                   (PassedTimeMS(5L, BGVAR(s32_Comm_Fdbk_Timer)))     )   )
               BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_INIT_POSITION;
         }
      break;

      case COMM_FDBK_INIT_POSITION:
         switch (BGVAR(u16_FdbkType))
         {
            case NK_COMM_FDBK: // Sanyo-Denky
               VAR(AX0_u16_Abs_Enc_Command_1Frame) = NK_READ_POS_CMD1B; // Position request
               VAR(AX0_u16_Abs_Enc_Command_2Frame) = NK_READ_POS_CMD2B; // Position request
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = NK_READ_POS_CMD1B;
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 1 + 0x0080 * enc_source) = NK_READ_POS_CMD2B;
            break;

            case TAMAGAWA_COMM_MULTI_TURN_FDBK:
            case TAMAGAWA_COMM_SINGLE_TURN_FDBK: // Position + Status request
               VAR(AX0_u16_Comm_Fdbk_Flags_B_1) &= ~0x00001; // Process Full Pos. Data
               VAR(AX0_u16_Abs_Enc_Command_1Frame) = (unsigned int)(TM_READ_POS_CMD2);
            break;

            case TAMAGAWA_CID0_SINGLE_TURN:
               VAR(AX0_u16_Comm_Fdbk_Flags_B_1) |= 0x00001; // Process ABSA0 - ABSA2 only
               VAR(AX0_u16_Abs_Enc_Command_1Frame) = (unsigned int)(TM_CID0_READ_POS_CMD);
            break;

            case SANKYO_COMM_FDBK:
               VAR(AX0_u16_Abs_Enc_Command_1Frame) = TM_READ_POS_CMD2;
               VAR(AX0_u16_Abs_Enc_Command_2Frame) = (unsigned int)((SK_READ_POS_CMD1 & 0x001F) << 11); // Position request
               VAR(AX0_u16_Abs_Enc_Command_3Frame) = (unsigned int)((SK_READ_POS_CMD1 & 0x07E0) >> 5); // Position request
            break;

            case PS_P_G_COMM_FDBK: // Position + Status request
               VAR(AX0_u16_Abs_Enc_Command_1Frame) = (unsigned int)(PS_P_G_READ_ABSS_CMD3);
               VAR(AX0_u16_Abs_Enc_Command_2Frame) = (unsigned int)((PS_P_G_READ_ABSS_CMD1 & 0x001F) << 11); // Position request
               VAR(AX0_u16_Abs_Enc_Command_3Frame) = (unsigned int)((PS_P_G_READ_ABSS_CMD1 & 0x07E0) >> 5); // Position request
            break;

            case PS_S_COMM_FDBK: // Full Position request
               VAR(AX0_u16_Abs_Enc_Command_1Frame) = (unsigned int)(PS_S_READ_POS_CMD3);
            break;

            case FANUC_COMM_FDBK:
               VAR(AX0_u16_Abs_Enc_Command_1Frame) = 0xFFFF;
               VAR(AX0_u16_Abs_Enc_Command_2Frame) = 0xF7FF; // Position request
               VAR(AX0_u16_Abs_Enc_Command_3Frame) = 0xFFDF; // Position request

               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG3_ADD + 0x0540 * enc_source) = 0xFFDF;
               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG2_ADD + 0x0540 * enc_source) = 0xF7FF;
               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG1_ADD + 0x0540 * enc_source) = 0xFFFF;
            break;

            case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
            case SERVOSENSE_MULTI_TURN_COMM_FDBK:
               // set velocity mode
               SrvSns_SetCommMode((SRVSNS_COMM_MODE_OPERATIONAL | SRVSNS_COMM_SUBMODE_VELOCITY), drive);
               // Release the ServoSense device
               SrvSns_Release(&u32_srvsns_owner, drive);
            break;

            case YASKAWA_ABS_COMM_FDBK:
            case YASKAWA_INC_COMM_FDBK:
               // The FPGA  module handles the position requests internally
            break;
         }

         // Differ between Multi-Turn and Single-Turn
         if ( (BGVAR(u16_FdbkType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK) ||
              (BGVAR(u16_FdbkType) == TAMAGAWA_CID0_SINGLE_TURN)        )
            VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_INIT_ABS_POS_MASK |
                                       COMM_FDBK_INIT_ABS_POS_IN_REV_MASK | COMM_FDBK_READY_MASK;
         else if ( (BGVAR(u16_FdbkType) == PS_P_G_COMM_FDBK)     || // Panasonic Incremental Encoder
                   (BGVAR(u16_FdbkType) == YASKAWA_INC_COMM_FDBK)  ) // and Yaskawa Incremental Encoder require
            VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_READY_MASK; // no Initialization
         else if (FEEDBACK_SERVOSENSE)//ServoSense
             VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_READY_MASK; // Multi-Turn already initialized
         else
         // Tamagawa Multi-Turn, SK, Nikon, Yaskawa, Fanuc, Yaskawa Absolute
            VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK;

         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_INIT_POSITION_DONE;
      break;

      case COMM_FDBK_INIT_POSITION_DONE:
         if (BGVAR(u16_FdbkType) == FANUC_COMM_FDBK)
         {
            do {
               u16_tmp_timer = Cntr_3125;
               u16_clear_faults_iter = VAR(AX0_u16_Abs_Enc_Data_0Frame);
               u16_temp_status = VAR(AX0_u16_Comm_Fdbk_Flags_A_1);
            } while (u16_tmp_timer != Cntr_3125);
            if ((u16_temp_status & 0xFFF8) == (COMM_FDBK_READY_MASK | ABS_POSITION_REQ_SENT_MASK))
            // Get Data_Frame3 during "normal" Communication Cycle
            {
               if ((u16_clear_faults_iter & 0x0100) != 0)
               // If no Absolute Position (Index Mark not yet found)
               {
                  if ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_REQUESTED_MASK) == 0)
                     // Initialize Commutation through Phase Find
                     BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_REQUESTED_MASK;
                  else // Once Phase-Find is requested, continue State-Machine
                     u16_clear_faults_iter = 0;
               }
               else // if Absolute Position is valid, avoid Phase-Find and continue
               {
                  BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
                  u16_clear_faults_iter = 0;
               }
            }
         }
         else
            u16_clear_faults_iter = 0;

         if ( (u16_clear_faults_iter == 0)                                                &&
              ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_INIT_ABS_POS_MASK) == 0)  )
         {
            if (LVAR(AX0_s32_Feedback_Ptr) == &PS_S_COMMUNICATION_FEEDBACK)
            {  // After Initialization, switch to querying In-Turn Position with Alarms
               // Field; Multi-Turn Position is updated on Roll-Over.
               // Disable Enc. Comm. before changing Command to avoid indeterminancy,
               // re-enable Enc. Comm. after new Command is in place.
               VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~COMM_FDBK_READY_MASK;
               VAR(AX0_u16_Abs_Enc_Command_1Frame) = (unsigned int)(PS_S_READ_POS_CMD4);
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(PS_S_READ_POS_CMD4); // Dual-Feedback FPGA
               VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_READY_MASK;
            }

            if ( (BGVAR(u16_FdbkType) == PS_P_G_COMM_FDBK)     || // Panasonic Incremental Encoders
                 (BGVAR(u16_FdbkType) == YASKAWA_INC_COMM_FDBK)  ) // and Yaskawa Incremental Encoder
            { //  require Hall-Effect readings.
               VAR(AX0_s16_Power_Up_Timer) = 0x7FFF;
               VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_INC_ENC & 0xffff);
               BGVAR(u16_Halls_Invalid_Value) = 0; // Reset Halls Validity Check Counter
               // to avoid Halls-Invalid indication before Halls are initialized
            }

            BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_ENABLE_STATE;
            BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;

            if ((FEEDBACK_SERVOSENSE) && BGVAR(u16_MTP_Mode))
            {
               if(BGVAR(u16_MTP_Load_Done) == 0) // trigger the MTP load only if the MTP loader has reset
               {
                  BGVAR(s16_DisableInputs) |= MTP_READ_DIS_MASK;
                  BGVAR(u16_Init_From_MTP) = 1; // Initiate MTP init if needed
               }
            }
         }
      break;

      case COMM_FDBK_ENABLE_STATE:
      // Added 100mSec. Delay if Encoder is Battery-Backed, to allow Battery-Fault to be
      // detected before Enable after Power-Cycle takes effect.
         if ( ( ((BGVAR(s64_Faults_Mask) & ABS_ENC_BATT_LOW_MASK) != 0) &&
                (PassedTimeMS(100L, BGVAR(s32_Comm_Fdbk_Timer)))          ) ||
              ( ((BGVAR(s64_Faults_Mask) & ABS_ENC_BATT_LOW_MASK) == 0) &&
                (PassedTimeMS(1L, BGVAR(s32_Comm_Fdbk_Timer)))            )   )
         {
            // Update the absolute multi turn feedback indication variable
            switch(BGVAR(u16_FdbkType))
            {
               case NK_COMM_FDBK:
               case YASKAWA_ABS_COMM_FDBK:
                  VAR(AX0_u16_Abs_Fdbk_Device) |= 0x0001; // Manipulate Bit 0 only as indicator for Motor Feedback
               break;

               case TAMAGAWA_COMM_MULTI_TURN_FDBK:
               case PS_S_COMM_FDBK:
               case SANKYO_COMM_FDBK:
                  VAR(AX0_u16_Abs_Fdbk_Device) |= ((VAR(AX0_u16_Abs_Enc_Data_3Frame) & 0x04000) == 0);
                  // Manipulate Bit 0 only as indicator for Motor Feedback
               break;

               case FANUC_COMM_FDBK:
                  VAR(AX0_u16_Abs_Fdbk_Device) |= ((VAR(AX0_u16_Abs_Enc_Data_0Frame) & 0x0020) == 0);
                  // Manipulate Bit 0 only as indicator for Motor Feedback
               break;

               case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
                  VAR(AX0_u16_Abs_Fdbk_Device) |= (unsigned int)(BGVAR(u16_SrvSns_ProductType) == SRVSNS_MULTI_TURN);
                  BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = 1UL;
               break;

               case SERVOSENSE_MULTI_TURN_COMM_FDBK:
                  if(BGVAR(u16_SrvSns_ProductType) == SRVSNS_SINGLE_TURN)
                  {  // Raise warning Feedbacktype Missmatch
                     BGVAR(u64_Sys_Warnings) |= FDBK_TYPE_MISMATCH_WRN_MASK;
                  }
                  else
                  {
                     if(BGVAR(u16_MTP_Mode) && BGVAR(u16_MTP_Load_Done))
                     {
                        if(BGVAR(u16_MTP_Mode) == 1)
                           BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = BGVAR(u32_MTP_Multi_Turn_Number);
                        else if (BGVAR(u16_MTP_Mode) == 3)
                           BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = (1UL << BGVAR(u32_MTP_Multi_Turn_Number));
                        else
                           BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = (unsigned long)SRVSNS_MT_ENCDR_MAX_NUM_TURNS;
                     }
                     else
                     {
                        BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = (unsigned long)SRVSNS_MT_ENCDR_MAX_NUM_TURNS;
                     }

                     VAR(AX0_u16_Abs_Fdbk_Device) |= 0x0001;
                  }
               break;
            }

            // This updates the operational homing offset with the user non-volatile variable
            // according to the feedback multi turn absolute.
            if (BGVAR(u16_Ignore_Abs_Fdbk_Batt_Faults) == IGNOREBATTFLT_MODE_INACTIVE)
                 // If the MT igorance is active, then ignore this call. Fix to Bugzilla 5913.
               UpdateRefOffsetValue(drive);

            // The following call updates the state of the MT Batt faults ignorance for the configured feedback type
            if (SalIgnoreAbsFdbkBattFlt(BGVAR(u16_Ignore_Abs_Fdbk_Batt_Faults), drive) == SAL_NOT_FINISHED)
               return;

            BGVAR(s16_DisableInputs) &= ~COMM_FDBK_DIS_MASK;     //clear disable bit
            BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_IDLE_STATE;
         }
         BGVAR(u16_Comm_Fdbk_Init_Ran) = 1;
      break;
   }
}


//**********************************************************
// Function Name: CommPosSecFdbkHandler
// Description:
//    Communication Position Second Feedback Handler , run each background.  "enc_source" Parameter signifies
//    Encoder Source, '0' for Feedback Connector C4 or '1' for  Machine Interface Connector C3.
//    Parsing Variable Set is fixed, AX0_CRRNT_9_VAR_PAGE for Motor Feedback; reference to Feedback_Ptr_2 only...
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
void CommPosSecFdbkHandler(int drive, int enc_source, int fdbk_dest)
{
   // AXIS_OFF;
   FDBK_OFF;
   int ret_val = 0, u16_tmp_timer, u16_temp_status;
   static int u16_clear_faults_iter = 0;
   static unsigned long u32_srvsns_owner = 0;

   fdbk_offset += 0;
   if ( (LVAR(AX0_s32_Feedback_Ptr_2) != &NK_COMMUNICATION_FEEDBACK)     &&
        (LVAR(AX0_s32_Feedback_Ptr_2) != &TM_COMMUNICATION_FEEDBACK)     &&
        (LVAR(AX0_s32_Feedback_Ptr_2) != &PS_P_G_COMMUNICATION_FEEDBACK) &&
        (LVAR(AX0_s32_Feedback_Ptr_2) != &PS_S_COMMUNICATION_FEEDBACK)   &&
        (LVAR(AX0_s32_Feedback_Ptr_2) != &SRVSNS_COMMUNICATION_FEEDBACK) &&
        (LVAR(AX0_s32_Feedback_Ptr_2) != &FANUC_COMMUNICATION_FEEDBACK)  &&
        (LVAR(AX0_s32_Feedback_Ptr_2) != &YS_COMMUNICATION_FEEDBACK)     &&
        (LVAR(AX0_s32_Feedback_Ptr_2) != &SK_COMMUNICATION_FEEDBACK)       )
   {
//    SanyoDenkyCommandHack();
      BGVAR(s16_DisableInputs) &= ~COMM_SEC_FDBK_DIS_MASK;
      return;
   }

   switch (BGVAR(s16_Comm_Sec_Fdbk_Init_State))
   {
      case COMM_FDBK_IDLE_STATE:
      break;

      case COMM_FDBK_START_INIT:
         BGVAR(s16_DisableInputs) |= COMM_SEC_FDBK_DIS_MASK;    // Set disable bit

         // Disable RT communication, clear all bits except for Sync Symbol
         VAR(AX0_u16_Comm_Fdbk_Flags_A_2) &= 0x00007;

         *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) = 0;

         switch (BGVAR(u16_SFBType))
         {
            case NK_COMM_FDBK: // Sanyo-Denky
               VAR(AX0_u16_CommEnc_InTurn_Less16_Bits_2) = GetBitNumber(BGVAR(u32_User_Motor_Enc_Res)) + 1 - 16;
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x20; // Nikon
               *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) = 1;
            break;

            case TAMAGAWA_COMM_MULTI_TURN_FDBK:
            case TAMAGAWA_COMM_SINGLE_TURN_FDBK:
            case TAMAGAWA_CID0_SINGLE_TURN:
            case PS_P_G_COMM_FDBK: // Panasonic
            case PS_S_COMM_FDBK: // Panasonic Absolute
            case SANKYO_COMM_FDBK:
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x21; // Tamagawa or Panasonic
               *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) = 1;   // Use registers in FPGA RAM
            break;

            case FANUC_COMM_FDBK:
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x22; // Fanuc
            break;

            case SERVOSENSE_SINGLE_TURN_COMM_FDBK: // Initialize the ServoSense driver
            case SERVOSENSE_MULTI_TURN_COMM_FDBK: // Initialize the ServoSense driver
               SrvSns_Init(drive);
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x24; // ServoSense
               *(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) = 1;     // Use registers in FPGA RAM
            break;

            case YASKAWA_ABS_COMM_FDBK:
            case YASKAWA_INC_COMM_FDBK:
               *(unsigned int *)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 0x27; // Yaskawa 20 bit in-turn, 12 bit multi-turn serial encoder
            break;
         }

         if (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) != 3)    // long delay big input capacitor workaround for
         {
            // ServoSense multi turn requires reset only when clearfaults issued.
            // It is done to prevent bad battery measurement due to short power reset.
            // It also ensures proper battery measurement when the drive's power was down for some long time.
            // The condition is for generic ServoSense feedback, because we might not know what ServoSense is actually connected.
            if (!SFB_SERVOSENSE)
            {
               RemoveFeedbackSecondary5Volt();                    // Power down the 5V to the encoder
            }
            else if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PWR_RESET_REQ_MASK)
            {
               VAR(AX0_u16_SrvSns_FWStatus) &= ~SRVSNS_FW_STATUS_PWR_RESET_REQ_MASK;
               RemoveFeedbackSecondary5Volt();                    // Power down the 5V to the encoder
            }
         }
         CommFdbkDefaultValues(drive, enc_source, fdbk_dest);

         BGVAR(s32_Sec_Comm_Fdbk_Timer) = Cntr_1mS;
         *(unsigned int *)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Fdbk_Clk_Interval);
         *(unsigned int *)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Xmt_Length);
         VAR(AX0_u16_Com_Enc_Xmt_Length_2) = BGVAR(u16_Xmt_Length);
         *(unsigned int *)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = BGVAR(u16_Rcv_Length);
         VAR(AX0_u16_Com_Enc_Xcv_Length_2) = BGVAR(u16_Rcv_Length);
         *(unsigned int *)(FPGA_TIME_WAIT_BEFORE_CAPTURE_RX_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Wait_Time_Clks);
         *(unsigned int *)(FPGA_SYNC_SYMBOL_REG_ADD + 0x0540 * enc_source) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & 0x0007);
         *(unsigned int *)(FPGA_FIELD_SIZE_REG_ADD + 0x0540 * enc_source) = BGVAR(u16_Field_Length);
         BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_POWER_DOWN_STATE;
      break;

      case COMM_FDBK_POWER_DOWN_STATE: // Wait 0.2 sec for the encoder to power down
         if ( ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)       &&
                (PassedTimeMS(1L, BGVAR(s32_Sec_Comm_Fdbk_Timer)))  )  ||
              ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) != 3)         &&
                (PassedTimeMS(200L, BGVAR(s32_Sec_Comm_Fdbk_Timer)))  )  )
         { // or if Reset-Multi-Turn wait minimal time only
            SetFeedbackSecondary5Volt(); // Power up the 5V to the encoder

            if (SFB_SERVOSENSE)
            {
               // Acquire ServoSense device
               ret_val = SrvSns_Acquire(&u32_srvsns_owner, drive);
               if (ret_val != SAL_SUCCESS)
               {
                  if(PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, BGVAR(s32_Sec_Comm_Fdbk_Timer)))
                  {
                     SrvSns_SetFWStatus(SRVSNS_FW_STATUS_ACQUIRE_FAILED_MASK, 1, drive);
                     // Set feedback state to error
                     BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_ERROR_STATE;
                  }
                  return;
               }
            }

            BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_POWER_UP_STATE;
            BGVAR(s32_Sec_Comm_Fdbk_Timer) = Cntr_1mS;
         }
      break;

      case COMM_FDBK_POWER_UP_STATE:
         if ( ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)           &&
                (PassedTimeMS(1L, BGVAR(s32_Sec_Comm_Fdbk_Timer)))  )    ||
              ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) != 3)              &&
                (PassedTimeMS(1750L, BGVAR(s32_Sec_Comm_Fdbk_Timer)))  )   )
         // Wait 1.75 second for Encoder to stabilize after Power-Up,
         { // or if Reset-Multi-Turn wait minimal time only.
            if (SFB_SERVOSENSE)
            {
                // Set response to multi-turn
                SrvSns_SetCommMode((SRVSNS_COMM_MODE_OPERATIONAL | SRVSNS_COMM_SUBMODE_MULTITURN), drive);

                // Reset init state
                SRVSNS_RTMBX_CLEAR_INIT_STATE(VAR(AX0_u16_SrvSns_MBX_Header));

                // Enable the request in RT
                VAR(AX0_u16_Comm_Fdbk_Flags_A_2) |= (COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK);
                BGVAR(s32_Sec_Comm_Fdbk_Timer) = Cntr_1mS;
            }
            BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS;
            u16_clear_faults_iter = 0;
         }
      break;

      case COMM_FDBK_CLEAR_FAULTS:
         switch (BGVAR(u16_SFBType))
         {
            case NK_COMM_FDBK: // Sanyo-Denky
               if (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)
               {
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = NK_RST_TURN_CMD1B;
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 1 + 0x0080 * enc_source) = NK_RST_TURN_CMD2B;
               }
               else
               {
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = NK_CLR_FLT_CMD1B;
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 1 + 0x0080 * enc_source) = NK_CLR_FLT_CMD2B;
               }

               u16_clear_faults_iter++;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;

               if (u16_clear_faults_iter >= 10) // Send the Tamagawa clear faults 10 times
                  BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;

            case TAMAGAWA_COMM_MULTI_TURN_FDBK:
            case TAMAGAWA_COMM_SINGLE_TURN_FDBK:
            case SANKYO_COMM_FDBK:
               if (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(TM_RST_TURN_CMD2); // Standard FPGA
               else
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(TM_CLR_FLT_CMD2); // Standard FPGA

               u16_clear_faults_iter++;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;

               if (BGVAR(u16_SFBType) == SANKYO_COMM_FDBK)
               {
                  if (u16_clear_faults_iter >= 20)
                     BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
               }
               else
               {
                  if (u16_clear_faults_iter >= 12) // Send the Tamagawa clear faults 10 times
                     BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
               }
            break;

            case PS_S_COMM_FDBK:
               if (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(PS_S_RST_TURN_CMD2); // Standard FPGA
               else
                  *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(PS_S_CLR_FLT_CMD2); // Standard FPGA

               u16_clear_faults_iter++;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;

               if (u16_clear_faults_iter >= 10) // Send the Tamagawa clear faults 10 times
                  BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;

            case PS_P_G_COMM_FDBK: // Panasonic Incremental
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(PS_P_G_RST_FLT_CMD3); // Standard FPGA
               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG3_ADD + 0x0540 * enc_source) = (unsigned int)((PS_P_G_RST_FLT_CMD1 & 0x07E0) >> 5);
               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG2_ADD + 0x0540 * enc_source) = (unsigned int)((PS_P_G_RST_FLT_CMD1 & 0x001F) << 11);

               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0;
               *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;
               BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;

            case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
            case SERVOSENSE_MULTI_TURN_COMM_FDBK:
               /* If ServoSense didn't respond to operational requests, suspect boot mode */
               if (VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & COMM_FDBK_TIME_OUT_ERROR_MASK)
               {
                  /* Release RT communication */
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_2) &= ~(COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK);

                  SRVSNS_FPGA_CONFIG_BOOT_MODE;      // Try to communicate with boot mode

                  CommFdbkFault(drive, FLT_CLR);                  // Clear fault

                  BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
               }
               else // Otherwise proceed to clear faults
               {
                  if ((ret_val = SrvSns_OnInitProcedure(u32_srvsns_owner, drive, 1)) != SAL_NOT_FINISHED)// on finished
                  {
                     if (ret_val != SAL_SUCCESS)// on error
                     {
                        // Retry SRVSNS_INIT_PROC_RETRIES times
                        if(BGVAR(u16_SrvSns_InitProcRetriesCntr) < SRVSNS_INIT_PROC_RETRIES)
                        {
                           // Clear FW status flags while retrying
                           VAR(AX0_u16_SrvSns_FWStatus) &= ~(SRVSNS_FW_STATUS_PRESENT_MASK | SRVSNS_FW_STATUS_CLRFLTS_FAILED_MASK);
                           BGVAR(u16_SrvSns_InitProcRetriesCntr)++;
                           break; // leave the switch case to try again
                        }
                     }

                     // Pass the error code to handler
                     SrvSns_OnInitProcResultHandler(ret_val, drive);

                     // Install MT faults indications
                     if(BGVAR(u16_SrvSns_ProductType) == SRVSNS_MULTI_TURN)
                        BGVAR(u32_SrvSns_FaultsIndications) |= SRVSNS_MT_FAULTS;

                     // Transition to the next state
                     BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;

                     // Take timer value for the next state
                     BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
                  }
                  else // ret_val == SAL_NOT_FINISHED, not finished, check timeout
                  {
                     if (PassedTimeMS(SRVSNS_INIT_PROC_TIMEOUT_mSEC, BGVAR(s32_Comm_Fdbk_Timer)))
                     {
                        // Release the ServoSense device
                        SrvSns_Release(&u32_srvsns_owner, drive);

                        // Pass the time out code to handler
                        SrvSns_OnInitProcResultHandler(SRVSNS_BUSY_TIMEOUT, drive);

                        // Explicitly initiate a fault to indicate the issue to the user
                        BGVAR(s64_SysNotOk_2) |= SRVSNS_ENCODER_FLT_MASK;

                        BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_ERROR_STATE;// Set feedback state to error
                     }
                  }
                  if(BGVAR(u16_SrvSns_ProductType) == SRVSNS_MULTI_TURN)
                     BGVAR(u32_SrvSns_FaultsIndications) |= SRVSNS_MT_FAULTS;
               }
            break;

            case YASKAWA_ABS_COMM_FDBK:
            case YASKAWA_INC_COMM_FDBK:
               BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;

            default: // Fanuc, EnDat2.2, etc
               BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_CLEAR_FAULTS_DONE;
            break;
         }

         BGVAR(s32_Sec_Comm_Fdbk_Timer) = Cntr_1mS;
      break;

      case COMM_FDBK_CLEAR_FAULTS_DONE:
         if (SFB_SERVOSENSE)
         {
            if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PRESENT_MASK)// the encoder FW is valid, check the fault status.
            {
               if ( PassedTimeMS(10L, BGVAR(s32_Sec_Comm_Fdbk_Timer)) ) // wait till the faults are cleared at ServoSense
               {
                  // check faults status
                  if ( SrvSns_GetStatus(drive) & SRVSNS_STATUS_FAULT_MASK )
                  {
                     // Let the user monitor the fault by passsing zero to feedback disable bit
                     SrvSns_SetFWStatus(SRVSNS_FW_STATUS_FLTS_NOT_CLEARED_MASK, 0, drive);
                  }
                  // move to position init state and disable the request
                  BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_INIT_POSITION;
                  // release RT communication
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_2) &= ~(COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK);
               }
            }
            else
            {  // the encoder might be in boot mode, check it's response
               ret_val = SrvSns_ValidateBootMode(*(unsigned int *)FPGA_RX_DATA_READY_REG_ADD, drive);
               if (ret_val == SAL_NOT_FINISHED)
               {
                  if (!PassedTimeMS(SRVSNS_BOOT_RESPONSE_TIME_mSEC, BGVAR(s32_Sec_Comm_Fdbk_Timer)))
                     break;

                  // the encoder is not connected or neither op FW nor boot are present on the encoder
                  SrvSns_SetFWStatus(SRVSNS_FW_STATUS_NOT_CONNECTED_MASK, 1, drive);

                  // Initiate timeout fault
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_2) |= COMM_FDBK_READY_MASK;
               }

               SrvSns_Release(&u32_srvsns_owner, drive);  // Release the ServoSense device

               // Set feedback state to error to prevent further initialization states to execute
               BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_ERROR_STATE;
            }
         }
         else
         {
            // Allow additional Time for Encoder with +5VDC Power to Stabilize if getting to
            // this State of Comm-Feedback Handler (Tamagawa Power-Up).
            if ( ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) != 3)              &&
                   (PassedTimeMS(1600L, BGVAR(s32_Sec_Comm_Fdbk_Timer)))  ) ||
                 ( (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) == 3)              &&
                   (PassedTimeMS(5L, BGVAR(s32_Sec_Comm_Fdbk_Timer)))     )   )
               BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_INIT_POSITION;
         }
      break;

      case COMM_FDBK_INIT_POSITION:
         switch (BGVAR(u16_SFBType))
         {
            case NK_COMM_FDBK: // Sanyo-Denky
               VAR(AX0_u16_Abs_Enc_Command_1Frame_2) = NK_READ_POS_CMD1B; // Position request
               VAR(AX0_u16_Abs_Enc_Command_2Frame_2) = NK_READ_POS_CMD2B; // Position request
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = NK_READ_POS_CMD1B;
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 1 + 0x0080 * enc_source) = NK_READ_POS_CMD2B;
            break;

            case TAMAGAWA_COMM_MULTI_TURN_FDBK:
            case TAMAGAWA_COMM_SINGLE_TURN_FDBK: // Position + Status request
               VAR(AX0_u16_Comm_Fdbk_Flags_B_2) &= ~0x00001; // Process Full Pos. Data
               VAR(AX0_u16_Abs_Enc_Command_1Frame_2) = (unsigned int)(TM_READ_POS_CMD2);
            break;

            case TAMAGAWA_CID0_SINGLE_TURN:
               VAR(AX0_u16_Comm_Fdbk_Flags_B_2) |= 0x00001; // Process ABSA0 - ABSA2 only
               VAR(AX0_u16_Abs_Enc_Command_1Frame_2) = (unsigned int)(TM_CID0_READ_POS_CMD);
            break;

            case SANKYO_COMM_FDBK:
               VAR(AX0_u16_Abs_Enc_Command_1Frame_2) = (unsigned int)(TM_READ_POS_CMD2);
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(TM_READ_POS_CMD2);
            break;

            case PS_P_G_COMM_FDBK: // Position + Status request
               VAR(AX0_u16_Abs_Enc_Command_1Frame_2) = (unsigned int)(PS_P_G_READ_ABSS_CMD3);
               VAR(AX0_u16_Abs_Enc_Command_2Frame_2) = (unsigned int)((PS_P_G_READ_ABSS_CMD1 & 0x001F) << 11); // Position request
               VAR(AX0_u16_Abs_Enc_Command_3Frame_2) = (unsigned int)((PS_P_G_READ_ABSS_CMD1 & 0x07E0) >> 5); // Position request
            break;

            case PS_S_COMM_FDBK: // Full Position request
               VAR(AX0_u16_Abs_Enc_Command_1Frame_2) = (unsigned int)(PS_S_READ_POS_CMD3);
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(PS_S_READ_POS_CMD3); // Standard FPGA
            break;

            case FANUC_COMM_FDBK:
               VAR(AX0_u16_Abs_Enc_Command_1Frame_2) = 0xFFFF;
               VAR(AX0_u16_Abs_Enc_Command_2Frame_2) = 0xF7FF; // Position request
               VAR(AX0_u16_Abs_Enc_Command_3Frame_2) = 0xFFDF; // Position request

               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG3_ADD + 0x0540 * enc_source) = 0xFFDF;
               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG2_ADD + 0x0540 * enc_source) = 0xF7FF;
               *(unsigned int *)(FPGA_SANYO_DENKY_TX_REG1_ADD + 0x0540 * enc_source) = 0xFFFF;
            break;

            case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
            case SERVOSENSE_MULTI_TURN_COMM_FDBK:
               // set velocity mode
               SrvSns_SetCommMode((SRVSNS_COMM_MODE_OPERATIONAL | SRVSNS_COMM_SUBMODE_VELOCITY), drive);
               // Release the ServoSense device
               SrvSns_Release(&u32_srvsns_owner, drive);
            break;

            case YASKAWA_ABS_COMM_FDBK:
            case YASKAWA_INC_COMM_FDBK:
               // The FPGA  module handles the position requests internally
            break;
         }

         // Differ between Multi-Turn and Single-Turn
         if ( (BGVAR(u16_SFBType) == TAMAGAWA_COMM_SINGLE_TURN_FDBK) ||
              (BGVAR(u16_SFBType) == TAMAGAWA_CID0_SINGLE_TURN)        )
            VAR(AX0_u16_Comm_Fdbk_Flags_A_2) |= COMM_FDBK_INIT_ABS_POS_MASK |
                                       COMM_FDBK_INIT_ABS_POS_IN_REV_MASK | COMM_FDBK_READY_MASK;
         else if ( (BGVAR(u16_SFBType) == PS_P_G_COMM_FDBK)     || // Panasonic Incremental Encoder
                   (BGVAR(u16_SFBType) == YASKAWA_INC_COMM_FDBK)  ) // and Yaskawa Incremental Encoder require
            VAR(AX0_u16_Comm_Fdbk_Flags_A_2) |= COMM_FDBK_READY_MASK; // no Initialization
         else if (SFB_SERVOSENSE)//ServoSense
            VAR(AX0_u16_Comm_Fdbk_Flags_A_2) |= COMM_FDBK_READY_MASK; // Multi-Turn already initialized
         else
         // Tamagawa Multi-Turn, SK, Nikon, Yaskawa, Fanuc, Yaskawa Absolute
            VAR(AX0_u16_Comm_Fdbk_Flags_A_2) |= COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK;

         BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_INIT_POSITION_DONE;
      break;

      case COMM_FDBK_INIT_POSITION_DONE:
         if (BGVAR(u16_SFBType) == FANUC_COMM_FDBK)
         {
            do {
               u16_tmp_timer = Cntr_3125;
               u16_clear_faults_iter = VAR(AX0_u16_Abs_Enc_Data_0Frame_2);
               u16_temp_status = VAR(AX0_u16_Comm_Fdbk_Flags_A_2);
            } while (u16_tmp_timer != Cntr_3125);
            if ((u16_temp_status & 0xFFF8) == (COMM_FDBK_READY_MASK | ABS_POSITION_REQ_SENT_MASK))
            // Get Data_Frame3 during "normal" Communication Cycle
            {
               if ((u16_clear_faults_iter & 0x0100) != 0)
               // If no Absolute Position (Index Mark not yet found)
               {
                  if ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_REQUESTED_MASK) == 0)
                     // Initialize Commutation through Phase Find
                     BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_REQUESTED_MASK;
                  else // Once Phase-Find is requested, continue State-Machine
                     u16_clear_faults_iter = 0;
               }
               else // if Absolute Position is valid, avoid Phase-Find and continue
               {
                  BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
                  u16_clear_faults_iter = 0;
               }
            }
         }
         else
            u16_clear_faults_iter = 0;

         if ( (u16_clear_faults_iter == 0)                                           &&
              ((VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & COMM_FDBK_INIT_ABS_POS_MASK) == 0)  )
         {
            if (BGVAR(u16_SFBType) == PS_S_COMM_FDBK)
            {  // After Initialization, switch to querying In-Turn Position with Alarms
               // Field; Multi-Turn Position is updated on Roll-Over.
               // Disable Enc. Comm. before changing Command to avoid indeterminancy,
               // re-enable Enc. Comm. after new Command is in place.
               VAR(AX0_u16_Comm_Fdbk_Flags_A_2) &= ~COMM_FDBK_READY_MASK;
               VAR(AX0_u16_Abs_Enc_Command_1Frame_2) = (unsigned int)(PS_S_READ_POS_CMD4);
               *(unsigned int *)(FPGA_MFB_TX_BUFFER_ADD + 0x0080 * enc_source) = (unsigned int)(PS_S_READ_POS_CMD4); // Dual-Feedback FPGA
               VAR(AX0_u16_Comm_Fdbk_Flags_A_2) |= COMM_FDBK_READY_MASK;
            }

            if ( (BGVAR(u16_SFBType) == PS_P_G_COMM_FDBK)     || // Panasonic Incremental Encoders
                 (BGVAR(u16_SFBType) == YASKAWA_INC_COMM_FDBK)  ) // and Yaskawa Incremental Encoder
            { //  require Hall-Effect readings.
               VAR(AX0_s16_Power_Up_Timer) = 0x7FFF;
               VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_INC_ENC & 0xffff);
               BGVAR(u16_Halls_Invalid_Value) = 0; // Reset Halls Validity Check Counter
               // to avoid Halls-Invalid indication before Halls are initialized
            }

            BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_ENABLE_STATE;
            BGVAR(s32_Sec_Comm_Fdbk_Timer) = Cntr_1mS;
         }
      break;

      case COMM_FDBK_ENABLE_STATE:
      // Added 100mSec. Delay if Encoder is Battery-Backed, to allow Battery-Fault to be
      // detected before Enable after Power-Cycle takes effect.
         if ( ( ((BGVAR(s64_Faults_Mask) & ABS_ENC_BATT_LOW_MASK) != 0) &&
                (PassedTimeMS(100L, BGVAR(s32_Sec_Comm_Fdbk_Timer)))          ) ||
              ( ((BGVAR(s64_Faults_Mask) & ABS_ENC_BATT_LOW_MASK) == 0) &&
                (PassedTimeMS(1L, BGVAR(s32_Sec_Comm_Fdbk_Timer)))            )   )
         {
            BGVAR(s16_DisableInputs) &= ~COMM_SEC_FDBK_DIS_MASK;     //clear disable bit
            BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_IDLE_STATE;

/*            // Update the absolute multi turn feedback indication variable
            switch(BGVAR(u16_SFBType))
            {
               case NK_COMM_FDBK:
               case YASKAWA_ABS_COMM_FDBK:
                  VAR(AX0_u16_Abs_Fdbk_Device) |= 0x0002;
               break;

               case TAMAGAWA_COMM_MULTI_TURN_FDBK:
               case PS_S_COMM_FDBK:
               case SANKYO_COMM_FDBK:
                  VAR(AX0_u16_Abs_Fdbk_Device) |= (((VAR(AX0_u16_Abs_Enc_Data_3Frame_2) & 0x04000) == 0) << 1);
                  // Verify correct structure for SK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
               break;

               case FANUC_COMM_FDBK:
                  VAR(AX0_u16_Abs_Fdbk_Device) |= (((VAR(AX0_u16_Abs_Enc_Data_0Frame_2) & 0x0020) == 0) << 1);
               break;

               case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
                  VAR(AX0_u16_Abs_Fdbk_Device) |= ((unsigned int)(BGVAR(u16_SrvSns_ProductType) == SRVSNS_MULTI_TURN) << 1);
                  BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = 0UL;
               break;

               case SERVOSENSE_MULTI_TURN_COMM_FDBK:
                  if(BGVAR(u16_SrvSns_ProductType) == SRVSNS_SINGLE_TURN)
                  {
                     // Raise warning Feedbacktype Mismatch
                     BGVAR(u64_Sys_Warnings) |= FDBK_TYPE_MISMATCH_WRN_MASK;
                  }
                  else
                  {
                     if(BGVAR(u16_MTP_Mode) && BGVAR(u16_MTP_Load_Done))
                        BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = BGVAR(u32_MTP_Multi_Turn_Number);
                     else
                        BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns) = (unsigned long)SRVSNS_MT_ENCDR_MAX_NUM_TURNS;

                     VAR(AX0_u16_Abs_Fdbk_Device) |= 0x0002;
                  }
               break;
            }

            // This updates the operational homing offset with the user non-volatile variable
            // according to the feedback multi turn absolute
            UpdateRefOffsetValue(drive); 

           // The following call updates the state of the MT Batt faults ignorance for the configured feedback type
           if (SalIgnoreAbsFdbkBattFlt(BGVAR(u16_Ignore_Abs_Fdbk_Batt_Faults), drive) == SAL_NOT_FINISHED)
              return;*/

//            if (BGVAR(u16_Ignore_Abs_Fdbk_Batt_Faults) == IGNOREBATTFLT_MODE_INACTIVE)
//                 // If the MT igorance is active, then ignore this call. Fix to Bugzilla 5913.
//               UpdateRefOffsetValue(drive);

            BGVAR(s16_DisableInputs) &= ~COMM_SEC_FDBK_DIS_MASK;     //clear disable bit
            BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_IDLE_STATE;
         }
         BGVAR(u16_Comm_Sec_Fdbk_Init_Ran) = 1;
      break;
   }
}


void RemoveFeedback5Volt(void)
{ // Power down the 5V to the encoder
   if ( (u16_Product == DDHD) || (u16_Product == SHNDR_HW) )
      GpioDataRegs.GPBDAT.bit.GPIO49 = 0;
   // For DDHD and SHNDR_HW control ALSO the bit in FPGA for Tamagawa initialization
   *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD &= ~0x01;
}


void SetFeedback5Volt(void)
{ // Power up the 5V to the encoder
   if ( (u16_Product == DDHD) || (u16_Product == SHNDR_HW) )
      GpioDataRegs.GPBDAT.bit.GPIO49 = 1;
   // For DDHD and SHNDR_HW control ALSO the bit in FPGA for Tamagawa initialization
   *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD |= 0x01;
}


unsigned int GetFeedback5VoltStatus(void)
{
   return ((*(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD & 0x01) > 0);
}


void RemoveFeedbackSecondary5Volt(void)
{ // Power down the 5V to the secondary encoder
   *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD &= ~0x02;
}


void SetFeedbackSecondary5Volt(void)
{ // Power up the 5V to the secondary encoder
   *(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD |= 0x02;
}


unsigned int GetFeedbackSec5VoltStatus(void)
{
   return ((*(unsigned int *)FPGA_FB_POWER_SUPPLY_REG_ADD & 0x02) > 0);
}


//**********************************************************
// Function Name: SalTmTurnResetCommand
// Description:
//   This function called in response to TMTURNRESET command
// Logic:  The added parameter indicates Primary or Secondary Encoder Source.  For Backward Compatibility,
//         no parameters means Primary Souce; Parameter '0' indicates Primary, Parameter '1' indicates
//         Secondary, all other combinations reported as incorrect.
//         Detect applicable Feedback Type, Initialization in progress, or Communication Faults.
//         Operate source-respective Comm.-Feedback Initialization State-Machine, with modification to issue
//         the Multi-Turn Zeroing Command instead of standard Reset Command.
//         Return Prompt after Initialization Process is complete.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalTmTurnResetCommand(int drive)
{
   unsigned int ret_val = SAL_NOT_FINISHED, u16_enc_source = 0; // Default when no Parameters are set.
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (s16_Number_Of_Parameters > 1)
      ret_val = VALUE_OUT_OF_RANGE;
   else if (s16_Number_Of_Parameters == 1)
   {
       if ( ((unsigned int)s64_Execution_Parameter[0] == 0) ||
            ((unsigned int)s64_Execution_Parameter[0] == 1)   )
       // If a single Parameter is present, then '0' for Primary Encoder Source, '1' for Secondary
//      u16_enc_source = (unsigned int)s64_Execution_Parameter[0];
         u16_enc_source = 0; // Temporary until including Secondary Feedback Source Code...
   }

   if ( ( (u16_enc_source == 0) && (LVAR(AX0_s32_Feedback_Ptr) != &TM_COMMUNICATION_FEEDBACK)   &&
        (LVAR(AX0_s32_Feedback_Ptr) != &PS_S_COMMUNICATION_FEEDBACK)                            &&
        (LVAR(AX0_s32_Feedback_Ptr) != &NK_COMMUNICATION_FEEDBACK)                                ) /* ||
        ( (u16_enc_source == 1) && (LVAR(AX0_s32_Feedback_Ptr_2) != &TM_COMMUNICATION_FEEDBACK)   &&
        (LVAR(AX0_s32_Feedback_Ptr_2) != &PS_S_COMMUNICATION_FEEDBACK)                            &&
        (LVAR(AX0_s32_Feedback_Ptr_2) != &NK_COMMUNICATION_FEEDBACK)                                ) */ )
      ret_val = MENCTYPE_MISMATCH; // Use only with Tamagawa, Nikon, and Panasonic Absolute Encoders.

   if ( (ret_val == SAL_NOT_FINISHED)                                                                &&
        (BGVAR(s64_SysNotOk) & (TAMAGAWA_ABS_ENC_FLT_MASK | SANYO_FDBK_FLT_MASK | COMM_FDBK_FLT_MASK))  )
      ret_val = FDBKTYPE_COMM_ERR;

   if ( (ret_val == SAL_NOT_FINISHED) && ((BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) & 0x7FFF) == 1)            &&
        ( ( (u16_enc_source == 0) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) )    ||
          ( (u16_enc_source == 1) && (BGVAR(s16_Comm_Sec_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) )  )  )
      ret_val = FDBKTYPE_INIT_PRCSS;

   // Use upper Bit of State Machine variable to allow resetting of Battery-Fault only
   // after TMTURNRESET was used (Customer request);
   if ( (ret_val == SAL_NOT_FINISHED) && ((BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) & 0x7FFF) == 1) )
      BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) &= 0x7FFF;

   switch (BGVAR(s16_Comm_Fdbk_TM_Rst_Turn))
   {
      case 1:
         if (ret_val == SAL_NOT_FINISHED)
            BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) = 2;
      break;

      case 2:
         if (u16_enc_source == 0)
            BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
         else
            BGVAR(s16_Comm_Sec_Fdbk_Init_State) = COMM_FDBK_START_INIT;
         BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) = 3;
      break;

      case 3:
         if ( ( (u16_enc_source == 0) && (BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_IDLE_STATE) )    ||
              ( (u16_enc_source == 1) && (BGVAR(s16_Comm_Sec_Fdbk_Init_State) == COMM_FDBK_IDLE_STATE) )  )
         { // Reset local State machine, respond SAL_SUCCESS to allow Prompt.
            ret_val = SAL_SUCCESS;
            BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) = 1;
         }
      break;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SalSkTurnResetCommand
// Description:
//   This function called in response to SKTURNRESET command
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalSkTurnResetCommand(int drive)
{
   int ret_val = SAL_NOT_FINISHED, i, j;

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (LVAR(AX0_s32_Feedback_Ptr) != &SK_COMMUNICATION_FEEDBACK)
      return MENCTYPE_MISMATCH; // Use only with Sankyo Encoder.

   BGVAR(s16_Comm_Fdbk_TM_Rst_Turn) &= 0x7FFF;

   if ( ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_SK_MULTI_TURN_RESET_MASK) == 0)  &&
        ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_SK_MULTI_TURN_RST_OFF_MASK) == 0)  )
   { // Initialize Sankyo Multi-Turn Reset Command once.
      u8_Scic_Rx_Data[0] = 0x62; // Command ID "C".
      u8_Scic_Rx_Data[1] = 0x00;
      u8_Scic_Rx_Data[2] = 0x00;
      u8_Scic_Rx_Data[3] = 0x00;

      for (j = 0; j < 4; j++)
      {
         for (i = 0; i < 8; i++) // Flip Fields to send LSB first...
            u8_Scic_Rx_Data[j] |= ((u8_Scic_Rx_Data[j] & (0x01 << i)) << (15 - (2 * i)));
         u8_Scic_Rx_Data[j] = (((u8_Scic_Rx_Data[j] >> 7) | 0x01) & 0x01FF);
         // Arrange with Leading 0, Terminating 1 (10-Bit Field)
      }

      VAR(AX0_u16_Com_Enc_Altern_4Frame) = 0x0020 | ((u8_Scic_Rx_Data[0] >> 5) & 0x01F); // Include '1' at beginning of Xmit
      VAR(AX0_u16_Com_Enc_Altern_3Frame) = ((u8_Scic_Rx_Data[0] << 11) & 0x0F800) | ((u8_Scic_Rx_Data[1] << 1) & 0x07FE) | ((u8_Scic_Rx_Data[2] >> 9) & 0x0001);
      VAR(AX0_u16_Com_Enc_Altern_2Frame) = ((u8_Scic_Rx_Data[2] << 7) & 0x0FF80) | ((u8_Scic_Rx_Data[3] >> 3) & 0x007F);
      VAR(AX0_u16_Com_Enc_Altern_1Frame) = ((u8_Scic_Rx_Data[3] << 13) & 0x0E000);
      VAR(AX0_u16_Com_Enc_Alt_Xmt_Length) = 11;
      VAR(AX0_u16_Com_Enc_Alt_Xcv_Length) = 60;

      VAR(AX0_s16_Zero_Position) = 15;

      VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_SK_MULTI_TURN_RESET_MASK;
   }

   if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_SK_MULTI_TURN_RST_OFF_MASK) != 0)
   { // After Multi-Turn Reset is finished, re-initialize Abs. Enc. Position
      VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_INIT_ABS_POS_MASK;
      VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~COMM_FDBK_SK_MULTI_TURN_RST_OFF_MASK;
      ret_val = SAL_SUCCESS;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SalMtTurnResetCommand
// Description:
//   This function called in response to MTTURNRESET command
//
//
// Author: A. O.
// Algorithm:
// Revisions:
//**********************************************************
int SalMtTurnResetCommand(int drive)
{
   register unsigned int ret_val = SAL_NOT_FINISHED;

   if(Enabled(drive))
      return DRIVE_ACTIVE;

   if(VAR(AX0_u16_Abs_Fdbk_Device) == 0)
      return NOT_AVAILABLE;

   switch(BGVAR(u16_FdbkType))
   {
      case SW_SINE_FDBK:
         if(!FEEDBACK_STEGMANN)
            return NOT_AVAILABLE;

         return NOT_AVAILABLE;// MTTURNRESET For Stegman Hyperface TODO

      case NK_COMM_FDBK:
      case TAMAGAWA_COMM_MULTI_TURN_FDBK:
      case PS_S_COMM_FDBK:
         ret_val =  SalTmTurnResetCommand(drive);
      break;

      case SANKYO_COMM_FDBK:
         ret_val = SalSkTurnResetCommand(drive);
      break;

      case SERVOSENSE_MULTI_TURN_COMM_FDBK:
         ret_val = SrvSns_ResetTurns(drive, 0); //added 0 to the function call since this SWITCH treats only feedbacktype and not sfbtype
      break;

      default:
         return NOT_AVAILABLE;
   }

   if(ret_val == SAL_SUCCESS)
   {
      // HOME DONE indication shall be reset
      // Reset homeoffset to zero
      ret_val = SalSavedHomeOfstCommand(0LL, drive);
   }

   return ret_val;
}


//**********************************************************
// Function Name: SalTmCommandGeneric
// Description:
//   This function called in response to TMCOMMAND command
// Instructions:
//   a.  Enter all Parameters of Tamagawa Absolute Encoder Command in order; the Function adds
//       Sync Code to 1st Parameter (Data ID) and calculates CRC.
//   b.  If printing is problematic, observe array of u8_Scic_Rx_Data[32] for response bytes
//       (including Command, specifics, and CRC).
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalTmCommandHack(int drive)
{
   // AXIS_OFF;
   unsigned int Temp_Calc, Num_Params, Parity, Command_CRC = 0, i, j, Bits_to_Send, Bits_to_Receive;
   REFERENCE_TO_DRIVE;
   if ( (LVAR(AX0_s32_Feedback_Ptr) != &TM_COMMUNICATION_FEEDBACK) &&
        (LVAR(AX0_s32_Feedback_Ptr) != &SK_COMMUNICATION_FEEDBACK)   )
      return MENCTYPE_MISMATCH; // Use only with TM or SK Encoders.

   Num_Params = s16_Number_Of_Parameters;
   for (i = 0; i < Num_Params; i++)
      u8_Scic_Rx_Data[i] = (unsigned int)s64_Execution_Parameter[i];

   if (Num_Params == 1) // Calculate Bits_to_Send and Bits_to_Receive according to Command Type
   {
      Bits_to_Send = 11;
      switch (u8_Scic_Rx_Data[0])
      {
         case 0: case 1: case 7: case 8: case 12: // Queries or Commands with responses of
            Bits_to_Receive = 60;     // Control Field, Status Field, 3 Data Fields, and CRC Field
         break;
         case 2:  // Query with response of Control Field, Status Field, Data Field, and CRC Field
            Bits_to_Receive = 40;
         break;
         case 3: default:  // Query with response of Control Field, Status Field, 8 Data Fields,
            Bits_to_Receive = 110; // and CRC Field; or unknown command...
         break;
      }
   }
   else
   { // Responses to all EEPROM Commands are 4 Fields long.
      Bits_to_Send = 10 * (Num_Params + 1) + 1;
      Bits_to_Receive = 40; // Response of Control, Address, Content, and CRC Fields
   }

// Set Command Field to desired action:  Data Readout, EEPROM Read/Write, or Reset; add Sync Code
   Temp_Calc = u8_Scic_Rx_Data[0];
   u8_Scic_Rx_Data[0] = ((Temp_Calc << 3) & 0x0FF) | 0x02; // Add Sync Code...
   Parity = 0;
   while (Temp_Calc != 0) // Calculate Data ID Parity...
   {
      Parity = !Parity;
      Temp_Calc &= (Temp_Calc - 1);
   }
   u8_Scic_Rx_Data[0] |= (Parity << 7); // ...and include ID Parity in Command Field

   if (Num_Params > 1)
   {
      Temp_Calc = (u8_Scic_Rx_Data[0] << 8) | u8_Scic_Rx_Data[1];
      for (i = 0; i < 8; i++) // Calculate CRC...
      {
         if ((Temp_Calc & 0x08000) != 0) // XOR required...
         {
            Temp_Calc <<= 1;
            Temp_Calc ^= 0x0100; // CRC Polynomial X^8 + 1...
         }
         else
            Temp_Calc <<= 1;
      }
      Command_CRC = Temp_Calc >> 8;
   }
   else
      u8_Scic_Rx_Data[1] = u8_Scic_Rx_Data[2] = u8_Scic_Rx_Data[3] = 0;

   if (Num_Params > 2)
   {
      Temp_Calc = (Command_CRC << 8) | u8_Scic_Rx_Data[2];
      for (i = 0; i < 8; i++) // Calculate Field CRC...
      {
         if ((Temp_Calc & 0x08000) != 0) // XOR required...
         {
            Temp_Calc <<= 1;
            Temp_Calc ^= 0x0100; // CRC Polynomial X^8 + 1...
         }
         else
            Temp_Calc <<= 1;
      }
      Command_CRC = Temp_Calc >> 8;
      u8_Scic_Rx_Data[3] = Command_CRC;
   }
   else
   {
      u8_Scic_Rx_Data[2] = Command_CRC;
      u8_Scic_Rx_Data[3] = 0;
   }

   for (j = 0; j <= Num_Params; j++)
   {  // Arrange with Leading 0, Terminating 1 (10-Bit Field)
      u8_Scic_Rx_Data[j] = (((u8_Scic_Rx_Data[j] << 1) | 0x200) & 0x03FF);
   }

   VAR(AX0_u16_Com_Enc_Altern_1Frame) = 0x0001 | ((u8_Scic_Rx_Data[0] << 1) & 0x07FE) | ((u8_Scic_Rx_Data[1] << 11) & 0x0F800);
   VAR(AX0_u16_Com_Enc_Altern_2Frame) = ((u8_Scic_Rx_Data[1] >> 5) & 0x0001F) | ((u8_Scic_Rx_Data[2] << 5) & 0x07FE0) | ((u8_Scic_Rx_Data[3] << 15) & 0x08000);
   VAR(AX0_u16_Com_Enc_Altern_3Frame) = ((u8_Scic_Rx_Data[3] >> 1) & 0x001FF) | 0X0FE00;

   VAR(AX0_u16_Com_Enc_Alt_Xmt_Length) = Bits_to_Send;
   VAR(AX0_u16_Com_Enc_Alt_Xcv_Length) = Bits_to_Receive;

   VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_ALTERN_CMD_SEND_MASK;
   // Set Serial Encoder Alternative Command bit

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadTmTemp
// Description:
// This function called in response to TMTEMP command
// Logic:  Calls the Internal Function ReadTmTemp, with parameters that indicate Primary or Secondary
//         Encoder Source.  For Backward Compatibility, no parameters means Primary Souce; Parameter '0'
//         indicates Primary, Parameter '1' indicates Secondary, all other combinations reported as incorrect.
//         Detect Initialization in progress, or Communication Faults.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalReadTmTemp(int drive)
{
   static unsigned int  u16_enc_source = 0; // Default when no Parameters are set.
   unsigned int ret_val = SAL_NOT_FINISHED;
   // AXIS_OFF;

   switch (BGVAR(s16_Comm_Fdbk_TM_Temp_Read))
   {
      case 1:
         if (s16_Number_Of_Parameters == 1)
         {  // If a Parameter is present, then '0' for Primary Encoder Source, '1' for Secondary
            if ( ((unsigned int)s64_Execution_Parameter[0] == 0) ||
                 ((unsigned int)s64_Execution_Parameter[0] == 1)   )
//               u16_enc_source = (unsigned int)s64_Execution_Parameter[0];
               u16_enc_source = 0; // Until Dual-Feedback Code is fully included...
            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            else
               ret_val = VALUE_OUT_OF_RANGE;
         }

         if ( ( (u16_enc_source == 0) && (LVAR(AX0_s32_Feedback_Ptr) != &TM_COMMUNICATION_FEEDBACK) ) /* ||
              ( (u16_enc_source == 1) && (LVAR(AX0_s32_Feedback_Ptr_2) != &TM_COMMUNICATION_FEEDBACK) ) */ )
            ret_val = MENCTYPE_MISMATCH; // Use only with TM Encoder.

         if (BGVAR(s64_SysNotOk) & (TAMAGAWA_ABS_ENC_FLT_MASK | COMM_FDBK_FLT_MASK))
            ret_val = FDBKTYPE_COMM_ERR;

         if ( ( (u16_enc_source == 0) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) )    ||
              ( (u16_enc_source == 1) && (BGVAR(s16_Comm_Sec_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) )  )
            ret_val = FDBKTYPE_INIT_PRCSS;

         BGVAR(s16_Comm_Fdbk_TM_Temp_Read) = 2;
      break;

      case 2:
         ret_val = ReadTmTemp(drive, u16_enc_source, 0);
         if (ret_val != SAL_NOT_FINISHED)
            BGVAR(s16_Comm_Fdbk_TM_Temp_Read) = 1;
      break;
   }

   return ret_val;
}


//**********************************************************
// Function Name: ReadTmTemp
// Description:
// This function is called from SalReadTmTemp
// Logic:
//   a.  Read TM Encoder EEPROM Address 0x4F by sending Control Field 0x0D, Address 0x4F,
//       and CRC to the Encoder, and reading the Third-to-the-Last Field from the response.
//   b.  Verify (calculate) correct CRC of Response.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int ReadTmTemp(int drive, int enc_source, int fdbk_dest)
{
   // AXIS_OFF;
   FDBK_OFF;
   static unsigned int TmTemp_state = 0;
   unsigned long Temp32;
   unsigned int Temp_Calc, Parity, i, j;

   REFERENCE_TO_DRIVE;
   enc_source += 0;
   switch (TmTemp_state)
   {
      case 0: // Prepare Command Register and Send
         u8_Scic_Rx_Data[0] = 0x0D;
         u8_Scic_Rx_Data[1] = 0x4F;

         Temp_Calc = u8_Scic_Rx_Data[0];
         u8_Scic_Rx_Data[0] = ((Temp_Calc << 3) & 0x0FF) | 0x02; // Add Sync Code...
         Parity = 0;
         while (Temp_Calc != 0) // Calculate Data ID Parity...
         {
            Parity = !Parity;
            Temp_Calc &= (Temp_Calc - 1);
         }
         u8_Scic_Rx_Data[0] |= (Parity << 7); // ...and include ID Parity in Command Field

         Temp_Calc = (u8_Scic_Rx_Data[0] << 8) | u8_Scic_Rx_Data[1];
         for (i = 0; i < 8; i++) // Calculate CRC...
         {
            if ((Temp_Calc & 0x08000) != 0) // XOR required...
            {
               Temp_Calc <<= 1;
               Temp_Calc ^= 0x0100; // CRC Polynomial X^8 + 1...
            }
            else
               Temp_Calc <<= 1;
         }
         u8_Scic_Rx_Data[2] = Temp_Calc >> 8; // CRC to send...
         u8_Scic_Rx_Data[3] = 0;

         for (j = 0; j < 3; j++)
         {  // Arrange with Leading 0, Terminating 1 (10-Bit Field)
            u8_Scic_Rx_Data[j] = (((u8_Scic_Rx_Data[j] << 1) | 0x200) & 0x03FF);
         }

         FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_1Frame)) = 0x0001 | ((u8_Scic_Rx_Data[0] << 1) & 0x07FE) | ((u8_Scic_Rx_Data[1] << 11) & 0x0F800);
         FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_2Frame)) = ((u8_Scic_Rx_Data[1] >> 5) & 0x0001F) | ((u8_Scic_Rx_Data[2] << 5) & 0x07FE0) | ((u8_Scic_Rx_Data[3] << 15) & 0x08000);
         FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_3Frame)) = ((u8_Scic_Rx_Data[3] >> 1) & 0x001FF) | 0X0FE00;

         FDBKVAR(VAR(AX0_u16_Com_Enc_Alt_Xmt_Length)) = 31;
         FDBKVAR(VAR(AX0_u16_Com_Enc_Alt_Xcv_Length)) = 40;

         FDBKVAR(VAR(AX0_u16_Comm_Fdbk_Flags_A_1)) |= COMM_FDBK_ALTERN_CMD_SEND_MASK;
         // Set Serial Encoder Alternative Command bit
         TmTemp_state = 1;
      break;

      case 1: // Wait for response, then print
         if ((FDBKVAR(VAR(AX0_u16_Comm_Fdbk_Flags_A_1)) & (COMM_FDBK_ALTERN_CMD_SEND_MASK | COMM_FDBK_ALTERN_RESP_RCV_MASK)) != 0)
            return SAL_NOT_FINISHED; // Wait until command is sent and response is received...

         i = Cntr_3125;
         while (Cntr_3125 < (i + 2)) {}; // a short delay...

         // Using four Fields (all stored in AX0_u32_Abs_Enc_Altern_1Data) to verify CRC
         Temp32 = FDBKLVAR(LVAR(AX0_u32_Abs_Enc_Altern_1Data));
         for (i = 0; i < 24; i++) // Calculate CRC...
         {
            if ((Temp32 & 0x080000000) != 0) // XOR required...
            {
               Temp32 <<= 1;
               Temp32 ^= 0x01000000L; // CRC Polynomial X^8 + 1...
            }
            else
               Temp32 <<= 1;
         }

         if (Temp32 != 0L)
         {
            TmTemp_state = 0;
            return TM_CRC_ERROR;
         }
         else
         {
            PrintUnsignedInteger((unsigned int)((FDBKLVAR(LVAR(AX0_u32_Abs_Enc_Altern_1Data)) >> 16) & 0x00FF));
            PrintString(" [deg C]", 0);
         }
         PrintCrLf();
         TmTemp_state = 0;
      break;
   }

   if (TmTemp_state == 0) return SAL_SUCCESS;
   else return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: SalReadSankyo
// Description:
// This function called in response to SKTEMPVOLT command
// Logic:
//   a.  Read SK Encoder Temperature and DC Voltage using Command ID 4, and extracting
//       Data Frame Df5 for Temperature and Data Frame Df6 for Battery Voltage.
//   b.  Verify (calculate) correct CRC of Response.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalReadSankyo(int drive)
{
   // AXIS_OFF;
   static unsigned int SkRead_state = 0;
   unsigned long Temp32;
   unsigned int Temp_Calc, i, j;
   REFERENCE_TO_DRIVE;
   if (LVAR(AX0_s32_Feedback_Ptr) != &SK_COMMUNICATION_FEEDBACK)
      return MENCTYPE_MISMATCH; // Use only with SK Encoder.

   if (BGVAR(s64_SysNotOk) &  COMM_FDBK_FLT_MASK)
      return FDBKTYPE_COMM_ERR;

   if (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE)
      return FDBKTYPE_INIT_PRCSS;

   switch (SkRead_state)
   {
      case 0: // Prepare Command Register and Send
         if (s16_Number_Of_Parameters > 0) // Writing Temp. Threshold
            u8_Scic_Rx_Data[0] = (unsigned int)s64_Execution_Parameter[0];
         else
            u8_Scic_Rx_Data[0] = 0x3A; // SK implementation of Command ID "4"
         u8_Scic_Rx_Data[1] = 0x00;
         u8_Scic_Rx_Data[2] = 0;
         u8_Scic_Rx_Data[3] = 0;

         for (j = 0; j < 4; j++)
         {
            for (i = 0; i < 8; i++) // Flip Fields to send LSB first...
               u8_Scic_Rx_Data[j] |= ((u8_Scic_Rx_Data[j] & (0x01 << i)) << (15 - (2 * i)));
            u8_Scic_Rx_Data[j] = (((u8_Scic_Rx_Data[j] >> 7) | 0x01) & 0x01FF);
            // Arrange with Leading 0, Terminating 1 (10-Bit Field)
         }

         VAR(AX0_u16_Com_Enc_Altern_4Frame) = 0x0020 | ((u8_Scic_Rx_Data[0] >> 5) & 0x01F); // Include '1' at beginning of Xmit
         VAR(AX0_u16_Com_Enc_Altern_3Frame) = ((u8_Scic_Rx_Data[0] << 11) & 0x0F800) | ((u8_Scic_Rx_Data[1] << 1) & 0x07FE) | ((u8_Scic_Rx_Data[2] >> 9) & 0x0001);
         VAR(AX0_u16_Com_Enc_Altern_2Frame) = ((u8_Scic_Rx_Data[2] << 7) & 0x0FF80) | ((u8_Scic_Rx_Data[3] >> 3) & 0x007F);
         VAR(AX0_u16_Com_Enc_Altern_1Frame) = ((u8_Scic_Rx_Data[3] << 13) & 0x0E000);
         VAR(AX0_u16_Com_Enc_Alt_Xmt_Length) = 11;
         VAR(AX0_u16_Com_Enc_Alt_Xcv_Length) = 110; //127 //130 // 125 // 120

         VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_ALTERN_CMD_SEND_MASK;
         // Set Serial Encoder Alternative Command bit
         SkRead_state = 1;
      break;

      case 1: // Wait for response, then print
         if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & (COMM_FDBK_ALTERN_CMD_SEND_MASK | COMM_FDBK_ALTERN_RESP_RCV_MASK)) != 0)
            return SAL_NOT_FINISHED; // Wait until command is sent and response is received...

         i = Cntr_3125;
         while (Cntr_3125 < (i + 2)) {}; // a short delay...

         // Start with Control field, Status Field, and Data Field 0 to calculate CRC...
         Temp32 = LVAR(AX0_u32_Abs_Enc_Altern_1Data);
         for (i = 0; i < 24; i++) // Calculate CRC...
         {
            if ((Temp32 & 0x080000000L) != 0) // XOR required...
            {
               Temp32 <<= 1;
               Temp32 ^= 0x01000000L; // CRC Polynomial X^8 + 1...
            }
            else
               Temp32 <<= 1;
         }

         // Partial CRC Data left in upper 8 Bits, add Data Fields 2, 3, 4, and 5...
         Temp32 ^= LVAR(AX0_u32_Abs_Enc_Altern_2Data);
         for (i = 0; i < 24; i++) // Calculate CRC...
         {
            if ((Temp32 & 0x080000000L) != 0) // XOR required...
            {
               Temp32 <<= 1;
               Temp32 ^= 0x01000000L; // CRC Polynomial X^8 + 1...
            }
            else
               Temp32 <<= 1;
         }

         // Partial CRC Data left in upper 8 Bits, add Data Fields 6, 7, and CRC...
         Temp32 ^= (LVAR(AX0_u32_Abs_Enc_Altern_3Data) & 0xFFFFFF00L);
         for (i = 0; i < 16; i++) // Calculate CRC...
         {
            if ((Temp32 & 0x080000000L) != 0) // XOR required...
            {
               Temp32 <<= 1;
               Temp32 ^= 0x01000000L; // CRC Polynomial X^8 + 1...
            }
            else
               Temp32 <<= 1;
         }
/* Fix CRC Calculations for Temperature & Voltage readings!!! Most likely reverse Bit-Order...
         if (Temp32 != 0L)
         {
            SkRead_state = 0;
            PrintUnsignedInteger((LVAR(AX0_u32_Abs_Enc_Altern_3Data) >> 16) & 0x00FFL); // Required Shift...
            PrintString(", ", 0); // Debugging...
            return TM_CRC_ERROR;
         }
         else */
         {
//            PrintUnsignedInteger(LVAR(AX0_u32_Abs_Enc_Altern_3Data) & 0x00FFL); // Required Shift...
//            PrintString(", ", 0); // Debugging...
            Temp_Calc = (unsigned int)((LVAR(AX0_u32_Abs_Enc_Altern_2Data) >> 24) & 0x00FFL);
            Temp_Calc = (unsigned int)(106.65 - (0.51 * Temp_Calc));
            PrintUnsignedInteger(Temp_Calc); // Df5 contains Temperature Indication
            PrintString(" [deg C], ", 0);
            Temp_Calc = (unsigned int)(LVAR(AX0_u32_Abs_Enc_Altern_3Data) & 0x00FFL);
            Temp32 = (long)((32.5 * (float)Temp_Calc) - 550);
            PrintString(DecimalPoint32ToAscii(Temp32), 0);
            PrintString(" [Volts]", 0);
         }
         PrintCrLf();
         SkRead_state = 0;
      break;
   }

   if (SkRead_state == 0) return SAL_SUCCESS;
   else return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: SalTmTempThresh
// Description:
// This function called in response to TMTEMPTHRESH command
// Logic:
//   a.  To read existing threshold, read TM Encoder EEPROM Address 0x4E by sending Control
//       Field 0x0D, Address 0x4E, and CRC.
//   b.  To set threshold, send desired value to TM Encoder EEPROM Address 0x4E by sending
//       Control Field 0x06, Address 0x4E, desired Temperature, and CRC.
//   c.  Verify (calculate) correct CRC of Response.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalTmTempThresh(int drive)
{
   // AXIS_OFF;
   static unsigned int Parity, TmThresh_state = 0;
   unsigned long Temp32;
   unsigned int Temp_Calc, i, j;
   REFERENCE_TO_DRIVE;
   if (LVAR(AX0_s32_Feedback_Ptr) != &TM_COMMUNICATION_FEEDBACK)
      return MENCTYPE_MISMATCH; // Use only with TM Encoder.

   if (BGVAR(s64_SysNotOk) & (TAMAGAWA_ABS_ENC_FLT_MASK | COMM_FDBK_FLT_MASK ))
      return FDBKTYPE_COMM_ERR;

   if (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE)
      return FDBKTYPE_INIT_PRCSS;

   switch (TmThresh_state)
   {
      case 0: // Prepare Command Register and Send
         u8_Scic_Rx_Data[1] = 0x4E;
         if (s16_Number_Of_Parameters > 0) // Writing Temp. Threshold
         {
            u8_Scic_Rx_Data[0] = 0x06; // Write to EEPROM Command
            u8_Scic_Rx_Data[2] = (unsigned int)s64_Execution_Parameter[0];
         }
         else                             // Reading existing threshold
         {
            u8_Scic_Rx_Data[0] = 0x0D; // Read EEPROM Command
            u8_Scic_Rx_Data[2] = 0;
         }

         Temp_Calc = u8_Scic_Rx_Data[0];
         u8_Scic_Rx_Data[0] = ((Temp_Calc << 3) & 0x0FF) | 0x02; // Add Sync Code...
         Parity = 0;
         while (Temp_Calc != 0) // Calculate Data ID Parity...
         {
            Parity = !Parity;
            Temp_Calc &= (Temp_Calc - 1);
         }
         u8_Scic_Rx_Data[0] |= (Parity << 7); // ...and include ID Parity in Command Field

         Temp_Calc = (u8_Scic_Rx_Data[0] << 8) | u8_Scic_Rx_Data[1];
         for (i = 0; i < 8; i++) // Calculate CRC...
         {
            if ((Temp_Calc & 0x08000) != 0) // XOR required...
            {
               Temp_Calc <<= 1;
               Temp_Calc ^= 0x0100; // CRC Polynomial X^8 + 1...
            }
            else
               Temp_Calc <<= 1;
         }

         if (u8_Scic_Rx_Data[2] == 0) // No Parameter, reading existing value...
         {
            u8_Scic_Rx_Data[2] = Temp_Calc >> 8; // CRC to send...
            u8_Scic_Rx_Data[3] = 0;
         }
         else // Writing new value...
         {
            Temp_Calc = Temp_Calc | u8_Scic_Rx_Data[2];
            for (i = 0; i < 8; i++) // Calculate Field CRC...
            {
               if ((Temp_Calc & 0x08000) != 0) // XOR required...
               {
                  Temp_Calc <<= 1;
                  Temp_Calc ^= 0x0100; // CRC Polynomial X^8 + 1...
               }
               else
                  Temp_Calc <<= 1;
            }
            u8_Scic_Rx_Data[3] = Temp_Calc >> 8;
         }

         Temp_Calc = 3 + (u8_Scic_Rx_Data[3] != 0); // Set Number of Fields to be sent...

         Parity = u8_Scic_Rx_Data[2]; // Maintain Input Value for later test...

         if (*(unsigned int *)FPGA_USE_TX_RX_BUFFER_REG_ADD == 0)
         {
            for (j = 0; j < Temp_Calc; j++)
            {
               for (i = 0; i < 8; i++) // Flip Fields to send LSB first...
                  u8_Scic_Rx_Data[j] |= ((u8_Scic_Rx_Data[j] & (0x01 << i)) << (15 - (2 * i)));
               u8_Scic_Rx_Data[j] = (((u8_Scic_Rx_Data[j] >> 7) | 0x01) & 0x01FF);
               // Arrange with Leading 0, Terminating 1 (10-Bit Field)
            }
         }
         else
         {
            for (j = 0; j < Temp_Calc; j++)
            {  // Arrange with Leading 0, Terminating 1 (10-Bit Field)
               u8_Scic_Rx_Data[j] = (((u8_Scic_Rx_Data[j] << 1) | 0x200) & 0x03FF);
            }
         }

         if (*(unsigned int *)FPGA_USE_TX_RX_BUFFER_REG_ADD == 0)
         {  // Arrange 10-Bit Fields in 16-Bit Register to transmit in required order...
            VAR(AX0_u16_Com_Enc_Altern_4Frame) = 0x0020 | ((u8_Scic_Rx_Data[0] >> 5) & 0x01F); // Include '1' at beginning of Xmit
            VAR(AX0_u16_Com_Enc_Altern_3Frame) = ((u8_Scic_Rx_Data[0] << 11) & 0x0F800) | ((u8_Scic_Rx_Data[1] << 1) & 0x07FE) | ((u8_Scic_Rx_Data[2] >> 9) & 0x0001);
            VAR(AX0_u16_Com_Enc_Altern_2Frame) = ((u8_Scic_Rx_Data[2] << 7) & 0x0FF80) | ((u8_Scic_Rx_Data[3] >> 3) & 0x007F);
            VAR(AX0_u16_Com_Enc_Altern_1Frame) = ((u8_Scic_Rx_Data[3] << 13) & 0x0E000);
         }
         else
         {
            VAR(AX0_u16_Com_Enc_Altern_1Frame) = 0x0001 | ((u8_Scic_Rx_Data[0] << 1) & 0x07FE) | ((u8_Scic_Rx_Data[1] << 11) & 0x0F800);
            VAR(AX0_u16_Com_Enc_Altern_2Frame) = ((u8_Scic_Rx_Data[1] >> 5) & 0x0001F) | ((u8_Scic_Rx_Data[2] << 5) & 0x07FE0) | ((u8_Scic_Rx_Data[3] << 15) & 0x08000);
            VAR(AX0_u16_Com_Enc_Altern_3Frame) = ((u8_Scic_Rx_Data[3] >> 1) & 0x001FF) | 0X0FE00;
         }

         VAR(AX0_u16_Com_Enc_Alt_Xmt_Length) = (Temp_Calc * 10) + 1;
         VAR(AX0_u16_Com_Enc_Alt_Xcv_Length) = 40;

         VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_ALTERN_CMD_SEND_MASK;
         // Set Serial Encoder Alternative Command bit

         TmThresh_state = 1;
      break;

      case 1: // Wait for response, then print
         if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & (COMM_FDBK_ALTERN_CMD_SEND_MASK | COMM_FDBK_ALTERN_RESP_RCV_MASK)) != 0)
            return SAL_NOT_FINISHED; // Wait until command is sent and response is received...

         i = Cntr_3125;
         while (Cntr_3125 < (i + 2)) {}; // a short delay...

         // Using four Fields (all stored in AX0_u32_Abs_Enc_Altern_1Data) to verify CRC
         Temp32 = LVAR(AX0_u32_Abs_Enc_Altern_1Data);
         for (i = 0; i < 24; i++) // Calculate CRC...
         {
            if ((Temp32 & 0x080000000) != 0) // XOR required...
            {
               Temp32 <<= 1;
               Temp32 ^= 0x01000000L; // CRC Polynomial X^8 + 1...
            }
            else
               Temp32 <<= 1;
         }

         if (Temp32 != 0L)
         {
            TmThresh_state = 0;
            return TM_CRC_ERROR;
         }
         else
         {
            if (u8_Scic_Rx_Data[3] == 0) // Read Command, display existing value
            {
               PrintUnsignedInteger((unsigned int)((LVAR(AX0_u32_Abs_Enc_Altern_1Data) >> 16) & 0x00FF));
               PrintString(" [deg C]", 0);
               PrintCrLf();
            }
            else                        // Write Command, verify correct value written
            {
               if ((unsigned int)((LVAR(AX0_u32_Abs_Enc_Altern_1Data) >> 16) & 0x00FF) != Parity)
               {
                  return TM_EEPROM_WRITE_FAILED;
               }
            }
         }
         TmThresh_state = 0;
      break;
   }

   if (TmThresh_state == 0) return SAL_SUCCESS;
   else return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: SalTmTempThreshExtended
// Description:
// This function called in response to TMTEMPTHRESHX command
// Logic:
//   A single Parameter to select Feedback Channel, '0' for Primary and '1' for Secondary Source.
//   Verify no Initialization, correct Feedback Type, and no Communication Errors.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalTmTempThreshExtended(int drive)
{
   static unsigned int  u16_enc_source;
   unsigned int u16_read_write, u16_thresh_value, ret_val = SAL_NOT_FINISHED;
   // AXIS_OFF;

   switch (BGVAR(s16_Comm_Fdbk_TM_Temp_Read))
   {
      case 1:
         if ( ((unsigned int)s64_Execution_Parameter[0] == 0) ||
              ((unsigned int)s64_Execution_Parameter[0] == 1)   )
//            u16_enc_source = (unsigned int)s64_Execution_Parameter[0];
            u16_enc_source = 0; // Temporary until Dual-Feedback Code is included.....
         else
            ret_val = VALUE_OUT_OF_RANGE;

         if (s16_Number_Of_Parameters == 2)
         {
            if ((unsigned int)s64_Execution_Parameter[1] < 10)
               ret_val = VALUE_TOO_LOW;
            else if ((unsigned int)s64_Execution_Parameter[1] > 120)
               ret_val = VALUE_TOO_HIGH;
         }

         if ( ( (u16_enc_source == 0) && (LVAR(AX0_s32_Feedback_Ptr) != &TM_COMMUNICATION_FEEDBACK) ) /* ||
              ( (u16_enc_source == 1) && (LVAR(AX0_s32_Feedback_Ptr_2) != &TM_COMMUNICATION_FEEDBACK) ) */ )
            ret_val = MENCTYPE_MISMATCH; // Use only with TM Encoder.

         if (BGVAR(s64_SysNotOk) & (TAMAGAWA_ABS_ENC_FLT_MASK | COMM_FDBK_FLT_MASK))
            ret_val = FDBKTYPE_COMM_ERR;

         if ( ( (u16_enc_source == 0) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) )    ||
              ( (u16_enc_source == 1) && (BGVAR(s16_Comm_Sec_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) )  )
            ret_val = FDBKTYPE_INIT_PRCSS;

         if (ret_val == SAL_NOT_FINISHED)
         {
            if (s16_Number_Of_Parameters > 1) // Writing Temp. Threshold
            {
               u16_read_write = 0x06; // Write to EEPROM Command
               u16_thresh_value = (unsigned int)s64_Execution_Parameter[1];
            }
            else                             // Reading existing threshold
            {
               u16_read_write = 0x0D; // Read EEPROM Command
               u16_thresh_value = 0;
            }
            BGVAR(s16_Comm_Fdbk_TM_Temp_Read) = 2;
         }
      break;

      case 2:
         ret_val = TmTempThreshExtended(drive, u16_enc_source, u16_read_write, u16_thresh_value, 0);
         if (ret_val != SAL_NOT_FINISHED)
            BGVAR(s16_Comm_Fdbk_TM_Temp_Read) = 1;
      break;
   }

   return ret_val;
}


//**********************************************************
// Function Name: TmTempThreshExtended
// Description:
// This function called from SalTmTempThreshExtended
// Logic:
//   a.  To read existing threshold, read TM Encoder EEPROM Address 0x4E by sending Control
//       Field 0x0D, Address 0x4E, and CRC.
//   b.  To set threshold, send desired value to TM Encoder EEPROM Address 0x4E by sending
//       Control Field 0x06, Address 0x4E, desired Temperature, and CRC.
//   c.  Verify (calculate) correct CRC of Response.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int TmTempThreshExtended(int drive, int enc_source, int read_write, int thresh_value, int fdbk_dest)
{
   // AXIS_OFF;
   FDBK_OFF;
   static unsigned int Parity, TmThresh_state = 0;
   unsigned long Temp32;
   unsigned int Temp_Calc, i, j;
   REFERENCE_TO_DRIVE;
   switch (TmThresh_state)
   {
      case 0: // Prepare Command Register and Send
         u8_Scic_Rx_Data[0] = read_write;
         u8_Scic_Rx_Data[1] = 0x4E;
         if (read_write == 0x06) // Writing Temp. Threshold
            u8_Scic_Rx_Data[2] = thresh_value;
         else                    // Reading existing threshold
            u8_Scic_Rx_Data[2] = 0;

         Temp_Calc = u8_Scic_Rx_Data[0];
         u8_Scic_Rx_Data[0] = ((Temp_Calc << 3) & 0x0FF) | 0x02; // Add Sync Code...
         Parity = 0;
         while (Temp_Calc != 0) // Calculate Data ID Parity...
         {
            Parity = !Parity;
            Temp_Calc &= (Temp_Calc - 1);
         }
         u8_Scic_Rx_Data[0] |= (Parity << 7); // ...and include ID Parity in Command Field

         Temp_Calc = (u8_Scic_Rx_Data[0] << 8) | u8_Scic_Rx_Data[1];
         for (i = 0; i < 8; i++) // Calculate CRC...
         {
            if ((Temp_Calc & 0x08000) != 0) // XOR required...
            {
               Temp_Calc <<= 1;
               Temp_Calc ^= 0x0100; // CRC Polynomial X^8 + 1...
            }
            else
               Temp_Calc <<= 1;
         }

         if (read_write == 0x0D) // Reading existing value...
         {
            u8_Scic_Rx_Data[2] = Temp_Calc >> 8; // CRC to send...
            u8_Scic_Rx_Data[3] = 0;
         }
         else // Writing new value...
         {
            Temp_Calc = Temp_Calc | u8_Scic_Rx_Data[2];
            for (i = 0; i < 8; i++) // Calculate Field CRC...
            {
               if ((Temp_Calc & 0x08000) != 0) // XOR required...
               {
                  Temp_Calc <<= 1;
                  Temp_Calc ^= 0x0100; // CRC Polynomial X^8 + 1...
               }
               else
                  Temp_Calc <<= 1;
            }
            u8_Scic_Rx_Data[3] = Temp_Calc >> 8;
         }

         Temp_Calc = 3 + (u8_Scic_Rx_Data[3] != 0); // Set Number of Fields to be sent...

         Parity = u8_Scic_Rx_Data[2]; // Maintain Input Value for later test...

         if (*(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) == 0)
         {
            for (j = 0; j < Temp_Calc; j++)
            {
               for (i = 0; i < 8; i++) // Flip Fields to send LSB first...
                  u8_Scic_Rx_Data[j] |= ((u8_Scic_Rx_Data[j] & (0x01 << i)) << (15 - (2 * i)));
               u8_Scic_Rx_Data[j] = (((u8_Scic_Rx_Data[j] >> 7) | 0x01) & 0x01FF);
               // Arrange with Leading 0, Terminating 1 (10-Bit Field)
            }
         }
         else
         {
            for (j = 0; j < Temp_Calc; j++)
            {  // Arrange with Leading 0, Terminating 1 (10-Bit Field)
               u8_Scic_Rx_Data[j] = (((u8_Scic_Rx_Data[j] << 1) | 0x200) & 0x03FF);
            }
         }

         if (*(unsigned int *)(FPGA_USE_TX_RX_BUFFER_REG_ADD + 0x0540 * enc_source) == 0)
         {  // Arrange 10-Bit Fields in 16-Bit Register to transmit in required order...
            FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_4Frame)) = 0x0020 | ((u8_Scic_Rx_Data[0] >> 5) & 0x01F); // Include '1' at beginning of Xmit
            FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_3Frame)) = ((u8_Scic_Rx_Data[0] << 11) & 0x0F800) | ((u8_Scic_Rx_Data[1] << 1) & 0x07FE) | ((u8_Scic_Rx_Data[2] >> 9) & 0x0001);
            FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_2Frame)) = ((u8_Scic_Rx_Data[2] << 7) & 0x0FF80) | ((u8_Scic_Rx_Data[3] >> 3) & 0x007F);
            FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_1Frame)) = ((u8_Scic_Rx_Data[3] << 13) & 0x0E000);
         }
         else
         {
            FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_1Frame)) = 0x0001 | ((u8_Scic_Rx_Data[0] << 1) & 0x07FE) | ((u8_Scic_Rx_Data[1] << 11) & 0x0F800);
            FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_2Frame)) = ((u8_Scic_Rx_Data[1] >> 5) & 0x0001F) | ((u8_Scic_Rx_Data[2] << 5) & 0x07FE0) | ((u8_Scic_Rx_Data[3] << 15) & 0x08000);
            FDBKVAR(VAR(AX0_u16_Com_Enc_Altern_3Frame)) = ((u8_Scic_Rx_Data[3] >> 1) & 0x001FF) | 0X0FE00;
         }

         FDBKVAR(VAR(AX0_u16_Com_Enc_Alt_Xmt_Length)) = (Temp_Calc * 10) + 1;
         FDBKVAR(VAR(AX0_u16_Com_Enc_Alt_Xcv_Length)) = 40;

         FDBKVAR(VAR(AX0_u16_Comm_Fdbk_Flags_A_1)) |= COMM_FDBK_ALTERN_CMD_SEND_MASK;
         // Set Serial Encoder Alternative Command bit

         TmThresh_state = 1;
      break;

      case 1: // Wait for response, then print
         if ((FDBKVAR(VAR(AX0_u16_Comm_Fdbk_Flags_A_1)) & (COMM_FDBK_ALTERN_CMD_SEND_MASK | COMM_FDBK_ALTERN_RESP_RCV_MASK)) != 0)
            return SAL_NOT_FINISHED; // Wait until command is sent and response is received...

         i = Cntr_3125;
         while (Cntr_3125 < (i + 2)) {}; // a short delay...

         // Using four Fields (all stored in AX0_u32_Abs_Enc_Altern_1Data) to verify CRC
         Temp32 = FDBKLVAR(LVAR(AX0_u32_Abs_Enc_Altern_1Data));
         for (i = 0; i < 24; i++) // Calculate CRC...
         {
            if ((Temp32 & 0x080000000) != 0) // XOR required...
            {
               Temp32 <<= 1;
               Temp32 ^= 0x01000000L; // CRC Polynomial X^8 + 1...
            }
            else
               Temp32 <<= 1;
         }

         if (Temp32 != 0L)
         {
            TmThresh_state = 0;
            return TM_CRC_ERROR;
         }
         else
         {
            if (u8_Scic_Rx_Data[3] == 0) // Read Command, display existing value
            {
               PrintUnsignedInteger((unsigned int)((FDBKLVAR(LVAR(AX0_u32_Abs_Enc_Altern_1Data)) >> 16) & 0x00FF));
               PrintString(" [deg C]", 0);
               PrintCrLf();
            }
            else                        // Write Command, verify correct value written
            {
               if ((unsigned int)((FDBKLVAR(LVAR(AX0_u32_Abs_Enc_Altern_1Data)) >> 16) & 0x00FF) != Parity)
               {
                  TmThresh_state = 0;
                  return TM_EEPROM_WRITE_FAILED;
               }
            }
         }
         TmThresh_state = 0;
      break;
   }

   if (TmThresh_state == 0) return SAL_SUCCESS;
   else return SAL_NOT_FINISHED;
}


// unsigned int sd_cmd = 0;
//**********************************************************
// Function Name: SalSdHackCommand
// Description:
//   This function called in response to SDHACK command
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalSdHackCommand(int drive)
{
   REFERENCE_TO_DRIVE;

//   sd_cmd = (unsigned int)s64_Execution_Parameter[0];
   *(unsigned int *)FPGA_MFB_ENCODER_SEL_REG_ADD = 0x20; // Nikon

   *(unsigned int *)FPGA_SANYO_DENKY_TX_REG1_ADD = (unsigned int)s64_Execution_Parameter[0]; // sd_cmd;
   *(unsigned int *)FPGA_SANYO_DENKY_TX_REG2_ADD = 0xFFFC;
   *(unsigned int *)FPGA_SANYO_DENKY_TX_REG3_ADD = 0x003F;

   *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 0;
   *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 1;

   return SAL_SUCCESS;
}


/*
unsigned long sd_cmd = 0;
void SanyoDenkyCommandHack()
{
   unsigned int data_lsb, data_msb, data, i, data_inv = 0, cc, com_cc;

   if (sd_cmd > 65536) return;

   if (*(unsigned int *)FPGA_RX_DATA_READY_REG_ADD & 0x0001)
   {
      data_msb = *(unsigned int *)(FPGA_SANYO_DENKY_RX_REG5_ADD) << 9;
      data_lsb = *(unsigned int *)(FPGA_SANYO_DENKY_RX_REG4_ADD) >> 7;
      data = data_msb | data_lsb;

      for (i = 0; i < 16; i++)
      {
         if (data & (1 << i)) data_inv = data_inv | (0x08000 >> i);
      }

      cc = (data_inv >> 5) & 0x001F;
      com_cc = (sd_cmd >> 5) & 0x001F;

      if ((cc == 0) ||  (cc == 0x03) || (cc == 0x08) || (cc == 0x0a))
      {
         if (cc == com_cc)
         {
            PrintUnsignedInteger(sd_cmd);
            PrintCrLf();
         }
      }
   }

   SetFeedback5Volt(); //; power up the 5V to the encoder
   *(unsigned int *)FPGA_MFB_ENCODER_SEL_REG_ADD = 0x20; // Nikon

   *(unsigned int *)FPGA_SANYO_DENKY_TX_REG1_ADD = sd_cmd;
   *(unsigned int *)FPGA_SANYO_DENKY_TX_REG2_ADD = 0xFFFC;
   *(unsigned int *)FPGA_SANYO_DENKY_TX_REG3_ADD = 0x003F;

   *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 0;
   *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 1;

   sd_cmd++;
} */



//**********************************************************
// Function Name: CommonFdbkIgnoreMTFaults
// Description:
//   This routine sets the IGNOREBATTFLT mode according
//    to the specified argument for feedback types:
//    SD_COMM_FDBK,
//    TAMAGAWA_COMM_MULTI_TURN_FDBK,
//    FANUC_COMM_FDBK,
//    PS_S_COMM_FDBK,
//    SANKYO_COMM_FDBK,
//    YASKAWA_ABS_COMM_FDBK,
//    YASKAWA_INC_COMM_FDBK.
//
// Note:
//    It is the responsibility of the caller to ensure the configured
//    feedback type is one of those in the list above.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************
int CommonFdbkIgnoreMTFaults(unsigned int u16_mode, int drive)
{
   if(u16_mode)// ignorance active
   {
      BGVAR(u64_Sys_Warnings) &= ~TM_BATT_LOW_VOLTAGE_WRN;
      BGVAR(s64_SysNotOk) &= ~ABS_ENC_BATT_LOW_MASK;
      BGVAR(s64_Faults_Mask) &= ~ABS_ENC_BATT_LOW_MASK;
   }
   else// ignorance inactive
   {
      BGVAR(s64_Faults_Mask) |= ABS_ENC_BATT_LOW_MASK;
   }
   drive += 0;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SrvSns_Init
// Description:
//   This routine initializes the ServoSense internal variables
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
void SrvSns_Init(int drive)
{
   // AXIS_OFF;

   VAR(AX0_u16_SrvSns_MBX_Header) = 0;   // Initialize mailbox header

   /* Initialize control variables */
   VAR(AX0_u16_SrvSns_Status_Field)   = 0;
   LVAR(AX0_u32_SrvSns_PacketPtr)     = (unsigned long)&VAR(AX0_u32_SrvSns_RequestFrameArray[0]);
   LVAR(AX0_u32_SrvSns_SubModeDataPtr)= (unsigned long)&VAR(AX0_s32_Comm_Abs_Pos_32_Hi);
   VAR(AX0_u16_SrvSns_PacketCntr)     = 0;
   VAR(AX0_u16_SrvSns_ReqPacketCntr)  = 0;
   VAR(AX0_u16_SrvSns_FWStatus)       &= SRVSNS_FW_STATUS_PWR_RESET_REQ_MASK;
   BGVAR(u32_SrvSns_ConsecFaultCtr)   = 0UL;
   BGVAR(u32_SrvSns_FaultsIndications)= SRVSNS_FAULTS_INIT;
   BGVAR(u32_SrvSns_WarnsIndications) = SRVSNS_WRNS_INDICATIONS;
   BGVAR(u16_SrvSns_InitProcErrorCode)    = 0;
   BGVAR(u16_SrvSns_InitProcRetriesCntr)  = 0;
   BGVAR(u32_SrvSns_BusyTimeOutCtr)   = 0;
   BGVAR(u32_SrvSns_Versions) = (unsigned long)(-1);

   /* Initialize mailbox */
   SrvSns_SetCommMode((SRVSNS_COMM_MODE_OPERATIONAL | SRVSNS_COMM_SUBMODE_MULTITURN), drive);/* Set mode operational multi-turn */
   VAR(AX0_u16_SrvSns_MBX_AddrBuffer) = 0;
   LVAR(AX0_u32_SrvSns_MBX_ValBuffer) = 0UL;
   BGVAR(u16_SrvSns_ReqStatus)        = 0;

   SrvSns_InitStateMachines(drive);   // Initialize state machines

   SRVSNS_RTMBX_POST_INIT_STATE(VAR(AX0_u16_SrvSns_MBX_Header));  // Set init state

   /* Initialize the semaphore */
   BGVAR(u32_SrvSns_Semaphore)        = (unsigned long)SRVSNS_SEMAPHORE_FREE;
   BGVAR(u32_SrvSns_OwnerCounter)     = 0UL;
   BGVAR(s32_SrvSns_ReleaseTimer)     = 0L;

   BGVAR(u16_SrvSns_HelpTaskState) = 0;   // Init help task state
}


//**********************************************************
// Function Name: SrvSns_InitStateMachines
// Description:
//   This routine initializes the ServoSense state machines of the driver
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
void SrvSns_InitStateMachines(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   /* Initialize state machines */
   BGVAR(u16_SrvSns_ManagerState) = SRVSNS_MANAGER_STATE_SERV_REQUEST;
   BGVAR(u16_SrvSns_ServiceState) = SRVSNS_SERVICE_STATE_REQUEST;
   BGVAR(u16_SrvSns_ReadState)    = SRVSNS_READ_STATE_LOAD_ADDR;
   BGVAR(u16_SrvSns_WaitState)    = SRVSNS_WAIT_STATE_IDLE;
}


//**********************************************************
// Function Name: SrvSns_GetStatus
// Description:
//   This routine returns the status field of the ServoSense Feedback:
//              Bit#:       Description
//              0           Fault
//              1           Warning
//              2           Busy
//              3           Request error
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
int SrvSns_GetStatus(int drive)
{
   unsigned int u16_status;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   u16_status = VAR(AX0_u16_SrvSns_Status_Field);   // Capture status

   if (u16_status & SRVSNS_STATUS_FAULT_MASK)
   {
      if (BGVAR(u32_SrvSns_ConsecFaultCtr) < 2)
      {
         BGVAR(u32_SrvSns_ConsecFaultCtr)++;
         u16_status &= ~SRVSNS_STATUS_FAULT_MASK;
      }
   }
   else
   {
      BGVAR(u32_SrvSns_ConsecFaultCtr) = 0;
   }
   return (u16_status);
}


//**********************************************************
// Function Name: SrvSns_Manager
// Description:
//   This function manages the ServoSense driver
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:     This function has to be called on each background
//**********************************************************
void SrvSns_Manager(int drive)
{

   /* Do not continue if ServoSense is not in use or not ready */
   if (!FEEDBACK_SERVOSENSE)
      return;

   BGVAR(u32_SrvSns_OwnerCounter)++; // increment the owner counter

   /* If ServoSense is acquired, check for release timeout */
   if (BGVAR(u32_SrvSns_Semaphore) != (unsigned long)SRVSNS_SEMAPHORE_FREE)
   {
       if(PassedTimeMS(SRVSNS_RELEASE_TIMEOUT_mSEC, BGVAR(s32_SrvSns_ReleaseTimer)))
       {
            BGVAR(u32_SrvSns_ReleaseTimeOutCtr)++;
            // Release ServoSense
            BGVAR(u32_SrvSns_Semaphore) = (unsigned long)SRVSNS_SEMAPHORE_FREE;

            /* Initialize state machines */
            SrvSns_InitStateMachines(drive);
            return;
       }
   }
}


//**********************************************************
// Function Name: SrvSns_SendServiceRequest
// Description:
//   This routine sends a service request to ServoSense according to the specified command.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
void SrvSns_SendServiceRequest(int drive, unsigned int u16_command, int u16_fdbk_source)
{
   static unsigned char u8_req_data_array[3] = {0, 0, 0};
   unsigned int u16_temp1 = 0, u16_temp2 = 0;
   int i = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   // Ensure the response ready flag is cleared
   SRVSNS_RTMBX_CLEAR_RSP_RDY(VAR(AX0_u16_SrvSns_MBX_Header));
   // Ensure the protocol error bit is cleared
   SRVSNS_RTMBX_CLEAR_PRTCL_ERR(VAR(AX0_u16_SrvSns_MBX_Header));

   // Ensure the BG response ready flag is cleared
   BGVAR(u16_SrvSns_ReqStatus) &= ~SRVSNS_SAL_RSP_READY_MASK;

   // Latch the command
   BGVAR(u16_SrvSns_Cmd) = u16_command;

   if(SrvSns_Commands[u16_command].u16_cmd_id != SRVSNS_CMD_ID_NO_CMD)
   {
      // parse request, serialize the data, post mailbox
      if(u16_command != SRVSNS_CMD_LOAD_ADDR)
      {
         if(SrvSns_Commands[u16_command].u16_password)
         {
            VAR(AX0_u16_SrvSns_PacketCntr) = SrvSns_Commands[u16_command].u16_password & 0x000F;
            BGVAR(u16_SrvSns_Address)     = (SrvSns_Commands[u16_command].u16_password & 0x0010) >> 4;
            BGVAR(u32_SrvSns_Value)     = (SrvSns_Commands[u16_command].u16_password & 0x0060) >> 5;
         }
         else
         {
            switch(u16_command)
            {
               case SRVSNS_CMD_GET_TEMP_HIST_BAR:
               case SRVSNS_CMD_GET_VEL_HIST_BAR:
                  BGVAR(u16_SrvSns_Address) = SrvSns_Commands[u16_command].u16_address + BGVAR(u32_SrvSns_Value) * SRVSNS_ADDRESS_ALIGNMENT;
                  BGVAR(u32_SrvSns_Value) = 0;
               break;

               default:
                  BGVAR(u16_SrvSns_Address) = SrvSns_Commands[u16_command].u16_address;
               break;
            }
         }
      }
      // Fill the request frame buffer
      for(i = SrvSns_Commands[u16_command].u16_packet_ctr; i > 0; i--)
      {
         // Build the request packet
         u8_req_data_array[0] = SRVSNS_REQ_ID | (SrvSns_Commands[u16_command].u16_cmd_id << 3) |
         ((VAR(AX0_u16_SrvSns_MBX_Header) & SRVSNS_RTMBX_HDR_MODE_BITS_MASK) << 6);

         if(SrvSns_Commands[u16_command].u16_password)
         {
            u16_temp1 = VAR(AX0_u16_SrvSns_PacketCntr);
            u16_temp2 = 0;
         }
         else
         {
            u16_temp1 = (i - 1);
            u16_temp2 = (i - 1);
         }

         u8_req_data_array[1] = ((BGVAR(u16_SrvSns_ReqStatus) & SRVSNS_SAL_REQ_RW_BIT_MASK) >> 1)  |
                                 /* Reserved bit --> zero */
                                 (u16_temp1 << 2)                                 |
                                 (((BGVAR(u16_SrvSns_Address) >> u16_temp2) & 0x01) << 6)          |
                                 (((BGVAR(u32_SrvSns_Value) >> (u16_temp2 << 1)) & 0x01) << 7);

         u8_req_data_array[2] = ((BGVAR(u32_SrvSns_Value) >> (u16_temp2 << 1)) & 0x02) >> 1;

         // Calculate CRC
         u16_temp1 = 0;
         u16_temp1 = COMM_FDBK_GET_CRC(u16_temp1, u8_req_data_array[0], u16_fdbk_source);
         u16_temp1 = COMM_FDBK_GET_CRC(u16_temp1, u8_req_data_array[1], u16_fdbk_source);

         // Handling of the last request bit for CRC
         u16_temp1 = (u16_temp1 << 1) | (u8_req_data_array[2] & 0x01);
         u16_temp1 ^= 0x09 * (u16_temp1 >> 7);
         u16_temp1 &= 0x7F;

         // Finish the CRC
         u8_req_data_array[2] |= (u16_temp1 << 1);

         // Build the UART frame array
         LVAR(AX0_u32_SrvSns_RequestFrameArray[SrvSns_Commands[u16_command].u16_packet_ctr - i]) = \
                                                               /* Start bit 0 */
                                                               (((unsigned long)u8_req_data_array[0]) << 1)  |
                                                               /* Stop bit 0*/
                                                               /* Start bit 1 */
                                                               (((unsigned long)u8_req_data_array[1]) << 11) |
                                                               /* Stop bit 1*/
                                                               /* Start bit 2 */
                                                               (((unsigned long)u8_req_data_array[2]) << 21);
                                                               /* 2 Stop bits 2*/

         LVAR(AX0_u32_SrvSns_RequestFrameArray[SrvSns_Commands[u16_command].u16_packet_ctr - i]) |=
                                                SRVSNS_UART_FRAME_MASK_32BIT;
      }
      // Load the packet counter
      VAR(AX0_u16_SrvSns_PacketCntr) = SrvSns_Commands[u16_command].u16_packet_ctr;

      // Load the packets limit for the current request
      VAR(AX0_u16_SrvSns_ReqPacketCntr) = VAR(AX0_u16_SrvSns_PacketCntr);

      u8_req_data_array[0] = 0;
      u8_req_data_array[1] = 0;
      u8_req_data_array[2] = 0;

      // Send the BG request to RT
      SRVSNS_RTMBX_POST_REQ_RDY(VAR(AX0_u16_SrvSns_MBX_Header));
   }
   else
   {
      switch (u16_command)
      {
         case SRVSNS_CMD_SET_MODE_MULTI_TURN:
            // set operational multi-turn mode
            SrvSns_SetCommMode((SRVSNS_COMM_MODE_OPERATIONAL | SRVSNS_COMM_SUBMODE_MULTITURN), drive);
         break;

         case SRVSNS_CMD_SET_MODE_VELOCITY:
            // set operational velocity mode
            SrvSns_SetCommMode((SRVSNS_COMM_MODE_OPERATIONAL | SRVSNS_COMM_SUBMODE_VELOCITY), drive);
         break;

         case SRVSNS_CMD_GET_SET_COMM:
         case SRVSNS_CMD_CALIB_SYNC:
         case SRVSNS_CMD_PWR_CNTRL:
            if ((BGVAR(u16_SrvSns_ReqStatus) & SRVSNS_SAL_REQ_RW_BIT_MASK) == 0) //  read
            {
               if (u16_command == SRVSNS_CMD_GET_SET_COMM)
               {
                  BGVAR(u32_SrvSns_Value) = (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) > 0;
               }
               else if(u16_command == SRVSNS_CMD_CALIB_SYNC)
               {
                  BGVAR(u32_SrvSns_Value) = *(unsigned int*)0x4049;// Special FPGA for ServoSense calibration
               }
               else if(u16_command == SRVSNS_CMD_PWR_CNTRL)
               {
                  BGVAR(u32_SrvSns_Value) = GetFeedback5VoltStatus();
               }
               break;
            }

            // write
            if (BGVAR(u32_SrvSns_Value) > 1)
            {
               BGVAR(u16_SrvSns_ReqStatus) |= SRVSNS_SAL_ERROR_BIT_MASK;
               BGVAR(u16_SrvSns_ReqStatus) &= ~SRVSNS_SAL_ERROR_WORD_MASK;
               BGVAR(u16_SrvSns_ReqStatus) |= ((unsigned int)SRVSNS_ERR_VAL_OUT_OF_RANGE) << SRVSNS_SAL_ERROR_WORD_SHIFT;
               break;
            }

            if (BGVAR(u32_SrvSns_Value) == 0)   // OFF
            {
               if (u16_command == SRVSNS_CMD_GET_SET_COMM)
               {
                  // release RT communication
                  VAR(AX0_u16_Time_Out_Consec_Err_Cntr) = 0;
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~(COMM_FDBK_READY_MASK);
               }
               else if(u16_command == SRVSNS_CMD_CALIB_SYNC)
               {
                  if (*(unsigned int*)FPGA_EMULATED_ENCODER_MODE_ADD != 0)
                  {
                     *(unsigned int*)FPGA_EMULATED_ENCODER_MODE_ADD = 0;
                  }
                  *(unsigned int*)0x4049 = 0;// Special FPGA for ServoSense calibration
               }
               else if(u16_command == SRVSNS_CMD_PWR_CNTRL)
               {
                  RemoveFeedback5Volt();//encoder power off
               }
            }
            else if (BGVAR(u32_SrvSns_Value) == 1)   // ON
            {
               if (u16_command == SRVSNS_CMD_GET_SET_COMM)
               {
                  // set RT communication
                  VAR(AX0_u16_Time_Out_Consec_Err_Cntr) = 0;
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= (COMM_FDBK_READY_MASK);
               }
               else if(u16_command == SRVSNS_CMD_CALIB_SYNC)
               {
                  *(unsigned int*)0x4049 = 1;// Special FPGA for ServoSense calibration
               }
               else if(u16_command == SRVSNS_CMD_PWR_CNTRL)
               {
                  SetFeedback5Volt();//encoder power on
               }
            }
            BGVAR(u16_SrvSns_ReqStatus) &= ~SRVSNS_SAL_REQ_RW_BIT_MASK;
         break;

         default:
         break;
      }
      BGVAR(u16_SrvSns_ReqStatus) |= SRVSNS_SAL_RSP_READY_MASK;
   }
   // Take the time for timeout
   BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
}


//**********************************************************
// Function Name: SrvSns_SendServiceRequest
// Description:
//   This routine returns a ServoSense response to the recently sent service request.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
int SrvSns_GetServiceResponse(int drive, long* p_s32_response, int u16_fdbk_source)
{
   int ret_val = SAL_NOT_FINISHED;
   static int error_code = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if(p_s32_response == NULL)
   {
      BGVAR(u16_SrvSns_WaitState) = SRVSNS_WAIT_STATE_IDLE;
      return SRVSNS_DRV_ERROR;
   }

   switch(BGVAR(u16_SrvSns_WaitState))
   {
      case SRVSNS_WAIT_STATE_IDLE:
         // Check response ready from RT
         if(   (SRVSNS_RTMBX_IS_RSP_RDY(VAR(AX0_u16_SrvSns_MBX_Header)))   ||
               (SrvSns_Commands[BGVAR(u16_SrvSns_Cmd)].u16_cmd_id == SRVSNS_CMD_ID_RESET_FW_DNLD))   // reset requested
         {
            SRVSNS_RTMBX_CLEAR_RSP_RDY(VAR(AX0_u16_SrvSns_MBX_Header));

            if(SRVSNS_RTMBX_IS_PRTCL_ERR(VAR(AX0_u16_SrvSns_MBX_Header)))// protocol error reported from RT
            {
               // Acknowledge
               SRVSNS_RTMBX_CLEAR_PRTCL_ERR(VAR(AX0_u16_SrvSns_MBX_Header));
               if (SrvSns_Commands[BGVAR(u16_SrvSns_Cmd)].u16_cmd_id != SRVSNS_CMD_ID_RESET_FW_DNLD)
                  ret_val = SrvSns_GetError(SRVSNS_ERR_PRTCL_ERR);
            }
            else if(SRVSNS_RTMBX_IS_ERR_SET(VAR(AX0_u16_SrvSns_MBX_Header)))// service request error reported from RT
            {
               ret_val = SrvSns_GetError(SRVSNS_RTMBX_GET_ERR_WORD(VAR(AX0_u16_SrvSns_MBX_Header)));

               // Acknowledge
               SRVSNS_RTMBX_CLEAR_ERR_BITS(VAR(AX0_u16_SrvSns_MBX_Header));

               SrvSns_SendServiceRequest(drive, SRVSNS_CMD_CLEAR_ERROR, u16_fdbk_source);
               error_code = ret_val;
               ret_val = SAL_NOT_FINISHED;
               break;
            }
            else// no error
            {
               // Validate the resulted data
               if (SrvSns_Commands[BGVAR(u16_SrvSns_Cmd)].u16_cmd_id == SRVSNS_CMD_LOAD_ADDR)
               {
                  if (VAR(AX0_u16_SrvSns_MBX_AddrBuffer) != BGVAR(u16_SrvSns_Address))
                     ret_val = SrvSns_GetError(SRVSNS_ERR_PRTCL_ERR);

                  if (BGVAR(u16_SrvSns_ReqStatus) & SRVSNS_SAL_REQ_RW_BIT_MASK)
                  {
                     if(LVAR(AX0_u32_SrvSns_MBX_ValBuffer) != BGVAR(u32_SrvSns_Value))
                        ret_val = SrvSns_GetError(SRVSNS_ERR_PRTCL_ERR);
                  }
               }
            }

            // Load the resulted values
            BGVAR(u32_SrvSns_Value)             = LVAR(AX0_u32_SrvSns_MBX_ValBuffer);
            LVAR(AX0_u32_SrvSns_MBX_ValBuffer)  = 0;
            BGVAR(u16_SrvSns_Address)           = VAR(AX0_u16_SrvSns_MBX_AddrBuffer);
            VAR(AX0_u16_SrvSns_MBX_AddrBuffer)  = 0;

            if(error_code)
            {
               ret_val = error_code;
               error_code = 0;
            }
            else if(ret_val == SAL_NOT_FINISHED)
            {
               *p_s32_response = (long)BGVAR(u32_SrvSns_Value);
               ret_val = SAL_SUCCESS;
            }
            SrvSns_InitBGVars(drive);
         }
         else if(BGVAR(u16_SrvSns_ReqStatus) & SRVSNS_SAL_RSP_READY_MASK)// Check response ready from BG
         {
            //acknowledge
            BGVAR(u16_SrvSns_ReqStatus) &= ~SRVSNS_SAL_RSP_READY_MASK;

            if(BGVAR(u16_SrvSns_ReqStatus) & SRVSNS_SAL_ERROR_BIT_MASK)
            {
               // convert the error word to SAL
               ret_val = SrvSns_GetError((BGVAR(u16_SrvSns_ReqStatus) & SRVSNS_SAL_ERROR_WORD_MASK) >> SRVSNS_SAL_ERROR_WORD_SHIFT);
            }
            else
            {
               *p_s32_response = (long)BGVAR(u32_SrvSns_Value);
               ret_val = SAL_SUCCESS;
            }
            SrvSns_InitBGVars(drive);
         }
         else if(PassedTimeMS(SRVSNS_SERV_REQ_TIMEOUT_mSEC, BGVAR(s32_Comm_Fdbk_Timer)))// Handle busy timeout
         {
            // Cleanup
            LVAR(AX0_u32_SrvSns_MBX_ValBuffer)  = 0;
            VAR(AX0_u16_SrvSns_MBX_AddrBuffer)  = 0;
            SrvSns_InitBGVars(drive);
            BGVAR(u32_SrvSns_BusyTimeOutCtr)++;
            ret_val = SRVSNS_BUSY_TIMEOUT;
         }
      break;

      case SRVSNS_WAIT_STATE_ERROR_HANDLE:
      break;

      default:
         BGVAR(u16_SrvSns_WaitState) = SRVSNS_WAIT_STATE_IDLE;
         ret_val = SRVSNS_DRV_ERROR;
      break;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SrvSns_GetDataSwitch
// Description:
//    This routine performs the switch from LOAD_ADDR to GET_DATA commands.
//    Dedicated for read service requests.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
int SrvSns_GetDataSwitch(int drive, int u16_fdbk_source)
{
   int ret_val;
   REFERENCE_TO_DRIVE;

   ret_val = (BGVAR(u16_SrvSns_ReadState) == SRVSNS_READ_STATE_GET_DATA) ? SAL_SUCCESS : SAL_NOT_FINISHED;
   BGVAR(u16_SrvSns_ReadState) ^= SRVSNS_READ_STATE_GET_DATA;

   if(ret_val != SAL_SUCCESS)
      SrvSns_SendServiceRequest(drive, SRVSNS_CMD_GET_DATA, u16_fdbk_source);

   return ret_val;
}


//**********************************************************
// Function Name: SrvSns_InitBGVars
// Description:
//   This routine initializes the BG variables related to service request.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
void SrvSns_InitBGVars(int drive)
{
   REFERENCE_TO_DRIVE;
   // initialize vars
   BGVAR(u16_SrvSns_Address)   = 0;
   BGVAR(u32_SrvSns_Value)     = 0;
   BGVAR(u16_SrvSns_Cmd)       = 0;
   BGVAR(u16_SrvSns_ReqStatus) &= ~(SRVSNS_SAL_REQ_RW_BIT_MASK);
}


//**********************************************************
// Function Name: SrvSns_GetError
// Description:
//   This converts ServoSense error to SAL error code.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
int SrvSns_GetError(int error)
{
   switch(error)
   {
      case SRVSNS_ERR_ADDR_OUT_OF_RANGE: return(SRVSNS_ADDR_OUT_OF_RANGE);
      case SRVSNS_ERR_VAL_OUT_OF_RANGE:  return(VALUE_OUT_OF_RANGE);
      case SRVSNS_ERR_NOT_PRGRMBLE:      return(NOT_PROGRAMMABLE);
      case SRVSNS_ERR_OCCUPIED:          return(SRVSNS_BUSY);
      case SRVSNS_ERR_ILLEGAL_REQUEST:   return(SRVSNS_ILLEGAL_REQUEST);
      case SRVSNS_ERR_EE_SAVE_FAILED:    return(SRVSNS_SAVE_FAILED);
      case SRVSNS_ERR_NOT_AVAILABLE:     return(NOT_AVAILABLE);
      case SRVSNS_ERR_UNKNOWN_REQUEST:   return(UNKNOWN_COMMAND);
      case SRVSNS_ERR_ADDR_NOT_ALIGNED:  return(SRVSNS_NOT_ALIGNED);
      case SRVSNS_ERR_BUSY_TIMEOUT:      return(SRVSNS_BUSY_TIMEOUT);
      case SRVSNS_ERR_INTERNAL_ERR:      return(SRVSNS_INTERNAL_REQ_ERR);
      case SRVSNS_ERR_CRC_ERROR:         return(SRVSNS_CRC_ERROR);
      case SRVSNS_ERR_PRTCL_ERR:         return(SRVSNS_PROTOCOL_ERROR);      // must be last
      default:                           return(SRVSNS_INTERNAL_REQ_ERR);
   }
}


//***********************************************************************
// Function Name: SrvSns_Acquire
// Description:
//  This routine acquires the ServoSense Encoder device for the owner
//  and returns the owner ID.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:   It is the responsibility of the caller to acquire the ServoSense device
//          before calling other driver functions.
//************************************************************************
int SrvSns_Acquire(unsigned long* p_owner_id, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(p_owner_id == NULL)
      return SRVSNS_DRV_ERROR;

   // wait till the semaphore is free
   if(BGVAR(u32_SrvSns_Semaphore) != (unsigned long)SRVSNS_SEMAPHORE_FREE)
      return SAL_NOT_FINISHED;

   // prevent owning the FREE ID
   if(BGVAR(u32_SrvSns_OwnerCounter) == (unsigned long)SRVSNS_SEMAPHORE_FREE)
      return SAL_NOT_FINISHED;

   // take the owner
   *p_owner_id                    = BGVAR(u32_SrvSns_OwnerCounter);
   BGVAR(u32_SrvSns_Semaphore)    = BGVAR(u32_SrvSns_OwnerCounter);
   BGVAR(s32_SrvSns_ReleaseTimer) = Cntr_1mS;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SrvSns_Release
// Description:
//   This routine releases the ServoSense Encoder device from the
//    provided owner.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notest:   This routine releases the owner as well.
//***********************************************************
int SrvSns_Release(unsigned long* p_owner_id, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (p_owner_id == NULL)
      return SRVSNS_DRV_ERROR;

   if (*p_owner_id != BGVAR(u32_SrvSns_Semaphore))
      return SRVSNS_DRV_OCCUPIED;

   BGVAR(u32_SrvSns_Semaphore) = (unsigned long)SRVSNS_SEMAPHORE_FREE;
   *p_owner_id = (unsigned long)SRVSNS_SEMAPHORE_FREE;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SrvSnsReadAddr
// Description:
//  This function reads a value from ServoSense Encoder
//  according to the specified address.
//  The value under *p_s32_response will contain the read value
//  if the execution is finished successfully.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:        It is the responsiblity of the caller to acquire the
//              ServoSense device and release it after the operation is complete.
//***********************************************************
int SrvSnsReadAddr(unsigned long u32_owner_id, unsigned int u16_address, long* p_s32_response, int drive, int u16_fdbk_source)
{
   // AXIS_OFF;
   int ret_val = SAL_NOT_FINISHED;

   if(VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PWR_OFF_MASK)
   {
      return NOT_AVAILABLE;
   }

   /* Do not continue if ServoSense is not in use or not ready */
   if (!FEEDBACK_SERVOSENSE)
   {
      return NOT_AVAILABLE;
   }

   if (p_s32_response == NULL)
   {
      BGVAR(u16_SrvSns_ServiceState) = SRVSNS_SERVICE_STATE_REQUEST;
      BGVAR(u16_SrvSns_ReadState) = SRVSNS_READ_STATE_LOAD_ADDR;
      return SRVSNS_DRV_ERROR;
   }

   if ((BGVAR(u32_SrvSns_Semaphore) != (unsigned long)SRVSNS_SEMAPHORE_FREE) && (BGVAR(u32_SrvSns_Semaphore) != u32_owner_id))
      return SAL_NOT_FINISHED;

   if (u32_owner_id == (unsigned long)SRVSNS_SEMAPHORE_FREE)
      return SRVSNS_DRV_OCCUPIED;

   switch(BGVAR(u16_SrvSns_ServiceState))
   {
      case SRVSNS_SERVICE_STATE_REQUEST:
         BGVAR(u16_SrvSns_Address)    = (unsigned int)u16_address;
         BGVAR(u32_SrvSns_Value)      = 0;
         BGVAR(u16_SrvSns_ReqStatus)  &= ~SRVSNS_SAL_REQ_RW_BIT_MASK;
         SrvSns_SendServiceRequest(drive, SRVSNS_CMD_LOAD_ADDR, u16_fdbk_source);
         BGVAR(u16_SrvSns_ServiceState) = SRVSNS_SERVICE_STATE_RESPONSE;
      break;

      case SRVSNS_SERVICE_STATE_RESPONSE:// Wait for response from SrvSns manager
         ret_val = SrvSns_GetServiceResponse(drive, p_s32_response, u16_fdbk_source);
         if(ret_val == SAL_SUCCESS)
            ret_val = SrvSns_GetDataSwitch(drive, u16_fdbk_source);
      break;

      default:
         ret_val = SRVSNS_DRV_ERROR;
      break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {
      BGVAR(u16_SrvSns_ServiceState) = SRVSNS_SERVICE_STATE_REQUEST;
      BGVAR(u16_SrvSns_ReadState) = SRVSNS_READ_STATE_LOAD_ADDR;
   }
   return ret_val;
}


//**********************************************************
// Function Name: SrvSnsWriteAddr
// Description:
//   This function writes a value to ServoSense Encoder
//   according to the specified address.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:      It is the responsiblity of the caller to acquire the
//            ServoSense device and release it after the operation is complete.
//***********************************************************
int SrvSnsWriteAddr(unsigned long u32_owner_id, unsigned int u16_address, unsigned long u32_value, int drive, int u16_fdbk_source)
{
   // AXIS_OFF;
   int ret_val = SAL_NOT_FINISHED;

   if(VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PWR_OFF_MASK)
   {
      return NOT_AVAILABLE;
   }

   /* Do not continue if ServoSense is not in use or not ready */
   if (!FEEDBACK_SERVOSENSE)
   {
      return NOT_AVAILABLE;
   }

   if ((BGVAR(u32_SrvSns_Semaphore) != (unsigned long)SRVSNS_SEMAPHORE_FREE) && (BGVAR(u32_SrvSns_Semaphore) != u32_owner_id))
       return SAL_NOT_FINISHED;

   if (u32_owner_id == (unsigned long)SRVSNS_SEMAPHORE_FREE)
       return SRVSNS_DRV_OCCUPIED;

   switch(BGVAR(u16_SrvSns_ServiceState))
   {
      case SRVSNS_SERVICE_STATE_REQUEST:// Fill the data for the SrvSns manager and set ready flag
         BGVAR(u16_SrvSns_Address)       = u16_address;
         BGVAR(u32_SrvSns_Value)         = u32_value;
         BGVAR(u16_SrvSns_ReqStatus)     |= SRVSNS_SAL_REQ_RW_BIT_MASK;
         BGVAR(u16_SrvSns_ServiceState)  = SRVSNS_SERVICE_STATE_RESPONSE;
         SrvSns_SendServiceRequest(drive, SRVSNS_CMD_LOAD_ADDR, u16_fdbk_source);
      break;

      case SRVSNS_SERVICE_STATE_RESPONSE:// Wait for status response from SrvSns manager
         ret_val = SrvSns_GetServiceResponse(drive, (long*)&u32_value, u16_fdbk_source);
      break;

      default:
         ret_val = SRVSNS_DRV_ERROR;
      break;
   }

   if(ret_val != SAL_NOT_FINISHED)
   {
      BGVAR(u16_SrvSns_ServiceState) = SRVSNS_SERVICE_STATE_REQUEST;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SrvSnsSendCmd
// Description:
//   This function sends a command to ServoSense Encoder.
//   *p_s32_response will contain a corresponding numeric response
//   when the execution is finished successfully.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:        It is the responsiblity of the caller to acquire the
//              ServoSense device and release it after the operation is complete.
//************************************************************
int SrvSnsSendCmd(unsigned long u32_owner_id, unsigned int u16_cmd, unsigned long u32_argument, long* p_s32_response, int drive, int u16_fdbk_source)
{
   int ret_val = SAL_NOT_FINISHED;
   static unsigned int u16_is_write = 0, u16_temp = 0;
   // AXIS_OFF;

   if(VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PWR_OFF_MASK)
   {
      return NOT_AVAILABLE;
   }

   if (!FEEDBACK_SERVOSENSE)
   {
      return NOT_AVAILABLE;
   }

   if (p_s32_response == NULL)
   {
      BGVAR(u16_SrvSns_ServiceState) = SRVSNS_SERVICE_STATE_REQUEST;
      BGVAR(u16_SrvSns_ReadState) = SRVSNS_READ_STATE_LOAD_ADDR;
      u16_is_write = 0;
      return SRVSNS_DRV_ERROR;
   }

   if ((BGVAR(u32_SrvSns_Semaphore) != (unsigned long)SRVSNS_SEMAPHORE_FREE) && (BGVAR(u32_SrvSns_Semaphore) != u32_owner_id))
      return SAL_NOT_FINISHED;

   if (u32_owner_id == (unsigned long)SRVSNS_SEMAPHORE_FREE)
      return SRVSNS_DRV_OCCUPIED;

   u16_is_write = (u16_cmd & SRVSNS_CMD_WRITE_MASK) > 0;   // Determine if read or write
   u16_cmd &= ~SRVSNS_CMD_WRITE_MASK;

   if(u16_cmd > SRVSNS_INTERFACE_NUM_OF_CMDS)
      return UNKNOWN_COMMAND;

   switch(BGVAR(u16_SrvSns_ServiceState))
   {
      case SRVSNS_SERVICE_STATE_REQUEST:// Fill the command specific data and raise ready flag for SrvSns manager
         switch(u16_cmd)
         {
            case SRVSNS_CMD_CLEAR_ERROR:
            case SRVSNS_CMD_GET_DATA:
            case SRVSNS_CMD_INCR_ADDR:
            case SRVSNS_CMD_RESET:
            case SRVSNS_CMD_FW_DNLD:
            break;

            case SRVSNS_CMD_GET_FAULTS:
            case SRVSNS_CMD_GET_WARNINGS:
            case SRVSNS_CMD_HALLS_SYNC_REC_DONE:
            case SRVSNS_CMD_ZEROPOSST:
            case SRVSNS_CMD_EE_ACCESS_ST:
            case SRVSNS_CMD_GETRUNTIME:
            case SRVSNS_CMD_GET_TEMP_HIST_BAR:
            case SRVSNS_CMD_GET_VEL_HIST_BAR:
               switch (u16_cmd)
               {
                  case SRVSNS_CMD_EE_ACCESS_ST:
                     BGVAR(u32_SrvSns_Value) = u32_argument;
                  break;

                  case SRVSNS_CMD_GET_TEMP_HIST_BAR:
                     if(u32_argument > SRVSNS_TEMP_HIST_NUMOFBARS)
                     {
                        ret_val = VALUE_TOO_HIGH;
                        break;
                     }
                     BGVAR(u32_SrvSns_Value) = u32_argument - 1;
                  break;

                  case SRVSNS_CMD_GET_VEL_HIST_BAR:
                     if(u32_argument > SRVSNS_VEL_HIST_NUMOFBARS)
                     {
                        ret_val = VALUE_TOO_HIGH;
                        break;
                     }
                     BGVAR(u32_SrvSns_Value) = u32_argument - 1;
                  break;

                  default:
                  break;
               }
            break;

            case SRVSNS_CMD_MT_CONFIG:
               if(!Enabled(drive))
               {
                  // load the required payload
                  BGVAR(u32_SrvSns_Value) = 1;
               }
               else
                  ret_val = DRIVE_ACTIVE;
            break;

            case SRVSNS_CMD_CLEAR_FAULTS:
            case SRVSNS_CMD_SAVE_EE:
            case SRVSNS_CMD_GET_LTCH_FLT_RECS:
            case SRVSNS_CMD_SAVEINTPARAMS:
            case SRVSNS_CMD_ACK_BATT_DN_FLT:
               // load the required payload
               BGVAR(u32_SrvSns_Value) = 1;
            break;

            case SRVSNS_CMD_SET_MODE_MULTI_TURN:
               // change the mode request
               BGVAR(u16_SrvSns_ReqStatus) &= ~SRVSNS_SAL_SET_MODE_MASK;
            break;

            case SRVSNS_CMD_SET_MODE_VELOCITY:
               // change the mode request
               BGVAR(u16_SrvSns_ReqStatus) |= SRVSNS_SAL_SET_MODE_MASK;
            break;

            case SRVSNS_CMD_EE_WRITE_ENABLE:
               // load the password
               BGVAR(u32_SrvSns_Value) = SRVSNS_ADDR_CMD_EN_DIS_EE_WR_EN | ((unsigned int)u32_argument);
               // set write bit
               BGVAR(u16_SrvSns_ReqStatus) |= SRVSNS_SAL_REQ_RW_BIT_MASK;
            break;

            case SRVSNS_CMD_EE_WRITE_DISABLE:
               // load the password
               BGVAR(u32_SrvSns_Value) = SRVSNS_ADDR_CMD_EN_DIS_EE_WR_DIS | ((unsigned int)u32_argument);
               // set write bit
               BGVAR(u16_SrvSns_ReqStatus) |= SRVSNS_SAL_REQ_RW_BIT_MASK;
            break;

            case SRVSNS_CMD_RESET_MT_CNTR:
               if(!Enabled(drive))
               {
                  // Load the password
                  BGVAR(u32_SrvSns_Value) = SRVSNS_ADDR_CMD_RESET_MT_PLD;
               }
               else
                  ret_val = DRIVE_ACTIVE;
            break;

            case SRVSNS_CMD_APDXA_ENABLE:
               // load the password
               BGVAR(u32_SrvSns_Value) = SRVSNS_ADDR_CMD_APDXA_EN_DIS_EN;
            break;

            case SRVSNS_CMD_APDXA_DISABLE:
               // load the password
               BGVAR(u32_SrvSns_Value) = SRVSNS_ADDR_CMD_APDXA_EN_DIS_DIS;
            break;

            case SRVSNS_CMD_SETZEROPOS:
               // load the password
               BGVAR(u32_SrvSns_Value) = SRVSNS_ADDR_CMD_ZEROPOS_PSW | ((unsigned int)u32_argument);
               // set write bit
               BGVAR(u16_SrvSns_ReqStatus) |= SRVSNS_SAL_REQ_RW_BIT_MASK;
            break;

            case SRVSNS_CMD_SETSUBMODEFUNC:
               if (u16_is_write)
               {
                  unsigned int u16_mode = VAR(AX0_u16_SrvSns_MBX_Header);
                  if (u32_argument > 0)
                  {
                     u16_mode |= SRVSNS_RTMBX_HDR_SUBMODE_FUNC_MASK;
                  }
                  else
                  {
                     u16_mode &= ~SRVSNS_RTMBX_HDR_SUBMODE_FUNC_MASK;
                  }
                  VAR(AX0_u16_SrvSns_MBX_Header) = u16_mode;
                  BGVAR(u16_SrvSns_ReqStatus) |= SRVSNS_SAL_REQ_RW_BIT_MASK;
                  BGVAR(u32_SrvSns_Value) = u32_argument;
               }
               else
               {
                  BGVAR(u16_SrvSns_ReqStatus) &= ~SRVSNS_SAL_REQ_RW_BIT_MASK;
                  BGVAR(u32_SrvSns_Value) = 0;
               }
            break;

            case SRVSNS_CMD_GET_SET_COMM:
            case SRVSNS_CMD_CALIB_SYNC:
            case SRVSNS_CMD_PWR_CNTRL:
            case SRVSNS_CMD_MT_CALIB:
               if ( (u16_cmd == SRVSNS_CMD_PWR_CNTRL)      ||
                    (u16_cmd == SRVSNS_CMD_GET_SET_COMM)   ||
                    (u16_cmd == SRVSNS_CMD_MT_CALIB)          )
               {
                  // Do not allow execution of these command when the drive is enabled
                  if (Enabled(drive))
                  {
                     ret_val = DRIVE_ACTIVE;
                     break;// leave the switch block here
                  }
               }

               if (u16_is_write)
               {
                  if (u32_argument > 1)
                  {
                     ret_val = VALUE_OUT_OF_RANGE;
                  }
                  else
                  {
                     BGVAR(u16_SrvSns_ReqStatus) |= SRVSNS_SAL_REQ_RW_BIT_MASK;
                     BGVAR(u32_SrvSns_Value) = u32_argument;
                  }
               }
               else
               {
                  BGVAR(u16_SrvSns_ReqStatus) &= ~SRVSNS_SAL_REQ_RW_BIT_MASK;
                  BGVAR(u32_SrvSns_Value) = 0;
               }
            break;

            case SRVSNS_CMD_FACTORY_RESTORE:
               // load the password
               BGVAR(u32_SrvSns_Value) = SRVSNS_ADDR_CMD_FACT_REST_PLD;
            break;

            default:
            return UNKNOWN_COMMAND;
         }
         if(ret_val == SAL_NOT_FINISHED)
         {
            SrvSns_SendServiceRequest(drive, u16_cmd, u16_fdbk_source);
            BGVAR(u16_SrvSns_ServiceState) = SRVSNS_SERVICE_STATE_RESPONSE;
         }
      break;

      case SRVSNS_SERVICE_STATE_RESPONSE:// Wait for status response from SrvSns manager
         ret_val = SrvSns_GetServiceResponse(drive, p_s32_response, u16_fdbk_source);
         switch(u16_cmd)
         {
            case SRVSNS_CMD_SETSUBMODEFUNC:
               if (u16_is_write)
               {
                  if((ret_val != SAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED)) // the submode function failed
                  {
                     // if the command failed, clear the submode function bit
                     unsigned int u16_mode = VAR(AX0_u16_SrvSns_MBX_Header);
                     u16_mode &= ~SRVSNS_RTMBX_HDR_SUBMODE_FUNC_MASK;
                     VAR(AX0_u16_SrvSns_MBX_Header) = u16_mode;
                  }
                  break;
               }
            case SRVSNS_CMD_GET_FAULTS:
            case SRVSNS_CMD_GET_WARNINGS:
            case SRVSNS_CMD_HALLS_SYNC_REC_DONE:
            case SRVSNS_CMD_ZEROPOSST:
            case SRVSNS_CMD_EE_ACCESS_ST:
            case SRVSNS_CMD_GETRUNTIME:
            case SRVSNS_CMD_GET_TEMP_HIST_BAR:
            case SRVSNS_CMD_GET_VEL_HIST_BAR:
            case SRVSNS_CMD_MT_CALIB:
               if(ret_val == SAL_SUCCESS)
                  ret_val = SrvSns_GetDataSwitch(drive, u16_fdbk_source);
            break;

            case SRVSNS_CMD_MT_CONFIG:
            case SRVSNS_CMD_RESET_MT_CNTR:
               if(ret_val == SAL_SUCCESS)
               {
                  // If the MT Config Required fault has not beed detected,
                  // hence user will not issue the clearfaults command, so the absolute PFB will not be updated.
                  // The PFB need to be updated due to MT counter reset while MT Config.
                  // This restarts the initialization of the absolute position in RT state machine in order to issue the PFB update
                  if((BGVAR(s64_SysNotOk_2) & SRVSNS_ENCODER_FLT_MASK) == 0)
                  {
                     // Store the current comm mode to temporary variable
                     u16_temp = SrvSns_GetCommMode(drive);

                     // Set response to multi-turn
                     SrvSns_SetCommMode((SRVSNS_COMM_MODE_OPERATIONAL | SRVSNS_COMM_SUBMODE_MULTITURN), drive);

                     // Transition to the next state in order to allow driver properly read the MT value
                     BGVAR(u16_SrvSns_ServiceState) = SRVSNS_SERVICE_STATE_AUXILIARY_0;
                     ret_val = SAL_NOT_FINISHED;
                  }
               }
            break;

            case SRVSNS_CMD_ACK_BATT_DN_FLT:
               // do nothing, further action depends on ret_val
            break;

            default:
               // do nothing, further action depends on ret_val
            break;
         }
      break;

      case SRVSNS_SERVICE_STATE_AUXILIARY_0:// Wait for some asynchronous drive operation to complete
         switch(u16_cmd)
         {
            case SRVSNS_CMD_MT_CONFIG:
            case SRVSNS_CMD_RESET_MT_CNTR:
               // Trigger the RT to read the absolute position from the encoder
               if (0 == u16_fdbk_source)
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= (COMM_FDBK_INIT_ABS_POS_MASK);// | COMM_FDBK_INIT_ABS_POS_IN_REV_MASK);
               else
                  VAR(AX0_u16_Comm_Fdbk_Flags_A_2) |= (COMM_FDBK_INIT_ABS_POS_MASK);// | COMM_FDBK_INIT_ABS_POS_IN_REV_MASK);
               BGVAR(u16_SrvSns_ServiceState) = SRVSNS_SERVICE_STATE_AUXILIARY_1;
            break;

            default:
               ret_val = SRVSNS_DRV_ERROR;
            break;
         }
      break;

      case SRVSNS_SERVICE_STATE_AUXILIARY_1:// Wait for some asynchronous drive operation to complete
         switch(u16_cmd)
         {
            case SRVSNS_CMD_MT_CONFIG:
            case SRVSNS_CMD_RESET_MT_CNTR:
               // Wait till the position feedback is initialized on the drive side
               if (((0 == u16_fdbk_source) && ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_INIT_ABS_POS_MASK) == 0)) ||
                   ((1 == u16_fdbk_source) && ((VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & COMM_FDBK_INIT_ABS_POS_MASK) == 0)))
               {
                  // Set back the comm mode
                  SrvSns_SetCommMode(u16_temp, drive);
                  ret_val = SAL_SUCCESS;
               }
            break;

            default:
               ret_val = SRVSNS_DRV_ERROR;
            break;
         }
      break;

      default:
         BGVAR(u16_SrvSns_ServiceState)   = SRVSNS_SERVICE_STATE_REQUEST;
         BGVAR(u16_SrvSns_ReadState)      = SRVSNS_READ_STATE_LOAD_ADDR;
         u16_is_write                     = 0;
         ret_val = SRVSNS_DRV_ERROR;
      break;
   }

   if(ret_val != SAL_NOT_FINISHED)
   {
      BGVAR(u16_SrvSns_ServiceState)   = SRVSNS_SERVICE_STATE_REQUEST;
      BGVAR(u16_SrvSns_ReadState)      = SRVSNS_READ_STATE_LOAD_ADDR;
      u16_is_write                     = 0;
   }
   return ret_val;
}


//**********************************************************
// Function Name: SrvSns_ServiceChRequest
// Description:
//   This function executes SrvSns service channel request state machine
//    according to the specified arguments.
//
// Author: Anatoly Odler
// Algorithm:  State Machine
// Revisions:
// Notes:
//**********************************************************
int SrvSns_ServiceChRequest(unsigned int u16_request_id, unsigned int u16_addr_id, unsigned long u32_value, long* p_s32_response, int drive, int u16_fdbk_source)
{
   static int state = 0;
   static long s32_acquisition_timer = 0L;
   static unsigned long u32_owner = 0UL;
   int ret_val = SAL_NOT_FINISHED;

   if ( (!FEEDBACK_SERVOSENSE) || BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
      /* Release BiSS-C device */
      SrvSns_Release(&u32_owner, drive);
      u32_owner = 0;
      s32_acquisition_timer = 0;
      state = 0;
      return NOT_AVAILABLE;
   }

   if(p_s32_response == NULL)
      return SRVSNS_INTERNAL_REQ_ERR;

   switch(state)
   {
      case 0:// take acquisition timer
         s32_acquisition_timer = Cntr_1mS;
         state++;
      break;

      case 1:// Acquire the ServoSense device
         ret_val = SrvSns_Acquire(&u32_owner, drive);
         if (ret_val == SAL_SUCCESS)
         {
            ret_val = SAL_NOT_FINISHED;
            state++;
            break;
         }
         if (PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, s32_acquisition_timer))
            ret_val = SRVSNS_DRV_ACQ_TIMEOUT;
      break;

      case 2:
         switch(u16_request_id)
         {
            case SRVSNS_DRVREQ_READ:
               ret_val = SrvSnsReadAddr(u32_owner, u16_addr_id, p_s32_response, drive, u16_fdbk_source);
            break;

            case SRVSNS_DRVREQ_WRITE:
               ret_val = SrvSnsWriteAddr(u32_owner, u16_addr_id, u32_value, drive, u16_fdbk_source);
            break;

            case SRVSNS_DRVREQ_COMMAND:
               ret_val = SrvSnsSendCmd(u32_owner, u16_addr_id, u32_value, p_s32_response, drive, u16_fdbk_source);
            break;

            default:
               ret_val = SRVSNS_DRV_ERROR;
            break;
         }
      break;

      default:
         ret_val = SRVSNS_INTERNAL_REQ_ERR;
      break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {  // Release ServoSense device
      SrvSns_Release(&u32_owner, drive);
      u32_owner = 0;
      s32_acquisition_timer = 0;
      state = 0;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SrvSnsReadAddrSafe
// Description:
//  This function reads a value from ServoSense Encoder
//  according to the specified address.
//  The value under *p_s32_response will contain the read value
//  if the execution is finished successfully.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:       This function executes in a safe manner, meaning
//             it acquires the ServoSense and releases it after the operation is complete,
//             i.e. it removes the need to acquire and release the ServoSense by the caller.
//             It is typically used when a single request to read a value is required.
//***********************************************************
int SrvSnsReadAddrSafe(unsigned int u16_addr, long* p_s32_response, int drive, int u16_fdbk_source)
{
   return(SrvSns_ServiceChRequest(SRVSNS_DRVREQ_READ, u16_addr, 0UL, p_s32_response, drive, u16_fdbk_source));
}


//**********************************************************
// Function Name: SrvSnsWriteAddrSafe
// Description:
//   This function writes a value to ServoSense Encoder
//   according to the specified address.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:       This function executes in a safe manner, meaning
//             it acquires the ServoSense and releases it after the operation is complete,
//             i.e. it removes the need to acquire and release the ServoSense by the caller.
//             It is typically used when a single request to write a value is required.
//***********************************************************
int SrvSnsWriteAddrSafe(unsigned int u16_addr, unsigned long u32_value, int drive, int u16_fdbk_source)
{
   long s32_dummy = 0;
   return(SrvSns_ServiceChRequest(SRVSNS_DRVREQ_WRITE, u16_addr, u32_value, &s32_dummy, drive, u16_fdbk_source));
}


//**********************************************************
// Function Name: SrvSnsSendCmdSafe
// Description:
//   This function sends a command to ServoSense Encoder.
//   *p_s32_response will contain a corresponding numeric response
//   when the execution is finished successfully.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:       This function executes in a safe manner, meaning
//             it acquires the ServoSense and releases it after the operation is complete,
//             i.e. it removes the need to acquire and release the ServoSense by the caller.
//             It is typically used when a single command request is required.
//***********************************************************
int SrvSnsSendCmdSafe(unsigned int u16_cmd, unsigned long u32_argument, long* p_s32_response, int drive, int u16_fdbk_source)
{
   return(SrvSns_ServiceChRequest(SRVSNS_DRVREQ_COMMAND, u16_cmd, u32_argument, p_s32_response, drive, u16_fdbk_source));
}


// Read the ServoSense feedback periodicaly
int ReadServoSenseTemperature(int drive, int u16_fdbk_source)
{
   static unsigned int u16_ss_temp_read_state = 0;
   static long s32_ss_temp_time_capture;
   static unsigned long u32_owner = 0;
   int u16_ret_val = SAL_NOT_FINISHED;
   long s32_srvsns_response = 0L;
   // AXIS_OFF;

   // If not on ServoSense or ServoSense not ready ignore
   if ((!FEEDBACK_SERVOSENSE) ||
       (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK) ||
       (BGVAR(u16_Read_Mtp_State) != MTP_INIT_INIT) || (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_TIME_OUT_ERROR_MASK) ||
       (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_ERROR_MASK))
   {
      SrvSns_Release(&u32_owner, drive);
      u16_ss_temp_read_state = 0;
      return (NOT_AVAILABLE);
   }

   // If not communicating (ServoSense calibration)
   if((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0)
      return NOT_AVAILABLE;

   switch (u16_ss_temp_read_state)
   {
      case 0:
         s32_ss_temp_time_capture = Cntr_1mS;
         u16_ss_temp_read_state++;
      break;

      case 1: // Acquire ServoSense
         // Acquire ServoSense device
         u16_ret_val = SrvSns_Acquire(&u32_owner, drive);
         if(u16_ret_val == SAL_SUCCESS)
         {
            u16_ret_val = SAL_NOT_FINISHED;
            u16_ss_temp_read_state++;
            break;
         }
         if (PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC ,s32_ss_temp_time_capture))
            u16_ret_val = SRVSNS_DRV_ACQ_TIMEOUT;
      break;

      case 2:
         if ((u16_ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_INT_PAR_TEMPERATURE, &s32_srvsns_response, drive, u16_fdbk_source)) == SAL_SUCCESS)
         {
            BGVAR(s16_SrvSns_Temperature) = (int)s32_srvsns_response;
            s32_ss_temp_time_capture = Cntr_1mS;
            u16_ss_temp_read_state++;
         }
         if (u16_ret_val != SAL_NOT_FINISHED)
         {
            SrvSns_Release(&u32_owner, drive);         // Release ServoSense device

            if(u16_ret_val == SAL_SUCCESS)
               u16_ret_val = SAL_NOT_FINISHED;
         }
      break;

      case 3:
         if (PassedTimeMS(5000L, s32_ss_temp_time_capture))
            u16_ret_val = SAL_SUCCESS;
      break;
   }

   if (u16_ret_val != SAL_NOT_FINISHED)
   {
      u16_ss_temp_read_state = 0;
      s32_ss_temp_time_capture = 0;
   }

   return u16_ret_val;
}


//**********************************************************
// Function Name: SrvSns_FWDnLdProcessorRT
// Description:
//   This function is called on feedback communication routine in RT.
//   Its purpose is to provide Encoder to PC retransmission
//   for FW download task.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
#pragma CODE_SECTION(SrvSns_FWDnLdProcessorRT, "ramfunc_2");
void SrvSns_FWDnLdProcessorRT(void)
{
   unsigned char u8_data_byte = 0;

   HWDisplay(E_LET, 4);   // Indicate FW download on the display
   if(u16_Product == SHNDR_HW)
   {
      HWDisplay(N_LET, 3);
      HWDisplay(F_LET, 2);
      HWDisplay(L_LET, 1);
      HWDisplay(d_LET, 0);
   }

   if( (u8_Output_Buffer_Free_Space > 0) && (*(unsigned int *)FPGA_RX_DATA_READY_REG_ADD == 1) )
   {
      u8_data_byte = *(unsigned int *)(FPGA_MFB_RX_BUFFER_ADD);
      // trig to reset data ready register
      *(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD = 1;
      *(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD = 0;

      u8_data_byte >>= 1;
      u8_data_byte &= 0x00FF;

      //  Copy data to the output buffer
      *p_u8_Out_Buf_In_Ptr = u8_data_byte;

      //  Increment output buffer inp pointer
      IncrementPtr(p_u8_Out_Buf_In_Ptr,u8_Output_Buffer,p_u8_Out_Buf_End);

      //  Decrement free space counter
      u8_Output_Buffer_Free_Space--;
   }

   return;
}


//**********************************************************
// Function Name: SrvSns_FWDnLdProcessorBG
// Description:
//   This function is called on each background.
//   Its purpose is to provide PC to Encoder retransmission
//   for FW download task.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:      This function checks for PC timeout in order
//          to indicate the end of the FW download process.
//          It doesn't provide any success or failure status of the
//          FW download.
//***********************************************************
void SrvSns_FWDnLdProcessorBG(int drive)
{
   unsigned char u8_data_byte = 0;
   static int start = 1;
   static long s32_pc_comm_timer = 0, s32_enc_comm_timer = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (start)
   {
      s32_pc_comm_timer = Cntr_1mS;
      start = 0;
   }

   while(u8_Input_Buffer_Free_Space < COMMS_BUFFER_SIZE)
   {
      // take timeout for encoder transmit
      s32_enc_comm_timer = Cntr_1mS;

      while((*(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD) & 0x0002)
      {
         if(PassedTimeMS(SRVSNS_TX_DONE_TIMEOUT_mSEC, s32_enc_comm_timer))
         {
            VAR(AX0_u16_SrvSns_FWStatus) |= SRVSNS_FW_STATUS_ERROR_MASK;
            break;
         }
      }

      if(VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_ERROR_MASK)
         break;

      u8_data_byte = *p_u8_In_Buf_Out_Ptr;

      u8_data_byte <<= 1;
      u8_data_byte |= 0xFE00;
      *(unsigned int *)FPGA_MFB_TX_BUFFER_ADD = u8_data_byte;
      // toggle --> transmit data
      *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 0;
      *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 1;

      // Increment input buffer output ptr
      IncrementPtr(p_u8_In_Buf_Out_Ptr,u8_Input_Buffer,p_u8_In_Buf_End);

      // Increment free space counter
      u8_Input_Buffer_Free_Space++;

      // take timeout for PC
      s32_pc_comm_timer = Cntr_1mS;
   }

   if(   PassedTimeMS(SRVSNS_FWDNLD_TIMEOUT_mSEC, s32_pc_comm_timer)         ||
         (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_ERROR_MASK) )
   {
      VAR(AX0_u16_SrvSns_FWStatus) &= ~SRVSNS_FW_STATUS_FWDNLD_RETRANSMIT_MASK;
      s32_pc_comm_timer = 0;
      s32_enc_comm_timer = 0;
      start = 1;
   }

   return;
}


//**********************************************************
// Function Name: SrvSns_SetCommMode
// Description:
//    This function sets the ServoSense communication
//    mode bits atomically.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
void SrvSns_SetCommMode(unsigned int u16_mode, int drive)
{
    // AXIS_OFF;
    unsigned int u16_temp_header = VAR(AX0_u16_SrvSns_MBX_Header);
    REFERENCE_TO_DRIVE;
    u16_temp_header &= ~SRVSNS_RTMBX_HDR_MODE_BITS_MASK;
    u16_temp_header |= u16_mode & SRVSNS_RTMBX_HDR_MODE_BITS_MASK;

    VAR(AX0_u16_SrvSns_MBX_Header) = u16_temp_header;

    return;
}


//**********************************************************
// Function Name: SrvSns_SetCommMode
// Description:
//    This function returns the SrvSns Communication mode.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
unsigned int SrvSns_GetCommMode(int drive)
{
   // AXIS_OFF;
   unsigned int u16_temp_header = VAR(AX0_u16_SrvSns_MBX_Header);
   REFERENCE_TO_DRIVE;
   u16_temp_header &= (SRVSNS_RTMBX_HDR_MODE_BITS_MASK);

   return u16_temp_header;
}


//**********************************************************
// Function Name: SrvSns_SetFWStatus
// Description:
//    This function sets the ServoSense FW status
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:       s16_fdbk_dis indicates COMM_FDBK_DIS_MASK in s16_DisableInputs
//***********************************************************
void SrvSns_SetFWStatus(unsigned int u16_fw_st_mask, int s16_fdbk_dis, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   VAR(AX0_u16_SrvSns_FWStatus) |= u16_fw_st_mask;

   if (s16_fdbk_dis)
   {  // Set disable bit to indicate the initialization incomplete
      BGVAR(s16_DisableInputs) |= COMM_FDBK_DIS_MASK;
   }
   else
   {  // Reset disable bit to allow the user monitor the state
      BGVAR(s16_DisableInputs) &= ~COMM_FDBK_DIS_MASK;
   }
}


//**********************************************************
// Function Name: SrvSns_OnInitProcedure
// Description:
//    This function is intended to be called after
//    the SRVSNS communication is established.
//    It performs the following system related readouts
//    and commands on the SRVSNS:
//    - Clearfaults command sent to SRVSNS.
//    - Product type is read (to identify if MT or ST).
//    - SW and HW versions are read and parsed.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SrvSns_OnInitProcedure(unsigned long u32_owner, int drive, int u16_fdbk_source)
{
   static unsigned int u16_state = 0;
   static unsigned long u32_version = 0;
   static int out_error_code = SAL_SUCCESS;
   int ret_val = SAL_NOT_FINISHED;
   long s32_dummy;

   switch(u16_state)
   {
      case 0:
         ret_val = SrvSnsSendCmd(u32_owner, SRVSNS_CMD_CLEAR_FAULTS, 0, &s32_dummy, drive, u16_fdbk_source);
         if(ret_val != SAL_NOT_FINISHED)// if finished
         {
            if(ret_val != SAL_SUCCESS)// If failed, set the appropriate flag
               SrvSns_SetFWStatus(SRVSNS_FW_STATUS_CLRFLTS_FAILED_MASK, 1, drive);

            // Continue anyway
            out_error_code = ret_val;
            ret_val = SAL_NOT_FINISHED;
            u16_state++;
         }
      break;

      case 1:
         ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_PRD_INFO_AREA_PRD_TYPE, &s32_dummy, drive, u16_fdbk_source);
         if(ret_val != SAL_NOT_FINISHED)// if finished
         {
            // If success, set the product type to the read value to indicate MT or ST model
            if(ret_val == SAL_SUCCESS)
               BGVAR(u16_SrvSns_ProductType) = (unsigned int)s32_dummy;
            else
               out_error_code = ret_val;

            // Continue anyway
            ret_val = SAL_NOT_FINISHED;
            u16_state++;
         }
      break;

      case 2:
         ret_val = ReadSrvSnsVersions(drive, u32_owner, &u32_version, u16_fdbk_source);
         if(ret_val != SAL_NOT_FINISHED)// if finished
         {
            if (ret_val == SAL_SUCCESS)
               BGVAR(u32_SrvSns_Versions) = u32_version;
            else
               out_error_code = ret_val;
         }
      break;
   }

   if(ret_val != SAL_NOT_FINISHED)
   {
      ret_val = out_error_code;
      u16_state = 0;
      u32_version = 0;
      out_error_code = SAL_SUCCESS;
   }
   return ret_val;
}


//**********************************************************
// Function Name: SrvSns_OnInitProcResultHandler
// Description:
//             This function handles the result of the SrvSns_OnInitProcedure
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
void SrvSns_OnInitProcResultHandler(int error_code, int drive)
{
   if((error_code != SAL_SUCCESS) && (error_code != SAL_NOT_FINISHED))
   {
      // Indicate the On Init Procedure failed and enable the COMM_FDBK_DIS_MASK bit to prevent enable
      SrvSns_SetFWStatus(SRVSNS_FW_STATUS_ON_INIT_PROC_FAILED_MASK, 1, drive);

      // Write the power up error code
      BGVAR(u16_SrvSns_InitProcErrorCode) = error_code;
   }
   // RT communication is working if we got here.
   // Set the status FW present to indicate the communication established properly
   // regardless the result of the SrvSns_OnInitProcedure.
   SrvSns_SetFWStatus(SRVSNS_FW_STATUS_PRESENT_MASK, 1, drive);
}


//**********************************************************
// Function Name: SrvSns_ValidateBootMode
// Description:
//    This function validates ServoSense boot mode
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SrvSns_ValidateBootMode(unsigned int u16_byte_received, int drive)
{
   static unsigned int u16_boot_corr_qualif = 0;

   if (u16_byte_received == 1)
   {
      unsigned char u8_data_byte = 0, temp_byte = 0, tx_timeout = 0;
      // AXIS_OFF;

      // get byte from the Encoder
      u8_data_byte = *(unsigned int *)FPGA_MFB_RX_BUFFER_ADD;

      // trig to reset data ready register
      *(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD = 1;
      *(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD = 0;

      temp_byte = u8_data_byte;
      // eliminate UART frame
      u8_data_byte >>= 1;
      u8_data_byte &= 0xFF;

      // if the ServoSense is in bootmode, it returns the ACK byte
      if(u8_data_byte == SRVSNS_FWDNLD_ACK_BYTE)
      {
         u16_boot_corr_qualif = 0;

         // transmit the ACK_BYTE to enter boot mode interactive
         *(unsigned int *)FPGA_MFB_TX_BUFFER_ADD = temp_byte | 0xFE00;
         // toggle --> transmit data
         *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 0;
         *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 1;

         // take timeout for transmit
         BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;

         // wait till current transmission has been completed
         while((*(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD) & 0x0002)
         {
            if(PassedTimeMS(SRVSNS_TX_DONE_TIMEOUT_mSEC, BGVAR(s32_Comm_Fdbk_Timer)))
            {
               SrvSns_SetFWStatus(SRVSNS_FW_STATUS_ERROR_MASK, 1, drive);
               tx_timeout = 1;
               break;
            }
         }
         if(tx_timeout == 0)
            SrvSns_SetFWStatus(SRVSNS_FW_STATUS_BOOT_PRESENT_MASK, 1, drive);
      }
      else
      {
         if (u16_boot_corr_qualif > 10)
         {
            SrvSns_SetFWStatus(SRVSNS_FW_STATUS_BOOT_CORRUPTED_MASK, 1, drive);
            u16_boot_corr_qualif = 0;
         }
         else
         {
            u16_boot_corr_qualif++;
            return SAL_NOT_FINISHED;
         }
      }

      VAR(AX0_u16_SrvSns_Status_Field) |= SRVSNS_STATUS_FAULT_MASK;  // declare a fault

      return SAL_SUCCESS;
   }

   u16_boot_corr_qualif = 0;
   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: int SrvSns_ResetTurns(int drive)
// Description:
//    This function resets the turns on SrvSns MT encoder
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SrvSns_ResetTurns(int drive, int u16_fdbk_source)
{
   static unsigned long u32_owner = 0;
   static long s32_acquisition_timer = 0;
   static int state = 0;
   unsigned int ret_val = SAL_NOT_FINISHED;
   long s32_dummy;
   // AXIS_OFF;

   if (  ((!FEEDBACK_SERVOSENSE_MULTI_TURN)                         ||
         (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK)  )                       ||
         ((0 == u16_fdbk_source) && ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0))   ||
         ((1 == u16_fdbk_source) && ((VAR(AX0_u16_Comm_Fdbk_Flags_A_2) & COMM_FDBK_READY_MASK) == 0))   ||
         (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK))
   {
      /* Release ServoSense device */
      if(u32_owner)
         SrvSns_Release(&u32_owner, drive);
      u32_owner = 0;
      s32_acquisition_timer = 0;
      return NOT_AVAILABLE;
   }

   switch(state)
   {
      case 0:// Capture timer
         s32_acquisition_timer = Cntr_1mS;
         state++;
      break;

      case 1:// Acquire ServoSense device
         ret_val = SrvSns_Acquire(&u32_owner, drive);
         if (ret_val == SAL_SUCCESS)
         {
            ret_val = SAL_NOT_FINISHED;
            state++;
            break;
         }
         if (PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, s32_acquisition_timer))
            ret_val = SRVSNS_DRV_ACQ_TIMEOUT;
      break;

      case 2:// Check whether battery down fault is active
         ret_val = SrvSnsSendCmd(u32_owner, SRVSNS_CMD_GET_FAULTS, 0, &s32_dummy, drive, u16_fdbk_source);
         if(ret_val == SAL_SUCCESS)
         {
            if(s32_dummy & SRVSNS_FAULT_MT_BATT_DWN_MASK)
               state = 3;// proceed to acknowledge the battery fault
            else
               state = 4;// proceed to MT config
            ret_val = SAL_NOT_FINISHED;
         }
      break;

      case 3:// Acknowledge battery fault
         ret_val = SrvSnsSendCmd(u32_owner, SRVSNS_CMD_ACK_BATT_DN_FLT, 0, &s32_dummy, drive, u16_fdbk_source);
         if(ret_val == SAL_SUCCESS)
         {
            BGVAR(s64_SysNotOk_2) &= ~SRVSNS_ENCODER_FLT_MASK;
            ret_val = SAL_NOT_FINISHED;
            state++;
         }
      break;

      case 4:// Reset multi-turn counter
         ret_val = SrvSnsSendCmd(u32_owner, SRVSNS_CMD_MT_CONFIG, 0, &s32_dummy, drive, u16_fdbk_source);
         if(ret_val == SAL_SUCCESS)
         {
            BGVAR(s64_SysNotOk_2) &= ~SRVSNS_ENCODER_FLT_MASK;
         }
      break;

      default:
         ret_val = SRVSNS_DRV_ERROR;
      break;
   }

   if(ret_val != SAL_NOT_FINISHED)
   {
      SrvSns_Release(&u32_owner, drive);
      state = 0;
      s32_acquisition_timer = 0;
      u32_owner = 0;
   }

   return ret_val;
}


//**********************************************************
// Function Name: int SrvSns_IgnoreMTFaults(unsigned int u16_mode, int drive)
// Description:
//   This routine sets the IGNOREBATTFLT mode according
//    to the specified argument for sensAR (ServoSense)encoder.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SrvSns_IgnoreMTFaults(unsigned int u16_mode, int drive, int u16_fdbk_source)
{
#define SRVSNS_IGNORE_MT_FLTS_STATE_READ     0
#define SRVSNS_IGNORE_MT_FLTS_STATE_MODIFY   1
#define SRVSNS_IGNORE_MT_FLTS_STATE_WRITE    2

   static unsigned int u16_state = 0, u16_addr = SRVSNS_INT_PAR_FLTS_MASK;
   static long s32_value = 0;
   static unsigned long u32_mask = SRVSNS_MT_FAULTS;
   int ret_val = SAL_NOT_FINISHED;

   // This state machine will make appropriate modifications to the fault mask
   // in the sensAR encoder in the Read-Modify-Write manner,
   // where there is a single state of read, modify and write.

   switch(u16_state)
   {
      case SRVSNS_IGNORE_MT_FLTS_STATE_READ:// Read state
         ret_val = SrvSnsReadAddrSafe(u16_addr, &s32_value, drive, u16_fdbk_source);
         if(ret_val == SAL_SUCCESS)
         {
            if(u16_mode)
            {
               s32_value &= ~u32_mask;
            }
            else
            {
               s32_value |= u32_mask;
            }
            u16_state = SRVSNS_IGNORE_MT_FLTS_STATE_WRITE;
            ret_val = SAL_NOT_FINISHED;
         }
      break;

      case SRVSNS_IGNORE_MT_FLTS_STATE_WRITE:// Write state
         ret_val = SrvSnsWriteAddrSafe(u16_addr, s32_value, drive, u16_fdbk_source);
         if(ret_val == SAL_SUCCESS)
         {
            if(u16_addr == SRVSNS_INT_PAR_FLTS_MASK)
            {
               u16_addr = SRVSNS_INT_PAR_WRNS_MASK;
               u32_mask = SRVSNS_MT_WRNS;
               u16_state = SRVSNS_IGNORE_MT_FLTS_STATE_READ;
               ret_val = SAL_NOT_FINISHED;
            }
         }
      break;

      default:
         ret_val = SRVSNS_DRV_ERROR;
      break;
   }

   // If the masking of the MT faults succeeded and the ignorance is active
   // then remove the faults from the fault indication at drive as well.
   // If there are other faults different from the MT related faults,
   // then the drive will detect them on the next BG cycle (or in the same one)
   if((ret_val == SAL_SUCCESS) && (u16_mode == IGNOREBATTFLT_MODE_ACTIVE))
   {
      BGVAR(u64_Sys_Warnings) &= ~SRVSNS_ENCODER_WRN_MASK;
      BGVAR(s64_SysNotOk_2) &= ~SRVSNS_ENCODER_FLT_MASK;
   }

   if(ret_val != SAL_NOT_FINISHED)
   {
      u16_state = 0;
      u16_addr = SRVSNS_INT_PAR_FLTS_MASK;
      u32_mask = SRVSNS_MT_FAULTS;
      s32_value = 0;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SrvSns_HelpTaskBG
// Description:
//    This task helps ServoSense in BG.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
void SrvSns_HelpTaskBG(int drive, int u16_fdbk_source)
{
   static unsigned long u32_owner = 0, u32_fw_ver_addr;
   static long s32_acquisition_timer = 0;
   static char s8_fw_version[8];
   long s32_srvsns_response = 0L;
   unsigned int ret_val = 0;
   // AXIS_OFF;

   if(VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_HELP_TASK_MASK)
      return; // return if already helping

   if (  ((!FEEDBACK_SERVOSENSE)                         ||
         (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK)  )                       ||
         ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0)   ||
         (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK))
   {
      /* Release ServoSense device */
      if(u32_owner)
         SrvSns_Release(&u32_owner, drive);
      u32_owner = 0;
      s32_acquisition_timer = 0;
      BGVAR(u16_SrvSns_HelpTaskState) = 0;
      return;
   }

   switch(BGVAR(u16_SrvSns_HelpTaskState))
   {
      case 0:// init state: runs once
            memset((void*)s8_fw_version, 0, sizeof(s8_fw_version));
            u32_fw_ver_addr = SRVSNS_PRD_INFO_AREA_FWVER;
            BGVAR(u16_SrvSns_HelpTaskState) = 1;
      break;

      case 1:// wait till feedback is ready and MTP has loaded
         if((VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PRESENT_MASK) && (BGVAR(u16_Init_From_MTP) == 0) && (BGVAR(u16_Read_Mtp_State) == MTP_INIT_INIT))
         /*if (  ((BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_IDLE_STATE ) || (BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_ERROR_STATE)) &&
               (BGVAR(u16_Read_Mtp_State) == MTP_INIT_INIT)                                                                                  )*/
         {
            u32_fw_ver_addr = SRVSNS_PRD_INFO_AREA_FWVER;
            s32_acquisition_timer = Cntr_1mS;
            BGVAR(u16_SrvSns_HelpTaskState) = 3;
         }
      break;

      case 2:// schedule the task each minute
         if(PassedTimeMS(60000L, s32_acquisition_timer))
         {
            s32_acquisition_timer = Cntr_1mS;
            BGVAR(u16_SrvSns_HelpTaskState) = 3;
         }
      break;

      case 3:// acquire ServoSense
         // Acquire ServoSense device
         if((ret_val = SrvSns_Acquire(&u32_owner, drive)) == SAL_SUCCESS)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 4;
         }// set the acquisition timeout to 15 sec in order to prevent longer delays if someone mistakenly acquired the ServoSense without releasing it
         else if(PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC * 15, s32_acquisition_timer))
            BGVAR(u16_SrvSns_HelpTaskState) = 1;
      break;

      case 4:// read ServoSense version
         // read the ServoSense version
         ret_val = SrvSnsReadAddr(u32_owner, u32_fw_ver_addr, &s32_srvsns_response, drive, u16_fdbk_source);
         if(ret_val == SAL_SUCCESS)
         {
            unpack_memcpy((UNSIGNED8*)&s8_fw_version[(u32_fw_ver_addr - SRVSNS_PRD_INFO_AREA_FWVER)],
                           (UNSIGNED8*)&s32_srvsns_response, SRVSNS_ADDRESS_ALIGNMENT, 1);

            u32_fw_ver_addr += SRVSNS_ADDRESS_ALIGNMENT;

            if (u32_fw_ver_addr >= (SRVSNS_PRD_INFO_AREA_FWVER + sizeof(s8_fw_version)))
            {
               s8_fw_version[6] = '\0'; // null terminate
               if(   (strcmp(s8_fw_version, "2.0.12") == 0) || (strcmp(s8_fw_version, "2.0.11") == 0) ||
                     (strcmp(s8_fw_version, "2.0.10") == 0) || (strcmp(s8_fw_version, "2.0.9") == 0)  ||
                     (strcmp(s8_fw_version, "2.0.8") == 0)  || (strcmp(s8_fw_version, "2.0.8a") == 0) ||
                     (strcmp(s8_fw_version, "2.0.9a") == 0))
               {
                  BGVAR(u16_SrvSns_HelpTaskState) = 6;// go to open Appendix A
               }
               else
               {
                  SrvSns_Release(&u32_owner, drive);
                  BGVAR(u16_SrvSns_HelpTaskState) = 11;// go to stay forever do nothing state
               }
            }
         }
         else if(ret_val != SAL_NOT_FINISHED)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 12;// go to return to schedule state
         }
         break;

      case 5:// wait for the flash failure warning
         if((ret_val = SrvSnsSendCmd(u32_owner, SRVSNS_CMD_GET_WARNINGS, 0, &s32_srvsns_response, drive, u16_fdbk_source)) == SAL_SUCCESS)
         {
            if(s32_srvsns_response & SRVSNS_WRN_FLASH_WARN_MASK)
            {
               BGVAR(u16_SrvSns_HelpTaskState) = 6;
            }
            else
            {
               BGVAR(u16_SrvSns_HelpTaskState) = 12;// go to return to schedule state
            }
         }
         else if(ret_val != SAL_NOT_FINISHED)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 12;// go to return to schedule state
         }
      break;

      case 6:// open Appdx A
         if((ret_val = SrvSnsSendCmd(u32_owner, SRVSNS_CMD_APDXA_ENABLE, 0, &s32_srvsns_response, drive, u16_fdbk_source)) == SAL_SUCCESS)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 7;
         }
         else if(ret_val != SAL_NOT_FINISHED)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 12;// go to return to schedule state
         }
      break;

      case 7:// open access to faults and warnings
         if((ret_val = SrvSnsWriteAddr(u32_owner, SRVSNS_FLT_APDXA_FLTS_WRNS_ACCESS, 1, drive, u16_fdbk_source)) == SAL_SUCCESS)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 8;
         }
         else if(ret_val != SAL_NOT_FINISHED)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 12;// go to return to schedule state
         }
      break;

      case 8:// disable power down fault: read the fault indications list
         if((ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_INT_PAR_FLTS_MASK, &s32_srvsns_response, drive, u16_fdbk_source)) == SAL_SUCCESS)
         {
            BGVAR(u32_SrvSns_FaultsIndications) = (unsigned long)s32_srvsns_response;
            if(s32_srvsns_response & SRVSNS_FAULT_POWER_DOWN_MASK)// if the fault exists in the indications list, then modify the list
               BGVAR(u16_SrvSns_HelpTaskState) = 9;
            else// otherwise, go to turn off the histogram
               BGVAR(u16_SrvSns_HelpTaskState) = 10;
         }
         else if(ret_val != SAL_NOT_FINISHED)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 12;// go to return to schedule state
         }
      break;

      case 9:// disable power down fault: modify the fault indications list
         if((ret_val = SrvSnsWriteAddr(u32_owner, SRVSNS_INT_PAR_FLTS_MASK, BGVAR(u32_SrvSns_FaultsIndications) & (~SRVSNS_FAULT_POWER_DOWN_MASK), drive, u16_fdbk_source)) == SAL_SUCCESS)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 10;
         }
         else if(ret_val != SAL_NOT_FINISHED)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 12;// go to return to schedule state
         }
      break;

      case 10:// trigger power down fault to disable histogram
         if((ret_val = SrvSnsWriteAddr(u32_owner, SRVSNS_FLT_APDXA_FAULTS, SRVSNS_FAULT_POWER_DOWN_MASK, drive, u16_fdbk_source)) == SAL_SUCCESS)
         {
            SrvSns_Release(&u32_owner, drive);
            VAR(AX0_u16_SrvSns_FWStatus) |= SRVSNS_FW_STATUS_HELP_TASK_MASK;
            BGVAR(u16_SrvSns_HelpTaskState) = 11;
         }
         else if(ret_val != SAL_NOT_FINISHED)
         {
            BGVAR(u16_SrvSns_HelpTaskState) = 12;// go to return to schedule state
         }
      break;

      case 11:// stay here forever
      break;

      case 12:// return to schedule state
         SrvSns_Release(&u32_owner, drive);
         BGVAR(u16_SrvSns_HelpTaskState) = 1;
      break;
   }
}



//**********************************************************
// Function Name: SalMtCommandsCommand
// Description:
//    This function sends comamnds to multi turn servosense encoder (P8-44)
//    the following commands are supported:
//     0: no command (to avoid sending command by mistake)
//     1: Clear battery fault
//     2: MT config/reset
//
// Author: Nitsan
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SalMtCommandsCommand(long long param,int drive)
{
   static unsigned long u32_owner_id = 0;
   static long s32_semaphore_aquire_time_capture = 0;
   unsigned long u32_argument = 0;
   unsigned int u16_cmd = (unsigned int)param;
   long s32_response;
   int ret_val = UNKNOWN_VARIABLE;
   
   // handle the incoming command
   switch (u16_cmd)
   {
      case 0:  // dont send command
         return SAL_SUCCESS;
      case 1:  // Clear battery fault command
         u16_cmd = SRVSNS_CMD_ACK_BATT_DN_FLT;
         break;
      case 2:  // MT reset command
         u16_cmd = SRVSNS_CMD_MT_CONFIG;
         break;
      default:
         return VALUE_TOO_HIGH;
   }

   switch (BGVAR(u16_P8_44_Mt_Cmd_State))
   {
      case 0:
         // timestamp for start aquiring semaphore
         s32_semaphore_aquire_time_capture = Cntr_1mS;
         BGVAR(u16_P8_44_Mt_Cmd_State)++;
         // fall through, no break
         
      case 1:
         // take semaphore
         ret_val = SrvSns_Acquire(&u32_owner_id, drive);
         if (ret_val == SAL_SUCCESS)
         {
            ret_val = SAL_NOT_FINISHED;   // set ret_val to allow parser run this SAL again in next state
            BGVAR(u16_P8_44_Mt_Cmd_State)++;
            break;
         }
         
         // wait up to 300 ms to aquire semaphore (retry in case aquire operation will fail)
         if (PassedTimeMS(300L ,s32_semaphore_aquire_time_capture))
         {
            BGVAR(u16_P8_44_Mt_Cmd_State) = 0;
            return SRVSNS_DRV_ACQ_TIMEOUT;
         }
      
         break;
         
      case 2:
         // send command
         if (SERVOSENSE_SINGLE_TURN_COMM_FDBK == BGVAR(u16_FdbkType))
            ret_val = SrvSnsSendCmd(u32_owner_id, u16_cmd, u32_argument, &s32_response, drive, 0);

         else if (SERVOSENSE_SINGLE_TURN_COMM_FDBK == BGVAR(u16_SFBType))
            ret_val = SrvSnsSendCmd(u32_owner_id, u16_cmd, u32_argument, &s32_response, drive, 1);

         if (ret_val == SAL_NOT_FINISHED)  // go to next state
         {
            BGVAR(u16_P8_44_Mt_Cmd_State)++;
         }
         else // if error or success, return the value and finish state SAL function handling
         {
            BGVAR(u16_P8_44_Mt_Cmd_State) = 0;
            SrvSns_Release(&u32_owner_id, drive);
            return ret_val;
         }
         
         break;
      
      default: // wait for command to finish execution
         if (SERVOSENSE_SINGLE_TURN_COMM_FDBK == BGVAR(u16_FdbkType))
            ret_val = SrvSnsSendCmd(u32_owner_id, u16_cmd, u32_argument, &s32_response, drive, 0);
         else if (SERVOSENSE_SINGLE_TURN_COMM_FDBK == BGVAR(u16_SFBType))
            ret_val = SrvSnsSendCmd(u32_owner_id, u16_cmd, u32_argument, &s32_response, drive, 1);
         if (ret_val == SAL_NOT_FINISHED)  // go to next state
         {
            // do nothing
         }
         else // if error or success, return the value and finish state SAL function handling
         {
            SrvSns_Release(&u32_owner_id, drive);
            BGVAR(u16_P8_44_Mt_Cmd_State) = 0;
            return ret_val;
         }
         
         break;
   }         

   return ret_val;   
}


//**********************************************************
// Function Name: SalReadFeedbacktypeViaPParam
// Description:
//    This function returns the feedback type that is connected to the drive (P8-45)
//    the following value can be returned:
//     0: no feedback is connected or unknown feedback type is connected
//     1: single turn servosense encoder
//     2: multi turn servosense encoder
//
// Author: Nitsan
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SalReadFeedbacktypeViaPParam(long long *data, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   
   // if not MTP mode, return unknown feedback type
   if (!BGVAR(u16_MTP_Mode))
   {
      *data = 0;
   }
   else if (BGVAR(s16_DisableInputs) & (MTP_READ_DIS_MASK))
   {
      // if MTP mode, but MTP read not finished yet or no feedback is connected, return unknown feedback type
      *data = 0;
   }
   else
   {
      if(BGVAR(u16_Comm_Fdbk_Init_Ran) == 0)// if feedback init stm hasn't finished yet, return unknown feedback type
      {
         *data = 0LL;
         return SAL_SUCCESS;
      }

      switch (BGVAR(u16_FdbkType))
      {
         case SERVOSENSE_SINGLE_TURN_COMM_FDBK:       // for single turn servosense
            *data = 1;
            break;

         case SERVOSENSE_MULTI_TURN_COMM_FDBK:    // for multi turn servosense
            *data = 2;
         break;

         default:                         // other types are not supported
            *data = 0;
            break;
      }
   }
   
   
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: ReadSrvSnsVersions
// Description:
//    This function returns the servos-sense sw and hw versions in a 32bit value (P8-47)
//    Format: xxaabbcc
//    xx: HW version
//    aa.bb.cc: SW version
//
//    SrvSns semaphore is taken and released by the calling function.
// Author: Nitsan
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int ReadSrvSnsVersions(int drive, unsigned long u32_owner, unsigned long *u32_version, unsigned int u16_fdbk_source)
{
   static int state = 0;
   static long s32_return_version = 0L;
   unsigned int ret_val = SAL_NOT_FINISHED;
   long s32_resp;
   long s32_tmp1, s32_tmp2;

   // AXIS_OFF;
   
   /* Do not continue if ServoSense is not in use or not ready */
   if (!FEEDBACK_SERVOSENSE)
   {
       state = 0;
       return NOT_AVAILABLE;
   }
   
   switch(state)
   {
      case 0:
         if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0)
         {
            ret_val = NOT_AVAILABLE;
            break;
         }

         // init
         s32_return_version = 0;
         state++;
         break;

      case 1: // Acquire ServoSense
         
         // write first part of fw version string to the low 16 bits
         ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_PRD_INFO_AREA_FWVER, &s32_resp, drive, u16_fdbk_source);
         if (ret_val == SAL_SUCCESS)
         {
            // take first char and third char and convert to number
            s32_return_version  = (((s32_resp ) - '0') & 0xff) * 10000;
            s32_return_version += (((s32_resp >> 16) - '0') & 0xff) * 100;
            state++;
            ret_val = SAL_NOT_FINISHED;
         }
         break;
      case 2:
         // write second part of fw version string to the high 16 bits
         ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_PRD_INFO_AREA_FWVER + SRVSNS_ADDRESS_ALIGNMENT, &s32_resp, drive, u16_fdbk_source);
         if (ret_val == SAL_SUCCESS)
         {
            // take first char and convert to number
            s32_tmp1 = (((s32_resp ) - 0x30) & 0xff) * 10;  // 0x30 is '0'
            s32_tmp2 = (((s32_resp >> 8) - 0x30) & 0xff);   // 0x30 is '0'
            s32_return_version += ((s32_tmp1 + s32_tmp2) & 0xff);
            
            //s32_return_version |= ((s32_resp ) - 0x30) & 0xff;  // 0x30 is '0'
            state++;
            ret_val = SAL_NOT_FINISHED;
         }
         break;
      
      case 3:
         // write hw version string 
         ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_PRD_INFO_AREA_HWREV, &s32_resp, drive, u16_fdbk_source);
         if (ret_val == SAL_SUCCESS)
         {
            s32_resp = (s32_resp >> 16) & 0xff;  // hw version is in the high 16 bit, so shift it to lower 16 bit
            s32_return_version += (s32_resp * 1000000);
            state++;
         }
         break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {
       state = 0;
       *u32_version = (unsigned long)s32_return_version;
   }
   
   return ret_val;
}

//**********************************************************
// Function Name: SalReadSrvSnsVersion
// Description:
//    This function returns the servos-sense sw and hw versions in a 32bit value (P8-47)
//    Format: xxaabbcc
//    xx: HW version
//    aa.bb.cc: SW version
// 
//    the value is formatted in background if there is a need to read servo-sense data
// Author: Nitsan
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SalReadSrvSnsVersion(long long *data, int drive)
{
   REFERENCE_TO_DRIVE;
   
   if (BGVAR(u32_SrvSns_Versions) == (unsigned long)(-1))
   {
       return NOT_AVAILABLE;
   }
   
   *data = BGVAR(u32_SrvSns_Versions);
   return SAL_SUCCESS;
}
