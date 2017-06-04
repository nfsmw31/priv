/*
 * BiSS_C.c
 * Creation: 08.04.2015
 * Description:
 *    This file contains the implementation of the BG part of the BiSS-C module,
 *    including the API of the driver.
 * Author: Anatoly Odler
 */

#include <string.h>

#include "BiSS_C.def"
#include "CommFdbk.def"
#include "Design.def"
#include "Err_Hndl.def"
#include "FltCntrl.def"
#include "FPGA.DEF"
#include "MultiAxis.def"
#include "Position.def"

#include "BiSS_C.var"
#include "CommFdbk.var"
#include "Drive.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Motor.var"
#include "MotorSetup.var"
#include "Position.var"

#include "BiSS_C.pro"
#include "CommFdbk.pro"
#include "Homing.pro"
#include "ModCntrl.pro"

//**********************************************Local Utilities*****************************************************//
unsigned char  BiSSC_UpdateCRCBits  (unsigned char u8_data, unsigned char u8_crc, unsigned int bits);
unsigned char  BiSSC_Reverse8       (register unsigned char u8_data);
unsigned int   BiSSC_Reverse16      (register unsigned int u16_data);


//**********************************************Local Types*********************************************************//
/* This is a control frame for the single register read/write requests */
typedef struct
{
   unsigned int start_addr    : 1;  /* 0     */
   unsigned int cts           : 1;  /* 1     */
   unsigned int id            : 3;  /* 2-4   */
   unsigned int address       : 7;  /* 5-11  */
   unsigned int crc_addr      : 4;  /* 12-15 */
   unsigned int read          : 1;  /* 16    */
   unsigned int write         : 1;  /* 17    */
   unsigned int start_val     : 1;  /* 18    */
   unsigned int val           : 8;  /* 19-26 */
   unsigned int crc_val       : 4;  /* 27-30 */
   unsigned int stop          : 1;  /* 31    */
} BiSSC_RegRequestFrame;


/* This is a control frame for the single command request */
typedef struct
{
   unsigned int start_ids     : 1;  /* 0     */
   unsigned int cts           : 1;  /* 1     */
   unsigned int ids           : 7;  /* 2-8   */
   unsigned int cmd           : 2;  /* 9-10 */
   unsigned int crc           : 4;  /* 11-14 */
   unsigned int start_req     : 1;  /* 15    */
   unsigned int ex            : 1;  /* 16    */
} BiSSC_CmdFrame;

/* This is a response frame for the single register request (both read and write) */
typedef struct
{
   unsigned int read       : 1;  /* 0     */
   unsigned int write      : 1;  /* 1     */
   unsigned int start      : 1;  /* 2     */
   unsigned int value      : 8;  /* 3-10  */
   unsigned int crc        : 4;  /* 11-14 */
   unsigned int stop       : 1;  /* 15    */
} BiSSC_RegRespFrame;

/* This is a control frame for the single command response */
typedef struct
{
   unsigned int idl        : 8;  /* 0-7  */
} BiSSC_CmdRespFrame;


//**********************************************API Implementation******************************************************//


//**********************************************************
// Function Name: void BiSSC_Init(int drive)
// Description:
//   This function initializes the BiSS-C SW module.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:
//**********************************************************
void BiSSC_Init(int drive, int fdbk_dest)
{
   FDBK_OFF;
   REFERENCE_TO_DRIVE;

   // Initialize the global variables
   BGVAR(u32_BiSSC_Lock) = (unsigned long)BISSC_LOCK_FREE;
   BGVAR(BiSSC_Driver_State) = BISSC_DRVST_SETREQUEST;
   
   //reset other feedbacks SM to idle
   BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_IDLE_STATE;

   // Reset mailbox
   BGVAR(BiSSC_BG_MailBox).u16_control    = 0;
   BGVAR(BiSSC_BG_MailBox).u16_cmd        = 0;
   BGVAR(BiSSC_BG_MailBox).u8_address     = 0;
   BGVAR(BiSSC_BG_MailBox).u8_value       = 0;
   BGVAR(u32_BiSSC_BG_RequestFrameCopy)   = 0;

   // Set the manager state to init
   BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_INIT_START;

   BGVAR(u32_BiSSC_ConsecFaultCtr) = 0;

   VAR(AX0_u16_BiSSC_RTMBX_Control) = 0;

   FDBKVAR(AX0_u16_Abs_Enc_Info_Frame) = 0;   // Reset the status
}


//**********************************************************
// Function Name: BiSSC_BGManager
// Description:
//   This function manages the BiSS C device in BG.
//
//
// Author: Anatoly Odler
// Algorithm:  SPOOLER state machine
// Revisions:
// Notes:     This function has to be called on each background
//**********************************************************
void BiSSC_BGManager(int drive, int enc_source, int fdbk_dest)
{
   register BiSSC_BGMBX* p_bg_mailbox = &BGVAR(BiSSC_BG_MailBox);
   register unsigned char u8_crc = 0;
   FDBK_OFF;

   // Do not continue if BiSS-C is not in use
   if ((fdbk_dest == 0) && !FEEDBACK_BISSC)
      return;

   if(SFB_BISSC && !IS_SECONDARY_FEEDBACK_ENABLED)
      return;

   // Do not continue if BiSS-C is not in use
   if(fdbk_dest && !SFB_BISSC)
      return;

   BGVAR(u32_BiSSC_LockCntr)++;   // generate the lock

   // If a lock is acquired, check for release timeout
   if ( (BGVAR(u32_BiSSC_Lock) != (unsigned long)BISSC_LOCK_FREE)    &&
        ( (BGVAR(BiSSC_Manager_State) != BISSC_MNGRST_IDLE)       ||
          (BGVAR(BiSSC_Manager_State) != BISSC_MNGRST_SETRESPONSE)  )  )
   {
      if (PassedTimeMS(BISSC_LOCK_RELEASE_TIMEOUT_mSEC, BGVAR(s32_BiSSC_LockReleaseTimer)))
      {
         BGVAR(u32_BiSSC_LockTimeoutCntr)++; // for debug monitoring

         BGVAR(u32_BiSSC_Lock) = (unsigned long)(BISSC_LOCK_FREE); // Release the device

         // Initialize the state machines
         // Set the manager state to idle
         BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_IDLE;

         // Set the driver state to request
         BGVAR(BiSSC_Driver_State) = BISSC_DRVST_SETREQUEST;
      }
   }

   switch(BGVAR(BiSSC_Manager_State))
   {
      case BISSC_MNGRST_INIT_START:// initialization state for low level communication
         // Disable RT communication, clear all bits except for Sync Symbol
         FDBKVAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= 0x00007;

         // Set disable bit to prevent enabling the drive
         BGVAR(s16_DisableInputs) |= BISSC_DIS_MASK;

         BiSSC_Init(drive, fdbk_dest);         // initialize module variables

         // Power down the 5V to the encoder
         if (enc_source == 0)
            RemoveFeedback5Volt();
         else
            RemoveFeedbackSecondary5Volt();

         BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
         BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_INIT_PWR_DN;
      break;

      case BISSC_MNGRST_INIT_PWR_DN:// on BiSS-C device power down state
         if (PassedTimeMS(10L, BGVAR(s32_Comm_Fdbk_Timer)))
         {
            // Power up the 5V to the encoder
            if (enc_source == 0)
               SetFeedback5Volt();
            else
               SetFeedbackSecondary5Volt();

            BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
            BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_INIT_PWR_UP;
         }
      break;

      case BISSC_MNGRST_INIT_PWR_UP:// on BiSS-C device power up state
         if (PassedTimeMS(2000L, BGVAR(s32_Comm_Fdbk_Timer)))
         {
            // configure FPGA
            *(unsigned int*)(FPGA_RX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = 3 + (LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) >> 24) + ((LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) >> 8) & 0x00FF) + 2 + 6 + 1;
            *(unsigned int*)(FPGA_MFB_ENCODER_SEL_REG_ADD + 0x0540 * enc_source) = 9;
            *(unsigned int*)(FPGA_SD_CLOCK_DIV_REG_ADD + 0x0540 * enc_source) = 23;
            *(unsigned int*)(FPGA_TX_DATA_LENGTH_REG_1_ADD + 0x0540 * enc_source) = 0;
            *(unsigned int*)(FPGA_BISS_C_CDM_REG_ADD + 0x0540 * enc_source) = 0;
            *(unsigned int*)(FPGA_ENDAT_ENCODER_CLOCK_REG_ADD + 0x0540 * enc_source)  = 3;// enable FPGA clock control
            /*
             * Perform the bus reset (according to the BiSS-C specification):
             * Start the first cycle without taking the input data into account.
             */
            *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 0; // Toggle Send Bit.
            *(unsigned int *)(FPGA_SANYO_DENKY_EN_REG_ADD + 0x0540 * enc_source) = 1;
            BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS;
            BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_INIT_POS_INIT;
         }
      break;

      case BISSC_MNGRST_INIT_POS_INIT:// initialize absolute position value
         // wait till Bus Reset occurs
         if (PassedTimeMS(50L, BGVAR(s32_Comm_Fdbk_Timer)))
         {
            if(*(unsigned int*)(FPGA_RX_DATA_READY_REG_ADD + 0x0540 * enc_source)!= 0x0000)
            {
               *(unsigned int*)(FPGA_RX_DATA_READY_REG_ADD + 0x0540 * enc_source) = 0x0000; // acknowledge Bus Reset

               // Verify the Multi Turn availability, or determine Direct-Drive-Rotary according to MPOLES, or Linear Motor:
               if ( (((unsigned int)(LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) >> 24)) > 0) ||
                    (BGVAR(u16_Mpoles) > 20) || (BGVAR(u16_MotorType) == LINEAR_MOTOR)          )
                  FDBKVAR(AX0_u16_Abs_Fdbk_Device) |= (0x0001 << fdbk_dest); // Manipulate Bit 0 only as indicator for Motor Feedback
               else
                  FDBKVAR(AX0_u16_Abs_Fdbk_Device) &= ~(0x0001 << fdbk_dest); // Manipulate Bit 0 only as indicator for Motor Feedback

               UpdateRefOffsetValue(drive);

               // Extract the MT + ST bit length
               LVAR(AX0_u32_BiSSC_MT_ST_Length) = (LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) << 16) + LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution);
               LVAR(AX0_u32_BiSSC_MT_ST_Length) >>= 24;
               // Turn on the RT Handler
               FDBKVAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= (COMM_FDBK_READY_MASK | COMM_FDBK_INIT_ABS_POS_MASK);
               BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_INIT_START_ENGINE;
            }
            else
            {
               // Initiate timeout fault
               FDBKVAR(AX0_u16_Comm_Fdbk_Flags_A_1) |= COMM_FDBK_READY_MASK;
               BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_ERROR;
            }
         }
      break;

      case BISSC_MNGRST_INIT_START_ENGINE:// starts the engine
         if((FDBKVAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_INIT_ABS_POS_MASK) == 0)
         {
            BGVAR(s16_DisableInputs) &= ~BISSC_DIS_MASK;    // clear disable bit
            BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_IDLE; // transfer to IDLE state
         }
      break;

      case BISSC_MNGRST_IDLE:// wait for request from the driver - typical runtime state
         if (p_bg_mailbox->u16_control & BISSC_BGMBX_REQ_READY_MASK)
         {
            register unsigned int u16_st_cts_id_addr;
            register BiSSC_RegRequestFrame* p_req_cntrl_frame = (BiSSC_RegRequestFrame*)&(LVAR(AX0_u32_BiSSC_RegCmdFrame[0]));

            // clear response ready in RT mailbox
            VAR(AX0_u16_BiSSC_RTMBX_Control) &= ~BISSC_RTMBX_RSP_READY_MASK;

            // acknowledge the request
            p_bg_mailbox->u16_control &= ~BISSC_BGMBX_REQ_READY_MASK;

            // clear the response buffer
            LVAR(AX0_u32_BiSSC_ResponseCmdFrame) = 0UL;

            if (p_bg_mailbox->u16_control & BISSC_BGMBX_CMD_BIT_MASK) // command request
            {
               // TODO not implemented, returns the error to the user
               p_bg_mailbox->u16_control &= ~BISSC_BGMBX_CMD_BIT_MASK;
               p_bg_mailbox->u16_control |= BISSC_BGMBX_ERROR_BIT_MASK;
               p_bg_mailbox->u16_control &= ~BISSC_BGMBX_ERROR_WORD_MASK;
               p_bg_mailbox->u16_control |= ((unsigned int)BISSC_ERR_NOT_AVAILABLE) << BISSC_BGMBX_ERROR_WORD_SHIFT;
               p_bg_mailbox->u16_control |= BISSC_BGMBX_RSP_READY_MASK;
               break;
            }
            else // register access request
            {
               // clear the request buffer
               LVAR(AX0_u32_BiSSC_RegCmdFrame[0]) = 0UL;

               // combine control frame data (for register access CTS = 1)
               u16_st_cts_id_addr = 0x0001   /* CTS = 1 */    |
                                    0x0000   /* ID2..0 = 0 */ |
                                    (((unsigned int)(BiSSC_Reverse8(p_bg_mailbox->u8_address)) >> 1) << 4) /* Address 6..0 */;

               // generate CRC for the CTS, ID and Address bits
               u8_crc = 0;
               u8_crc = BiSSC_UpdateCRCBits((u16_st_cts_id_addr & 0xFF), u8_crc, 8);
               u8_crc = BiSSC_UpdateCRCBits(((u16_st_cts_id_addr >> 8) & 0xFF), u8_crc, 3);
               u8_crc = (~u8_crc) & 0x0F;

               // Fill the request control frame
               p_req_cntrl_frame->start_addr = 1;
               p_req_cntrl_frame->cts        = 1;
               p_req_cntrl_frame->id         = 0x00;
               p_req_cntrl_frame->address    = BiSSC_Reverse8(p_bg_mailbox->u8_address) >> 1;
               p_req_cntrl_frame->crc_addr   = u8_crc;
               p_req_cntrl_frame->start_val  = 1;

               if (p_bg_mailbox->u16_control & BISSC_BGMBX_REQ_RW_BIT_MASK)    // write register request
               {
                  // For the write request modify read/write, value and value CRC bits
                  p_req_cntrl_frame->read    = 0;
                  p_req_cntrl_frame->write   = 1;
                  p_req_cntrl_frame->val     = BiSSC_Reverse8(p_bg_mailbox->u8_value);

                  // Generate CRC for the data value
                  u8_crc = 0;
                  u8_crc = BiSSC_UpdateCRCBits(p_req_cntrl_frame->val, u8_crc, 8);
                  u8_crc = (~u8_crc) & 0x0F;
                  p_req_cntrl_frame->crc_val = u8_crc;
               }
               else// read register request
               {
                  // For the write request modify read/write and set value and value CRC bits to zero
                  p_req_cntrl_frame->read       = 1;
                  p_req_cntrl_frame->write      = 0;
                  p_req_cntrl_frame->val        = 0x00;
                  p_req_cntrl_frame->crc_val    = 0x00;
               }
               // Stop bit is zero for both read and write
               p_req_cntrl_frame->stop = 0;
            }
            BGVAR(u32_BiSSC_BG_RequestFrameCopy) = LVAR(AX0_u32_BiSSC_RegCmdFrame[0]);

            BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_SETRESPONSE; // Switch the state

            // Set request ready to RT mailbox
            VAR(AX0_u16_BiSSC_RTMBX_Control) |= BISSC_RTMBX_REQ_READY_MASK;

            BGVAR(s32_Comm_Fdbk_Timer) = Cntr_1mS; // Capture the timer for timeout
         }
      break;

      case BISSC_MNGRST_SETRESPONSE:// generate response for the driver
         if (VAR(AX0_u16_BiSSC_RTMBX_Control) & BISSC_RTMBX_RSP_READY_MASK)
         {
            // Acknowledge
            VAR(AX0_u16_BiSSC_RTMBX_Control) &= ~BISSC_RTMBX_RSP_READY_MASK;
            // Here we must parse the received data.

            if (p_bg_mailbox->u16_control & BISSC_BGMBX_CMD_BIT_MASK)// command request
            {
               //register BiSSC_CmdFrame* p_request_frame = (BiSSC_CmdFrame*)&(LVAR(AX0_u32_BiSSC_ResponseCmdFrame));
               // parse the response data
               // Not supported yet
               //p_bg_mailbox->u16_control &= ~BISSC_BGMBX_CMD_BIT_MASK;
            }
            else // for register access request
            {
               register BiSSC_RegRequestFrame* p_request_frame = (BiSSC_RegRequestFrame*)&(BGVAR(u32_BiSSC_BG_RequestFrameCopy));
               register BiSSC_RegRespFrame* p_response_frame;
               unsigned int u16_response_frame = (unsigned int)LVAR(AX0_u32_BiSSC_ResponseCmdFrame);

               u16_response_frame = BiSSC_Reverse16(u16_response_frame);

               p_response_frame = (BiSSC_RegRespFrame*)&u16_response_frame;

               // Handle possible errors:
               // 1. Illegal access for read: RW = 00
               // 2. Illegal access for write: RW = 11
               // 3. CRC error on response.
               // Check the RW bits of the response
               if (p_response_frame->read ^ p_response_frame->write)// if the read and write bits are not equal
               {
                  if (p_request_frame->write)
                  {
                     // This sequence if true indicates a CRC error on receive
                     if ((p_request_frame->val - p_response_frame->value) || (p_request_frame->crc_val - p_response_frame->crc))
                     {
                        // CRC error is returned by:
                        p_bg_mailbox->u16_control |= BISSC_BGMBX_ERROR_BIT_MASK;
                        p_bg_mailbox->u16_control &= ~BISSC_BGMBX_ERROR_WORD_MASK;
                        p_bg_mailbox->u16_control |= ((unsigned int)BISSC_ERR_CRC_ERROR) << BISSC_BGMBX_ERROR_WORD_SHIFT;
                     }
                  }
                  else
                  {
                        u8_crc = 0;
                        u8_crc = BiSSC_UpdateCRCBits(p_response_frame->value, u8_crc, 8);
                        u8_crc = (~u8_crc) & 0x0F;
                        if (u8_crc != p_response_frame->crc)
                        {
                           // CRC error is returned by:
                           p_bg_mailbox->u16_control |= BISSC_BGMBX_ERROR_BIT_MASK;
                           p_bg_mailbox->u16_control &= ~BISSC_BGMBX_ERROR_WORD_MASK;
                           p_bg_mailbox->u16_control |= ((unsigned int)BISSC_ERR_CRC_ERROR) << BISSC_BGMBX_ERROR_WORD_SHIFT;
                        }
                  }
               }
               else  // if the read and write bits are equal, then the request error has been ocurred
               {
                  p_bg_mailbox->u16_control |= BISSC_BGMBX_ERROR_BIT_MASK;
                  p_bg_mailbox->u16_control &= ~BISSC_BGMBX_ERROR_WORD_MASK;

                  if (p_request_frame->write)// if register write has been requested
                  {
                     // Illegal register write error response:
                     p_bg_mailbox->u16_control |= ((unsigned int)BISSC_ERR_NOT_PRGRMBLE) << BISSC_BGMBX_ERROR_WORD_SHIFT;
                  }
                  else if (p_request_frame->read)// if register read has been requested
                  {
                     // Illegal register read error response:
                     p_bg_mailbox->u16_control |= ((unsigned int)BISSC_ERR_NOT_AVAILABLE) << BISSC_BGMBX_ERROR_WORD_SHIFT;
                  }
               }

               if ((p_bg_mailbox->u16_control & BISSC_BGMBX_ERROR_BIT_MASK) == 0)
                  p_bg_mailbox->u8_value = BiSSC_Reverse8(p_response_frame->value);
            }
            // Acknowledge write request
            p_bg_mailbox->u16_control &= ~BISSC_BGMBX_REQ_RW_BIT_MASK;
            // Indicate to the drive that response is ready
            p_bg_mailbox->u16_control |= BISSC_BGMBX_RSP_READY_MASK;
            BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_IDLE;
            return;
         }

         // Handle busy timeout
         if (PassedTimeMS(BISSC_SERV_REQ_TIMEOUT_mSEC, BGVAR(s32_Comm_Fdbk_Timer)))
         {
            // Indicate busy timeout
            p_bg_mailbox->u16_control |= BISSC_BGMBX_ERROR_BIT_MASK;
            p_bg_mailbox->u16_control &= ~BISSC_BGMBX_ERROR_WORD_MASK;
            p_bg_mailbox->u16_control |= ((unsigned int)BISSC_ERR_BUSY_TIMEOUT) << BISSC_BGMBX_ERROR_WORD_SHIFT;

            // Cleanup
            VAR(AX0_u16_BiSSC_RTMBX_Control) = 0;

            // Post response is ready
            p_bg_mailbox->u16_control |= BISSC_BGMBX_RSP_READY_MASK;
            BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_IDLE;
            BGVAR(u32_BiSSC_BusyTimeoutCntr)++;
         }
      break;

      case BISSC_MNGRST_STOPPED:// in this state the Engine is stopped
         // stop the RT handler
         VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~COMM_FDBK_READY_MASK;

         // Release the device
         BGVAR(u32_BiSSC_Lock) = (unsigned long)(BISSC_LOCK_FREE);

         // set disable bit to prevent enable
         BGVAR(s16_DisableInputs) &= ~BISSC_DIS_MASK;

         // Set the driver state to request
         BGVAR(BiSSC_Driver_State) = BISSC_DRVST_SETREQUEST;
      break;

      case BISSC_MNGRST_RESTARTED:// in this state the Engine operation is restarted
         BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_INIT_START;
      break;

      case BISSC_MNGRST_ERROR:
         // Error state does nothing
         break;

      default:
         // Cleanup
         VAR(AX0_u16_BiSSC_RTMBX_Control) = 0;
         BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_IDLE;
      break;
   }
}


//**********************************************************
// Function Name: BiSSC_GetStatus
// Description:
//   This function returns the status field of the BiSS C device.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:
//**********************************************************
unsigned int BiSSC_GetStatus(int drive, int fdbk_dest)
{
   register unsigned int u16_status;
   FDBK_OFF;
   REFERENCE_TO_DRIVE;

   u16_status = FDBKVAR(AX0_u16_Abs_Enc_Info_Frame);   // Capture status

   if (u16_status & BISSC_STATUS_FAULT_MASK)
   {
      if (BGVAR(u32_BiSSC_ConsecFaultCtr) < BGVAR(u32_BiSSC_FaultQualifierLim))
      {
         BGVAR(u32_BiSSC_ConsecFaultCtr)++;
         u16_status &= ~SRVSNS_STATUS_FAULT_MASK;
      }
   }
   else
   {
      BGVAR(u32_BiSSC_ConsecFaultCtr) = 0;
   }
   return (u16_status);
}


//**********************************************************
// Function Name: BiSSC_SetDataFields
//
// Description:
// The Function sets the Data Field values:  Multi-Turn Length, MT Resolution,
// Single-Turn Length, and ST Resolution.
//
// * Multi-Turn Length - Number of Bits dedicated to Multi-Turn Data
// * MT Resolution - MT Resolution expressed in Bits; if this is less than MT Length
//   then the Data is Right-Aligned and padded with Zeros.
// * Single-Turn Length - Number of Bits dedicated to Single-Turn Data
// * ST Resolution - ST Resolution expressed in Bits; if this is less than ST Length
//   then the Data is Left-Aligned and padded with Zeros.
// "Resolution" must be equal to or less than respective "Length".
// MT Length (and MT Resolution) may be Zero for Signle-Turn or Linear Devices.
//
// Author: Anatoly Odler/ A.H.
// Algorithm:
// Revisions:
// Note: It is the responsibility of the caller to allocate a buffer of 4 unsigned integers pointed by p_fields
//**********************************************************
int BiSSC_SetDataFields(unsigned int* p_fields, int drive)
{
   unsigned long temp;
   REFERENCE_TO_DRIVE;

   // Verify invalid buffer
   if (p_fields == NULL)
      return BISSC_DRV_ERROR;

   // Verify maximum length
   if ( ((p_fields[0] + p_fields[2]) > 55)                    ||
        // If total Data Length exceeds 55 Bits
        (p_fields[0] < p_fields[1]) || (p_fields[2] < p_fields[3])  )
        // or Effective Bits is more than respeective Length
      return VALUE_OUT_OF_RANGE;

   // Verify if CONFIG is needed using temporary variable
   temp =   ((unsigned long)p_fields[0] << 24) | ((unsigned long)p_fields[1] << 16) |\
            ((unsigned long)p_fields[2] << 8) | (unsigned long)p_fields[3];

   if (temp != LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution))
   {
      LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) = temp;
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
      VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: BiSSC_GetDataFields
// Description:
// The Function returns the Data Field values:  Multi-Turn Length, MT Resolution,
// Single-Turn Length, and ST Resolution.
//
// Author: Anatoly Odler/ A.H.
// Algorithm:
// Revisions:
// Note: It is the responsibility of the caller to allocate a buffer of 4 unsigned integers pointed by p_fields
//**********************************************************
int BiSSC_GetDataFields(unsigned int* p_fields, int drive)
{
   REFERENCE_TO_DRIVE;

   if (p_fields == NULL)
      return BISSC_DRV_ERROR;

   p_fields[0] = (unsigned int)(LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) >> 24);
   p_fields[1] = ((unsigned int)(LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) >> 16) & 0x00FF);
   p_fields[2] = ((unsigned int)(LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) >> 8) & 0x00FF);
   p_fields[3] = (unsigned int)(LVAR(AX0_u32_BiSSC_MT_ST_Length_Resolution) & 0x00FF);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: BiSSC_Acquire
// Description:
//   This function aqcuires the BiSS-C driver with a lock for the caller.
//    If the driver is locked by other owner, the function will
//    return SAL_NOT_FINISHED till it is released.
//    Returns SAL_SUCCESS on success.
//    The returned lock has to be used for BiSS-C driver
//    request invocation.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:   This function has to be called prior to any 
//          BiSS-C driver request invocation.
//**********************************************************
int BiSSC_Acquire(unsigned long* p_lock, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (p_lock == NULL)
      return BISSC_DRV_ERROR;

   // wait till the lock is released
   if (BGVAR(u32_BiSSC_Lock) != (unsigned long)BISSC_LOCK_FREE)
      return SAL_NOT_FINISHED;

   // prevent owning the FREE lock
   if (BGVAR(u32_BiSSC_LockCntr) == (unsigned long)BISSC_LOCK_FREE)
      return SAL_NOT_FINISHED;

   // take the lock
   *p_lock                             = BGVAR(u32_BiSSC_LockCntr);
   BGVAR(u32_BiSSC_Lock)               = BGVAR(u32_BiSSC_LockCntr);
   BGVAR(s32_BiSSC_LockReleaseTimer)   = Cntr_1mS;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: BiSSC_Release
// Description:
//   This function releases the BiSS-C driver from the specified lock.
//    If the provided lock is not equal to owned one, the function
//    will return BISSC_DRV_OCCUPIED, indicating the driver is currenlty
//    occupied by another owner.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:   This function has to be called after BiSS-C driver operations
//          are finished.
//**********************************************************
int BiSSC_Release(unsigned long* p_lock, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (p_lock == NULL)
      return BISSC_DRV_ERROR;

   // check if the provided lock is legal
   if (*p_lock != BGVAR(u32_BiSSC_Lock))
      return BISSC_DRV_OCCUPIED;

   // release the lock
   BGVAR(u32_BiSSC_Lock) = (unsigned long)BISSC_LOCK_FREE;
   *p_lock               = (unsigned long)BISSC_LOCK_FREE;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: BiSSC_Request
// Description:
//   This function runs the BiSS-C driver with a request
//    according to the specified arguments.
//    The function returns SAL_NOT_FINISHED if the driver is locked
//    by another owner (the provided lock doesn't correspond to the current owner).
//
// Author: Anatoly Odler
// Algorithm:  State Machine
// Revisions:
// Notes:      A lock is required to run the driver.
//             BiSS-C supports byte-by-byte data reading.
//**********************************************************
int BiSSC_Request(BiSSC_DrvReq request, unsigned long u32_lock, unsigned int u16_addr_id, char s8_value, char* p_s8_response, int drive)
{
   int ret_val = SAL_NOT_FINISHED;
   register BiSSC_BGMBX* p_mailbox = &BGVAR(BiSSC_BG_MailBox);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (p_s8_response == NULL)
   {
      BGVAR(BiSSC_Driver_State) = BISSC_DRVST_SETREQUEST;
      return BISSC_DRV_ERROR;
   }

   // check for illegal lock and driver occupied
   if ( (BGVAR(u32_BiSSC_Lock) != (unsigned long)BISSC_LOCK_FREE) &&
        (BGVAR(u32_BiSSC_Lock) != u32_lock)                         )
      return SAL_NOT_FINISHED;

   // prevent using the open lock
   if (u32_lock == (unsigned long)BISSC_LOCK_FREE)
      return BISSC_DRV_OCCUPIED;

   switch(BGVAR(BiSSC_Driver_State))
   {
      case BISSC_DRVST_SETREQUEST:
         p_mailbox->u16_control  &= ~BISSC_BGMBX_REQ_RW_BIT_MASK;
         p_mailbox->u8_address   = 0;
         p_mailbox->u8_value     = 0;
         p_mailbox->u16_cmd      = 0;

         switch(request)
         {
            case BISSC_DRVREQ_READ:
               if (u16_addr_id <= 0x7F)
                  p_mailbox->u8_address = u16_addr_id;
               else
                  ret_val = BISSC_ADDR_OUT_OF_RANGE;
            break;

            case BISSC_DRVREQ_WRITE:
               if (u16_addr_id <= 0x7F)
               {
                  p_mailbox->u8_address = u16_addr_id;
                  p_mailbox->u8_value = s8_value;
                  p_mailbox->u16_control |= BISSC_BGMBX_REQ_RW_BIT_MASK;
               }
               else
                  ret_val = BISSC_ADDR_OUT_OF_RANGE;
            break;

            case BISSC_DRVREQ_COMMAND:
               if (u16_addr_id <= 0x03)
               {
                  p_mailbox->u16_cmd = u16_addr_id;
                  p_mailbox->u16_control |= BISSC_BGMBX_CMD_BIT_MASK;
               }
               else
                  ret_val = VALUE_OUT_OF_RANGE;
            break;
         }

         p_mailbox->u16_control |= BISSC_BGMBX_REQ_READY_MASK;
         BGVAR(BiSSC_Driver_State) = BISSC_DRVST_GETRESPONSE;
      break;

      case BISSC_DRVST_GETRESPONSE:
         if (p_mailbox->u16_control & BISSC_BGMBX_RSP_READY_MASK)
         {
            p_mailbox->u16_control &= ~BISSC_BGMBX_RSP_READY_MASK;

            if (p_mailbox->u16_control & BISSC_BGMBX_ERROR_BIT_MASK)
            {
               ret_val = BiSSC_GetError(((p_mailbox->u16_control) & BISSC_BGMBX_ERROR_WORD_MASK) >> BISSC_BGMBX_ERROR_WORD_SHIFT);
               // clear the error indication
               p_mailbox->u16_control &= ~(BISSC_BGMBX_ERROR_WORD_MASK | BISSC_BGMBX_ERROR_BIT_MASK);
            }
            else
            {
               *p_s8_response = p_mailbox->u8_value;
               ret_val = SAL_SUCCESS;
            }

            BGVAR(BiSSC_Driver_State) = BISSC_DRVST_SETREQUEST;
         }
      break;

      default:
         BGVAR(BiSSC_Driver_State) = BISSC_DRVST_SETREQUEST;
         p_mailbox->u16_control  = 0;
         p_mailbox->u8_address   = 0;
         p_mailbox->u8_value     = 0;
         p_mailbox->u16_cmd      = 0;
         ret_val = BISSC_DRV_ERROR;
      break;
   }

   return ret_val;
}


//**********************************************************
// Function Name: BiSSC_GetError
// Description:
//   This function converts BiSS-C module errors to SAL error codes.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:
//**********************************************************
int BiSSC_GetError(int error)
{
   switch(error)
   {
      case BISSC_ERR_ADDR_OUT_OF_RANGE: return(BISSC_ADDR_OUT_OF_RANGE);
      case BISSC_ERR_VAL_OUT_OF_RANGE:  return(VALUE_OUT_OF_RANGE);
      case BISSC_ERR_NOT_PRGRMBLE:      return(NOT_PROGRAMMABLE);
      case BISSC_ERR_OCCUPIED:          return(BISSC_BUSY);
      case BISSC_ERR_ILLEGAL_REQUEST:   return(BISSC_ILLEGAL_REQUEST);
      case BISSC_ERR_EE_SAVE_FAILED:    return(BISSC_SAVE_FAILED);
      case BISSC_ERR_NOT_AVAILABLE:     return(NOT_AVAILABLE);
      case BISSC_ERR_UNKNOWN_REQUEST:   return(UNKNOWN_COMMAND);
      case BISSC_ERR_BUSY_TIMEOUT:      return(BISSC_BUSY_TIMEOUT);
      case BISSC_ERR_INTERNAL_ERR:      return(BISSC_INTERNAL_REQ_ERR);
      case BISSC_ERR_CRC_ERROR:         return(BISSC_CRC_ERROR);
      case BISSC_ERR_PRTCL_ERR:         return(BISSC_PROTOCOL_ERROR);      // must be last
      default:                          return(BISSC_INTERNAL_REQ_ERR);
   }
}


//**********************************************************
// Function Name: void BiSSC_Restart(int drive)
// Description:
//   This function explicitly restarts the BiSS-C Engine.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:      Can be restarted from any state.
//**********************************************************
void BiSSC_Restart(int drive)
{
   REFERENCE_TO_DRIVE;

   BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_RESTARTED;
}


//**********************************************************
// Function Name: void BiSSC_Stop(int drive)
// Description:
//   This function explicitly stops the BiSS-C Engine.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:      Can be stopped only if in run state.
//**********************************************************
void BiSSC_Stop(int drive)
{
   REFERENCE_TO_DRIVE;

   if (BGVAR(BiSSC_Manager_State) == BISSC_MNGRST_IDLE)
      BGVAR(BiSSC_Manager_State) = BISSC_MNGRST_STOPPED;

   return;
}


//**********************************************************
// Function Name: BiSSC_CntrlChRequest
// Description:
//   This function executes control channel request state machine
//    according to the specified arguments.
//
// Author: Anatoly Odler
// Algorithm:  State Machine
// Revisions:
// Notes:
//**********************************************************
int BiSSC_CntrlChRequest (BiSSC_DrvReq request, unsigned int u16_addr_id, unsigned int u16_value, int* p_s16_response, int drive)
{
   static int state = 0;
   static long s32_acquisition_timer = 0L;
   static unsigned long u32_lock = 0UL;
   char s8_value = 0;
   int ret_val = SAL_NOT_FINISHED;

   // If not yet ready
   if(BGVAR(s16_DisableInputs) & BISSC_DIS_MASK)
   {
      state = 0;
      return NOT_AVAILABLE;
   }

   // Do not continue if BiSS-C is not in use
   if(!FEEDBACK_BISSC && (SFB_BISSC && !IS_SECONDARY_FEEDBACK_ENABLED))
   {
      state = 0;
      return NOT_AVAILABLE;
   }

   if (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
      /* Release BiSS-C device */
      BiSSC_Release(&u32_lock, drive);
      u32_lock = 0;
      s32_acquisition_timer = 0;
      state = 0;
      return NOT_AVAILABLE;
   }

   switch(state)
   {
      case 0:// take acquisition timer
         s32_acquisition_timer = Cntr_1mS;
         state++;
      break;

      case 1:// Acquire the BiSS-C device
         ret_val = BiSSC_Acquire(&u32_lock, drive);
         if (ret_val == SAL_SUCCESS)
         {
            ret_val = SAL_NOT_FINISHED;
            state++;
            break;
         }
         if (PassedTimeMS(BISSC_ACQ_TIMEOUT_mSEC, s32_acquisition_timer))
            ret_val = BISSC_DRV_ACQ_TIMEOUT;
      break;

      case 2:
         ret_val = BiSSC_Request(request, u32_lock, u16_addr_id, u16_value, &s8_value, drive);
         if (ret_val == SAL_SUCCESS)
            *p_s16_response = s8_value;
      break;

      default:
         ret_val = BISSC_DRV_ERROR;
      break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {  // Release BiSS-C device
      BiSSC_Release(&u32_lock, drive);
      u32_lock = 0;
      s32_acquisition_timer = 0;
      state = 0;
   }

   return ret_val;
}


//**********************************************************
// Function Name: BiSSC_WriteRegister
// Description:
//   This function writes a value to the specified register address on the BiSS-C slave device.
//    Returns SAL_NOT_FINISHED till the execution is finished.
//    SAL_SUCCESS - on success.
//
// Author: Anatoly Odler
// Algorithm:  State Machine
// Revisions:
// Notes:
//**********************************************************
int BiSSC_WriteRegister(unsigned int u16_addr, unsigned int u16_value, int drive)
{
   int s16_dummy = 0;
   return(BiSSC_CntrlChRequest(BISSC_DRVREQ_WRITE, u16_addr, u16_value, &s16_dummy, drive));
}


//**********************************************************
// Function Name: BiSSC_ReadRegister
// Description:
//   This function reads a value from the specified register address on the BiSS-C slave device.
//    Returns SAL_NOT_FINISHED till the execution is finished.
//    SAL_SUCCESS - on success.
//
// Author: Anatoly Odler
// Algorithm:  State Machine
// Revisions:
// Notes:
//**********************************************************
int BiSSC_ReadRegister(unsigned int u16_addr, int* p_s16_response, int drive)
{
   return(BiSSC_CntrlChRequest(BISSC_DRVREQ_READ, u16_addr, 0x0000, p_s16_response, drive));
}


//**********************************************************
// Function Name: BiSSC_SendCommand
// Description:
//   This function sends a generic command to the BiSS-C slave device.
//    Returns SAL_NOT_FINISHED till the execution is finished.
//    SAL_SUCCESS - on success.
//
// Author: Anatoly Odler
// Algorithm:  State Machine
// Revisions:
// Notes:
//**********************************************************
int BiSSC_SendCommand(unsigned int u16_cmd, int drive)
{
   int s16_dummy = 0;
   return(BiSSC_CntrlChRequest(BISSC_DRVREQ_COMMAND, u16_cmd, 0x0000, &s16_dummy, drive));
}


//**********************************************Local Utilities*****************************************************//


//**********************************************************
// Function Name: unsigned char BiSSC_UpdateCRCBits(unsigned char u8_data, unsigned char u8_crc, unsigned int bits)
// Description:
//   This utility updates the CRC remainder for a given number of bits.
//   The Polynomial used is standard 4-bit BiSS-C Control channel polynomial of 0x13 reversed, i.e. 0x0C
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:
//**********************************************************
unsigned char BiSSC_UpdateCRCBits(unsigned char u8_data, unsigned char u8_crc, unsigned int bits)
{
   register unsigned int bit;

   u8_crc ^= u8_data;

   for(bit = bits; bit > 0; bit--)
   {
      if (u8_crc & 0x01)
         u8_crc = (u8_crc >> 1) ^ BISSC_CNTRL_CH_CRC_POLY;
      else
         u8_crc >>= 1;
   }

   return (u8_crc & 0x0F);
}


//**********************************************************
// Function Name: unsigned char BiSSC_Reverse8(register unsigned char u8_data)
// Description:
//   This utility reverses the bits order for a given unsigned char value.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:
//**********************************************************
unsigned char BiSSC_Reverse8(register unsigned char u8_data)
{
   u8_data = ((u8_data & 0xF0) >> 4) | ((u8_data & 0x0F) << 4);
   u8_data = ((u8_data & 0xCC) >> 2) | ((u8_data & 0x33) << 2);
   u8_data = ((u8_data & 0xAA) >> 1) | ((u8_data & 0x55) << 1);
   return u8_data;
}


//**********************************************************
// Function Name: unsigned int BiSSC_Reverse16(register unsigned int u16_data)
// Description:
//   This utility reverses the bits order for a given unsigned int value.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:
//**********************************************************
unsigned int BiSSC_Reverse16(register unsigned int u16_data)
{
   u16_data = (((u16_data & 0xaaaa) >> 1) | ((u16_data & 0x5555) << 1));
   u16_data = (((u16_data & 0xcccc) >> 2) | ((u16_data & 0x3333) << 2));
   u16_data = (((u16_data & 0xf0f0) >> 4) | ((u16_data & 0x0f0f) << 4));
   u16_data = (((u16_data & 0xff00) >> 8) | ((u16_data & 0x00ff) << 8));
   return u16_data;
}

