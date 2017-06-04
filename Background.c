#include <DSP2834x_Device.h>
#include <DSP2834x_EQep.h>
#include <Extrn_Asm.var>
#include <402fsa.h>
#include <utility.h>
#include <timer.h>
#include <nmterr.h>
#include <cpu_320.h>
#include <objects.h>
#include <nmt.h>
#include <heartbt.h>

#include <AutoTune.pro>
#include <AutoTune.def>
#include <Hiface.def>
#include <MultiAxis.def>
#include <FltCntrl.def>
#include <ModCntrl.def>
#include <SysInit.def>
#include <Init.def>
#include <Display.def>
#include <design.def>
#include <FPGA.def>
#include <i2c.def>
#include <ExFbVar.def>
#include <Err_Hndl.def>
#include <Flash.def>
#include <Modbus_comm.def>
#include <CommFdbk.def>
#include <PtpGenerator.def>
#include <Exe_IO.def>
#include <Homing.def>
#include <402fsa.def>
#include <ModCntrl.def>
#include <BiSS_C.def>

#include <AutoTune.var>
#include <Background.var>
#include <MotorSetup.var>
#include <Init.var>
#include <Drive.var>
#include <Display.var>
#include <Foldback.var>
#include <User_Var.var>
#include <Motor.var>
#include <i2c.var>
#include <Modbus_Comm.var>
#include <An_Fltr.var>
#include <ExFbVar.var>
#include <Velocity.var>
#include <Units.var>
#include <Lxm_profile.var>
#include <PtpGenerator.var>
#include <FltCntrl.var>
#include <Ser_Comm.var>
#include <Hiface.var>
#include <Homing.var>
#include <FlashHandle.var>
#include <Position.var>
#include <CommFdbk.var>
#include <ModCntrl.var>
#include <BiSS_C.var>

#include <Prototypes.pro>
#include <Lxm_profile.pro>
#include <BiSS_C.pro>
#include <Exe_IO.var>

int drive = 0;
int axis_offset = 0;

void Background(void)
{
    long long s64_i;
    //int drive = 0;
    int s16_control = 1;
    //AXIS_OFF;
    //  Enable Watch Dog mechanism in FPGA
    *((int*)FPGA_WATCH_DOG_ENABLE_REG_ADD) = 0x01;
    s32_Display_Sync_Timer = Cntr_1mS;
    BGVAR(s32_Current_Derating_Timer) = Cntr_1mS;
    s16_Prev_Counter_3125 = Cntr_3125;
    //  Mark that background ran
    u16_background_ran = 1;

    //  to defeat compiler remark. Originally it was "while(1)"
    while (s16_control == 1)
    {
        //  Uncomment as required...
        //TimeStampBgGet(0);
        s16_Bg_Watchdog = 3200;
        s16_LMJREstSimplexState = LMJREstimationSimplex(drive,BGVAR(s16_Lmjr_Simplex_Reset));

        // if during AT re-run to make lmjr est during AT faster - at the expense of high BG load
        if ((VAR(AX0_s16_Cycle_Bits) & POS_TUNE_RT_ACTIVE) != 0)
        {
            s16_LMJREstSimplexState = LMJREstimationSimplex(drive,BGVAR(s16_Lmjr_Simplex_Reset));
            s16_LMJREstSimplexState = LMJREstimationSimplex(drive,BGVAR(s16_Lmjr_Simplex_Reset));
            s16_LMJREstSimplexState = LMJREstimationSimplex(drive,BGVAR(s16_Lmjr_Simplex_Reset));
            s16_LMJREstSimplexState = LMJREstimationSimplex(drive,BGVAR(s16_Lmjr_Simplex_Reset));
        }
        if( u8_Is_Retro_Display == 0 )
        {
           LocalHmiControl(drive);
           HmiManager(drive);
        }
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        //  Avoid updating RUNtime if AT is in process. BZ5413
        PosTuneActiveCommand(&s64_i,drive);
        if((int)s64_i == 0)
        {
            //  update BG clock
            RunTimeHandler();
        }
        ScanI2CDevices();
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        //  Perform Modbus Comm Tasks
        ModbusProcessor(0);
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        //  Perform ModeCntrl Tasks
        ModeControl(0);
        //  limit the current as function of IPM temperature
        CurrentDerating(0);
        //  limit current according to analog input current limit and active disable current limit
        UpdateCurrentLimit(0);
        //  Foldback
        Foldback(0);
        SetCurrentCommandSaturation(0);
        ScanI2CDevices();
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
        //  call multiple time so that state machine will execute at a higher rate
        CommsProcessor(0);
        //  burnin task
        BurninHandler(0);
        //  Inrush Handler
        InrushHandler();
        //  Uncomment as required...
        //TimeStampBgGet(1);

        //  if COMMODE = 1 - do not allow to enable drive later when COMMODE = 0 (when SWENMODE = 1)
        if((0 == u16_SwEnMode_Drive_Enabled) && (BGVAR(s16_DisableInputs) & FB_EN_MASK))
        {
            u16_SwEnMode_Drive_Enabled++;
        }

        //  Allow SWENMODE to enable the drive only after Current Sensor Adjust, and only once
        if ((0 == u16_SwEnMode_Drive_Enabled)  && (1 == BGVAR(u8_SwEnMode)) && (BGVAR(s16_DisableInputs) == SW_EN_MASK))
        {
            BGVAR(s16_DisableInputs) &= ~SW_EN_MASK;
            u16_SwEnMode_Drive_Enabled++;
        }

        //  Uncomment as required...
        //TimeStampBgGet(2);
        //  Performs CANopen tasks
        if (IS_CAN_DRIVE_AND_COMMODE_1)
        {
            //if we have a valid cob id or cobid is 0 but length of message is 2 (NMT - reset communication & reset application)
            if ((CAN_Msg_BG_buffer[CAN_Msg_BG_buffer_rd_index].cobId != 0) || (CAN_Msg_BG_buffer[CAN_Msg_BG_buffer_rd_index].length == 2))
            {
                //  let CAN handle the message
                msgIdentificationBg(BACKGROUND_CONTEXT, &CAN_Msg_BG_buffer[CAN_Msg_BG_buffer_rd_index]);
                //  delete the message that was just handled
                FalResetCanMsg();
                //  increment read pointer (cyclic buffer)
                CAN_Msg_BG_buffer_rd_index = (((CAN_Msg_BG_buffer_rd_index + 1) & (CAN_SDO_BUFFER_SIZE - 1)));
            }

            //  only for can drive. Try to clear bus off state
            ClearBusOff();
        }

        //  Performs fieldbus tasks (CANopen,EtherCAT)
        if ((IS_CAN_DRIVE_AND_COMMODE_1  ||   // CAN board
             IS_PN_DRIVE_AND_COMMODE_1   ||   // Profy board
            (IS_EC_DRIVE_AND_COMMODE_1   && (u16_Load_Micro_Blaze_Exit_Code >= 90))))  // EtherCAT board and micro Blaze firmware was downloaded successfully
        {
            if (BGVAR(u16_CAN_BootUp_Requests_Counter) > 0)
            {
                //  Fix IPR 520 CANOpen: EMCY message is missing.
                CAN_SendBootUpMsg(BACKGROUND_CONTEXT, drive, 0);
            }

            //  Validate is can drive and commode and not in operational mode
            if ((IS_CAN_DRIVE_AND_COMMODE_1) &&
                (GL_ARRAY(co_Node).eState != OPERATIONAL))
            {
                u16_Is_Pdo_After_Operation_Mode = 0;
            }

            if (IS_EC_DRIVE_AND_COMMODE_1)
            {
               //  For ASCII command ECREADCOMMSTATE
               BGVAR(u16_Ec_Nmt_State) = (*(unsigned int*)p_u16_tx_nmt_state);

               //  validate not in operational mode
               if (*p_u16_tx_nmt_state < EC_NMTSTATE_OP)
               {
                  u16_Is_Pdo_After_Operation_Mode = 0;
               }

               //  on boot state test if Microblaze is trying to send a file for download
               if ((*p_u16_tx_nmt_state == EC_NMTSTATE_BOOTSTRAP) || (*p_u16_tx_nmt_state == EC_NMTSTATE_PREOP))
               {
                  if ((*p_u16_tx_foe_active == 0xAA55) && (*p_u16_tx_nmt_state == EC_NMTSTATE_BOOTSTRAP))
                  {
                     EmberCommand();
                  }
                  else if (*p_u16_tx_foe_active == 0xCAFE)
                  {
                     // start FOE params file state machine
                     BGVAR(u16_Foe_ParamsFile_State) = FOE_PARAMS_FILE_DWLD_INIT;
                     if (s16_Comms_Processor_State == PRE_PROCESSOR)
                     {
                        // params file - signal uBlaze that DSP is ready
                        SET_DPRAM_FPGA_MUX_REGISTER(0);
                        *p_u16_tx_foe_active = 0xFECA;   // Indicate to the micro Blaze to continue with the process
                     }
                  }
                  else if (*p_u16_tx_foe_active == 0xFECA)
                  {
                     FoeParamsFileDownload();
                  }
               }                
            }
            else
            {
                //  For ASCII command ECREADCOMMSTATE
                BGVAR(u16_Ec_Nmt_State) = 0xFFFF;
            }

            FalExecuteFieldBusBgRxObject(BACKGROUND_CONTEXT);
            //  change opmode if requested by RPDO, immediatly update status word for new opmode.
            CheckCanOpmode(0);
            UpdateStateMachine(BACKGROUND_CONTEXT, 0);
            UpdateTargetReachedBit(0);
            //  handle save/load requests from SDOs
            HandleEepromOperations();
            //  update tpdos mapping if needed (support for event TPDOs)
            Can_UpdatePdoTable(0);
            //  perform parameter change if received by RPDO
            CheckCanAcc(0);
            CheckCanDec(0);
            CheckCanGearIn(0);
            CheckCanGearOut(0);
            CheckCanDigitalOutputs(0);
        }
        else
        {
            //  For ASCII command ECREADCOMMSTATE
            BGVAR(u16_Ec_Nmt_State) = 0xFFFF;
        }

	    //  Disable PCOM upon feedback fault
	    if (((IS_PCOM1_EN)||(IS_PCOM2_EN)) && ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0) ||((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0))   
	    {
	        PcomDisable(1);
	        PcomDisable(2);
	    }
        
        ScanI2CDevices(); 
        //  Uncomment as required...
        //TimeStampBgGet(3);
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        //  PhaseFinding bg task
        PhaseFindHandler(0);
        //  Zeroing task
        ZeroingHandler(0);
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
		
		ScanI2CDevices();

        //  Run EnDat Handler for EnDat Comm.-Only or for EnDat with Sine-Signals
        if(FEEDBACK_ENDAT)
        { 
            if (BGVAR(u16_Fdbk_Source) == 0)
            {
                //  EnDat Encoder connected at C4, processed as Motor Encoder
                EndatHandler(0, 0, 0);        
            }
            if (BGVAR(u16_Fdbk_Source) == 1)
            {
                //  EnDat Encoder connected at C3, processed as Motor Encoder
                EndatHandler(0, 1, 0);        
            }
        }
        //  Modify line above to allow SFBTYPE 3 SFBENCTYPE 9 for Load Feedback at C4.
        else if (BGVAR(u16_SFBType) == ENDAT2X_COMM_FDBK)
        {
            if (BGVAR(u16_Fdbk_Source) == 0)
            {
                //  EnDat Encoder connected at C3, processed as Load Encoder
                EndatHandler(0, 1, 1);        
            }
            if (BGVAR(u16_Fdbk_Source) == 1)
            {
                //  EnDat Encoder connected at C4, processed as Load Encoder
                EndatHandler(0, 0, 1);        
            }
        }

        //  Hiperface handler
        HifaceHandler(0);
        //  Communication position (Primary) feedback bg handler
        CommPosFdbkHandler(0, BGVAR(u16_Fdbk_Source), 0);      
        // Comm.-Pos. (Secondary) feedback bg handler
        CommPosSecFdbkHandler(0, (1 - BGVAR(u16_Fdbk_Source)), 1);  
        //  ServoSense driver manager task
        SrvSns_Manager(0);
        ScanI2CDevices();

        //  Run BiSS-C manager task
        if (FEEDBACK_BISSC)
           BiSSC_BGManager(0, BGVAR(u16_Fdbk_Source), 0);
        else if (SFB_BISSC)
           BiSSC_BGManager(0, (1 - BGVAR(u16_Fdbk_Source)), 1);

        if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_FWDNLD_RETRANSMIT_MASK)
        {
            SrvSns_FWDnLdProcessorBG(0);
        }

        //  call for DisplayControl was removed for CDHD2 
        if (u8_Is_Retro_Display == 1)
        {
            //  after fault cntrl so no junk at power up
            DisplayControl(0);
        }

        //  Transfer BG data between two axes of DDHD */
        if (u16_Product == DDHD)
        {
            TransferDataBetweenDDHDAxes();
        }

        //  Uncomment as required...
        //TimeStampBgGet(4);
        NLTuneHanlder(0);
        //  support homing
        HomingHandler(0);
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        GainInterpolationByLMJR(0);
        ScanI2CDevices(); 
        //  Init comm if in error condition.
        //  This is to work around Dell laptop RS-232 port problem
        if (ScibRegs.SCIRXST.bit.RXERROR == 1)
        {
            InitSerCommunication();
        }

        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        ScanI2CDevices();
        FanHandler();
        StepHandler(0);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        TorqueStepHandler(0);
        MotionBufferHandler(0);
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
        //TimeStampBgGet(5);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        ScanI2CDevices();
        MotorSetupHandler(0);
        PllHandler(0);
        SensrolessHandler(0);
        HoldScan(0);
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        ScriptHandler(0);
        UpdatePcomOutputs();
        PathContRunHandler(0);
        //  motor parameters estimation
        MotorParametersEstimationHandler(0);
        PfbBackupFeatureReadSupport(0);
        PfbBackupFeatureWriteSupport(0);
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
		ScanI2CDevices();
        //  Uncomment as required...
        //TimeStampBgGet(6);
        AutoHcHandler(0);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        AutoHomeHandler(0);
        //  call multiple time so that state machine will execute at a higher rate
        AutotTuneHandlerV2(0);
        //  Perform Serial Comm Tasks
        CommsProcessor(0);
        ScanI2CDevices();
        EncSimTestModeHandler(0);

        //  Check if init from MTP is needed
        if ((BGVAR(u16_MTP_Mode) && BGVAR(u16_Init_From_MTP)) || (BGVAR(u16_Read_Mtp_State) != MTP_INIT_INIT))
        {
            if (InitFromMTP(drive) != SAL_NOT_FINISHED)
            {
                BGVAR(u16_Init_From_MTP) = 0;
                //  Zero Bundle-Data Indicator to restart Bundle-Data Search
                BGVAR(s16_Bundle_Design_Lock) = -1;
            }
        }
        // If MTP not used clear the MTP fault indication and bundle check fault indication
        else
        {
            if (!BGVAR(u16_MTP_Mode))
            {
                // Supress MTP read failure
                BGVAR(s16_DisableInputs) &= ~MTP_READ_DIS_MASK;
                BGVAR(u16_Init_From_MTP) = 0;
                BGVAR(s16_MTP_Error) = 0;
                // Supress bundle check failure
                BGVAR(s16_Motor_Drive_Mismatch) = 0;
            }
        }

        // CDHD2 ECAT HMI Handler
        if (IS_EC2_DRIVE_KEYPAD)
        {
            LocalHmiUpdateKeyStates(drive);
        }

        //  Uncomment as required...
        //TimeStampBgGet(7);
        ReadServoSenseTemperature(drive,0);
        SrvSns_HelpTaskBG(drive, 0);

        // Special function for Frencken, which is an autonomous process for a certain voltage correction feature
        SFBVoltageCorrectionCalibrationProcess(drive);
        //Voltage correction feature for Frencken
        BGVAR(s16_Voltage_Correct_2_Result) = GetAnalogInput2ValueCorrected(drive);
        
        //  Check if the Drive is currently in regular operation
        if ((LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) != &VAR(AX0_s16_Burnin_Command))     &&
            (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) != &BGVAR(s16_Motor_Setup_Current))  &&
            (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) != &VAR(AX0_s16_Wns_Crrnt_Cmnd))     &&
            (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) != &VAR(AX0_s16_Vforced_Torque_Cmd)) &&
            (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) != &VAR(AX0_s16_Zero_Crrnt_Cmnd)))
        {
            //  Drive is in regular operation
            BGVAR(u8_CDHD_Normal_Operation) = 1;
        }
        else
        {
            //  Drive does a specific action and is not in regular operation
            BGVAR(u8_CDHD_Normal_Operation) = 0;
        }

        //  Uncomment as required...
        //TimeStampBgGet(8);
        //  Check is restart needed because of work on 5V from =S= RJ45 (In Box Programming)
        CheckRestart();

        s32_Bg_Counter++;

        //  This is for Debug of the modbus infrastructure
        //  Set the modbus index value in u16_debug_p_number, and get the result in s32_debug_result

        //  P-Parameter reading-writing operation:
        //  Set Debug_Write to 1 for reading, 2 for writing; the Function will return the value to Zero.
        //  Temp Only !! Should be removed, or fix to handle: decimal point, SAL_NOT_FINISH and more.
        if (u16_debug_read_write > 0)
        {
            u16_debug_ret_val = ExecutePParam(0, u16_debug_p_number, u16_debug_read_write, &s32_debug_result, LEX_CH_MODBUS_RS485);
            if (u16_debug_ret_val != SAL_NOT_FINISHED)
            {
                u16_debug_read_write = 0;
            }
        }

        //  Inhibit encoder simulation if PFB did not init
        if (BGVAR(u8_EncoutMode) && (BGVAR(s16_DisableInputs) & (INVALID_FDBK_MASK | MTP_READ_DIS_MASK | ENC_STATE_DIS_MASK | ENDAT_DIS_MASK | HIFACE_DIS_MASK | COMM_FDBK_DIS_MASK | RESOLVER_FDBK_DIS_MASK)))
        {
            VAR(AX0_u16_Encsim_Freq_Flags) |= INHIBIT_ENC_SIM_PFB_NOT_READY_MASK;
        }
        else
        {
            VAR(AX0_u16_Encsim_Freq_Flags) &= ~INHIBIT_ENC_SIM_PFB_NOT_READY_MASK;
        }

        //  Avoid the Encoder-Testing State-Machine if not starting with ServoSense...
        if (BGVAR(s16_Auto_FdbkType_Cntr) > 0)
        {
            if (FEEDBACK_SERVOSENSE)
            {
                //  If feedback state machine got in the error and the FW status indicates that
                //  communication has not been established, then proceed to Hiperface
                if ((BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_ERROR_STATE) &&
                    ((VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PRESENT_MASK) == 0))
                {
                    //  Sine Encoder...
                    BGVAR(u16_FdbkType) = SW_SINE_FDBK;
                    //  Hiperface Feedback
                    VAR(AX0_s16_Motor_Enc_Type) = 10;
                    //  Motor Type Plate for Hiperface
                    BGVAR(u16_MTP_Mode) = 3;
                    MtpModeInit(BGVAR(u16_MTP_Mode), drive);
                    BGVAR(u16_Init_From_MTP) = 1;
                    FdbkTypeCommand((long long)BGVAR(u16_FdbkType), drive);
                    DesignRoutine(1, DRIVE_PARAM);
                    BGVAR(s16_Auto_FdbkType_Cntr) = 2;
                }
                // if fdbk state machine finished OR communication with ServoSense has been established
                else if ((BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_IDLE_STATE) ||
                        ((VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PRESENT_MASK) != 0))
                {
                    BGVAR(u16_FdbkType_User) = BGVAR(u16_FdbkType);
                    BGVAR(u16_MTP_Mode_User) = BGVAR(u16_MTP_Mode);
                    BGVAR(s16_Auto_FdbkType_Cntr) = 0;
                }
            }
            if ((BGVAR(u16_FdbkType) == SW_SINE_FDBK) && (VAR(AX0_s16_Motor_Enc_Type) == 10))
            {
                //  Successful Initialization of Hiperface Feedback, including reading of MTP Data
                if (BGVAR(u16_Hiface_Init_State) == HIFACE_IDLE_STATE)
                {
                    BGVAR(u16_FdbkType_User) = BGVAR(u16_FdbkType);
                    //  Verify MTP Reading Status
                    MtpStatusCommand(&s64_i, drive);
                    //  MTP Reading ended successfully
                    if ((s64_i == 1) && (BGVAR(u16_Init_From_MTP) == 0))
                    {
                        BGVAR(u16_MTP_Mode_User) = BGVAR(u16_MTP_Mode);
                        BGVAR(s16_Auto_FdbkType_Cntr) = 0;
                    }
                    //  MtP-Mode is not used, or MTP Reading failed, or MTP-Mode requires Power-Cycle
                    else if (((s64_i == 0) || (s64_i == 2) || (s64_i == 3) ) && (BGVAR(u16_Init_From_MTP) == 0))
                    {
                        //  Zero MTPMODE to allow Initialization
                        BGVAR(u16_MTP_Mode_User) = BGVAR(u16_MTP_Mode) = 0;
                        //  with no MTP-related messages.
                        BGVAR(s16_Auto_FdbkType_Cntr) = 0;
                    }
                }
                //  Failure to Initialize Hiperface as well, incorrect or disconnected Feedback
                else if (BGVAR(u16_Hiface_Init_State) == HIFACE_INIT_ERROR_STATE)
                {
                    BGVAR(u16_MTP_Mode_User) = BGVAR(u16_MTP_Mode) = 0;
                    //  Moved to FltCntrl.c Fault Handling...
                    BGVAR(s16_Auto_FdbkType_Cntr) = -1;
                }
            }
        }

        //  Uncomment as required...
        //TimeStampBgGet(9);
        //  If MTP Mode in effect and successful completion of Auto-Detect or No Auto-Detect...
        if ((BGVAR(u16_MTP_Mode) != 0) && (BGVAR(s16_Auto_FdbkType_Cntr) == 0) &&
            (BGVAR(s16_Bundle_Design_Lock) == -1) && (BGVAR(u32_MotorFileName) != 0))
        {
            //  Search Bundle Data
            BGVAR(s16_Bundle_Design_Lock) = SearchBundleCurrentTuning(drive);
         
            if ((BGVAR(s16_Bundle_Design_Lock) >= 0) && (BGVAR(s16_Bundle_Design_Lock) <= CDHD_PRO2_MOTOR_DATA_ENTRIES))
            {
                BGVAR(s32_CL_Bemf_Gain) = s_Pro2_CDHD_Bundle_Data[BGVAR(s16_Bundle_Design_Lock)].kcbemf;
                BGVAR(u32_Kcd_Gain)     = s_Pro2_CDHD_Bundle_Data[BGVAR(s16_Bundle_Design_Lock)].kcd;
                BGVAR(s32_CL_Kff_User)  = s_Pro2_CDHD_Bundle_Data[BGVAR(s16_Bundle_Design_Lock)].kcff;
                BGVAR(s32_CL_Ki_User)   = s_Pro2_CDHD_Bundle_Data[BGVAR(s16_Bundle_Design_Lock)].kci;
                BGVAR(s32_CL_Kp_User)   = s_Pro2_CDHD_Bundle_Data[BGVAR(s16_Bundle_Design_Lock)].kcp;

                CrrntConfig(DRIVE_PARAM);

                //  If a factory-restore has previously happened and a bundle has been identified,
                //  set OPMODE to 8 according to Bugzilla item 5037, but not for a Schneider Drive.
                if (BGVAR(u16_Post_Factory_Restore_Actions) & POST_FACT_REST_SET_OPMODE_UPON_BUNDLE)
                {
                    SalOpmodeCommand(8LL, drive);
                    BGVAR(u16_Post_Factory_Restore_Actions) &= ~POST_FACT_REST_SET_OPMODE_UPON_BUNDLE;
                }
            }
        }

        // Validate Feedback Configuration is in Process and if it is we want all these Inputs to be finished
        if ((BGVAR(u16_Feedback_Config_Process_Active)) &&
            (BGVAR(s16_DisableInputs) & (COMM_FDBK_DIS_MASK | BISSC_DIS_MASK | RESOLVER_FDBK_DIS_MASK | HIFACE_DIS_MASK | INVALID_FDBK_MASK | ENDAT_DIS_MASK | ENC_STATE_DIS_MASK)) == 0)
        {
            // Reset Feedback Configuration in Process
            BGVAR(u16_Feedback_Config_Process_Active) = FALSE;
            // Initialize the Modulo Mode State Machine
            InitializeModuloModeStateMachine(drive);
        }

        //  Uncomment as required...
        //TimeStampBgGet(10);

        //  THIS SHOULD BE AT THE END OF THE BACKGROUND LOOP
        //  Measure the background time in 31.25 uS counts
        s16_Bg_Time_3125 = Cntr_3125 - s16_Prev_Counter_3125;
        s16_Prev_Counter_3125 = Cntr_3125;

        if (s16_First_Bg_Time_3125 == 0)
        {
            s16_First_Bg_Time_3125 = s16_Bg_Time_3125;
        }

        if (s16_Bg_Time_3125 > s16_Max_Bg_Time_3125)
        {
            s16_Max_Bg_Time_3125 = s16_Bg_Time_3125;
        }

        if (s16_Bg_Time_3125 < s16_Min_Bg_Time_3125)
        {
            s16_Min_Bg_Time_3125 = s16_Bg_Time_3125;
        }
        // DO NOT ADD CODE AFTER THIS
    }
}

// Uncomment "TimeStampBgGet(n)" Statements in Background Function to measure Background Time Intervals
void TimeStampBgGet(unsigned int u16_stamp_location)
{
   if (u16_Bg_First_Time_Stamp == u16_stamp_location)
      u16_Bg_1st_Stamp_Time_3125 = Cntr_3125;
   else if (u16_Bg_Last_Time_Stamp == u16_stamp_location)
   {
      u16_Bg_Interval_Time_3125 = Cntr_3125 - u16_Bg_1st_Stamp_Time_3125;
      if (u16_Bg_Interval_Time_3125 > u16_Bg_Max_Interval_Time_3125)
         u16_Bg_Max_Interval_Time_3125 = u16_Bg_Interval_Time_3125;
      if (u16_Bg_Interval_Time_3125 <u16_Bg_Min_Interval_Time_3125)
         u16_Bg_Min_Interval_Time_3125 = u16_Bg_Interval_Time_3125;
   }
}


#pragma CODE_SECTION(ONE_MSEC_ISR, "ramfunc_4");
interrupt void ONE_MSEC_ISR(void)     // Interrupt 11.1
{
    int drive = 0;
    // AXIS_OFF;

    unsigned long u32_tmp = 0;
    unsigned int u16_sto_flt_temp;
    float f_temp;

    //  enable  WD_ISR, MTS_ISR, Main CAN ISR (CAN_int_ecan), CAN_TIME_STAMP_ISR & ADC_ISR
    IER = (M_INT1 | M_INT3 | M_INT5 | M_INT9 | M_INT12);
    EINT;

    //s16_Time_1Ms = Cntr_3125;

    //  Continuous Counting of DC Bus Voltage Signal...
    //  Keep at beginning of ISR!!!
    u16_Vbus_Running = (unsigned int)EQep2Regs.QPOSCNT;
    //  Clear pending interrupt
    EPwm9Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

    //  4msec
    s16_1ms_Isr_Watchdog = 16;
    Cntr_1mS++;
    //  trigger the BG ISR every 1ms
    PieCtrlRegs.PIEIFR10.bit.INTx8 = 1  ;

    u16_Vbus_Counts = u16_Vbus_Running - u16_Vbus_RunningPrev;
    u16_Vbus_RunningPrev = u16_Vbus_Running;
    u16_Vbus_HW_Measure_Flt = 0;

    if (u16_Vbus_Counts < 194)
    {
        if (u16_Vbus_Counts < 50)
        {
            u16_Vbus_HW_Measure_Flt = 1;
        }
        BGVAR(u16_Vbus_Volts) = 0;
        AX0_s16_Vbus_RT_Factor = 16384;
    }
    else
    {
        u32_tmp = ((u16_Vbus_Counts - 194) * (BGVAR(u16_Vbus_Scale) / 256));
        u32_tmp = ((u32_tmp * 20480) >> 16);
        BGVAR(u16_Vbus_Volts) = (unsigned int)u32_tmp;

        //  calc vbus factor
        if (BGVAR(s16_Adaptive_Vbus) == 1)
        {
            f_temp = (float)BGVAR(u16_Vbus_Volts);

            if (f_temp < (float)BGVAR(s16_Vbus_Design))
            {
                f_temp = (float)BGVAR(s16_Vbus_Design);
            }

            AX0_s16_Vbus_RT_Factor = (int)(16384.0*(float)BGVAR(s16_Vbus_Design)/f_temp);
        }
    }

    //  Check Over-Voltage Condition here at 1mSec. Response Time to ensure fast response
    if (BGVAR(u16_Vbus_Volts) > u16_OV_Threshold)
    {
        //  Delay as much as set by OVDELAY...
        if (BGVAR(u16_Ov_Delay_Cntr) < BGVAR(u16_Ov_Delay))
        {
            BGVAR(u16_Ov_Delay_Cntr)++;
        }
        //  One-Pass to set Internal
        else if (BGVAR(u16_Ov_Delay_Cntr) == BGVAR(u16_Ov_Delay))
        {
            //  Indication and cause disabling of PWM Outputs.
            BGVAR(u16_Ov_Delay_Cntr)++;
            BGVAR(u16_Ov_Fast_Exist) = 1;
            //  Signal the real-time to disable the PWM
            VAR(AX0_s16_Transition_To_Disable) = 1;
        }
    }
    //  Reset Internal Indication and Delay Counter
    else
    {
        BGVAR(u16_Ov_Fast_Exist) = 0;
        BGVAR(u16_Ov_Delay_Cntr) = 0;
    }

    //  check if feedback communication error occured, and disable drive immediatly to avoid jumps due to false pfb
    if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_TIME_OUT_ERROR_MASK ) != 0)
    {
        BGVAR(s64_RT_Faults) |= COMM_FDBK_FLT_MASK;
        //  Signal the real-time to disable the PWM
        VAR(AX0_s16_Transition_To_Disable) = 1;
    }
    else
    {
        BGVAR(s64_RT_Faults) &= ~COMM_FDBK_FLT_MASK;
    }

    //  Check for runaway condition
    RunawayDetection(drive);
    CheckGearInPos(drive);
    GetExternalEncDelta(drive);
    PowerSupplyFilter(drive);
    MotorTempFilter(drive);
    RegenHandler(drive);

    if ((VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_FWDNLD_IDLE_MASK) == 0)
    {
        RtRecordOutputProcessor();
        SciInputProcessor();
        SciModbusInputProcessor(); //Udi
        SciOutputProcessor();
        SciModbusOutputProcessor(); //Udi
    }
    else
    {
        if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_FWDNLD_RETRANSMIT_MASK)
        {
            SciInputProcessor();
            SciOutputProcessor();
        }
    }

    CalcEqCurrent(drive);
    CheckReducePwmFreq(drive);
    WriteAnalogOutputToDAC(drive);
    LimitSwitchHold(drive);
    MotorMoving(drive);
    MotorStoppedBlock(drive);
    //  CAN timer
    co_timerIsr();

    //  check semaphore
    if (s16_Background_Sdo_Semaphore == 0)
    {
        //  moved from can ISR. this function need to be executed cyclicly in order to determine if the canopen timer have elapsed.
        //  original location was in flagIdentification(), which is called only when a can packet is received.
        checkTimerEvent(INT_1MSEC_CONTEXT CO_LINE_PARA);
        //  check if there are event TPDOs that should be sent out.
        Can_TpdoSender(INT_1MSEC_CONTEXT, drive);

        if (s16_Can_NodeGuard_Recieved)
        {
            s16_Can_NodeGuard_Recieved = 0;
            NMT_NodeGuardingMsg(INT_1MSEC_CONTEXT CO_LINE_PARA);
        }
    }
    else
    {
        s16_Can_Semaphore_Busy++;
    }

    //  nitsan - fix timers watchdog. call addTimerEvent() in 1ms ISR insterad of CAN ISR to avoid timers list data corruption: NULL termination was overwritten and causing endless loop in checkTimerEvent()
    if (BGVAR(s16_CAN_Flags) & CANOPEN_ADD_TIMER_IND_MASK)
    {
        //  pointer to node struct
        HB_CONS_T *pHBCons;
        BGVAR(s16_CAN_Flags) &= ~CANOPEN_ADD_TIMER_IND_MASK;
        pHBCons = &GL_VAR(hbConsList)[BGVAR(u16_HB_ConsList_Idx)];
        addTimerEvent(&pHBCons->timer, pHBCons->timer.timerVal,
                      CO_TIMER_TYPE_HB_CONS CO_COMMA_LINE_PARA);
    }

    IndexDuration(drive);

    //  Filter the STO indication.
    //  STO standard allows drops of 1 ms without considered faults. On the other hand, do not filter too much as OC fault will occur
    //  because STO disconnects the power to it. This is the reason the STO filter was moved to the 1ms ISR.
    u16_sto_flt_temp = *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD);
    u16_sto_flt_temp &= PWR_STO_STAT;

    //  Clear the fault on FPGA
    *(unsigned int *)(FPGA_FAULT_MANAGEMENT_REG_ADD) &= u16_sto_flt_temp;

    //  Translate to 0/1
    u16_sto_flt_temp = (u16_sto_flt_temp > 0);

    // In case of DDHD or SHNDR_HW or new CDHD, get the STO status from GPIO27
    if (((u16_Product == DDHD) && !IS_DDHD_EC2_DRIVE) || IS_CAN2_DRIVE)
    {
        u16_sto_flt_temp = 0;
        if (GpioDataRegs.GPADAT.bit.GPIO27 == 1)
        {
            u16_sto_flt_temp = 1;
        }
    }

    //  Inhibit STO indication if Bit#1 in IGNOREBRKFLT is set
    if ((BGVAR(u16_Ignore_Power_Brake_Fault) & 0x0002) == 0x0002)
    {
        u16_sto_flt_temp = 0;
    }

    //  Filter the STO indication (outcome of the EMI test)
    if (u16_sto_flt_temp == 1)
    {
        //  do not filter too much as OC fault will occur because STO disconnects the power to it
        if (BGVAR(u16_Sto_Indication_Filter) < 2)
        {
            BGVAR(u16_Sto_Indication_Filter)++;
            u16_sto_flt_temp = 0;
        }
    }
    else
    {
        BGVAR(u16_Sto_Indication_Filter) = 0;
    }

    u16_STO_Flt_Indication = u16_sto_flt_temp;

    //  check if sto was out during init - will be used in STO FAST ENABLE feature
    if (0x00ff == u16_STO_Out_On_Init)
    {
        //  sto is out
        if (1 == u16_STO_Flt_Indication)
        {
            u16_STO_Out_On_Init = 1;
        }
        //  sto is in
        else if (0 == u16_Sto_Indication_Filter)
        {
            u16_STO_Out_On_Init = 0;
        }
    }

    HallsValidityCheck(drive);

    /*s16_Time_1Ms = Cntr_3125 - s16_Time_1Ms;
    if (s16_Time_1Ms > s16_Max_Time_1Ms)
    {
        s16_Max_Time_1Ms = s16_Time_1Ms;
    }*/
}

//**********************************************************************************************
// This ISR if for BG tasks that need to be executed in faster rate than the BG time.
// This ISR runs from flash, not from RAM, hence, flash program and erase code disable this ISR.
// This ISR resides in RAM, otherwise the time it takes till it enables other interrupts is too
// long and it affects the motor phase current reading, but the functions it calls reside in flash.
// There is no watchdog protection to this ISR. If this ISR gets stuck, the BG watchdog will occur.
//**********************************************************************************************
#pragma CODE_SECTION(BACKGROUND_ISR, "ramfunc_4");
interrupt void BACKGROUND_ISR(void)     // Interrupt 10.8
{
    //int drive = 0;
    // AXIS_OFF;

    //  enable  WD_ISR, MTS_ISR, Main CAN ISR (CAN_int_ecan), CAN_TIME_STAMP_ISR, ONE_MSEC_ISR & ADC_ISR
    IER = (M_INT1 | M_INT3 | M_INT5 | M_INT9 | M_INT11 | M_INT12);
    EINT;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    u32_Background_Isr_Counter++;

    TouchProbeHandler(0);
    PhaseFindSmoothIncCurr(drive);
    // Handle SFB PE between the load and the motor
    SFBPositionErrorHandler(drive);
    CycleIndetification(drive);
    // OnLineLmjrEst(drive);
    // FIX QC#1249 run PATH handler in 1ms for faster response.
    PathHandler(0);
}


//**********************************************************
// The following two functions meant to test the CheckCalls tool. The tool lists RAM functions that call flash function.
// One function resides in flash, and is called by a function that resides in RAM, just to be detected by CheckCalls.
//**********************************************************
#pragma CODE_SECTION(CheckCallsRAMFunction, "ramfunc_4");
void CheckCallsRAMFunction()
{
   CheckCallsFlashFunction();
}

void CheckCallsFlashFunction()
{
}


//**********************************************************
// Function Name: CheckReducePwmFreq
// Description:
//   Check if PWM should be regular (as specified in the EEPROM) or reduce to half, to prevent over heating.
//   The condition to reduce PWM frequenfy are:
//   1. Indication from the power EEPROM that this feature in needed.
//   2. Current is greater than DICONT.
//   3. Velocity is lower than 100ms per transistor (1/6 of electrical cycle).
//
//   It is possible to force regular PWM frequency or half PWM frequency (for debug).
//
//   This algorithm is executed every 2ms to allow averaging of the current and electrical velocity.
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(CheckReducePwmFreq, "ramfunc_2");
void CheckReducePwmFreq(int drive)
{
   // AXIS_OFF;

   long s32_elect_pos_delta;
   REFERENCE_TO_DRIVE;
   // Don't support PWM reducing for 4 kHz drive
   if(BGVAR(u32_Pwm_Freq) == 4000L) return;
   if (Cntr_1mS & 1L)
   {
      s32_elect_pos_delta = LVAR(AX0_s32_Elect_Pos_With_Phase_Adv) - BGVAR(s32_Prev_Elect_Pos_With_Phase_Adv);
      BGVAR(s32_Prev_Elect_Pos_With_Phase_Adv) = LVAR(AX0_s32_Elect_Pos_With_Phase_Adv);

      if (s32_elect_pos_delta < 0)
         s32_elect_pos_delta = -s32_elect_pos_delta;

      VAR(AX0_u16_Vel_Dsrgrd)=(s32_elect_pos_delta > 1750)?1:0;

      if (s32_elect_pos_delta < 219)
         BGVAR(u16_In_Stall_Flag) = 1;
      else if (s32_elect_pos_delta > 438)
         BGVAR(u16_In_Stall_Flag) = 0;

      if (BGVAR(u16_Force_Regular_Pwm_Freq))
      {
         BGVAR(u16_Pwm_Freq_State) = REGULAR_PWM_FREQ;
      }
      else if (BGVAR(u16_Force_Half_Pwm_Freq))
      {
         BGVAR(u16_Pwm_Freq_State) = HALF_PWM_FREQ;
      }
      else
      {
         if ( (BGVAR(u16_Half_Pwm_Freq_Eeprom))                                           &&   // Indication from the power EEPROM
              ((long)BGVAR(u32_EqCrrnt_Avg_2) > (long)BGVAR(s32_Drive_I_Cont_Internal))   &&   // Current is greater than DICONT
              (s32_elect_pos_delta < 219)                                                   )  // min delta at 2ms for stall condition
         {
            // Delay 10 mS for ignoring a false Stall condition. This condition may occurs at high current step while motor in rest.
            if (BGVAR(u16_Pwm_Freq_State_Cntr) == 5)
               BGVAR(u16_Pwm_Freq_State) = HALF_PWM_FREQ;
            else
               BGVAR(u16_Pwm_Freq_State_Cntr)++;

         }

         if ( ((long)BGVAR(u32_EqCrrnt_Avg_2) < (((long)BGVAR(s32_Drive_I_Cont_Internal)*3)>>2))   ||        // Current is less than DICONT*3/4
              (s32_elect_pos_delta > 438)                                                             )      // twice min delta at 2ms for stall condition
         {
            BGVAR(u16_Pwm_Freq_State) = REGULAR_PWM_FREQ;
            BGVAR(u16_Pwm_Freq_State_Cntr) = 0;
         }
      }

      if ( (BGVAR(u16_Pwm_Freq_State) == HALF_PWM_FREQ) && (BGVAR(u16_Pwm_Freq_Prev_State) == REGULAR_PWM_FREQ) )
      {
         VAR(AX0_u16_Pwm_Change_Indication) = CHANGE_TO_HALF_PWM_FREQ_INDICATION;
      }
      else if ( (BGVAR(u16_Pwm_Freq_State) == REGULAR_PWM_FREQ) && (BGVAR(u16_Pwm_Freq_Prev_State) == HALF_PWM_FREQ) )
      {
         VAR(AX0_u16_Pwm_Change_Indication) = CHANGE_TO_REGULAR_PWM_FREQ_INDICATION;
      }

      BGVAR(u16_Pwm_Freq_Prev_State) = BGVAR(u16_Pwm_Freq_State);
   }
}


//**********************************************************
// Function Name: WatchdogHandler
// Description:
//   Code that is executed when watchdog occurred.
//   The input parameter indicates who called this function.
//   0 - called from WD_ISR. In this case display steady watchdog indication (3 bars).
//   1 - called from ILLEGAL_ISR. In this case display flashing watchdog indication (3 bars).
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
void WatchdogHandler (int s16_source)
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // put a mark in the stack
   asm("   MOV    @AL,#1234h");
   asm("   MOV    @AH,#5001h");
   asm("   PUSH   ACC");
   asm("   MOV    AL,@SP");
   asm("   MOVL   XAR0,#07feh");
   asm("   MOV    *XAR0,AL");

//
//  Lets say the SP value, which was written to location 0x07fe is N.
//  The relevant stack area is:
//  N    -   xxxx
//  N-1  - 0x5001 - AH from WatchdogHandler
//  N-2  - 0x1234 - AL from WatchdogHandler
//  N-3  -   xxxx
//  N-4  -   xxxx
//  N-5  -   xxxx
//  N-6  -   xxxx
//  N-7  -   xxxx
//  N-8  -   xxxx
//  N-9  -   AAAA - Hi part of the RPC (return value of program counter) of the function before illegal instruction (or equivalent)
//  N-10 -   AAAA - Lo part of ...
//  N-11 - 0x5000 - AH from ILLEGAL_ISR (if occurred)
//  N-12 - 0x1234 - AL from ILLEGAL_ISR (if occurred)
//

   // Write to FPGA to route all "1" to PWM out
   *((int*)FPGA_PWM_ENABLE_REG_ADD) = 0;
   EALLOW;
   /* generate TZ event - PWM outputs will become in drive disable mode */
   EPwm1Regs.TZFRC.bit.OST = 1;
   EPwm2Regs.TZFRC.bit.OST = 1;
   EPwm3Regs.TZFRC.bit.OST = 1;
//   SysCtrlRegs.WDCR = 0x0E8; //clr wd bit
   EDIS;
   DINT;
   s16_Wd_State = 1;
   drive = 0;
   BGVAR(s64_SysNotOk) |= WATCH_DOG_FLT_MASK;
   drive = 1;
   BGVAR(s64_SysNotOk) |= WATCH_DOG_FLT_MASK;
   drive = 0;

   if (s16_source == 0)
      HWDisplay(WATCHDOG_DISPLAY, 4);
   else if (s16_source == 1)
      HWDisplay(ILLEGAL_OPERATION_DISPLAY, 4);
   else
      HWDisplay(UNEXPECTED_ISR_DISPLAY, 4);

   if (u16_Product == SHNDR_HW)
   {
      HWDisplay(NONE,  3);
      HWDisplay(NONE,  2);
      HWDisplay(NONE,  1);
      HWDisplay(NONE,  0);
   }

   s16_Wd_Source = s16_source;

/***************************************************************************************
*
*  AZ debug
*
*  In case there is no communication to retreave the content of the stack, uncomment to
*  code below, and use HyperTerminal to see the information sent here.
*
*  Move the declaration of the below variable to outside this function (above).
*  Do not declate the variables as loacal since it changes the order of the  stack content.
*
*  unsigned int u16_temp, u16_addr;  // AZ debug
*
*   SCIB_SendByte(CR);
*   SCIB_SendByte(LF);
*   SCIB_SendByte('1');
*   SCIB_SendByte('>');
*   u16_temp = (s16_Wd_Source >> 4) & 0x000f;
*   if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*   SCIB_SendByte(u16_temp);
*   u16_temp = s16_Wd_Source & 0x000f;
*   if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*   SCIB_SendByte(u16_temp);
*   SCIB_SendByte(CR);
*   SCIB_SendByte(LF);
*   SCIB_SendByte('2');
*   SCIB_SendByte('>');
*   u16_addr = *(unsigned int *)0x7fe;
*   if (u16_addr == 0xbeef)
*   {
*      SCIB_SendByte('x');
*   }
*   else
*   {
*      SCIB_SendByte('0');
*      SCIB_SendByte('x');
*      u16_addr -= 9;
*      u16_temp = (*(unsigned int *)u16_addr >> 12)  & 0x000f;
*      if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*      SCIB_SendByte(u16_temp);
*      u16_temp = (*(unsigned int *)u16_addr >> 8)  & 0x000f;
*      if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*      SCIB_SendByte(u16_temp);
*      u16_temp = (*(unsigned int *)u16_addr >> 4)  & 0x000f;
*      if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*      SCIB_SendByte(u16_temp);
*      u16_temp = *(unsigned int *)u16_addr & 0x000f;
*      if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*      SCIB_SendByte(u16_temp);
*      SCIB_SendByte(',');
*      u16_addr -= 1;
*      u16_temp = (*(unsigned int *)u16_addr >> 12)  & 0x000f;
*      if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*      SCIB_SendByte(u16_temp);
*      u16_temp = (*(unsigned int *)u16_addr >> 8)  & 0x000f;
*      if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*      SCIB_SendByte(u16_temp);
*      u16_temp = (*(unsigned int *)u16_addr >> 4)  & 0x000f;
*      if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*      SCIB_SendByte(u16_temp);
*      u16_temp = *(unsigned int *)u16_addr & 0x000f;
*      if (u16_temp < 10) u16_temp += '0'; else u16_temp = u16_temp - 10 + 'a';
*      SCIB_SendByte(u16_temp);
*      SCIB_SendByte(CR);
*      SCIB_SendByte(LF);
*
*      // more debug code of the same manner can be added below
*   }
*/

   // In case of Watchdog set all digital OUTPUTS to low.
   // disable any output inv.
   *(unsigned int*)FPGA_FPGA_C_OUT_POLARITY_REG_ADD = 0;
   *(unsigned int*)FPGA_FPGA_M_FAST_OUT_POLARITY_REG_ADD = 0;

   // set all outputs to low.
   *(unsigned int*)FPGA_CONTROL_OUTPUT_REG_ADD = 0;
   *(unsigned int*)FPGA_MACHINE_OUTPUT_REG_ADD = 0;
   *(int*)FPGA_PWR_BRK_CMD_REG_ADD |= 0x0001;// lock the brake
   *(int*)FPGA_DIGITAL_OUTPUT_REG_ADD = 0;

   // Reset the Serial comms processor state machine, and exit Modbus (in case it was set)
   InitSerCommunication();
   s16_Comms_Processor_State = PRE_PROCESSOR;
   BGVAR(s16_Serial_Type) = SERIAL_COMM_TYPE;

   for(;;)
   {
      SciInputProcessor();
      SciOutputProcessor();
      CommsProcessor(0);
      RS485_TRANSMIT();
   }
}


interrupt void WD_ISR(void)
{
   WatchdogHandler(0);
}


//**********************************************************
// Function Name: CheckGearInPos
// Description: Updates STOPPED state in GEAR mode.
//          motion profile considered accomplished if no PCMD not changes for 2ms.
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(CheckGearInPos, "ramfunc_3");
void CheckGearInPos(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (VAR(AX0_s16_Opmode) != 4) return;

   BGVAR(u32_Pcmd_Delta) = BGVAR(u32_Pcmd_Prev) - LVAR(AX0_u32_Pos_Cmd_User_Lo);
   BGVAR(u32_Pcmd_Prev) = LVAR(AX0_u32_Pos_Cmd_User_Lo);
   if (BGVAR(u32_Pcmd_Delta) != 0L) BGVAR(u32_Pcmd_Delta_Zero_Time) = Cntr_1mS;

   if ((BGVAR(u32_Pcmd_Delta) == 0) && PassedTimeMS(2L, BGVAR(u32_Pcmd_Delta_Zero_Time)))
   {
      if (VAR(AX0_s16_Stopped) < 1) VAR(AX0_s16_Stopped) = 1;
      if (VAR(AX0_u16_Inpos) == 1) VAR(AX0_s16_Stopped) = 2;
   }
   else VAR(AX0_s16_Stopped) = 0;
}


//**********************************************************
// Function Name: CheckCanOpmode
// Description: if opmode is changed via RPDO, initiate opmode change process here.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void CheckCanOpmode(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_CAN_Flags) & CANOPEN_OPMODE_CHANGED_MASK)
   {
      // Start OPMODE change acia CAN and EtherCAT. In this case the function
      // "SetCanOpmode" needs to be called from the BG handler so the lower byte
      // that holds the CDHD OPMODE does not matter.
      BGVAR(s16_Start_Opmode_Change) = OPMODE_CHANGE_SOURCE_CAN_ECAT;
   }
}


//**********************************************************
// Function Name: SetCanOpmode
// Description: if opmode is changed via RPDO, update it here.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void SetCanOpmode(int drive)
{
   // AXIS_OFF;

   int s16_tmp, s16_disallow_change;
   unsigned int u16_p402_statusword_shadow;

   EXIT_IF_NOT_FIELDBUS   //exit if not fieldbus

   if (BGVAR(s16_CAN_Flags) & CANOPEN_OPMODE_CHANGED_MASK)
   {

      if (BGVAR(s8_BurninParam) != 0)
      {
      BGVAR(s16_CAN_Flags) &= ~CANOPEN_OPMODE_CHANGED_MASK;
         return; // if in burnin mode return
      }

      // Do not allow an OPMODE switch request to the already adjusted mode of operation
      s16_disallow_change = (p402_modes_of_operation_display == BGVAR(s16_CAN_Opmode_Temp));


      if (s16_disallow_change)
      {
         // revert back to current opmode.
         p402_modes_of_operation = p402_modes_of_operation_display;
      }
      else // we allow change
      {
         if (ConvertCanOpenOpmode(drive, BGVAR(s16_CAN_Opmode_Temp), &s16_tmp) == FAL_SUCCESS)
         {
            VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_VIA_FB_PENDING_MASK; // Indicate that an OPMODE change via fieldbus is pending
            BGVAR(s16_CAN_Opmode) = BGVAR(s16_CAN_Opmode_Temp);

            //if we move from Homing to Sync position we need to update sync position aux variables
            if (((p402_modes_of_operation_display == HOMING_MODE) && ((BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_POSITION_MODE) || (BGVAR(s16_CAN_Opmode) == INTRPOLATED_POSITION_MODE))) ||
                ((p402_modes_of_operation_display == PROFILE_POSITION_MODE) && ((BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_POSITION_MODE) || (BGVAR(s16_CAN_Opmode) == INTRPOLATED_POSITION_MODE))))
            {
               HomingToSyncPositionTransaction(drive);
            }

            // if request is to change opmode to homing and currently in a different mode
            if ((p402_modes_of_operation == HOMING_MODE) && (p402_modes_of_operation_display != HOMING_MODE))
            {
               if (BGVAR(u8_Homing_State) != HOMING_TARGET_REACHED)
               {
                  // nitsan: per Joachim Eichner (SE) request, cancel homing bits in canopen status word when entering home mode
                  // in order to prevent immediate homing failure from master side in case that previous homing failed and HomeState == HOMING_FAILED.
                  // update CANopen statusword in case it is mapped into TPDO and did not updated yet in background
                  HomeCommand(0);     // cancel homing
                  u16_p402_statusword_shadow = p402_statusword;
                  u16_p402_statusword_shadow &= ~(HM_ERROR | HM_ATTAINED);
                  u16_p402_statusword_shadow |= HM_TARGET_REACHED;
                  p402_statusword = u16_p402_statusword_shadow;
               }
            }

            BGVAR(u8_Homing_Ignore_LS) = 0x03;  // Ignore positive and negative LS for Lexium in order to make the initialization of the LS variables thread-safe.
                                                // This instruction has only an impact on the Lexium Drive.
            p402_modes_of_operation = BGVAR(s16_CAN_Opmode); // Set mode of operation now and mode of operation display later

            if ((p402_modes_of_operation != CYCLIC_SYNCHRONOUS_POSITION_MODE) &&         // Bug-fix for IPR668. Only if we are NOT switching to CSP because the
                (p402_modes_of_operation != INTRPOLATED_POSITION_MODE))           // switch to CSP handles the initiatization via "BGVAR(u8_Align_Prev_Temp_Pos_LS)".
            {                                                            
               // Initialize LS variables to indicate "no motion"
               BGVAR(s16_Move_Abs_Issued) = 0;
               BGVAR(s64_Temp_Pos_LS) = 0LL;
            }
            BGVAR(u8_Homing_Ignore_LS) = 0x00; // Consider positive and negative LS for Lexium.

            SetOpmode(drive, s16_tmp);

            if (BGVAR(s16_CAN_Opmode) >= 0) //for cyclic synchronous modes
               FalSetFBSyncOpmode(drive, BGVAR(s16_CAN_Opmode));

            // Set modes of operation display AFTER finishing the OPMODE change since the mode of operation display
            // is a confirmation to the fieldbus that the OPMODE switch is really finished.
            p402_modes_of_operation_display = p402_modes_of_operation;

            // Retrigger updating the command value of the fieldbus master when
            // switching to profile torque/velocity since the command value of the
            // loop is overwritten during the OPMODE change process.
            if (p402_modes_of_operation_display == PROFILE_TORQUE_MODE)
            {
               BGVAR(u16_Fb_Opmode_Change_Actions) |= UPDATE_PT_COMMAND_VALUE;
            }
            else if (p402_modes_of_operation_display == PROFILE_VELOCITY_MODE)
            {
               BGVAR(u16_Fb_Opmode_Change_Actions) |= UPDATE_PV_COMMAND_VALUE;
            }

            VAR(AX0_s16_Opmode_Change_Flags) &= ~OPMODE_CHANGE_VIA_FB_PENDING_MASK; // Clear flag that indicates that an OPMODE change via fieldbus is pending
         }
      }
      // Revert the OPMODE changed mask at the end of the function in order to avoid a race condition where a new OPMODE
      // change request appears via the 0x6060 PDO although a pending OPMODE change is not yet successfully finished.
      BGVAR(s16_CAN_Flags) &= ~CANOPEN_OPMODE_CHANGED_MASK;
   }
}


//**********************************************************
// Function Name: CheckCanAcc
// Description: if acc is changed via RPDO, update it here.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CheckCanAcc(int drive)
{
   int ret_val;

   EXIT_IF_NOT_FIELDBUS   //exit if not fieldbus

   if (BGVAR(s16_CAN_Flags) & CANOPEN_ACC_CHANGED_MASK)
   {
      BGVAR(s16_CAN_Flags) &= ~CANOPEN_ACC_CHANGED_MASK;
      ret_val = Can_SetAcc(BGVAR(u32_CAN_Rpdo_Acc), drive);
      if (ret_val != SAL_SUCCESS)
      {
         // if fail, restore original value
         p402_profile_acceleration = (long)MultS64ByFixS64ToS64((BGVAR(u64_CAN_Acc)),
                   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);
      }
   }
}


//**********************************************************
// Function Name: CheckCanAcc
// Description: if acc is changed via RPDO, update it here.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CheckCanDec(int drive)
{
   int ret_val;
   // AXIS_OFF;

   EXIT_IF_NOT_FIELDBUS   //exit if not fieldbus

   if (BGVAR(s16_CAN_Flags) & CANOPEN_DEC_CHANGED_MASK)
   {
      //BZ#5510: Wrong Quick Stop Deceleration (0x6085) in profile velocity mode when Profile Deceleration object (0x6084) mapped in PDO.
      // - wait till quick stop finish to update the deceleration via PDO
      //IPR#1675: When drive decelerates due to an Quickstop command has been performed, the motor stopped abruptly after a couple of seconds
      // - if any disable inputs are on, wait till the motor stops to update the deceleration via PDO
      if ( (((p402_statusword & STATUSWORD_STATE) == QUICK_STOP_ACTIVE) || BGVAR(s16_DisableInputs))
         && (VAR(AX0_u16_Motor_Stopped) != 2))
      {
         return;
      }

      BGVAR(s16_CAN_Flags) &= ~CANOPEN_DEC_CHANGED_MASK;
      ret_val = Can_SetDec(BGVAR(u32_CAN_Rpdo_Dec), drive);
      if (ret_val != SAL_SUCCESS)
      {
         // if fail, restore original value
         p402_profile_deceleration = (long)MultS64ByFixS64ToS64((BGVAR(u64_DecRate)),
                   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);
      }
   }
}


//**********************************************************
// Function Name: CheckCanGearIn
// Description: if gear in is changed via RPDO, update it here.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CheckCanGearIn(int drive)
{
   int ret_val;
   long s32_tmp;

   EXIT_IF_NOT_FIELDBUS   //exit if not fieldbus

   if (BGVAR(s16_CAN_Flags) & CANOPEN_GEAR_IN_CHANGED_MASK)
   {
      BGVAR(s16_CAN_Flags) &= ~CANOPEN_GEAR_IN_CHANGED_MASK;
      s32_tmp = BGVAR(s32_CAN_Rpdo_GearIn);
      ret_val = SalGearInCommand(s32_tmp, drive);
      if (ret_val == SAL_SUCCESS)
      {
         BGVAR(s32_Fb_Gear_In) = s32_tmp;
      }
      else
      {  // if fail, restore original value
         manu_spec_s32_Lxm_Gear_Arr[1] = BGVAR(s32_Fb_Gear_In);
      }
   }
}


//**********************************************************
// Function Name: CheckCanGearOut
// Description: if gear out is changed via RPDO, update it here.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CheckCanGearOut(int drive)
{
   int ret_val;
   long s32_tmp;

   //exit if not fieldbus
   EXIT_IF_NOT_FIELDBUS

   if (BGVAR(s16_CAN_Flags) & CANOPEN_GEAR_OUT_CHANGED_MASK)
   {
      BGVAR(s16_CAN_Flags) &= ~CANOPEN_GEAR_OUT_CHANGED_MASK;
      s32_tmp = BGVAR(s32_CAN_Rpdo_GearOut);
      ret_val = SalGearOutCommand(s32_tmp, drive);
      if (ret_val == SAL_SUCCESS)
      {
         BGVAR(s32_Fb_Gear_Out) = s32_tmp;
      }
      else
      {
         // if fail, restore original value
         manu_spec_s32_Lxm_Gear_Arr[2] = BGVAR(s32_Fb_Gear_Out);
      }
   }
}


//**********************************************************
// Function Name: HomingToSyncPositionTransaction
// Description: update sync position aux variables.
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void HomingToSyncPositionTransaction(int drive)
{
   unsigned int u16_temp_time;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   //set sync position interpolation vars to PCMD in order to not get a jump upon moveing to sync position mode
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_FieldBus_Int_Lo) = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
      LLVAR(AX0_u32_FieldBus_Int_Target_Lo) = LLVAR(AX0_u32_FieldBus_Int_Lo);
      //LLVAR(AX0_u32_FieldBus_Int_Linear_Prev_Target_Lo_2) = LLVAR(AX0_u32_FieldBus_Int_Lo);
      LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1) = LLVAR(AX0_u32_FieldBus_Int_Lo);
      LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2) = LLVAR(AX0_u32_FieldBus_Int_Lo);
      LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_3) = LLVAR(AX0_u32_FieldBus_Int_Lo);

     //cubic interpolation vars
     LLVAR(AX0_u32_FieldBus_Int_Current_Target_Lo) = LLVAR(AX0_u32_FieldBus_Int_Lo);
     LLVAR(AX0_u32_FieldBus_Int_d_Lo) = LLVAR(AX0_u32_FieldBus_Int_Lo);
     LLVAR(AX0_u32_FieldBus_Int_d_tmp_Lo) = LLVAR(AX0_u32_FieldBus_Int_Lo);
   } while (u16_temp_time != Cntr_3125);

}


//**********************************************************
// Function Name: CheckCanDigitalOutputs
// Description: if digital outputs (0x60FE) is changed via RPDO, update it here.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void CheckCanDigitalOutputs(int drive)
{
   // AXIS_OFF;
   int i;
   REFERENCE_TO_DRIVE;
   // If the requested digital output state has been written by the master (in RT via PDO)
   if (VAR(AX0_s16_CAN_Dig_Out_Changed_Flag))
   {
      VAR(AX0_s16_CAN_Dig_Out_Changed_Flag) = 0;      // Clear the flag

      // Write the RT value to the 0x60FE Sub 1 array entry
      p402_digital_outputs[1] = LVAR(AX0_s32_CAN_Dig_Out);

      for (i = 1; i <= s16_Num_Of_Outputs; i++)
      {
         // If it is allowed to set the digital output since it is masked in 0x60FE Sub 2
         if ((p402_digital_outputs[2] >> (unsigned long)(i - 1 + 16)) & 0x1)
         {
            // Set Output value according to upper 16-Bits in "AX0_s32_CAN_Dig_Out". Note that the digital
            // Output mode needs to be "Idle" in order to let the fieldbus control the state.
            OutCommand((long long)i,(long long)((p402_digital_outputs[1] >> (unsigned long)(i - 1 + 16)) & 0x1));
         }
      }
   }
}

//**********************************************************
// Function Name: CheckCanPcom
// Description: if Pcom control (0x2191 & 2192) is changed via RPDO, update it here.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void CheckCanPcom(int drive)
{
   drive+=0;
   if(u16_Execute_Pcom1_From_Pdo)
   {
      u16_Execute_Pcom1_From_Pdo = 0;
	  SalWritePcomCntrl1Command((long long)BGVAR(u16_PCom_Cntrl1),0);

   }

   if(u16_Execute_Pcom2_From_Pdo)
   {
      u16_Execute_Pcom2_From_Pdo = 0;
	  SalWritePcomCntrl2Command((long long)BGVAR(u16_PCom_Cntrl2),0);

   }
}


//**********************************************************
// Function Name: HandleEepromOperations
// Description: handle eeprom operations (save/load) initiated by can open SDOs
//              need to prevent executing of one opertion if the other one is in progress.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void HandleEepromOperations()
{
   int ret_val;

   switch (s16_CAN_EEprom_Operation_Type)
   {
      case 0:       // do nothing
      break;

      case 1:       // perform save
         ret_val = SalSaveToFlash(0);
         if (ret_val == SAL_SUCCESS)
         {
            s16_CAN_EEprom_Operation_Type = 0;
            manu_spec_nonvolatile_command_stat = SAL_SUCCESS;
         }
         else if (ret_val != SAL_NOT_FINISHED)
         {  // error in saving
            s16_CAN_EEprom_Operation_Type = 0;
            manu_spec_nonvolatile_command_stat = ret_val;
         }
      break;

      case 2:      // perform load
         ret_val = SalLoadFromFlashCommand(0);
         if (ret_val == SAL_SUCCESS)
         {
            s16_CAN_EEprom_Operation_Type = 0;
            manu_spec_nonvolatile_command_stat = SAL_SUCCESS;
         }
         else if (ret_val != SAL_NOT_FINISHED)
         {  // error in loading
            s16_CAN_EEprom_Operation_Type = 0;
            manu_spec_nonvolatile_command_stat = ret_val;
         }
      break;
   }
}


//**********************************************************
// Function Name: MoterTorqueOvershoot
// Description: Check if drive's current torque is greater
// than the torque limit treshold CRSHA(P1-57)
// for the CRSHT (P1-58) amount of time in ms and if so set a fault!
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(MoterTorqueOvershoot, "ramfunc_3");
void MoterTorqueOvershoot(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

// check if s16_P1_57_CRSHA equal to 0 if so the feature is disabled so return
   if (BGVAR(s16_P1_57_CRSHA) == 0) return;
   if (BGVAR(s16_MOTOR_OVRSHOOT_STATE) == 0) // state machine idle state check if there is an overshoot
   {
       if (BGVAR(s32_P1_57_CRSHA_Internal) < BGVAR(u32_EqCrrnt))
       {//if overshoot detected capture the current time and step the state machine to next step
            BGVAR(s32_P1_58_TIMER) = Cntr_1mS;
            BGVAR(s16_MOTOR_OVRSHOOT_STATE)++;
       }
   }
   else if (BGVAR(s16_MOTOR_OVRSHOOT_STATE) == 1)// overshoot has detected in idle state
   {
       if (BGVAR(s32_P1_57_CRSHA_Internal) >= BGVAR(u32_EqCrrnt))
       {// if the overshoot ended set state machine back to idle
            BGVAR(s16_MOTOR_OVRSHOOT_STATE) = 0;
       }// else still in overshoot check if time over and if so set an alarm
       else if (PassedTimeMS(BGVAR(s16_P1_58_CRSHT), BGVAR(s32_P1_58_TIMER)))
       {// set motor overshoot alarm. fault will be raised in background (flt handler will capture:  BGVAR(s16_MOTOR_OVRSHOOT_STATE) = 2).
            BGVAR(s16_MOTOR_OVRSHOOT_STATE) = 2;   // signal fault
       }
   }
}


//**********************************************************
// Function Name: TransferDataBetweenDDHDAxes
// Description: ransfer BG data between two axes of DDHD.
//    This function is called only if the product is DDHD.
//
// Author: AZ
// Algorithm:
// Revisions:
//*********************************************************
void TransferDataBetweenDDHDAxes(void)
{
   unsigned int u16_checksum, u16_temp_display_500ms_counter_lo, u16_temp_display_500ms_counter_hi;
   static unsigned int u16_DDHD_Fpga_Wrote_Eeprom, u16_BGcounter = 0;
   int s16_temp_power_temperature_deg, s16_temp_power_temperature_deg_x256, s16_temp_cntrl_temperature_deg;
   int s16_temp_cntrl_temperature_deg_x256, drive = 0;
   // AXIS_OFF;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (u16_Axis_Num == 0)
   {
      //   Check first if Axis2 signals it needs EEPROM copy between FPGAs (after reset / FW upgrade)
      if ( (*(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD == 0xAC13) && (!u16_DDHD_Fpga_Wrote_Eeprom) )
      // check if the FPGA buffer on second axis is ready
      {
         VAR(AX0_u16_Is_Eeprom_Copied_Between_Fpgas) = 1; // this flag will disable A2D transfer between FPGAs on RT
         CopyEEpromDataBetweenDDHDFpgas();
         u16_DDHD_Fpga_Wrote_Eeprom = 1;
      }
      else if ( (u16_DDHD_Fpga_Wrote_Eeprom) && (*(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD != 0xAC14) )
      {
         u16_BGcounter++;
         if (2 < u16_BGcounter)
         {
            *(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD = 0xAC14;
            u16_BGcounter = 0;
         }
      }
      // check if other DSP read prev data (it sends back the counter value), or if other DSP finished read EEPROM copy (sending back 0xAC14)
      else if ( (*(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD == u16_Axis_To_Axis_Sync_Counter) ||
                (*(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD == 0xAC14)                          )
      {
         if (*(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD == 0xAC14)
         {
            VAR(AX0_u16_Is_Eeprom_Copied_Between_Fpgas) = 0;  //Enable A2D transfer between FPGAs on RT
            *(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD = u16_Axis_To_Axis_Sync_Counter;
            u16_DDHD_Fpga_Wrote_Eeprom = 0;
         }
         u16_Axis_To_Axis_Sync_Counter++;
         if (0xAC13 == u16_Axis_To_Axis_Sync_Counter) //skip values 0xAC13 and 0xAC14 which are related to EEPROM copy
            u16_Axis_To_Axis_Sync_Counter += 2;

         u16_checksum = 0;
         *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 8) = u16_Axis_To_Axis_Sync_Counter;   // Offset 8 is the beginning of BG buffer
         u16_checksum += (unsigned int)u16_Axis_To_Axis_Sync_Counter;
         *(int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 9) = BGVAR(s16_Power_Temperature_Deg);
         u16_checksum += (unsigned int)BGVAR(s16_Power_Temperature_Deg);
         *(int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 10) = s16_Power_Temperature_Deg_x256;
         u16_checksum += (unsigned int)s16_Power_Temperature_Deg_x256;
         *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 11) = (unsigned int)u32_Display_500ms_Counter;
         u16_checksum += (unsigned int)u32_Display_500ms_Counter;
         *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 12) = (unsigned int)(u32_Display_500ms_Counter >> 16);
         u16_checksum += (unsigned int)(u32_Display_500ms_Counter >> 16);
         *(int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 13) = BGVAR(s16_Control_Temperature_Deg);
         u16_checksum += (unsigned int)BGVAR(s16_Control_Temperature_Deg);
         *(int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 14) = s16_Control_Temperature_Deg_x256;
         u16_checksum += (unsigned int)s16_Control_Temperature_Deg_x256;
         *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 15) = u16_checksum;

         // Write number of words to transmit to indicate the FPGA that the data is ready to be transferred
         // to the other FPGA (it is the FPGA responsibility to make sure the other FPGA is ready).
         AX0_u16_Axis_To_Axis_Bg_Word_Num = 8;   // Write number of words to transmit
      }
      else
      {
         u16_FPGA_BG_Buffer_Not_Ready++;
      }
   }
   else
   {
      if (*(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 8) != u16_Axis_To_Axis_Sync_Counter)    // check if the FPGA buffer is ready
      {
         u16_checksum = 0;
         u16_Axis_To_Axis_Sync_Counter = *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 8);
         u16_checksum += u16_Axis_To_Axis_Sync_Counter;
         s16_temp_power_temperature_deg = *(int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 9);
         u16_checksum += (unsigned int)s16_temp_power_temperature_deg;
         s16_temp_power_temperature_deg_x256 = *(int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 10);
         u16_checksum += (unsigned int)s16_temp_power_temperature_deg_x256;
         u16_temp_display_500ms_counter_lo = *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 11);
         u16_checksum += u16_temp_display_500ms_counter_lo;
         u16_temp_display_500ms_counter_hi = *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 12);
         u16_checksum += u16_temp_display_500ms_counter_hi;
         s16_temp_cntrl_temperature_deg = *(int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 13);
         u16_checksum += (unsigned int)s16_temp_cntrl_temperature_deg;
         s16_temp_cntrl_temperature_deg_x256 = *(int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 14);
         u16_checksum += (unsigned int)s16_temp_cntrl_temperature_deg_x256;
         u16_checksum -= *(unsigned int *)(FPGA_AXIS1_TO_AXIS2_BUFFER_ADD + 15);

         if (u16_checksum == 0)
         {
            BGVAR(s16_Power_Temperature_Deg) = s16_temp_power_temperature_deg;
            s16_Power_Temperature_Deg_x256 = s16_temp_power_temperature_deg_x256;
            BGVAR(s16_Control_Temperature_Deg) = s16_temp_cntrl_temperature_deg;
            s16_Control_Temperature_Deg_x256 = s16_temp_cntrl_temperature_deg_x256;
            u32_Display_500ms_Counter = (((unsigned long)u16_temp_display_500ms_counter_hi) << 16)    |
                                        (((unsigned long)u16_temp_display_500ms_counter_lo) & 0xffffL);
            u16_FPGA_BG_Buffer_Checksum = 0;
            u16_FPGA_BG_Buffer_Transfer_Success++;
         }
         else
         {
            if (u16_FPGA_BG_Buffer_Checksum < 0x7fff)
               u16_FPGA_BG_Buffer_Checksum++;
         }

         // Indicate to the FPGA that the data was read
         *(unsigned int *)FPGA_AXIS2_TO_AXIS1_BUFFER_ADD = u16_Axis_To_Axis_Sync_Counter;

         u16_FPGA_BG_Buffer_Consecutive_Not_Ready = 0;
      }
      else
      {
         if (u16_FPGA_BG_Buffer_Consecutive_Not_Ready < 0x7fff)
            u16_FPGA_BG_Buffer_Consecutive_Not_Ready++;
         u16_FPGA_BG_Buffer_Total_Not_Ready++;
      }
   }
}


// If PE>SFBPEMAX or PE>SFBPETHRESH for SFBPETIME msecs continously
//#pragma CODE_SECTION(SFBPositionErrorHandler, "ramfunc_4");
void SFBPositionErrorHandler(int drive)
{
   // AXIS_OFF;

   static unsigned int u16_prev_enabled_state = 0;
   unsigned int u16_enabled_state;
   int s16_temp;
   long long s64_abs_sfb_pfb_error;

   if (BGVAR(s16_SFBMode) != 0)
   {
      // Here handle the realignment counter feature. This feature gives the RT task time to at least 1-time
      // calculate SFB with new offset variables before realigning SFB and PFB and generating an error. This
      // is to avoid a race condition between different RT/BG tasks.
      if (BGVAR(u8_Sfb_Pfb_Realignment_Counter) > 0)
      {
         BGVAR(u8_Sfb_Pfb_Realignment_Counter)--;
         if (BGVAR(u8_Sfb_Pfb_Realignment_Counter) == 0)
         {
            BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) |= SFB_PFB_COMPARE_TRIGGER_ALIGNMENT;  // Trigger realignment
         }
         else
         {
            return; // No error compare before realignment is handled
         }
      }

      // Capture the diff between the load and the motor pfb on the very first dis->en,
      u16_enabled_state = Enabled(drive);
      if (u16_enabled_state && (!u16_prev_enabled_state))
      {
         u16_prev_enabled_state = u16_enabled_state;     // One time update the state
         BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) |= SFB_PFB_COMPARE_TRIGGER_ALIGNMENT;  // Trigger realignment
      }

      // If we need to look at digital input mode for realignment and enable of the comparison feature
      if (BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) & SFB_PFB_COMPARE_DIG_INPUT_CONFIGURED)
      {
         if (VAR(AX0_u16_Mode_Input_Arr[EN_SFB_PFB_COMPARE]))
         {
            // Upon rising edge
            if ((BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) & SFB_PFB_COMPARE_PREV_DIG_INPUT_STATE) == 0)
            {
               BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) |= (SFB_PFB_COMPARE_TRIGGER_ALIGNMENT | SFB_PFB_COMPARE_PREV_DIG_INPUT_STATE); // Trigger realignment and set previous input state to true
            }
         }
         else
         {
            // Clear previous input state
            BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) &= ~SFB_PFB_COMPARE_PREV_DIG_INPUT_STATE; // Clear previous state bit
         }
      }

      // If a realignment is requested
      if (BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) & SFB_PFB_COMPARE_TRIGGER_ALIGNMENT)
      {
         do
         {
            s16_temp = Cntr_3125;
            BGVAR(s64_Sfb_Pe_Capture) = LLVAR(AX0_u32_Pext_Fdbk_User_Lo) - LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
         }
         while (s16_temp != Cntr_3125);
         // Clear realignment request bit 0
         BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) &= ~SFB_PFB_COMPARE_TRIGGER_ALIGNMENT;
      }

      // Monitor Fault only during enable and either no input assigned to mode "EN_SFB_PFB_COMPARE" OR
      // an input assigned to mode "EN_SFB_PFB_COMPARE" is active.
      if ( ((u16_enabled_state) && ((BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) & SFB_PFB_COMPARE_DIG_INPUT_CONFIGURED) == 0x00))                             ||
           ((BGVAR(u8_Sfb_Pfb_Compare_Gen_Purpose) & SFB_PFB_COMPARE_INPUT_CONFIGURED_ENABLE_COMPARE) == SFB_PFB_COMPARE_INPUT_CONFIGURED_ENABLE_COMPARE)  )
      {
         // Calculate PE between load and motor (compensating the initial offset between them)
         do {
            s16_temp = Cntr_3125;
            BGVAR(s64_Sfb_Pfb_Pe) = LLVAR(AX0_u32_Pext_Fdbk_User_Lo) - LLVAR(AX0_u32_Pos_Fdbk_User_Lo) - BGVAR(s64_Sfb_Pe_Capture);
         } while (s16_temp != Cntr_3125);

         // Build the absolute value of the error between SFB and PFB
         if (BGVAR(s64_Sfb_Pfb_Pe) < 0LL)
         {
            s64_abs_sfb_pfb_error = -BGVAR(s64_Sfb_Pfb_Pe);
         }
         else
         {
            s64_abs_sfb_pfb_error = BGVAR(s64_Sfb_Pfb_Pe);
         }

         if ((s64_abs_sfb_pfb_error >= BGVAR(u64_Sfb_Pe_Max)) && (BGVAR(u64_Sfb_Pe_Max) > 0LL))
            BGVAR(u16_Sfb_Pe_fault) = 1;
         else
         {
            if ((s64_abs_sfb_pfb_error >= BGVAR(u64_Sfb_Pe_Thresh)) && (BGVAR(u64_Sfb_Pe_Thresh) > 0LL))
            {
               BGVAR(u16_Sfb_Pe_fault_Timer)++; // Runs in 1msec interrupt so ++ is enough
               if (BGVAR(u16_Sfb_Pe_fault_Timer) >= BGVAR(u16_Sfb_Pe_Time))
               {
                  BGVAR(u16_Sfb_Pe_fault) = 1;
                  BGVAR(u16_Sfb_Pe_fault_Timer) = BGVAR(u16_Sfb_Pe_Time);
               }
            }
            else BGVAR(u16_Sfb_Pe_fault_Timer) = 0;
         }
      }
      else
      {
         BGVAR(s64_Sfb_Pfb_Pe) = 0LL; // Clear the error in case that the monitoring is not active
         BGVAR(u16_Sfb_Pe_fault_Timer) = 0;
      }
   }
   else // Not in SFBMODE
   {
      BGVAR(u16_Sfb_Pe_fault) = 0;
   }
}


void EncSimTestModeHandler(int drive)
{
   // AXIS_OFF;
   static unsigned int cycle_num=0;
   REFERENCE_TO_DRIVE;
   switch (BGVAR(u8_Enc_Sim_Test_Request))
   {
      case ENC_SIM_TEST_IDLE:
         // do nothing
      break;

      case ENC_SIM_TEST_ENABLE:
         BGVAR(u8_Enc_Sim_Test_Request) = ENC_SIM_TEST_BUSY;
         // handle test mode according to user parameters
         if (cycle_num < BGVAR(u16_Enc_Sim_Test_Number_Of_Cycles))
         {
            VAR(AX0_s16_Encsim_Counts_In_MTS) = BGVAR(u32_Enc_Sim_Test_Frequency) / 32000; // calculate counts per sec, divide by 32000 MTS in sec
            VAR(AX0_s16_Encsim_Num_Of_Loops) = BGVAR(u16_Enc_Sim_Test_Counts) / BGVAR(AX0_s16_Encsim_Counts_In_MTS);
            VAR(AX0_s16_Encsim_Counts_Remainder) = BGVAR(u16_Enc_Sim_Test_Counts) % VAR(AX0_s16_Encsim_Counts_In_MTS); // calculate the remainder
            // Calculate the spacing by 1800/NUM_OF_COUNTS+1 ; FGPA clock is 60Mhz, yields 1850 clocks per MTS
            VAR(AX0_s16_Encsim_Space) =  1800/(AX0_s16_Encsim_Counts_In_MTS+1);
            VAR(AX0_s16_Encsim_State_Busy) = 1; // Start sending
            VAR(AX0_s16_Encsim_Test_Mode) = 1; // Enable EncSim test mode
            cycle_num++;
         }
         else
         {
            // after sending all cycles, back to idle
            BGVAR(u8_Enc_Sim_Test_Request) = ENC_SIM_TEST_IDLE;
            cycle_num = 0;
         }
      break;

      case ENC_SIM_TEST_WAIT_BETWEEN_CYCLES:
         if (!(PassedTimeMS(500L,(long)BGVAR(u32_enc_sim_test_timer)))) break;
         BGVAR(u8_Enc_Sim_Test_Request) = ENC_SIM_TEST_ENABLE;
      break;

      case ENC_SIM_TEST_BUSY:
         if (VAR(AX0_s16_Encsim_State_Busy)) break; // RT has not finish to send yet
         //RT has finished
         if (cycle_num < BGVAR(u16_Enc_Sim_Test_Number_Of_Cycles)) // if it's not the last cycle, we need to wait 500msec
         {
            BGVAR(u32_enc_sim_test_timer) = Cntr_1mS; //delay between cycles is 500ms
            BGVAR(u8_Enc_Sim_Test_Request) = ENC_SIM_TEST_WAIT_BETWEEN_CYCLES;
         }
         else
         {
            BGVAR(u8_Enc_Sim_Test_Request) = ENC_SIM_TEST_ENABLE;
         }
      break;

      case ENC_SIM_TEST_DISABLE: // user has disable test mode
         VAR(AX0_s16_Encsim_Test_Mode) = 0; //Disable EncSim test mode
         cycle_num = 0;
         VAR(AX0_s16_Encsim_Counts_In_MTS) = 0;
         VAR(AX0_s16_Encsim_Num_Of_Loops) = 0;
         VAR(AX0_s16_Encsim_Counts_Remainder) = 0;
         VAR(AX0_s16_Encsim_State_Busy) = 0;
         VAR(AX0_s16_Encsim_Space) = 0;
         BGVAR(u16_Enc_Sim_Test_Number_Of_Cycles) = 0;
         u8_Enc_Sim_Test_Request = ENC_SIM_TEST_IDLE;
      break;
   }
}


//**********************************************************
// Function Name: CheckRestart
// Description:
// Detect case when main power is turened on while the drive is on 5V from 485 cable
// "In Box Programing". In this case full restart is needed.
// Called from the background.
// Author:  Udi
// Algorithm:
//**********************************************************
void CheckRestart(void)
{
   if (u16_Product != SHNDR_HW)
      return;

   if (u16_Power_Board_Off == 1)
   { //Drive was started on 5v, need to restart if main power is now on
      if (GpioDataRegs.GPBDAT.bit.GPIO60 == 1)
      { // Power-on detected while working as "In Box Programing"
           // if (s16_Power_Back_Counter > 0)
           //    s16_Power_Back_Counter --;  // Wait for x successive reads
          //  else //Reset the drive
          //  {
             //  u16_Power_Board_Off = 0;
               RestartDriveFirmware();
          //  }
      }
      else
         s16_Power_Back_Counter = 10; // Reset the counter
   }
   else
   { // We started with power on - mark if now main is off and we are on 5V
      if (GpioDataRegs.GPBDAT.bit.GPIO60 == 0)
      { // Power-off detected  (we are now in "In Box Programing")
            if (s16_Power_Off_Counter > 0)
               s16_Power_Off_Counter --;  // Wait for x successive reads
            else //Reset the drive
               u16_Power_Board_Off = 1;
             // Also clear FPGA ?!
      }
      else
         s16_Power_Off_Counter = 10; // Reset the counter
   }
}

